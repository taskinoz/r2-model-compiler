"""Compile source fixtures and inspect the written MDL, PHY and animation data."""
from pathlib import Path
import math
import os
import subprocess
import struct
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
import decompile53 as dc
import mdl53
import phy
import roundtrip
import validate53


def cube(center, normal, bone=0):
    vertices = [(x + center[0], y + center[1], z + center[2])
                for x, y, z in ((-1,-1,-1), (1,-1,-1), (1,1,-1), (-1,1,-1),
                                (-1,-1,1), (1,-1,1), (1,1,1), (-1,1,1))]
    faces = ((0,2,1),(0,3,2),(4,5,6),(4,6,7),(0,1,5),(0,5,4),
             (1,2,6),(1,6,5),(2,3,7),(2,7,6),(3,0,4),(3,4,7))
    return [('models/electricalboxes/pipesmulti',
             [(vertices[i], normal, (0,0), [(bone,1.)]) for i in face]) for face in faces]


class CompilerRegressions(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.compiler = Path(os.environ.get('R2C_COMPILER', ROOT / 'build/r2studiomdl.exe')).resolve()
        if not cls.compiler.exists():
            raise unittest.SkipTest('build r2studiomdl first or set R2C_COMPILER')

    def setUp(self):
        scratch = ROOT / 'test-output'
        scratch.mkdir(exist_ok=True)
        self.temporary = tempfile.TemporaryDirectory(dir=scratch)
        self.addCleanup(self.temporary.cleanup)
        self.work = Path(self.temporary.name)

    def compile(self, qc, name='test'):
        path = self.work / (name + '.qc')
        path.write_text(qc)
        output = self.work / (name + '.mdl')
        run = subprocess.run([str(self.compiler), '-quiet', '-o', str(output), str(path)],
                             capture_output=True, text=True, timeout=30)
        self.assertEqual(run.returncode, 0, run.stdout + run.stderr)
        return output

    def test_touching_collision_pieces_keep_smoothing_groups(self):
        bones = [{'name':'root', 'parent':-1}]
        geometry = cube((0,0,0), (0,0,1)) + cube((2,2,0), (0,1,0))
        dc.write_smd(self.work / 'body.smd', bones, [[((0,0,0),(0,0,0))]], geometry)
        qc = ('$modelname "test.mdl"\n$staticprop\n$body body "body.smd"\n'
              '$sequence idle "body.smd"\n$collisionmodel "body.smd" { $concave %s }\n')
        for option, count, volume in [('',2,16.), ('$weldnormal -0.5',1,24.)]:
            output = self.compile(qc % option)
            data = output.read_bytes()
            header = mdl53.Model(data).hdr
            physical = phy.parse_phy(data[header['phyOffset']:header['phyOffset']+header['phySize']])
            self.assertEqual(header['staticCollisionCount'], count)
            self.assertAlmostEqual(roundtrip.summary(output)['phy']['volume'], volume, places=3)
            self.assertEqual(len([ledge for ledge in physical['solids'][0]['ledges']
                                  if not ledge['has_children']]), count)

    def test_bone_followers_and_concave_per_joint_roundtrip(self):
        # Animated single-player props: one joint owns two separate convex pieces
        # ($concaveperjoint, no "concave" key in the PHY text, as in retail) and is
        # listed as a bone follower after the collision block.
        bones = [{'name':'root', 'parent':-1}, {'name':'arm', 'parent':0}]
        bind = [((0,0,0),(0,0,0)), ((0,0,10),(0,0,0))]
        dc.write_smd(self.work / 'body.smd', bones, [bind], cube((0,0,0),(0,0,1),0) + cube((0,0,10),(0,0,1),1))
        dc.write_smd(self.work / 'phys.smd', bones, [bind],
                     cube((0,0,10),(0,0,1),1) + cube((6,0,10),(0,1,0),1))
        output = self.compile('$modelname "test.mdl"\n$body body "body.smd"\n$sequence idle "body.smd"\n'
                              '$collisionjoints "phys.smd" { $concaveperjoint }\n$bonefollower "arm"\n')
        data = output.read_bytes()
        header = mdl53.Model(data).hdr
        physical = phy.parse_phy(data[header['phyOffset']:header['phyOffset']+header['phySize']])
        self.assertEqual([len([L for L in s['ledges'] if not L['has_children']]) for s in physical['solids']], [2])
        self.assertNotIn('"concave"', physical['text'])
        self.assertEqual(header['boneFollowerCount'], 1)
        self.assertTrue(all(b['flags'] & 0x80000 for b in mdl53.Model(data).bones()))
        self.assertGreaterEqual(header['boneFollowerOffset'], header['collisionOffset'])
        self.assertEqual(roundtrip.summary(output)['followers'], ['arm'])
        qc2 = Path(dc.decompile(output, self.work / 'decompiled')).read_text()
        self.assertIn('$concaveperjoint', qc2)
        self.assertIn('$bonefollower "arm"', qc2)
        second = self.compile(qc2.replace('"test_', '"decompiled/test_').replace('"anim_', '"decompiled/anim_'), 'second')
        self.assertEqual(roundtrip.compare(roundtrip.summary(output), roundtrip.summary(second)), [])

    def test_dmx_polygons_become_map_collision_faces(self):
        # Retail map-collision faces are the source mesh's polygons (Maya quads), not a
        # planarity merge: a slightly bent quad stays one face. DMX keeps polygons; SMD
        # triangles fall back to merging by plane.
        bones = [{'name': 'root', 'parent': -1}]
        corners = [(0,0,0), (10,0,0), (10,10,0), (0,10,0), (0,0,10), (10,0,10), (10,10,10.2), (0,10,10)]
        quads = [(0,3,2,1), (4,5,6,7), (0,1,5,4), (1,2,6,5), (2,3,7,6), (3,0,4,7)]
        up = (0, 0, 1)
        dc.write_dmx_mesh(str(self.work / 'phys.dmx'), bones, [((0,0,0), (0,0,0))],
                          [[(corners[i], up, [(0, 1.0)]) for i in q] for q in quads])
        tris = [('phys', [(corners[i], up, (0,0), [(0, 1.0)]) for i in (q[0], q[k], q[k + 1])])
                for q in quads for k in (1, 2)]
        dc.write_smd(self.work / 'phys.smd', bones, [[((0,0,0), (0,0,0))]], tris)
        dc.write_smd(self.work / 'body.smd', bones, [[((0,0,0), (0,0,0))]], cube((5,5,5),(0,0,1)))
        faces = {}
        for ext in ('dmx', 'smd'):
            output = self.compile('$modelname "test.mdl"\n$body body "body.smd"\n$sequence idle "body.smd"\n'
                                  '$collisionmodel "phys.%s" { }\n' % ext, ext)
            data = output.read_bytes()
            header = mdl53.Model(data).hdr
            self.assertEqual(header['staticCollisionCount'], 1)
            faces[ext] = struct.unpack_from('<i', data, header['collisionOffset'])[0]
        self.assertEqual(faces, {'dmx': 6, 'smd': 7})
        # the decompiler writes the block's faces back out as DMX polygons
        qc2 = Path(dc.decompile(self.work / 'dmx.mdl', self.work / 'decompiled')).read_text()
        self.assertIn('_physics.dmx"', qc2)
        second = self.compile(qc2.replace('"dmx_', '"decompiled/dmx_').replace('"anim_', '"decompiled/anim_'), 'second')
        self.assertEqual(roundtrip.compare(roundtrip.summary(self.work / 'dmx.mdl'), roundtrip.summary(second)), [])
        self.assertTrue(validate53.compare_physics(self.work / 'dmx.mdl',second)['ok'])

    def test_collision_keeps_surface_points_and_polygon_faces(self):
        # Retail keeps points lying on a piece's surface (an edge midpoint, a cap centre)
        # as vertices, and its faces are the source polygons even where they share a
        # plane. Here the top is a triangle plus a quad meeting at an edge midpoint, and a
        # loose vertex sits at the centre of the bottom.
        bones = [{'name': 'root', 'parent': -1}]
        p = [(0,0,0), (10,0,0), (10,10,0), (0,10,0), (0,0,10), (10,0,10), (10,10,10), (0,10,10), (5,0,10)]
        polygons = [(0,3,2,1), (0,1,5,8,4), (1,2,6,5), (2,3,7,6), (3,0,4,7), (4,8,7), (8,5,6,7)]
        up = (0, 0, 1)
        dc.write_dmx_mesh(str(self.work / 'phys.dmx'), bones, [((0,0,0), (0,0,0))],
                          [[(p[i], up, [(0, 1.0)]) for i in poly] for poly in polygons],
                          loose=[((5, 5, 0), [(0, 1.0)], 0)])
        tris = [('phys', [(p[i], up, (0,0), [(0, 1.0)]) for i in (poly[0], poly[k], poly[k + 1])])
                for poly in polygons for k in range(1, len(poly) - 1)]
        dc.write_smd(self.work / 'phys.smd', bones, [[((0,0,0), (0,0,0))]], tris)
        dc.write_smd(self.work / 'body.smd', bones, [[((0,0,0), (0,0,0))]], cube((5,5,5),(0,0,1)))
        counts = {}
        for ext in ('dmx', 'smd'):
            output = self.compile('$modelname "test.mdl"\n$body body "body.smd"\n$sequence idle "body.smd"\n'
                                  '$collisionmodel "phys.%s" { }\n' % ext, ext)
            data = output.read_bytes()
            header = mdl53.Model(data).hdr
            fc, fb, ec, vc = struct.unpack_from('<4i', data, header['collisionOffset'])
            counts[ext] = (fc, vc)
        self.assertEqual(counts, {'dmx': (7, 10), 'smd': (6, 9)})
        # a T-junction: the front face is a plain quad although the top's polygons meet at
        # a point on its top edge; the edge is split so the faces still meet edge to edge
        tee = [poly if poly != (0,1,5,8,4) else (0,1,5,4) for poly in polygons]
        dc.write_dmx_mesh(str(self.work / 'tee.dmx'), bones, [((0,0,0), (0,0,0))],
                          [[(p[i], up, [(0, 1.0)]) for i in poly] for poly in tee])
        output = self.compile('$modelname "test.mdl"\n$body body "body.smd"\n$sequence idle "body.smd"\n'
                              '$collisionmodel "tee.dmx" { }\n', 'tee')
        data = output.read_bytes()
        header = mdl53.Model(data).hdr
        fc, fb, ec, vc = struct.unpack_from('<4i', data, header['collisionOffset'])
        self.assertEqual((fc, vc), (7, 9))
        # the DMX shares one vertex per position, so the piece is a connected mesh
        text = (self.work / 'phys.dmx').read_text()
        positions = text.split('"position$0" "vector3_array" [')[1].split(']')[0].count('"')
        self.assertEqual(positions // 2, 10)

    def test_nomapcollision_omits_the_block(self):
        (self.work / 'body.smd').write_text((ROOT / 'tests/box/box.smd').read_text())
        output = self.compile('$modelname "test.mdl"\n$body body "body.smd"\n$sequence idle "body.smd"\n'
                              '$collisionmodel "body.smd" { $nomapcollision }\n')
        data = output.read_bytes()
        model = mdl53.Model(data)
        header = model.hdr
        self.assertGreater(header['phySize'], 0)
        self.assertEqual((header['collisionOffset'], header['staticCollisionCount']), (0, 0))
        self.assertEqual((model.bones()[0]['collisionIndex'], model.bones()[0]['collisionCount']), (0, 0))
        qc2 = Path(dc.decompile(output, self.work / 'decompiled')).read_text()
        self.assertIn('$nomapcollision', qc2)
        second = self.compile(qc2.replace('"test_', '"decompiled/test_').replace('"anim_', '"decompiled/anim_'), 'second')
        self.assertEqual(roundtrip.compare(roundtrip.summary(output), roundtrip.summary(second)), [])

    def test_animation_reencoding_and_selected_frame_decode(self):
        bones = [{'name':'root','parent':-1}, {'name':'tip','parent':0}]
        bind = [((0,0,0),(0,0,0)), ((6000,0,0),(0,0,0))]
        dc.write_smd(self.work / 'body.smd', bones, [bind], cube((6000,0,0),(0,0,1),1))
        frames = [[((0,0,0),(.7*math.sin(i*.3), .2*math.cos(i*.4), i*.05)), bind[1]]
                  for i in range(65)]
        dc.write_smd(self.work / 'motion.smd', bones, frames)
        first = self.compile('$modelname "test.mdl"\n$body body "body.smd"\n'
                             '$sectionframes 30 30\n$sequence move "motion.smd" fps 30\n')
        data = first.read_bytes()
        model = mdl53.Model(data)
        a, name, all_frames = dc.decode_anim(data, model, model.bones(), 0)
        samples = [0,29,30,31,60,63,64]
        _, _, selected = dc.decode_anim(data, model, model.bones(), 0, samples)
        self.assertEqual(selected, [all_frames[i] for i in samples])
        with self.assertRaises(ValueError):
            dc.decode_anim(data, model, model.bones(), 0, [65])
        qc = dc.decompile(first, self.work / 'decompiled')
        second = self.compile(Path(qc).read_text().replace('"body_bp', '"decompiled/body_bp')
                              .replace('"test_bp', '"decompiled/test_bp')
                              .replace('"anim_', '"decompiled/anim_'), 'second')
        self.assertEqual(roundtrip.compare(roundtrip.summary(first), roundtrip.summary(second)), [])

    def test_constant_section_excludes_shared_boundary_frame(self):
        bones = [{'name':'root','parent':-1}, {'name':'tip','parent':0}]
        bind = [((0,0,0),(0,0,0)), ((10,0,0),(0,0,0))]
        dc.write_smd(self.work / 'body.smd', bones, [bind], cube((10,0,0),(0,0,1),1))
        frames = [[bind[0], ((11 if i < 30 else 12,0,0),(0,0,0))] for i in range(65)]
        dc.write_smd(self.work / 'move.smd', bones, frames)
        output = self.compile('$modelname "test.mdl"\n$body body "body.smd"\n'
            '$sectionframes 30 30\n$sequence move "move.smd"\n')
        d = output.read_bytes()
        m = mdl53.Model(d)
        ao = m.hdr['localanimindex']
        a = mdl53.u(mdl53.ANIMDESC_FIELDS, d, ao)
        first = ao + struct.unpack_from('<i', d, ao + a['sectionindex'])[0]
        while d[first + 4] != 1:
            jump = struct.unpack_from('<i', d, first + 28)[0]
            self.assertNotEqual(jump, 0, 'tip record missing')
            first += jump
        self.assertEqual(d[first + 4], 1)
        self.assertTrue(d[first + 5] & 2)  # RAWPOS: frames 0..29 are constant
        _, _, decoded = dc.decode_anim(d, m, m.bones(), 0, [29,30,31])
        self.assertEqual([fr[1][0][0] for fr in decoded], [11,12,12])

    def test_retained_model_metadata_and_generated_boxes(self):
        bones = [{'name':'root','parent':-1}, {'name':'tip','parent':0}]
        bind = [((0,0,0),(0,0,0)), ((0,0,10),(0,0,0))]
        dc.write_smd(self.work / 'body.smd', bones, [bind], cube((0,0,10),(0,0,1),1))
        output = self.compile('$modelname "test.mdl"\n$body body "body.smd"\n$sequence idle "body.smd"\n'
            '$modelsource "body.smd" "C:/original/../unsafe/source.dmx"\n'
            '$respawnflag 1\n$pertricollision 0\n$zerounusedlods\n$boneusage "root" 65536\n'
            '$generatedhitboxes\n$hbox 0 "tip" -2 -3 -4 5 6 7\n'
            '$keyvalues { prop_data { "allowstatic" "1" } }\n')
        def inspect(path):
            data = path.read_bytes(); model = mdl53.Model(data); h = model.hdr
            self.assertTrue(h['flags'] & 1)
            self.assertTrue(h['flags'] & 0x800000)
            self.assertTrue(model.bones()[0]['flags'] & 65536)
            self.assertEqual(struct.unpack_from('<i', data, h['perTriAABBIndex'])[0], 0)
            vv = mdl53.u(mdl53.VVD_HDR, data, h['vvdOffset'])
            self.assertFalse(any(vv['numLODVertexes'][1:]))
            bp = mdl53.u(mdl53.BODYPART_FIELDS, data, h['bodypartindex'])
            md = mdl53.u(mdl53.MODEL_FIELDS, data, h['bodypartindex']+bp['modelindex'])
            self.assertEqual(md['name'], 'C:/original/../unsafe/source.dmx')
            hs = mdl53.u(mdl53.HBOXSET_FIELDS, data, h['hitboxsetindex'])
            hb = mdl53.u(mdl53.BBOX_FIELDS, data, h['hitboxsetindex']+hs['hitboxindex'])
            self.assertEqual(hb['bbmin'], [-2,-3,-4])
            self.assertEqual(hb['bbmax'], [5,6,7])
        inspect(output)
        qc = dc.decompile(output, self.work/'decompiled')
        second = self.work/'second.mdl'
        subprocess.run([str(self.compiler), '-quiet', '-o', str(second), qc], check=True,
                       capture_output=True, timeout=30)
        inspect(second)
        self.assertFalse((self.work/'unsafe').exists())

    def test_full_width_logical_model_name_roundtrip(self):
        bones=[{'name':'root','parent':-1}]
        dc.write_smd(self.work/'body.smd',bones,[[((0,0,0),(0,0,0))]],cube((0,0,0),(0,0,1)))
        logical='C:/'+('x'*57)+'.dmx';self.assertEqual(len(logical),64)
        first=self.compile('$modelname "test.mdl"\n$body body "body.smd"\n$sequence idle "body.smd"\n'
            '$modelsource "body.smd" "'+logical+'"\n')
        def name(path):
            data=path.read_bytes();h=mdl53.Model(data).hdr;at=h['bodypartindex']
            bp=mdl53.u(mdl53.BODYPART_FIELDS,data,at)
            return mdl53.u(mdl53.MODEL_FIELDS,data,at+bp['modelindex'])['name']
        self.assertEqual(name(first),logical)
        qc=dc.decompile(first,self.work/'decompiled');second=self.work/'second.mdl'
        subprocess.run([str(self.compiler),'-quiet','-o',str(second),qc],check=True,capture_output=True,timeout=30)
        self.assertEqual(name(second),logical)
        self.assertTrue(validate53.compare_metadata(first,second)['ok'])

    def test_retained_bone_usage_is_exact(self):
        bones=[{'name':'root','parent':-1}]
        dc.write_smd(self.work/'body.smd',bones,[[((0,0,0),(0,0,0))]],cube((0,0,0),(0,0,1)))
        first=self.compile('$modelname "test.mdl"\n$body body "body.smd"\n$sequence idle "body.smd"\n'
            '$attachment probe root 0 0 0\n$boneusage root 1024\n')
        self.assertEqual(mdl53.Model(first.read_bytes()).bones()[0]['flags']&0xfff00,1024)
        qc=dc.decompile(first,self.work/'decompiled');second=self.work/'second.mdl'
        subprocess.run([str(self.compiler),'-quiet','-o',str(second),qc],check=True,capture_output=True,timeout=30)
        self.assertEqual(mdl53.Model(second.read_bytes()).bones()[0]['flags']&0xfff00,1024)

    def test_named_empty_bodygroup_roundtrip(self):
        bones=[{'name':'root','parent':-1}]
        dc.write_smd(self.work/'body.smd',bones,[[((0,0,0),(0,0,0))]],cube((0,0,0),(0,0,1)))
        first=self.compile('$modelname "test.mdl"\n$bodygroup sight\n{\nstudio "body.smd"\nblankname "dmx/empty_sight.dmx"\nblank\n}\n'
            '$sequence idle "body.smd"\n')
        def names(path):
            data=path.read_bytes();h=mdl53.Model(data).hdr;at=h['bodypartindex']
            bp=mdl53.u(mdl53.BODYPART_FIELDS,data,at)
            return [mdl53.u(mdl53.MODEL_FIELDS,data,at+bp['modelindex']+i*mdl53.size(mdl53.MODEL_FIELDS))['name'] for i in range(bp['nummodels'])]
        self.assertEqual(names(first),['body.smd','dmx/empty_sight.dmx',''])
        qc=dc.decompile(first,self.work/'decompiled');second=self.work/'second.mdl'
        subprocess.run([str(self.compiler),'-quiet','-o',str(second),qc],check=True,capture_output=True,timeout=30)
        self.assertEqual(names(first),names(second))

    def test_header_bounds_retained_with_wider_triangle_collision(self):
        bones=[{'name':'root','parent':-1}]
        dc.write_smd(self.work/'body.smd',bones,[[((0,0,0),(0,0,0))]],cube((0,0,0),(0,0,1)))
        dc.write_pertri_mesh(self.work/'hit.pertri',{'bounds':(-20,-20,-20,20,20,20),
            'vertices':[(0,0,0),(65535,0,0),(0,65535,0)],'triangles':[(0,1,2,22)]})
        first=self.compile('$modelname "test.mdl"\n$body body "body.smd"\n'
            '$sequence idle "body.smd" { bbox -2 -2 -2 2 2 2 }\n$bbox -2 -2 -2 2 2 2\n$pertrimesh "hit.pertri"\n')
        qc=dc.decompile(first,self.work/'decompiled');second=self.work/'second.mdl'
        subprocess.run([str(self.compiler),'-quiet','-o',str(second),qc],check=True,capture_output=True,timeout=30)
        h=mdl53.Model(second.read_bytes()).hdr
        self.assertEqual(h['hull_min'],[-2,-2,-2]);self.assertEqual(h['hull_max'],[2,2,2])
        self.assertTrue(validate53.compare_metadata(first,second)['ok'])
        data=bytearray(second.read_bytes());offset=0
        for key,layout in mdl53.HDR_FIELDS:
            if key=='hull_min':break
            offset+=struct.calcsize('<'+layout)
        struct.pack_into('<f',data,offset,-3)
        corrupt=self.work/'wrong_bounds.mdl';corrupt.write_bytes(data)
        self.assertIn('hull_min differs',validate53.compare_metadata(first,corrupt)['issues'])

    def test_stored_legacy_movement_roundtrip(self):
        bones = [{'name':'root','parent':-1}]
        pose = [((0,0,0),(0,0,0))]
        dc.write_smd(self.work/'body.smd', bones, [pose], cube((0,0,0),(0,0,1)))
        dc.write_smd(self.work/'move.smd', bones, [pose]*5)
        output = self.compile('$modelname "test.mdl"\n$body body "body.smd"\n'
            '$animation move "move.smd" movementrecord 4 1 1 2 30 1 0 0 5 6 7\n$sequence move move\n')
        def movement(path):
            data=path.read_bytes(); h=mdl53.Model(data).hdr; ao=h['localanimindex']
            a=mdl53.u(mdl53.ANIMDESC_FIELDS,data,ao)
            self.assertEqual(a['nummovements'],1)
            return struct.unpack_from('<2i9f',data,ao+a['movementindex'])
        self.assertEqual(movement(output),(4,1,1,2,30,1,0,0,5,6,7))
        qc=dc.decompile(output,self.work/'decompiled'); second=self.work/'second.mdl'
        subprocess.run([str(self.compiler),'-quiet','-o',str(second),qc],check=True,
                       capture_output=True,timeout=30)
        self.assertEqual(movement(second),movement(output))

    def test_empty_active_lod_has_zero_vertices(self):
        bones=[{'name':'root','parent':-1}]
        mesh=cube((0,0,0),(0,0,1)) + [('models/second',tri) for _,tri in cube((4,0,0),(0,0,1))]
        dc.write_smd(self.work/'body.smd',bones,[[((0,0,0),(0,0,0))]],mesh)
        output=self.compile('$modelname "test.mdl"\n$staticprop\n$body body "body.smd"\n'
            '$sequence idle "body.smd"\n$lod 100 { removemodel "body.smd" }\n')
        d=output.read_bytes(); h=mdl53.Model(d).hdr
        vv=mdl53.u(mdl53.VVD_HDR,d,h['vvdOffset'])
        self.assertGreater(vv['numFixups'],0)
        self.assertEqual(vv['numLODs'],2)
        self.assertGreater(vv['numLODVertexes'][0],0)
        self.assertEqual(vv['numLODVertexes'][1:],[0]*7)

    def test_validator_checks_all_collision_triangles_and_node_bounds(self):
        bones=[{'name':'root','parent':-1}]
        dc.write_smd(self.work/'body.smd',bones,[[((0,0,0),(0,0,0))]],
                     cube((0,0,0),(0,0,1))+cube((4,0,0),(0,0,1)))
        output=self.compile('$modelname "test.mdl"\n$staticprop\n$body body "body.smd"\n$sequence idle "body.smd"\n')
        self.assertEqual(validate53.validate(output)['collision_triangles'],24)
        data=bytearray(output.read_bytes()); h=mdl53.Model(data).hdr
        self.assertGreater(h['perTriAABBNodeCount'],0)
        struct.pack_into('<6H',data,h['perTriAABBIndex']+60,*([0]*6))
        corrupt=self.work/'bad-node.mdl'; corrupt.write_bytes(data)
        with self.assertRaisesRegex(ValueError,'node bounds do not enclose child'):
            validate53.validate(corrupt)
        data=bytearray(output.read_bytes())
        leaf=h['perTriAABBIndex']+60+20*h['perTriAABBNodeCount']
        struct.pack_into('<H',data,leaf+16,65535)
        corrupt.write_bytes(data)
        with self.assertRaisesRegex(ValueError,'leaf vertex out of range'):
            validate53.validate(corrupt)

    def test_custom_pertriangle_mesh_roundtrip(self):
        bones=[{'name':'root','parent':-1}]
        dc.write_smd(self.work/'body.smd',bones,[[((0,0,0),(0,0,0))]],cube((0,0,0),(0,0,1)))
        points=[(x,y,z) for x,y,z in ((0,0,0),(65535,0,0),(65535,65535,0),(0,65535,0),
                                       (0,0,65535),(65535,0,65535),(65535,65535,65535),(0,65535,65535))]
        faces=((0,2,1),(0,3,2),(4,5,6),(4,6,7),(0,1,5),(0,5,4),
               (1,2,6),(1,6,5),(2,3,7),(2,7,6),(3,0,4),(3,4,7))
        dc.write_pertri_mesh(self.work/'hit.pertri',{'bounds':(-10,-10,-10,10,10,10),
            'vertices':points+[(32768,32768,32768)],'triangles':[f+(22,) for f in faces]})
        first=self.compile('$modelname "test.mdl"\n$staticprop\n$body body "body.smd"\n'
            '$sequence idle "body.smd"\n$pertrimesh "hit.pertri"\n')
        before=validate53.validate(first,False)
        self.assertFalse(before['collision_matches_render_mesh'])
        self.assertEqual(before['collision_triangles'],12)
        h=mdl53.Model(first.read_bytes()).hdr
        self.assertEqual(h['hull_min'],[-10,-10,-10])
        self.assertEqual(h['hull_max'],[10,10,10])
        qc=dc.decompile(first,self.work/'decompiled'); second=self.work/'second.mdl'
        subprocess.run([str(self.compiler),'-quiet','-o',str(second),qc],check=True,capture_output=True,timeout=30)
        after=validate53.validate(second,False)
        self.assertEqual(after['collision_signature'],before['collision_signature'])
        decoded=dc.decode_pertri_mesh(second.read_bytes(),mdl53.Model(second.read_bytes()).hdr)
        self.assertEqual(len(decoded['vertices']),9)  # loose source points survive too
        (self.work/'hit.pertri').write_text('r2c_pertri 1\n-1 -1 -1 1 1 1\n1 1\n0 0 0\n0 1 2 22\n')
        result=subprocess.run([str(self.compiler),'-quiet',str(self.work/'test.qc')],capture_output=True,text=True,timeout=30)
        self.assertNotEqual(result.returncode,0)
        self.assertIn('malformed per-triangle mesh',result.stdout+result.stderr)

    def test_full_frame_validation_detects_unsampled_changes(self):
        bones=[{'name':'root','parent':-1},{'name':'tip','parent':0}]
        bind=[((0,0,0),(0,0,0)),((0,0,10),(0,0,0))]
        dc.write_smd(self.work/'body.smd',bones,[bind],cube((0,0,10),(0,0,1),1))
        for name,distance in (('a',1),('b',2)):
            frames=[bind,[bind[0],((distance,0,10),(0,0,0))],bind,bind]
            dc.write_smd(self.work/(name+'.smd'),bones,frames)
            self.compile('$modelname "test.mdl"\n$body body "body.smd"\n$sequence move "'+name+'.smd"\n',name)
        a,b=self.work/'a.mdl',self.work/'b.mdl'
        self.assertEqual(roundtrip.compare(roundtrip.summary(a),roundtrip.summary(b)),[])
        report=validate53.compare_animations(a,b)
        plain=validate53.compare_animations(a,b,accelerated=False)
        for key in report['maximum']:
            self.assertAlmostEqual(report['maximum'][key],plain['maximum'][key],places=10)
        self.assertEqual(report['frames'],4)
        self.assertEqual(report['where']['position'][1],1)
        self.assertTrue(report['issues'])

    def test_stored_animation_tracks_and_quantization(self):
        bones = [{'name':'root','parent':-1}, {'name':'tip','parent':0}]
        bind = [((0,0,0),(0,0,0)), ((6000,0,0),(0,0,0))]
        dc.write_smd(self.work / 'body.smd', bones, [bind], cube((6000,0,0),(0,0,1),1))
        frames = [[bind[0], ((6000 + i * .125,0,0),(.03125 * i,0,0))] for i in range(7)]
        dc.write_smd(self.work / 'move.smd', bones, frames)
        # Zero scales are valid for absent channels. The root pose stays separate.
        track = [(i * .25, 0, -i * .5, i * 1.25) for i in range(7)]
        (self.work / 'move.fmtrack').write_text('0.25 0 0.5 1.25\n' +
            ''.join(' '.join(map(str, row)) + '\n' for row in track))
        first = self.compile('$modelname "test.mdl"\n$body body "body.smd"\n'
            '$bonerotscale "tip" .0009765625 .001 .001\n'
            '$animation move "move.smd" fmtrack "move.fmtrack" posscale "tip" .125 hasscale\n'
            '$sequence move move\n')
        data = first.read_bytes()
        model = mdl53.Model(data)
        self.assertEqual(dc.decode_frame_movement(data, model, 0, range(7)), track)
        self.assertEqual(dc.anim_pos_scales(data, model.hdr, 0), [(1, .125)])
        self.assertEqual(model.bones()[1]['rotscale'][0], .0009765625)
        a, _, pose = dc.decode_anim(data, model, model.bones(), 0)
        self.assertTrue(a['flags'] & 0x20000)  # retained even with all bone scales equal to 1
        self.assertEqual(pose[-1][1][0], (6000.75, 0, 0))
        qc = Path(dc.decompile(first, self.work / 'decompiled'))
        second = self.work / 'second.mdl'
        subprocess.run([str(self.compiler), '-quiet', '-o', str(second), str(qc)], check=True,
                       capture_output=True, timeout=30)
        d2 = second.read_bytes()
        m2 = mdl53.Model(d2)
        self.assertEqual(dc.decode_frame_movement(d2, m2, 0, range(7)), track)
        self.assertEqual(dc.anim_pos_scales(d2, m2.hdr, 0), [(1, .125)])
        self.assertEqual(roundtrip.compare(roundtrip.summary(first), roundtrip.summary(second)), [])
        (self.work / 'move.fmtrack').write_text('0.25 0 0.5 1.25\n' +
            ''.join(' '.join(map(str, row)) + '\n' for row in track[:-1]))
        result = subprocess.run([str(self.compiler), '-quiet', str(self.work / 'test.qc')],
                                capture_output=True, text=True, timeout=30)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('track has 6 frames', result.stdout + result.stderr)
        for bad in ('0.25 0 0.5 1.25\n', '0.25 0 0.5 1.25\n0 0 0 0\njunk',
                    '0.25 0 0.5 1.25\n0 0 nan 0', '0.25 0 0.5 1.25\n0 0 0'):
            (self.work / 'move.fmtrack').write_text(bad)
            result = subprocess.run([str(self.compiler), '-quiet', str(self.work / 'test.qc')],
                                    capture_output=True, text=True, timeout=30)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('malformed animation data', result.stdout + result.stderr)

    def test_stored_ik_target_and_error_stream(self):
        bones = [{'name':'root','parent':-1}, {'name':'hip','parent':0},
                 {'name':'knee','parent':1}, {'name':'foot','parent':2}]
        bind = [((0,0,0),(0,0,0)), ((0,0,30),(0,0,0)), ((0,0,-15),(0,0,0)), ((0,0,-15),(0,0,0))]
        dc.write_smd(self.work / 'body.smd', bones, [bind], cube((0,0,0),(0,0,1),3))
        dc.write_smd(self.work / 'move.smd', bones, [bind] * 5)
        errors = [(i * .25, -i * .5, 0, i * .125, 0, 0) for i in range(5)]
        (self.work / 'stored.ikerr').write_text('.25 .5 0 .125 0 0\n' +
            ''.join(' '.join(map(str, row)) + '\n' for row in errors))
        first = self.compile('$modelname "test.mdl"\n$body body "body.smd"\n'
            '$ikchain "leg" "foot" knee 1 0 0\n'
            '$animation move "move.smd" ikrule "leg" footstep range 0 1 3 4 contact 1 '
            'storedtarget 123 -456 0 0 0 0 1 errors "stored.ikerr"\n$sequence move move\n')
        def values(path):
            d = path.read_bytes()
            r = dc.decode_ik_rules(d, mdl53.Model(d).hdr, 0)[0]
            return tuple(r['pos']), tuple(r['q']), [sum(dc.decode_ik_error(d, r, i), ()) for i in range(5)]
        self.assertEqual(values(first), ((123., -456., 0.), (0., 0., 0., 1.), errors))
        qc = dc.decompile(first, self.work / 'decompiled')
        second = self.work / 'second.mdl'
        subprocess.run([str(self.compiler), '-quiet', '-o', str(second), qc], check=True,
                       capture_output=True, timeout=30)
        self.assertEqual(values(second), values(first))

    def test_bone_scale_animation_roundtrips(self):
        bones = [{'name':'root','parent':-1}, {'name':'tip','parent':0}]
        bind = [((0,0,0),(0,0,0)), ((10,0,0),(0,0,0))]
        dc.write_smd(self.work / 'body.smd', bones, [bind], cube((10,0,0),(0,0,1),1))
        # animated scale on the tip; the SMD extension adds "sx sy sz" after the rotation
        scales = [(1 + .5 * math.sin(i * .3), 1., 1 - .25 * i / 11) for i in range(12)]
        dc.write_smd(self.work / 'grow.smd', bones,
                     [[bind[0] + ((1,1,1),), bind[1] + (s,)] for s in scales])
        dc.write_smd(self.work / 'still.smd', bones, [bind] * 3)
        first = self.compile('$modelname "test.mdl"\n$body body "body.smd"\n'
                             '$sequence grow "grow.smd" fps 30\n$sequence still "still.smd" fps 30\n')
        data = first.read_bytes()
        model = mdl53.Model(data)
        a, _, frames = dc.decode_anim(data, model, model.bones(), 0, with_scale=True)
        self.assertTrue(a['flags'] & 0x20000)
        for got, want in zip(frames, scales):
            for g, w in zip(got[1][2], want):
                self.assertAlmostEqual(g, w, delta=1 / 2048)
        b, _, _ = dc.decode_anim(data, model, model.bones(), 1)
        self.assertFalse(b['flags'] & 0x20000)
        flags = [mdl53.u(mdl53.SEQDESC_FIELDS, data, model.hdr['localseqindex'] + i * 0xE8)['flags'] for i in range(2)]
        self.assertEqual([f & 0x20000 for f in flags], [0x20000, 0])
        qc = dc.decompile(first, self.work / 'decompiled')
        second = self.compile(Path(qc).read_text().replace('"test_bp', '"decompiled/test_bp')
                              .replace('"anim_', '"decompiled/anim_'), 'second')
        self.assertEqual(roundtrip.compare(roundtrip.summary(first), roundtrip.summary(second)), [])

    def test_bind_scale_and_scaled_skinning_roundtrip(self):
        bones = [{'name':'root','parent':-1}, {'name':'tip','parent':0}]
        bind = [((0,0,0),(0,0,0),(1,1,1)), ((10,0,0),(0,0,0),(.1,.1,.1))]
        dc.write_smd(self.work / 'body.smd', bones, [bind], cube((12,0,0),(0,0,1),1))
        frames = [bind, [bind[0], ((10,0,0),(0,0,0),(.2,.2,.2))]]
        dc.write_smd(self.work / 'move.smd', bones, frames)
        first = self.compile('$modelname "test.mdl"\n$body body "body.smd"\n$sequence move "move.smd"\n')
        def skin(path):
            d = path.read_bytes()
            m = mdl53.Model(d)
            b = m.bones()
            self.assertAlmostEqual(b[1]['scale'][0], .1)
            self.assertAlmostEqual(b[1]['poseToBone'][0], 10)
            self.assertAlmostEqual(b[1]['poseToBone'][3], -100)
            vv = mdl53.u(mdl53.VVD_HDR, d, m.hdr['vvdOffset'])
            vertex = struct.unpack_from('<3f', d, m.hdr['vvdOffset'] + vv['vertexDataStart'] + 16)
            inv = [b[1]['poseToBone'][i:i+4] for i in (0,4,8)]
            local = tuple(sum(inv[r][c] * vertex[c] for c in range(3)) + inv[r][3] for r in range(3))
            _, _, poses = dc.decode_anim(d, m, b, 0, with_scale=True)
            result = []
            for pose in poses:
                world = []
                for j,(pos,rot,scale) in enumerate(pose):
                    mat = [[rot[r][c] * scale[c] for c in range(3)] + [pos[r]] for r in range(3)]
                    world.append(dc.concat(world[b[j]['parent']], mat) if b[j]['parent'] >= 0 else mat)
                tip = world[1]
                result.append(tuple(sum(tip[r][c]*local[c] for c in range(3)) + tip[r][3] for r in range(3)))
            self.assertAlmostEqual(math.dist(result[0], tuple(row[3] for row in world[1])),
                                   math.dist(vertex, (10,0,0)), delta=.01)
            return result
        before = skin(first)
        qc = dc.decompile(first, self.work / 'decompiled')
        second = self.work / 'second.mdl'
        subprocess.run([str(self.compiler), '-quiet', '-o', str(second), qc], check=True,
                       capture_output=True, timeout=30)
        after = skin(second)
        for a,b in zip(before, after):
            self.assertLess(math.dist(a,b), .01)

    def test_nonuniform_dmx_bind_scale_and_mesh_normals(self):
        bones=[{'name':'root','parent':-1}]
        faces=[(mat,0,[(p,n,w,{'uv':uv}) for p,n,uv,w in tri])
               for mat,tri in cube((0,0,0),(1,1,1))]
        path=self.work/'body.dmx'
        dc.write_dmx(path,bones,[((0,0,0),(0,0,0),(2,3,4))],faces)
        text=path.read_text()
        old='"scale" "vector3" "1 1 1"'
        before,after=text.rsplit(old,1)  # mesh DmeDag, not the skeleton transform
        path.write_text(before+'"scale" "vector3" "2 3 4"'+after)
        output=self.compile('$modelname "test.mdl"\n$definebone "root" "" 0 0 0 0 0 0 scale 2 3 4\n'
            '$body body "body.dmx"\n$sequence idle "body.dmx"\n')
        data=output.read_bytes(); model=mdl53.Model(data); h=model.hdr
        self.assertEqual(model.bones()[0]['scale'],[2,3,4])
        vv=mdl53.u(mdl53.VVD_HDR,data,h['vvdOffset'])
        at=h['vvdOffset']+vv['vertexDataStart']
        pos=struct.unpack_from('<3f',data,at+16); nrm=struct.unpack_from('<3f',data,at+28)
        self.assertEqual(tuple(abs(x) for x in pos),(2,3,4))
        expected=(.5,1/3,.25); length=math.sqrt(sum(x*x for x in expected))
        self.assertLess(math.dist(nrm,tuple(x/length for x in expected)),1e-5)
        validate53.validate(output)
        qc=dc.decompile(output,self.work/'decompiled'); second=self.work/'second.mdl'
        subprocess.run([str(self.compiler),'-quiet','-o',str(second),qc],check=True,capture_output=True,timeout=30)
        self.assertFalse(validate53.compare_animations(output,second)['issues'])

    def test_constant_bind_scale_is_not_rounded_through_half(self):
        bones=[{'name':'root','parent':-1},{'name':'tip','parent':0}]
        bind=[((0,0,0),(0,0,0),(1,1,1)),((10,0,0),(0,0,0),(.1,.1,.1))]
        dc.write_smd(self.work/'body.smd',bones,[bind],cube((12,0,0),(0,0,1),1))
        frames=[[bind[0],((10+i,0,0),(0,0,0),(.1,.1,.1))] for i in range(4)]
        dc.write_smd(self.work/'move.smd',bones,frames)
        output=self.compile('$modelname "test.mdl"\n$body body "body.smd"\n$sequence move "move.smd"\n')
        data=output.read_bytes(); model=mdl53.Model(data); bones=model.bones()
        _,_,poses=dc.decode_anim(data,model,bones,0,with_scale=True)
        for pose in poses:self.assertEqual(tuple(pose[1][2]),tuple(bones[1]['scale']))

    def test_implicit_near_unit_scale_survives_final_section(self):
        scale=.9999998211860657
        bones=[{'name':'root','parent':-1},{'name':'tip','parent':0}]
        bind=[((0,0,0),(0,0,0),(1,1,1)),((2403.032959,1907.639893,-3451.259521),(0,0,0),(scale,scale,scale))]
        dc.write_smd(self.work/'body.smd',bones,[bind],cube((2403,1907,-3451),(0,0,1),1))
        frames=[[bind[0],((bind[1][0][0]+(10 if i<60 else 0),*bind[1][0][1:]),(0,0,0),(scale,scale,scale))] for i in range(131)]
        dc.write_smd(self.work/'move.smd',bones,frames)
        first=self.compile('$modelname "test.mdl"\n$body body "body.smd"\n$sequence move "move.smd"\n')
        data=first.read_bytes(); h=mdl53.Model(data).hdr
        self.assertFalse(mdl53.u(mdl53.ANIMDESC_FIELDS,data,h['localanimindex'])['flags']&0x20000)
        qc=dc.decompile(first,self.work/'decompiled');second=self.work/'second.mdl'
        subprocess.run([str(self.compiler),'-quiet','-o',str(second),qc],check=True,capture_output=True,timeout=30)
        result=validate53.compare_animations(first,second)
        self.assertFalse(result['issues'],result)

    def test_physics_mass_distribution_properties_and_rules_roundtrip(self):
        bones=[{'name':'root','parent':-1},{'name':'arm','parent':0}]
        bind=[((0,0,0),(0,0,0)),((0,0,10),(0,0,0))]
        dc.write_smd(self.work/'body.smd',bones,[bind],cube((0,0,0),(0,0,1),0)+cube((0,0,10),(0,1,0),1))
        qc=('$modelname "test.mdl"\n$body body "body.smd"\n$sequence idle "body.smd"\n'
            '$collisionjoints "body.smd"\n{\n$mass 30\n$jointparent "root" "root"\n$jointmassbias "root" 2\n'
            '$jointdamping "root" .1\n$jointdamping "arm" .2\n$jointinertia "arm" 2\n$jointinertia "root" -2\n'
            '$jointrotdamping "arm" .4\n$jointcollide "root" "arm"\n'
            '$animatedfriction 1 3 .2 .5 .4\n$jointconstrain "arm" z limit -20 30 .2\n}\n'
            '$collisiontext\n{\nbreak { "model" "models/test_debris.mdl" "fadetime" "10" "debris" "1" }\n}\n')
        first=self.compile(qc); physical=validate53.physics_data(first)
        masses=[float(x) for x in __import__('re').findall(r'"mass"\s+"([^"]+)"',physical['text'])]
        self.assertEqual(masses,[20,10])
        qc=dc.decompile(first,self.work/'decompiled');second=self.work/'second.mdl'
        subprocess.run([str(self.compiler),'-quiet','-o',str(second),qc],check=True,capture_output=True,timeout=30)
        report=validate53.compare_physics(first,second)
        self.assertTrue(report['ok'],report)
        self.assertEqual(report['solids'],2)
        self.assertEqual(report['convex_pieces'],2)
        validate53.validate(second)

    def test_explicit_damping_and_mass_center(self):
        bones=[{'name':'root','parent':-1}]
        mesh=[(mat,[((p[0]*20,p[1],p[2]),n,uv,w) for p,n,uv,w in tri]) for mat,tri in cube((0,0,0),(0,0,1))]
        dc.write_smd(self.work/'body.smd',bones,[[((0,0,0),(0,0,0))]],mesh)
        first=self.compile('$modelname "test.mdl"\n$body body "body.smd"\n$sequence idle "body.smd"\n'
            '$collisionmodel "body.smd" { $mass 50 $rotdamping 0 $masscenter 10 0 0 }\n')
        physical=validate53.physics_data(first)
        self.assertIn('"rotdamping" "0.000000"',physical['text'])
        self.assertAlmostEqual(physical['solids'][0]['mass_center'][0],.254,places=6)

    def test_single_frame_uses_precise_rotation_and_position(self):
        bones=[{'name':'root','parent':-1}]
        dc.write_smd(self.work/'body.smd',bones,[[((0,0,0),(0,0,0))]],cube((0,0,0),(0,0,1)))
        angle=math.pi-.001
        dc.write_smd(self.work/'pose.smd',bones,[[((6000.1,0,0),(angle,0,0))]],[])
        output=self.compile('$modelname "test.mdl"\n$body body "body.smd"\n$sequence pose "pose.smd"\n')
        data=output.read_bytes();model=mdl53.Model(data)
        _,_,frames=dc.decode_anim(data,model,model.bones(),0,with_scale=True)
        # Import applies a default quarter turn to the root, preserving its tilt.
        expected=dc.concat(dc.angle_matrix((0,0,math.pi/2)),dc.angle_matrix((angle,0,0)))
        self.assertLess(validate53.rotation_error(expected,frames[0][0][1]),.01)
        self.assertLess(math.dist(frames[0][0][0],(0,6000.1,0)),.01)

    def test_euler_poles_and_pi_endpoint_roundtrip(self):
        bones=[{'name':'root','parent':-1},{'name':'tip','parent':0}]
        bind=[((0,0,0),(0,0,0)),((0,0,10),(0,0,0))]
        dc.write_smd(self.work/'body.smd',bones,[bind],cube((0,0,10),(0,0,1),1))
        frames=[[bind[0],((0,0,10),(math.pi-1.51e-7,-1.424264,math.pi-1.51e-7) if i<15
                                      else (1.9,math.pi/2-.0005,-3.1))] for i in range(31)]
        dc.write_smd(self.work/'pose.smd',bones,frames,[])
        first=self.compile('$modelname "test.mdl"\n$body body "body.smd"\n'
            '$bonerotscale tip .00009587379463482648 .00004791469109477475 .00009587379463482648\n'
            '$sequence pose "pose.smd"\n')
        qc=dc.decompile(first,self.work/'decompiled');second=self.work/'second.mdl'
        subprocess.run([str(self.compiler),'-quiet','-o',str(second),qc],check=True,capture_output=True,timeout=30)
        report=validate53.compare_animations(first,second)
        self.assertFalse(report['issues'],report)

    def test_rigid_physics_undoes_first_animation_pose(self):
        bones=[{'name':'root','parent':-1}]
        dc.write_smd(self.work/'body.smd',bones,[[((0,0,0),(0,0,0))]],cube((0,0,0),(0,0,1)))
        dc.write_smd(self.work/'pose.smd',bones,[[((32,7,5),(.2,.3,.4))]],[])
        first=self.compile('$modelname "test.mdl"\n$body body "body.smd"\n'
            '$sequence idle "pose.smd"\n$collisionmodel "body.smd" { $mass 10 }\n')
        qc=dc.decompile(first,self.work/'decompiled');second=self.work/'second.mdl'
        subprocess.run([str(self.compiler),'-quiet','-o',str(second),qc],check=True,capture_output=True,timeout=30)
        report=validate53.compare_physics(first,second)
        self.assertTrue(report['ok'],report)

    def test_render_validator_checks_attributes_and_winding(self):
        (self.work/'body.smd').write_text((ROOT/'tests/box/box.smd').read_text())
        first=self.compile('$modelname "test.mdl"\n$body body "body.smd"\n$sequence idle "body.smd"\n')
        self.assertGreater(validate53.compare_render(first,first)['triangles'],0)
        original=first.read_bytes();data=bytearray(original);h=mdl53.Model(data).hdr
        vv=mdl53.u(mdl53.VVD_HDR,data,h['vvdOffset']);at=h['vvdOffset']+vv['tangentDataStart']
        struct.pack_into('<f',data,at,struct.unpack_from('<f',data,at)[0]+.1)
        corrupt=self.work/'corrupt.mdl';corrupt.write_bytes(data)
        with self.assertRaisesRegex(ValueError,'render triangle'):
            validate53.compare_render(first,corrupt)
        data=bytearray(original);tree=mdl53.parse_vtx(data,h['vtxOffset'])
        sg=tree['bodyparts'][0]['models'][0]['lods'][0]['meshes'][0]['stripgroups'][0]
        at=sg['_off']+sg['indexOffset'];a,b=struct.unpack_from('<2H',data,at)
        struct.pack_into('<2H',data,at,b,a);corrupt.write_bytes(data)
        with self.assertRaisesRegex(ValueError,'oriented render triangle'):
            validate53.compare_render(first,corrupt)

    def test_small_positive_skin_weights_survive_both_inputs(self):
        bones=[{'name':'root','parent':-1},{'name':'tip','parent':0}]
        bind=[((0,0,0),(0,0,0))]*2
        mesh=[(mat,[(p,n,uv,[(0,.99998),(1,.00002)]) for p,n,uv,w in tri])
              for mat,tri in cube((0,0,0),(0,0,1))]
        dc.write_smd(self.work/'body.smd',bones,[bind],mesh)
        dc.write_dmx_mesh(str(self.work/'body.dmx'),bones,bind,
                          [[(p,n,w) for p,n,uv,w in tri] for mat,tri in mesh])
        for ext in ('smd','dmx'):
            first=self.compile('$modelname "test.mdl"\n$body body "body.%s"\n$sequence idle "body.%s"\n'%(ext,ext),ext)
            data=first.read_bytes();vertices,_=validate53.geometry(data,mdl53.Model(data).hdr)
            for vertex in vertices:
                self.assertEqual(len(vertex[1]),2)
                self.assertAlmostEqual(dict(vertex[1])[1],.00002,places=9)
            qc=dc.decompile(first,self.work/('decompiled-'+ext));second=self.work/(ext+'-second.mdl')
            subprocess.run([str(self.compiler),'-quiet','-o',str(second),qc],check=True,capture_output=True,timeout=30)
            validate53.compare_render(first,second)

    def test_rle_euler_branch_survives_past_ninety_degrees(self):
        bones=[{'name':'root','parent':-1},{'name':'tip','parent':0}]
        bind=[((0,0,0),(0,0,0)),((0,0,10),(0,0,0))]
        dc.write_smd(self.work/'body.smd',bones,[bind],cube((0,0,10),(0,0,1),1))
        frames=[[bind[0],((0,0,10),(.005,math.pi/2+(i-15)*.0001,.1))] for i in range(31)]
        dc.write_smd(self.work/'pose.smd',bones,frames,[])
        first=self.compile('$modelname "test.mdl"\n$body body "body.smd"\n'
            '$bonerotscale tip .00001198459 .0000479384 .0000958729\n$sequence pose "pose.smd"\n')
        qc=dc.decompile(first,self.work/'decompiled');second=self.work/'second.mdl'
        subprocess.run([str(self.compiler),'-quiet','-o',str(second),qc],check=True,capture_output=True,timeout=30)
        report=validate53.compare_animations(first,second)
        self.assertFalse(report['issues'],report)

    def test_explicit_model_space_vertices_are_not_rebound_or_rotated(self):
        bones=[{'name':'root','parent':-1}]
        points=[(12345.125,-9876.25,7000.5),(12346.125,-9876.25,7000.5),(12345.125,-9875.25,7000.5)]
        dc.write_dmx_mesh(str(self.work/'body.dmx'),bones,[((12345,0,0),(0,0,0))],
                          [[(p,(0,0,1),[(0,1)]) for p in points]])
        source=self.work/'body.dmx'
        source.write_text(source.read_text().replace('"jointCount" "int" "1"',
                          '"jointCount" "int" "1"\n"r2c_model_space" "bool" "1"'))
        for static in (False,True):
            output=self.compile('$modelname "test.mdl"\n$body body "body.dmx"\n$sequence idle "body.dmx"\n'+
                ('$staticprop\n' if static else '$definebone root "" 2000 1000 0 0 0 0\n'),str(static))
            data=output.read_bytes();vertices,_=validate53.geometry(data,mdl53.Model(data).hdr)
            self.assertEqual({v[0] for v in vertices},set(points))

    def test_partial_final_section_excludes_the_separate_last_frame(self):
        bones=[{'name':'root','parent':-1},{'name':'tip','parent':0}]
        bind=[((0,0,0),(0,0,0)),((0,0,10),(0,math.pi/2,0))]
        dc.write_smd(self.work/'body.smd',bones,[bind],cube((0,0,10),(0,0,1),1))
        angle=(1.4,math.pi/2-.000002,.18)
        frames=[[bind[0],((0,0,10),angle)] for i in range(132)]+[[bind[0],((0,0,10),(0,0,0))]]
        dc.write_smd(self.work/'pose.smd',bones,frames,[])
        first=self.compile('$modelname "test.mdl"\n$body body "body.smd"\n'
            '$bonerotscale tip .00001198459 .00001198459 .000042612\n$sequence pose "pose.smd"\n')
        data=first.read_bytes();model=mdl53.Model(data)
        _,_,decoded=dc.decode_anim(data,model,model.bones(),0,[120,131,132])
        for pose in decoded[:2]:self.assertLess(validate53.rotation_error(pose[1][1],dc.angle_matrix(angle)),.01)
        self.assertLess(validate53.rotation_error(decoded[2][1][1],dc.angle_matrix((0,0,0))),.01)

    def test_model_space_attributes_preserve_native_float_values(self):
        bones=[{'name':'root','parent':-1}]
        points=[(0,0,0),(1,0,0),(0,1,0)]
        extra={'uv':(.12345679104328156,-4095.999755859375),'uv2':(.25,-8191.99951171875),
               'color':(17/255,23/255,31/255,1),'tangent':(1.0123,.01,.01,-1)}
        dc.write_dmx(str(self.work/'body.dmx'),bones,[((0,0,0),(0,0,0))],
                     [('mat',0,[(p,(0,0,1.0123),[(0,1)],extra) for p in points])])
        source=self.work/'body.dmx';text=source.read_text()
        text=text.replace('"jointCount" "int" "1"','"jointCount" "int" "1"\n"r2c_model_space" "bool" "1"')
        source.write_text(text.replace('"flipVCoordinates" "bool" "1"','"flipVCoordinates" "bool" "0"'))
        first=self.compile('$modelname "test.mdl"\n$body body "body.dmx"\n$sequence idle "body.dmx"\n$staticprop\n')
        data=first.read_bytes();vertices,_=validate53.geometry(data,mdl53.Model(data).hdr)
        expected=tuple(validate53.f32(x) for x in (0,0,1.0123,*extra['uv'],*extra['tangent'],*extra['uv2']))
        for vertex in vertices:
            self.assertEqual(vertex[2][0],expected)
            self.assertEqual(vertex[2][1],(17,23,31,255))

    def test_render_comparison_does_not_merge_nontransitive_vertices(self):
        (self.work/'body.smd').write_text((ROOT/'tests/box/box.smd').read_text())
        model=self.compile('$modelname "test.mdl"\n$body body "body.smd"\n$sequence idle "body.smd"\n')
        data=bytearray(model.read_bytes());h=mdl53.Model(data).hdr;vv=mdl53.u(mdl53.VVD_HDR,data,h['vvdOffset'])
        at=h['vvdOffset']+vv['vertexDataStart'];ta=h['vvdOffset']+vv['tangentDataStart']
        for i,p in enumerate(((0,0,0),(.0018,0,0),(.0009,.00001,0))):
            data[at+i*48:at+(i+1)*48]=data[at:at+48]
            struct.pack_into('<3f',data,at+i*48+16,*p)
            data[ta+i*16:ta+(i+1)*16]=data[ta:ta+16]
        model.write_bytes(data)
        validate53.compare_render(model,model)

    def test_empty_triangle_collision_survives_decompilation(self):
        (self.work/'body.smd').write_text((ROOT/'tests/box/box.smd').read_text())
        (self.work/'empty.pertri').write_text('r2c_pertri 1\n-32 -32 0 32 32 64\n0 0\n')
        first=self.compile('$modelname "test.mdl"\n$staticprop\n$body body "body.smd"\n'
            '$sequence idle "body.smd"\n$pertrimesh "empty.pertri"\n')
        qc=dc.decompile(first,self.work/'decompiled');second=self.work/'second.mdl'
        self.assertIn('$pertrimesh',Path(qc).read_text())
        subprocess.run([str(self.compiler),'-quiet','-o',str(second),qc],check=True,capture_output=True,timeout=30)
        self.assertEqual(validate53.validate(second,False)['collision_triangles'],0)
        self.assertTrue(validate53.validate_against(first,second)['ok'])

    def test_map_collision_comparison_detects_an_outward_plane(self):
        (self.work/'body.smd').write_text((ROOT/'tests/box/box.smd').read_text())
        first=self.compile('$modelname "test.mdl"\n$body body "body.smd"\n$sequence idle "body.smd"\n'
            '$collisionmodel "body.smd" { $mass 50 }\n')
        data=bytearray(first.read_bytes());h=mdl53.Model(data).hdr;base=h['collisionOffset']
        at=base+struct.unpack_from('<i',data,base+16)[0]+12
        struct.pack_into('<f',data,at,struct.unpack_from('<f',data,at)[0]+1)
        corrupt=self.work/'corrupt.mdl';corrupt.write_bytes(data)
        with self.assertRaisesRegex(ValueError,'brush geometry'):
            validate53.compare_map_collision(first,corrupt)

    def test_unchanged_bind_euler_branch_survives_world_rebuild(self):
        bones=[{'name':'root','parent':-1},{'name':'tip','parent':0}]
        bind=[((100,200,300),(.3,.7,.9)),((7,11,13),(0,math.pi/2-.0000014,1.68130135))]
        dc.write_smd(self.work/'body.smd',bones,[bind],cube((0,0,10),(0,0,1),1))
        pitch=math.degrees(bind[1][1][1]);yaw=math.degrees(bind[1][1][2])
        first=self.compile('$modelname "test.mdl"\n$body body "body.smd"\n'
            '$definebone root "" 100 200 300 %.9g %.9g %.9g\n'
            '$definebone tip root 7 11 13 %.9g %.9g 0\n$sequence idle "body.smd"\n'%
            (math.degrees(.7),math.degrees(.9),math.degrees(.3),pitch,yaw))
        b=mdl53.Model(first.read_bytes()).bones()[1]
        self.assertLess(validate53.rotation_error(dc.angle_matrix(bind[1][1]),dc.angle_matrix(b['rot'])),.0001)
        self.assertEqual(b['pos'],[7,11,13])

    def test_literal_collision_face_and_bevel_planes_roundtrip(self):
        (self.work/'body.smd').write_text((ROOT/'tests/box/box.smd').read_text())
        first=self.compile('$modelname "test.mdl"\n$body body "body.smd"\n$sequence idle "body.smd"\n'
            '$collisionmodel "body.smd" { $mass 50 }\n')
        data=bytearray(first.read_bytes());h=mdl53.Model(data).hdr;base=h['collisionOffset']
        at=base+struct.unpack_from('<i',data,base+16)[0]+12
        struct.pack_into('<f',data,at,struct.unpack_from('<f',data,at)[0]-.125)
        first.write_bytes(data)
        qc=dc.decompile(first,self.work/'decompiled');second=self.work/'second.mdl'
        self.assertIn('r2c_collision_planes',(self.work/'decompiled'/'test_physics.dmx').read_text())
        subprocess.run([str(self.compiler),'-quiet','-o',str(second),qc],check=True,capture_output=True,timeout=30)
        before=dc.collision_block_faces(data,h,mdl53.Model(data).bones(),with_planes=True)
        db=second.read_bytes();mb=mdl53.Model(db)
        after=dc.collision_block_faces(db,mb.hdr,mb.bones(),with_planes=True)
        self.assertEqual(before[0][2],after[0][2])
        validate53.compare_map_collision(first,second)
        pa=validate53.physics_data(first);pb=validate53.physics_data(second)
        self.assertEqual(validate53.convex_pieces(pa['solids'][0])[0][3],
                         validate53.convex_pieces(pb['solids'][0])[0][3])
        self.assertTrue(validate53.compare_physics(first,second)['ok'])
        # Native brush planes can extend inside their vertices; comparing a
        # brush against itself must still succeed without relaxing the limit.
        self.assertEqual(validate53.compare_map_collision(first,first)['maximum_plane_error'],0)

    def test_stored_collision_surface_rejects_invalid_indices(self):
        (self.work/'body.smd').write_text((ROOT/'tests/box/box.smd').read_text())
        first=self.compile('$modelname "test.mdl"\n$body body "body.smd"\n$sequence idle "body.smd"\n'
            '$collisionmodel "body.smd" { $mass 50 }\n')
        qc=dc.decompile(first,self.work/'decompiled')
        physics=self.work/'decompiled'/'test_physics.dmx'
        text=physics.read_text();marker='"triangles" "int_array"'
        at=text.index(marker)+len(marker);end=text.index(']',at)
        content=text[at:end];import re
        changed=re.sub(r'"-?\d+"','"999999"',content,count=1)
        self.assertNotEqual(content,changed)
        physics.write_text(text[:at]+changed+text[end:])
        run=subprocess.run([str(self.compiler),'-quiet','-o',str(self.work/'bad.mdl'),qc],capture_output=True,text=True,timeout=30)
        self.assertNotEqual(run.returncode,0)
        self.assertIn('stored physics point index out of range',run.stdout+run.stderr)

    def test_physics_comparison_is_reflexive_for_nonconvex_closed_surface(self):
        from unittest.mock import patch
        (self.work/'body.smd').write_text((ROOT/'tests/box/box.smd').read_text())
        first=self.compile('$modelname "test.mdl"\n$body body "body.smd"\n$sequence idle "body.smd"\n'
            '$collisionmodel "body.smd" { $mass 50 }\n')
        physical=validate53.physics_data(first)
        solid=physical['solids'][0];points=solid['points']
        # Indent a corner without changing the closed edge topology. Infinite
        # triangle planes no longer describe this surface, as with skinny native
        # ledges, but its literal oriented triangles still compare exactly.
        p=points[0];points[0]=tuple(sum(q[k] for q in points)/len(points) for k in range(3))+tuple(p[3:])
        piece=validate53.convex_pieces(solid)[0]
        self.assertGreater(validate53.support_error(piece,piece),validate53.PHYSICS_TOLERANCE)
        with patch.object(validate53,'physics_data',return_value=physical):
            self.assertTrue(validate53.compare_physics(first,first)['ok'])

    def test_serialized_physics_is_convex_after_unit_conversion(self):
        bones=[{'name':'root','parent':-1}]
        profile=[(0,0),(60,0),(60,5),(30,5.000001),(0,5)]
        points=[(x,y,z) for z in (0,96) for x,y in profile];n=len(profile)
        faces=[list(reversed(range(n))),list(range(n,n*2))]+[[i,(i+1)%n,(i+1)%n+n,i+n] for i in range(n)]
        dc.write_dmx_mesh(str(self.work/'body.dmx'),bones,[((0,0,0),(0,0,0))],
                          [[(points[i],(0,0,1),[(0,1)]) for i in face] for face in faces])
        first=self.compile('$modelname "test.mdl"\n$body body "body.dmx"\n$sequence idle "body.dmx"\n'
            '$collisionmodel "body.dmx" { $mass 50 }\n')
        for surface in validate53.physics_data(first)['solids']:
            for piece in validate53.convex_pieces(surface):
                self.assertLess(v_error:=validate53.support_error(piece,piece),.001,v_error)

    def test_validate_corpus_duplicate_names_and_resume(self):
        import hashlib
        import json
        import shutil
        (self.work/'body.smd').write_text((ROOT/'tests/box/box.smd').read_text())
        output=self.compile('$modelname "test.mdl"\n$staticprop\n$body body "body.smd"\n$sequence idle "body.smd"\n')
        folder=self.work/'corpus';folder.mkdir(); report={}
        for name in ('a','b'):
            source=self.work/name/'same.mdl';source.parent.mkdir();shutil.copyfile(output,source)
            suffix=hashlib.sha256(str(source).encode()).hexdigest()[:12]
            work=folder/('same-'+suffix);work.mkdir();shutil.copyfile(output,work/'recompiled.mdl')
            report[str(source)]={'status':'ok','input_stamp':[source.stat().st_size,source.stat().st_mtime_ns]}
        path=folder/'results.json';path.write_text(json.dumps(report))
        dest=self.work/'verified'
        command=[sys.executable,str(ROOT/'tools/validate_corpus.py'),'--jobs','1','--report',str(path),'--out',str(dest)]
        for extra in ([],['--resume']):
            run=subprocess.run(command+extra,capture_output=True,text=True,timeout=60)
            self.assertEqual(run.returncode,0,run.stdout+run.stderr)
        summary=json.loads((dest/'summary.json').read_text())
        self.assertEqual(summary['counts'],{'ok':2})
        self.assertEqual(summary['resumed'],2)
        path.write_text(path.read_text()+'\n')
        run=subprocess.run(command+['--resume'],capture_output=True,text=True,timeout=60)
        self.assertEqual(run.returncode,2)
        self.assertIn('cannot resume',run.stderr)

    def test_ik_rules_compile_and_roundtrip(self):
        bones = [{'name':'root','parent':-1}, {'name':'hip','parent':0},
                 {'name':'knee','parent':1}, {'name':'foot','parent':2}]
        bind = [((0,0,0),(0,0,0)), ((0,0,30),(0,0,0)), ((0,0,-15),(0,0,0)), ((0,0,-15),(0,0,0))]
        dc.write_smd(self.work / 'body.smd', bones, [bind], cube((0,0,0),(0,0,1),3))
        # a foot that lifts and plants while the knee swings
        frames = [[bind[0], bind[1], ((0,0,-15),(.4 * math.sin(i * .5),0,0)),
                   ((0,0,-15 + 3 * max(0, math.sin(i * .5))),(0,0,0))] for i in range(13)]
        dc.write_smd(self.work / 'walk.smd', bones, frames)
        qc = ('$modelname "test.mdl"\n$body body "body.smd"\n'
              '$ikchain "leg" "foot" knee 1 0 0\n'
              '$animation walk "walk.smd" fps 30 loop ikrule "leg" footstep range 0 2 6 8 contact 4 '
              'height 50 radius 15 floor 0\n'
              '$animation still "walk.smd" fps 30\n'
              '$sequence walk walk\n$sequence still still\n')
        first = self.compile(qc)
        data = first.read_bytes()
        model = mdl53.Model(data)
        walk = dc.decode_ik_rules(data, model.hdr, 0)
        self.assertEqual([r['type'] for r in walk], [3])
        self.assertTrue(walk[0]['compressedikerrorindex'])
        self.assertAlmostEqual(walk[0]['endHeight'], -100)
        self.assertEqual([round(walk[0][x] * 12) for x in ('start','peak','tail','end','contact')], [0,2,6,8,4])
        # automatic release for a chain the animation moves without rules
        self.assertEqual([r['type'] for r in dc.decode_ik_rules(data, model.hdr, 1)], [4])
        qc2 = dc.decompile(first, self.work / 'decompiled')
        second = self.compile(Path(qc2).read_text().replace('"test_bp', '"decompiled/test_bp')
                              .replace('"anim_', '"decompiled/anim_'), 'second')
        self.assertEqual(roundtrip.compare(roundtrip.summary(first), roundtrip.summary(second)), [])

    def test_ik_overwritten_ranges_roundtrip(self):
        # studiomdl's "." range values overwrite a neighbouring rule's end or start after
        # that rule's errors were computed. When the overwritten range stays in order,
        # only the error stream shows it; the decompiler must recover the original range
        # (BT's single-player animations do this).
        bones = [{'name':'root','parent':-1}, {'name':'hip','parent':0},
                 {'name':'knee','parent':1}, {'name':'foot','parent':2}]
        bind = [((0,0,0),(0,0,0)), ((0,0,30),(0,0,0)), ((0,0,-15),(0,0,0)), ((0,0,-15),(0,0,0))]
        dc.write_smd(self.work / 'body.smd', bones, [bind], cube((0,0,0),(0,0,1),3))
        frames = [[bind[0], ((3 * math.sin(i * .07), 2 * math.cos(i * .05), 30),(0,0,0)),
                   ((0,0,-15),(.4 * math.sin(i * .1),0,0)), ((0,0,-15),(0,0,0))] for i in range(117)]
        dc.write_smd(self.work / 'walk.smd', bones, frames)
        step = 'height 50 radius 15 floor 0'
        qc = ('$modelname "test.mdl"\n$body body "body.smd"\n$ikchain "leg" "foot" knee 1 0 0\n'
              # B's "." start rewrites A's end from 44 to 39 (still after A's tail)
              '$animation endcase "walk.smd" fps 30 ikrule "leg" footstep range 0 11 28 44 contact 11 %s '
              'ikrule "leg" footstep range . 50 62 70 contact 50 %s\n'
              # C's "." end wraps and rewrites A's start from 0 to 5 (still before A's peak)
              '$animation startcase "walk.smd" fps 30 ikrule "leg" footstep range 0 11 28 39 contact 11 %s '
              'ikrule "leg" footstep range 39 50 62 70 contact 50 %s '
              'ikrule "leg" footstep range 70 78 116 . contact 78 %s\n'
              '$sequence endcase endcase\n$sequence startcase startcase\n') % ((step,) * 5)
        first = self.compile(qc)
        data = first.read_bytes()
        header = mdl53.Model(data).hdr
        rules = [dc.decode_ik_rules(data, header, a) for a in (0, 1)]
        self.assertEqual([round(rules[0][0][x] * 116) for x in ('start', 'end')], [0, 39])
        self.assertEqual(dc.ik_error_frames(data, rules[0][0]), 45)
        self.assertEqual([round(rules[1][0][x] * 116) for x in ('start', 'end')], [5, 39])
        self.assertEqual(dc.ik_error_frames(data, rules[1][0]), 40)
        qc2 = dc.decompile(first, self.work / 'decompiled')
        second = self.compile(Path(qc2).read_text().replace('"test_bp', '"decompiled/test_bp')
                              .replace('"anim_', '"decompiled/anim_'), 'second')
        data2 = second.read_bytes()
        header2 = mdl53.Model(data2).hdr
        for a in (0, 1):
            self.assertEqual([dc.ik_error_frames(data2, r) for r in dc.decode_ik_rules(data2, header2, a)],
                             [dc.ik_error_frames(data, r) for r in rules[a]])
        self.assertEqual(roundtrip.compare(roundtrip.summary(first), roundtrip.summary(second)), [])

    def test_ikfixup_locks_the_foot(self):
        bones = [{'name':'root','parent':-1}, {'name':'hip','parent':0},
                 {'name':'knee','parent':1}, {'name':'foot','parent':2}]
        bind = [((0,0,0),(0,0,0)), ((0,0,30),(0,0,0)), ((5,0,-15),(0,0,0)), ((-5,0,-15),(0,0,0))]
        dc.write_smd(self.work / 'body.smd', bones, [bind], cube((0,0,0),(0,0,1),3))
        # the hip slides forward, dragging the planted foot with it
        frames = [[bind[0], ((i, 0, 30), (0,0,0)), bind[2], bind[3]] for i in range(9)]
        dc.write_smd(self.work / 'slide.smd', bones, frames)
        qc = ('$modelname "test.mdl"\n$body body "body.smd"\n$ikchain "leg" "foot" knee 1 0 0\n'
              '$sequence slide "slide.smd" fps 30 %s\n')
        def legs(output):
            data = output.read_bytes()
            model = mdl53.Model(data)
            _, _, poses = dc.decode_anim(data, model, model.bones(), 0)
            return [roundtrip.world_positions(model.bones(), p) for p in poses]
        plain_legs = legs(self.compile(qc % '', 'plain'))
        fixed_legs = legs(self.compile(qc % 'ikfixup "leg" footstep range 0 2 6 8 contact 4', 'fixed'))
        plain, fixed = [w[3] for w in plain_legs], [w[3] for w in fixed_legs]
        for w in fixed_legs:                                    # the solve bends the knee, not the bones
            self.assertAlmostEqual(math.dist(w[1], w[2]), math.dist(plain_legs[0][1], plain_legs[0][2]), places=2)
            self.assertAlmostEqual(math.dist(w[2], w[3]), math.dist(plain_legs[0][2], plain_legs[0][3]), places=2)
        self.assertGreater(math.dist(plain[2], plain[6]), 3)    # without the fixup the foot slides
        for f in range(2, 7):                                   # with it the foot stays planted
            self.assertLess(math.dist(fixed[f], plain[4]), 0.1)
        self.assertLess(math.dist(fixed[0], plain[0]), 0.1)     # outside the range nothing changes
        self.assertLess(math.dist(fixed[8], plain[8]), 0.1)

    def write_procedural_model(self, folder, procedural=True):
        """Driver bone turns 90 degrees; follower (quatinterp), aimer (aim at target) and
        axisbone (axisinterp) react. Attachments expose the results."""
        folder.mkdir(parents=True, exist_ok=True)
        bones = [{'name':'root','parent':-1}, {'name':'driver','parent':0}, {'name':'follower','parent':0},
                 {'name':'aimer','parent':0}, {'name':'target','parent':0}, {'name':'axisbone','parent':0}]
        bind = [((0,0,0),(0,0,0)), ((0,0,20),(0,0,0)), ((10,0,20),(0,0,0)),
                ((-10,0,20),(0,0,0)), ((-10,30,20),(0,0,0)), ((0,10,20),(0,0,0))]
        geo = cube((0,0,0),(0,0,1),0) + cube((10,0,20),(0,0,1),2) + cube((-10,0,20),(0,0,1),3)
        dc.write_smd(folder / 'body.smd', bones, [bind], geo)
        frames = []
        for i in range(31):
            f = list(bind)
            f[1] = ((0,0,20),(0,0,math.radians(3 * i)))      # driver yaw 0 -> 90
            f[4] = ((-10,30,20 + i),(0,0,0))                 # target rises 30 units
            frames.append(f)
        dc.write_smd(folder / 'turn.smd', bones, frames)
        (folder / 'proc.vrd').write_text(
            '<helper> follower root root driver\n<display> 1 1 1 0\n<basepos> 0 0 0\n'
            '<trigger> 90 0 0 0 0 0 0 10 0 20\n<trigger> 90 0 0 90 0 0 0 10 0 40\n'
            '<aimconstraint> aimer root target\n<aimvector> 1 0 0\n<upvector> 0 0 1\n<basepos> 0 0 0\n')
        (folder / 'axis.txt').write_text(
            'version 1\nproceduralbones\nbone "axisbone" "root" "driver" "root" 3\nbasepos 0 0 0\n' +
            ''.join('axis %d 0 10 %d 0 0 0\n' % (k, 20 + k) for k in range(6)) + 'end\n')
        qc = ('$modelname "proc.mdl"\n$body body "body.smd"\n'
              '$attachment "follower_att" "follower" 0 0 0\n$attachment "aimer_tip" "aimer" 10 0 0\n'
              '$attachment "driver_tip" "driver" 10 0 0\n$attachment "target_att" "target" 0 0 0\n'
              '$sequence turn "turn.smd" fps 10\n')
        if procedural:
            qc += '$proceduralbones "proc.vrd"\n$proceduralbones "axis.txt"\n'
        (folder / 'proc.qc').write_text(qc)
        return folder / 'proc.qc'

    def test_procedural_bones_compile_and_roundtrip(self):
        qc = self.write_procedural_model(self.work / 'proc')
        output = self.work / 'proc.mdl'
        run = subprocess.run([str(self.compiler), '-quiet', '-o', str(output), str(qc)], capture_output=True, text=True, timeout=30)
        self.assertEqual(run.returncode, 0, run.stdout + run.stderr)
        data = output.read_bytes()
        model = mdl53.Model(data)
        bones = {b['name']: (i, b) for i, b in enumerate(model.bones())}
        self.assertEqual({n: bones[n][1]['proctype'] for n in ('follower', 'aimer', 'axisbone')},
                         {'follower': 2, 'aimer': 3, 'axisbone': 1})
        follower = dc.decode_procedural(data, model.hdr, *bones['follower'])
        self.assertEqual(follower['control'], bones['driver'][0])
        self.assertEqual(len(follower['triggers']), 2)
        self.assertAlmostEqual(follower['triggers'][1]['pos'][2], 40, places=3)
        aimer = dc.decode_procedural(data, model.hdr, *bones['aimer'])
        self.assertEqual(aimer['aim'], bones['target'][0])
        self.assertTrue(bones['follower'][1]['flags'] & 0x04)
        qc2 = dc.decompile(output, self.work / 'decompiled')
        second = self.compile(Path(qc2).read_text().replace('"proc_bp', '"decompiled/proc_bp')
                              .replace('"anim_', '"decompiled/anim_')
                              .replace('"proc_procedural.vrd', '"decompiled/proc_procedural.vrd')
                              .replace('"proc_axisinterp.txt', '"decompiled/proc_axisinterp.txt'), 'second')
        self.assertEqual(roundtrip.compare(roundtrip.summary(output), roundtrip.summary(second)), [])

    def test_ui_panel_hash_and_roundtrip(self):
        # The client finds a panel by a case-folded murmur3 hash of the weapon script's
        # "mesh" name; the compiler derives it, and an explicit hash still overrides it.
        (self.work / 'body.smd').write_text((ROOT / 'tests/box/box.smd').read_text())
        panel = ('{\n parent "root"\n vertex 0 0 -1 1\n vertex 0 0 1 1\n vertex 0 0 1 0\n'
                 ' face 0 1 2 ccw\n bounds 0 1 1 0 0 0 1 1\n}\n')
        output = self.compile('$modelname "test/ui.mdl"\n$body body "body.smd"\n$sequence idle "body.smd"\n'
                              '$uipanel "models/weapons/attachments/p2011_rui_lower"\n' + panel +
                              '$uipanel "models/test/other" 0x12345678\n' + panel)
        data = output.read_bytes()
        panels = dc.decode_ui_panels(data, mdl53.Model(data).hdr)
        # 0xf4ab7fb9 is retail ptpov_p2011's hash for the same name
        self.assertEqual([(p['name'], p['hash']) for p in panels],
                         [('models/weapons/attachments/p2011_rui_lower', 0xf4ab7fb9), ('models/test/other', 0x12345678)])
        self.assertEqual(dc.ui_panel_hash('MODELS/Weapons/attachments/p2011_rui_lower'), 0xf4ab7fb9)
        self.assertEqual(panels[0]['ccw'], 1)
        qc2 = dc.decompile(output, self.work / 'decompiled')
        text = Path(qc2).read_text()
        self.assertIn('$uipanel "models/weapons/attachments/p2011_rui_lower"\n', text)
        self.assertIn('$uipanel "models/test/other" 0x12345678', text)
        second = self.compile(text.replace('"test_bp', '"decompiled/test_bp').replace('"anim_', '"decompiled/anim_'), 'second')
        self.assertEqual(roundtrip.compare(roundtrip.summary(output), roundtrip.summary(second)), [])

    def test_crowbar_command_line_and_output_path(self):
        # Crowbar sets the working directory to the QC folder, supplies the QC
        # basename, and passes -game plus its selected studiomdl options.
        source = (ROOT / 'tests/box/box.smd').read_text()
        (self.work / 'body.smd').write_text(source)
        (self.work / 'fixture.qc').write_text('$modelname "test/subfolder/box.mdl"\n'
            '$staticprop\n$body body "body.smd"\n$sequence idle "body.smd"\n')
        game = self.work / 'game with spaces'
        run = subprocess.run([str(self.compiler), '-game', str(game), '-nop4', '-verbose',
                              '-nox360', 'fixture.qc'], cwd=self.work, capture_output=True, text=True)
        self.assertEqual(run.returncode, 0, run.stdout + run.stderr)
        model = game / 'models/test/subfolder/box.mdl'
        header = mdl53.Model(model.read_bytes()).hdr
        self.assertEqual(header['version'], 53)
        self.assertEqual(header['length'], model.stat().st_size)
        self.assertIn('Completed "fixture.qc"', run.stdout)

    def test_binary_and_keyvalues2_dmx_match(self):
        summaries = []
        for suffix in ('', '_kv2'):
            # Use absolute source paths, as this QC lives in a temporary folder.
            mesh = (ROOT / ('tests/dmx/cube_arm%s.dmx' % suffix)).as_posix()
            motion = (ROOT / ('tests/dmx/x_axis%s.dmx' % suffix)).as_posix()
            output = self.compile('$modelname "dmx.mdl"\n$body body "%s"\n'
                                  '$sequence idle "%s"\n$sequence move "%s" fps 30\n'
                                  % (mesh,mesh,motion), 'dmx' + suffix)
            summaries.append(roundtrip.summary(output))
        self.assertEqual(roundtrip.compare(*summaries), [])

    def test_dmx_second_uv_set_and_vertex_colour(self):
        # Blender Source Tools (model format 22) writes extra UV maps as texcoord$1 and
        # colour attributes as color$0; both land in the VVC block, as the UV2 and
        # vertex-colour header flags announce.
        text = (ROOT / 'tests/dmx/cube_arm_kv2.dmx').read_text()
        text = text.replace('"texcoord$0", ', '"texcoord$0", "texcoord$1", "color$0", ')
        extra = ('\t"texcoord$1" "vector2_array" ["0.25 0.75"]\n'
                 '\t"texcoord$1Indices" "int_array" [%s]\n'
                 '\t"color$0" "vector4_array" ["1.0 0.0 0.0 1.0"]\n'
                 '\t"color$0Indices" "int_array" [%s]\n') % ((', '.join(['"0"'] * 24),) * 2)
        anchor = '\t"texcoord$0" "vector2_array"'
        self.assertIn(anchor, text)
        (self.work / 'uv2.dmx').write_text(text.replace(anchor, extra + anchor, 1))
        output = self.compile('$modelname "uv2.mdl"\n$body body "uv2.dmx"\n$sequence idle "uv2.dmx"\n')
        data = output.read_bytes()
        header = mdl53.Model(data).hdr
        self.assertTrue(header['flags'] & 0x2000000)  # uses UV2
        self.assertTrue(header['flags'] & 0x1000000)  # uses vertex colour
        vvc = data[header['vvcOffset']:header['vvcOffset'] + header['vvcSize']]
        self.assertEqual(vvc[:4], b'IDCV')
        lod0 = struct.unpack_from('<i', vvc, 16)[0]
        colours, uv2 = struct.unpack_from('<2i', vvc, 48)
        self.assertGreater(lod0, 0)
        for i in range(lod0):
            self.assertEqual(tuple(vvc[colours + 4 * i:colours + 4 * i + 4]), (255, 0, 0, 255))
            self.assertEqual(struct.unpack_from('<2f', vvc, uv2 + 8 * i), (0.25, 0.25))  # V is flipped

    def test_dmx_tangents_are_kept_and_keep_vertices_apart(self):
        # Decompiled meshes carry retail's tangents (tangent$0). The compiler stores them as
        # given, and corners that differ only in tangent stay separate vertices.
        text = (ROOT / 'tests/dmx/cube_arm_kv2.dmx').read_text()
        text = text.replace('"texcoord$0", ', '"texcoord$0", "tangent$0", ')
        # one normal and one UV for every corner, so only positions and tangents differ
        zeros = '[%s]' % ', '.join(['"0"'] * 24)
        lines = text.split('\n')
        for i, line in enumerate(lines):
            if line.lstrip().startswith(('"normal$0Indices"', '"texcoord$0Indices"')):
                lines[i] = line[:line.index('[')] + zeros
        text = '\n'.join(lines)
        anchor = '\t"texcoord$0" "vector2_array"'
        self.assertIn(anchor, text)

        def build(name, tangent_indices):
            extra = ('\t"tangent$0" "vector4_array" ["0.0 1.0123 0.0 -1.0", "0.0 0.0 1.0 1.0"]\n'
                     '\t"tangent$0Indices" "int_array" [%s]\n') % ', '.join('"%d"' % i for i in tangent_indices)
            (self.work / (name + '.dmx')).write_text(text.replace(anchor, extra + anchor, 1))
            output = self.compile('$modelname "%s.mdl"\n$body body "%s.dmx"\n$sequence idle "%s.dmx"\n' % ((name,) * 3), name)
            data = output.read_bytes()
            header = mdl53.Model(data).hdr
            vvd = mdl53.u(mdl53.VVD_HDR, data, header['vvdOffset'])
            count = vvd['numLODVertexes'][0]
            tangents = [struct.unpack_from('<4f', data, header['vvdOffset'] + vvd['tangentDataStart'] + 16 * i)
                        for i in range(count)]
            return count, tangents

        same, tangents = build('same', [0] * 24)
        for t in tangents:
            self.assertAlmostEqual(t[1],1.0123,places=6)
            self.assertEqual((t[0],t[2],t[3]),(0.0,0.0,-1.0))
        split, tangents = build('split', [i % 2 for i in range(24)])
        self.assertGreater(split, same)
        self.assertIn((0.0, 0.0, 1.0, 1.0), tangents)

    def test_retail_header_extensions_roundtrip(self):
        # r2c extensions for retail values plain QC can't express: header mass without
        # physics, an exact contents mask, source bone transforms and a non-rotation
        # attachment matrix. A single-bone model also gets retail's 0x800000 flag.
        (self.work / 'body.smd').write_text((ROOT / 'tests/box/box.smd').read_text())
        pre = '1 0 0 0 0 1 0 0 0 0 1 0'
        post = '1 0 0 0.25 0 1 0 0 0 0 1 0'
        output = self.compile('$modelname "test.mdl"\n$body body "body.smd"\n$sequence idle "body.smd"\n'
                              '$mass 8\n$contents 2\n'
                              '$srcbonetransform "root" %s %s\n' % (pre, post) +
                              '$attachment "zero" "root" 0 0 0 matrix 0 0 0 0 0 0 0 0 0 0 0 0\n')
        data = output.read_bytes()
        header = mdl53.Model(data).hdr
        self.assertEqual(header['mass'], 8.0)
        self.assertEqual(header['contents'], 2)
        self.assertTrue(header['flags'] & 0x800000)
        self.assertEqual(header['numsrcbonetransform'], 1)
        o = header['srcbonetransformindex']
        self.assertEqual(mdl53.cstr(data, o + struct.unpack_from('<i', data, o)[0]), 'root')
        self.assertEqual(struct.unpack_from('<12f', data, o + 52)[3], 0.25)
        attachment = mdl53.u(mdl53.ATTACH_FIELDS, data, header['localattachmentindex'])
        self.assertEqual(list(attachment['local']), [0.0] * 12)
        qc_path = dc.decompile(output, self.work / 'decoded')
        qc = Path(qc_path).read_text()
        for line in ('$mass 8', '$contents 2', '$srcbonetransform "root"', 'matrix 0 0 0 0 0 0 0 0 0 0 0 0'):
            self.assertIn(line, qc)
        rebuilt = self.work / 'rebuilt.mdl'
        run = subprocess.run([str(self.compiler), '-quiet', '-o', str(rebuilt), str(qc_path)],
                             capture_output=True, text=True, timeout=30)
        self.assertEqual(run.returncode, 0, run.stdout + run.stderr)
        self.assertEqual(roundtrip.compare(roundtrip.summary(output), roundtrip.summary(rebuilt)), [])

    def test_single_frame_retail_nan_event_cycle(self):
        (self.work / 'body.smd').write_text((ROOT / 'tests/box/box.smd').read_text())
        output = self.compile('$modelname "test.mdl"\n$body body "body.smd"\n'
                              '$sequence idle "body.smd" { event "AE_SCRIPTED_ANIMATION" 0 "test" }\n')
        data = bytearray(output.read_bytes())
        model = mdl53.Model(data)
        seq = model.hdr['localseqindex']
        event_index = struct.unpack_from('<i', data, seq + 0x1c)[0]
        struct.pack_into('<f', data, seq + event_index, float('nan'))
        output.write_bytes(data)
        qc = Path(dc.decompile(output, self.work / 'decoded')).read_text()
        self.assertIn('event "AE_SCRIPTED_ANIMATION" 0 "test"', qc)

    def test_noforceloop_and_quoted_event_options(self):
        bones = [{'name':'root','parent':-1}]
        frames = [[((float(i),0,0),(0,0,0))] for i in range(3)]
        dc.write_smd(self.work / 'body.smd', bones, [frames[0]], cube((0,0,0),(0,0,1)))
        dc.write_smd(self.work / 'motion.smd', bones, frames)
        options = 'say "hello" models\\northstar'
        output = self.compile('$modelname "test.mdl"\n$body body "body.smd"\n'
            '$sequence move "motion.smd" loop noforceloop { event "AE_SCRIPTED_ANIMATION" 0 %s }\n'
            % dc.qc_quote(options))
        data = output.read_bytes()
        model = mdl53.Model(data)
        _, _, poses = dc.decode_anim(data, model, model.bones(), 0)
        start, end = poses[0][0][0], poses[-1][0][0]
        self.assertAlmostEqual(math.dist(start,end), 2., places=2)
        seq = model.hdr['localseqindex']
        event_index = struct.unpack_from('<i', data, seq+0x1c)[0]
        event = mdl53.u(mdl53.EVENT_FIELDS,data,seq+event_index)
        self.assertEqual(event['options'],options)
        qc = Path(dc.decompile(output,self.work/'decoded')).read_text()
        self.assertIn(dc.qc_quote(options),qc)

    def test_unterminated_input_strings_report_errors(self):
        qc = self.work / 'bad.qc'
        qc.write_text('$modelname "unfinished')
        run = subprocess.run([str(self.compiler), '-quiet', str(qc)],capture_output=True,text=True,timeout=10)
        self.assertNotEqual(run.returncode,0)
        self.assertIn('unterminated quoted string',run.stdout)
        dmx = self.work / 'bad.dmx'
        dmx.write_text('<!-- dmx encoding keyvalues2 1 format model 22 -->\n"DmeModel" { "name" "string" "unfinished')
        qc.write_text('$modelname "test.mdl"\n$body body "bad.dmx"\n')
        run = subprocess.run([str(self.compiler), '-quiet', str(qc)],capture_output=True,text=True,timeout=10)
        self.assertNotEqual(run.returncode,0)
        self.assertIn('unterminated keyvalues2 string',run.stdout)

    def test_raw_rotation_near_180_degrees_roundtrips(self):
        bones = [{'name':'root','parent':-1}, {'name':'tip','parent':0}]
        bind = [((0,0,0),(0,0,0)), ((6000,0,0),(0,0,0))]
        dc.write_smd(self.work/'body.smd',bones,[bind],cube((6000,0,0),(0,0,1),1))
        dc.write_smd(self.work/'motion.smd',bones,[[((0,0,0),(math.pi,0,0)),bind[1]]]*3)
        first = self.compile('$modelname "test.mdl"\n$body body "body.smd"\n$sequence move "motion.smd"\n')
        qc = dc.decompile(first,self.work/'decoded')
        output = self.work/'second.mdl'
        run = subprocess.run([str(self.compiler),'-quiet','-o',str(output),str(qc)],capture_output=True,text=True)
        self.assertEqual(run.returncode,0,run.stdout+run.stderr)
        self.assertEqual(roundtrip.compare(roundtrip.summary(first),roundtrip.summary(output)),[])


if __name__ == '__main__':
    unittest.main()
