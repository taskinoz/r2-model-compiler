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
