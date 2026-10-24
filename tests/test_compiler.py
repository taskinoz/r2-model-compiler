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
