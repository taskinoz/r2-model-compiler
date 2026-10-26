"""Regression coverage for interrupted corpus runs and duplicate model names."""
import contextlib
import io
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import roundtrip
import structdiff


class CorpusReports(unittest.TestCase):
    def test_structdiff_checks_duplicate_model_names(self):
        scratch = Path(__file__).resolve().parents[1] / 'test-output'
        scratch.mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=scratch) as temporary:
            root = Path(temporary)
            report = {}
            for i, folder in enumerate(('a', 'b')):
                source = root / folder / 'same.mdl'
                source.parent.mkdir()
                source.write_bytes(bytes([i]))
                report[str(source)] = {'status': 'ok'}
                suffix = hashlib.sha256(str(source).encode()).hexdigest()[:12]
                work = root / ('same-' + suffix)
                work.mkdir()
                (work / 'recompiled.mdl').write_bytes(bytes([i]))
            (root / 'results.json').write_text(json.dumps(report))
            output = root / 'diff.json'
            argv = ['structdiff.py', '--report', str(root / 'results.json'), '--out', str(output)]
            with mock.patch.object(sys, 'argv', argv), \
                 mock.patch.object(structdiff, 'dump', side_effect=lambda data: {'value': data[0]}), \
                 contextlib.redirect_stdout(io.StringIO()):
                structdiff.main()
            self.assertEqual(json.loads(output.read_text()), {'compared': 2, 'fields': {}})

    def test_checkpoint_resume_and_tool_change(self):
        scratch = Path(__file__).resolve().parents[1] / 'test-output'
        scratch.mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=scratch) as temporary:
            root = Path(temporary)
            inputs = []
            for folder in ('a', 'b'):
                path = root / folder / 'same.mdl'
                path.parent.mkdir()
                path.write_bytes(b'fixture')
                inputs.append(str(path))
            compiler = root / 'compiler.exe'
            compiler.write_bytes(b'version 1')
            output = root / 'report'
            argv = ['roundtrip.py', '--compiler', str(compiler), '--out', str(output), *inputs]
            workdirs = []

            def decompile(source, work):
                workdirs.append(work)
                if source == inputs[0]:
                    raise ValueError('simulated decode failure')
                return 'fixture.qc'

            with mock.patch.object(sys, 'argv', argv), \
                 mock.patch.object(roundtrip.dc, 'decompile', side_effect=decompile), \
                 mock.patch.object(roundtrip.subprocess, 'run', return_value=mock.Mock(returncode=0)), \
                 mock.patch.object(roundtrip, 'summary', return_value={}), \
                 mock.patch.object(roundtrip, 'compare', return_value=[]), \
                 contextlib.redirect_stdout(io.StringIO()):
                roundtrip.main()
            report = json.loads((output / 'results.json').read_text())
            self.assertEqual(report[inputs[0]]['status'], 'exception')
            self.assertEqual(report[inputs[1]]['status'], 'ok')
            self.assertEqual(len(set(workdirs)), 2)
            self.assertFalse((output / 'results.json.tmp').exists())

            argv.insert(1, '--resume')
            with mock.patch.object(sys, 'argv', argv), \
                 mock.patch.object(roundtrip.dc, 'decompile') as decode, \
                 contextlib.redirect_stdout(io.StringIO()):
                roundtrip.main()
                decode.assert_not_called()

            compiler.write_bytes(b'version 2')
            with mock.patch.object(sys, 'argv', argv), \
                 contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
                roundtrip.main()
            self.assertEqual(error.exception.code, 2)

    def test_interruption_keeps_completed_results_and_clears_old_run(self):
        scratch = Path(__file__).resolve().parents[1] / 'test-output'
        scratch.mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=scratch) as temporary:
            root = Path(temporary)
            compiler = root / 'compiler.exe'
            compiler.write_bytes(b'fixture compiler')
            inputs = [root / 'one.mdl', root / 'two.mdl']
            for path in inputs:
                path.write_bytes(b'fixture input')
            output = root / 'report'
            output.mkdir()
            (output / 'results.json').write_text('{"stale": {"status": "ok"}}')
            argv = ['roundtrip.py', '--compiler', str(compiler), '--out', str(output), *map(str, inputs)]
            with mock.patch.object(sys, 'argv', argv), \
                 mock.patch.object(roundtrip.dc, 'decompile', side_effect=['fixture.qc', KeyboardInterrupt]), \
                 mock.patch.object(roundtrip.subprocess, 'run', return_value=mock.Mock(returncode=0)), \
                 mock.patch.object(roundtrip, 'summary', return_value={}), \
                 mock.patch.object(roundtrip, 'compare', return_value=[]), \
                 contextlib.redirect_stdout(io.StringIO()), self.assertRaises(KeyboardInterrupt):
                roundtrip.main()
            report = json.loads((output / 'results.json').read_text())
            self.assertEqual(list(report), [str(inputs[0])])
            self.assertEqual(report[str(inputs[0])]['status'], 'ok')


if __name__ == '__main__':
    unittest.main()
