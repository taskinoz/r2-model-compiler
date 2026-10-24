"""Recompile saved QC/SMD exports with a new compiler, without decoding them again.

The input model stamps and decompiler/comparison tool hashes must still match the
original round-trip report. Do not edit the cached exports before running this.
"""
import argparse
import collections
import concurrent.futures
import hashlib
import json
import os
from pathlib import Path
import subprocess
import traceback

import roundtrip as rt


def recheck(job):
    compiler, source, qc, output, stamp = job
    try:
        if not Path(qc).is_file():
            raise FileNotFoundError('cached QC missing: ' + qc)
        run = subprocess.run([compiler, '-quiet', '-o', output, qc], capture_output=True, text=True, timeout=600)
        if run.returncode:
            result = {'status': 'compile-failed', 'error': (run.stdout + run.stderr).strip()}
        else:
            issues = rt.compare(rt.summary(source), rt.summary(output))
            result = {'status': 'diff' if issues else 'ok', 'issues': issues}
    except Exception as error:
        result = {'status': 'exception', 'error': repr(error), 'trace': traceback.format_exc()}
    result.update(input_stamp=stamp, cached_qc=qc)
    return source, result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--from-report', required=True)
    parser.add_argument('--compiler', required=True)
    parser.add_argument('--out', required=True)
    parser.add_argument('--jobs', type=int, default=1)
    args = parser.parse_args()
    if args.jobs < 1:
        parser.error('--jobs must be at least one')
    cache = Path(args.from_report).resolve()
    previous = json.loads(cache.read_text())
    signature = rt.run_signature(args.compiler)
    old_signature = json.loads((cache.parent/'run.json').read_text())
    for name, digest in signature.items():
        if name.endswith('.py') and old_signature.get(name) != digest:
            parser.error('cached exports were generated with different tools: ' + name)
    output = Path(args.out).resolve()
    output.mkdir(parents=True, exist_ok=True)
    jobs = []
    for source, result in previous.items():
        stat = os.stat(source)
        stamp = [stat.st_size, stat.st_mtime_ns]
        if stamp != result.get('input_stamp'):
            parser.error('input changed since export: ' + source)
        stem = Path(source).stem
        hashed = stem + '-' + hashlib.sha256(source.encode()).hexdigest()[:12]
        work = cache.parent/hashed if (cache.parent/hashed).is_dir() else cache.parent/stem
        destination = output/work.name
        destination.mkdir(exist_ok=True)
        jobs.append((str(Path(args.compiler).resolve()), source, str(work/(stem+'.qc')),
                     str(destination/'recompiled.mdl'), stamp))
    rt.checkpoint(str(output/'run.json'), signature)
    rt.checkpoint(str(output/'cache.json'), {'report': str(cache), 'note': 'Existing QC/SMD exports reused without modification.'})
    results = {}
    with concurrent.futures.ProcessPoolExecutor(max_workers=args.jobs) as pool:
        for completed in concurrent.futures.as_completed([pool.submit(recheck, job) for job in jobs]):
            source, result = completed.result()
            results[source] = result
            rt.checkpoint(str(output/'results.json'), results)
            detail = '; '.join(result.get('issues', [])) or result.get('error', '')
            print('%-14s%-50s %s' % (result['status'].upper(), Path(source).stem, detail[:160]), flush=True)
    counts = collections.Counter(row['status'] for row in results.values())
    print('\n%d/%d match; %s' % (counts['ok'], len(results), dict(counts)))


if __name__ == '__main__':
    main()
