"""Round-trip test: decompile retail v53 models, recompile with r2studiomdl, compare.

usage: python tools/roundtrip.py [--compiler build/r2studiomdl.exe] [--out DIR] [--limit N] model.mdl|dir ...
Prints one line per model and a summary; writes results.json in the output dir.
"""
import argparse
import collections
import concurrent.futures
import hashlib
import json
import math
import os
from pathlib import Path
import re
import struct
import subprocess
import sys
import traceback

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import decompile53 as dc  # noqa: E402
import mdl53  # noqa: E402
import phy as phylib  # noqa: E402


def checkpoint(path, value):
    """Replace the report atomically so an interrupted run keeps its last result."""
    temporary = path + '.tmp'
    with open(temporary, 'w') as report:
        json.dump(value, report, indent=1)
    os.replace(temporary, path)


def run_signature(compiler):
    paths = [compiler, __file__, dc.__file__, mdl53.__file__, phylib.__file__]
    return {os.path.basename(p): hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in paths}


def world_positions(bones, frame):
    out = []
    mats = []
    for i, (pos, rm) in enumerate(frame):
        m = [row[:] for row in rm]
        for k in range(3):
            m[k][3] = pos[k]
        if bones[i]['parent'] >= 0:
            m = dc.concat(mats[bones[i]['parent']], m)
        mats.append(m)
        out.append((m[0][3], m[1][3], m[2][3]))
    return out


def summary(path):
    with open(path, 'rb') as source:
        d = source.read()
    m = mdl53.Model(d)
    h = m.hdr
    s = {'flags': h['flags'], 'bones': [(b['name'], b['parent']) for b in m.bones()]}
    bones = m.bones()
    s['bind'] = [tuple(b['pos']) + tuple(b['rot']) for b in bones]
    tex = []
    for i in range(h['numtextures']):
        o = h['textureindex'] + i * 0x2C
        tex.append(mdl53.cstr(d, o + struct.unpack_from('<i', d, o)[0]).lower().replace('/', '\\'))
    s['textures'] = sorted(tex)
    tris = []
    if h['vtxSize']:
        t = mdl53.parse_vtx(d, h['vtxOffset'])
        for bp in t['bodyparts']:
            for md in bp['models']:
                for li, ld in enumerate(md['lods']):
                    n = sum(sg['numIndices'] // 3 for me in ld['meshes'] for sg in me['stripgroups'])
                    tris.append(n)
    s['tris'] = tris
    seqs = []
    for i in range(h['numlocalseq']):
        o = h['localseqindex'] + i * 0xE8
        q = mdl53.u(mdl53.SEQDESC_FIELDS, d, o)
        seqs.append((mdl53.cstr(d, o + q['szlabelindex']), q['numevents'], q['groupsize'][0], q['groupsize'][1]))
    s['seqs'] = seqs
    anims = []
    for ai in range(h['numlocalanim']):
        a = mdl53.u(mdl53.ANIMDESC_FIELDS, d, h['localanimindex'] + ai * 0x5C)
        nf = a['numframes']
        sample = sorted({0, nf // 2, nf - 1}) if nf > 0 else []
        a, name, frames = dc.decode_anim(d, m, bones, ai, sample)
        anims.append({'frames': nf, 'delta': bool(a['flags'] & 4),
                      'world': {f: world_positions(bones, pose) for f, pose in zip(sample, frames)}})
    s['anims'] = anims
    if h['phySize']:
        p = phylib.parse_phy(d[h['phyOffset']:h['phyOffset'] + h['phySize']])
        vols = [float(v) for v in re.findall(r'"volume"\s+"([^"]+)"', p['text'])]
        s['phy'] = {'solids': p['solidCount'], 'volume': sum(vols)}
    s['attach'] = h['numlocalattachments']
    s['hitboxsets'] = h['numhitboxsets']
    s['coll'] = h['staticCollisionCount']
    s['pertri'] = h['perTriAABBLeafCount']
    return s


def compare(a, b):
    issues = []
    if a['bones'] != b['bones']:
        issues.append('bones differ (%d vs %d)' % (len(a['bones']), len(b['bones'])))
    else:
        worst = 0
        for x, y in zip(a['bind'], b['bind']):
            worst = max(worst, max(abs(p - q) for p, q in zip(x[:3], y[:3])))
        if worst > 0.01:
            issues.append('bind pose off by %.3f' % worst)
    if a['textures'] != b['textures']:
        issues.append('textures differ')
    if a['tris'] != b['tris']:
        issues.append('triangle counts %s vs %s' % (a['tris'][:6], b['tris'][:6]))
    if [x[0] for x in a['seqs']] != [x[0] for x in b['seqs']]:
        issues.append('sequence names differ')
    if len(a['anims']) != len(b['anims']):
        issues.append('anim count %d vs %d' % (len(a['anims']), len(b['anims'])))
    elif a['bones'] == b['bones']:
        worst = 0.0
        for x, y in zip(a['anims'], b['anims']):
            if x['frames'] != y['frames']:
                issues.append('anim frames %d vs %d' % (x['frames'], y['frames']))
                break
            if x['delta']:
                continue
            for f in x['world']:
                for p, q in zip(x['world'][f], y['world'][f]):
                    worst = max(worst, math.dist(p, q))
        if worst > 0.25:
            issues.append('anim world positions off by %.2f' % worst)
    if 'phy' in a:
        if 'phy' not in b:
            issues.append('phy missing')
        else:
            if a['phy']['solids'] != b['phy']['solids']:
                issues.append('phy solids %d vs %d' % (a['phy']['solids'], b['phy']['solids']))
            va, vb = a['phy']['volume'], b['phy']['volume']
            if va > 0 and abs(va - vb) / va > 0.02:
                issues.append('phy volume %.1f vs %.1f' % (va, vb))
    if a['attach'] != b['attach']:
        issues.append('attachments %d vs %d' % (a['attach'], b['attach']))
    if bool(a['coll']) != bool(b['coll']):
        issues.append('collision block %d vs %d' % (a['coll'], b['coll']))
    return issues


def process_model(job):
    compiler, source, work, stamp = job
    try:
        qc = dc.decompile(source, work)
        outmdl = os.path.join(work, 'recompiled.mdl')
        run = subprocess.run([compiler, '-quiet', '-o', outmdl, qc], capture_output=True, text=True, timeout=600)
        if run.returncode:
            lines = (run.stdout + run.stderr).strip().splitlines()
            result = {'status': 'compile-failed', 'error': lines[-1] if lines else ''}
        else:
            issues = compare(summary(source), summary(outmdl))
            result = {'status': 'diff' if issues else 'ok', 'issues': issues}
    except Exception as error:
        result = {'status': 'exception', 'error': repr(error), 'trace': traceback.format_exc()}
    result['input_stamp'] = stamp
    return source, result


def main():
    ap = argparse.ArgumentParser()
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ap.add_argument('--compiler', default=os.path.join(root, 'build', 'r2studiomdl.exe'))
    ap.add_argument('--out', default=os.path.join(root, 'test-output', 'roundtrip'))
    ap.add_argument('--limit', type=int, default=0)
    ap.add_argument('--jobs', type=int, default=1, help='number of independent model workers')
    ap.add_argument('--resume', action='store_true', help='skip saved results with unchanged inputs and tools')
    ap.add_argument('inputs', nargs='+')
    a = ap.parse_args()
    if a.jobs < 1:
        ap.error('--jobs must be at least 1')
    files = []
    for i in a.inputs:
        if os.path.isdir(i):
            for dp, dn, fn in os.walk(i):
                files += [os.path.join(dp, f) for f in fn if f.endswith('.mdl')]
        else:
            files.append(i)
    files = sorted(set(os.path.abspath(f) for f in files))
    if a.limit:
        files = files[:a.limit]
    os.makedirs(a.out, exist_ok=True)
    report = os.path.join(a.out, 'results.json')
    manifest = os.path.join(a.out, 'run.json')
    signature = run_signature(a.compiler)
    results = {}
    if a.resume and os.path.exists(report):
        if not os.path.exists(manifest) or json.loads(Path(manifest).read_text()) != signature:
            ap.error('cannot resume: compiler or test tools changed; use a new --out directory or omit --resume')
        results = json.loads(Path(report).read_text())
    # Clear an old report before installing a new signature, including when the
    # first model is interrupted before producing a result.
    checkpoint(report, results)
    checkpoint(manifest, signature)
    names = collections.Counter(os.path.splitext(os.path.basename(f))[0] for f in files)
    reused = 0
    jobs = []
    for f in files:
        stamp = [os.stat(f).st_size, os.stat(f).st_mtime_ns]
        if a.resume and f in results and results[f].get('input_stamp') == stamp:
            reused += 1
            continue
        results.pop(f, None)
        name = os.path.splitext(os.path.basename(f))[0]
        # Different model folders can contain the same basename.
        suffix = '-' + hashlib.sha256(f.encode()).hexdigest()[:12] if names[name] > 1 else ''
        work = os.path.join(a.out, name + suffix)
        jobs.append((a.compiler, f, work, stamp))

    def record(completed):
        for source, result in completed:
            results[source] = result
            checkpoint(report, results)
            detail = '; '.join(result.get('issues', [])) or result.get('error', '')
            print('%-14s%-50s %s' % (result['status'].upper(), Path(source).stem, detail[:160]), flush=True)

    if a.jobs == 1:
        record(map(process_model, jobs))
    else:
        with concurrent.futures.ProcessPoolExecutor(max_workers=a.jobs) as pool:
            pending = [pool.submit(process_model, job) for job in jobs]
            record(future.result() for future in concurrent.futures.as_completed(pending))
    ok = sum(results[f]['status'] == 'ok' for f in files)
    print('\n%d/%d identical in the compared fields (%d resumed)' % (ok, len(files), reused))


if __name__ == '__main__':
    main()
