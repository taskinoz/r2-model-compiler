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
import time
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
    # Windows can briefly deny the replace while another process (antivirus, indexer)
    # holds the report open; retry rather than abort a long run.
    for attempt in range(20):
        try:
            os.replace(temporary, path)
            return
        except PermissionError:
            if attempt == 19:
                raise
            time.sleep(0.25)


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
        keys = ()
        if q['posekeyindex'] and (q['groupsize'][0] > 1 or q['groupsize'][1] > 1):
            n = q['groupsize'][0] + q['groupsize'][1]
            keys = tuple(round(k, 2) for k in struct.unpack_from('<%df' % n, d, o + q['posekeyindex']))
        weights = ()
        if q['weightlistindex']:
            weights = tuple(round(w, 3) for w in struct.unpack_from('<%df' % len(bones), d, o + q['weightlistindex']))
        seqs.append((mdl53.cstr(d, o + q['szlabelindex']), q['numevents'], q['groupsize'][0], q['groupsize'][1],
                     q['flags'], keys, weights, q['numikrules']))
    s['seqs'] = seqs
    anims = []
    for ai in range(h['numlocalanim']):
        a = mdl53.u(mdl53.ANIMDESC_FIELDS, d, h['localanimindex'] + ai * 0x5C)
        nf = a['numframes']
        sample = sorted({0, nf // 2, nf - 1}) if nf > 0 else []
        a, name, frames = dc.decode_anim(d, m, bones, ai, sample, with_scale=True)
        scales = {f: [tuple(e[2]) for e in pose] for f, pose in zip(sample, frames)}
        frames = [[(e[0], e[1]) for e in pose] for pose in frames]
        fms = dc.decode_frame_movement(d, m, ai, sample)
        if fms is not None and not (a['flags'] & 4):
            frames = [dc.apply_root_motion(bones, fr, fm) for fr, fm in zip(frames, fms)]
        delta = bool(a['flags'] & 4)
        # Delta animations are compared as raw local values (they are added onto a pose):
        # the translation plus the rotation's basis vectors at a 50-unit lever.
        def local_delta(pose):
            return [tuple(p) + tuple(50 * rm[r][c] for r in range(3) for c in range(3)) for p, rm in pose]
        ik = []
        for r in dc.decode_ik_rules(d, h, ai):
            rng = (r['start'], r['peak'], r['tail'], r['end'], r['contact'])
            errs = []
            if r['compressedikerrorindex'] and nf > 1:
                first = r['iStart']
                last = first + int(round((r['end'] - r['start']) * (nf - 1)))
                for f in sorted({first, (first + last) // 2, last}):
                    e = dc.decode_ik_error(d, r, f)
                    errs.append(e[0] + tuple(50 * v for v in e[1]))  # angles at a 50-unit lever
            ik.append({'key': (r['index'], r['type'], r['chain'], r['bone'], r['slot'], r['attachment'],
                               tuple(round(v, 3) for v in rng), round(r['height'], 2), round(r['radius'], 2),
                               round(r['floor'], 2), round(r['endHeight'], 2)),
                       'pos': tuple(r['pos']), 'errors': errs})
        anims.append({'frames': nf, 'delta': delta, 'flags': a['flags'], 'scale': scales, 'ik': ik,
                      'world': {f: (local_delta(pose) if delta else world_positions(bones, pose))
                                for f, pose in zip(sample, frames)}})
    s['anims'] = anims
    if h['phySize']:
        p = phylib.parse_phy(d[h['phyOffset']:h['phyOffset'] + h['phySize']])
        vols = [float(v) for v in re.findall(r'"volume"\s+"([^"]+)"', p['text'])]
        s['phy'] = {'solids': p['solidCount'], 'volume': sum(vols)}
    s['attach'] = h['numlocalattachments']
    s['hitboxsets'] = h['numhitboxsets']
    s['coll'] = h['staticCollisionCount']
    # map-collision entries: header-counted (rigid) or bone-referenced (jointed)
    entries = []
    if h['collisionOffset'] and h['phySize']:
        n = h['staticCollisionCount'] or sum(b['collisionCount'] for b in bones if b['collisionIndex'] != 0xFFFF)
        for i in range(n):
            entries.append(struct.unpack_from('<4i', d, h['collisionOffset'] + i * 20))
    # Entry order is not meaningful (retail jointed blocks follow neither bone nor PHY
    # order), so entries are grouped per owning bone and sorted within each group.
    groups = {'static': entries} if h['staticCollisionCount'] else {
        i: entries[b['collisionIndex']:b['collisionIndex'] + b['collisionCount']]
        for i, b in enumerate(bones) if b['collisionIndex'] != 0xFFFF and b['collisionCount']}
    s['collEntries'] = {str(k): sorted(v) for k, v in groups.items()}
    # bone references: the index only matters when there are no pieces (0 vs 0xFFFF)
    s['boneColl'] = [(b['collisionIndex'] if not b['collisionCount'] else 'ref', b['collisionCount']) for b in bones]
    s['pertri'] = h['perTriAABBLeafCount']
    # procedural bones: type per bone, and jiggle parameters
    def flat(v):
        if isinstance(v, dict):
            return tuple(flat(x) for x in v.values())
        if isinstance(v, (list, tuple)):
            return tuple(flat(x) for x in v)
        return round(v, 3) if isinstance(v, float) else v
    s['proc'] = [(b['name'], b['proctype'], b['jiggle']['flags'] if b['proctype'] == 5 and b.get('jiggle') else 0,
                  tuple(round(v, 3) for k, v in b['jiggle'].items() if k != 'flags') if b['proctype'] == 5 and b.get('jiggle')
                  else flat(dc.decode_procedural(d, h, i, b)))
                 for i, b in enumerate(bones) if b['proctype']]
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
    elif [x[4:] for x in a['seqs']] != [x[4:] for x in b['seqs']]:
        bad = [x[0] for x, y in zip(a['seqs'], b['seqs']) if x[4:] != y[4:]]
        issues.append('sequence flags/pose keys/weights differ (%d, e.g. %s)' % (len(bad), bad[0]))
    if len(a['anims']) != len(b['anims']):
        issues.append('anim count %d vs %d' % (len(a['anims']), len(b['anims'])))
    elif a['bones'] == b['bones']:
        worst = 0.0
        for x, y in zip(a['anims'], b['anims']):
            if x['frames'] != y['frames']:
                issues.append('anim frames %d vs %d' % (x['frames'], y['frames']))
                break
            if x['delta'] != y['delta']:
                issues.append('delta flag differs on an animation')
                break
            if x.get('flags') != y.get('flags'):
                issues.append('animation flags %s vs %s' % (hex(x.get('flags') or 0), hex(y.get('flags') or 0)))
                break
            for f in x['world']:
                for p, q in zip(x['world'][f], y['world'][f]):
                    worst = max(worst, math.dist(p, q))
        if worst > 0.25:
            issues.append('anim world positions off by %.2f' % worst)
        ikbad, ikworst = None, 0.0
        for x, y in zip(a['anims'], b['anims']):
            xi, yi = x.get('ik', []), y.get('ik', [])
            if [r['key'] for r in xi] != [r['key'] for r in yi]:
                ikbad = ikbad or (len(xi), len(yi), next((p['key'], q['key']) for p, q in zip(xi + [{'key': None}], yi + [{'key': None}]) if p['key'] != q['key']))
                continue
            for p, q in zip(xi, yi):
                ikworst = max([ikworst, math.dist(p['pos'], q['pos'])] +
                              [math.dist(u, v) for u, v in zip(p['errors'], q['errors'])])
        if ikbad:
            issues.append('ik rules differ (%d vs %d rules, e.g. %s vs %s)' % (ikbad[0], ikbad[1], ikbad[2][0], ikbad[2][1]))
        elif ikworst > 0.25:
            issues.append('ik rule targets/errors off by %.2f' % ikworst)
        sworst = 0.0
        for x, y in zip(a['anims'], b['anims']):
            for f in x.get('scale', {}):
                for p, q in zip(x['scale'][f], y.get('scale', {}).get(f, [])):
                    sworst = max(sworst, max(abs(u - v) for u, v in zip(p, q)))
        if sworst > 0.01:
            issues.append('bone scale off by %.3f' % sworst)
    if 'phy' in a:
        if 'phy' not in b:
            issues.append('phy missing')
        else:
            if a['phy']['solids'] != b['phy']['solids']:
                issues.append('phy solids %d vs %d' % (a['phy']['solids'], b['phy']['solids']))
            va, vb = a['phy']['volume'], b['phy']['volume']
            if va > 0 and abs(va - vb) / va > 0.02:
                issues.append('phy volume %.1f vs %.1f' % (va, vb))
    if a.get('proc', []) != b.get('proc', []):
        pa, pb = a.get('proc', []), b.get('proc', [])
        issues.append('procedural bones differ (%d vs %d, e.g. %s)' % (len(pa), len(pb), next(
            (x[0] for x, y in zip(pa + [('',)], pb + [('',)]) if x != y), '')))
    if a['attach'] != b['attach']:
        issues.append('attachments %d vs %d' % (a['attach'], b['attach']))
    if a['coll'] != b['coll']:
        issues.append('collision block count %d vs %d' % (a['coll'], b['coll']))
    ea, eb = a.get('collEntries', {}), b.get('collEntries', {})
    na, nb = sum(map(len, ea.values())), sum(map(len, eb.values()))
    if na != nb or set(ea) != set(eb):
        issues.append('collision entries %d vs %d' % (na, nb))
    elif ea != eb:
        pairs = [(x, y) for k in ea for x, y in zip(ea[k], eb[k]) if x != y]
        issues.append('collision entry counts differ in %d/%d pieces (e.g. %s vs %s)' % (
            len(pairs), na, tuple(pairs[0][0]), tuple(pairs[0][1])))
    if a.get('boneColl') and a['bones'] == b['bones'] and a['boneColl'] != b['boneColl']:
        issues.append('bone collision references differ')
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
