"""Round-trip test: decompile retail v53 models, recompile with r2studiomdl, compare.

usage: python tools/roundtrip.py [--compiler build/r2studiomdl.exe] [--out DIR] [--limit N] model.mdl|dir ...
Prints one line per model and a summary; writes results.json in the output dir.
"""
import argparse
import json
import math
import os
import re
import struct
import subprocess
import sys
import traceback

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import decompile53 as dc  # noqa: E402
import mdl53  # noqa: E402
import phy as phylib  # noqa: E402


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
    d = open(path, 'rb').read()
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
        a, name, frames = dc.decode_anim(d, m, bones, ai)
        sample = sorted({0, len(frames) // 2, len(frames) - 1})
        anims.append({'frames': len(frames), 'delta': bool(a['flags'] & 4),
                      'world': {f: world_positions(bones, frames[f]) for f in sample}})
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


def main():
    ap = argparse.ArgumentParser()
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ap.add_argument('--compiler', default=os.path.join(root, 'build', 'r2studiomdl.exe'))
    ap.add_argument('--out', default=os.path.join(root, 'test-output', 'roundtrip'))
    ap.add_argument('--limit', type=int, default=0)
    ap.add_argument('inputs', nargs='+')
    a = ap.parse_args()
    files = []
    for i in a.inputs:
        if os.path.isdir(i):
            for dp, dn, fn in os.walk(i):
                files += [os.path.join(dp, f) for f in fn if f.endswith('.mdl')]
        else:
            files.append(i)
    files.sort()
    if a.limit:
        files = files[:a.limit]
    results = {}
    ok = 0
    for f in files:
        name = os.path.splitext(os.path.basename(f))[0]
        work = os.path.join(a.out, name)
        try:
            qc = dc.decompile(f, work)
            outmdl = os.path.join(work, 'recompiled.mdl')
            r = subprocess.run([a.compiler, '-quiet', '-o', outmdl, qc], capture_output=True, text=True, timeout=600)
            if r.returncode != 0:
                msg = (r.stdout + r.stderr).strip().splitlines()
                results[f] = {'status': 'compile-failed', 'error': msg[-1] if msg else ''}
                print('FAIL compile  %-50s %s' % (name, results[f]['error'][:120]))
                continue
            issues = compare(summary(f), summary(outmdl))
            results[f] = {'status': 'ok' if not issues else 'diff', 'issues': issues}
            if issues:
                print('DIFF          %-50s %s' % (name, '; '.join(issues)[:160]))
            else:
                ok += 1
                print('OK            %s' % name)
        except Exception as e:
            results[f] = {'status': 'exception', 'error': repr(e), 'trace': traceback.format_exc()}
            print('EXCEPTION     %-50s %r' % (name, e))
    print('\n%d/%d identical in the compared fields' % (ok, len(files)))
    os.makedirs(a.out, exist_ok=True)
    json.dump(results, open(os.path.join(a.out, 'results.json'), 'w'), indent=1)


if __name__ == '__main__':
    main()
