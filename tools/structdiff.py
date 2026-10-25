"""Field-by-field comparison of rebuilt v53 models against retail.

roundtrip.py compares a chosen summary; this tool parses every structure the reader
knows (header, bones, attachments, hitboxes, animation and sequence descs and their
events, layers, locks, pose keys and weight lists, body parts, models, meshes,
textures, skins, IK chains, pose parameters, include models, key values, linear bone
table, VTX tree, VVD/VVC/PHY/per-tri headers) and compares every field except offsets,
sizes and checksums, which move whenever anything else changes size.

usage: python tools/structdiff.py --report results.json [--out structdiff.json]
       (results.json from roundtrip.py; each model's work folder must hold recompiled.mdl)
"""
import argparse
import collections
import json
import math
import os
import re
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mdl53  # noqa: E402
from mdl53 import cstr, size, u  # noqa: E402

# offsets, sizes and checksums legitimately differ; everything else is compared
SKIP = re.compile(r'(index|Index|idx$|offset|Offset|ptr|Start$|^length$|[sS]ize$|checksum|checkSum|^_off$|lookup|Lookup)')
ACTMOD_FIELDS = [('sznameindex', 'i'), ('negate', 'i')]
INCLUDE_FIELDS = [('sznameindex', 'i'), ('sznameindex2', 'i')]
FIXUP_FIELDS = [('lod', 'i'), ('sourceVertexID', 'i'), ('numVertexes', 'i')]


def put(out, prefix, rec):
    for k, v in rec.items():
        if SKIP.search(k):
            continue
        if isinstance(v, (list, tuple)):
            for i, x in enumerate(v):
                out['%s.%s[%d]' % (prefix, k, i)] = x
        elif isinstance(v, bytes):
            out['%s.%s' % (prefix, k)] = v.split(b'\0')[0].decode('latin-1')
        else:
            out['%s.%s' % (prefix, k)] = v


def dump(d):
    """Flat {path: value} of everything comparable in a v53 model."""
    m = mdl53.Model(d)
    h = m.hdr
    out = {}
    put(out, 'hdr', h)
    out['hdr.surfaceprop'] = cstr(d, h['surfacepropindex'])
    if h['keyvaluesize']:
        out['hdr.keyvalues'] = d[h['keyvalueindex']:h['keyvalueindex'] + h['keyvaluesize']].split(b'\0')[0].decode('latin-1')
    bsz = size(mdl53.BONE_FIELDS)
    for i in range(h['numbones']):
        o = h['boneindex'] + i * bsz
        b = u(mdl53.BONE_FIELDS, d, o)
        p = 'bones[%d]' % i
        put(out, p, b)
        out[p + '.name'] = cstr(d, o + b['sznameindex'])
        out[p + '.surfaceprop'] = cstr(d, o + b['surfacepropidx'])
    asz = size(mdl53.ATTACH_FIELDS)
    for i in range(h['numlocalattachments']):
        o = h['localattachmentindex'] + i * asz
        a = u(mdl53.ATTACH_FIELDS, d, o)
        put(out, 'attachments[%d]' % i, a)
        out['attachments[%d].name' % i] = cstr(d, o + a['sznameindex'])
    for i in range(h['numhitboxsets']):
        o = h['hitboxsetindex'] + i * size(mdl53.HBOXSET_FIELDS)
        s = u(mdl53.HBOXSET_FIELDS, d, o)
        p = 'hitboxsets[%d]' % i
        put(out, p, s)
        out[p + '.name'] = cstr(d, o + s['sznameindex'])
        for k in range(s['numhitboxes']):
            bo = o + s['hitboxindex'] + k * size(mdl53.BBOX_FIELDS)
            bb = u(mdl53.BBOX_FIELDS, d, bo)
            put(out, '%s.boxes[%d]' % (p, k), bb)
            if bb['szhitboxnameindex']:
                out['%s.boxes[%d].name' % (p, k)] = cstr(d, bo + bb['szhitboxnameindex'])
    for i in range(h['numlocalanim']):
        o = h['localanimindex'] + i * 0x5C
        a = u(mdl53.ANIMDESC_FIELDS, d, o)
        p = 'anims[%d]' % i
        put(out, p, a)
        out[p + '.name'] = cstr(d, o + a['sznameindex'])
    for i in range(h['numlocalseq']):
        o = h['localseqindex'] + i * 0xE8
        s = u(mdl53.SEQDESC_FIELDS, d, o)
        p = 'seqs[%d]' % i
        put(out, p, s)
        out[p + '.label'] = cstr(d, o + s['szlabelindex'])
        out[p + '.activity'] = cstr(d, o + s['szactivitynameindex'])
        for k in range(s['numevents']):
            eo = o + s['eventindex'] + k * size(mdl53.EVENT_FIELDS)
            ev = u(mdl53.EVENT_FIELDS, d, eo)
            put(out, '%s.events[%d]' % (p, k), ev)
            out['%s.events[%d].name' % (p, k)] = cstr(d, eo + ev['szeventindex'])
        for k in range(s['numautolayers']):
            put(out, '%s.autolayers[%d]' % (p, k),
                u(mdl53.AUTOLAYER_FIELDS, d, o + s['autolayerindex'] + k * size(mdl53.AUTOLAYER_FIELDS)))
        for k in range(s['numiklocks']):
            put(out, '%s.iklocks[%d]' % (p, k),
                u(mdl53.IKLOCK_FIELDS, d, o + s['iklockindex'] + k * size(mdl53.IKLOCK_FIELDS)))
        if s['posekeyindex']:
            n = s['groupsize'][0] + s['groupsize'][1]
            for k, v in enumerate(struct.unpack_from('<%df' % n, d, o + s['posekeyindex'])):
                out['%s.posekeys[%d]' % (p, k)] = v
        if s['weightlistindex']:
            for k, v in enumerate(struct.unpack_from('<%df' % h['numbones'], d, o + s['weightlistindex'])):
                out['%s.weights[%d]' % (p, k)] = v
        for k in range(s['numactivitymodifiers']):
            ao = o + s['activitymodifierindex'] + k * size(ACTMOD_FIELDS)
            am = u(ACTMOD_FIELDS, d, ao)
            put(out, '%s.actmods[%d]' % (p, k), am)
            out['%s.actmods[%d].name' % (p, k)] = cstr(d, ao + am['sznameindex'])
        if s['keyvaluesize']:
            out[p + '.keyvalues'] = cstr(d, o + s['keyvalueindex'])
    tsz = size(mdl53.TEXTURE_FIELDS)
    for i in range(h['numtextures']):
        o = h['textureindex'] + i * tsz
        t = u(mdl53.TEXTURE_FIELDS, d, o)
        put(out, 'textures[%d]' % i, t)
        out['textures[%d].name' % i] = cstr(d, o + t['sznameindex'])
    for i in range(h['numcdtextures']):
        out['cdtextures[%d]' % i] = cstr(d, struct.unpack_from('<i', d, h['cdtextureindex'] + 4 * i)[0])
    n = h['numskinref'] * h['numskinfamilies']
    for k, v in enumerate(struct.unpack_from('<%dh' % n, d, h['skinindex'])):
        out['skins[%d]' % k] = v
    bpsz, msz, esz = size(mdl53.BODYPART_FIELDS), size(mdl53.MODEL_FIELDS), size(mdl53.MESH_FIELDS)
    for i in range(h['numbodyparts']):
        bo = h['bodypartindex'] + i * bpsz
        bp = u(mdl53.BODYPART_FIELDS, d, bo)
        p = 'bodyparts[%d]' % i
        put(out, p, bp)
        out[p + '.name'] = cstr(d, bo + bp['sznameindex'])
        for j in range(bp['nummodels']):
            mo = bo + bp['modelindex'] + j * msz
            md = u(mdl53.MODEL_FIELDS, d, mo)
            q = '%s.models[%d]' % (p, j)
            put(out, q, md)
            for k in range(md['nummeshes']):
                put(out, '%s.meshes[%d]' % (q, k), u(mdl53.MESH_FIELDS, d, mo + md['meshindex'] + k * esz))
    for i in range(h['numikchains']):
        o = h['ikchainindex'] + i * size(mdl53.IKCHAIN_FIELDS)
        c = u(mdl53.IKCHAIN_FIELDS, d, o)
        p = 'ikchains[%d]' % i
        put(out, p, c)
        out[p + '.name'] = cstr(d, o + c['sznameindex'])
        for k in range(c['numlinks']):
            put(out, '%s.links[%d]' % (p, k), u(mdl53.IKLINK_FIELDS, d, o + c['linkindex'] + k * size(mdl53.IKLINK_FIELDS)))
    for i in range(h['numlocalposeparameters']):
        o = h['localposeparamindex'] + i * size(mdl53.POSEPARAM_FIELDS)
        pp = u(mdl53.POSEPARAM_FIELDS, d, o)
        put(out, 'poseparams[%d]' % i, pp)
        out['poseparams[%d].name' % i] = cstr(d, o + pp['sznameindex'])
    for i in range(h['numincludemodels']):
        o = h['includemodelindex'] + i * size(INCLUDE_FIELDS)
        im = u(INCLUDE_FIELDS, d, o)
        out['includemodels[%d].label' % i] = cstr(d, o + im['sznameindex'])
        out['includemodels[%d].name' % i] = cstr(d, o + im['sznameindex2'])
    for i in range(h['numsrcbonetransform']):
        o = h['srcbonetransformindex'] + i * size(mdl53.SRCBONE_FIELDS)
        sb = u(mdl53.SRCBONE_FIELDS, d, o)
        put(out, 'srcbonetransforms[%d]' % i, sb)
        out['srcbonetransforms[%d].name' % i] = cstr(d, o + sb['sznameindex'])
    if h['linearboneindex']:
        o = h['linearboneindex']
        lb = u(mdl53.LINEARBONE_FIELDS, d, o)
        put(out, 'linearbone', lb)
        nb = lb['numbones']
        for name, fmt, n in (('flags', 'i', 1), ('parent', 'i', 1), ('pos', 'f', 3), ('quat', 'f', 4), ('rot', 'f', 3),
                             ('posetobone', 'f', 12), ('posscale', 'f', 3), ('rotscale', 'f', 3), ('qalignment', 'f', 4)):
            idx = lb.get(name + 'index') or lb.get({'posetobone': 'posetoboneindex'}.get(name, ''), 0)
            if not idx:
                continue
            vals = struct.unpack_from('<%d%s' % (nb * n, fmt), d, o + idx)
            for k, v in enumerate(vals):
                out['linearbone.%s[%d]' % (name, k)] = v
    if h['vtxSize']:
        def walk(prefix, node):
            for k, v in node.items():
                if isinstance(v, list) and v and isinstance(v[0], dict):
                    for i, x in enumerate(v):
                        walk('%s.%s[%d]' % (prefix, k, i), x)
                elif isinstance(v, dict):
                    walk('%s.%s' % (prefix, k), v)
                elif not SKIP.search(k):
                    if isinstance(v, (list, tuple)):
                        for i, x in enumerate(v):
                            out['%s.%s[%d]' % (prefix, k, i)] = x
                    else:
                        out['%s.%s' % (prefix, k)] = v
        walk('vtx', mdl53.parse_vtx(d, h['vtxOffset']))
    if h['vvdSize']:
        vv = u(mdl53.VVD_HDR, d, h['vvdOffset'])
        put(out, 'vvd', vv)
        for k in range(vv['numFixups']):
            put(out, 'vvd.fixups[%d]' % k,
                u(FIXUP_FIELDS, d, h['vvdOffset'] + vv['fixupTableStart'] + k * size(FIXUP_FIELDS)))
    if h['vvcSize']:
        put(out, 'vvc', u(mdl53.VVC_HDR, d, h['vvcOffset']))
    if h['phySize']:
        put(out, 'phy', u(mdl53.PHY_HDR, d, h['phyOffset']))
    if h['perTriAABBIndex']:
        put(out, 'pertri', u(mdl53.PERTRI_HDR, d, h['perTriAABBIndex']))
    return out


def same(a, b):
    if isinstance(a, float) and isinstance(b, float):
        if math.isnan(a) and math.isnan(b):
            return True
        return abs(a - b) <= 1e-3 + 1e-4 * max(abs(a), abs(b))
    return a == b


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--report', required=True, help='results.json from roundtrip.py')
    ap.add_argument('--out', help='write the full comparison as JSON')
    ap.add_argument('--examples', type=int, default=3)
    a = ap.parse_args()
    r = json.load(open(a.report))
    r = r.get('results', r)
    work = os.path.dirname(os.path.abspath(a.report))
    fields = collections.defaultdict(lambda: {'models': 0, 'examples': []})
    compared = 0
    for src in sorted(r):
        name = os.path.splitext(os.path.basename(src))[0]
        rebuilt = os.path.join(work, name, 'recompiled.mdl')
        if not os.path.exists(rebuilt) or not os.path.exists(src):
            continue
        try:
            x, y = dump(open(src, 'rb').read()), dump(open(rebuilt, 'rb').read())
        except Exception as e:  # noqa: BLE001
            fields['<parse error>']['models'] += 1
            fields['<parse error>']['examples'].append((name, str(e)))
            continue
        compared += 1
        seen = set()
        for k in set(x) | set(y):
            if same(x.get(k, '<missing>'), y.get(k, '<missing>')):
                continue
            key = re.sub(r'\[\d+\]', '[*]', k)
            if key not in seen:
                seen.add(key)
                fields[key]['models'] += 1
                if len(fields[key]['examples']) < a.examples:
                    fields[key]['examples'].append((name, k, repr(x.get(k, '<missing>'))[:60], repr(y.get(k, '<missing>'))[:60]))
    print('compared %d models; %d field paths differ somewhere' % (compared, len(fields)))
    for k, v in sorted(fields.items(), key=lambda kv: -kv[1]['models']):
        print('%5d  %s' % (v['models'], k))
        for e in v['examples']:
            print('         ', e)
    if a.out:
        json.dump({'compared': compared, 'fields': fields}, open(a.out, 'w'), indent=1)


if __name__ == '__main__':
    main()
