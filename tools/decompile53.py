"""Decompile a Titanfall 2 (v53) .mdl into QC + SMD files that r2studiomdl recompiles.

Conventions match studiomdl so a decompile/recompile round trip is lossless where
the format allows: animation root bones get studiomdl's built-in 90 degree yaw
removed, static prop vertices are rotated back, and the bind pose is written as is.

usage: python tools/decompile53.py model.mdl outdir
"""
import math
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mdl53  # noqa: E402
import phy as phylib  # noqa: E402


# ---------------------------------------------------------------- math
def angle_matrix(r):
    # RadianEuler (roll, pitch, yaw) -> 3x4
    sr, cr = math.sin(r[0]), math.cos(r[0])
    sp, cp = math.sin(r[1]), math.cos(r[1])
    sy, cy = math.sin(r[2]), math.cos(r[2])
    m = [[0.0] * 4 for _ in range(3)]
    m[0][0] = cp * cy
    m[1][0] = cp * sy
    m[2][0] = -sp
    m[0][1] = sp * sr * cy - cr * sy
    m[1][1] = sp * sr * sy + cr * cy
    m[2][1] = sr * cp
    m[0][2] = sp * cr * cy + sr * sy
    m[1][2] = sp * cr * sy - sr * cy
    m[2][2] = cr * cp
    return m


def matrix_angles(m):
    fwd = (m[0][0], m[1][0], m[2][0])
    left = (m[0][1], m[1][1], m[2][1])
    xy = math.sqrt(fwd[0] ** 2 + fwd[1] ** 2)
    if xy > 0.001:
        yaw = math.atan2(fwd[1], fwd[0])
        pitch = math.atan2(-fwd[2], xy)
        roll = math.atan2(left[2], m[2][2])
    else:
        yaw = math.atan2(-left[0], left[1])
        pitch = math.atan2(-fwd[2], xy)
        roll = 0.0
    return (roll, pitch, yaw)


def quat_matrix(q):
    x, y, z, w = q
    m = [[0.0] * 4 for _ in range(3)]
    m[0][0] = 1 - 2 * y * y - 2 * z * z
    m[1][0] = 2 * x * y + 2 * w * z
    m[2][0] = 2 * x * z - 2 * w * y
    m[0][1] = 2 * x * y - 2 * w * z
    m[1][1] = 1 - 2 * x * x - 2 * z * z
    m[2][1] = 2 * y * z + 2 * w * x
    m[0][2] = 2 * x * z + 2 * w * y
    m[1][2] = 2 * y * z - 2 * w * x
    m[2][2] = 1 - 2 * x * x - 2 * y * y
    return m


def concat(a, b):
    o = [[0.0] * 4 for _ in range(3)]
    for i in range(3):
        for j in range(4):
            o[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j]
        o[i][3] += a[i][3]
    return o


def transform(m, v):
    return tuple(m[i][0] * v[0] + m[i][1] * v[1] + m[i][2] * v[2] + m[i][3] for i in range(3))


def rotate(m, v):
    return tuple(m[i][0] * v[0] + m[i][1] * v[1] + m[i][2] * v[2] for i in range(3))


def unpack_q64(v):
    x = ((v & 0x1FFFFF) - 1048576) * (1 / 1048576.5)
    y = (((v >> 21) & 0x1FFFFF) - 1048576) * (1 / 1048576.5)
    z = (((v >> 42) & 0x1FFFFF) - 1048576) * (1 / 1048576.5)
    d = 1 - (x * x + y * y + z * z)
    w = math.sqrt(d) if d > 0 else 0.0
    if v >> 63:
        w = -w
    return (x, y, z, w)


def half(h):
    return struct.unpack('<e', struct.pack('<H', h))[0]


ROT_M90 = angle_matrix((0, 0, -math.pi / 2))  # inverse of studiomdl's default rotation


# ---------------------------------------------------------------- anim decode
def extract(d, p, frame, scale):
    if p is None:
        return 0.0
    k = frame
    while True:
        valid, total = d[p], d[p + 1]
        if total == 0:
            return 0.0
        if total <= k:
            k -= total
            p += (valid + 1) * 2
            continue
        if valid > k:
            return struct.unpack_from('<h', d, p + (k + 1) * 2)[0] * scale
        return struct.unpack_from('<h', d, p + valid * 2)[0] * scale


def decode_anim(d, m, bones, ai):
    """Returns (desc, frames) with frames[f][bone] = (pos, matrix) local transforms."""
    h = m.hdr
    o = h['localanimindex'] + ai * 0x5C
    a = mdl53.u(mdl53.ANIMDESC_FIELDS, d, o)
    name = mdl53.cstr(d, o + a['sznameindex'])
    nf = a['numframes']
    delta = bool(a['flags'] & 4)
    frames = []
    for f in range(nf):
        if a['sectionframes']:
            sf = a['sectionframes']
            if nf > sf and f == nf - 1:
                sec, lf = (nf - 1) // sf + 1, 0
            else:
                sec, lf = f // sf, f - (f // sf) * sf
            start = o + struct.unpack_from('<i', d, o + a['sectionindex'] + sec * 4)[0]
        else:
            start, lf = o + a['animindex'], f
        pose = {}
        r = start
        for _ in range(512):
            posscale, bone, flags = struct.unpack_from('<fBB', d, r)
            if bone == 255:
                break
            b = bones[bone]
            # rotation
            if flags & 0x04:
                q = unpack_q64(struct.unpack_from('<Q', d, r + 8)[0])
                rm = quat_matrix(q)
            elif flags & 0x10:
                rm = angle_matrix((0, 0, 0)) if delta else angle_matrix(b['rot'])
            else:
                offs = struct.unpack_from('<3h', d, r + 8)
                ang = [extract(d, r + 8 + offs[c] if offs[c] > 0 else None, lf, b['rotscale'][c]) for c in range(3)]
                if not delta:
                    ang = [ang[c] + b['rot'][c] for c in range(3)]
                rm = angle_matrix(ang)
            # position
            if flags & 0x02:
                hs = struct.unpack_from('<3H', d, r + 16)
                pos = tuple(half(x) for x in hs)
            else:
                offs = struct.unpack_from('<3h', d, r + 16)
                pos = [extract(d, r + 16 + offs[c] if offs[c] > 0 else None, lf, posscale) for c in range(3)]
                if not delta:
                    pos = [pos[c] + b['pos'][c] for c in range(3)]
                pos = tuple(pos)
            pose[bone] = (pos, rm)
            nxt = struct.unpack_from('<i', d, r + 28)[0]
            if nxt == 0:
                break
            r += nxt
        out = []
        for bi, b in enumerate(bones):
            if bi in pose:
                out.append(pose[bi])
            elif delta:
                out.append(((0.0, 0.0, 0.0), angle_matrix((0, 0, 0))))
            else:
                out.append((tuple(b['pos']), angle_matrix(b['rot'])))
        frames.append(out)
    return a, name, frames


# ---------------------------------------------------------------- writers
def fmt(v):
    s = '%.6f' % v
    return '0.000000' if s == '-0.000000' else s


def write_smd(path, bones, skeleton_frames, tris=None):
    with open(path, 'w', newline='\n') as f:
        f.write('version 1\nnodes\n')
        for i, b in enumerate(bones):
            f.write('%d "%s" %d\n' % (i, b['name'], b['parent']))
        f.write('end\nskeleton\n')
        for t, fr in enumerate(skeleton_frames):
            f.write('time %d\n' % t)
            for i, (pos, rot) in enumerate(fr):
                f.write('%d %s %s %s %s %s %s\n' % (i, fmt(pos[0]), fmt(pos[1]), fmt(pos[2]), fmt(rot[0]), fmt(rot[1]), fmt(rot[2])))
        f.write('end\n')
        if tris is not None:
            f.write('triangles\n')
            for mat, verts in tris:
                f.write(mat + '\n')
                for v in verts:
                    pos, nrm, uv, weights = v
                    line = '0 %s %s %s %s %s %s %s %s %d' % (
                        fmt(pos[0]), fmt(pos[1]), fmt(pos[2]), fmt(nrm[0]), fmt(nrm[1]), fmt(nrm[2]),
                        fmt(uv[0]), fmt(1 - uv[1]), len(weights))
                    for bi, w in weights:
                        line += ' %d %s' % (bi, fmt(w))
                    f.write(line + '\n')
            f.write('end\n')


def decompile(path, outdir):
    d = open(path, 'rb').read()
    m = mdl53.Model(d)
    h = m.hdr
    os.makedirs(outdir, exist_ok=True)
    base = os.path.splitext(os.path.basename(path))[0]
    static = bool(h['flags'] & 0x10)
    bones = m.bones()
    for b in bones:
        b['pos'] = tuple(b['pos'])
        b['rot'] = tuple(b['rot'])

    def local_frame_to_smd(bi, pos, rm, root_fix):
        if root_fix and bones[bi]['parent'] == -1:
            mm = [row[:] for row in rm]
            for i in range(3):
                mm[i][3] = pos[i]
            mm = concat(ROT_M90, mm)
            return (mm[0][3], mm[1][3], mm[2][3]), matrix_angles(mm)
        return pos, matrix_angles(rm)

    bind = [(b['pos'], b['rot']) for b in bones]

    # textures / skins
    tex = []
    for i in range(h['numtextures']):
        o = h['textureindex'] + i * 0x2C
        tex.append(mdl53.cstr(d, o + struct.unpack_from('<i', d, o)[0]))
    cds = [mdl53.cstr(d, struct.unpack_from('<i', d, h['cdtextureindex'] + i * 4)[0]) or '' for i in range(h['numcdtextures'])]
    skins = []
    for fi in range(h['numskinfamilies']):
        skins.append(list(struct.unpack_from('<%dh' % h['numskinref'], d, h['skinindex'] + fi * h['numskinref'] * 2)))

    # vertices
    vvd = mdl53.u(mdl53.VVD_HDR, d, h['vvdOffset']) if h['vvdSize'] else None

    def vvd_vertex(i):
        o = h['vvdOffset'] + vvd['vertexDataStart'] + i * 48
        w = struct.unpack_from('<3f', d, o)
        bn = struct.unpack_from('<3B', d, o + 12)
        nb = d[o + 15]
        pos = struct.unpack_from('<3f', d, o + 16)
        nrm = struct.unpack_from('<3f', d, o + 28)
        uv = struct.unpack_from('<2f', d, o + 40)
        return pos, nrm, uv, [(bn[k], w[k]) for k in range(nb)]

    qc = ['$modelname "%s"' % h['name']]
    if static:
        qc.append('$staticprop')
    for c in cds:
        qc.append('$cdmaterials "%s"' % c)
    surf = mdl53.cstr(d, h['surfacepropindex'])
    qc.append('$surfaceprop "%s"' % surf)
    if h['fadeDistance'] != -1:
        qc.append('$fadedistance %s' % fmt(h['fadeDistance']))
    for flag, cmd in ((0x4, '$opaque'), (0x800, '$noforcedfade'), (0x10000, '$ambientboost'), (0x20000, '$donotcastshadows'),
                      (0x40000, '$casttextureshadows'), (0x200, '$obsolete')):
        if h['flags'] & flag:
            qc.append(cmd)
    # bones
    for i, b in enumerate(bones):
        parent = bones[b['parent']]['name'] if b['parent'] >= 0 else ''
        ang = b['rot']
        qc.append('$definebone "%s" "%s" %s %s %s %s %s %s 0 0 0 0 0 0' % (
            b['name'], parent, fmt(b['pos'][0]), fmt(b['pos'][1]), fmt(b['pos'][2]),
            fmt(math.degrees(ang[1])), fmt(math.degrees(ang[2])), fmt(math.degrees(ang[0]))))

    vtx = mdl53.parse_vtx(d, h['vtxOffset']) if h['vtxSize'] else None
    sz = mdl53.size(mdl53.BODYPART_FIELDS)
    lod_models = {}  # lod -> list of (old smd name, new smd name)
    num_lods = vtx['hdr']['numLODs'] if vtx else 1
    switch = [0.0] * num_lods
    first_smd = None
    for bpi in range(h['numbodyparts']):
        bo = h['bodypartindex'] + bpi * sz
        bp = mdl53.u(mdl53.BODYPART_FIELDS, d, bo)
        bpname = mdl53.cstr(d, bo + bp['sznameindex'])
        entries = []
        vstart = 0
        for mi in range(bp['nummodels']):
            mo = bo + bp['modelindex'] + mi * mdl53.size(mdl53.MODEL_FIELDS)
            md = mdl53.u(mdl53.MODEL_FIELDS, d, mo)
            vstart = md['vertexindex'] // 48
            if md['nummeshes'] == 0 or not vtx:
                entries.append(None)
                continue
            meshes = [mdl53.u(mdl53.MESH_FIELDS, d, mo + md['meshindex'] + k * 0x74) for k in range(md['nummeshes'])]
            vmodel = vtx['bodyparts'][bpi]['models'][mi]
            names = []
            for lod in range(num_lods):
                ld = vmodel['lods'][lod]
                switch[lod] = ld['switchPoint']
                tris = []
                for ei, me in enumerate(ld['meshes']):
                    mesh = meshes[ei]
                    mat = tex[skins[0][mesh['material']]] if skins else tex[mesh['material']]
                    for sg in me['stripgroups']:
                        go = sg['_off']
                        verts = [mdl53.u(mdl53.VTX_VERT, d, go + sg['vertOffset'] + k * 9) for k in range(sg['numVerts'])]
                        idx = struct.unpack_from('<%dH' % sg['numIndices'], d, go + sg['indexOffset'])
                        for t in range(0, len(idx), 3):
                            tri = []
                            # studiomdl reverses winding on load; write (a, c, b)
                            for c in (0, 2, 1):
                                vv = verts[idx[t + c]]
                                pos, nrm, uv, w = vvd_vertex(vstart + mesh['vertexoffset'] + vv['origMeshVertID'])
                                if static:
                                    pos = rotate(ROT_M90, pos)
                                    nrm = rotate(ROT_M90, nrm)
                                tri.append((pos, nrm, uv, w))
                            tris.append((mat, tri))
                smd_name = '%s_%s_lod%d.smd' % (base, md['name'].replace('/', '_').replace('\\', '_').replace('.', '_') or 'model%d' % mi, lod) \
                    if False else '%s_bp%d_m%d_lod%d.smd' % (base, bpi, mi, lod)
                skel = [[(b['pos'], b['rot']) for b in bones]]
                if static:
                    skel = [[((0, 0, 0), (0, 0, 0)) for b in bones]]
                write_smd(os.path.join(outdir, smd_name), bones, skel, tris)
                names.append(smd_name)
                if lod:
                    lod_models.setdefault(lod, []).append((names[0], smd_name))
            entries.append(names[0])
            if first_smd is None:
                first_smd = names[0]
        if len(entries) == 1 and entries[0]:
            qc.append('$body "%s" "%s"' % (bpname, entries[0]))
        else:
            qc.append('$bodygroup "%s"\n{' % bpname)
            for e in entries:
                qc.append('\tstudio "%s"' % e if e else '\tblank')
            qc.append('}')
    for lod in range(1, num_lods):
        qc.append('$lod %s\n{' % fmt(switch[lod]))
        for a, b in lod_models.get(lod, []):
            qc.append('\treplacemodel "%s" "%s"' % (a, b))
        qc.append('}')
    if len(skins) > 1:
        used = sorted({i for fam in skins for i in range(len(fam)) if any(f[i] != i for f in skins)})
        qc.append('$texturegroup "skinfamilies"\n{')
        for fam in skins:
            qc.append('\t{ ' + ' '.join('"%s"' % tex[fam[i]] for i in used) + ' }')
        qc.append('}')

    # attachments
    for i in range(h['numlocalattachments']):
        o = h['localattachmentindex'] + i * 0x5C
        a = mdl53.u(mdl53.ATTACH_FIELDS, d, o)
        mtx = [list(a['local'][0:4]), list(a['local'][4:8]), list(a['local'][8:12])]
        if static:
            mtx = concat(ROT_M90, mtx)  # studiomdl re-applies the default rotation
        ang = matrix_angles(mtx)
        qc.append('$attachment "%s" "%s" %s %s %s rotate %s %s %s' % (
            mdl53.cstr(d, o + a['sznameindex']), bones[a['localbone']]['name'], fmt(mtx[0][3]), fmt(mtx[1][3]), fmt(mtx[2][3]),
            fmt(math.degrees(ang[1])), fmt(math.degrees(ang[2])), fmt(math.degrees(ang[0]))))
    # hitboxes
    if not (h['flags'] & 1):
        for s in range(h['numhitboxsets']):
            so = h['hitboxsetindex'] + s * 12
            hs = mdl53.u(mdl53.HBOXSET_FIELDS, d, so)
            qc.append('$hboxset "%s"' % mdl53.cstr(d, so + hs['sznameindex']))
            for k in range(hs['numhitboxes']):
                bo = so + hs['hitboxindex'] + k * 0x44
                bb = mdl53.u(mdl53.BBOX_FIELDS, d, bo)
                nm = mdl53.cstr(d, bo + bb['szhitboxnameindex']) if bb['szhitboxnameindex'] else ''
                line = '$hbox %d "%s" %s %s %s %s %s %s' % (bb['group'], bones[bb['bone']]['name'], *(fmt(x) for x in bb['bbmin'] + bb['bbmax']))
                if nm or bb['forceCritPoint']:
                    line += ' "%s"' % nm
                if bb['forceCritPoint']:
                    line += ' %d' % bb['forceCritPoint']
                qc.append(line)
    # pose parameters
    for i in range(h['numlocalposeparameters']):
        o = h['localposeparamindex'] + i * 0x14
        p = mdl53.u(mdl53.POSEPARAM_FIELDS, d, o)
        line = '$poseparameter "%s" %s %s' % (mdl53.cstr(d, o + p['sznameindex']), fmt(p['start']), fmt(p['end']))
        if p['flags'] & 1:
            line += ' loop %s' % fmt(p['loop'])
        qc.append(line)
    # ik chains
    for i in range(h['numikchains']):
        o = h['ikchainindex'] + i * 0x20
        c = mdl53.u(mdl53.IKCHAIN_FIELDS, d, o)
        links = [mdl53.u(mdl53.IKLINK_FIELDS, d, o + c['linkindex'] + k * 0x1C) for k in range(c['numlinks'])]
        knee = links[0]['kneeDir'] if links else (0, 0, 0)
        qc.append('$ikchain "%s" "%s" knee %s %s %s' % (mdl53.cstr(d, o + c['sznameindex']), bones[links[-1]['bone']]['name'], *(fmt(x) for x in knee)))
    # include models
    for i in range(h['numincludemodels']):
        o = h['includemodelindex'] + i * 8
        nm = mdl53.cstr(d, o + struct.unpack_from('<i', d, o + 4)[0])
        if nm.lower().startswith('models/'):
            nm = nm[7:]
        qc.append('$includemodel "%s"' % nm)
    # keyvalues
    if h['keyvaluesize']:
        kv = d[h['keyvalueindex']:h['keyvalueindex'] + h['keyvaluesize']].split(b'\0')[0].decode('latin1')
        inner = kv.split('{', 1)[1].rsplit('}', 1)[0] if '{' in kv else kv
        qc.append('$keyvalues\n{\n%s\n}' % inner.strip())

    # animations
    anim_names = []
    for ai in range(h['numlocalanim']):
        a, name, frames = decode_anim(d, m, bones, ai)
        delta = bool(a['flags'] & 4)
        smd = 'anim_%s.smd' % name.replace('@', 'at_').replace('/', '_').replace('\\', '_')
        skel = []
        for fr in frames:
            row = []
            for bi, (pos, rm) in enumerate(fr):
                if static:
                    row.append(((0, 0, 0), (0, 0, 0)))
                else:
                    row.append(local_frame_to_smd(bi, pos, rm, root_fix=not delta))
            skel.append(row)
        write_smd(os.path.join(outdir, smd), bones, skel)
        opts = ['fps %s' % fmt(a['fps'])]
        if a['flags'] & 1:
            opts.append('loop')
        if delta:
            opts.append('// delta animation: decoded as raw deltas')
        anim_names.append(('a_' + name.replace('@', 'at_'), smd, opts, a))
        qc.append('$animation "%s" "%s" %s' % (anim_names[-1][0], smd, ' '.join(o for o in opts if not o.startswith('//'))))
    # sequences
    for si in range(h['numlocalseq']):
        o = h['localseqindex'] + si * 0xE8
        s = mdl53.u(mdl53.SEQDESC_FIELDS, d, o)
        label = mdl53.cstr(d, o + s['szlabelindex'])
        act = mdl53.cstr(d, o + s['szactivitynameindex'])
        n = s['groupsize'][0] * s['groupsize'][1]
        idxs = struct.unpack_from('<%dh' % n, d, o + s['animindexindex'])
        lines = ['$sequence "%s"\n{' % label]
        lines.append('\t' + ' '.join('"%s"' % anim_names[i][0] for i in idxs))
        if s['groupsize'][0] > 1 or s['groupsize'][1] > 1:
            lines.append('\tblendwidth %d' % s['groupsize'][0])
            for p in range(2):
                if s['paramindex'][p] >= 0:
                    po = h['localposeparamindex'] + s['paramindex'][p] * 0x14
                    pn = mdl53.cstr(d, po + struct.unpack_from('<i', d, po)[0])
                    lines.append('\tblend "%s" %s %s' % (pn, fmt(s['paramstart'][p]), fmt(s['paramend'][p])))
        if act:
            lines.append('\tactivity "%s" %d' % (act, s['actweight']))
        if s['flags'] & 1:
            lines.append('\tloop')
        if s['flags'] & 4:
            lines.append('\tdelta')
        if s['flags'] & 0x400:
            lines.append('\thidden')
        if s['flags'] & 0x8:
            lines.append('\tautoplay')
        if s['flags'] & 0x100:
            lines.append('\trealtime')
        lines.append('\tfadein %s' % fmt(s['fadeintime']))
        lines.append('\tfadeout %s' % fmt(s['fadeouttime']))
        first = anim_names[idxs[0]][3]
        for e in range(s['numevents']):
            eo = o + s['eventindex'] + e * 0x50
            ev = mdl53.u(mdl53.EVENT_FIELDS, d, eo)
            ename = mdl53.cstr(d, eo + ev['szeventindex']) if ev['type'] else str(ev['event'])
            frame = int(round(ev['cycle'] * max(1, first['numframes'] - 1)))
            lines.append('\t{ event "%s" %d "%s" }' % (ename, frame, ev['options']))
        lines.append('}')
        qc.append('\n'.join(lines))

    # collision
    if h['phySize']:
        p = phylib.parse_phy(d[h['phyOffset']:h['phyOffset'] + h['phySize']])
        text = p['text']
        jointed = p['solidCount'] > 1 or 'parent' in text
        tris = []
        solid_names = []
        import re
        for blk in re.findall(r'solid\s*\{([^}]*)\}', text):
            kv = dict(re.findall(r'"([^"]+)"\s+"([^"]*)"', blk))
            solid_names.append(kv.get('name', ''))
        # bind-pose world transforms
        world = []
        for i, b in enumerate(bones):
            lm = angle_matrix(b['rot'])
            for k in range(3):
                lm[k][3] = b['pos'][k]
            world.append(concat(world[b['parent']], lm) if b['parent'] >= 0 else lm)
        for si, s in enumerate(p['solids']):
            pts = [phylib.ivp_to_hl(q[:3]) for q in s['points']]
            bone = 0
            if jointed:
                nm = solid_names[si] if si < len(solid_names) else ''
                bone = next((i for i, b in enumerate(bones) if b['name'].lower() == nm.lower()), 0)
            for L in s['ledges']:
                if L['has_children']:
                    continue
                for t in L['tris']:
                    tri = []
                    for c in (0, 2, 1):
                        pt = pts[t['edges'][c]['start']]
                        if jointed:
                            pt = transform(world[bone], pt)
                        elif static:
                            pt = rotate(ROT_M90, pt)
                        tri.append((pt, (0, 0, 1), (0, 0), [(bone, 1.0)]))
                    tris.append(('phys', tri))
        skel = [[(b['pos'], b['rot']) for b in bones]] if not static else [[((0, 0, 0), (0, 0, 0)) for b in bones]]
        write_smd(os.path.join(outdir, base + '_physics.smd'), bones, skel, tris)
        mass = re.search(r'"totalmass"\s+"([^"]+)"', text)
        cmd = '$collisionjoints' if jointed else '$collisionmodel'
        lines = ['%s "%s_physics.smd"\n{' % (cmd, base)]
        if mass:
            lines.append('\t$mass %s' % mass.group(1))
        if '"concave" "1"' in text:
            lines.append('\t$concave')
        for blk in re.findall(r'ragdollconstraint\s*\{([^}]*)\}', text):
            kv = dict(re.findall(r'"([^"]+)"\s+"([^"]*)"', blk))
            child = solid_names[int(kv['child'])]
            for ax in 'xyz':
                lines.append('\t$jointconstrain "%s" %s limit %s %s %s' % (child, ax, kv[ax + 'min'], kv[ax + 'max'], kv[ax + 'friction']))
        lines.append('}')
        qc.append('\n'.join(lines))

    qcpath = os.path.join(outdir, base + '.qc')
    with open(qcpath, 'w', newline='\n') as f:
        f.write('// decompiled by r2-model-compiler tools/decompile53.py\n')
        f.write('\n'.join(qc) + '\n')
    return qcpath


if __name__ == '__main__':
    print(decompile(sys.argv[1], sys.argv[2]))
