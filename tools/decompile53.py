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


def decode_frame_movement(d, m, ai, frame_indices):
    """Respawn root motion (STUDIO_FRAMEMOVEMENT): (x, y, z, yaw degrees) per frame, or None."""
    o = m.hdr['localanimindex'] + ai * 0x5C
    a = mdl53.u(mdl53.ANIMDESC_FIELDS, d, o)
    if not (a['flags'] & 0x40000) or not a['framemovementindex']:
        return None
    fo = o + a['framemovementindex']
    scale = struct.unpack_from('<4f', d, fo)
    offs = struct.unpack_from('<4h', d, fo + 16)
    return [tuple(extract(d, fo + offs[c] if offs[c] > 0 else None, f, scale[c]) for c in range(4)) for f in frame_indices]


def apply_root_motion(bones, pose, fm):
    """Premultiplies root bones by a frame-movement transform."""
    if fm is None:
        return pose
    mv = angle_matrix((0, 0, math.radians(fm[3])))
    for k in range(3):
        mv[k][3] = fm[k]
    out = []
    for bi, (pos, rm) in enumerate(pose):
        if bones[bi]['parent'] == -1:
            mm = [row[:] for row in rm]
            for k in range(3):
                mm[k][3] = pos[k]
            mm = concat(mv, mm)
            pos = (mm[0][3], mm[1][3], mm[2][3])
            rm = [[mm[r][c] for c in range(3)] + [0.0] for r in range(3)]
        out.append((pos, rm))
    return out


def decode_anim(d, m, bones, ai, frame_indices=None, with_scale=False, absent=None):
    """Return descriptor, name and local poses in the requested frame order.

    Each pose entry is (pos, rotation matrix), or (pos, rotation matrix, scale) with
    with_scale. Scale is bind scale + value * scalescale (no bind scale for deltas)."""
    h = m.hdr
    o = h['localanimindex'] + ai * 0x5C
    a = mdl53.u(mdl53.ANIMDESC_FIELDS, d, o)
    name = mdl53.cstr(d, o + a['sznameindex'])
    nf = a['numframes']
    delta = bool(a['flags'] & 4)
    frames = []
    for f in range(nf) if frame_indices is None else frame_indices:
        if f < 0 or f >= nf:
            raise ValueError('animation frame out of range: %d' % f)
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
            # scale
            if flags & 0x08:
                scl = tuple(half(x) for x in struct.unpack_from('<3H', d, r + 22))
            else:
                offs = struct.unpack_from('<3h', d, r + 22)
                scl = [extract(d, r + 22 + offs[c] if offs[c] > 0 else None, lf, b['scalescale'][c]) for c in range(3)]
                if not delta:
                    scl = [scl[c] + b['scale'][c] for c in range(3)]
                scl = tuple(scl)
            pose[bone] = (pos, rm, scl) if with_scale else (pos, rm)
            nxt = struct.unpack_from('<i', d, r + 28)[0]
            if nxt == 0:
                break
            r += nxt
        out = []
        for bi, b in enumerate(bones):
            one = ((1.0, 1.0, 1.0),) if with_scale else ()
            if bi in pose:
                out.append(pose[bi])
            elif delta:
                out.append(((0.0, 0.0, 0.0), angle_matrix((0, 0, 0))) + one)
            elif absent is not None and absent[bi] is not None:
                out.append(tuple(absent[bi][:2]) + one)  # bone not stored: caller's fallback pose
            else:
                out.append((tuple(b['pos']), angle_matrix(b['rot'])) + one)
        frames.append(out)
    return a, name, frames


# ---------------------------------------------------------------- writers
def fmt(v):
    s = '%.6f' % v
    return '0.000000' if s == '-0.000000' else s


def qc_quote(value):
    return '"' + value.replace('\\', '\\\\').replace('"', '\\"') + '"'


def write_smd(path, bones, skeleton_frames, tris=None):
    with open(path, 'w', newline='\n') as f:
        f.write('version 1\nnodes\n')
        for i, b in enumerate(bones):
            f.write('%d "%s" %d\n' % (i, b['name'], b['parent']))
        f.write('end\nskeleton\n')
        for t, fr in enumerate(skeleton_frames):
            f.write('time %d\n' % t)
            for i, entry in enumerate(fr):
                pos, rot = entry[0], entry[1]
                line = '%d %s %s %s %s %s %s' % (i, fmt(pos[0]), fmt(pos[1]), fmt(pos[2]), fmt(rot[0]), fmt(rot[1]), fmt(rot[2]))
                if len(entry) > 2:  # optional local scale (r2-model-compiler SMD extension)
                    line += ' %s %s %s' % tuple(fmt(v) for v in entry[2])
                f.write(line + '\n')
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


def decode_ik_rules(d, h, ai):
    """IK rules of an animation: list of (rule fields, compressed error offset or 0)."""
    o = h['localanimindex'] + ai * 0x5C
    a = mdl53.u(mdl53.ANIMDESC_FIELDS, d, o)
    size = mdl53.size(mdl53.IKRULE_FIELDS)
    out = []
    for k in range(a['numikrules']):
        ro = o + a['ikruleindex'] + k * size
        r = mdl53.u(mdl53.IKRULE_FIELDS, d, ro)
        r['attachment'] = mdl53.cstr(d, ro + r['szattachmentindex']) if r['szattachmentindex'] else ''
        r['offset'] = ro
        out.append(r)
    return out


def decode_ik_error(d, rule, frame):
    """(pos, angles) of a rule's compressed error at an animation frame, or None."""
    if not rule['compressedikerrorindex']:
        return None
    co = rule['offset'] + rule['compressedikerrorindex']
    scale = struct.unpack_from('<6f', d, co)
    offs = struct.unpack_from('<6h', d, co + 24)
    f = frame - rule['iStart']
    vals = [extract(d, co + offs[k] if offs[k] > 0 else None, max(f, 0), scale[k]) for k in range(6)]
    return tuple(vals[:3]), tuple(vals[3:])


def ik_error_frames(d, rule):
    """Number of frames in a rule's compressed error stream (studiomdl's numerror)."""
    co = rule['offset'] + rule['compressedikerrorindex']
    p = co + struct.unpack_from('<6h', d, co + 24)[0]
    n = 0
    for _ in range(4096):
        valid, total = d[p], d[p + 1]
        if total == 0:
            break
        n += total
        p += (valid + 1) * 2
        if p >= co + struct.unpack_from('<6h', d, co + 24)[1]:
            break
    return n


def ik_rule_options(d, h, ai, a, bones, chain_names, chain_ends, delta):
    """QC ikrule options reproducing an animation's retail rules, in order."""
    rules = decode_ik_rules(d, h, ai)
    nf = a['numframes']
    opts = []
    frames = [[int(round(r[x] * (nf - 1))) if nf > 1 else 0 for x in ('start', 'peak', 'tail', 'end')] for r in rules]
    # studiomdl quirk: a rule written with start "." overwrites the previous same-chain
    # rule's end after that rule was processed (so it can end up before its tail,
    # unwrapped). Recover the original end from the error stream length and write the
    # next rule's start as "." so the compiler repeats it.
    dot_start = set()
    for i, r in enumerate(rules):
        st, pk, tl, en = frames[i]
        if nf > 1 and en < tl and r['compressedikerrorindex']:
            n = ik_error_frames(d, r)
            for cand in (st + n - 1, st + n - 3):
                if tl <= cand and (cand >= nf) == (cand != st + n - 1):
                    frames[i][3] = cand
                    break
            for k in range(1, len(rules)):
                j = (i + k) % len(rules)
                if rules[j]['chain'] == r['chain']:
                    dot_start.add(j)
                    break
        elif nf > 1 and en < tl:
            for k in range(1, len(rules)):
                j = (i + k) % len(rules)
                if rules[j]['chain'] == r['chain']:
                    dot_start.add(j)
                    break
    # The mirror case: a rule written with end "." overwrites the next same-chain rule's
    # start; when that rule came earlier it keeps a start after its peak, unwrapped.
    # Recover its original start from the error stream and write the previous rule's end
    # as "." so the compiler repeats the overwrite.
    dot_end = set()
    for i, r in enumerate(rules):
        st, pk, tl, en = frames[i]
        if nf > 1 and st > pk:
            if r['compressedikerrorindex']:
                n = ik_error_frames(d, r)
                for cand in (en - n + 1, en - n + 3):
                    if 0 <= cand <= pk and (en >= nf) == (cand != en - n + 1):
                        frames[i][0] = cand
                        break
            else:
                frames[i][0] = pk
            for k in range(1, len(rules)):
                j = (i - k) % len(rules)
                if rules[j]['chain'] == r['chain']:
                    dot_end.add(j)
                    break
    for i, r in enumerate(rules):
        if not 0 <= r['chain'] < len(chain_names):
            continue
        kind = {1: 'touch', 2: None, 3: 'footstep', 4: 'release', 5: 'attachment', 6: 'unlatch'}.get(r['type'])
        if not kind:
            continue
        o = 'ikrule "%s" %s' % (chain_names[r['chain']], kind)
        if kind == 'touch':
            o += ' "%s"' % (bones[r['bone']]['name'] if 0 <= r['bone'] < len(bones) else '')
        elif kind == 'attachment':
            o += ' "%s"' % r['attachment']
        if r['slot'] != r['chain']:
            o += ' target %d' % r['slot']
        if nf > 1:
            fr = [str(v) for v in frames[i]]
            if i in dot_start:
                fr[0] = '.'
            if i in dot_end:
                fr[3] = '.'
            o += ' range %s %s %s %s' % tuple(fr)
            o += ' contact %d' % int(round(r['contact'] * (nf - 1)))
        if kind == 'footstep':
            o += ' height %s radius %s floor %s endheight %s' % (fmt(r['height']), fmt(r['radius']), fmt(r['floor']), fmt(r['endHeight']))
        if kind == 'attachment' and r['bone'] == -1:
            o += ' fakeorigin %s %s %s' % tuple(fmt(v) for v in r['pos'])
        if kind == 'attachment' and 0 <= r['bone'] < len(bones) and r['bone'] != chain_ends[r['chain']]:
            o += ' bone "%s"' % bones[r['bone']]['name']
        opts.append(o)
    # studiomdl adds release rules for chains an animation moves without rules; retail
    # animations that lack them were compiled with noautoik
    if not delta and chain_names and {r['chain'] for r in rules} != set(range(len(chain_names))):
        opts.append('noautoik')
    return opts


def anim_record_bones(d, h, ai):
    """Bones that have a record in any section of an animation."""
    o = h['localanimindex'] + ai * 0x5C
    a = mdl53.u(mdl53.ANIMDESC_FIELDS, d, o)
    if a['sectionframes']:
        count = (a['numframes'] - 1) // a['sectionframes'] + 2
        starts = [o + struct.unpack_from('<i', d, o + a['sectionindex'] + s * 4)[0] for s in range(count)]
    else:
        starts = [o + a['animindex']]
    out = set()
    for r in starts:
        for _ in range(512):
            bone = d[r + 4]
            if bone == 255:
                break
            out.add(bone)
            nxt = struct.unpack_from('<i', d, r + 28)[0]
            if not nxt:
                break
            r += nxt
    return out


def decode_procedural(d, h, bi, b):
    """Non-jiggle procedural bone data (types 1-4) as a dict, or None."""
    t = b['proctype']
    if t not in (1, 2, 3, 4) or not b['procindex']:
        return None
    o = h['boneindex'] + bi * 0xF4 + b['procindex']
    if t == 1:
        control, axis = struct.unpack_from('<2i', d, o)
        return {'type': 1, 'control': control, 'axis': axis,
                'pos': [struct.unpack_from('<3f', d, o + 8 + 12 * k) for k in range(6)],
                'quat': [struct.unpack_from('<4f', d, o + 80 + 16 * k) for k in range(6)]}
    if t == 2:
        control, n, ti = struct.unpack_from('<3i', d, o)
        trig = []
        for k in range(n):
            q = o + ti + 48 * k
            trig.append({'inv_tolerance': struct.unpack_from('<f', d, q)[0], 'trigger': struct.unpack_from('<4f', d, q + 4),
                         'pos': struct.unpack_from('<3f', d, q + 20), 'quat': struct.unpack_from('<4f', d, q + 32)})
        return {'type': 2, 'control': control, 'triggers': trig}
    parent, aim = struct.unpack_from('<2i', d, o)
    return {'type': t, 'parent': parent, 'aim': aim, 'aimvector': struct.unpack_from('<3f', d, o + 8),
            'upvector': struct.unpack_from('<3f', d, o + 20), 'basepos': struct.unpack_from('<3f', d, o + 32)}


def euler_degrees(q):
    """RadianEuler (x, y, z) of a quaternion, in degrees."""
    return tuple(math.degrees(v) for v in matrix_angles(quat_matrix(q)))


def procedural_files(d, h, bones, attachments, outdir, base):
    """Writes $proceduralbones files for types 1-4; returns the QC lines."""
    vrd, axis = [], []
    for bi, b in enumerate(bones):
        p = decode_procedural(d, h, bi, b)
        if not p:
            continue
        name = b['name']
        parent = bones[b['parent']]['name'] if b['parent'] >= 0 else ''
        if p['type'] == 2:
            c = bones[p['control']]
            cparent = bones[c['parent']]['name'] if c['parent'] >= 0 else ''
            vrd.append('<helper> %s %s %s %s' % (name, parent, cparent, c['name']))
            vrd.append('<display> 1 1 1 0')
            vrd.append('<basepos> 0 0 0')
            for t in p['triggers']:
                tol = math.degrees(1 / t['inv_tolerance']) if t['inv_tolerance'] else 0
                vrd.append('<trigger> %s %s %s %s %s %s %s %s %s %s' % (
                    fmt(tol), *(fmt(v) for v in euler_degrees(t['trigger'])), *(fmt(v) for v in euler_degrees(t['quat'])),
                    *(fmt(v) for v in t['pos'])))
        elif p['type'] in (3, 4):
            aim = attachments[p['aim']] if p['type'] == 4 else bones[p['aim']]['name']
            vrd.append('<aimconstraint> %s %s %s' % (name, bones[p['parent']]['name'], aim))
            vrd.append('<aimvector> %s %s %s' % tuple(fmt(v) for v in p['aimvector']))
            vrd.append('<upvector> %s %s %s' % tuple(fmt(v) for v in p['upvector']))
            vrd.append('<basepos> %s %s %s' % tuple(fmt(v) for v in p['basepos']))
        else:
            c = bones[p['control']]
            cparent = bones[c['parent']]['name'] if c['parent'] >= 0 else ''
            axis.append('bone "%s" "%s" "%s" "%s" %d' % (name, parent, c['name'], cparent, p['axis'] + 1))
            axis.append('basepos 0 0 0')
            for k in range(6):
                roll, pitch, yaw = euler_degrees(p['quat'][k])
                axis.append('axis %d %s %s %s %s %s %s' % (k, *(fmt(v) for v in p['pos'][k]), fmt(roll), fmt(pitch), fmt(yaw)))
    lines = []
    if vrd:
        with open(os.path.join(outdir, base + '_procedural.vrd'), 'w', newline='\n') as f:
            f.write('\n'.join(vrd) + '\n')
        lines.append('$proceduralbones "%s_procedural.vrd"' % base)
    if axis:
        with open(os.path.join(outdir, base + '_axisinterp.txt'), 'w', newline='\n') as f:
            f.write('version 1\nproceduralbones\n' + '\n'.join(axis) + '\nend\n')
        lines.append('$proceduralbones "%s_axisinterp.txt"' % base)
    return lines


def decode_ui_panels(d, h):
    """RUI panel meshes: list of dicts (hash, name, parents, vertices, faces, ccw, bounds)."""
    out = []
    for i in range(h['uiPanelCount']):
        ho = h['uiPanelOffset'] + i * 8
        hsh, mo = struct.unpack_from('<Ii', d, ho)
        o = ho + mo
        pc, vc, fc, po, vo, io, bo, wind = struct.unpack_from('<7iI', d, o)
        out.append({'hash': hsh, 'name': mdl53.cstr(d, o + 32),
                    'parents': list(struct.unpack_from('<%dh' % pc, d, o + po)),
                    'vertices': [struct.unpack_from('<i3f', d, o + vo + 16 * k) for k in range(vc)],
                    'faces': [struct.unpack_from('<3h', d, o + io + 6 * k) for k in range(fc)],
                    'ccw': wind,
                    'bounds': [struct.unpack_from('<8f', d, o + bo + 32 * k) for k in range(fc)]})
    return out


def ui_panel_hash(name):
    # The client's RUI mesh hash: MurmurHash3 x86_32, seed 0, bytes ANDed with 0xDF.
    m = 0xffffffff
    rol = lambda x, r: ((x << r) | (x >> (32 - r))) & m
    mix = lambda k: (rol((k * 0xcc9e2d51) & m, 15) * 0x1b873593) & m
    data = bytes(c & 0xdf for c in name.encode('latin-1'))
    h, full = 0, len(data) // 4 * 4
    for i in range(0, full, 4):
        h = (rol(h ^ mix(int.from_bytes(data[i:i + 4], 'little')), 13) * 5 + 0xe6546b64) & m
    h ^= mix(int.from_bytes(data[full:], 'little')) ^ (len(data) + 1)  # the nul counts
    h ^= h >> 16; h = (h * 0x85ebca6b) & m; h ^= h >> 13; h = (h * 0xc2b2ae35) & m
    return h ^ (h >> 16)


def uipanel_lines(panel, bones):
    fmt = lambda v: '%.9g' % v  # exact float round trip
    # The compiler derives the hash from the name; only an unusual one is written.
    hsh = '' if panel['hash'] == ui_panel_hash(panel['name']) else ' 0x%08x' % panel['hash']
    lines = ['$uipanel "%s"%s' % (panel['name'], hsh), '{']
    lines += ['	parent "%s"' % bones[b]['name'] for b in panel['parents']]
    lines += ['	vertex %d %s %s %s' % (v[0], fmt(v[1]), fmt(v[2]), fmt(v[3])) for v in panel['vertices']]
    for k, f in enumerate(panel['faces']):
        lines.append('	face %d %d %d%s' % (f[0], f[1], f[2], ' ccw' if panel['ccw'] >> k & 1 else ''))
    lines += ['	bounds ' + ' '.join(fmt(v) for v in b) for b in panel['bounds']]
    lines.append('}')
    return lines


def jigglebone_lines(name, j):
    """$jigglebone block for a decoded mstudiojigglebone_t (angles stored in radians)."""
    f = j['flags']
    deg = lambda v: fmt(math.degrees(v))
    out = ['$jigglebone "%s"' % name, '{']
    if f & 0x03:
        out.append('	%s' % ('is_flexible' if f & 0x01 else 'is_rigid'))
        out.append('	{')
        body = [('length', j['length']), ('tip_mass', j['tipMass']),
                ('yaw_stiffness', j['yawStiffness']), ('yaw_damping', j['yawDamping']),
                ('pitch_stiffness', j['pitchStiffness']), ('pitch_damping', j['pitchDamping']),
                ('along_stiffness', j['alongStiffness']), ('along_damping', j['alongDamping'])]
        out += ['		%s %s' % (k, fmt(v)) for k, v in body]
        if f & 0x20:
            out.append('		allow_length_flex')
        if f & 0x10:
            out.append('		angle_constraint %s' % deg(j['angleLimit']))
        if f & 0x04:
            out.append('		yaw_constraint %s %s' % (deg(j['minYaw']), deg(j['maxYaw'])))
        out += ['		yaw_friction %s' % fmt(j['yawFriction']), '		yaw_bounce %s' % fmt(j['yawBounce'])]
        if f & 0x08:
            out.append('		pitch_constraint %s %s' % (deg(j['minPitch']), deg(j['maxPitch'])))
        out += ['		pitch_friction %s' % fmt(j['pitchFriction']), '		pitch_bounce %s' % fmt(j['pitchBounce'])]
        out.append('	}')
    if f & 0x40:
        out += ['	has_base_spring', '	{',
                '		stiffness %s' % fmt(j['baseStiffness']), '		damping %s' % fmt(j['baseDamping']),
                '		base_mass %s' % fmt(j['baseMass']),
                '		left_constraint %s %s' % (fmt(j['baseMinLeft']), fmt(j['baseMaxLeft'])),
                '		left_friction %s' % fmt(j['baseLeftFriction']),
                '		up_constraint %s %s' % (fmt(j['baseMinUp']), fmt(j['baseMaxUp'])),
                '		up_friction %s' % fmt(j['baseUpFriction']),
                '		forward_constraint %s %s' % (fmt(j['baseMinForward']), fmt(j['baseMaxForward'])),
                '		forward_friction %s' % fmt(j['baseForwardFriction']), '	}']
    out.append('}')
    return out


def decompile(path, outdir):
    with open(path, 'rb') as source:
        d = source.read()
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

    # jiggle bones (procedural type 5), every value written so the struct is reproduced
    for b in bones:
        if b['proctype'] == 5 and b.get('jiggle'):
            qc.extend(jigglebone_lines(b['name'], b['jiggle']))
    # other procedural bones go to .vrd / axis files
    att_names = []
    for i in range(h['numlocalattachments']):
        o = h['localattachmentindex'] + i * mdl53.size(mdl53.ATTACH_FIELDS)
        att_names.append(mdl53.cstr(d, o + struct.unpack_from('<i', d, o)[0]))
    qc.extend(procedural_files(d, h, bones, att_names, outdir, base))
    # bone followers (bones whose physics objects follow their animation)
    if h['boneFollowerCount']:
        for k in struct.unpack_from('<%di' % h['boneFollowerCount'], d, h['boneFollowerOffset']):
            qc.append('$bonefollower "%s"' % bones[k]['name'])
    # RUI panel meshes, written out in full (with the original hash)
    for panel in decode_ui_panels(d, h):
        qc.extend(uipanel_lines(panel, bones))
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
    chain_names, chain_ends = [], []
    for i in range(h['numikchains']):
        o = h['ikchainindex'] + i * 0x20
        c = mdl53.u(mdl53.IKCHAIN_FIELDS, d, o)
        links = [mdl53.u(mdl53.IKLINK_FIELDS, d, o + c['linkindex'] + k * 0x1C) for k in range(c['numlinks'])]
        knee = links[0]['kneeDir'] if links else (0, 0, 0)
        qc.append('$ikchain "%s" "%s" knee %s %s %s' % (mdl53.cstr(d, o + c['sznameindex']), bones[links[-1]['bone']]['name'], *(fmt(x) for x in knee)))
        chain_names.append(mdl53.cstr(d, o + c['sznameindex']))
        chain_ends.append(links[-1]['bone'] if links else -1)
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
    ref_pose = None
    if h['numlocalseq'] and h['numlocalanim']:
        q0 = mdl53.u(mdl53.SEQDESC_FIELDS, d, h['localseqindex'])
        ref_ai = struct.unpack_from('<h', d, h['localseqindex'] + q0['animindexindex'])[0]
        if 0 <= ref_ai < h['numlocalanim']:
            ref_a = mdl53.u(mdl53.ANIMDESC_FIELDS, d, h['localanimindex'] + ref_ai * 0x5C)
            if not ref_a['flags'] & 4:
                ref_pose = decode_anim(d, m, bones, ref_ai, [0])[2][0]
    # Weight lists. Respawn sequences carry their own lists ("seqweights"), which need not
    # match the weights the animations were encoded with. Each animation gets an encoding
    # list of 1 for the bones it stores and 0 for the rest, so exactly those are encoded.
    weight_names = {}

    def weightlist(w, prefix):
        if w not in weight_names:
            weight_names[w] = '%s_%d' % (prefix, len(weight_names))
            body = ' '.join('"%s" %s' % (b['name'], fmt(x)) for b, x in zip(bones, w))
            qc.append('$weightlist "%s" { %s }' % (weight_names[w], body))
        return weight_names[w]

    seq_weights, zero_in_seq = {}, {}
    for si in range(h['numlocalseq']):
        o = h['localseqindex'] + si * 0xE8
        q = mdl53.u(mdl53.SEQDESC_FIELDS, d, o)
        if not q['weightlistindex']:
            continue
        w = struct.unpack_from('<%df' % len(bones), d, o + q['weightlistindex'])
        seq_weights[si] = weightlist(w, 'weights')  # explicit even when all 1
        for k in range(q['groupsize'][0] * q['groupsize'][1]):
            ai = struct.unpack_from('<h', d, o + q['animindexindex'] + k * 2)[0]
            zero_in_seq.setdefault(ai, set()).update(bi for bi, x in enumerate(w) if x == 0)
    anim_weight_names = {}
    for ai in range(h['numlocalanim']):
        stored = anim_record_bones(d, h, ai)
        if len(stored) < len(bones):
            anim_weight_names[ai] = weightlist(tuple(1.0 if bi in stored else 0.0 for bi in range(len(bones))), 'stored')
    for ai in range(h['numlocalanim']):
        # Bones an animation does not store and its sequence zeroes are filled from the
        # reference animation rather than the bind pose: retail computed values such as IK
        # targets from the full source pose, whose root carries the usual rotation.
        stored = anim_record_bones(d, h, ai)
        zeros = zero_in_seq.get(ai, set())
        absent = [ref_pose[bi] if bi in zeros and bi not in stored else None for bi in range(len(bones))] if ref_pose else None
        a, name, frames = decode_anim(d, m, bones, ai, with_scale=True, absent=absent)
        # bone scale goes into the SMD's optional scale columns, only for animations
        # that use it (STUDIO_HAS_SCALE), so other exports stay plain studiomdl SMD
        scales = [[e[2] for e in fr] for fr in frames] if a['flags'] & 0x20000 else None
        frames = [[(e[0], e[1]) for e in fr] for fr in frames]
        delta = bool(a['flags'] & 4)
        fms = decode_frame_movement(d, m, ai, range(len(frames)))
        if fms is not None and not delta:
            # bake root motion back into the root bones; "rm" re-extracts it on compile
            frames = [apply_root_motion(bones, fr, fm) for fr, fm in zip(frames, fms)]
        smd = 'anim_%s.smd' % name.replace('@', 'at_').replace('/', '_').replace('\\', '_')
        skel = []
        for fi, fr in enumerate(frames):
            row = []
            for bi, (pos, rm) in enumerate(fr):
                if static:
                    row.append(((0, 0, 0), (0, 0, 0)))
                else:
                    row.append(local_frame_to_smd(bi, pos, rm, root_fix=not delta))
                if scales is not None:
                    row[-1] = tuple(row[-1]) + (scales[fi][bi],)
            skel.append(row)
        write_smd(os.path.join(outdir, smd), bones, skel)
        opts = ['fps %s' % fmt(a['fps'])]
        if a['flags'] & 1:
            opts.append('loop noforceloop')
        if a['flags'] & 2:
            opts.append('snap')
        if fms is not None and not static:
            opts.append('rm')
        if delta:
            opts.append('delta')  # frames are raw local deltas
        if a['flags'] & 0x20:
            # all-zero delta (retail flags 0x24); keep the frame count when there is one
            opts.append('noanimation' if a['numframes'] <= 1 else 'noanim_keepduration')
        if ai in anim_weight_names:
            opts.append('weightlist "%s"' % anim_weight_names[ai])
        opts += ik_rule_options(d, h, ai, a, bones, chain_names, chain_ends, delta)
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
        if si in seq_weights:
            lines.append('\tseqweights "%s"' % seq_weights[si])
        if s['groupsize'][0] > 1 or s['groupsize'][1] > 1:
            lines.append('\tblendwidth %d' % s['groupsize'][0])
            for p in range(2):
                if s['paramindex'][p] >= 0:
                    po = h['localposeparamindex'] + s['paramindex'][p] * 0x14
                    pn = mdl53.cstr(d, po + struct.unpack_from('<i', d, po)[0])
                    lines.append('\tblend "%s" %s %s' % (pn, fmt(s['paramstart'][p]), fmt(s['paramend'][p])))
            # retail blend positions need not be evenly spaced (e.g. a neutral pose at 0)
            nk = s['groupsize'][0] + s['groupsize'][1]
            if s['posekeyindex']:
                keys = struct.unpack_from('<%df' % nk, d, o + s['posekeyindex'])
                even = []
                for p in range(2):
                    g = s['groupsize'][p]
                    for k in range(g):
                        f = k / (g - 1) if g > 1 else 0.0
                        even.append(s['paramstart'][p] * (1 - f) + s['paramend'][p] * f)
                if any(abs(a - b) > 1e-3 for a, b in zip(keys, even)):
                    lines.append('\tposekeys %d %s' % (nk, ' '.join(fmt(k) for k in keys)))
        if act:
            lines.append('\tactivity "%s" %d' % (act, s['actweight']))
        if s['flags'] & 1:
            lines.append('\tloop noforceloop')
        if s['flags'] & 2:
            lines.append('\tsnap')
        if s['flags'] & 0x200:
            lines.append('\tlocal')
        if s['flags'] & 0x4000:
            lines.append('\tworldspace')  # also sets post
        elif s['flags'] & 0x10 and not s['flags'] & 4:
            lines.append('\tpost')
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
            # Retail Titan clips with one frame can store 0/0 (NaN) cycles.
            # Their only possible source-frame event time is zero.
            frame = 0 if first['numframes'] <= 1 else int(round(ev['cycle'] * (first['numframes'] - 1)))
            lines.append('\t{ event %s %d %s }' % (qc_quote(ename), frame, qc_quote(ev['options'])))
        lines.append('}')
        qc.append('\n'.join(lines))

    # collision
    if h['phySize']:
        p = phylib.parse_phy(d[h['phyOffset']:h['phyOffset'] + h['phySize']])
        text = p['text']
        # jointed solids carry ledge client data = bone + 1 (rigid $collisionmodel uses 0)
        jointed = p['solidCount'] > 1 or 'parent' in text or any(
            L['client_or_node'] for sd in p['solids'] for L in sd['ledges'] if not L['has_children'])
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
            pieces = [L for L in s['ledges'] if not L['has_children']]
            for piece, L in enumerate(pieces):
                # Collision-only normals encode one smoothing group per piece;
                # geometry comes from the triangle positions. Distinct unit
                # normals preserve even flush or overlapping convex pieces.
                z = 1 - 2 * (piece + .5) / len(pieces)
                radius = math.sqrt(max(0., 1 - z*z))
                angle = piece * math.pi * (3 - math.sqrt(5))
                piece_normal = (radius * math.cos(angle), radius * math.sin(angle), z)
                for t in L['tris']:
                    tri = []
                    for c in (0, 2, 1):
                        index = t['edges'][c]['start']
                        pt, normal = pts[index], piece_normal
                        if jointed:
                            pt = transform(world[bone], pt)
                            normal = rotate(world[bone], normal)
                        elif static:
                            pt = rotate(ROT_M90, pt)
                            normal = rotate(ROT_M90, normal)
                        tri.append((pt, normal, (0, 0), [(bone, 1.0)]))
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
        # several convex pieces under one joint (no "concave" key in retail's text)
        if jointed and any(len([L for L in s['ledges'] if not L['has_children']]) > 1 for s in p['solids']):
            lines.append('\t$concaveperjoint')
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
