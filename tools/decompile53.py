"""Decompile a Titanfall 2 (v53) .mdl into QC + SMD files that r2studiomdl recompiles.

Conventions match studiomdl so a decompile/recompile round trip is lossless where
the format allows: animation root bones get studiomdl's built-in 90 degree yaw
removed, static prop vertices are rotated back, and the bind pose is written as is.

usage: python tools/decompile53.py model.mdl outdir
"""
import collections
import math
import os
import random
import re
import struct
import sys
import uuid

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


def matrix_quaternion(m):
    """3x4 rotation -> (x, y, z, w)."""
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return ((m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s, 0.25 * s)
    if m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
        return (0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s, (m[2][1] - m[1][2]) / s)
    if m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
        return ((m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s, (m[0][2] - m[2][0]) / s)
    s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
    return ((m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s, (m[1][0] - m[0][1]) / s)


def write_dmx(path, bones, skeleton, faces, loose=(), joint_order=None):
    """A keyvalues2 DMX (model format 22) holding a skeleton and one mesh of polygons.

    skeleton: per bone (pos, RadianEuler rot), parent relative.
    faces: (material, piece, corners); a corner is (pos, normal, [(bone, weight)], extra)
    with optional extra keys 'uv', 'uv2', 'color' (RGBA floats 0-1) and 'tangent' (xyz, sign). Corners with the
    same position and weights within a piece share one vertex, so each piece imports as
    a connected mesh.
    loose: (pos, [(bone, weight)], piece) points no face uses, written as loose vertices
    (retail collision pieces keep points such as cylinder-cap centres).
    joint_order: bone indices in jointList order (default: bone order); the compiler builds
    collision solids in this order."""
    rng = random.Random(os.path.basename(path))  # stable element ids for identical exports
    ids = (uuid.UUID(int=rng.getrandbits(128)) for _ in iter(int, 1))
    # keyvalues2 strings escape backslashes and quotes (material paths keep their slashes)
    q = lambda s: '"%s"' % str(s).replace('\\', '\\\\').replace('"', '\\"')
    vec = lambda v: q(' '.join(fmt(x) for x in v))

    def transform(name, pos, quat):
        return ('"DmeTransform"\n{\n"id" "elementid" %s\n"name" "string" %s\n"position" "vector3" %s\n'
                '"orientation" "quaternion" %s\n}' % (q(next(ids)), q(name), vec(pos), vec(quat)))

    joint_ids = [str(next(ids)) for _ in bones]
    order = list(joint_order) if joint_order is not None else list(range(len(bones)))
    slot = {b: k for k, b in enumerate(order)}  # bone -> jointList index (blend indices use it)
    children = {i: [] for i in range(-1, len(bones))}
    for i, b in enumerate(bones):
        children[b['parent']].append(i)

    def joint(i):
        pos, rot = skeleton[i]
        kids = ',\n'.join(joint(c) for c in children[i])
        return ('"DmeJoint"\n{\n"id" "elementid" %s\n"name" "string" %s\n"transform" %s\n'
                '"children" "element_array"\n[\n%s\n]\n}' % (
                    q(joint_ids[i]), q(bones[i]['name']), transform(bones[i]['name'], pos, matrix_quaternion(angle_matrix(rot))), kids))

    positions, weights = [], []                                  # per shared vertex
    loop_pos, normals, uvs, uv2s, colors, tangents, vids = [], [], [], [], [], [], []  # per polygon corner (loop)
    shared = {}
    material_faces = {}                                          # material -> loops, -1 ends a polygon

    def vertex(piece, pos, wts):
        key = (piece, tuple(pos), tuple(wts))
        if key not in shared:
            shared[key] = len(positions)
            positions.append(pos)
            weights.append(wts)
        return shared[key]
    for material, piece, corners, *flags in faces:
        # a fourth element True: vertices no triangle uses (own face set, r2c_vertices_only)
        out = material_faces.setdefault((material, bool(flags and flags[0])), [])
        for pos, nrm, wts, extra in corners:
            out.append(len(loop_pos))
            loop_pos.append(vertex(piece, pos, wts))
            normals.append(nrm)
            uvs.append(extra.get('uv'))
            uv2s.append(extra.get('uv2'))
            colors.append(extra.get('color'))
            tangents.append(extra.get('tangent'))
            vids.append(extra.get('id'))
        out.append(-1)
    for pos, wts, piece in loose:
        vertex(piece, pos, wts)
    joint_count = max([1] + [len(w) for w in weights])
    blend_w, blend_i = [], []
    for w in weights:
        padded = list(w) + [(0, 0.0)] * (joint_count - len(w))
        blend_w += [fmt(x[1]) for x in padded]
        blend_i += [str(slot[x[0]]) for x in padded]
    loops = [q(str(i)) for i in range(len(loop_pos))]
    arr = lambda kind, items: '"%s" [%s]' % (kind, ', '.join(items))
    fmt_names = ['position$0', 'normal$0']
    attrs = ['"position$0" %s\n"position$0Indices" %s' % (arr('vector3_array', [vec(p) for p in positions]),
                                                        arr('int_array', [q(str(i)) for i in loop_pos])),
             '"normal$0" %s\n"normal$0Indices" %s' % (arr('vector3_array', [vec(n) for n in normals]), arr('int_array', loops))]
    for name, kind, values, default in (('texcoord$0', 'vector2_array', uvs, (0, 0)),
                                        ('texcoord$1', 'vector2_array', uv2s, (0, 0)),
                                        ('color$0', 'vector4_array', colors, (1, 1, 1, 1)),
                                        ('tangent$0', 'vector4_array', tangents, (1, 0, 0, 1))):
        if any(v is not None for v in values):
            fmt_names.append(name)
            filled = [v if v is not None else default for v in values]
            attrs.append('"%s" %s\n"%sIndices" %s' % (name, arr(kind, [vec(v) for v in filled]), name, arr('int_array', loops)))
    if any(i is not None for i in vids):
        # r2c: the retail vertex each corner came from, so vertices retail kept apart
        # (identical in every attribute) stay apart; not listed in vertexFormat
        attrs.append('"r2c_vertex_id" %s\n"r2c_vertex_idIndices" %s' % (
            arr('int_array', [q(str(i if i is not None else -1)) for i in vids]), arr('int_array', loops)))
    fmt_names += ['blendweights$0', 'blendindices$0']
    mesh_id, vd_id, dag_id, model_id = (str(next(ids)) for _ in range(4))
    vertex_data = ('"DmeVertexData"\n{\n"id" "elementid" %s\n"name" "string" "bind"\n'
                   '"vertexFormat" "string_array" [%s]\n"flipVCoordinates" "bool" "1"\n"jointCount" "int" "%d"\n'
                   '%s\n"blendweights$0" %s\n"blendindices$0" %s\n}') % (
        q(vd_id), ', '.join(q(n) for n in fmt_names), joint_count, '\n'.join(attrs),
        arr('float_array', [q(w) for w in blend_w]), arr('int_array', [q(i) for i in blend_i]))
    mesh_name = os.path.splitext(os.path.basename(path))[0]
    face_sets = ',\n'.join(
        '"DmeFaceSet"\n{\n"id" "elementid" %s\n"name" "string" %s\n"material" "DmeMaterial"\n{\n"id" "elementid" %s\n'
        '"name" "string" %s\n"mtlName" "string" %s\n}\n"faces" %s%s\n}' % (
            q(next(ids)), q(mat), q(next(ids)), q(mat), q(mat), arr('int_array', [q(str(i)) for i in idx]),
            '\n"r2c_vertices_only" "bool" "1"' if only else '')
        for (mat, only), idx in material_faces.items())
    dag = ('"DmeDag"\n{\n"id" "elementid" %s\n"name" "string" %s\n"shape" "DmeMesh"\n{\n'
           '"id" "elementid" %s\n"name" "string" %s\n"currentState" "element" %s\n'
           '"baseStates" "element_array" ["element" %s]\n"faceSets" "element_array"\n[\n%s\n]\n}\n"transform" %s\n}') % (
        q(dag_id), q(mesh_name), q(mesh_id), q(mesh_name), q(vd_id), q(vd_id), face_sets,
        transform(mesh_name, (0, 0, 0), (0, 0, 0, 1)))
    roots = ',\n'.join([joint(r) for r in children[-1]] + ['"element" %s' % q(dag_id)])
    model = ('"DmeModel"\n{\n"id" "elementid" %s\n"name" "string" %s\n"children" "element_array"\n[\n%s\n]\n'
             '"jointList" "element_array"\n[\n%s\n]\n"transform" %s\n}') % (
        q(model_id), q(mesh_name), roots, ',\n'.join('"element" %s' % q(joint_ids[b]) for b in order),
        transform(mesh_name, (0, 0, 0), (0, 0, 0, 1)))
    with open(path, 'w', newline='\n') as f:
        f.write('<!-- dmx encoding keyvalues2 1 format model 22 -->\n')
        f.write('"DmElement"\n{\n"id" "elementid" %s\n"name" "string" %s\n"skeleton" "element" %s\n'
                '"model" "element" %s\n}\n\n' % (q(next(ids)), q(mesh_name), q(model_id), q(model_id)))
        f.write(model + '\n\n' + dag + '\n\n' + vertex_data + '\n')


def write_dmx_mesh(path, bones, skeleton, polygons, material='phys', pieces=None, loose=(), joint_order=None):
    """write_dmx for a collision mesh: polygons of (pos, normal, [(bone, weight)]) corners,
    one material, and an optional piece id per polygon."""
    write_dmx(path, bones, skeleton,
              [(material, pieces[k] if pieces else 0, [(p, n, w, {}) for p, n, w in poly]) for k, poly in enumerate(polygons)],
              loose, joint_order)


def collision_block_faces(d, h, bones):
    """Map-collision block entries as (vertices, faces); each face lists its vertex
    indices in order around its plane normal. Retail faces are the source mesh's
    polygons, so they carry the grouping that triangle sources lose."""
    count = h['staticCollisionCount']
    if count <= 0:
        count = max([b['collisionIndex'] + b['collisionCount'] for b in bones if b['collisionCount']] or [0])
    if count <= 0 or not h['collisionOffset']:
        return []
    out = []
    for i in range(count):
        base = h['collisionOffset'] + i * 20
        fc, fb, ec, vc, off = struct.unpack_from('<5i', d, base)
        o = base + off
        planes = [struct.unpack_from('<4f', d, o + 16 * k) for k in range(fb)]
        eo = o + 16 * fb
        edges = [struct.unpack_from('<6f4B', d, eo + 28 * k) for k in range(ec)]
        verts = [struct.unpack_from('<3f', d, eo + 28 * ec + 12 * k) for k in range(vc)]
        faces = []
        for s in range(fc):
            loop = sorted({v for e in edges if s in e[6:8] for v in e[8:10]})
            n = planes[s][:3]
            c = [sum(verts[v][k] for v in loop) / len(loop) for k in range(3)]
            a = (1, 0, 0) if abs(n[0]) < 0.9 else (0, 1, 0)
            u = (n[1] * a[2] - n[2] * a[1], n[2] * a[0] - n[0] * a[2], n[0] * a[1] - n[1] * a[0])
            w = (n[1] * u[2] - n[2] * u[1], n[2] * u[0] - n[0] * u[2], n[0] * u[1] - n[1] * u[0])
            rel = lambda v: [verts[v][k] - c[k] for k in range(3)]
            loop.sort(key=lambda v: math.atan2(sum(x * y for x, y in zip(rel(v), w)), sum(x * y for x, y in zip(rel(v), u))))
            faces.append(loop)
        out.append((verts, faces))
    return out


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
    # An overwrite can also leave the range in order; then only the error stream, sized
    # from the original range, shows it. A footstep's error is zero in x/y at its contact
    # frame, which tells an overwritten start (stream begins earlier) from an
    # overwritten end (stream runs longer).
    def planted(r, index):
        pos = decode_ik_error(d, r, r['iStart'] + index)[0]
        return abs(pos[0]) < 0.05 and abs(pos[1]) < 0.05
    in_order_overwrite, start_overwrite = set(), set()
    for i, r in enumerate(rules):
        st, pk, tl, en = frames[i]
        if not (nf > 1 and r['compressedikerrorindex'] and st <= pk <= tl <= en and r['iStart'] == st):
            continue
        n = ik_error_frames(d, r)
        if n == en - st + 1 + (2 if en >= nf else 0):
            continue
        in_order_overwrite.add(i)
        contact = int(round(r['contact'] * (nf - 1)))
        s0 = en - n + (3 if en >= nf else 1)
        if r['type'] == 3 and 0 <= contact - s0 < n and planted(r, contact - s0) and not (
                contact - st < n and planted(r, contact - st)):
            start_overwrite.add(i)
    dot_start = set()
    for i, r in enumerate(rules):
        st, pk, tl, en = frames[i]
        overwritten = i in in_order_overwrite and i not in start_overwrite
        if nf > 1 and (en < tl or overwritten) and r['compressedikerrorindex']:
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
        if nf > 1 and (st > pk or i in start_overwrite):
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

    def vvd_tangent(i):
        return struct.unpack_from('<4f', d, h['vvdOffset'] + vvd['tangentDataStart'] + i * 16)

    # vertex colours / second UVs (VVC), per VVD vertex; such models export DMX meshes
    vvc = mdl53.u(mdl53.VVC_HDR, d, h['vvcOffset']) if h['vvcSize'] else None
    has_uv2 = bool(vvc and h['flags'] & 0x2000000 and vvc['uv2DataStart'])

    def vvc_extra(i):
        o = h['vvcOffset']
        extra = {'color': tuple(c / 255.0 for c in d[o + vvc['colorDataStart'] + 4 * i:o + vvc['colorDataStart'] + 4 * i + 4])}
        if has_uv2:
            u2 = struct.unpack_from('<2f', d, o + vvc['uv2DataStart'] + 8 * i)
            extra['uv2'] = (u2[0], 1 - u2[1])
        return extra

    qc = ['$modelname "%s"' % (mdl53.cstr(d, h['sznameindex']) if h['sznameindex'] else h['name'])]  # full name; the inline one is cut at 64
    if static:
        qc.append('$staticprop')
    for c in cds:
        qc.append('$cdmaterials "%s"' % c)
    surf = mdl53.cstr(d, h['surfacepropindex'])
    qc.append('$surfaceprop "%s"' % surf)
    # $illumposition and $eyeposition take studiomdl's rotated axes: stored (-y, x, z)
    il = h['illumposition']
    qc.append('$illumposition %s %s %s' % (fmt(il[1]), fmt(-il[0]), fmt(il[2])))
    # the source files studiomdl recorded (Maya scenes), ';'-terminated
    if h['sourceFilenameOffset']:
        for src in mdl53.cstr(d, h['sourceFilenameOffset']).split(';'):
            if src:
                qc.append('$sourcefile "%s"' % src)  # QC paths are literal (no escapes)
    if h['contents'] != 1:
        qc.append('$contents %d' % h['contents'])  # exact mask (r2c extension)
    if not h['phySize'] and h['mass'] != 1:
        qc.append('$mass %.9g' % h['mass'])  # header mass of a model without physics (r2c extension)
    if any(h['eyeposition']):
        ey = h['eyeposition']
        qc.append('$eyeposition %s %s %s' % (fmt(ey[1]), fmt(-ey[0]), fmt(ey[2])))
    if any(h['view_bbmin']) or any(h['view_bbmax']):
        qc.append('$cbox %s %s' % (' '.join(fmt(x) for x in h['view_bbmin']), ' '.join(fmt(x) for x in h['view_bbmax'])))
    if h['numAllowedRootLODs']:
        qc.append('$allowrootlods %d' % h['numAllowedRootLODs'])
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
    # bones listed for bone merge (the flag cannot be derived; retail sets it per model)
    if not static:
        for b in bones:
            if b['flags'] & 0x40000:
                qc.append('$bonemerge "%s"' % b['name'])

    vtx = mdl53.parse_vtx(d, h['vtxOffset']) if h['vtxSize'] else None
    sz = mdl53.size(mdl53.BODYPART_FIELDS)
    lod_models = {}  # lod -> list of (old smd name, new smd name)
    num_lods = vtx['hdr']['numLODs'] if vtx else 1
    switch = [0.0] * num_lods
    first_smd = None
    # LOD 0 meshes keep the source file name studiomdl stored (model name) when it is a
    # plain relative .dmx path used once; otherwise they get generated names
    source_names = collections.Counter()
    for bpi in range(h['numbodyparts']):
        bp = mdl53.u(mdl53.BODYPART_FIELDS, d, h['bodypartindex'] + bpi * sz)
        for mi in range(bp['nummodels']):
            md = mdl53.u(mdl53.MODEL_FIELDS, d, h['bodypartindex'] + bpi * sz + bp['modelindex'] + mi * mdl53.size(mdl53.MODEL_FIELDS))
            source_names[md['name'].replace(chr(92), '/').lower()] += 1

    def source_name(name):
        n = name.replace(chr(92), '/')
        parts = n.split('/')
        if (source_names[n.lower()] != 1 or not n.lower().endswith('.dmx') or n.startswith('/') or ':' in n
                or any(p in ('', '.', '..') for p in parts)):
            return None
        return name  # as written; the checks above use the normalised form

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

            def corner(vi):
                pos, nrm, uv, w = vvd_vertex(vi)
                tan = vvd_tangent(vi)
                if static:
                    pos = rotate(ROT_M90, pos)
                    nrm = rotate(ROT_M90, nrm)
                    tan = tuple(rotate(ROT_M90, tan[:3])) + (tan[3],)
                extra = dict(vvc_extra(vi) if vvc else {}, uv=(uv[0], 1 - uv[1]), tangent=tan, id=vi)
                return (pos, nrm, w, extra)
            # mesh vertices no triangle of any LOD uses; each belongs to the coarsest LOD
            # whose vertex count covers it (meshes store their vertices coarse-first)
            used = {}
            for ld in vmodel['lods']:
                for ei, me in enumerate(ld['meshes']):
                    for sg in me['stripgroups']:
                        for k in range(sg['numVerts']):
                            used.setdefault(ei, set()).add(mdl53.u(mdl53.VTX_VERT, d, sg['_off'] + sg['vertOffset'] + k * 9)['origMeshVertID'])
            orphans = {}
            for ei, mesh in enumerate(meshes):
                for i in range(mesh['numvertices']):
                    if i not in used.get(ei, ()):
                        lod_of = max([l for l in range(num_lods) if i < mesh['numLODVertexes'][l]] or [0])
                        orphans.setdefault((ei, lod_of), []).append(vstart + mesh['vertexoffset'] + i)
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
                                tri.append(corner(vstart + mesh['vertexoffset'] + vv['origMeshVertID']))
                            tris.append((mat, tri))
                    if (ei, lod) in orphans:
                        tris.append((mat, [corner(vi) for vi in orphans[(ei, lod)]], True))
                # DMX keeps what SMD can't: vertex colours, second UVs and the stored tangents,
                # which also decide which vertices studiomdl kept apart
                smd_name = (lod == 0 and source_name(md['name'])) or '%s_bp%d_m%d_lod%d.dmx' % (base, bpi, mi, lod)
                os.makedirs(os.path.dirname(os.path.join(outdir, smd_name)), exist_ok=True)
                skel = [[(b['pos'], b['rot']) for b in bones]]
                if static:
                    skel = [[((0, 0, 0), (0, 0, 0)) for b in bones]]
                write_dmx(os.path.join(outdir, smd_name), bones, skel[0], [(t[0], 0) + t[1:] for t in tris])
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
        att = '$attachment "%s" "%s"' % (mdl53.cstr(d, o + a['sznameindex']), bones[a['localbone']]['name'])
        rot = [[a['local'][r * 4 + c] for c in range(3)] for r in range(3)]
        orthonormal = all(abs(sum(rot[k][i] * rot[k][j] for k in range(3)) - (i == j)) < 1e-3 for i in range(3) for j in range(3))
        if not orthonormal:
            # not a rotation (retail has all-zero ones): the stored matrix as is (r2c extension)
            qc.append('%s 0 0 0 matrix %s' % (att, ' '.join('%.9g' % v for v in a['local'])))
            continue
        ang = matrix_angles(mtx)
        qc.append('%s %s %s %s rotate %s %s %s' % (
            att, fmt(mtx[0][3]), fmt(mtx[1][3]), fmt(mtx[2][3]),
            fmt(math.degrees(ang[1])), fmt(math.degrees(ang[2])), fmt(math.degrees(ang[0]))))
    # hitboxes: generated ones are usually empty in retail; a few models have generated boxes
    if h['flags'] & 1:
        if any(mdl53.u(mdl53.HBOXSET_FIELDS, d, h['hitboxsetindex'] + s * 12)['numhitboxes'] for s in range(h['numhitboxsets'])):
            qc.append('$autohitboxes')
    else:
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
        line = '$ikchain "%s" "%s" knee %s %s %s' % (mdl53.cstr(d, o + c['sznameindex']), bones[links[-1]['bone']]['name'], *(fmt(x) for x in knee))
        if abs(c['unk_10'] - 0.707) > 1e-6:
            line += ' unk %s' % fmt(c['unk_10'])
        # retail chains can skip twist bones; spell the links out unless they are the
        # end bone, its parent and its grandparent
        chain = [l['bone'] for l in links]
        if len(chain) == 3 and not (bones[chain[2]]['parent'] == chain[1] and bones[chain[1]]['parent'] == chain[0]):
            line += ' links %s' % ' '.join('"%s"' % bones[b]['name'] for b in chain)
        qc.append(line)
        chain_names.append(mdl53.cstr(d, o + c['sznameindex']))
        chain_ends.append(links[-1]['bone'] if links else -1)
    # source bone transforms
    for i in range(h['numsrcbonetransform']):
        o = h['srcbonetransformindex'] + i * 100
        nm = mdl53.cstr(d, o + struct.unpack_from('<i', d, o)[0])
        xf = struct.unpack_from('<24f', d, o + 4)
        qc.append('$srcbonetransform "%s" %s' % (nm, ' '.join('%.9g' % v for v in xf)))  # exact floats
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
        # one $keyvalues per "mdlkeyvalue { ... }" block: retail can hold several in a row
        blocks = [b for b in re.split(r'(?m)^mdlkeyvalue\s*$', kv) if b.strip()] if kv.startswith('mdlkeyvalue') else [kv]
        for blk in blocks:
            inner = blk.split('{', 1)[1].rsplit('}', 1)[0] if '{' in blk else blk
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
        anim_names.append((name, smd, opts, a))  # retail names, '@seq' included
        qc.append('$animation "%s" "%s" %s' % (anim_names[-1][0], smd, ' '.join(o for o in opts if not o.startswith('//'))))
    # sequences
    node_names = [mdl53.cstr(d, struct.unpack_from('<i', d, h['localnodenameindex'] + 4 * i)[0])
                  for i in range(h['numlocalnodes'])]
    # every node in table order (r2c extension), including any no sequence uses
    for nm in node_names:
        qc.append('$node "%s"' % nm)
    if h['localNodeUnk']:
        qc.append('$localnodeunk %d' % h['localNodeUnk'])
    # an explicit hull ($bbox) when it is not sequence 0's box (or not zero without sequences)
    hull = list(h['hull_min']) + list(h['hull_max'])
    if h['numlocalseq']:
        s0 = mdl53.u(mdl53.SEQDESC_FIELDS, d, h['localseqindex'])
        derived = list(s0['bbmin']) + list(s0['bbmax'])
    else:
        derived = [0.0] * 6
    if hull != derived:
        qc.append('$bbox %s' % ' '.join('%.9g' % v for v in hull))
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
        # transition nodes (1-based indices into the node name table; 0 = none)
        if s['localentrynode'] or s['localexitnode']:
            entry, exit_ = node_names[s['localentrynode'] - 1], node_names[s['localexitnode'] - 1]
            if s['localentrynode'] == s['localexitnode'] and not s['nodeflags'] & 1:
                lines.append('\tnode "%s"' % entry)
            else:
                lines.append('\t%s "%s" "%s"' % ('rtransition' if s['nodeflags'] & 1 else 'transition', entry, exit_))
        # auto layers: retail stores zero ramps; flags beyond the named ones (0x2000) stay raw
        for k in range(s['numautolayers']):
            al = mdl53.u(mdl53.AUTOLAYER_FIELDS, d, o + s['autolayerindex'] + k * mdl53.size(mdl53.AUTOLAYER_FIELDS))
            other = mdl53.cstr(d, h['localseqindex'] + al['iSequence'] * 0xE8 +
                               struct.unpack_from('<i', d, h['localseqindex'] + al['iSequence'] * 0xE8 + 4)[0])
            flags = al['flags']
            ramp = [al[x] for x in ('start', 'peak', 'tail', 'end')]
            if not flags & 0x4000:  # cycle ramps are written in frames
                nf = max(1, anim_names[idxs[0]][3]['numframes'] - 1)
                ramp = [x * nf for x in ramp]
            opts = []
            for bit, word in ((0x80, 'xfade'), (0x40, 'spline'), (0x200, 'noblend'), (0x1000, 'local'), (0x8000, 'realtime')):
                if flags & bit:
                    opts.append(word)
                    flags &= ~bit
            if flags & 0x4000:
                po = h['localposeparamindex'] + al['iPose'] * 0x14
                opts.append('poseparameter "%s"' % mdl53.cstr(d, po + struct.unpack_from('<i', d, po)[0]))
                flags &= ~0x4000
            if flags:
                opts.append('flags %d' % flags)
            lines.append('\tblendlayer "%s" %s %s' % (other, ' '.join(fmt(x) for x in ramp), ' '.join(opts)))
        # activity modifiers (name offset, then a negate byte)
        for k in range(s['numactivitymodifiers']):
            ao = o + s['activitymodifierindex'] + k * 8
            lines.append('\tactivitymodifier "%s"%s' % (mdl53.cstr(d, ao + struct.unpack_from('<i', d, ao)[0]),
                                                       ' negate' if d[ao + 4] else ''))
        if s['ikResetMask']:
            lines.append('\tikresetmask %d' % s['ikResetMask'])
        if s['unk_C4']:
            lines.append('\tunk_c4 %d' % s['unk_C4'])
        # retail's stored box (r2c extension); some are not what the animation gives
        # (offset source animations), so it is kept as is. Delete the line to recompute.
        lines.append('\tbbox %s' % ' '.join('%.9g' % v for v in list(s['bbmin']) + list(s['bbmax'])))
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
        # The map-collision block holds each piece's faces as the source mesh's polygons
        # (in the same solid space as the PHY points); with it the physics mesh is
        # written as DMX polygons, so the compiler rebuilds the same faces.
        block = collision_block_faces(d, h, bones)
        unused_entries = list(range(len(block)))
        polygons = []
        polygon_pieces = []
        loose_points = []
        piece_id = 0
        all_matched = bool(block)
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

                def corner(pt):
                    normal = piece_normal
                    if jointed:
                        pt = transform(world[bone], pt)
                        normal = rotate(world[bone], normal)
                    elif static:
                        pt = rotate(ROT_M90, pt)
                        normal = rotate(ROT_M90, normal)
                    return pt, normal
                for t in L['tris']:
                    tri = []
                    for c in (0, 2, 1):
                        pt, normal = corner(pts[t['edges'][c]['start']])
                        tri.append((pt, normal, (0, 0), [(bone, 1.0)]))
                    tris.append(('phys', tri))
                # the block entry with this piece's vertices supplies its polygons
                piece_pts = [pts[i] for i in sorted({e['start'] for t in L['tris'] for e in t['edges']})]
                match = None
                for k in unused_entries:
                    verts = block[k][0]
                    if len(verts) == len(piece_pts) and all(
                            min(math.dist(v, q2) for q2 in piece_pts) < 1e-3 for v in verts):
                        match = k
                        break
                if match is None:
                    all_matched = False
                    continue
                unused_entries.remove(match)
                verts, faces = block[match]
                # one normal per piece (as for the SMD path): the compiler separates
                # pieces by position and normal, so pieces that touch stay apart
                placed = [corner(v) for v in verts]
                for loop in faces:
                    polygons.append([(placed[vi][0], placed[vi][1], [(bone, 1.0)]) for vi in loop])
                    polygon_pieces.append(piece_id)
                # points on the piece that no face uses (e.g. a cylinder cap's centre)
                used = {vi for loop in faces for vi in loop}
                loose_points.extend((placed[vi][0], [(bone, 1.0)], piece_id) for vi in range(len(verts)) if vi not in used)
                piece_id += 1
        skel = [[(b['pos'], b['rot']) for b in bones]] if not static else [[((0, 0, 0), (0, 0, 0)) for b in bones]]
        if all_matched:
            physics_file = base + '_physics.dmx'
            # joints listed in retail's solid order, which the compiler builds solids in
            by_name = {b['name'].lower(): i for i, b in enumerate(bones)}
            order = [by_name[n.lower()] for n in solid_names if n.lower() in by_name]
            order, wanted = [], order + list(range(len(bones)))
            for i in wanted:  # parents first: a bone's missing ancestors go just before it
                chain = []
                while i != -1 and i not in order and i not in chain:
                    chain.append(i)
                    i = bones[i]['parent']
                order += reversed(chain)
            write_dmx_mesh(os.path.join(outdir, physics_file), bones, skel[0], polygons, pieces=polygon_pieces,
                           loose=loose_points, joint_order=order)
        else:
            physics_file = base + '_physics.smd'
            write_smd(os.path.join(outdir, physics_file), bones, skel, tris)
        mass = re.search(r'"totalmass"\s+"([^"]+)"', text)
        cmd = '$collisionjoints' if jointed else '$collisionmodel'
        lines = ['%s "%s"\n{' % (cmd, physics_file)]
        if mass:
            lines.append('\t$mass %s' % mass.group(1))
        if '"concave" "1"' in text:
            lines.append('\t$concave')
        # retail models built without the map-collision block (collisionOffset 0)
        if not h['collisionOffset']:
            lines.append('\t$nomapcollision')
            if h['staticCollisionCount']:
                lines.append('\t$staticcollisioncount %d' % h['staticCollisionCount'])  # retail -1 (r2c)
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
