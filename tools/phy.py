"""Reader for Source/Titanfall 2 PHY data (IVP compact surfaces + keyvalue text).

Used to study retail collision and to validate compiler output.
"""
import struct
import sys

IVP_SCALE = 0.0254  # inches -> metres


def ivp_to_hl(p):
    # vphysics ConvertPositionToIVP: ivp = (x, -z, y) * 0.0254
    x, y, z = p
    return (x / IVP_SCALE, z / IVP_SCALE, -y / IVP_SCALE)


def parse_ledge(d, lo):
    c_point_offset, client = struct.unpack_from('<ii', d, lo)
    bits = struct.unpack_from('<I', d, lo + 8)[0]
    ntri, future = struct.unpack_from('<hh', d, lo + 12)
    led = {
        'off': lo, 'c_point_offset': c_point_offset, 'client_or_node': client,
        'has_children': bits & 3, 'is_compact': (bits >> 2) & 3, 'dummy': (bits >> 4) & 0xF,
        'size_div_16': bits >> 8, 'ntri': ntri, 'future': future, 'tris': [],
    }
    for t in range(ntri):
        to = lo + 16 + t * 16
        tb = struct.unpack_from('<I', d, to)[0]
        tri = {'tri_index': tb & 0xFFF, 'pierce_index': (tb >> 12) & 0xFFF, 'material': (tb >> 24) & 0x7F,
               'is_virtual': tb >> 31, 'edges': []}
        for e in range(3):
            eb = struct.unpack_from('<I', d, to + 4 + e * 4)[0]
            opp = (eb >> 16) & 0x7FFF
            if opp & 0x4000:
                opp -= 0x8000
            tri['edges'].append({'start': eb & 0xFFFF, 'opposite': opp, 'virtual': eb >> 31})
        led['tris'].append(tri)
    return led


def parse_surface(d, so):
    """so = offset of compactsurfaceheader_t (the 'size' field)."""
    size, vphys, ver, mtype, surfsize = struct.unpack_from('<iihhi', d, so)
    drag = struct.unpack_from('<3f', d, so + 16)
    axismap = struct.unpack_from('<i', d, so + 28)[0]
    s = {'size': size, 'vphysicsID': hex(vphys), 'version': hex(ver), 'modelType': mtype, 'surfaceSize': surfsize,
         'drag': drag, 'axisMapSize': axismap}
    lo = so + 32  # legacy surface header
    mc = struct.unpack_from('<3f', d, lo)
    ri = struct.unpack_from('<3f', d, lo + 12)
    ulr = struct.unpack_from('<f', d, lo + 24)[0]
    bits = struct.unpack_from('<I', d, lo + 28)[0]
    root = struct.unpack_from('<i', d, lo + 32)[0]
    dummy = struct.unpack_from('<3i', d, lo + 36)
    s.update({'mass_center': mc, 'rot_inertia': ri, 'upper_limit_radius': ulr, 'max_dev': bits & 0xFF,
              'byte_size': bits >> 8, 'ledgetree_root': root, 'dummy': [hex(x) for x in dummy], 'legacy_off': lo})
    # walk the ledge tree
    nodes = []
    ledges = {}

    def walk(no, depth):
        r, cl = struct.unpack_from('<ii', d, no)
        c = struct.unpack_from('<4f', d, no + 8)
        box = struct.unpack_from('<4B', d, no + 24)
        node = {'off': no - lo, 'right': r, 'ledge': cl, 'center': c[:3], 'radius': c[3], 'box': box[:3],
                'free': box[3], 'depth': depth}
        nodes.append(node)
        if cl:
            ledges[no + cl] = parse_ledge(d, no + cl)
        if r:
            walk(no + 28, depth + 1)
            walk(no + r, depth + 1)

    walk(lo + root, 0)
    s['nodes'] = nodes
    s['ledges'] = [ledges[k] for k in sorted(ledges)]
    # points: from the first ledge's point offset to the ledge tree
    if s['ledges']:
        l0 = s['ledges'][0]
        pstart = l0['off'] + l0['c_point_offset']
        maxidx = max(e['start'] for L in s['ledges'] for t in L['tris'] for e in t['edges'])
        s['points_off'] = pstart - lo
        s['points'] = [struct.unpack_from('<4f', d, pstart + 16 * i) for i in range(maxidx + 1)]
    return s


def parse_phy(d, base=0):
    size, pid, count, ck = struct.unpack_from('<4i', d, base)
    out = {'hdr_size': size, 'id': pid, 'solidCount': count, 'checkSum': ck, 'solids': []}
    so = base + size
    for i in range(count):
        s = parse_surface(d, so)
        s['offset'] = so - base
        out['solids'].append(s)
        so += s['size'] + 4
    end = d.find(b'\0', so)
    out['text'] = d[so:end].decode('latin1')
    return out


if __name__ == '__main__':
    sys.path.insert(0, __file__.rsplit('\\', 1)[0].rsplit('/', 1)[0])
    import mdl53
    d = open(sys.argv[1], 'rb').read()
    if d[:4] == b'IDST':
        h = mdl53.Model(d).hdr
        p = parse_phy(d[h['phyOffset']:h['phyOffset'] + h['phySize']])
    else:
        p = parse_phy(d)
    for s in p['solids']:
        print({k: v for k, v in s.items() if k not in ('nodes', 'ledges', 'points')})
        print(' nodes', len(s['nodes']), 'ledges', len(s['ledges']), 'points', len(s.get('points', [])))
        for n in s['nodes'][:6]:
            print('  node', n)
        for L in s['ledges'][:3]:
            print('  ledge', {k: v for k, v in L.items() if k != 'tris'})
            for t in L['tris'][:4]:
                print('    tri', t)
        for pt in s.get('points', [])[:4]:
            print('  point', pt)
    print(p['text'][:600])
