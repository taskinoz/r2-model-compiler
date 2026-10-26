"""Independent structural and full-frame validation of compiled Titanfall 2 models.

This validates decoded output, not compiler-internal objects. It does not equate
binary identity with valid geometry. --against evaluates every stored animation
frame and rendered material slot for every skin/LOD against the original model.
"""
import argparse
import collections
import hashlib
import itertools
import json
import math
import re
from pathlib import Path
import struct
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))

import decompile53 as dc
import mdl53
import phy as phylib


POSITION_TOLERANCE = .25  # inches; the existing corpus comparison's threshold
SCALE_TOLERANCE = .01
ROTATION_TOLERANCE = .01  # degrees, independent of translation error
PHYSICS_TOLERANCE = .01  # inches, PHY geometry and map signed-distance functions
VERTEX_POSITION_TOLERANCE = .001  # inches, used render vertices
VERTEX_ATTRIBUTE_TOLERANCE = 1e-5  # normals, tangents, UVs and weights


def require(condition, message):
    if not condition:
        raise ValueError(message)


def span(data, offset, count, stride, label):
    require(offset >= 0 and count >= 0 and offset + count * stride <= len(data),
            label + ': data outside file')


def f32(value):
    return struct.unpack('<f', struct.pack('<f', value))[0]


def quantize(position, low, high):
    out = []
    for x, lo, hi in zip(position, low, high):
        extent = f32(hi-lo)
        q = f32(f32(f32(x-lo)/extent)*65535) if extent > 0 else 0
        out.append(min(65535, max(0, math.floor(q+.5))))
    return tuple(out)


def triangle_key(vertices):
    a,b,c = vertices
    ab = tuple(b[i]-a[i] for i in range(3))
    ac = tuple(c[i]-a[i] for i in range(3))
    cross = (ab[1]*ac[2]-ab[2]*ac[1], ab[2]*ac[0]-ab[0]*ac[2], ab[0]*ac[1]-ab[1]*ac[0])
    if not any(cross):  # includes the format's all-zero padding triangles
        return None
    return min((a,b,c), (b,c,a), (c,a,b))


def geometry(data, header):
    """Vertices and mesh/LOD triangle lists, in stored VVD order."""
    if not header['vvdSize']:
        return [], []
    vo = header['vvdOffset']
    vv = mdl53.u(mdl53.VVD_HDR, data, vo)
    require(vv['id'] == 0x56534449 and vv['version'] == 4, 'invalid VVD header')
    require(1 <= vv['numLODs'] <= 8, 'invalid LOD count')
    count = vv['numLODVertexes'][0]
    span(data, vo+vv['vertexDataStart'], count, 48, 'VVD vertices')
    span(data, vo+vv['tangentDataStart'], count, 16, 'VVD tangents')
    vc=mdl53.u(mdl53.VVC_HDR,data,header['vvcOffset']) if header['vvcSize'] else None
    if vc:
        span(data,header['vvcOffset']+vc['colorDataStart'],count,4,'VVC colors')
        if header['flags']&0x2000000:span(data,header['vvcOffset']+vc['uv2DataStart'],count,8,'VVC UV2')
    vertices = []
    for i in range(count):
        at = vo+vv['vertexDataStart']+48*i
        weight = struct.unpack_from('<3f', data, at)
        bones = data[at+12:at+15]
        n = data[at+15]
        p = struct.unpack_from('<3f', data, at+16)
        require(1 <= n <= 3, 'invalid number of vertex weights')
        attributes=struct.unpack_from('<8f',data,at+16)
        tangent=struct.unpack_from('<4f',data,vo+vv['tangentDataStart']+i*16)
        require(all(math.isfinite(x) for x in (*weight,*attributes,*tangent)), 'nonfinite vertex')
        require(all(weight[k] >= 0 and bones[k] < header['numbones'] for k in range(n)), 'invalid vertex weight/bone')
        require(abs(sum(weight[:n])-1) < .0001, 'vertex weights do not sum to one')
        color=tuple(data[header['vvcOffset']+vc['colorDataStart']+i*4:header['vvcOffset']+vc['colorDataStart']+i*4+4]) if vc else (255,255,255,255)
        uv2=struct.unpack_from('<2f',data,header['vvcOffset']+vc['uv2DataStart']+i*8) if vc and header['flags']&0x2000000 else (0.,0.)
        require(all(math.isfinite(x) for x in uv2),'nonfinite UV2')
        vertices.append((p, tuple(zip(bones[:n], weight[:n])),(attributes[3:]+tangent+uv2,color)))
    tree = mdl53.parse_vtx(data, header['vtxOffset'])
    meshes = []
    for bi,bp in enumerate(tree['bodyparts']):
        bpo = header['bodypartindex'] + bi*mdl53.size(mdl53.BODYPART_FIELDS)
        stored_bp = mdl53.u(mdl53.BODYPART_FIELDS,data,bpo)
        for mi,model in enumerate(bp['models']):
            mo = bpo+stored_bp['modelindex']+mi*mdl53.size(mdl53.MODEL_FIELDS)
            md = mdl53.u(mdl53.MODEL_FIELDS,data,mo)
            for li,lod in enumerate(model['lods']):
                for ei,mesh in enumerate(lod['meshes']):
                    me = mdl53.u(mdl53.MESH_FIELDS,data,mo+md['meshindex']+ei*mdl53.size(mdl53.MESH_FIELDS))
                    base = md['vertexindex']//48+me['vertexoffset']
                    require(0 <= me['material'] < header['numskinref'], 'mesh skin slot out of range')
                    triangles = []
                    for sg in mesh['stripgroups']:
                        go=sg['_off']
                        refs=[mdl53.u(mdl53.VTX_VERT,data,go+sg['vertOffset']+i*9)['origMeshVertID'] for i in range(sg['numVerts'])]
                        require(all(0 <= v < me['numvertices'] and base+v < count for v in refs), 'VTX vertex out of range')
                        idx=struct.unpack_from('<%dH'%sg['numIndices'],data,go+sg['indexOffset'])
                        require(len(idx)%3 == 0 and all(v < len(refs) for v in idx), 'VTX triangle index out of range')
                        triangles.extend(tuple(base+refs[v] for v in idx[t:t+3]) for t in range(0,len(idx),3))
                    meshes.append({'key':(bi,mi,li,ei),'material':me['material'],'triangles':triangles})
    return vertices, meshes


def validate_tree(data, header, vertices, meshes, require_render_collision=True):
    offset=header['perTriAABBIndex']
    if not offset:
        require(not any(header[k] for k in ('perTriAABBNodeCount','perTriAABBLeafCount','perTriAABBVertCount')),
                'missing per-triangle header with nonzero counts')
        return {'triangles':0,'signature':hashlib.sha256(b'empty').hexdigest(),'matches_render_geometry':True}
    span(data,offset,1,60,'per-triangle header')
    ph=mdl53.u(mdl53.PERTRI_HDR,data,offset)
    nn,nl,nv=(header[k] for k in ('perTriAABBNodeCount','perTriAABBLeafCount','perTriAABBVertCount'))
    require(ph['version'] in (0,2),'unsupported per-triangle version')
    if ph['version']==0:
        require(nn==nl==nv==0,'version-zero tree contains nodes')
        return {'triangles':0,'signature':hashlib.sha256(b'empty').hexdigest(),'matches_render_geometry':True}
    nodes_at=offset+60; leaves_at=nodes_at+nn*20; verts_at=leaves_at+nl*112
    span(data,nodes_at,nn,20,'per-triangle nodes')
    span(data,leaves_at,nl,112,'per-triangle leaves')
    span(data,verts_at,nv,6,'per-triangle vertices')
    require(all(math.isfinite(v) for v in ph['bbmin']+ph['bbmax']),'nonfinite tree bounds')
    qverts=[struct.unpack_from('<3H',data,verts_at+i*6) for i in range(nv)]
    seen_nodes=set(); seen_leaves=set(); active=set()
    def walk(link):
        if link<0:
            leaf=~link
            require(0 <= leaf < nl,'tree leaf reference out of range')
            require(leaf not in seen_leaves,'tree leaf has multiple parents')
            seen_leaves.add(leaf)
            return struct.unpack_from('<6H',data,leaves_at+112*leaf)
        require(link < nn,'tree node reference out of range')
        require(link not in active and link not in seen_nodes,'tree cycle or repeated node')
        active.add(link); seen_nodes.add(link)
        bounds=struct.unpack_from('<6H',data,nodes_at+20*link)
        require(all(bounds[k] <= bounds[k+3] for k in range(3)),'inverted node bounds')
        children=struct.unpack_from('<2i',data,nodes_at+20*link+12)
        for child in children:
            child_bounds=walk(child)
            require(all(bounds[k] <= child_bounds[k] <= child_bounds[k+3] <= bounds[k+3] for k in range(3)),
                    'node bounds do not enclose child')
        active.remove(link)
        return bounds
    if nn: walk(0)
    elif nl: walk(~0)
    require(len(seen_nodes)==nn and len(seen_leaves)==nl,'unreachable tree data')
    actual=collections.Counter()
    surfaces=collections.Counter()
    for i in range(nl):
        at=leaves_at+112*i
        bounds=struct.unpack_from('<6H',data,at)
        require(all(bounds[k] <= bounds[k+3] for k in range(3)),'inverted leaf bounds')
        base=struct.unpack_from('<i',data,at+12)[0]
        for j in range(12):
            tri=struct.unpack_from('<4H',data,at+16+8*j)
            require(all(0 <= base+v < nv for v in tri[:3]),'leaf vertex out of range')
            points=tuple(qverts[base+v] for v in tri[:3])
            key=triangle_key(points)
            if key is not None:
                require(all(bounds[k] <= p[k] <= bounds[k+3] for p in points for k in range(3)), 'leaf bounds do not enclose triangle')
                actual[key]+=1
                surfaces[(key,tri[3])]+=1
    expected=collections.Counter()
    for mesh in meshes:
        if mesh['key'][1:3] != (0,0): continue
        for tri in mesh['triangles']:
            points=tuple(quantize(vertices[v][0],ph['bbmin'],ph['bbmax']) for v in tri)
            key=triangle_key(points)
            if key is not None: expected[key]+=1
    matches=actual==expected
    if require_render_collision:
        require(matches,'per-triangle collision does not cover the rendered LOD0 triangles')
    digest=hashlib.sha256(b'empty' if not actual else struct.pack('<6f',*(ph['bbmin']+ph['bbmax'])))
    for (points,surface),count in sorted(surfaces.items()):
        digest.update(struct.pack('<10HI',*(v for p in points for v in p),surface,count))
    return {'triangles':sum(actual.values()),'signature':digest.hexdigest(),'matches_render_geometry':matches}


def validate(path, require_render_collision=True):
    data=Path(path).read_bytes(); model=mdl53.Model(data); h=model.hdr; bones=model.bones()
    require(h['version']==53 and h['length']==len(data),'invalid MDL version/length')
    require(1 <= h['numbones'] <= 256,'invalid bone count')
    for i,b in enumerate(bones):
        require(-1 <= b['parent'] < i,'bone parent out of order or cyclic')
        require(all(math.isfinite(x) for key in ('pos','rot','scale','poseToBone') for x in b[key]),'nonfinite bone transform')
        require(all(abs(x)>1e-20 for x in b['scale']),'singular bind scale')
    bind=world_pose(bones,[(b['pos'],dc.angle_matrix(b['rot']),b['scale']) for b in bones])
    for i,b in enumerate(bones):
        inverse=[b['poseToBone'][r:r+4] for r in (0,4,8)]
        product=dc.concat(bind[i],inverse)
        for r in range(3):
            for c in range(4):
                magnitude=sum(abs(bind[i][r][k]*inverse[k][c]) for k in range(3))
                if c==3:magnitude+=abs(bind[i][r][3])
                # Bound accumulated float32 matrix/angle rounding, including large origins.
                require(abs(product[r][c]-(1 if r==c else 0)) <= 4e-5+2e-6*magnitude,
                        'inverse bind matrix disagrees with local bone transforms')
    for name in ('vtx','vvd','vvc','phy'):
        span(data,h[name+'Offset'],h[name+'Size'],1,name)
    vertices,meshes=geometry(data,h)
    for mesh in meshes:
        usage=0x400 << mesh['key'][2]
        for v in {v for t in mesh['triangles'] for v in t}:
            for bone,weight in vertices[v][1]:
                if weight>0:require(bones[bone]['flags'] & usage,'vertex bone missing its LOD usage flag')
    collision=validate_tree(data,h,vertices,meshes,require_render_collision)
    physical=physics_data(path); physical_pieces=0
    if physical:
        require(physical['solidCount']>0,'empty PHY block')
        for solid in physical['solids']:
            require(all(math.isfinite(x) for key in ('mass_center','rot_inertia','drag') for x in solid[key]),'nonfinite PHY properties')
            require(math.isfinite(solid['upper_limit_radius']) and solid['upper_limit_radius']>=0,'invalid PHY radius')
            physical_pieces+=len(convex_pieces(solid))
    return {'vertices':len(vertices),'meshes':len(meshes),'collision_triangles':collision['triangles'],
            'collision_signature':collision['signature'],'collision_matches_render_mesh':collision['matches_render_geometry'],
            'bones':len(bones),'physics_convex_pieces':physical_pieces}


def world_pose(bones, pose):
    out=[]
    for b,(position,rotation,scale) in zip(bones,pose):
        mat=[[rotation[r][c]*scale[c] for c in range(3)]+[position[r]] for r in range(3)]
        out.append(dc.concat(out[b['parent']],mat) if b['parent']>=0 else mat)
    return out


def rotation_error(a,b):
    # Relative basis Frobenius norm = 2 sqrt(2) sin(angle/2).
    difference=math.sqrt(sum((a[r][c]-b[r][c])**2 for r in range(3) for c in range(3)))
    return math.degrees(2*math.asin(min(1,difference/(2*math.sqrt(2)))))


def accelerated_pose_metrics(ba,bb,fa,fb,fma,fmb,delta):
    """Optional analysis accelerator; the tool also works with only the stdlib."""
    try:
        import numpy as np
    except ImportError:
        return None
    def arrays(frames):
        p=np.asarray([[v[0] for v in frame] for frame in frames],dtype=float)
        r=np.asarray([[v[1] for v in frame] for frame in frames],dtype=float)[...,:3]
        s=np.asarray([[v[2] for v in frame] for frame in frames],dtype=float)
        require(np.isfinite(p).all() and np.isfinite(r).all() and np.isfinite(s).all(),'nonfinite decoded animation')
        return p,r,s
    pa,ra,sa=arrays(fa); pb,rb,sb=arrays(fb)
    def positions(bones,p,r,s,fm):
        world=np.empty((*p.shape[:2],3,4),dtype=float)
        for i,bone in enumerate(bones):
            rot=r[:,i]*s[:,i,None,:]
            parent=bone['parent']
            if parent<0:
                world[:,i,:,:3]=rot;world[:,i,:,3]=p[:,i]
            else:
                basis=world[:,parent,:,:3]
                world[:,i,:,:3]=np.einsum('fij,fjk->fik',basis,rot)
                world[:,i,:,3]=np.einsum('fij,fj->fi',basis,p[:,i])+world[:,parent,:,3]
        out=world[...,3]
        if fm is not None:
            fm=np.asarray(fm,dtype=float); angle=np.radians(fm[:,3])
            c,s=np.cos(angle)[:,None],np.sin(angle)[:,None]
            x,y=out[...,0].copy(),out[...,1].copy()
            out[...,0]=c*x-s*y+fm[:,None,0]
            out[...,1]=s*x+c*y+fm[:,None,1]
            out[...,2]+=fm[:,None,2]
        require(np.isfinite(out).all(),'nonfinite world animation')
        return out
    rotation=np.sqrt(np.sum((ra-rb)**2,axis=(-2,-1)))
    errors={'local_position':np.linalg.norm(pa-pb,axis=2),
            'rotation_degrees':np.degrees(2*np.arcsin(np.minimum(1,rotation/(2*math.sqrt(2))))),
            'scale':np.max(np.abs(sa-sb),axis=2)}
    if not delta:errors['position']=np.linalg.norm(positions(ba,pa,ra,sa,fma)-positions(bb,pb,rb,sb,fmb),axis=2)
    result={}
    for name,array in errors.items():
        index=np.unravel_index(np.argmax(array),array.shape)
        result[name]=(float(array[index]),int(index[0]),int(index[1]))
    return result


def compare_animations(original, rebuilt, accelerated=True):
    da,db=Path(original).read_bytes(),Path(rebuilt).read_bytes()
    ma,mb=mdl53.Model(da),mdl53.Model(db); ba,bb=ma.bones(),mb.bones()
    require([(b['name'],b['parent']) for b in ba]==[(b['name'],b['parent']) for b in bb],'bone hierarchy differs')
    require(ma.hdr['numlocalanim']==mb.hdr['numlocalanim'],'animation count differs')
    worst={'position':0.,'local_position':0.,'rotation_degrees':0.,'scale':0.}
    where={}; frames=0; bone_frames=0; ik_frames=0; backend='python'
    for ai in range(ma.hdr['numlocalanim']):
        aa,name,fa=dc.decode_anim(da,ma,ba,ai,with_scale=True)
        ab,other,fb=dc.decode_anim(db,mb,bb,ai,with_scale=True)
        require(name==other and aa['numframes']==ab['numframes'] and aa['flags']==ab['flags'],'animation identity/flags differ')
        nf=aa['numframes']; frames+=nf; delta=bool(aa['flags']&4)
        fma=dc.decode_frame_movement(da,ma,ai,range(nf))
        fmb=dc.decode_frame_movement(db,mb,ai,range(nf))
        require((fma is None)==(fmb is None),'frame movement presence differs')
        if fma is not None: require(fma==fmb,'stored frame movement samples differ')
        rules_a=dc.decode_ik_rules(da,ma.hdr,ai);rules_b=dc.decode_ik_rules(db,mb.hdr,ai)
        require(len(rules_a)==len(rules_b),'IK rule count differs')
        for a,b in zip(rules_a,rules_b):
            for key in ('index','type','chain','bone','slot','attachment','iStart'):
                require(a[key]==b[key],'IK '+key+' differs')
            for key in ('start','peak','tail','end','contact','height','radius','floor','endHeight'):
                require(abs(a[key]-b[key])<=1e-5,'IK '+key+' differs')
            require(math.dist(a['pos'],b['pos'])<=POSITION_TOLERANCE,'IK target position differs')
            require(rotation_error(dc.quat_matrix(a['q']),dc.quat_matrix(b['q']))<=ROTATION_TOLERANCE,'IK target rotation differs')
            require(bool(a['compressedikerrorindex'])==bool(b['compressedikerrorindex']),'IK error presence differs')
            if a['compressedikerrorindex']:
                na,nb=dc.ik_error_frames(da,a),dc.ik_error_frames(db,b)
                require(na==nb,'IK error stream length differs')
                ik_frames+=na
                for k in range(na):
                    ea,eb=dc.decode_ik_error(da,a,a['iStart']+k),dc.decode_ik_error(db,b,b['iStart']+k)
                    require(math.dist(ea[0],eb[0])<=POSITION_TOLERANCE,'IK error position differs')
                    require(rotation_error(dc.angle_matrix(ea[1]),dc.angle_matrix(eb[1]))<=ROTATION_TOLERANCE,'IK error rotation differs')
        require(aa['nummovements']==ab['nummovements'],'legacy movement count differs')
        for k in range(aa['nummovements']):
            oa=ma.hdr['localanimindex']+ai*0x5c+aa['movementindex']+k*44
            ob=mb.hdr['localanimindex']+ai*0x5c+ab['movementindex']+k*44
            require(struct.unpack_from('<2i9f',da,oa)==struct.unpack_from('<2i9f',db,ob),'legacy movement values differ')
        metrics=accelerated_pose_metrics(ba,bb,fa,fb,fma,fmb,delta) if accelerated and nf else None
        if metrics is not None:
            backend='numpy';bone_frames+=nf*len(ba)
            for key,(value,frame,bone) in metrics.items():
                if value>worst[key]:worst[key]=value;where[key]=(name,frame,bone)
            continue
        def measure(key,value,frame,bone):
            if value>worst[key]: worst[key]=value; where[key]=(name,frame,bone)
        for frame,(pa,pb) in enumerate(zip(fa,fb)):
            bone_frames+=len(ba)
            wa,wb=world_pose(ba,pa),world_pose(bb,pb)
            if not delta:
                for mats,fm in ((wa,fma),(wb,fmb)):
                    if fm is not None:
                        x,y,z,yaw=fm[frame]
                        root=dc.angle_matrix((0,0,math.radians(yaw)))
                        for r,v in enumerate((x,y,z)): root[r][3]=v
                        mats[:]=[dc.concat(root,m) for m in mats]
            for i,(a,b) in enumerate(zip(pa,pb)):
                require(all(math.isfinite(x) for pose in (a,b) for x in (*pose[0],*pose[2],*(v for row in pose[1] for v in row))),
                        'nonfinite decoded animation')
                measure('local_position',math.dist(a[0],b[0]),frame,i)
                measure('rotation_degrees',rotation_error(a[1],b[1]),frame,i)
                measure('scale',max(abs(x-y) for x,y in zip(a[2],b[2])),frame,i)
                if not delta: measure('position',math.dist(tuple(r[3] for r in wa[i]),tuple(r[3] for r in wb[i])),frame,i)
    failures=[]
    for key,tolerance in (('position',POSITION_TOLERANCE),('local_position',POSITION_TOLERANCE),
                          ('rotation_degrees',ROTATION_TOLERANCE),('scale',SCALE_TOLERANCE)):
        if worst[key]>tolerance: failures.append('%s %.9g > %.9g at %s'%(key,worst[key],tolerance,where[key]))
    return {'frames':frames,'bone_frames':bone_frames,'ik_error_frames':ik_frames,'backend':backend,
            'maximum':worst,'where':where,'issues':failures}


def material_slots(path):
    data=Path(path).read_bytes(); h=mdl53.Model(data).hdr
    _,meshes=geometry(data,h)
    textures=[]
    for i in range(h['numtextures']):
        at=h['textureindex']+i*0x2c
        name=mdl53.cstr(data,at+struct.unpack_from('<i',data,at)[0])
        textures.append(name.lower().replace(chr(92),'/'))
    skins=[struct.unpack_from('<%dh'%h['numskinref'],data,h['skinindex']+2*h['numskinref']*i) for i in range(h['numskinfamilies'])]
    return [(me['key'],tuple(textures[skin[me['material']]] for skin in skins)) for me in meshes]


def assign_pairs(choices):
    """Find a complete bipartite matching without recursion or greedy choices."""
    assigned={}
    for start in range(len(choices)):
        pending=[start];parent={start:None};via={};seen=set();found=False
        while pending and not found:
            current=pending.pop()
            for target in choices[current]:
                if target in seen:continue
                seen.add(target)
                if target not in assigned:
                    while True:
                        assigned[target]=current
                        if current==start:break
                        target,current=via[current],parent[current]
                    found=True;break
                child=assigned[target]
                if child not in parent:
                    parent[child]=current;via[child]=target;pending.append(child)
        if not found:return None
    return assigned


def compare_render(original,rebuilt):
    """Match oriented triangles directly; tolerant vertex equality is nontransitive."""
    def read(path):
        data=Path(path).read_bytes();return geometry(data,mdl53.Model(data).hdr)
    va,ma=read(original);vb,mb=read(rebuilt)
    require([m['key'] for m in ma]==[m['key'] for m in mb],'render mesh/LOD layout differs')
    wa=[{i:w for i,w in vertex[1] if w>0} for vertex in va]
    wb=[{i:w for i,w in vertex[1] if w>0} for vertex in vb]
    maximum={'position':0.,'attributes':0.,'weights':0.};total=0
    def cell(vertices,triangle):
        return tuple(math.floor(sum(vertices[i][0][k] for i in triangle)/3/VERTEX_POSITION_TOLERANCE) for k in range(3))
    def corner(ai,bi):
        a,b=va[ai],vb[bi]
        if a[2][1]!=b[2][1]:return None
        p=math.dist(a[0],b[0])
        if p>VERTEX_POSITION_TOLERANCE:return None
        attr=max(abs(x-y) for x,y in zip(a[2][0],b[2][0]))
        if attr>VERTEX_ATTRIBUTE_TOLERANCE:return None
        weight=max((abs(wa[ai].get(i,0)-wb[bi].get(i,0)) for i in wa[ai].keys()|wb[bi].keys()),default=0.)
        return (p,attr,weight) if weight<=VERTEX_ATTRIBUTE_TOLERANCE else None
    for a,b in zip(ma,mb):
        ta,tb=a['triangles'],b['triangles'];require(len(ta)==len(tb),'render triangle count differs');total+=len(ta)
        buckets=collections.defaultdict(list)
        for i,t in enumerate(ta):buckets[cell(va,t)].append(i)
        choices=[];costs={}
        for bi,t in enumerate(tb):
            candidates=[];key=cell(vb,t)
            for delta in itertools.product((-1,0,1),repeat=3):
                for ai in buckets.get(tuple(key[k]+delta[k] for k in range(3)),()):
                    errors=[]
                    for turn in range(3):
                        values=[corner(ta[ai][(k+turn)%3],t[k]) for k in range(3)]
                        if all(value is not None for value in values):
                            errors.append(tuple(max(value[c] for value in values) for c in range(3)))
                    if errors:candidates.append(ai);costs[bi,ai]=min(errors)
            require(bool(candidates),'oriented render triangle %d has no matching position/normal/tangent/UV/color/weights in %s'%(bi,a['key']))
            choices.append(candidates)
        matched=assign_pairs(choices)
        require(matched is not None,'oriented render triangle multiplicity differs in '+str(a['key']))
        for ai,bi in matched.items():
            for key,error in zip(maximum,costs[bi,ai]):maximum[key]=max(maximum[key],error)
    return {'triangles':total,'used_vertices':len({i for m in ma for t in m['triangles'] for i in t}),'maximum_error':maximum}


def physics_data(path):
    data=Path(path).read_bytes(); h=mdl53.Model(data).hdr
    if not h['phySize']: return None
    return phylib.parse_phy(data[h['phyOffset']:h['phyOffset']+h['phySize']])


def physics_blocks(text):
    """Runtime properties, with float text normalized as the engine's float32."""
    blocks=collections.defaultdict(list)
    for kind,body in re.findall(r'(\w+)\s*\{([^{}]*)\}',text):
        if kind=='editparams':continue  # editor history, not simulation data
        pairs=[]
        for key,value in re.findall(r'"([^"]+)"\s+"([^"]*)"',body):
            if key=='massbias':continue  # editor's derivation; the actual mass is compared
            if key in ('name','parent','surfaceprop','model','rootname'):
                value=value.lower().replace(chr(92),'/')
            elif key=='collisionpair': value=tuple(sorted(map(int,value.split(','))))
            else:
                try:value=tuple(f32(float(v)) for v in value.split())
                except ValueError: pass
            pairs.append((key,value))
        # Duplicate/reversed disabled pairs have the same physical meaning.
        if kind=='collisionrules':pairs=list(set(pairs))
        if kind=='collisionrules' and not pairs:continue  # no exclusions is the default
        blocks[kind].append(tuple(sorted(pairs,key=lambda p:p[0])))
    return {kind:sorted(values,key=repr) for kind,values in blocks.items()}


def convex_pieces(surface):
    points=[phylib.ivp_to_hl(p[:3]) for p in surface['points']]
    pieces=[]
    for ledge in surface['ledges']:
        if ledge['has_children']:continue
        triangles=ledge['tris']; ids=sorted({e['start'] for tri in triangles for e in tri['edges']})
        require(all(0 <= i < len(points) for i in ids),'PHY point outside pool')
        verts=[points[i] for i in ids]
        require(bool(verts),'empty PHY convex piece')
        center=tuple(sum(p[k] for p in verts)/len(verts) for k in range(3))
        planes=[]
        for ti,tri in enumerate(triangles):
            require(tri['pierce_index'] < len(triangles),'PHY pierce index outside ledge')
            for ei,edge in enumerate(tri['edges']):
                slot=ti*4+1+ei; target=slot+edge['opposite']
                require(0 <= target < len(triangles)*4 and target%4!=0,'PHY opposite edge outside ledge')
                ot,oe=target//4,target%4-1
                other=triangles[ot]['edges'][oe]
                require(target+other['opposite']==slot,'PHY edge pairing is not reciprocal')
                require(other['start']==tri['edges'][(ei+1)%3]['start'] and
                        triangles[ot]['edges'][(oe+1)%3]['start']==edge['start'],'PHY edges do not form a closed surface')
            a,b,c=(points[e['start']] for e in tri['edges'])
            u=tuple(b[k]-a[k] for k in range(3));v=tuple(c[k]-a[k] for k in range(3))
            n=(u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0])
            length=math.sqrt(sum(x*x for x in n))
            if length<1e-12:continue
            n=tuple(x/length for x in n);d=sum(n[k]*a[k] for k in range(3))
            if sum(n[k]*center[k] for k in range(3))>d:n=tuple(-x for x in n);d=-d
            planes.append((n,d))
        require(bool(planes),'PHY piece has no nondegenerate faces')
        bounds=tuple((min(p[k] for p in verts),max(p[k] for p in verts)) for k in range(3))
        signature=collections.Counter()
        for tri in triangles:
            corners=tuple((points[e['start']],e['virtual']) for e in tri['edges'])
            signature[(min(corners,corners[1:]+corners[:1],corners[2:]+corners[:2]),tri['material'],tri['is_virtual'])]+=1
        pieces.append((verts,planes,bounds,signature))
    return pieces


def support_error(a,b):
    def outside(points,planes):
        return max(0.,max(sum(n[k]*p[k] for k in range(3))-d for p in points for n,d in planes))
    return max(outside(a[0],b[1]),outside(b[0],a[1]))


def map_collision(path):
    data=Path(path).read_bytes();model=mdl53.Model(data);h=model.hdr;bones=model.bones()
    if not h['collisionOffset']:return {}
    count=h['staticCollisionCount'];groups={}
    if count>0:groups['static']=range(count)
    else:
        for bone in bones:
            if bone['collisionCount'] and bone['collisionIndex']!=65535:
                groups[bone['name']]=range(bone['collisionIndex'],bone['collisionIndex']+bone['collisionCount'])
        count=max((i+1 for indices in groups.values() for i in indices),default=0)
    span(data,h['collisionOffset'],count,20,'map-collision headers');pieces={}
    for owner,indices in groups.items():
        pieces[owner]=[]
        for i in indices:
            base=h['collisionOffset']+i*20;fc,fb,ec,vc,off=struct.unpack_from('<5i',data,base)
            require(0<fc<=fb and ec>0 and vc>0,'invalid map-collision counts')
            at=base+off;span(data,at,fb,16,'map planes');eo=at+fb*16
            span(data,eo,ec,28,'map edges');vo=eo+ec*28;span(data,vo,vc,12,'map vertices')
            vertices=[struct.unpack_from('<3f',data,vo+k*12) for k in range(vc)]
            require(all(math.isfinite(x) for p in vertices for x in p),'nonfinite map vertex')
            center=tuple(sum(p[k] for p in vertices)/vc for k in range(3));planes=[]
            for k in range(fb):
                nx,ny,nz,d=struct.unpack_from('<4f',data,at+k*16);n=(nx,ny,nz)
                require(all(math.isfinite(x) for x in (*n,d)) and .99<math.dist(n,(0,0,0))<1.01,'invalid map plane')
                if sum(n[c]*center[c] for c in range(3))>d:n=tuple(-x for x in n);d=-d
                planes.append((n,d))
            for k in range(ec):
                edge=struct.unpack_from('<6f4B',data,eo+k*28)
                require(all(math.isfinite(x) for x in edge[:6]) and all(s<fb for s in edge[6:8]) and
                        all(v<vc for v in edge[8:]),'invalid map edge')
            bounds=tuple((min(p[k] for p in vertices),max(p[k] for p in vertices)) for k in range(3))
            pieces[owner].append((vertices,planes,bounds))
    return pieces


def compare_map_collision(original,rebuilt):
    a,b=map_collision(original),map_collision(rebuilt)
    require(a.keys()==b.keys(),'map-collision ownership differs')
    worst=0.;count=0
    def plane_error(a,b):
        bounds=tuple((min(a[2][k][0],b[2][k][0]),max(a[2][k][1],b[2][k][1])) for k in range(3))
        center=tuple((lo+hi)/2 for lo,hi in bounds);half=tuple((hi-lo)/2 for lo,hi in bounds)
        choices=[];costs={}
        for ai,(na,da) in enumerate(a[1]):
            compatible=[]
            for bi,(nb,db) in enumerate(b[1]):
                delta=tuple(na[k]-nb[k] for k in range(3))
                # Maximum difference of the two signed-distance functions on
                # the complete bounding box, including a plane moved outward.
                error=abs(sum(delta[k]*center[k] for k in range(3))-da+db)+sum(abs(delta[k])*half[k] for k in range(3))
                if error<=PHYSICS_TOLERANCE:compatible.append(bi);costs[ai,bi]=error
            choices.append(compatible)
        assigned=assign_pairs(choices)
        return None if assigned is None else max((costs[ai,bi] for bi,ai in assigned.items()),default=0.)
    for owner,pa in a.items():
        pb=b[owner];require(len(pa)==len(pb),'map-collision piece count differs');count+=len(pa)
        choices=[];costs={}
        for ai,piece in enumerate(pa):
            compatible=[]
            for i,other in enumerate(pb):
                if len(piece[0])!=len(other[0]) or len(piece[1])!=len(other[1]):continue
                if any(abs(x-y)>PHYSICS_TOLERANCE for x,y in zip(sum(piece[2],()),sum(other[2],()))):continue
                points=max(max(min(math.dist(p,q) for q in other[0]) for p in piece[0]),
                           max(min(math.dist(p,q) for q in piece[0]) for p in other[0]))
                planes=plane_error(piece,other) if points<=PHYSICS_TOLERANCE else None
                if planes is not None:compatible.append(i);costs[ai,i]=max(points,planes)
            choices.append(compatible)
        assigned=assign_pairs(choices)
        require(assigned is not None,'map-collision brush geometry differs for '+owner)
        for bi,ai in assigned.items():worst=max(worst,costs[ai,bi])
    return {'pieces':count,'maximum_plane_error':worst}


def compare_physics(original,rebuilt):
    a,b=physics_data(original),physics_data(rebuilt)
    if a is None or b is None:
        return {'ok':a is None and b is None,'solids':0,'convex_pieces':0,'maximum_plane_error':0.,'issues':[] if a is None and b is None else ['PHY presence differs']}
    issues=[];count=0;worst=0.
    if a['solidCount']!=b['solidCount']:issues.append('PHY solid count differs')
    if physics_blocks(a['text'])!=physics_blocks(b['text']):issues.append('physics mass/material/damping/constraints/friction/break properties differ')
    for i,(sa,sb) in enumerate(zip(a['solids'],b['solids'])):
        for key in ('mass_center','rot_inertia','drag','upper_limit_radius'):
            if sa[key]!=sb[key]:issues.append('solid %d %s differs'%(i,key))
        pa,pb=convex_pieces(sa),convex_pieces(sb);count+=len(pa)
        if len(pa)!=len(pb):issues.append('solid %d convex piece count differs'%i);continue
        compatible=[]; costs={}
        for ai,va in enumerate(pa):
            choices=[]
            for bi,vb in enumerate(pb):
                if any(abs(x-y)>PHYSICS_TOLERANCE for x,y in zip(sum(va[2],()),sum(vb[2],()))):continue
                # Identical oriented triangle surfaces are equal even when a
                # retail ledge is not perfectly convex. Extending those bounded
                # triangles to infinite supporting planes would reject itself.
                error=0. if va[3]==vb[3] else support_error(va,vb)
                if error<=PHYSICS_TOLERANCE:choices.append(bi);costs[ai,bi]=error
            compatible.append(choices)
        assigned={}
        def match(index,seen):
            for choice in compatible[index]:
                if choice in seen:continue
                seen.add(choice)
                if choice not in assigned or match(assigned[choice],seen):
                    assigned[choice]=index;return True
            return False
        if any(not match(ai,set()) for ai in range(len(pa))):issues.append('solid %d convex geometry differs'%i)
        for bi,ai in assigned.items():worst=max(worst,costs[ai,bi])
    return {'ok':not issues,'solids':a['solidCount'],'convex_pieces':count,'maximum_plane_error':worst,'issues':issues}


def compare_metadata(original,rebuilt):
    def read(path):
        data=Path(path).read_bytes();model=mdl53.Model(data);h=model.hdr
        names=[]
        for i in range(h['numbodyparts']):
            at=h['bodypartindex']+i*mdl53.size(mdl53.BODYPART_FIELDS)
            bp=mdl53.u(mdl53.BODYPART_FIELDS,data,at)
            names.append((mdl53.cstr(data,at+bp['sznameindex']),
                          [mdl53.u(mdl53.MODEL_FIELDS,data,at+bp['modelindex']+j*mdl53.size(mdl53.MODEL_FIELDS))['name'] for j in range(bp['nummodels'])]))
        return {**{key:h[key] for key in ('hull_min','hull_max','view_bbmin','view_bbmax','flags')},
                'bone_flags':[(b['name'],b['flags']) for b in model.bones()],'model_names':names}
    a,b=read(original),read(rebuilt)
    issues=[key+' differs' for key in a if a[key]!=b[key]]
    return {'ok':not issues,'issues':issues}


def validate_against(original,rebuilt):
    result={'structure':validate(rebuilt,False)}
    data=Path(original).read_bytes(); header=mdl53.Model(data).hdr
    collision=validate_tree(data,header,[],[],False)
    result['collision_geometry_equal']=result['structure']['collision_signature']==collision['signature']
    result['animations']=compare_animations(original,rebuilt)
    result['material_slots_equal']=material_slots(original)==material_slots(rebuilt)
    result['render_geometry']=compare_render(original,rebuilt)
    result['physics']=compare_physics(original,rebuilt)
    result['map_collision']=compare_map_collision(original,rebuilt)
    result['metadata']=compare_metadata(original,rebuilt)
    result['ok']=not result['animations']['issues'] and result['collision_geometry_equal'] and result['material_slots_equal'] and result['physics']['ok'] and result['metadata']['ok']
    return result


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('model'); ap.add_argument('--against'); ap.add_argument('--report')
    ap.add_argument('--allow-custom-collision',action='store_true',help='permit intentional collision geometry different from the render mesh')
    a=ap.parse_args(); result={}
    try:
        if a.against:
            result=validate_against(a.against,a.model)
        else:
            result['structure']=validate(a.model,not a.allow_custom_collision)
            result['ok']=True
    except (ValueError,IndexError,KeyError,OSError,OverflowError,RuntimeError,struct.error) as error:
        result.update(ok=False,error=str(error))
    if a.report: Path(a.report).write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))
    raise SystemExit(0 if result['ok'] else 1)


if __name__=='__main__': main()
