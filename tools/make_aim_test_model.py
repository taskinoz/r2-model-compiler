"""Aim-at-attachment in-game test model. 'target' rises 40 units; the attachment
'aim_att' sits 40 units above it (marked by the upper cube). The bar on 'aimer'
points along +Y and is aimed at the attachment (type 4), at the target bone itself
(type 3, for comparison) or not at all."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import decompile53 as dc

def box(center, half, bone):
    hx, hy, hz = half
    v = [(center[0] + x * hx, center[1] + y * hy, center[2] + z * hz)
         for x, y, z in ((-1,-1,-1), (1,-1,-1), (1,1,-1), (-1,1,-1), (-1,-1,1), (1,-1,1), (1,1,1), (-1,1,1))]
    faces = ((0,2,1),(0,3,2),(4,5,6),(4,6,7),(0,1,5),(0,5,4),(1,2,6),(1,6,5),(2,3,7),(2,7,6),(3,0,4),(3,4,7))
    return [('models/electricalboxes/pipesmulti', [(v[i], (0,0,1), (0,0), [(bone, 1.)]) for i in f]) for f in faces]

out = Path(sys.argv[1]); out.mkdir(parents=True, exist_ok=True)
bones = [{'name': 'root', 'parent': -1}, {'name': 'target', 'parent': 0}, {'name': 'aimer', 'parent': 0}]
bind = [((0,0,0),(0,0,0)), ((0,60,20),(0,0,0)), ((0,0,20),(0,0,0))]
geo = (box((0,30,0), (10,45,2), 0)            # base plate
       + box((0,60,20), (3,3,3), 1)           # small cube: the target bone
       + box((0,60,60), (6,6,6), 1)           # large cube: the attachment
       + box((0,25,20), (2,25,2), 2))         # bar along the aimer's +Y
dc.write_smd(out / 'body.smd', bones, [bind], geo)
frames = []
for i in range(31):
    f = list(bind); f[1] = ((0,60,20 + 40 * i / 30),(0,0,0))
    frames.append(f)
dc.write_smd(out / 'rise.smd', bones, frames)
for name, aim in (('attach', 'aim_att'), ('bone', 'target')):
    (out / ('aim_%s.vrd' % name)).write_text('<aimconstraint> aimer root %s\n<aimvector> 0 1 0\n'
                                              '<upvector> 0 0 1\n<basepos> 0 0 20\n' % aim)
qc = ('$modelname "aim.mdl"\n$body body "body.smd"\n$attachment "aim_att" "target" 0 0 40\n'
      '$attachment "bar_tip" "aimer" 0 50 0\n$sequence rise "rise.smd" fps 10\n')
(out / 'none.qc').write_text(qc)
for name in ('attach', 'bone'):
    (out / ('%s.qc' % name)).write_text(qc + '$proceduralbones "aim_%s.vrd"\n' % name)
