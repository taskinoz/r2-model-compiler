"""Axis-interpolation in-game test model: 'driver' yaws 0->90 degrees; 'axisbone' is
driven by the driver's X axis (in root space): +X -> low pose, +Y -> high, rolled pose."""
import math, sys
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
bones = [{'name': 'root', 'parent': -1}, {'name': 'driver', 'parent': 0}, {'name': 'axisbone', 'parent': 0}]
bind = [((0,0,0),(0,0,0)), ((0,0,20),(0,0,0)), ((0,-30,20),(0,0,0))]
# base plate, a long bar on the driver (shows its yaw), a cube on the axis bone
geo = box((0,0,0), (20,40,2), 0) + box((15,0,20), (15,3,3), 1) + box((0,-30,20), (6,6,6), 2)
dc.write_smd(out / 'body.smd', bones, [bind], geo)
frames = []
for i in range(31):
    f = list(bind); f[1] = ((0,0,20),(0,0,math.radians(3 * i)))
    frames.append(f)
dc.write_smd(out / 'turn.smd', bones, frames)
half = list(bind); half[1] = ((0,0,20),(0,0,math.radians(45)))
dc.write_smd(out / 'half.smd', bones, [half, half])  # driver held at 45 degrees
# axis 1 = the driver's X column. +X (index 0) keeps the bind pose; +Y (index 2) lifts
# the cube 60 units and rolls it 45 degrees. -X/-Y/±Z repeat the bind pose.
lines = ['version 1', 'proceduralbones', 'bone "axisbone" "root" "driver" "root" 1', 'basepos 0 0 0']
for k in range(6):
    lines.append('axis %d 0 -30 80 45 0 0' % k if k == 2 else 'axis %d 0 -30 20 0 0 0' % k)
(out / 'axis.txt').write_text('\n'.join(lines + ['end']) + '\n')
qc = ('$modelname "axis.mdl"\n$cdmaterials ""\n$body body "body.smd"\n$attachment "cube_att" "axisbone" 0 0 0\n'
      '$sequence turn "turn.smd" fps 10\n$sequence half "half.smd" loop\n')
(out / 'without.qc').write_text(qc)
(out / 'with.qc').write_text(qc + '$proceduralbones "axis.txt"\n')
