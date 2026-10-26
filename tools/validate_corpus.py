"""Validate every rebuilt model in a roundtrip report, including every animation frame.

Compilation and validation are separate so a validator repair never requires another
corpus compile. Results checkpoint after each model. The input report and validation
tools must be unchanged for --resume; source and output stamps are checked per model.
"""
import argparse
import collections
import concurrent.futures
import hashlib
import json
import os
from pathlib import Path
import sys
import traceback

sys.path.insert(0,str(Path(__file__).resolve().parent))
import decompile53
import mdl53
import roundtrip
import validate53


def stamp(path):
    s=Path(path).stat()
    return [s.st_size,s.st_mtime_ns]


def check(job):
    source,rebuilt,input_stamp=job
    try:
        if input_stamp != stamp(source): raise ValueError('retail input changed since compilation')
        result=validate53.validate_against(source,rebuilt)
        result['status']='ok' if result['ok'] else 'diff'
        result['stamps']=[stamp(source),stamp(rebuilt)]
        result['rebuilt_sha256']=hashlib.sha256(Path(rebuilt).read_bytes()).hexdigest()
    except Exception as error:
        result={'status':'exception','error':repr(error),'trace':traceback.format_exc()}
    return source,result


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--report',required=True,help='results.json from roundtrip.py')
    ap.add_argument('--out',required=True)
    ap.add_argument('--jobs',type=int,default=1)
    ap.add_argument('--resume',action='store_true')
    a=ap.parse_args()
    if a.jobs<1:ap.error('--jobs must be at least 1')
    report=Path(a.report).resolve(); work=report.parent; output=Path(a.out).resolve()
    inputs=json.loads(report.read_text());inputs=inputs.get('results',inputs)
    output.mkdir(parents=True,exist_ok=True)
    paths=[Path(__file__),Path(validate53.__file__),Path(decompile53.__file__),Path(mdl53.__file__),Path(validate53.phylib.__file__),report]
    signature={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    try:
        import numpy
        signature['numpy_version']=numpy.__version__
    except ImportError: signature['numpy_version']=None
    signature['python_version']=sys.version
    signature['thresholds']={'position_inches':validate53.POSITION_TOLERANCE,
                             'rotation_degrees':validate53.ROTATION_TOLERANCE,'scale':validate53.SCALE_TOLERANCE,
                             'physics_plane_inches':validate53.PHYSICS_TOLERANCE,
                             'vertex_position_inches':validate53.VERTEX_POSITION_TOLERANCE,
                             'vertex_attributes':validate53.VERTEX_ATTRIBUTE_TOLERANCE}
    manifest=output/'run.json'; saved=output/'results.json'; results={}
    if a.resume and saved.exists():
        if not manifest.exists() or json.loads(manifest.read_text())!=signature:
            ap.error('cannot resume: report or validator tools/runtime changed; use a new --out or omit --resume')
        results=json.loads(saved.read_text())
    roundtrip.checkpoint(str(manifest),signature)
    names=collections.Counter(Path(s).stem for s in inputs); jobs=[]; resumed=0
    for source,compiled in sorted(inputs.items()):
        name=Path(source).stem
        suffix='-'+hashlib.sha256(source.encode()).hexdigest()[:12] if names[name]>1 else ''
        rebuilt=work/(name+suffix)/'recompiled.mdl'
        if compiled['status'] not in ('ok','diff'):
            results[source]={'status':'not-compiled','error':compiled.get('error','compiler did not produce a result')}
            continue
        try: stamps=[stamp(source),stamp(rebuilt)]
        except OSError: stamps=None
        if a.resume and stamps and results.get(source,{}).get('stamps')==stamps:
            resumed+=1;continue
        results.pop(source,None)
        jobs.append((source,str(rebuilt),compiled.get('input_stamp')))
    roundtrip.checkpoint(str(saved),results)
    with concurrent.futures.ProcessPoolExecutor(max_workers=a.jobs) as pool:
        futures=[pool.submit(check,job) for job in jobs]
        for future in concurrent.futures.as_completed(futures):
            source,result=future.result();results[source]=result
            roundtrip.checkpoint(str(saved),results)
            if result['status']!='ok' or len(results)%25==0:
                detail=result.get('error',result.get('animations',{}).get('issues',[]))
                print('%d/%d %s %s %s'%(len(results),len(inputs),result['status'].upper(),Path(source).stem,detail),flush=True)
    counts=dict(collections.Counter(v['status'] for v in results.values()))
    summary={'counts':counts,'resumed':resumed,
             'animation_frames':sum(v.get('animations',{}).get('frames',0) for v in results.values()),
             'bone_frames':sum(v.get('animations',{}).get('bone_frames',0) for v in results.values()),
             'ik_error_frames':sum(v.get('animations',{}).get('ik_error_frames',0) for v in results.values()),
             'collision_triangles':sum(v.get('structure',{}).get('collision_triangles',0) for v in results.values()),
             'render_triangles':sum(v.get('render_geometry',{}).get('triangles',0) for v in results.values()),
             'physics_solids':sum(v.get('physics',{}).get('solids',0) for v in results.values()),
             'physics_convex_pieces':sum(v.get('physics',{}).get('convex_pieces',0) for v in results.values())}
    summary['map_collision_pieces']=sum(v.get('map_collision',{}).get('pieces',0) for v in results.values())
    roundtrip.checkpoint(str(output/'summary.json'),summary)
    print(json.dumps(summary,indent=2),flush=True)
    raise SystemExit(0 if counts.get('ok',0)==len(inputs) else 1)


if __name__=='__main__':main()
