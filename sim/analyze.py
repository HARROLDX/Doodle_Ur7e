#!/usr/bin/env python3
"""Independent actual-tip vs planned stroke metrics; never feeds commands or ink."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
import yaml
from PIL import Image,ImageDraw
from doodle_ur7e.core import load_config,load_strokes,waypoints
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'runs/isaac'
c=load_config(ROOT/'config/isaac.yaml');cal=yaml.safe_load((OUT/'paper.yaml').read_text())
trace=np.loadtxt(OUT/'actual_tip.csv',delimiter=',');contact=trace[trace[:,4]==1,1:4]
if len(contact)==0:raise RuntimeError('no actual pen contact samples recorded')
# Ground-truth paper frame for evaluation, not camera pose fed back into robot control.
xy=contact[:,:2]-np.array([-.075,.425])
canvas=Image.new('RGB',(1000,1000),'white');draw=ImageDraw.Draw(canvas)
for a,b in zip(trace[:-1],trace[1:]):
    if a[4] and b[4] and np.linalg.norm(a[1:4]-b[1:4])<.003:
        coords=[]
        for row in [a,b]:coords.append(((row[1]+.1)/.2*1000,(.6-row[2])/.2*1000))
        draw.line(coords,fill='black',width=3)
canvas.save(OUT/'actual_drawing.png')
plan=json.loads((OUT/'drawing/plan.json').read_text())
targets=np.asarray(plan['tip_waypoints']);paper_z=cal['position'][2]
segments=[]
for a,b in zip(targets[:-1],targets[1:]):
    if abs(a[2]-paper_z)<.002 and abs(b[2]-paper_z)<.002:
        segments.extend(np.linspace(a,b,max(2,int(np.linalg.norm(a-b)/.0002)+1)))
ideal=np.asarray(segments)
if len(ideal)==0:raise RuntimeError('no planned contact segments')
errors=cKDTree(ideal).query(contact)[0];coverage=cKDTree(contact).query(ideal)[0]
world=json.loads((OUT/'world_result.json').read_text())
report={'executed':plan['executed'],'trajectory_fraction':plan['fraction'],'duration_sim_seconds':float(trace[-1,0]-trace[0,0]),
 'contact_samples':len(contact),'actual_to_planned_rms_m':float(np.sqrt(np.mean(errors**2))),
 'actual_to_planned_p95_m':float(np.percentile(errors,95)),'planned_to_actual_p95_m':float(np.percentile(coverage,95)),
 'camera_position_error_m':cal['simulation_position_error_m'],'frames':world['frames'],
 'limitations':['Geometric ink threshold, no calibrated pen force or ink physics.','Camera intrinsics and mount are simulation assumptions, not measured D435i calibration.','UR speed scaling is fixed to 1.0; hardware safety behavior is not emulated.']}
report['world_finished']=world['finished']
report['passed']=bool(report['world_finished'] and report['executed'] and report['trajectory_fraction']>=.999999 and report['actual_to_planned_p95_m']<.002 and report['planned_to_actual_p95_m']<.002)
(OUT/'metrics.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
if not report['passed']:raise RuntimeError('simulation drawing acceptance failed')
