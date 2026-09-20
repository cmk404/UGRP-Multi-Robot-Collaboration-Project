"""Render completed trials side by side; display crop only, decision-step clock.

Usage: python scripts/render_jev_motion.py COHORT OUTPUT
"""
from pathlib import Path
import cv2,json,subprocess,sys
import numpy as np
base=Path(sys.argv[1]);out=Path(sys.argv[2]);out.mkdir(parents=True,exist_ok=True)
policies=['rule','gemini','jev'];cases=['straight','left_offset','right_offset']
trials={}
for case in cases:
 for policy in policies:
  p=base/(policy+'-'+case)
  trials[case,policy]=(p,json.loads((p/'result.json').read_text()),json.loads((p/'turns.json').read_text()) if (p/'turns.json').exists() else [])
W,H=1440,1090
proc=subprocess.Popen(['ffmpeg','-y','-loglevel','error','-f','rawvideo','-pix_fmt','bgr24','-s',f'{W}x{H}','-r','4','-i','-','-an','-c:v','libx264','-crf','21','-pix_fmt','yuv420p','-movflags','+faststart',str(out/'comparison.mp4')],stdin=subprocess.PIPE)
def label(img,text,xy,scale=.55,color=(225,230,238)):
 cv2.putText(img,text,xy,cv2.FONT_HERSHEY_SIMPLEX,scale,color,1,cv2.LINE_AA)
count=max(len(t[2]) for t in trials.values())+8
for i in range(count):
 canvas=np.full((H,W,3),(22,19,17),np.uint8)
 label(canvas,'Jev direct motion | identical RGB estimates + discrete 0.2 s actions',(18,30),.85)
 label(canvas,'Rows: 3 start poses. Columns: policies. Aligned by decision step, NOT wall time. Inference pauses SIM.',(18,57),.61)
 for y,case in enumerate(cases):
  for x,policy in enumerate(policies):
   p,r,turns=trials[case,policy];x0=x*480;y0=83+y*320
   finished=i>=len(turns);row=turns[min(i,len(turns)-1)] if turns else None
   src=p/(row['images']['shared_top_rgb']['path'] if row else 'rgb/probe-after-top.jpg')
   image=cv2.imread(str(src));crop=image[445:650,45:390]
   crop=cv2.resize(crop,(456,271));canvas[y0+27:y0+298,x0+12:x0+468]=crop
   status=('PASS' if r['success'] else 'FAIL') if finished else (row['action'] if row else 'STOPPED')
   label(canvas,f'{case} | {policy.upper()} | {status}',(x0+12,y0+18),.56,(145,230,130) if finished and r['success'] else (230,210,170))
   if finished:
    detail=f"END: {r['model_calls']} API calls | {r['wall_s']:.1f}s wall | {r['sim_s']:.1f}s SIM"
   else:detail=f"step {i} | RGB {row['observation']['range_m']:.3f}m / {row['observation']['bearing_deg']:+.1f}deg"
   label(canvas,detail,(x0+12,y0+316),.51)
 label(canvas,'Display-only TOP crop. Last observed frames held after episode end. Approach only; no grasp/carry or real-time claim.',(18,1080),.57)
 if i in (0,count//2,count-9):cv2.imwrite(str(out/f'preview-{i:03d}.png'),canvas)
 proc.stdin.write(canvas.tobytes())
proc.stdin.close()
if proc.wait()!=0:raise RuntimeError('ffmpeg failed')
print(out/'comparison.mp4')
