"""Render all 12 new poses, repeat zero. Usage: COHORT OUTPUT AUDIT_REPORT_JSON."""
from pathlib import Path
import cv2,json,subprocess,sys
import numpy as np
base=Path(sys.argv[1]);out=Path(sys.argv[2]);out.mkdir(parents=True,exist_ok=True)
audited={r['trial_id']:r for r in json.loads(Path(sys.argv[3]).read_text())['results']}
arms=['rule-numeric','jev-numeric','jev-semantic','gemini-numeric','gemini-semantic'];W,H=2000,990
trials={}
for ci in range(1,13):
 case=f'new{ci:02d}'
 for arm in arms:
  p=base/f'new-{case}-r0-{arm}'
  trials[case,arm]=(p,json.loads((p/'result.json').read_text()),json.loads((p/'turns.json').read_text()) if (p/'turns.json').exists() else [])
def label(im,text,xy,scale=.48,color=(230,230,230)):cv2.putText(im,text,xy,cv2.FONT_HERSHEY_SIMPLEX,scale,color,1,cv2.LINE_AA)
proc=subprocess.Popen(['ffmpeg','-y','-loglevel','error','-f','rawvideo','-pix_fmt','bgr24','-s',f'{W}x{H}','-r','4','-i','-','-an','-c:v','libx264','-crf','21','-pix_fmt','yuv420p','-movflags','+faststart',str(out/'comparison.mp4')],stdin=subprocess.PIPE)
for group in range(4):
 cases=[f'new{ci:02d}' for ci in range(1+group*3,4+group*3)]
 count=max(len(trials[c,a][2]) for c in cases for a in arms)+8
 for step in range(count):
  im=np.full((H,W,3),(22,19,17),np.uint8)
  label(im,f'Jev semantic-state closed loop | new start poses {group*3+1}-{group*3+3} | repeat 0 of 3',(16,32),.9)
  label(im,'Same RGB observer, 7 motor actions and goal. Decision-step alignment; inference pauses SIM. Full results include all 3 repeats.',(16,62),.64)
  for y,c in enumerate(cases):
   for x,a in enumerate(arms):
    p,r,rows=trials[c,a];done=step>=len(rows);row=rows[min(step,len(rows)-1)] if rows else None
    src=p/(row['images']['shared_top_rgb']['path'] if row else 'rgb/probe-after-top.jpg')
    frame=cv2.imread(str(src));crop=cv2.resize(frame[430:668,0:400],(380,226));x0=x*400;y0=88+y*280
    im[y0+28:y0+254,x0+10:x0+390]=crop
    checked=audited[p.name];passed=checked['audited_success']
    status=('PASS*' if checked['numerical_time_correction'] else 'PASS' if passed else 'FAIL') if done else row['action']
    label(im,f'{c} | {a} | {status}',(x0+10,y0+19),.44,(130,230,140) if done and passed else (235,210,165))
    detail=f"END {r['model_calls']} calls | {r['wall_s']:.0f}s wall" if done else f"step {step} | RGB {row['observation']['range_m']:.3f}m / {row['observation']['bearing_deg']:+.1f}deg"
    label(im,detail,(x0+10,y0+271),.46)
  label(im,'Display-only TOP crop. Last observed frames held. Single-box approach/alignment; no grasp/carry or real-time control claim.',(16,948),.63)
  label(im,'PASS* = post-run correction of floating-point elapsed time at 0.350s; original verdict retained. No control or geometry threshold changed.',(16,976),.62)
  if step in (0,count//2,count-1):cv2.imwrite(str(out/f'preview-g{group}-{step:03d}.jpg'),im)
  proc.stdin.write(im.tobytes())
proc.stdin.close()
if proc.wait():raise RuntimeError('ffmpeg failed')
print(out/'comparison.mp4')
