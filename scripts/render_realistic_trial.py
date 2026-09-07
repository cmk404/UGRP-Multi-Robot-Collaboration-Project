#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, math, subprocess, os
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from sim.continuous_physics import ContinuousPhysicsWorld, GROUND_POSE

W,H,FPS=1280,720,30
OV_W,OV_H=1280,720
IN_W,IN_H=360,203
FONT_CAND=['/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf']
BOLD_CAND=['/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf']

def font(sz,b=False):
 p=(BOLD_CAND if b else FONT_CAND)[0]
 return ImageFont.truetype(p,sz) if Path(p).exists() else ImageFont.load_default()
F18=font(18); F18B=font(18,True); F22B=font(22,True); F14=font(14)

def main():
 ap=argparse.ArgumentParser(); ap.add_argument('--trial',required=True); ap.add_argument('--out',required=True); ap.add_argument('--hold',type=float,default=5.0); ap.add_argument('--initial',type=float,default=1.5); a=ap.parse_args()
 cfg=json.loads(Path(a.trial).read_text()); face=bool(cfg['face_aligned']); actions=cfg['actions']; seed=int(cfg['seed'])
 w=ContinuousPhysicsWorld(seed=seed,width=960,height=540,render=True); w.reset(seed=seed,domain_randomize=True)
 cmd=['ffmpeg','-loglevel','error','-y','-f','rawvideo','-pix_fmt','rgb24','-s',f'{W}x{H}','-r',str(FPS),'-i','-','-an','-c:v','libx264','-preset','medium','-crf','21','-pix_fmt','yuv420p','-movflags','+faststart',a.out]
 proc=subprocess.Popen(cmd,stdin=subprocess.PIPE)
 frame_count=0; sim_start=float(w.data.time); policy_step=0
 def emit(phase, note=''):
  nonlocal frame_count
  ov=Image.fromarray(w.render_rgb('overview')).resize((OV_W,OV_H),Image.Resampling.LANCZOS).convert('RGBA')
  rb=Image.fromarray(w.render_rgb('robot_cam')).resize((IN_W,IN_H),Image.Resampling.LANCZOS).convert('RGBA')
  canvas=ov
  d=ImageDraw.Draw(canvas,'RGBA')
  # camera inset, always fixed in screen space; camera itself is rigidly attached to robot mast.
  ix,iy=W-IN_W-28,28
  d.rounded_rectangle((ix-5,iy-29,ix+IN_W+5,iy+IN_H+5),radius=7,fill=(5,7,10,210),outline=(245,245,245,230),width=2)
  canvas.alpha_composite(rb,(ix,iy))
  d.text((ix+8,iy-25),'ONBOARD CAMERA · fixed mast',font=F14,fill=(245,245,245,255))
  # small experiment identifier
  d.rounded_rectangle((22,20,405,56),radius=7,fill=(5,7,10,190))
  d.text((34,27),'UGRP · CONTINUOUS PHYSICS TRIAL',font=F18B,fill=(255,255,255,255))
  # status strip
  st=w.state(); speed=float(np.linalg.norm(np.array(st['base_vxyw'][:2],float))); z=st['red_xyz'][2]
  d.rounded_rectangle((18,H-87,W-18,H-18),radius=10,fill=(5,7,10,205))
  d.text((34,H-76),phase,font=F22B,fill=(255,255,255,255))
  d.text((34,H-47),f"t={w.data.time-sim_start:5.2f}s   base={speed:0.3f} m/s   block z={z:0.3f} m   L={st['left_normal_N']:0.2f} N   R={st['right_normal_N']:0.2f} N   bilateral={st['bilateral_contact']}   stable={st['stable']}",font=F18,fill=(235,238,242,255))
  if note: d.text((790,H-76),note,font=F18B,fill=(255,214,92,255))
  proc.stdin.write(np.asarray(canvas.convert('RGB'),dtype=np.uint8).tobytes()); frame_count+=1
 def step_record(n,phase,note='',stride=8):
  for k in range(n):
   w.step_physics(1)
   if (k+1)%stride==0: emit(phase,note)
 # actual simulated settling; no frozen image
 step_record(int(a.initial/w.model.opt.timestep),'SETTLE · scene at rest')
 # navigation uses only bounded continuous base actuation
 cbn={'n':0}
 def navcb(_):
  cbn['n']+=1
  if cbn['n']%2==0: emit('APPROACH · continuous base dynamics','face-aligned path' if face else 'raw target bearing')
 if face: nav=w.navigate_to_face_aligned_pregrasp(callback=navcb)
 else: nav=w.navigate_to_pregrasp(callback=navcb)
 # arm moves continuously through servo targets
 cbn['n']=0
 def armcb(_):
  cbn['n']+=1
  if cbn['n']%2==0: emit('PRE-GRASP · continuous arm motion')
 w.move_arm_continuous(GROUND_POSE,1.6,callback=armcb)
 # replay exactly the learned action sequence, one rendered frame per 8 physics steps
 for i,act in enumerate(actions,1):
  policy_step=i; w.apply_delta_action(act,frame_skip=8,motor_noise=0)
  emit('PPO GRASP · learned continuous control',f'policy step {i}/{len(actions)}')
  if w.state()['stable']: break
 # keep physics running after success/failure; never freeze a frame.
 final=w.state(); phase='VERIFY · stable physical hold' if final['stable'] else 'VERIFY · unsuccessful grasp / object remains dynamic'
 step_record(int(a.hold/w.model.opt.timestep),phase,'physics still running')
 proc.stdin.close(); rc=proc.wait(); w.close()
 if rc: raise SystemExit(rc)
 print(json.dumps({'out':a.out,'frames':frame_count,'duration_s':frame_count/FPS,'nav_ok':nav,'final':final},indent=2))
if __name__=='__main__': main()
