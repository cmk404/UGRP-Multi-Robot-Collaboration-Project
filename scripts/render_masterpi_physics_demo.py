#!/usr/bin/env python3
from __future__ import annotations
import math, os, subprocess
from pathlib import Path
os.environ.setdefault("MUJOCO_GL", "osmesa")
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from sim.masterpi_physics import CARRY_POSE, GROUND_POSE, MasterPiPhysicsWorld

FPS=12; W=480; H=270

def get_font(size):
    p='/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
    return ImageFont.truetype(p,size) if Path(p).exists() else ImageFont.load_default()

def main(out):
    w=MasterPiPhysicsWorld(seed=19,width=W,height=H)
    r=mujoco.Renderer(w.model,height=H,width=W)
    f1=get_font(17); f2=get_font(11)
    ff=subprocess.Popen(['ffmpeg','-y','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24','-s',f'{W}x{H}','-r',str(FPS),'-i','-','-an','-c:v','libx264','-preset','veryfast','-crf','24','-pix_fmt','yuv420p','-movflags','+faststart',out],stdin=subprocess.PIPE)
    def emit(phase):
        r.update_scene(w.data,camera='overview'); im=Image.fromarray(r.render()); d=ImageDraw.Draw(im)
        st=w.state()
        d.rounded_rectangle((8,8,260,66),radius=7,fill=(12,12,12,185))
        d.text((16,13),'MASTERPI · 4DOF + GRIPPER',font=f1,fill=(255,255,255))
        d.text((16,39),phase,font=f2,fill=(240,240,240))
        d.rounded_rectangle((8,220,472,262),radius=7,fill=(12,12,12,185))
        d.text((16,226),f"50 mm cubes · floor z=0 · no table    contacts {st['left_contact']}/{st['right_contact']}",font=f2,fill=(245,245,245))
        d.text((16,244),f"block z={st['red_xyz'][2]:.3f} m    stable={st['stable']}    arm DOF={st['arm_dof']} + gripper={st['gripper_dof']}",font=f2,fill=(245,245,245))
        ff.stdin.write(np.asarray(im,dtype=np.uint8).tobytes())
    def hold(p,n):
        for _ in range(n): emit(p)
    hold('OBSERVE',8)
    robot0=w._body_pos('robot'); block=w._body_pos('red_block'); dx,dy=block[0]-robot0[0],block[1]-robot0[1]; dist=max(float(math.hypot(dx,dy)),1e-8)
    target=block[:2]-np.array([dx,dy])/dist*.155; yaw1=math.atan2(dy,dx); start=robot0[:2].copy()
    for t in np.linspace(0,1,18):
        u=t*t*(3-2*t); xy=(1-u)*start+u*target; yaw=u*yaw1
        w.data.mocap_pos[0]=[float(xy[0]),float(xy[1]),.040]; w.data.mocap_quat[0]=[math.cos(yaw/2),0,0,math.sin(yaw/2)]; w.base_yaw=yaw; w._set_actuator('a_yaw',0); mujoco.mj_forward(w.model,w.data); w.step_physics(3); emit('APPROACH')
    w.base_yaw=yaw1; hold('RE-OBSERVE',4)
    w.open_gripper(); start=w._arm_targets(); target_pose=GROUND_POSE.copy(); target_pose[0]=0
    for t in np.linspace(0,1,24):
        u=t*t*(3-2*t); w._set_arm_target((1-u)*start+u*target_pose); w.step_physics(6); emit('4DOF ALIGN TO FLOOR CUBE')
    for c in np.linspace(0,1,14):
        w.close_gripper(float(c)); w.step_physics(14); emit('PHYSICAL GRASP')
    hold('BILATERAL CONTACT',6)
    start=w._arm_targets(); target_pose=CARRY_POSE.copy(); target_pose[0]=start[0]
    for t in np.linspace(0,1,28):
        u=t*t*(3-2*t); w._set_arm_target((1-u)*start+u*target_pose); w.close_gripper(1); w.step_physics(8); emit('LIFT / CARRY')
    w.step_physics(80); hold('SUCCESS · STABLE HOLD',12)
    final=w.state(); ff.stdin.close(); rc=ff.wait(); r.close(); w.close()
    if rc: raise SystemExit(rc)
    print(final,flush=True)

if __name__=='__main__':
    import argparse
    ap=argparse.ArgumentParser(); ap.add_argument('--out',default='outputs/masterpi_4dof_physics_demo.mp4'); a=ap.parse_args(); Path(a.out).parent.mkdir(parents=True,exist_ok=True); main(a.out)
