"""Physics owner for the research dispatch arena; never given to an actor."""
from __future__ import annotations

import hashlib
from pathlib import Path
from unittest.mock import patch

from sim.research_dispatch_arena import ROBOTS, FIXED_TOP, build_scene_xml


class DispatchScene:
    def __init__(self,config,output,*,render=True):
        self.config,self.out,self.render = config,Path(output),render
        self.world=None
        self.ports={}
        self.command_history={r:[] for r in ROBOTS}
        self.sequence=0
        self.physics_steps=self.weld_steps=self.obstacle_contact_steps=0

    def open(self):
        import mujoco
        import sim.multi_masterpi_production as production
        from sim.camera_robot_port import CameraRobotPort
        from scripts.probe_dual_grasp_sync import _plain_beam_xml
        original=_plain_beam_xml(production.build_multi_robot_xml)
        def builder(*args,**kwargs):
            kwargs['navigation_camera']=False
            xml,self.manifest=build_scene_xml(original(*args,**kwargs),self.config)
            self.xml=xml
            return xml
        self.out.mkdir(parents=True,exist_ok=False)
        (self.out/'rgb').mkdir()
        with patch.object(production,'build_multi_robot_xml',builder):
            self.world=production.MultiMasterPiProductionV2(seed=self.config['seed'],
                                    width=960,height=720,render=self.render)
        w=self.world
        # One-time authored reset only, before observations/actor calls.
        for r,pose in self.config['setup_only']['spawns'].items():
            w.controllers[r].set_base_pose_for_test(tuple(pose[:3]),pose[3])
        for obj,joint in [('beam','team_beam_free'),('box','dispatch_box_free')]:
            jid=mujoco.mj_name2id(w.model,mujoco.mjtObj.mjOBJ_JOINT,joint)
            q=int(w.model.jnt_qposadr[jid]);v=int(w.model.jnt_dofadr[jid])
            w.data.qpos[q:q+7]=[*self.config['setup_only']['cargo'][obj],1,0,0,0]
            w.data.qvel[v:v+6]=0
        w.data.eq_active[:]=0
        mujoco.mj_forward(w.model,w.data)
        w._team_joint_move_servos({r:{1:2000,3:740,4:2320,5:1320,6:1500} for r in ROBOTS},.6,settle_s=.4)
        self.look_observer((2.6,-4.8,3.4))
        self.ports={r:CameraRobotPort(w,r,allow_reverse=True,allow_mecanum=True) for r in ROBOTS}
        self.obstacle_ids={i for i in range(w.model.ngeom)
            if (mujoco.mj_id2name(w.model,mujoco.mjtObj.mjOBJ_GEOM,i) or '').startswith('dispatch_')
            and w.model.geom_contype[i] and i != mujoco.mj_name2id(w.model,mujoco.mjtObj.mjOBJ_GEOM,'dispatch_box_geom')}
        self.robot_ids={i for i in range(w.model.ngeom)
            if (mujoco.mj_id2name(w.model,mujoco.mjtObj.mjOBJ_GEOM,i) or '').startswith(tuple(r+'__' for r in ROBOTS))}
        self.initial_invariants=self.invariants()
        (self.out/'scene.xml').write_text(self.xml)
        return self

    def look_observer(self,position):
        import mujoco
        import numpy as np
        w=self.world
        cid=mujoco.mj_name2id(w.model,mujoco.mjtObj.mjOBJ_CAMERA,'cctv_warehouse')
        pos=np.array(position,dtype=float);target=np.array([.55,-2.,.03])
        forward=target-pos;forward/=np.linalg.norm(forward)
        right=np.cross(forward,[0.,0.,1.]);right/=np.linalg.norm(right)
        up=np.cross(right,forward);quat=np.empty(4)
        mujoco.mju_mat2Quat(quat,np.column_stack((right,up,-forward)).ravel())
        w.model.cam_pos[cid]=pos;w.model.cam_quat[cid]=quat;w.model.cam_fovy[cid]=46
        mujoco.mj_forward(w.model,w.data)

    def invariants(self):
        import mujoco
        w=self.world
        cameras={}
        for name in ('cctv_top',*(r+'__robot_cam' for r in ROBOTS)):
            cid=mujoco.mj_name2id(w.model,mujoco.mjtObj.mjOBJ_CAMERA,name)
            cameras[name]={'position':w.model.cam_pos[cid].tolist(),
                'quaternion':w.model.cam_quat[cid].tolist(),'fovy':float(w.model.cam_fovy[cid])}
        return {**self.manifest,'policy_cameras':cameras,
                'weld_active':bool(w.data.eq_active.any()),
                'solver':{'impratio':float(w.model.opt.impratio),'noslip_iterations':int(w.model.opt.noslip_iterations)}}

    def capture(self,label):
        from scripts.camera_approach_scene import image_record
        self.sequence+=1
        top=self.world.render_team_jpeg(camera='cctv_top',quality=95)
        top_ref=image_record(self.out/'rgb'/f'{label}-top.jpg',self.out,top)
        frames={}
        for rid in ROBOTS:
            own=self.world.render_jpeg(robot_id=rid,camera='robot_cam',quality=95)
            frames[rid]={'own_bytes':own,'top_bytes':top,'frame_id':self.sequence,
                'own_rgb':image_record(self.out/'rgb'/f'{label}-{rid}.jpg',self.out,own),
                'shared_top_rgb':top_ref}
        (self.out/f'{label}-overview.jpg').write_bytes(self.world.render_team_jpeg(camera='cctv_warehouse',quality=95))
        return frames

    def step(self,seconds):
        w=self.world
        for _ in range(round(seconds/w.model.opt.timestep)):
            now=float(w.data.time)
            for port in self.ports.values():port.tick(now)
            w._physics_step_for(w.controllers['r1'])
            self.physics_steps+=1;self.weld_steps+=bool(w.data.eq_active.any())
            self.obstacle_contact_steps+=any(
                (int(c.geom1) in self.robot_ids and int(c.geom2) in self.obstacle_ids)
                or (int(c.geom2) in self.robot_ids and int(c.geom1) in self.obstacle_ids)
                for c in w.data.contact[:w.data.ncon] if c.dist<0)

    def evaluate_positions(self):
        """Output-only, not used for motor commands, stage changes or LLM input."""
        return {r:list(map(float,self.world.controllers[r].base_xyz())) for r in ROBOTS}

    def close(self):
        try:
            for p in self.ports.values():p.stop()
        finally:
            if self.world:self.world.close();self.world=None
