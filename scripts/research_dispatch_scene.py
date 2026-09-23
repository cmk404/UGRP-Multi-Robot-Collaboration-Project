"""Physics owner for the research dispatch arena; never given to an actor."""
from __future__ import annotations

from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass
from io import BytesIO
from pathlib import Path
import time

from PIL import Image

from sim.research_dispatch_arena import ROBOTS
from sim.session_scenes import Scene


@dataclass(frozen=True)
class PairPartialCapture:
    top: Future
    full: Future


class DispatchScene:
    def __init__(self,config,output,*,render=True):
        self.config,self.out,self.render = config,Path(output),render
        self.world=None
        self.ports={}
        self.command_history={r:[] for r in ROBOTS}
        self.sequence=0
        self.physics_steps=self.weld_steps=self.obstacle_contact_steps=0
        self.definition=None
        self._capture_workers=None
        self._top_capture_worker=None

    def open(self):
        import mujoco
        from sim.multi_masterpi_production import MultiMasterPiProductionV2
        from sim.camera_robot_port import CameraRobotPort
        self.definition=Scene.from_dispatch_config(self.config)
        self.out.mkdir(parents=True,exist_ok=False)
        (self.out/'rgb').mkdir()
        try:
            self.world=MultiMasterPiProductionV2(seed=self.config['seed'],
                        width=960,height=720,render=self.render,
                        warehouse_layout=self.definition.engine_layout,
                        warehouse_cargo_ids=self.definition.scene['cargo_ids'],
                        xml_transform=self.definition.transform)
            w=self.world
            self.manifest=self.definition.manifest
            self.xml=w.scene_xml
            self.definition.setup(w)
            self.ports={r:CameraRobotPort(w,r,allow_reverse=True,allow_mecanum=True) for r in ROBOTS}
            self.obstacle_ids={i for i in range(w.model.ngeom)
                if (mujoco.mj_id2name(w.model,mujoco.mjtObj.mjOBJ_GEOM,i) or '').startswith('dispatch_')
                and w.model.geom_contype[i] and i != mujoco.mj_name2id(w.model,mujoco.mjtObj.mjOBJ_GEOM,'dispatch_box_geom')}
            self.robot_ids={i for i in range(w.model.ngeom)
                if (mujoco.mj_id2name(w.model,mujoco.mjtObj.mjOBJ_GEOM,i) or '').startswith(tuple(r+'__' for r in ROBOTS))}
            self.initial_invariants=self.invariants()
            (self.out/'scene.xml').write_text(self.xml)
        except BaseException:
            for port in self.ports.values():
                try:
                    port.stop()
                except Exception:
                    pass
            if self.world is not None:
                self.world.close()
                self.world=None
            raise
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

    def capture(self,label,*,own_robots=None,overview=True):
        from scripts.camera_approach_scene import image_record
        selected=set(ROBOTS if own_robots is None else own_robots)
        if not selected or not selected.issubset(ROBOTS):
            raise ValueError('capture requires known robot cameras')
        self.sequence+=1
        top=self.world.render_team_jpeg(camera='cctv_top',quality=95)
        top_ref=image_record(self.out/'rgb'/f'{label}-top.jpg',self.out,top)
        frames={}
        for rid in ROBOTS:
            frames[rid]={'top_bytes':top,'frame_id':self.sequence,'shared_top_rgb':top_ref}
            if rid in selected:
                own=self.world.render_jpeg(robot_id=rid,camera='robot_cam',quality=95)
                frames[rid].update(own_bytes=own,
                    own_rgb=image_record(self.out/'rgb'/f'{label}-{rid}.jpg',self.out,own))
        if overview:
            (self.out/f'{label}-overview.jpg').write_bytes(self.world.render_team_jpeg(camera='cctv_warehouse',quality=95))
        return frames

    def capture_async(self,label,*,own_robots=None,overview=False):
        """Capture one immutable physics instant, then encode/store RGB off owner.

        The render broker owns its GL thread and supplies a single frame id and
        SIM timestamp for all cameras. At most the caller's bounded in-flight
        requests should be outstanding; this method never advances physics.
        """
        from scripts.camera_approach_scene import image_record
        selected=set(ROBOTS if own_robots is None else own_robots)
        if not selected or not selected.issubset(ROBOTS):
            raise ValueError('capture requires known robot cameras')
        cameras=[(None,'cctv_top'),*((rid,'robot_cam') for rid in ROBOTS if rid in selected)]
        if overview:cameras.append((None,'cctv_warehouse'))
        requested_wall_s=time.monotonic()
        capture_on_render_ready=bool(getattr(self,'realtime_control',False))
        batch_future=self.world.render_snapshot_async(
            cameras,capture_on_render_ready=capture_on_render_ready)
        submitted_wall_s=time.monotonic()
        self.sequence+=1
        if self._capture_workers is None:
            self._capture_workers=ThreadPoolExecutor(max_workers=2,thread_name_prefix='dispatch-rgb')

        def materialize():
            batch=batch_future.result(timeout=30.)
            rendered_wall_s=time.monotonic()
            def jpeg(key):
                stream=BytesIO()
                Image.fromarray(batch.rgb[key]).save(stream,format='JPEG',quality=95)
                return stream.getvalue()
            encoded={(None,'cctv_top'):jpeg((None,'cctv_top'))}
            for rid in ROBOTS:
                if rid in selected:encoded[(rid,'robot_cam')]=jpeg((rid,'robot_cam'))
            if overview:encoded[(None,'cctv_warehouse')]=jpeg((None,'cctv_warehouse'))
            encoded_wall_s=time.monotonic()
            top=encoded[(None,'cctv_top')]
            top_ref=image_record(self.out/'rgb'/f'{label}-top.jpg',self.out,top)
            frames={}
            for rid in ROBOTS:
                frame={'top_bytes':top,'frame_id':batch.frame_id,
                       'observed_at_s':float(batch.sim_time),'shared_top_rgb':top_ref,
                       'capture_mode':('render_ready' if capture_on_render_ready else 'request_time'),
                       'capture_requested_wall_s':requested_wall_s,
                       'snapshot_submitted_wall_s':submitted_wall_s,
                       'render_completed_wall_s':rendered_wall_s,
                       'encode_completed_wall_s':encoded_wall_s}
                if rid in selected:
                    own=encoded[(rid,'robot_cam')]
                    frame.update(own_bytes=own,
                                 own_rgb=image_record(self.out/'rgb'/f'{label}-{rid}.jpg',self.out,own))
                frames[rid]=frame
            if overview:
                (self.out/f'{label}-overview.jpg').write_bytes(encoded[(None,'cctv_warehouse')])
            for frame in frames.values():frame['materialized_wall_s']=time.monotonic()
            return frames
        return self._capture_workers.submit(materialize)

    def capture_pair_partial_async(self,label,*,own_robots):
        """Start TOP JPEG as soon as its coherent snapshot view is rendered.

        This is only the open realtime pair carry path. The full result still
        contains both own views before a policy can receive the observation.
        """
        from scripts.camera_approach_scene import image_record
        selected=set(own_robots)
        if len(selected)!=2 or not selected.issubset(ROBOTS):
            raise ValueError('partial pair capture requires two robot cameras')
        cameras=[(None,'cctv_top'),*((rid,'robot_cam') for rid in ROBOTS if rid in selected)]
        requested_wall_s=time.monotonic()
        top_latch,batch_future=self.world.render_pair_snapshot_with_top_async(cameras)
        submitted_wall_s=time.monotonic()
        self.sequence+=1
        if self._top_capture_worker is None:
            self._top_capture_worker=ThreadPoolExecutor(max_workers=1,
                thread_name_prefix='dispatch-pair-top-jpeg')
        if self._capture_workers is None:
            self._capture_workers=ThreadPoolExecutor(max_workers=2,
                thread_name_prefix='dispatch-rgb')

        def materialize_top():
            rendered=top_latch.result(timeout=30.)
            cpu_started=time.monotonic()
            stream=BytesIO()
            Image.fromarray(rendered.rgb).save(stream,format='JPEG',quality=95)
            top=stream.getvalue()
            top_ref=image_record(self.out/'rgb'/f'{label}-top.jpg',self.out,top)
            return {'frame_id':rendered.frame_id,'observed_at_s':float(rendered.sim_time),
                'top_bytes':top,'shared_top_rgb':top_ref,'_rgb':rendered.rgb,
                'top_render_completed_wall_s':rendered.render_completed_wall_s,
                'top_cpu_started_wall_s':cpu_started,
                'top_materialized_wall_s':time.monotonic()}
        top_future=self._top_capture_worker.submit(materialize_top)

        def materialize_full():
            batch=batch_future.result(timeout=30.)
            top=top_future.result(timeout=30.)
            if (batch.frame_id!=top['frame_id'] or
                    abs(float(batch.sim_time)-top['observed_at_s'])>1e-9 or
                    batch.rgb[(None,'cctv_top')] is not top['_rgb'] or
                    set(batch.rgb)!=set(cameras)):
                raise RuntimeError('partial pair views have mismatched frozen provenance')
            encoded={}
            for rid in ROBOTS:
                if rid in selected:
                    stream=BytesIO()
                    Image.fromarray(batch.rgb[(rid,'robot_cam')]).save(
                        stream,format='JPEG',quality=95)
                    encoded[rid]=stream.getvalue()
            encoded_wall_s=time.monotonic()
            frames={}
            for rid in ROBOTS:
                frame={'top_bytes':top['top_bytes'],'frame_id':batch.frame_id,
                    'observed_at_s':float(batch.sim_time),
                    'shared_top_rgb':top['shared_top_rgb'],
                    'capture_mode':'render_ready',
                    'capture_requested_wall_s':requested_wall_s,
                    'snapshot_submitted_wall_s':submitted_wall_s,
                    'render_completed_wall_s':batch.render_completed_wall_s,
                    'encode_completed_wall_s':encoded_wall_s,
                    'top_render_completed_wall_s':top['top_render_completed_wall_s'],
                    'top_cpu_started_wall_s':top['top_cpu_started_wall_s'],
                    'top_materialized_wall_s':top['top_materialized_wall_s']}
                if rid in selected:
                    own=encoded[rid]
                    frame.update(own_bytes=own,
                        own_rgb=image_record(self.out/'rgb'/f'{label}-{rid}.jpg',self.out,own))
                frames[rid]=frame
            for frame in frames.values():
                frame['materialized_wall_s']=time.monotonic()
            return frames
        full_future=self._capture_workers.submit(materialize_full)
        return PairPartialCapture(top_future,full_future)

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
            if self._top_capture_worker:
                self._top_capture_worker.shutdown(wait=True,cancel_futures=True)
                self._top_capture_worker=None
            if self._capture_workers:
                self._capture_workers.shutdown(wait=True,cancel_futures=True)
                self._capture_workers=None
            for p in self.ports.values():p.stop()
        finally:
            if self.world:self.world.close();self.world=None
