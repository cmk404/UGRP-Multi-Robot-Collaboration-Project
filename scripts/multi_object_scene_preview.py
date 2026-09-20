"""Owns physics and evaluation-only visibility checks; no actor or motor policy."""
from __future__ import annotations

import itertools
from unittest.mock import patch

from scripts.research_dispatch_scene import DispatchScene
from sim.multi_object_scene import build_multi_object_xml
from sim.research_dispatch_arena import ROBOTS, FIXED_TOP


class MultiObjectScene(DispatchScene):
    def open(self):
        import mujoco
        import sim.multi_masterpi_production as production
        from scripts.probe_dual_grasp_sync import _plain_beam_xml
        original = _plain_beam_xml(production.build_multi_robot_xml)

        def builder(*args, **kwargs):
            kwargs['navigation_camera'] = False
            self.xml, self.manifest = build_multi_object_xml(original(*args, **kwargs), self.config)
            return self.xml

        self.out.mkdir(parents=True, exist_ok=False); (self.out/'rgb').mkdir()
        with patch.object(production, 'build_multi_robot_xml', builder):
            self.world = production.MultiMasterPiProductionV2(seed=self.config['seed'], width=960, height=720, render=self.render)
        w = self.world
        for rid, pose in self.config['setup_only']['spawns'].items():
            w.controllers[rid].set_base_pose_for_test(tuple(pose[:3]), pose[3])
        for item in self.config['setup_only']['objects'].values():
            jid = mujoco.mj_name2id(w.model, mujoco.mjtObj.mjOBJ_JOINT, item['joint_name'])
            q = int(w.model.jnt_qposadr[jid]); v = int(w.model.jnt_dofadr[jid])
            w.data.qpos[q:q+7] = [*item['position_m'], 1, 0, 0, 0]
            w.data.qvel[v:v+6] = 0
        w.data.eq_active[:] = 0
        mujoco.mj_forward(w.model, w.data)
        self.initial_contacts = self.contacts()
        # Same fixed folded-arm initialization as the preceding map preview.
        w._team_joint_move_servos({r: {1:2000,3:740,4:2320,5:1320,6:1500} for r in ROBOTS}, .6, settle_s=.4)
        self.look_observer((2.6,-4.8,3.4))
        (self.out/'scene.xml').write_text(self.xml)
        return self

    def entities(self):
        import mujoco
        m = self.world.model
        roots = {item['body_name']: oid for oid,item in self.config['setup_only']['objects'].items()}
        roots.update({rid+'__robot': rid for rid in ROBOTS})
        owners = {}
        for gid in range(m.ngeom):
            bid = int(m.geom_bodyid[gid])
            while bid:
                name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, bid)
                if name in roots:
                    owners[gid] = roots[name]; break
                bid = int(m.body_parentid[bid])
            else:
                name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, gid) or ''
                if name.startswith('dispatch_') and m.geom_contype[gid]: owners[gid] = 'map'
        return owners

    def contacts(self):
        """Exclude intended floor contacts and within-robot link contacts."""
        owners = self.entities(); data = self.world.data
        return [{'entities': [owners[int(c.geom1)],owners[int(c.geom2)]], 'depth_m': float(-c.dist)}
                for c in data.contact[:data.ncon] if c.dist < -1e-5 and
                int(c.geom1) in owners and int(c.geom2) in owners and owners[int(c.geom1)] != owners[int(c.geom2)]]

    def compiled_audit(self):
        import mujoco
        import numpy as np
        m, d = self.world.model, self.world.data
        result, addresses = {}, set()
        for oid, item in self.config['setup_only']['objects'].items():
            bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, item['body_name'])
            jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, item['joint_name'])
            gid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, item['body_name']+'_geom')
            if min(bid,jid,gid) < 0 or m.jnt_type[jid] != mujoco.mjtJoint.mjJNT_FREE:
                raise ValueError('missing independent dynamic cargo')
            q = int(m.jnt_qposadr[jid])
            if q in addresses: raise ValueError('cargo shares joint coordinates')
            addresses.add(q)
            if not np.allclose(m.geom_size[gid],item['half_extents_m'],atol=1e-10,rtol=0):
                raise ValueError('compiled cargo dimension mismatch')
            expected_mass = .196 if item['kind']=='beam' else .03
            if not np.isclose(m.body_mass[bid],expected_mass) or not m.geom_contype[gid] or not m.geom_conaffinity[gid]:
                raise ValueError('cargo dynamics mismatch')
            result[oid] = {'kind': item['kind'], 'body_id': bid, 'joint_id': jid, 'geom_id': gid,
                           'mass_kg': float(m.body_mass[bid]), 'position_m': d.xpos[bid].tolist(),
                           'initial_xy_drift_m': float(np.linalg.norm(d.xpos[bid,:2]-item['position_m'][:2]))}
        obstacles = []
        for box in self.config['static_map']['obstacles']:
            gid = mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'dispatch_'+box['id'])
            if gid < 0 or not m.geom_contype[gid] or not np.allclose(m.geom_size[gid],[*box['half_extents_m'],box['height_m']/2]):
                raise ValueError('compiled boundary/map mismatch')
            if not np.allclose(m.geom_pos[gid],[*box['center_m'],box['height_m']/2]):
                raise ValueError('compiled map pose mismatch')
            obstacles.append(box['id'])
        for did,dock in self.config['static_map']['docks'].items():
            for kind,slot in dock['slots'].items():
                gid = mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'dispatch_'+did+'_'+kind)
                if gid < 0 or m.geom_contype[gid] or m.geom_conaffinity[gid] or not np.allclose(m.geom_pos[gid,:2],slot['center_m']) or not np.allclose(m.geom_size[gid,:2],slot['half_extents_m']):
                    raise ValueError('compiled destination paint mismatch')
        if d.eq_active.any(): raise ValueError('weld must be OFF')
        inv = self.invariants()
        top = inv['policy_cameras']['cctv_top']
        if top != {'position': FIXED_TOP['position_m'], 'quaternion': FIXED_TOP['quaternion_wxyz'], 'fovy': FIXED_TOP['fov_y_deg']}:
            raise ValueError('fixed TOP calibration changed')
        # Record actual measured intrinsics and fisheye profile, beyond fovy.
        for name, row in inv['policy_cameras'].items():
            cid = mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_CAMERA,name)
            row.update(intrinsic=m.cam_intrinsic[cid].tolist(), sensorsize=m.cam_sensorsize[cid].tolist(),
                       resolution=m.cam_resolution[cid].tolist())
        return {'cargo': result, 'compiled_obstacles': obstacles, 'invariants': inv,
                'initial_contacts': self.initial_contacts, 'settled_contacts': self.contacts(),
                'settle_sim_seconds': float(d.time), 'transport_success': None}

    def visibility(self, audit):
        """Segmentation is privileged QA output only; never a student image/input."""
        import cv2
        import mujoco
        import numpy as np
        w = self.world

        def render():
            with w.physics_lock, w.render_lock:
                result = {oid: {} for oid in audit['cargo']}
                for name in ('top', *ROBOTS):
                    renderer = w.observer_renderer if name=='top' else w.renderer
                    opts = mujoco.MjvOption(); opts.geomgroup[:] = 1
                    if name!='top': opts = w.controllers[name]._robot_sensor_scene_option
                    camera = 'cctv_top' if name=='top' else name+'__robot_cam'
                    renderer.enable_segmentation_rendering()
                    try:
                        renderer.update_scene(w.data,camera=camera,scene_option=opts)
                        labels = renderer.render().copy()
                    finally:
                        renderer.disable_segmentation_rendering()
                    for oid, info in audit['cargo'].items():
                        mask = ((labels[:,:,0]==info['geom_id']) & (labels[:,:,1]==int(mujoco.mjtObj.mjOBJ_GEOM))).astype(np.uint8)
                        if name!='top':
                            mx,my = w.controllers[name]._robot_fisheye_map
                            mask = cv2.remap(mask,mx,my,cv2.INTER_NEAREST,borderMode=cv2.BORDER_CONSTANT)
                        row = {'visible_pixels': int(mask.sum())}
                        if name=='top':
                            gid = info['geom_id']; cid = mujoco.mj_name2id(w.model,mujoco.mjtObj.mjOBJ_CAMERA,camera)
                            corners = np.array(list(itertools.product((-1,1),repeat=3))) * w.model.geom_size[gid]
                            points = corners @ w.data.geom_xmat[gid].reshape(3,3).T + w.data.geom_xpos[gid]
                            local = (points-w.data.cam_xpos[cid]) @ w.data.cam_xmat[cid].reshape(3,3)
                            focal = w.observer_height/(2*math_tan_half_fov(w.model.cam_fovy[cid]))
                            uv = np.column_stack((w.observer_width/2+focal*local[:,0]/-local[:,2],
                                                  w.observer_height/2-focal*local[:,1]/-local[:,2]))
                            area = float(cv2.contourArea(cv2.convexHull(uv.astype(np.float32))))
                            row.update(projected_silhouette_area_px=area,
                                       approximate_visible_fraction=min(1.,row['visible_pixels']/max(area,1.)))
                        result[oid][name] = row
                return result
        return w._render_executor.submit(render).result(timeout=60.)


def math_tan_half_fov(fovy):
    import math
    return math.tan(math.radians(float(fovy))/2)
