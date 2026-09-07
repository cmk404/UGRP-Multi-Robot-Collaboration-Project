"""Nonblocking single-robot grasp/carry/release of a small cargo body."""
from __future__ import annotations

from dataclasses import replace
import math
import numpy as np

from sim.adaptive_warehouse import TerrainObservation, plan_local_path
from sim.crew_navigation import DynamicPeerYield, ForwardPathController
from sim.masterpi_dynamics_v2 import FORWARD_PATTERN, LEFT_PATTERN
from sim.warehouse_mission import WAREHOUSE_ZONES, cargo_pose, evaluate_cargo_delivery


def _transport_heading(spec):
    delta=np.asarray(spec.goal_xyz[:2],dtype=float)-np.asarray(spec.start_xyz[:2],dtype=float)
    if np.linalg.norm(delta)<1e-6:raise ValueError("WAREHOUSE_ROUTE_HAS_ZERO_LENGTH")
    direction=delta/np.linalg.norm(delta)
    return direction,math.atan2(direction[1],direction[0])


def _navigation_bounds(world,target,margin=.65):
    if getattr(world,"warehouse_layout","standard")!="arena":
        return (-.6,3.4,-2.3,2.3)
    points=[np.asarray(target,dtype=float)]
    zones=getattr(world,"warehouse_zones",None) or WAREHOUSE_ZONES
    for zone in zones.values():
        center,half=np.asarray(zone.center_xy),np.asarray(zone.half_extents_xy)
        points.extend((center-half,center+half))
    points.extend(np.asarray(world.robot(rid).base_xyz()[:2],dtype=float) for rid in world.robot_ids)
    points=np.asarray(points)
    return (float(points[:,0].min()-margin),float(points[:,0].max()+margin),
            float(points[:,1].min()-margin),float(points[:,1].max()+margin))


def _future_joint_parking_obstacles(world):
    """Reserve ideal-SIM carrier release lanes before those robots arrive."""
    if getattr(world,"warehouse_layout","standard")!="arena":
        return ()
    plank=next((spec for spec in world.warehouse_specs
                if spec.required_carriers==2),None)
    if plank is None:
        return ()
    long_axis=np.asarray((-math.sin(plank.goal_yaw_rad),
                          math.cos(plank.goal_yaw_rad)),dtype=float)
    goal=np.asarray(plank.goal_xyz[:2],dtype=float)
    # Each rectangle contains the carrier centre's complete post-release
    # retreat from +/-0.36 m through its +/-0.515 m parking anchor. Its axis
    # extents are that segment's projection, with a .10 m chassis floor.
    # Inflating by the .30 m solo footprint retains the .40 m peer boundary
    # without covering compact arena goal docks.
    retreat_half=tuple(max(.10,.155*abs(float(axis))) for axis in long_axis)
    return tuple(TerrainObservation(
        terrain_id=f"future_joint_parking_{index}",kind="reserved_peer_lane",
        center_xy=tuple(float(v) for v in goal+sign*.515*long_axis),
        half_extents_xy=retreat_half,height_m=.20,traversable=False,
        cost_multiplier=math.inf,
    ) for index,sign in enumerate((-1.,1.),1))


class SoloCargoTask:
    def __init__(self, world, spec, robot_id, route):
        if spec.required_carriers != 1 or tuple(spec.carriers) != (robot_id,):
            raise ValueError("SOLO_CARGO_AWARD_REQUIRED")
        self.world,self.spec,self.rid,self.route=world,spec,robot_id,tuple(route)
        self.robot=world.robot(robot_id)
        self.precision=world._precision_module()
        self.state="APPROACH"
        self.done=self.ok=False
        self.reason="RUNNING"
        self.started=float(world.data.time)
        self.traffic=DynamicPeerYield(robot_id,self.started)
        if not hasattr(world,"_solo_traffic_tasks"):
            world._solo_traffic_tasks={}
        world._solo_traffic_tasks[robot_id]=self
        self.phase_started=self.started
        self.radius=.165
        self.hover={6:1500,1:2000,**self.precision.solve_ik(16.5,8.,-90.)}
        self.low={6:1500,**self.precision.solve_ik(16.5,-.6,-90.)}
        self.last_motor=np.zeros(4)
        self.servo_start=dict(self.robot.servo_command_pulses)
        self.pose_target=dict(self.hover)
        self.pose_duration=.6
        pos=cargo_pose(world.data,world.model,spec)["position"]
        self.transport_direction,self.transport_yaw=_transport_heading(spec)
        if getattr(world,"warehouse_layout","standard")=="arena":
            approach=np.asarray(pos[:2])-self.transport_direction*.18
            approach_yaw=self.transport_yaw
        else:
            approach=(pos[0]-.18,pos[1]);approach_yaw=0.
        self.navigator=self._path(approach,final_yaw=approach_yaw)
        self.grasp_deadline=None
        self.target_center=None
        self.paused=False
        self.pause_event=None
        self.paused_at=None
        self.load_sensor=None
        self.load_estimate={"available":False,"reason":"NOT_WEIGHED"}
        self.help_required=False
        self._event("solo_started")

    def _event(self, phase, **fields):
        self.world._record_warehouse_phase(phase,cargo_id=self.spec.cargo_id,robot_id=self.rid,**fields)

    def _path(self, target, final_yaw=0.):
        carrying = self.world._cargo_constraint_active(self.spec.cargo_id, self.rid)
        obstacles=self.world._warehouse_navigation_observations(
            exclude_robot=self.rid,
            exclude_cargo=self.spec.cargo_id if carrying else None,
        )
        obstacles=tuple(obstacles)+_future_joint_parking_obstacles(self.world)
        # The traffic forecast yields below 0.40 m center separation.  Match
        # the planner's swept footprint to that same boundary; otherwise a
        # stationary peer can produce an endlessly repeated path that the
        # actuator correctly refuses to execute (0.13 + 0.27 = 0.40 m).
        obstacles=tuple(replace(o,half_extents_xy=(max(.27,o.half_extents_xy[0]),
                                                   max(.27,o.half_extents_xy[1])))
                        if o.kind=="peer" else o for o in obstacles)
        # The arena carry pose holds the payload about .165 m ahead of the
        # chassis.  Cover the deployed arm/payload while turning, plus one
        # planner cell of tolerance.  Pickup approach remains tight because
        # its target cargo is deliberately included as the terminal contact.
        loaded_arena = (getattr(self.world,"warehouse_layout","standard")=="arena"
                        and carrying)
        footprint = (.30,.30) if loaded_arena else (.13,.13)
        path=plan_local_path(tuple(self.robot.base_xyz()[:2]),target,obstacles,
                             footprint_xy=footprint,resolution_m=.12,
                             bounds=_navigation_bounds(self.world,target))
        # The final base pose places the grip site at the cargo center. Keep
        # this contact approach tighter than ordinary route acceptance so the
        # subsequent fine alignment does not push an already touched cargo.
        return ForwardPathController(path,final_yaw=final_yaw,tolerance_m=.035,
                                     final_anchor_tolerance_m=.01)

    def _navigate_tick(self, dt):
        command=self.navigator.update(self.robot.base_xyz()[:2],self.robot.base_rpy()[2],dt)
        peers=[]
        traffic_tasks=getattr(self.world,"_solo_traffic_tasks",{})
        for peer_id in self.world.robot_ids:
            if peer_id==self.rid:
                continue
            peer=self.world.robot(peer_id)
            task=traffic_tasks.get(peer_id)
            # A completed/paused solo task and any joint or otherwise
            # externally controlled robot are physical obstacles.  Give them
            # right of way because this controller cannot make them yield.
            priority=(task.traffic.priority if task is not None and not task.done and not task.paused
                      else (-math.inf,str(peer_id)))
            peers.append((peer_id,peer.base_xyz()[:2],peer.base_rpy()[2],
                          peer.motor_command,priority))
        yielding=self.traffic.should_yield(self.robot.base_xyz()[:2],self.robot.base_rpy()[2],command,peers)
        now=float(self.world.data.time)
        if self.traffic.note(yielding,now):
            target=self.navigator.path[-1]
            final_yaw=self.navigator.final_yaw
            try:
                self.navigator=self._path(target,final_yaw=final_yaw)
                self._event("solo_yield_replanned",waypoints=[list(p) for p in self.navigator.path])
            except ValueError:
                pass
        return np.zeros(4) if yielding else command

    def _phase(self, state, pose=None, duration=.6):
        self.state=state;self.phase_started=float(self.world.data.time)
        self.servo_start=dict(self.robot.servo_command_pulses)
        self.pose_target=dict(pose or {})
        self.pose_duration=duration
        self._event("solo_phase",state=state)

    def _servo_tick(self):
        u=min(1.,(float(self.world.data.time)-self.phase_started)/max(.001,self.pose_duration))
        if self.pose_target:
            self.robot.set_servo_pulses({k:int(round(self.servo_start.get(k,v)+(v-self.servo_start.get(k,v))*u))
                                         for k,v in self.pose_target.items()})
        return u>=1.

    def _align(self, target_xy, tolerance=.003):
        grip=np.asarray(self.robot.site_xyz("grip_site")[:2]); delta=np.asarray(target_xy)-grip
        if np.linalg.norm(delta)<=tolerance:
            return True
        yaw=self.robot.base_rpy()[2];cy,sy=math.cos(yaw),math.sin(yaw)
        local=(cy*delta[0]+sy*delta[1],-sy*delta[0]+cy*delta[1])
        command=FORWARD_PATTERN*local[0]+LEFT_PATTERN*local[1]
        self.robot.set_motor_commands(np.clip(command*8.,-.18,.18))
        return False

    def before_step(self):
        if self.done:return
        if self.paused:
            self.robot.set_motor_commands(np.zeros(4))
            return
        try:self._tick()
        except Exception as exc:self._fail(str(exc))

    def pause(self,event_id):
        if self.paused or self.done:raise ValueError("TASK_NOT_PAUSABLE")
        self.paused=True;self.pause_event=event_id;self.paused_at=float(self.world.data.time)
        self.robot.set_motor_commands(np.zeros(4))

    def recover(self,event_id,action):
        if not self.paused or self.pause_event!=event_id:raise ValueError("STALE_RECOVERY_EVENT")
        if action in {"cancel","request_help"}:
            self.exit_reason="ASSISTANCE_REQUESTED" if action=="request_help" else "CANCELLED_BY_AGENT"
            self.paused=False;self.pause_event=None;self.help_required=False
            self.started+=float(self.world.data.time)-self.paused_at;self.paused_at=None
            self._phase("SAFE_LOWER",self.low,.7)
            return
        if action not in {"resume","replan"}:raise ValueError("INVALID_RECOVERY_ACTION")
        if self.help_required:raise ValueError("OVERLOAD_REQUIRES_ASSISTANCE")
        if action=="replan" and self.state in {"APPROACH","CARRY"}:
            self.navigator=self._path(self.navigator.path[-1],final_yaw=self.navigator.final_yaw)
        delay=float(self.world.data.time)-self.paused_at
        self.started+=delay;self.phase_started+=delay
        self.paused=False;self.pause_event=None;self.paused_at=None

    def _tick(self):
        now=float(self.world.data.time);dt=float(self.world.model.opt.timestep)
        budget = (60. + 40.*float(np.linalg.norm(np.asarray(self.spec.goal_xyz[:2])-np.asarray(self.spec.start_xyz[:2])))
                  if getattr(self.world,"warehouse_layout","standard")=="arena" else 90.)
        if now-self.started>budget:raise RuntimeError("SOLO_TIMEOUT:"+self.state)
        self.robot.set_motor_commands(np.zeros(4))
        servo_done=self._servo_tick()
        if self.state=="APPROACH":
            self.robot.set_motor_commands(self._navigate_tick(dt))
            if self.navigator.done and servo_done:self._phase("ALIGN")
        elif self.state=="ALIGN":
            pos=cargo_pose(self.world.data,self.world.model,self.spec)["position"]
            if self._align(pos[:2],.004):self._phase("LOWER",self.low,.7)
        elif self.state=="LOWER" and servo_done:
            self._phase("FINAL_ALIGN")
        elif self.state=="FINAL_ALIGN":
            pos=cargo_pose(self.world.data,self.world.model,self.spec)["position"]
            if self._align(pos[:2],.003):self._phase("CLOSE",{1:1500},.6)
        elif self.state=="CLOSE" and servo_done:
            contact=self.robot.finger_cargo_contact(self.spec.cargo_id)
            if contact["bilateral"]:
                self.world._activate_cargo_constraint(self.spec.cargo_id,self.robot)
                self._event("solo_contact_verified",bilateral=True)
                self._phase("LIFT",{**self.hover,1:1500},.7)
            elif now-self.phase_started>2.:
                raise RuntimeError("SOLO_BILATERAL_CONTACT_MISSING")
        elif self.state=="LIFT" and servo_done:
            if cargo_pose(self.world.data,self.world.model,self.spec)["position"][2]<.04:
                raise RuntimeError("SOLO_LIFT_FAILED")
            from sim.wrist_load_sensor import WristLoadSensor
            self.load_sensor=WristLoadSensor()
            self._phase("WEIGH")
        elif self.state=="WEIGH":
            self.load_sensor.sample(self.world,self.rid)
            if now-self.phase_started<.4:return
            self.load_estimate=self.load_sensor.estimate()
            if not self.load_estimate.get('available'):return
            if self.load_estimate['mass_kg']+2*self.load_estimate['mass_std_kg']>.10:
                self.help_required=True
                self._phase("AWAIT_HELP")
                return
            if getattr(self.world,"warehouse_layout","standard")=="arena":
                target=np.asarray(self.spec.goal_xyz[:2])-self.transport_direction*.165
                final_yaw=self.transport_yaw
            else:
                target=(self.spec.goal_xyz[0]-.165,self.spec.goal_xyz[1]);final_yaw=0.
            self.navigator=self._path(target,final_yaw=final_yaw)
            self._phase("CARRY")
        elif self.state=="CARRY":
            if not self.world._cargo_constraint_active(self.spec.cargo_id,self.rid):
                raise RuntimeError("SOLO_PAYLOAD_LOST")
            self.robot.set_motor_commands(self._navigate_tick(dt))
            if self.navigator.done:self._phase("DOCK")
        elif self.state=="DOCK":
            if self._align(self.spec.goal_xyz[:2],.025):self._phase("LOWER_GOAL",self.low,.7)
        elif self.state=="LOWER_GOAL" and servo_done:
            self._phase("OPEN",{1:2000},.5)
        elif self.state=="OPEN" and servo_done:
            self.world._release_cargo_constraints(self.spec.cargo_id)
            self._phase("RETRACT",self.hover,.6)
        elif self.state=="RETRACT" and servo_done:
            self._phase("VERIFY")
        elif self.state=="VERIFY" and now-self.phase_started>.6:
            pose=cargo_pose(self.world.data,self.world.model,self.spec)
            result=evaluate_cargo_delivery(pose,self.spec)
            jadr=int(self.world.model.jnt_dofadr[self.world.warehouse_joint_ids[self.spec.cargo_id]])
            velocity=self.world.data.qvel[jadr:jadr+6]
            if not result["success"] or np.linalg.norm(velocity[:3])>.015:
                raise RuntimeError("SOLO_DELIVERY_NOT_VERIFIED:"+str(result))
            self.done=self.ok=True;self.reason="SOLO_DELIVERED";self.state="DONE"
            self._event("solo_delivered",stable=True)
        elif self.state=="SAFE_LOWER" and servo_done:
            self._phase("SAFE_OPEN",{1:2000},.5)
        elif self.state=="SAFE_OPEN" and servo_done:
            self.world._release_cargo_constraints(self.spec.cargo_id)
            self._phase("SAFE_RETRACT",self.hover,.6)
        elif self.state=="SAFE_RETRACT" and servo_done:
            self._fail(self.exit_reason)

    def _fail(self, reason):
        self.done=True;self.ok=False;self.reason=reason;self.state="FAILED"
        self.robot.set_motor_commands(np.zeros(4))
        self.world._release_cargo_constraints(self.spec.cargo_id)
        self._event("solo_failed",reason=reason)
