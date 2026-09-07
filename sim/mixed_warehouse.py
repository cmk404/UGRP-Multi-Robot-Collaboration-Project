"""Live mixed task executor: participant-only reservations and shared physics.

One physics owner advances both the joint skill and robot-local solo state
machines. The request thread remains available for idle robots to claim work.
"""
from __future__ import annotations

from collections import deque
from dataclasses import replace
import copy
import threading
import time
import uuid

from harness.mixed_warehouse_protocol import MixedWarehouseReferee
from sim.crew_motion_metrics import CrewMotionRecorder
from sim.warehouse_mission import cargo_pose, warehouse_route_zones
from harness.task_recovery import RecoveryEvent, measurement_confidence, handling_decision


class MixedEngine:
    def __init__(self, world, destination="B", condition="llm_peer_comm", information_mode="legacy_shared"):
        self.world = world
        # A new episode must not inherit retired actuator traffic priorities.
        self.world._solo_traffic_tasks = {}
        self.id = uuid.uuid4().hex
        self.destination = destination
        self.condition = condition
        if information_mode not in {"legacy_shared", "strict_local"}:
            raise ValueError("UNKNOWN_INFORMATION_MODE")
        self.information_mode = information_mode
        self.revision = 1
        self.lock = world.physics_lock
        self.wake = threading.Event()
        self.stopping = False
        self.retired_robots = set()
        self.solo_tasks = {}
        self.solo_assignments = {}
        self.joint_queue = deque()
        self.joint_active = None
        self.completed = set()
        self.failed = {}
        self.help_requests={}
        self.history = []
        self.observations = {}
        self.memories = {rid: {} for rid in world.robot_ids}
        self.sequence = {rid: 0 for rid in world.robot_ids}
        self.scan_initialized = set()
        self.recovery_events={}
        self.recovery_assignments={}
        self._pause_tick=False
        self.joint_pause_started=None
        self.recovery_history=[]
        self.joint_replan_requested=False
        self.last_sample = -1.
        self.last_frame = -1.
        self.next_wall_tick = time.monotonic()
        self.peer_collision_events=[]
        self.peer_contact_seconds = {}
        self.terrain_contact_seconds = {}
        self.task_overlap_seconds=0.
        self.cargo_motion_overlap_seconds=0.
        self.last_cargo_sample={}
        self.geometry_owner={gid: next((rid for rid in world.robot_ids
            if (__import__('mujoco').mj_id2name(world.model,__import__('mujoco').mjtObj.mjOBJ_GEOM,gid) or '').startswith(rid+'__')),None)
            for gid in range(world.model.ngeom)}
        self.manifest = [{"cargo_id": s.cargo_id, "required_carriers": s.required_carriers,
                          "type": s.cargo_type, "label_color": s.label_color}
                         for s in world.warehouse_specs]
        self.referee = MixedWarehouseReferee(self.manifest, revision=1)
        self.recorder = CrewMotionRecorder()
        self.thread = threading.Thread(target=self._run, name="mixed-physics-owner", daemon=True)

    def start(self):
        self.thread.start()

    def retire(self, rid):
        """End this planner's consent lease without stopping unrelated motion."""
        if rid not in self.world.robot_ids: raise ValueError("UNKNOWN_ROBOT")
        with self.lock:
            self.retired_robots.add(rid)
            for author, proposal in tuple(self.referee.pending.items()):
                if rid in proposal.participants:
                    self.referee.pending.pop(author, None)
            return {"ok": True, "retired": rid}

    def status(self):
        with self.lock:
            active = [a.as_dict() for a in self.referee.active_assignments]
            busy = {rid for a in active for rid in a["participants"]}
            return {"ok": True, "episode_id": self.id, "revision": self.revision,
                    "sim_time":float(self.world.data.time),
                    "destination": self.destination, "world_seed": self.world.seed,
                    "manifest": copy.deepcopy(self.manifest),
                    "complete": len(self.completed) == len(self.manifest),
                    "help_requests":copy.deepcopy(self.help_requests),
                    "completed_ids": sorted(self.completed), "failed": dict(self.failed),
                    "available_ids": [s["cargo_id"] for s in self.manifest if s["cargo_id"] not in self.completed and s["cargo_id"] not in self.help_requests
                                      and all(a["cargo_id"] != s["cargo_id"] for a in active)],
                    "idle_robots": [r for r in self.world.robot_ids if r not in busy],
                    "active": active, "history": copy.deepcopy(self.history[-40:]),
                    "recovery_events": [e.public() for e in self.recovery_events.values() if not e.resolved],
                    "recovery_history": copy.deepcopy(self.recovery_history[-30:]),
                    "activities": {r: t.state for r, t in self.solo_tasks.items()}}

    def observe(self, rid):
        from sim.warehouse_observation import observe_robot, observe_robots_scan
        with self.lock:
            recovering=any(rid in e.participants and not e.resolved for e in self.recovery_events.values())
            if rid not in self.status()["idle_robots"] and not recovering:
                raise ValueError("ROBOT_BUSY")
            no_jobs = not self.referee.active_assignments
        # Never hold the engine lock across renderer broker calls.
        initial_scan = rid not in self.scan_initialized if self.information_mode == "strict_local" else no_jobs and not self.memories[rid]
        if initial_scan:
            ids = {s["cargo_id"] for s in self.manifest} if self.information_mode == "strict_local" else set(self.status()["available_ids"])
            samples = observe_robots_scan(self.world, (rid,), pending_ids=ids,
                                          sensor_seed=self.world.seed)
            sample = samples[rid]
            self.scan_initialized.add(rid)
        else:
            sample = observe_robot(self.world, rid, sensor_seed=self.world.seed)
        with self.lock:
            self.sequence[rid] += 1
            oid = f"{self.id}:{rid}:{self.sequence[rid]}"
            for c in sample.get("cargo", []):
                self.memories[rid][c["cargo_id"]] = {"cargo_id": c["cargo_id"], "observed_at": oid,
                    "observed_sim_time": c.get("observed_sim_time", float(self.world.data.time))}
            current = {c["cargo_id"]: c for c in sample.get("cargo", [])}
            cargo = [{"cargo_id": cid, "current_view": cid in current,
                      "range_m": current.get(cid, {}).get("range_m"),
                      "bearing_rad": current.get(cid, {}).get("bearing_rad"),
                      "confidence":current.get(cid,{}).get("confidence"),
                      "range_std_m":current.get(cid,{}).get("range_std_m"),
                      "observed_sim_time": self.memories[rid][cid].get("observed_sim_time"),
                      "source": "rgbd_aruco" if cid in current else "local_identity_memory"}
                     for cid in self.memories[rid] if self.information_mode == "strict_local" or cid not in self.completed]
            for c in cargo:
                c["measurement_gate"]=measurement_confidence(c)
                # A decoded marker is an identity measurement, not a scale or
                # load cell. Do not manufacture mass/width from fixture truth.
                c["handling_estimate"]=handling_decision()
            obs = {"robot_id": rid, "observation_id": oid, "revision": self.revision,
                   "sim_time": float(self.world.data.time),
                   "destination": self.destination, "cargo": cargo, "can_carry": True,
                   "controller_contract":"ideal_sim_geometry",
                   "capacity_contract":"operator_manifest; sensor mass/width unmeasured"}
            self.observations[rid] = obs
            self.referee.observe({"robot_id": rid, "observation_id": oid, "revision": self.revision,
                                  "cargo_ids": [c["cargo_id"] for c in cargo]})
            return copy.deepcopy(obs)

    def local_status(self, rid):
        """Actor view: own actuator evidence only; status() is supervisor-only."""
        if rid not in self.world.robot_ids:
            raise ValueError("UNKNOWN_ROBOT")
        with self.lock:
            active = next((a.as_dict() for a in self.referee.active_assignments if rid in a.participants), None)
            events = []
            for event in self.recovery_events.values():
                if event.resolved or rid not in event.participants:
                    continue
                public = event.public()
                known_to = event.evidence.get("known_to", list(event.participants))
                if rid not in known_to:
                    public["reason"] = "cause_unknown"
                    public["evidence"] = {"source": "own_controller_stop", "cause_known": False}
                else:
                    public["evidence"].pop("known_to", None)
                # No participant decisions or remote state enter this view.
                events.append(public)
            pending = self.referee.pending.get(rid)
            return {"ok": True, "robot_id": rid, "episode_id": self.id,
                    "revision": self.revision, "sim_time": float(self.world.data.time),
                    "active": active,
                    "pending": {"cargo_id": pending.cargo_id, "participants": list(pending.participants)} if pending else None,
                    "history": copy.deepcopy([e for e in self.history if rid in e.get("participants", [])][-20:]),
                    "recovery_events": copy.deepcopy(events)}

    def pause_assignment(self, assignment_id, reason, evidence):
        with self.lock:
            assignment=next((a for a in self.referee.active_assignments if a.assignment_id==assignment_id),None)
            if assignment is None:raise ValueError("UNKNOWN_ASSIGNMENT")
            if any(not e.resolved and self.recovery_assignments[k]==assignment_id for k,e in self.recovery_events.items()):
                raise ValueError("ALREADY_PAUSED")
            event=RecoveryEvent(tuple(assignment.participants),reason,copy.deepcopy(evidence))
            self.recovery_events[event.event_id]=event
            self.recovery_assignments[event.event_id]=assignment_id
            for rid in assignment.participants:
                if rid in self.solo_tasks:self.solo_tasks[rid].pause(event.event_id)
                self.world.robot(rid).set_motor_commands([0.,0.,0.,0.])
            self.recovery_history.append({"event":"paused","event_id":event.event_id,"assignment_id":assignment_id,"sim_time":float(self.world.data.time)})
            return event.public()

    def recover(self,rid,event_id,action):
        with self.lock:
            event=self.recovery_events.get(event_id)
            if event is None:raise ValueError("UNKNOWN_RECOVERY_EVENT")
            decision=event.submit(rid,event_id,action)
            if decision is not None:
                try:
                    for participant in event.participants:
                        if participant in self.solo_tasks:
                            self.solo_tasks[participant].recover(event_id,decision)
                    if self.joint_active and self.recovery_assignments[event_id]==self.joint_active.assignment_id:
                        if decision in {"cancel","request_help"}:
                            raise ValueError("JOINT_SAFE_HANDOFF_REQUIRED")
                        if decision=="replan":
                            self.joint_replan_requested=True
                            # Navigation tasks retain their own goals and use the
                            # latest low-level obstacle map. No role is changed.
                            crew=getattr(self.world,"_warehouse_crew",None)
                            if crew:
                                for participant in event.participants:
                                    task=crew.tasks.get(participant)
                                    if task:crew.navigate(participant,task.path[-1],crew.activities[participant],final_yaw=task.final_yaw)
                except Exception:
                    event.resolved=False;event.decisions.clear();raise
                self.recovery_history.append({"event":"recovered","action":decision,"event_id":event_id,"sim_time":float(self.world.data.time)})
            return {"ok":True,"resolved":event.resolved}

    def joint_is_paused(self):
        return self.joint_active is not None and any(not e.resolved and self.recovery_assignments[k]==self.joint_active.assignment_id
                                                    for k,e in self.recovery_events.items())

    def service_joint_pause(self):
        if self._pause_tick:return
        paused_at=float(self.world.data.time)
        while self.joint_is_paused() and not self.stopping:
            self._pause_tick=True
            try:self.world._physics_step_for(self.world.robot("r1"))
            finally:self._pause_tick=False
        paused_duration=float(self.world.data.time)-paused_at
        crew=getattr(self.world,"_warehouse_crew",None)
        if crew and paused_duration:
            crew.started+=paused_duration
            crew.paused_seconds=getattr(crew,"paused_seconds",0.)+paused_duration

    def submit(self, proposals, reports=()):
        with self.lock:
            if self.stopping: raise ValueError("EPISODE_CLOSED")
            for p in proposals:
                if self.retired_robots.intersection(p.get("participants", [])):
                    raise ValueError("PARTICIPANT_UNAVAILABLE")
                if p.get("destination") != self.destination or p.get("cargo_id") in self.completed:
                    raise ValueError("INVALID_TASK_GOAL")
            if self.condition != "llm_no_comm":
                for report in reports:
                    sender, cid = report["sender_id"], report["cargo_id"]
                    obs = self.referee.observations.get(sender)
                    if cid not in self.completed and obs is not None and cid in obs.cargo_ids:
                        self.referee.report_peer_observation(sender, cid, report.get("recipients"))
            verdict = self.referee.submit(proposals)
            for rejection in verdict.rejected:
                p = rejection.proposal
                self.history.append({"event": "claim_rejected", "sim_time": float(self.world.data.time),
                    "assignment_id": "proposal:" + p.robot_id + ":" + p.observation_id,
                    "cargo_id": p.cargo_id, "participants": [p.robot_id],
                    "requested_participants": list(p.participants), "reason": rejection.reason})
            for a in verdict.accepted:
                self.history.append({"event": "accepted", "sim_time": float(self.world.data.time), **a.as_dict()})
                if len(a.participants) == 2:
                    self.joint_queue.append(a)
                else:
                    try:
                        self._start_solo(a)
                    except Exception as exc:
                        self._complete(a, False, "SOLO_START_FAILED:"+str(exc))
            self.wake.set()
            return {"ok": True, "accepted": [a.as_dict() for a in verdict.accepted],
                    "rejected": [{"robot_id": r.proposal.robot_id, "reason": r.reason} for r in verdict.rejected],
                    **self.status()}

    def _spec_for(self, assignment):
        spec = self.world.warehouse_spec_by_id[assignment.cargo_id]
        pose = cargo_pose(self.world.data, self.world.model, spec)
        updated = replace(spec, carriers=tuple(assignment.participants), scout="",
                          start_xyz=tuple(pose["position"]),
                          goal_xyz=self.world.warehouse_zone_positions[spec.cargo_id][assignment.destination])
        self.world._replace_warehouse_spec(updated)
        return updated

    def _start_solo(self, assignment):
        from sim.solo_cargo_task import SoloCargoTask
        spec = self._spec_for(assignment)
        rid = assignment.participants[0]
        self.solo_tasks[rid] = SoloCargoTask(self.world, spec, rid, (getattr(self.world, "warehouse_source_zone", "A"), assignment.destination))
        self.solo_assignments[rid] = assignment

    def before_step(self):
        if self.stopping:
            raise RuntimeError("MIXED_EXECUTION_CANCELLED")
        with self.lock:
            if self.joint_is_paused():
                for rid in self.joint_active.participants:self.world.robot(rid).set_motor_commands([0.,0.,0.,0.])
            for rid, task in tuple(self.solo_tasks.items()):
                if not task.done:
                    task.before_step()

    def after_step(self):
        with self.lock:
            # Count shallow sustained contact too; sparse deep-penetration
            # events alone hid the earlier 76-second contact deadlock.
            touching = set()
            for contact in self.world.data.contact:
                a, b = self.geometry_owner.get(int(contact.geom1)), self.geometry_owner.get(int(contact.geom2))
                if a and b and a != b and float(contact.dist) <= 0.:
                    touching.add("/".join(sorted((a, b))))
            terrain_names = {t.terrain_id for t in self.world.warehouse_terrain_specs}
            terrain_touching = set()
            mj = __import__('mujoco')
            for contact in self.world.data.contact:
                if float(contact.dist) > 0.: continue
                for body_geom, terrain_geom in ((int(contact.geom1), int(contact.geom2)), (int(contact.geom2), int(contact.geom1))):
                    name = mj.mj_id2name(self.world.model, mj.mjtObj.mjOBJ_GEOM, terrain_geom)
                    owner = self.geometry_owner.get(body_geom)
                    if owner and name in terrain_names:
                        terrain_touching.add(owner + "/" + name)
            for pair in terrain_touching:
                self.terrain_contact_seconds[pair] = self.terrain_contact_seconds.get(pair, 0.) + float(self.world.model.opt.timestep)
            for pair in touching:
                self.peer_contact_seconds[pair] = self.peer_contact_seconds.get(pair, 0.) + float(self.world.model.opt.timestep)
            if self.joint_active and self.solo_tasks:
                self.task_overlap_seconds+=float(self.world.model.opt.timestep)
            for rid, task in tuple(self.solo_tasks.items()):
                if task.help_required and not task.paused:
                    assignment=self.solo_assignments[rid]
                    self.pause_assignment(assignment.assignment_id,'overload',task.load_estimate)
                if hasattr(task, "after_step"):
                    task.after_step()
                if task.done:
                    assignment = self.solo_assignments.pop(rid)
                    self._complete(assignment, task.ok, task.reason)
                    del self.solo_tasks[rid]
            now = float(self.world.data.time)
            if now-self.last_sample >= .1:
                old_sample=self.last_sample
                self.last_sample = now
                self.recorder.observe(now, {r: {"position": tuple(float(x) for x in self.world.robot(r).base_xyz()),
                                                "yaw": float(self.world.robot(r).base_rpy()[2])}
                                            for r in self.world.robot_ids})
                cargo_sample={s.cargo_id:tuple(cargo_pose(self.world.data,self.world.model,s)["position"])
                              for s in self.world.warehouse_specs}
                if old_sample>=0 and self.last_cargo_sample:
                    moved=[cid for cid,pos in cargo_sample.items()
                           if __import__('math').dist(pos[:2],self.last_cargo_sample[cid][:2])/(now-old_sample)>.02]
                    if any(self.referee.required_carriers[c]==2 for c in moved) and any(self.referee.required_carriers[c]==1 for c in moved):
                        self.cargo_motion_overlap_seconds+=now-old_sample
                self.last_cargo_sample=cargo_sample
                for contact in self.world.data.contact:
                    a,b=self.geometry_owner.get(int(contact.geom1)),self.geometry_owner.get(int(contact.geom2))
                    if a and b and a!=b and float(contact.dist)<-.002:
                        self.peer_collision_events.append({"sim_time":now,"robots":[a,b],"penetration":float(contact.dist)})
                if hasattr(self, "sample_sink"):
                    self.sample_sink({"sim_time":now,"tasks":{r:t.state for r,t in self.solo_tasks.items()},
                                      "cargo":{s.cargo_id:list(cargo_pose(self.world.data,self.world.model,s)["position"])
                                               for s in self.world.warehouse_specs}})
        if self.world.frame_callback and now-self.last_frame >= .10:
            self.last_frame = now
            self.world.frame_callback()
        # Advance simulation near real time so completion permits another
        # independent LLM decision while the other task is still in progress.
        self.next_wall_tick += float(self.world.model.opt.timestep)
        if time.monotonic()-self.next_wall_tick>1.:
            self.next_wall_tick=time.monotonic()
        delay = self.next_wall_tick-time.monotonic()
        if delay > 0:
            time.sleep(min(delay, .01))

    def _complete(self, assignment, ok, reason):
        if ok:
            self.completed.add(assignment.cargo_id)
        else:
            self.failed[assignment.cargo_id] = reason
            if reason=="ASSISTANCE_REQUESTED":
                self.help_requests[assignment.cargo_id]={"requester":assignment.participants[0],
                    "status":"SAFE_HANDOFF_NEEDED","supported_next_action":"operator-selected assistance mode"}
        # Referee lifecycle acknowledgement is actuator evidence, not an LLM assertion.
        if ok:
            for rid in assignment.participants:
                self.referee.complete(assignment.assignment_id, rid, self.revision)
            self.referee.release(assignment.assignment_id, self.revision)
        else:
            self.referee.abort(assignment.assignment_id, reason)
        self.history.append({"event": "completed" if ok else "failed", "sim_time": float(self.world.data.time),
                             "reason": reason, **assignment.as_dict()})
        self.world.warehouse_status = "SUCCESS" if len(self.completed)==len(self.manifest) else "RUNNING"
        self.wake.set()

    def _run(self):
        try:
            while not self.stopping:
                with self.lock:
                    assignment = self.joint_queue.popleft() if self.joint_queue else None
                    self.joint_active = assignment
                    solo = bool(self.solo_tasks)
                if assignment:
                    try:
                        spec = self._spec_for(assignment)
                        self.world._transport_warehouse_cargo(spec, (getattr(self.world, "warehouse_source_zone", "A"), assignment.destination))
                        ok, reason = True, "JOINT_DELIVERED"
                    except Exception as exc:
                        ok, reason = False, str(exc)
                        for rid in assignment.participants:
                            self.world.robot(rid).set_motor_commands([0.,0.,0.,0.])
                        self.world._release_cargo_constraints(assignment.cargo_id)
                    with self.lock:
                        self._complete(assignment, ok, reason)
                        self.joint_active = None
                elif solo:
                    self.world._physics_step_for(self.world.robot("r1"))
                else:
                    self.wake.wait(.05); self.wake.clear()
                    self.next_wall_tick = time.monotonic()
        except Exception as exc:
            with self.lock:
                self.failed["engine"] = f"{type(exc).__name__}: {exc}"
                self.stopping = True
        finally:
            for robot in self.world.controllers.values():
                robot.set_motor_commands([0.,0.,0.,0.])

    def close(self):
        self.stopping = True
        self.wake.set()
        if self.thread.is_alive() and threading.current_thread() is not self.thread:
            self.thread.join(timeout=120)
            if self.thread.is_alive():
                raise RuntimeError("mixed physics task did not stop")


def dispatch_mixed(world, rid, request):
    operation = request.get("operation")
    if operation == "mixed_begin":
        if world.warehouse_layout not in {"mixed", "arena"}:
            raise ValueError("MIXED_SCENE_REQUIRED: start worker with UGRP_WAREHOUSE_LAYOUT=mixed")
        previous = getattr(world, "_mixed_engine", None)
        if previous:
            previous.close()
        destination = request.get("destination_zone", getattr(world, "warehouse_destination_zone", "B"))
        source = getattr(world, "warehouse_source_zone", "A")
        warehouse_route_zones(source, destination)
        if request.get("condition", "llm_peer_comm") not in {"rule","llm_peer_comm","llm_no_comm","llm_structured_comm"}:
            raise ValueError("UNKNOWN_MIXED_CONDITION")
        for spec in world.warehouse_specs:
            pose=cargo_pose(world.data,world.model,spec)
            if world._warehouse_zone_for_position(spec.cargo_id,pose["position"]) != source:
                raise ValueError("MIXED_RESET_REQUIRED: cargo must begin in source zone")
        engine = MixedEngine(world, destination, request.get("condition", "llm_peer_comm"), request.get("information_mode", "legacy_shared"))
        world._sync_warehouse_goal_specs([s.cargo_id for s in world.warehouse_specs], destination)
        world.warehouse_mission = {"source_zone": source, "destination_zone": destination,
                                   "decision_mode": "mixed_participant_consensus", "mission_id": engine.id}
        world.warehouse_status = "RUNNING"
        world.warehouse_trace = []
        world._mixed_engine = engine
        engine.start()
        return engine.status()
    engine = getattr(world, "_mixed_engine", None)
    if engine is None or request.get("episode_id") != engine.id:
        raise ValueError("STALE_MIXED_EPISODE")
    if operation == "mixed_status":
        return engine.status()
    if operation == "mixed_local_status":
        return engine.local_status(rid)
    if operation == "mixed_retire":
        return engine.retire(rid)
    if operation == "mixed_observe":
        return engine.observe(rid)
    if operation == "mixed_submit":
        return engine.submit(request.get("proposals", []), request.get("reports", []))
    if operation == "mixed_local_submit":
        proposals = request.get("proposals", [])
        if any(p.get("robot_id") != rid for p in proposals):
            raise ValueError("FOREIGN_PROPOSAL")
        result = engine.submit(proposals, request.get("reports", []))
        return {"ok": result["ok"],
                "accepted": [a for a in result["accepted"] if rid in a["participants"]],
                "rejected": [r for r in result["rejected"] if r["robot_id"] == rid]}
    if operation == "mixed_pause":
        return engine.pause_assignment(request["assignment_id"],request.get("reason","operator_pause"),request.get("evidence",{}))
    if operation == "mixed_obstacle":
        import mujoco
        from sim.warehouse_mission import TerrainSpec
        position=request.get("position_xy")
        if (not isinstance(position,list) or len(position)!=2 or
                not all(isinstance(v,(int,float)) for v in position) or
                not (-.5<=position[0]<=3 and -2.2<=position[1]<=2.2)):
            raise ValueError("INVALID_OBSTACLE_POSITION")
        known_to = request.get("known_to", list(world.robot_ids))
        if not isinstance(known_to, list) or not known_to or set(known_to)-set(world.robot_ids):
            raise ValueError("INVALID_EVENT_OBSERVERS")
        event=engine.pause_assignment(request["assignment_id"],"obstacle_changed",
                                       {"source":"controlled_scenario_event","position_xy":position,"requires_replan":True,"known_to":known_to})
        with world.physics_lock:
            gid=mujoco.mj_name2id(world.model,mujoco.mjtObj.mjOBJ_GEOM,"mixed_event_barrier")
            world.model.geom_pos[gid]=[position[0],position[1],.06]
            world.warehouse_terrain_specs=tuple(t for t in world.warehouse_terrain_specs if t.terrain_id!="mixed_event_barrier")+(
                TerrainSpec("mixed_event_barrier","barrier",tuple(position),(.06,.09),.12,False,float("inf")),)
            mujoco.mj_forward(world.model,world.data)
        return event
    if operation == "mixed_recover":
        return engine.recover(rid,request["event_id"],request["action"])
    if operation == "mixed_stop":
        engine.close()
        return engine.status()
    raise ValueError("UNKNOWN_MIXED_OPERATION")
