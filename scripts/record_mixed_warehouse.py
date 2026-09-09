"""Record mixed solo/joint cargo work and its independent agent decisions."""
from pathlib import Path
import argparse
import json
import threading

from harness.gemini_proxy import GeminiProxyCompleter
from harness.warehouse_runtime import LocalEnvironment
from harness.mixed_warehouse_runtime import MixedLLMPolicy, MixedRulePolicy, run_mixed_episode
from scripts.record_parallel_warehouse import CrewVideo
from sim.multi_masterpi_production import MultiMasterPiProductionV2


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--condition",choices=("rule","llm_peer_comm"),default="llm_peer_comm")
    parser.add_argument("--output",required=True)
    parser.add_argument("--scenario",choices=("normal","joint_obstacle"),default="normal")
    parser.add_argument("--model", default="gemini-3.8-flash")
    parser.add_argument("--timeout", type=float, default=240)
    parser.add_argument("--max-calls", type=int, default=12)
    args=parser.parse_args()
    if args.timeout <= 0 or args.max_calls <= 0:
        parser.error("timeout and max-calls must be positive")
    out=Path(args.output).resolve();out.mkdir(parents=True,exist_ok=False)
    world=MultiMasterPiProductionV2(warehouse_layout="mixed",render=True)
    try:
        video=CrewVideo(world,out/"mixed-solo-joint-1x.mp4")
    except BaseException:
        world.close()
        raise
    sink=(out/"cargo-motion.jsonl").open("w")
    dialogue=(out/"dialogue.jsonl").open("w")
    dialogue_lock=threading.Lock()
    def on_message(robot_id,message):
        # Sample the physics clock at delivery, not the unrelated LLM wall clock.
        with world.physics_lock, dialogue_lock:
            sim_time=float(world.data.time)
            dialogue.write(json.dumps({"robot_id":robot_id,"message":message,
                "sim_time":sim_time,"video_time":max(0.,sim_time-video.start),
                "source":"agent_message_callback","condition":args.condition},ensure_ascii=False)+"\n")
            dialogue.flush()
        print(robot_id,message,flush=True)
    video_lock=threading.Lock()
    def capture():
        with video_lock:video.capture()
    class Environment(LocalEnvironment):
        def request(self,rid,request):
            result=super().request(rid,request)
            if request["operation"]=="mixed_begin":
                world._mixed_engine.sample_sink=lambda row:sink.write(json.dumps(row)+"\n")
            return result
    try:
        video.capture(force=True);world.frame_callback=capture
        policies={r:MixedRulePolicy(r) if args.condition=="rule" else MixedLLMPolicy(r,GeminiProxyCompleter(model=args.model,max_tokens=384,timeout=30)) for r in world.robot_ids}
        injected=[]
        def scenario(environment,eid,status):
            if args.scenario!="joint_obstacle" or injected:return
            pair=next((a for a in status.get("active",[]) if len(a['participants'])==2),None)
            if pair and status.get("activities") and float(world.data.time)>14.:
                event=environment.request('r1',{'operation':'mixed_obstacle','episode_id':eid,
                    'assignment_id':pair['assignment_id'],'position_xy':[.72,-.64]})
                injected.append(event)
                if not event.get('event_id'):raise RuntimeError('SCENARIO_NOT_INJECTED:'+str(event))
                print('INJECTED_OBSTACLE',event,flush=True)
        result=run_mixed_episode(Environment(world),policies,condition=args.condition,
                                 timeout_s=args.timeout,max_calls_per_robot=args.max_calls,journal_path=out/"episode.jsonl",
                                 scenario_hook=scenario,
                                 event_callback=on_message)
        result['scenario']=args.scenario
        result['injected_events']=injected
        result["motion"]=world._mixed_engine.recorder.summary()
        result["joint_solo_task_overlap_seconds"]=world._mixed_engine.task_overlap_seconds
        result["joint_solo_cargo_moving_overlap_seconds"]=world._mixed_engine.cargo_motion_overlap_seconds
        result["peer_collision_events"]=world._mixed_engine.peer_collision_events
        (out/"result.json").write_text(json.dumps(result,ensure_ascii=False,indent=2))
        (out/"physics.json").write_text(json.dumps(world.warehouse_state(),indent=2))
        print(json.dumps({k:result[k] for k in ['success','reason','calls','elapsed_s']}),flush=True)
        return 0 if result["success"] else 1
    finally:
        world.frame_callback=None
        try:
            engine=getattr(world,"_mixed_engine",None)
            if engine is not None:
                engine.close()
        finally:
            try:
                video.close()
            finally:
                sink.close();dialogue.close();world.close()
        from scripts.render_warehouse_dialogue import render_dialogue_video
        render_dialogue_video(out/"mixed-solo-joint-1x.mp4",out/"dialogue.jsonl",
                              out/"mixed-dialogue-1x.mp4")

if __name__=="__main__":raise SystemExit(main())
