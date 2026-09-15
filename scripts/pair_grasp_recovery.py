"""Paired bounded place/regrasp execution using RGB and issued commands only.

Physical contact/lift scoring is performed later, never as a transition input.
"""
from harness.pair_navigation import ROBOTS
from harness.pair_grasp_spacing import PairGraspSpacing
from harness.pair_carry_sync import PairCarrySync
from harness.grasp_student_inference import predict_student
from scripts.run_camera_approach_student import write


def slip_requested(decisions):
    return any((d.get('own_carry_observation') or {}).get('slip_suspected') is True
               for d in decisions.values())


def recover_pair(scene, grasp_models, index, *, regrasp):
    record={'index':index,'regrasp_requested':regrasp,'error':None,
            'start_sim_time_s':scene.time(),'trace_start':len(scene.trace)}
    old=(scene.spacing_actors,scene.spacing_steps,scene.spacing_sync)
    scene.active_recovery_index=index
    try:
        for port in scene.ports.values(): port.stop()
        scene.capture('before-place')
        scene.place()
        record['place_commands_completed']=True
        if regrasp:
            scene.spacing_actors={r:PairGraspSpacing(scene.map,r) for r in ROBOTS}
            scene.spacing_steps=[]
            scene.spacing_sync=PairCarrySync(scene.map['map_id']+f'-recovery-{index}-grasp-spacing')
            scene.finish_grasp(predict_student,grasp_models,after_close=scene.anchor_spacing,
                               hold=scene.hold_spacing,close_pulse=1600)
            scene.hold_spacing(8.)
            scene.finish_spacing()
            record['regrasp_commands_completed']=True
    except Exception as error:
        record['error']=str(error)
    finally:
        if regrasp and scene.spacing_steps is not old[1]:
            record['spacing_steps']=scene.spacing_steps
            record['spacing_sync_events']=scene.spacing_sync.events
            record['grasp_report']=f'recovery-{index}-grasp-result.json'
            write(scene.out/record['grasp_report'],scene.grasp_report)
        record.update(end_sim_time_s=scene.time(),trace_end=len(scene.trace))
        scene.spacing_actors,scene.spacing_steps,scene.spacing_sync=old
        scene.active_recovery_index=0
    return record
