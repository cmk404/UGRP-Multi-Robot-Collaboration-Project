"""Small systems comparison: existing action-selected skills vs semantic VLM steps.

Both planners get own RGB + unchanged shared top RGB and own model-action history.
World truth is only written to an offline referee stream. No destination task.
"""
from __future__ import annotations

import argparse
import base64
import hashlib
import json
from pathlib import Path
import platform
import subprocess
import sys
import time
from urllib.request import urlopen


def dump(path, value):
    Path(path).write_text(json.dumps(value, indent=2, ensure_ascii=False) + '\n')


def sha(data):
    return hashlib.sha256(data).hexdigest()


def score_grasp(samples):
    """Only called after the control loop. Continuous lifted bilateral hold."""
    longest = duration = 0.0
    previous = None
    prev_good = False
    max_lift = 0.0
    first_hold = None
    constraint_free = True
    for row in samples:
        now = float(row['elapsed_sim_s'])
        lift = float(row['lift_m'])
        constraints = row['constraints_active']
        if not constraints or not all(isinstance(v, bool) for v in constraints.values()):
            raise ValueError('missing/invalid constraint samples')
        if previous is not None and now <= previous:
            raise ValueError('nonmonotonic referee samples')
        constraint_free = constraint_free and not any(constraints.values())
        max_lift = max(max_lift, lift)
        good = lift >= .04 and row['bilateral_contact'] is True and not any(constraints.values())
        contiguous = previous is not None and 0 < now - previous <= .151
        duration = duration + now - previous if good and prev_good and contiguous else 0.0
        longest = max(longest, duration)
        if duration >= 2 and first_hold is None:
            first_hold = now
        previous, prev_good = now, good
    return {'physics_grasp_success': bool(longest >= 2 and constraint_free),
            'max_lift_m': max_lift, 'longest_bilateral_hold_s': longest,
            'first_2s_hold_sim_s': first_hold, 'constraint_free': constraint_free,
            'visual_review_required_for_positive': True}


class PairVideo:
    """Only recorded pixels; no camera repositioning, no actor input overlays."""
    def __init__(self, path, fps=5):
        self.fps = fps
        self.next_time = 0.0
        self.frames = 0
        self.process = subprocess.Popen([
            'ffmpeg', '-hide_banner', '-loglevel', 'error', '-y', '-f', 'rawvideo',
            '-pix_fmt', 'bgr24', '-s', '1280x520', '-r', str(fps), '-i', '-',
            '-an', '-c:v', 'libx264', '-preset', 'ultrafast', '-crf', '23',
            '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(path)], stdin=subprocess.PIPE)

    def capture(self, own, top, label, now):
        import cv2
        import numpy as np
        frames = [cv2.imdecode(np.frombuffer(value, np.uint8), cv2.IMREAD_COLOR)
                  for value in (own, top)]
        canvas = np.zeros((520, 1280, 3), np.uint8)
        for index, frame in enumerate(frames):
            canvas[40:520, index * 640:(index + 1) * 640] = cv2.resize(frame, (640, 480))
        cv2.putText(canvas, label[:120], (12, 27), cv2.FONT_HERSHEY_SIMPLEX, .65, (240, 240, 240), 1)
        self.process.stdin.write(canvas.tobytes())
        self.frames += 1
        self.next_time = now + 1 / self.fps

    def close(self):
        try:
            self.process.stdin.close()
            status = self.process.wait(timeout=30)
        except (BrokenPipeError, subprocess.TimeoutExpired):
            self.process.terminate()
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=5)
            raise
        if status:
            raise RuntimeError(f'ffmpeg exited {status}')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--condition', choices=('skill', 'semantic'), required=True)
    ap.add_argument('--seed', type=int, required=True)
    ap.add_argument('--seconds', type=float, default=120)
    ap.add_argument('--max-calls', type=int, default=30)
    ap.add_argument('--max-input-tokens', type=int, default=180000)
    ap.add_argument('--model', default='gemini-3.8-flash')
    ap.add_argument('--timeout', type=float, default=40)
    ap.add_argument('--capture-only', action='store_true')
    ap.add_argument('--scripted-units', nargs='+', help='Development grounding only; no VLM or scored cohort')
    args = ap.parse_args()
    if not 0 < args.seconds <= 300 or not 1 <= args.max_calls <= 60:
        ap.error('invalid bounded budget')
    out = args.output.resolve()
    import cv2
    import mujoco
    import numpy as np
    from harness.gemini_proxy import GeminiProxyCompleter
    from harness.llm_transport_skill import LLMTransportSkill
    from harness.semantic_pick_policy import PickMatchPlanner, SemanticPickInterpreter
    from harness.visual_macro_runtime import VisualMacroExecutor
    from sim.camera_robot_port import CameraRobotPort
    from sim.multi_masterpi_production import MultiMasterPiProductionV2

    revision = subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip()
    dirty = subprocess.check_output(['git', 'status', '--porcelain'], text=True).strip()
    if dirty:
        raise RuntimeError('commit source before experiment')
    if args.scripted_units and args.condition != 'semantic':
        ap.error('scripted units require semantic condition')
    out.mkdir(parents=True, exist_ok=False)
    (out / 'inputs').mkdir()
    (out / 'wire').mkdir()
    config = {**vars(args), 'output': str(out), 'git_sha': revision,
              'input_contract': 'own RGB + fixed existing top RGB + own issued model actions only',
              'claim_scope': 'single_robot_camera_matched_systems_pilot_not_original_Show_Harness_reproduction',
              'physics_paused_during_inference': True, 'automatic_retry_count': 0,
              'physics': {'impratio': 10, 'noslip_iterations': 0},
              'seconds_is_sim_budget': True}
    dump(out / 'config.json', config)
    dump(out / 'environment.json', {'python': sys.version, 'platform': platform.platform(),
        'modules': {'mujoco': mujoco.__version__, 'numpy': np.__version__, 'cv2': cv2.__version__},
        'ffmpeg': subprocess.check_output(['ffmpeg', '-version'], text=True).splitlines()[0]})
    started = time.monotonic()
    world = None
    video = None
    samples = []
    calls = []
    macros = []
    blocked = []
    error = None
    reason = 'SIM_BUDGET'
    terminal = False
    wire_count = 0
    frames = 0
    simulated = 0.0
    snapshot_hashes = []
    interpreter = None
    actor = None
    current_action = None
    scripted_units = list(args.scripted_units or [])
    scripted_steps = []
    files = [open(out / name, 'w') for name in ('raw-commands.jsonl', 'control.jsonl', 'evaluation-only.jsonl')]
    command_file, control_file, referee_file = files
    try:
        world = MultiMasterPiProductionV2(warehouse_layout='camera_team', seed=args.seed,
                    render=True, width=640, height=480, warehouse_cargo_ids=('small_box_01',))
        world.model.opt.impratio = 10
        world.model.opt.noslip_iterations = 0
        # Fixture setup only. No assisted grasp is ever activated by this runner.
        for eid in world.warehouse_weld_ids.values():
            world.data.eq_active[int(eid)] = 0
        port = CameraRobotPort(world, 'r1')
        initial_obs = port.capture()
        initial_commands = dict(initial_obs['actuator_state']['servo_pulses'])
        interpreter = SemanticPickInterpreter(initial_commands)
        actor = LLMTransportSkill('r1', 'small_box_01', 'B')
        initial = world.warehouse_state()
        initial_height = float(initial['cargo']['small_box_01']['position'][2])
        fixture = {'seed': args.seed, 'initial_issued_commands': initial_commands,
                   'initial_evaluation_state': initial,
                   'initial_qpos_sha256': sha(world.data.qpos.tobytes()),
                   'camera_parameters_sha256': sha(b''.join(v.tobytes() for v in
                       (world.model.cam_pos, world.model.cam_quat, world.model.cam_fovy))),
                   'geometry_sha256': sha(b''.join(v.tobytes() for v in
                       (world.model.geom_size, world.model.geom_rgba, world.model.geom_friction)))}
        dump(out / 'fixture-evaluation-only.json', fixture)

        def emit(row):
            value = {**row, 'model_action': current_action}
            raw = row.get('raw_action')
            if raw:
                interpreter.record_issued(raw)
            command_file.write(json.dumps(value) + '\n')
            command_file.flush()

        # Same bounded actuator executor, no third-camera drive guard on either side.
        # Existing skill controller stays intact; capture metadata never goes to VLM.
        ex = VisualMacroExecutor(port, log_callback=emit)

        def audited_open(request, *, timeout):
            nonlocal wire_count
            wire_count += 1
            (out / 'wire' / f'{wire_count:03d}.json').write_bytes(request.data)
            return urlopen(request, timeout=timeout)

        completer = GeminiProxyCompleter(model=args.model, max_tokens=650, timeout=args.timeout,
                       temperature=.2, reasoning_effort='none', http_open=audited_open)
        planner = PickMatchPlanner(completer, mode=args.condition)
        video = PairVideo(out / 'motion.mp4')
        start = float(world.data.time)
        next_sample = start

        def images():
            return (world.render_jpeg(robot_id='r1', camera='robot_cam'),
                    world.render_team_jpeg(camera='cctv_top'))

        def capture_referee(now):
            cargo = world.warehouse_state()['cargo']['small_box_01']
            contact = world.robot('r1').finger_cargo_contact('small_box_01')
            row = {'elapsed_sim_s': now - start,
                   'lift_m': float(cargo['position'][2]) - initial_height,
                   'bilateral_contact': bool(contact.get('bilateral')),
                   # Small-box specs have no assigned carriers, so their public
                   # state dictionary is empty even though welds are registered.
                   'constraints_active': {rid: bool(world.data.eq_active[int(eid)])
                       for (cid, rid), eid in world.warehouse_weld_ids.items()
                       if cid == 'small_box_01'},
                   'stable': cargo.get('stable'), 'position': cargo['position']}
            samples.append(row)
            referee_file.write(json.dumps(row) + '\n')

        while float(world.data.time) - start < args.seconds:
            now = float(world.data.time)
            simulated = now - start
            ex.tick(now)
            if now + 1e-9 >= next_sample:
                capture_referee(now)
                next_sample += .1
            if now >= video.next_time:
                own, top = images()
                video.capture(own, top, f'{args.condition} seed={args.seed} sim={simulated:.1f}s '
                    f'calls={len(calls)}  own RGB | shared top RGB', now)
            if args.capture_only:
                own, top = images()
                (out / 'initial-own.jpg').write_bytes(own)
                (out / 'initial-top.jpg').write_bytes(top)
                reason = 'CAPTURE_ONLY'
                break
            if ex.idle and not terminal:
                obs = port.capture()
                macro = None
                if args.scripted_units:
                    if scripted_units:
                        current_action = {'kind': 'semantic', 'unit': scripted_units.pop(0)}
                        try:
                            macro = interpreter.compile(current_action)
                            scripted_steps.append({'sim_time_s': simulated, 'action': current_action,
                                                   'macro': macro})
                        except ValueError as exc:
                            scripted_steps.append({'sim_time_s': simulated, 'action': current_action,
                                                   'rejected': str(exc)})
                            macro = {'kind': 'wait', 'duration': .3}
                    else:
                        terminal = True
                        reason = 'SCRIPTED_GROUNDING_ONLY'
                if args.condition == 'skill' and actor.operation is not None:
                    macro = actor.advance(obs)
                    if actor.state in ('failure', 'finished', 'carrying'):
                        terminal = True
                        reason = f'SKILL_{actor.state}:{actor.reason}'
                if not terminal and macro is None and not args.scripted_units and (args.condition == 'semantic' or actor.operation is None):
                    used = sum((call.get('usage') or {}).get('prompt_tokens', 0) for call in calls)
                    if len(calls) >= args.max_calls or used >= args.max_input_tokens:
                        terminal = True
                        reason = 'CALL_OR_REPORTED_INPUT_BUDGET'
                    else:
                        own, top = images()
                        index = len(calls) + 1
                        for label, jpeg in (('own', own), ('top', top)):
                            (out / 'inputs' / f'call-{index:03d}-{label}.jpg').write_bytes(jpeg)
                        record = {'call': index, 'sim_time_s': simulated,
                                  'own_sha256': sha(own), 'top_sha256': sha(top)}
                        try:
                            decision = planner.decide(own, top)
                            action = decision['action']
                            record['decision'] = decision
                            record['action'] = action
                            current_action = action
                            try:
                                if args.condition == 'skill':
                                    macro = actor.request(action)
                                    planner.record_issued(action)
                                    if macro is None:
                                        macro = actor.advance(obs)
                                    if actor.state in ('failure', 'finished', 'carrying'):
                                        terminal = True
                                        reason = f'SKILL_{actor.state}:{actor.reason}'
                                else:
                                    macro = interpreter.compile(action)
                                    planner.record_issued(action)
                            except ValueError as exc:
                                # A rejected command is not an observed state or a new plan.
                                # No rejection reason is supplied to the model on either side.
                                blocked.append({'call': index, 'reason': str(exc), 'action': action})
                                macro = {'kind': 'wait', 'duration': .3}
                                record['dispatch_rejected'] = str(exc)
                        except Exception as exc:
                            error = f'{type(exc).__name__}: {exc}'
                            record['error'] = error
                            record['error_latency_ms'] = getattr(exc, 'latency_ms', None)
                            terminal = True
                            reason = 'MODEL_ERROR'
                        finally:
                            record.update({'response': planner.last_response,
                                'usage': completer.last_usage, 'model': completer.last_model,
                                'latency_ms': completer.last_latency_ms or record.get('error_latency_ms')})
                            calls.append(record)
                            dump(out / 'calls.json', calls)
                            print(json.dumps({'condition': args.condition, 'seed': args.seed,
                                'call': index, 'sim_s': round(simulated, 2), 'action': record.get('action'),
                                'rejected': record.get('dispatch_rejected'), 'error': record.get('error')}, ensure_ascii=False), flush=True)
                if macro is not None and not terminal:
                    # Save all internal-skill input frames; none are privileged observations.
                    frames += 1
                    jpeg = base64.b64decode(obs['image'])
                    path = f'inputs/execution-{frames:04d}-own.jpg'
                    (out / path).write_bytes(jpeg)
                    snapshot_hashes.append(sha(jpeg))
                    row = {'macro': macro, 'phase': actor.phase if args.condition == 'skill' else 'semantic',
                           'source_path': path, 'sha256': obs['sha256'],
                           'own_issued_commands': obs['actuator_state']['servo_pulses'],
                           'sim_time_s': simulated}
                    macros.append(row)
                    control_file.write(json.dumps(row) + '\n')
                    control_file.flush()
                    # Common smoothing/settle behavior; phase never changes executor speed.
                    ex.submit(macro, obs, 'match', now)
            world._physics_step_for(world.robot('r1'))
        ex.cancel(float(world.data.time), 'fixed_budget_end')
        port.stop()
        own, top = images()
        (out / 'final-own.jpg').write_bytes(own)
        (out / 'final-top.jpg').write_bytes(top)
        dump(out / 'final-fixture-evaluation-only.json', {
            'camera_parameters_sha256': sha(b''.join(v.tobytes() for v in
                (world.model.cam_pos, world.model.cam_quat, world.model.cam_fovy))),
            'geometry_sha256': sha(b''.join(v.tobytes() for v in
                (world.model.geom_size, world.model.geom_rgba, world.model.geom_friction)))})
    except Exception as exc:
        error = f'{type(exc).__name__}: {exc}'
        reason = 'RUNNER_ERROR'
    finally:
        cleanup_errors = []
        if world is not None:
            try:
                world.robot('r1').set_motor_commands([0, 0, 0, 0])
            except Exception as exc:
                cleanup_errors.append(str(exc))
        if video is not None:
            try:
                video.close()
            except Exception as exc:
                cleanup_errors.append(str(exc))
        if world is not None:
            try:
                world.close()
            except Exception as exc:
                cleanup_errors.append(f'world.close: {exc}')
        for handle in files:
            handle.close()
        try:
            evaluation = score_grasp(samples) if samples else {'physics_grasp_success': False}
        except Exception as exc:
            cleanup_errors.append(f'score_grasp: {exc}')
            evaluation = {'physics_grasp_success': None, 'error': str(exc)}
        result = {'condition': args.condition, 'seed': args.seed, 'git_sha': revision,
                  'reason': reason, 'error': error, 'cleanup_errors': cleanup_errors,
                  'sim_seconds': simulated, 'wall_seconds': time.monotonic() - started,
                  'calls': len(calls), 'wire_requests': wire_count, 'macro_count': len(macros),
                  'blocked_commands': blocked, 'evaluation': evaluation,
                  'usage_records': sum(bool(call.get('usage')) for call in calls),
                  'input_tokens': sum((call.get('usage') or {}).get('prompt_tokens', 0) for call in calls),
                  'output_tokens': sum((call.get('usage') or {}).get('completion_tokens', 0) for call in calls),
                  'billing_amount': None, 'actor_state': actor.state if actor is not None else None,
                  'scripted_steps': scripted_steps,
                  'raw_storage': 'local_only', 'frame_count': video.frames if video is not None else 0}
        result['artifacts'] = {str(p.relative_to(out)): sha(p.read_bytes()) for p in out.rglob('*') if p.is_file()}
        dump(out / 'result.json', result)
        print(json.dumps(result, ensure_ascii=False), flush=True)
    return int(error is not None or bool(cleanup_errors))


if __name__ == '__main__':
    raise SystemExit(main())
