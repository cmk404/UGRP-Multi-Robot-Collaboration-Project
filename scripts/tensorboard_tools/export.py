"""Convert recorded evidence to TensorBoard, without importing any robot runtime.

The event wall clock is EXPORT TIME, not a fabricated historical run timestamp.
Training curves use recorded training steps. Execution curves use decision indices;
SIM time is an explicitly named scalar. Source files are never modified.
"""
from __future__ import annotations

import argparse
import hashlib
import html
import io
import json
import math
from pathlib import Path
import re
import time
import tempfile
import shutil
import subprocess
from scripts.carry_failure_metrics import issued_command_count

from scripts.tensorboard_tools.rgb_communication import EXTRA_METRICS, RUN_SCHEMA, export_communication

MAX_BYTES = 64 * 1024 * 1024
HP_METRICS = ('process/exit_code', 'result/wall_s', 'result/sim_s', 'result/commands', 'result/model_calls',
              'result/input_tokens', 'result/output_tokens', 'result/cost_usd', 'result/model_latency_s',
              'claims/operator_session_complete',
              'evaluation/reported_success', 'claims/protocol_complete',
              'evaluation/simultaneous_loaded_motion_s', 'evaluation/robot_robot_contact_samples',
              'claims/completed_task_claims', 'claims/tasks', 'claims/final_object_claims',
               'training/final_loss', 'development/final_selection_score',
               'offline/episodes', 'offline/premature_pair_hold_episodes',
               'offline/missed_terminal_episodes', 'offline/termination_pass') + EXTRA_METRICS
SECRET = re.compile(r'authorization|cookie|password|secret|api.?key|access.?token|refresh.?token', re.I)


def redact(value):
    if isinstance(value, dict):
        return {str(k): '[REDACTED]' if SECRET.search(str(k)) else redact(v) for k, v in value.items()}
    if isinstance(value, list): return [redact(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value): return None
    if isinstance(value, str):
        if value.lstrip().startswith(('{', '[')):
            try: return json.dumps(redact(json.loads(value)), ensure_ascii=False)
            except (ValueError, RecursionError): pass
        value = re.sub(r'data:image/[^\s"\']+', '[embedded image omitted]', value)
        return re.sub(r'(?i)bearer\s+[A-Za-z0-9._~+/=-]+', 'Bearer [REDACTED]', value)
    return value


def obj(value): return value if isinstance(value, dict) else {}
def rows(value): return [v for v in value if isinstance(v, dict)] if isinstance(value, list) else []
def finite(value): return type(value) in (int, float) and math.isfinite(value)
def sha(data): return hashlib.sha256(data).hexdigest()
def slug(value): return re.sub(r'[^a-zA-Z0-9._-]+', '-', str(value)).strip('-')[:100] or 'run'
def stable_digest(value): return sha(json.dumps(value, sort_keys=True, allow_nan=False).encode())


def inside(root, relative):
    if not isinstance(relative, str) or Path(relative).is_absolute(): return None
    p = root / relative
    try:
        if p.is_symlink() or not p.resolve().is_relative_to(root.resolve()) or not p.is_file(): return None
        return p.resolve()
    except OSError: return None


class Source:
    def __init__(self, root):
        self.root = root.resolve()
        self.files = {}
        self.warnings = []

    def read(self, relative, required=False):
        p = inside(self.root, relative)
        if not p:
            if required: raise ValueError(f'Missing source: {self.root / relative}')
            return None
        try:
            if p.stat().st_size > MAX_BYTES: raise ValueError('source exceeds 64 MiB')
            data = p.read_bytes()
            parsed = json.loads(data)
            self.files[relative] = {'sha256': sha(data), 'size': len(data),
                                    'mtime_s': p.stat().st_mtime}
            return parsed
        except (OSError, ValueError, UnicodeError) as e:
            if required: raise ValueError(f'Cannot read {p}: {e}') from e
            self.warnings.append(f'{relative}: unreadable, partial or oversized JSON; omitted')
            return None

    def image(self, reference, prefix=''):
        ref = reference.get('path') if isinstance(reference, dict) else reference
        if not isinstance(ref, str): return None
        rel = str(Path(prefix) / ref)
        p = inside(self.root, rel)
        if not p or p.suffix.lower() not in {'.jpg', '.jpeg', '.png'}:
            self.warnings.append(f'Image unavailable or outside run: {rel}')
            return None
        data = p.read_bytes()
        expected = reference.get('sha256') if isinstance(reference, dict) else None
        if expected and expected != sha(data):
            self.warnings.append(f'Image hash mismatch: {rel}; omitted')
            return None
        self.files[rel] = {'sha256': sha(data), 'size': len(data), 'mtime_s': p.stat().st_mtime}
        return data

    def read_jsonl(self, relative):
        p = inside(self.root, relative)
        if not p or p.stat().st_size > MAX_BYTES:
            raise ValueError('Missing or oversized JSONL: ' + relative)
        data = p.read_bytes()
        parsed = [json.loads(line) for line in data.decode().splitlines() if line.strip()]
        self.files[relative] = {'sha256': sha(data), 'size': len(data), 'mtime_s': p.stat().st_mtime}
        return parsed


class Writer:
    """Small wrapper over TensorBoard's own event protobufs; no TF/torch install."""
    def __init__(self, path, exported_at):
        from tensorboard.summary.writer.event_file_writer import EventFileWriter
        self.writer = EventFileWriter(str(path), max_queue_size=50, flush_secs=5)
        self.at = exported_at
        self.counts = {'scalars': 0, 'texts': 0, 'images': 0}

    def add(self, summary, step=0):
        from tensorboard.compat.proto.event_pb2 import Event
        self.writer.add_event(Event(wall_time=self.at, step=step, summary=summary))

    def scalar(self, tag, value, step=0):
        if not finite(value): return
        from tensorboard.compat.proto.summary_pb2 import Summary
        self.add(Summary(value=[Summary.Value(tag=tag, simple_value=float(value))]), step)
        self.counts['scalars'] += 1

    def text(self, tag, value, step=0, *, markdown=False):
        from tensorboard.compat.proto import summary_pb2, tensor_pb2, tensor_shape_pb2, types_pb2
        from tensorboard.plugins.text.metadata import create_summary_metadata
        text = value if markdown else '<pre>' + html.escape(json.dumps(redact(value), ensure_ascii=False, indent=2)) + '</pre>'
        tensor = tensor_pb2.TensorProto(dtype=types_pb2.DT_STRING,
            tensor_shape=tensor_shape_pb2.TensorShapeProto(dim=[tensor_shape_pb2.TensorShapeProto.Dim(size=1)]),
            string_val=[text.encode()])
        self.add(summary_pb2.Summary(value=[summary_pb2.Summary.Value(tag=tag, tensor=tensor,
            metadata=create_summary_metadata(tag, 'Imported evidence; event wall_time is export time.'))]), step)
        self.counts['texts'] += 1

    def image(self, tag, data, step=0):
        from PIL import Image
        from tensorboard.compat.proto.summary_pb2 import Summary
        with Image.open(io.BytesIO(data)) as im:
            im = im.convert('RGB'); im.thumbnail((640, 480))
            encoded = io.BytesIO(); im.save(encoded, format='PNG')
            value = Summary.Image(height=im.height, width=im.width, colorspace=3,
                                  encoded_image_string=encoded.getvalue())
        self.add(Summary(value=[Summary.Value(tag=tag, image=value)]), step)
        self.counts['images'] += 1

    def hparams(self, values, metrics):
        from tensorboard.compat.proto.summary_pb2 import Summary
        from tensorboard.plugins.hparams import api_pb2, metadata, plugin_data_pb2
        from tensorboard.util.tensor_util import make_tensor_proto
        start = plugin_data_pb2.SessionStartInfo(start_time_secs=self.at)
        for k, v in values.items(): start.hparams[k].string_value = str(v)
        experiment = api_pb2.Experiment(
            hparam_infos=[api_pb2.HParamInfo(name=k, type=api_pb2.DATA_TYPE_STRING) for k in values],
            metric_infos=[api_pb2.MetricInfo(name=api_pb2.MetricName(tag=k)) for k in metrics],
            time_created_secs=self.at)
        # HParams status describes import completion, not robot task success.
        end = plugin_data_pb2.SessionEndInfo(status=api_pb2.STATUS_SUCCESS, end_time_secs=self.at)
        for tag, data in [(metadata.EXPERIMENT_TAG, plugin_data_pb2.HParamsPluginData(experiment=experiment)),
                          (metadata.SESSION_START_INFO_TAG, plugin_data_pb2.HParamsPluginData(session_start_info=start)),
                          (metadata.SESSION_END_INFO_TAG, plugin_data_pb2.HParamsPluginData(session_end_info=end))]:
            self.add(Summary(value=[Summary.Value(tag=tag, tensor=make_tensor_proto([], dtype='float32'),
                                                   metadata=metadata.create_summary_metadata(data))]))

    def close(self): self.writer.close()


def numeric_leaves(value, prefix=''):
    if finite(value): yield prefix, value
    elif isinstance(value, dict):
        for k, v in value.items(): yield from numeric_leaves(v, f'{prefix}/{k}' if prefix else str(k))
    elif isinstance(value, list):
        for i, v in enumerate(value): yield from numeric_leaves(v, f'{prefix}/{i}')


def sample_indices(length, maximum):
    if maximum <= 0 or length == 0: return set()
    if maximum == 1: return {length - 1}
    return {round(i * (length - 1) / (min(length, maximum) - 1)) for i in range(min(length, maximum))} if length > 1 else {0}


def emit_images(w, src, images, step, prefix=''):
    for view, ref in images.items():
        data = src.image(ref, prefix)
        if data:
            try: w.image(f'observations/{view}', data, step)
            except (OSError, ValueError): src.warnings.append(f'Image decode failed at step {step}: {view}')


def export_training(src, w, data):
    progress = rows(data.get('progress'))
    if not progress: raise ValueError('No training progress rows')
    previous = -1
    for row in progress:
        step = row.get('step')
        if type(step) is not int or step <= previous: raise ValueError('Training steps must be strictly increasing integers')
        previous = step
        w.scalar('training/loss', row.get('loss'), step)
        w.scalar('training/elapsed_s', row.get('elapsed_s'), step)
        for key, value in numeric_leaves(obj(row.get('development'))): w.scalar('development/' + key, value, step)
    w.scalar('training/final_loss', progress[-1].get('loss'))
    w.scalar('development/final_selection_score', obj(progress[-1].get('development')).get('selection_score'))
    w.text('training/selection', {k: data.get(k) for k in ('selected', 'selection', 'seed', 'steps', 'complete')})
    return {'family': 'training', 'policy': 'ACT', 'case': src.root.name,
            'source_sha': data.get('source_sha'), 'seed': data.get('seed'),
            'dataset_sha256': data.get('dataset_sha256'), 'complete': data.get('complete')}, {}


def export_execution(src, w, result, max_images):
    if result.get('schema_version') == RUN_SCHEMA:
        return export_communication(src, w, result)
    cfg = obj(result.get('config')); usage = obj(result.get('usage'))
    if (src.root / 'turns.json').is_file(): family = 'jev-motion'
    elif (src.root / 'actor-static-task.json').is_file(): family = 'multi-object'
    elif (src.root / 'pair-decisions.json').is_file(): family = 'dispatch-act'
    else: family = 'result-only'
    policy = result.get('policy') or result.get('model') or ('ACT/' + Path(cfg['carry_act_model']).parent.name if cfg.get('carry_act_model') else ('RGB skills' if family == 'dispatch-act' else 'unrecorded'))
    meta = {'family': family, 'policy': policy, 'case': result.get('case', cfg.get('variant')),
            'source_sha': result.get('source_sha'), 'scope': result.get('scope'), 'clock': result.get('clock'),
            'seed': result.get('seed', cfg.get('seed')), 'goal': result.get('goal'),
            'spawn_offset': cfg.get('spawn_offset'), 'contact_profile': cfg.get('contact_profile'),
            'model_provenance': result.get('model_provenance'), 'plan_replay_sha256': result.get('plan_replay_sha256'),
            'limits': result.get('limits')}
    for setup in ('setup-only.json', 'episode-setup-only.json', 'setup-evaluation-only.json'):
        data = src.read(setup)
        if data is not None: meta['setup_sha256'] = src.files[setup]['sha256']; break
    metrics = {'result/wall_s': result.get('wall_s'), 'result/sim_s': result.get('sim_s'),
        'result/commands': result.get('commands', result.get('issued_commands')),
        'result/model_calls': result.get('model_calls', result.get('llm_calls')),
        'result/input_tokens': result.get('input_tokens', usage.get('prompt_tokens')),
        'result/output_tokens': result.get('output_tokens', usage.get('completion_tokens')),
        'result/cost_usd': result.get('cost_usd'), 'result/model_latency_s': result.get('model_latency_s')}
    if (src.root / 'skill-bindings.json').is_file():
        # Existing skill runners keep receipts separately from result.json.
        calls = rows(obj(src.read('team/team.json')).get('calls'))
        if calls and all(finite(c.get('latency_ms')) for c in calls):
            metrics['result/model_latency_s'] = sum(c['latency_ms'] for c in calls) / 1000
        commands = src.read('issued-commands.json')
        if isinstance(commands, dict):
            metrics['result/recorded_raw_commands'] = sum(
                'action' in row for values in commands.values() for row in rows(values))
            meta['command_count_scope'] = 'recorded raw action rows only; excludes setup descriptions and internal macro servo commands'
    carries = []
    if family == 'dispatch-act':
        entries = rows(src.read('pair-decisions.json', required=True))
        carries = [(i, row) for i, row in enumerate(entries) if row.get('kind') == 'act_carry']
        completed = sum(bool(obj(decision)) for _, row in carries
                        for decision in obj(row.get('decisions')).values())
        meta['act_carry_decision_rows'] = len(carries)
        meta['completed_act_responses'] = completed
        # llm_calls excludes local ACT. Preserve an explicit total if supplied.
        if not finite(result.get('model_calls')):
            external = result.get('llm_calls')
            metrics['result/model_calls'] = completed + (external if finite(external) else 0)
            meta['model_calls_scope'] = ('completed ACT responses plus recorded llm_calls'
                                        if finite(external) else 'completed ACT responses only; external calls unknown')
        if not finite(metrics['result/commands']):
            issued = src.read('issued-commands.json')
            if isinstance(issued, dict):
                metrics['result/commands'] = issued_command_count(issued)
                meta['commands_source'] = 'issued-commands.json; excludes initial SETUP target snapshot'
    success_field = next((k for k in ('success', 'transport_success', 'physical_success') if type(result.get(k)) is bool), None)
    evaluation=obj(result.get('evaluation'))
    concurrency=obj(evaluation.get('concurrent_transport'))
    audit=src.read('concurrency-audit.json')
    if audit is not None:
        expected=obj(audit.get('source_files_sha256'))
        if set(expected)!={'result.json','issued-commands.json','referee-only.jsonl'}:
            raise ValueError('concurrency audit requires all original source hashes')
        for name,digest in expected.items():
            path=inside(src.root,name)
            if path is None or sha(path.read_bytes())!=digest:
                raise ValueError('concurrency audit source hash mismatch')
            src.files[name]={'sha256':digest,'size':path.stat().st_size,'mtime_s':path.stat().st_mtime}
        concurrency=obj(audit.get('concurrent_transport'))
        meta['concurrency_evaluation_source']='concurrency-audit.json; original result preserved'
        w.text('evaluation/concurrency_audit',audit)
    metrics['evaluation/simultaneous_loaded_motion_s']=concurrency.get('simultaneous_loaded_motion_s')
    metrics['evaluation/robot_robot_contact_samples']=evaluation.get('robot_robot_contact_samples')
    if success_field: metrics['evaluation/reported_success'] = int(result[success_field])
    meta['success_source_field'] = success_field
    meta['outcome'] = str(result[success_field]) if success_field else 'unrecorded'
    if type(result.get('protocol_complete')) is bool: metrics['claims/protocol_complete'] = int(result['protocol_complete'])
    if type(result.get('plan_committed')) is bool: metrics['claims/plan_committed'] = int(result['plan_committed'])
    if type(result.get('operator_session_complete')) is bool:
        metrics['claims/operator_session_complete'] = int(result['operator_session_complete'])
    for name, val in obj(result.get('protocol')).items():
        if name in ('completed_task_claims', 'tasks', 'final_object_claims') and finite(val): metrics['claims/' + name] = val
    for k, v in metrics.items(): w.scalar(k, v)
    w.text('result/summary', {k: result.get(k) for k in ('success', 'physical_success', 'transport_success', 'stop_reason', 'error', 'phase', 'scope', 'clock', 'protocol_complete', 'protocol')})
    w.text('evaluation/referee_only', result.get('evaluation', result.get('final_evaluation', {})))
    if family == 'jev-motion':
        entries = rows(src.read('turns.json', required=True))
        selected = sample_indices(len(entries), max_images)
        for i, row in enumerate(entries):
            w.scalar('execution/sim_time_s', row.get('observed_at_sim_s'), i)
            for k in ('range_m', 'bearing_deg', 'target_forward_m', 'target_left_m'):
                w.scalar('rgb_estimate/' + k, obj(row.get('observation')).get(k), i)
            w.scalar('execution/observation_to_issue_wall_s', row.get('observation_to_issue_wall_s'), i)
            w.scalar('execution/model_latency_s', obj(row.get('response')).get('latency_s'), i)
            # Keep policy state/choice, not raw transport headers/provider strings.
            w.text('decisions/r2', {k: row.get(k) for k in ('turn', 'observed_at_sim_s', 'state', 'action', 'decision_source', 'images')}, i)
            if i in selected:
                images = obj(row.get('images'))
                emit_images(w, src, {'r2/own': images.get('own_rgb'), 'shared/top': images.get('shared_top_rgb')}, i)
    elif family == 'multi-object':
        files = sorted(src.root.glob('turn-*.json')); selected = sample_indices(len(files), max_images)
        for i, f in enumerate(files):
            turn = obj(src.read(f.name, required=True));w.scalar('execution/sim_time_s', turn.get('at_s'), i)
            for rid, reply in obj(turn.get('replies')).items():
                reply = obj(reply); req = obj(src.read('runtime/' + str(reply.get('request_id', '')) + '.json'))
                w.text('decisions/' + slug(rid), {'request': req, 'reply': reply, 'sim_time_s': turn.get('at_s')}, i)
                if finite(reply.get('confidence')): w.scalar('model_confidence_not_success/' + slug(rid), reply['confidence'], i)
                if i in selected: emit_images(w, src, {slug(rid) + '/own': req.get('own_rgb'), 'shared/top/' + slug(rid): req.get('top_rgb')}, i, 'scene')
    elif family == 'dispatch-act':
        selected = sample_indices(len(carries), max_images)
        # Carry prerequisites are not inferred from ACT being configured.
        call_index = 0
        for j, (i, row) in enumerate(carries):
            w.scalar('execution/sim_time_s', row.get('sim_time_s'), j)
            for slot, inp in obj(row.get('inputs')).items():
                inp = obj(inp); rid = slug(inp.get('physical_robot_id', slot)); decision = obj(obj(row.get('decisions')).get(slot))
                if decision:
                    w.scalar('execution/model_latency_s', inp.get('inference_wall_s'), call_index)
                    call_index += 1
                w.text('decisions/' + rid, {'input': inp, 'decision': decision, 'permission': row.get('permission'), 'source_index': i}, j)
                for k in ('forward', 'left', 'turn'): w.scalar('issued_prediction/' + rid + '/' + k, obj(decision.get('action')).get(k), j)
                if type(decision.get('done')) is bool: w.scalar('claims/' + rid + '/done', int(decision['done']), j)
                if j in selected:
                    images = obj(inp.get('images'));emit_images(w, src, {rid + '/own': images.get('own'), rid + '/top': images.get('top')}, j)
    return meta, {k: v for k, v in metrics.items() if finite(v)}


def exporter_version():
    root = Path(__file__).resolve().parents[2]
    try:
        head = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=root, text=True, timeout=3).strip()
        dirty = bool(subprocess.check_output(['git', 'status', '--porcelain'], cwd=root, text=True, timeout=3).strip())
        return {'sha': head, 'working_tree_dirty': dirty}
    except (OSError, subprocess.SubprocessError): return {'sha': None, 'working_tree_dirty': None}


def export_cloud_job(src, w, data):
    """Process completion is distinct from robot success, including setup failures."""
    if (data.get('status') not in {'complete', 'completed', 'failed'}
            or type(data.get('exit_code')) is not int
            or not finite(data.get('finished_at_unix'))):
        raise ValueError('Cloud job has no terminal process evidence')
    metrics = {'process/exit_code': data['exit_code']}
    start, end = data.get('started_at_unix'), data['finished_at_unix']
    if finite(start) and end >= start:
        metrics['result/wall_s'] = end - start
    for name, value in metrics.items(): w.scalar(name, value)
    w.text('process/result', data)
    recovery = src.read('result/recovery-status.json')
    if recovery is not None: w.text('process/recovery', recovery)
    return {'family': 'cloud-job', 'policy': 'environment',
            'source_sha': data.get('source_sha'), 'scope': data.get('scope'),
            'outcome': 'process_exit_' + str(data['exit_code']),
            'success_source_field': None}, metrics


def export_hardware_probe(src, w, data):
    torch = obj(data.get('torch'))
    if type(torch.get('available')) is not bool or type(torch.get('count')) is not int:
        raise ValueError('No completed GPU inventory evidence')
    metrics = {'hardware/gpu_count': torch['count'],
               'hardware/cuda_available': int(torch['available']),
               'hardware/internet_http_status': data.get('internet_http_status')}
    for name, value in metrics.items(): w.scalar(name, value)
    w.text('hardware/inventory', data)
    provenance = src.read('probe-provenance.json')
    if provenance: w.text('hardware/provenance', provenance)
    return {'family':'hardware-probe', 'policy':'environment',
            'outcome':'gpu_available' if torch['available'] else 'gpu_unavailable',
            'scope':'Observed cloud devices and connectivity; no robot evaluation',
            'success_source_field':None}, metrics


def export_termination_audit(src, w, data):
    """Recompute the saved-prediction audit; never call it physical success."""
    from scripts.audit_carry_termination import audit
    predictions=src.read('predictions.json',required=True)
    if src.files['predictions.json']['sha256']!=data.get('predictions_sha256'):
        raise ValueError('termination audit prediction hash mismatch')
    checked=audit(predictions,threshold=data['threshold'])
    if any(data.get(k)!=v for k,v in checked.items()):
        raise ValueError('termination audit does not match saved predictions')
    metrics={'offline/'+k:checked[k] for k in (
        'episodes','premature_pair_hold_episodes','missed_terminal_episodes')}
    metrics['offline/termination_pass']=int(checked['offline_termination_pass'])
    for name,value in metrics.items():w.scalar(name,value)
    w.text('offline/termination_audit',data)
    return {'family':'act-termination-audit','policy':'ACT','case':src.root.name,
            'outcome':'offline_pass' if checked['offline_termination_pass'] else 'offline_fail',
            'scope':checked['scope'],'success_source_field':None,
            'predictions_sha256':data['predictions_sha256']},metrics


def convert(source, output, *, max_images=8, media_port=6007, allow_synthetic=False):
    """Export one source once. Existing destinations are rejected (no duplicate steps)."""
    source, output = Path(source).resolve(), Path(output).resolve()
    if not source.is_dir(): raise ValueError(f'Not a source directory: {source}')
    if output == source or output.is_relative_to(source): raise ValueError('Export must be outside the source directory')
    src = Source(source)
    result = src.read('result.json')
    if isinstance(result, dict) and result.get('schema_version') == RUN_SCHEMA:
        if result.get('evidence_kind') not in {'deterministic_physical_replay', 'live_llm'}:
            if not allow_synthetic or not output.is_relative_to(Path(tempfile.gettempdir()).resolve()):
                raise ValueError('Synthetic communication evidence requires explicit temporary-logdir opt-in')
    training = src.read('report.json')
    if isinstance(training, dict) and rows(training.get('progress')):
        kind, data = 'training', training
    elif isinstance(result, dict): kind, data = 'execution', result
    elif (source / 'termination-audit.json').exists():
        kind, data = 'termination-audit', src.read('termination-audit.json', required=True)
    elif (source / 'gpu-inventory.json').exists():
        kind, data = 'hardware-probe', src.read('gpu-inventory.json', required=True)
    elif (source / 'run.json').exists():
        kind, data = 'cloud-job', src.read('run.json', required=True)
    elif (source / 'progress.json').exists():
        kind, data = 'training', {'progress': src.read('progress.json', required=True)}
    else: raise ValueError('No complete result.json or supported training progress; source left untouched')
    at = time.time(); output.mkdir(parents=True, exist_ok=False)
    temporary = tempfile.TemporaryDirectory(prefix='ugrp-tensorboard-export-')
    w = Writer(Path(temporary.name), at)
    manifest = {'schema': 'ugrp.tensorboard-export.v1', 'source': str(source), 'exported_at_s': at,
                'event_wall_time': 'export time, not historical execution time', 'exporter': exporter_version(), 'complete': False}
    try:
        if kind == 'training': meta, metrics = export_training(src, w, data)
        elif kind == 'cloud-job': meta, metrics = export_cloud_job(src, w, data)
        elif kind == 'hardware-probe': meta, metrics = export_hardware_probe(src, w, data)
        elif kind == 'termination-audit': meta, metrics = export_termination_audit(src, w, data)
        else: meta, metrics = export_execution(src, w, data, max_images)
        videos = []
        video_names = ('motion.mp4', 'execution.mp4')
        if isinstance(result, dict) and result.get('schema_version') == RUN_SCHEMA:
            video_names += ('backend/execution.mp4',)
        for name in video_names:
            p = inside(source, name)
            if p:
                st = p.stat(); ident = sha(str(p).encode())[:20]
                videos.append({'id': ident, 'path': str(p), 'size': st.st_size, 'mtime_ns': st.st_mtime_ns})
                w.text('media/' + name, f'[원본 {name} 재생](http://127.0.0.1:{media_port}/video/{ident})\n\n'
                       '로컬 미디어 서버가 필요합니다. 영상 시간과 SIM 시간의 자동 동기화는 하지 않습니다.', markdown=True)
        w.text('provenance/source', {'source_directory': str(source), 'export_time_s': at,
            'event_wall_time': '변환 시각입니다. 실행 시작·종료 시각이 아닙니다. 가로축은 STEP으로 보세요.',
            'step_axis': 'training: recorded optimizer step; execution: recorded decision order',
            'source_metadata': meta, 'warnings': src.warnings,
            'limits': '선택한 실행들의 개별 기록입니다. 성공률 집계·조건 동등성·실물 성능을 자동 주장하지 않습니다. 미기록 비용/시각은 0으로 채우지 않습니다.'})
        hp = {k: str(meta.get(k) if meta.get(k) is not None else 'unrecorded') for k in ('family', 'policy', 'case', 'source_sha', 'seed', 'outcome', 'clock', 'setup_sha256', 'run_id', 'condition')}
        hp['condition_fingerprint'] = stable_digest({k: meta.get(k) for k in ('family','case','source_sha','seed','scope','clock','goal','spawn_offset','contact_profile','setup_sha256','limits')})
        w.hparams(hp, HP_METRICS)
        manifest.update(metadata=meta, source_files=src.files, warnings=src.warnings, videos=videos, counts=w.counts)
        w.close()
        for relative, record in src.files.items():
            current = inside(src.root, relative)
            if current is None or sha(current.read_bytes()) != record['sha256']:
                raise ValueError(f'Source changed during export: {relative}; no event file published')
        for event_file in Path(temporary.name).iterdir():
            shutil.copy2(event_file, output / event_file.name)
        manifest['complete'] = True
    except Exception:
        w.close()
        manifest.update(source_files=src.files, warnings=src.warnings)
        (output / 'manifest.json').write_text(json.dumps(redact(manifest), ensure_ascii=False, indent=2)+'\n')
        raise
    finally:
        temporary.cleanup()
    (output / 'manifest.json').write_text(json.dumps(redact(manifest), ensure_ascii=False, indent=2)+'\n')
    return manifest


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source', type=Path, action='append', required=True, help='Completed run or ACT training directory; repeatable')
    p.add_argument('--output', type=Path, required=True, help='New export collection directory')
    p.add_argument('--max-images', type=int, default=8, help='Evenly sampled decisions per run, including first/last; 0 disables images')
    p.add_argument('--media-port', type=int, default=6007)
    args = p.parse_args()
    if not 0 <= args.max_images <= 100: p.error('--max-images must be in 0..100')
    if not 1 <= args.media_port <= 65535: p.error('invalid media port')
    sources = list(dict.fromkeys(x.resolve() for x in args.source))
    output = args.output.resolve()
    if any(output == s or output.is_relative_to(s) for s in sources): p.error('output must be outside every source directory')
    if output.exists(): p.error('output must be new; existing events are never overwritten or appended')
    output.mkdir(parents=True)
    exported, failed = [], []
    for source in sources:
        name = slug(source.parent.name) + '__' + slug(source.name) + '__' + sha(str(source).encode())[:8]
        try:
            m = convert(source, output/name, max_images=args.max_images, media_port=args.media_port)
            exported.append({'name': name, 'source': str(source), 'counts': m['counts']})
            print(json.dumps(exported[-1], ensure_ascii=False), flush=True)
        except (ValueError, OSError) as e:
            failed.append({'source': str(source), 'error': str(e)});print(json.dumps(failed[-1]), flush=True)
    (output/'collection.json').write_text(json.dumps({'exported': exported, 'failed': failed},ensure_ascii=False,indent=2)+'\n')
    return 1 if failed else 0
