#!/usr/bin/env python3
"""Bounded native-window replay of saved LLM peer text, without physics."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import time


def saved_dialogue(source: Path):
    raw = source.read_bytes()
    saved = json.loads(raw)
    if saved.get('mode') != 'llm':
        raise ValueError('source must be a saved LLM run, not a fixture run')
    calls = {call['request_id']: call for call in saved['calls']}
    messages = []
    for row in saved['rounds']:
        for sender, reply in row['replies'].items():
            if not reply or not reply.get('message'):
                continue
            call = calls.get(reply['request_id'])
            if (not call or not call.get('model') or
                    call['model'] == 'scripted-fixture-not-llm' or call.get('reply') != reply):
                raise ValueError('message lacks a saved non-fixture model receipt')
            messages.append({'seq': len(messages)+1, 'sender': sender,
                             'recipients': [r for r in ('r1', 'r2', 'r3') if r != sender],
                             'turn': row['turn'], 'text': reply['message'],
                             'source': 'saved_llm_replay'})
    if not messages:
        raise ValueError('source has no saved LLM peer messages')
    state = {'status': '저장된 실제 LLM 대화 재생 (새 호출 0건)',
             'fresh_peer_messages': 0, 'recent_messages': messages[-3:],
             'full_log': str(source)}
    return state, hashlib.sha256(raw).hexdigest(), len(messages)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True,
                        help='existing team/team.json from a completed real-LLM run')
    parser.add_argument('--audit', type=Path, required=True,
                        help='new output JSON; source is never changed')
    parser.add_argument('--duration-s', type=float, default=10.)
    args = parser.parse_args(argv)
    if not 0 < args.duration_s <= 30:
        parser.error('duration must be within (0, 30] seconds')
    if args.audit.exists():
        parser.error('audit output already exists')
    state, digest, count = saved_dialogue(args.source)
    from harness.communication_overlay import render_dialogue_overlay
    import mujoco
    import mujoco.viewer
    pixels, korean_font = render_dialogue_overlay(state)
    model = mujoco.MjModel.from_xml_string(
        '<mujoco><worldbody><geom type="plane" size="1 1 .1"/></worldbody></mujoco>')
    data = mujoco.MjData(model)
    started = time.monotonic()
    with mujoco.viewer.launch_passive(model, data,
                                      show_left_ui=False, show_right_ui=False) as viewer:
        viewer.set_images((mujoco.MjrRect(12, 12, pixels.shape[1], pixels.shape[0]), pixels))
        until = started + args.duration_s
        while viewer.is_running() and time.monotonic() < until:
            viewer.sync(state_only=True)
            time.sleep(.05)
    # MuJoCo 3.12 close signals the render thread; wait for its teardown before
    # releasing the model, as in DispatchNativeView.close.
    close_deadline = time.monotonic() + 10.
    while viewer._sim() is not None:
        if time.monotonic() >= close_deadline:
            raise RuntimeError('native replay viewer did not close within 10 seconds')
        time.sleep(.01)
    audit = {'scope': 'saved actual LLM replies replayed in a static observer window; '
                      'no new model call, robot command, or physical replay',
             'source': str(args.source.resolve()), 'source_sha256': digest,
             'saved_peer_messages': count, 'displayed_recent_messages': min(3, count),
             'fresh_messages': 0, 'korean_font_available': korean_font,
             'requested_duration_s': args.duration_s,
             'viewer_elapsed_s': time.monotonic()-started}
    args.audit.parent.mkdir(parents=True, exist_ok=True)
    with args.audit.open('x', encoding='utf-8') as file:
        json.dump(audit, file, ensure_ascii=False, indent=2)
        file.write('\n')
    print(json.dumps(audit, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
