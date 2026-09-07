from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from PIL import Image, ImageDraw

from harness.catalog import default_registry
from harness.groq import GroqCompleter
from harness.vlm import VlmError
from harness.loop import run_loop
from harness.protocol import ToolCall, Look, Wait, FinalAnswer, Plan

ROOT = Path('/tmp/ugrp-agent-bench-auto')
ROOT.mkdir(parents=True, exist_ok=True)
# Synthetic frames drive a stateful world; Groq calls are live, robot actions are simulated.

@dataclass
class World:
    name: str
    distance: int = 3  # 3 far, 2 mid, 1 near
    x: float = 0.5
    visible: bool = True
    held: bool = False
    stable: bool = False
    blue_decoy: bool = False
    forced_pick_failures: int = 0
    slips_after_first_grasp: bool = False
    lose_after_first_move: bool = False
    move_count: int = 0
    pick_attempts: int = 0
    grasp_count: int = 0
    observation_count: int = 0
    action_log: list[str] = field(default_factory=list)
    frame_index: int = 0
    slipped: bool = False
    lost: bool = False

    def run(self, name: str) -> dict[str, Any]:
        self.action_log.append(name)
        if name == 'approach':
            self.move_count += 1
            if self.visible and not self.held:
                self.distance = max(1, self.distance - 1)
        elif name == 'track':
            if self.visible and not self.held:
                self.x = 0.5
        elif name == 'pick':
            self.pick_attempts += 1
            if self.visible and not self.held and self.distance == 1 and abs(self.x - 0.5) <= 0.16:
                if self.pick_attempts > self.forced_pick_failures:
                    self.held = True
                    self.grasp_count += 1
                    self.stable = False
        elif name == 'carry':
            if self.held:
                self.stable = True
        elif name == 'fetch':
            self.move_count += 1
            if self.visible and not self.held:
                self.distance = 1
                self.x = 0.5
                self.pick_attempts += 1
                if self.pick_attempts > self.forced_pick_failures:
                    self.held = True
                    self.grasp_count += 1
                    self.stable = False
        # Keep result intentionally non-semantic, matching current robot_actions.py.
        return {'skill': name, 'exit_code': 0}

    def before_observe(self) -> None:
        self.observation_count += 1
        # Inject disturbances only after an action has actually happened.
        if self.lose_after_first_move and self.move_count >= 1 and not self.lost:
            self.visible = False
            self.held = False
            self.lost = True
        if self.slips_after_first_grasp and self.grasp_count >= 1 and not self.slipped:
            self.held = False
            self.stable = False
            self.visible = True
            self.distance = 1
            self.x = 0.78
            self.slipped = True

    def render(self, *, observe: bool = False) -> str:
        if observe:
            self.before_observe()
        self.frame_index += 1
        W,H = 640,480
        im = Image.new('RGB',(W,H),(220,225,229))
        d = ImageDraw.Draw(im)
        # walls + floor perspective
        d.rectangle((0,0,W,300), fill=(224,229,234))
        d.polygon([(0,300),(W,300),(W,H),(0,H)], fill=(173,168,158))
        for y in (335,385,440):
            d.line((0,y,W,y), fill=(145,140,132), width=2)
        for x in (100,220,320,420,540):
            d.line((320,300,x,H), fill=(152,147,139), width=1)
        # simple gripper at top center
        d.rectangle((305,0,335,95), fill=(70,75,82))
        d.rectangle((270,82,315,102), fill=(70,75,82))
        d.rectangle((325,82,370,102), fill=(70,75,82))
        # held block is visually between gripper fingers
        if self.held:
            d.rectangle((292,82,348,138), fill=(220,25,35), outline=(120,0,0), width=4)
            if self.stable:
                # stability cue: gripper closed symmetrically around block
                d.rectangle((275,90,292,128), fill=(55,60,66))
                d.rectangle((348,90,365,128), fill=(55,60,66))
        elif self.visible:
            sizes = {3:52,2:92,1:150}
            sz=sizes[self.distance]
            cx=int(self.x*W)
            cy={3:285,2:310,1:335}[self.distance]
            # cube-ish red block
            d.rectangle((cx-sz//2,cy-sz//2,cx+sz//2,cy+sz//2), fill=(220,25,35), outline=(120,0,0), width=4)
            d.polygon([(cx-sz//2,cy-sz//2),(cx-sz//2+16,cy-sz//2-12),(cx+sz//2+16,cy-sz//2-12),(cx+sz//2,cy-sz//2)], fill=(245,65,72))
            d.polygon([(cx+sz//2,cy-sz//2),(cx+sz//2+16,cy-sz//2-12),(cx+sz//2+16,cy+sz//2-12),(cx+sz//2,cy+sz//2)], fill=(170,12,22))
        if self.blue_decoy:
            # blue decoy closer and intentionally salient
            bx,by,bs=155,330,115
            d.rectangle((bx-bs//2,by-bs//2,bx+bs//2,by+bs//2), fill=(30,90,220), outline=(10,40,120), width=4)
        out=ROOT/f'{self.name}-{self.frame_index:02d}.jpg'
        im.save(out, quality=90)
        return str(out)


class RetryingCompleter:
    def __init__(self):
        self.inner = GroqCompleter(max_tokens=192, temperature=0.2)
        self.rate_limit_retries = 0
    def complete(self, messages, image=None):
        for attempt in range(30):
            try:
                return self.inner.complete(messages, image=image)
            except VlmError as exc:
                if "한도" not in str(exc):
                    raise
                self.rate_limit_retries += 1
                # Groq currently returns Retry-After~=2s for this 8k TPM bucket.
                time.sleep(2.2)
        raise VlmError("rate-limit retries exhausted")

def run_case(case_name: str, goal: str, world: World, max_steps: int = 10) -> dict[str, Any]:
    initial=world.render(observe=False)
    reg=default_registry(runner=world.run)
    model=RetryingCompleter()
    def observe():
        return world.render(observe=True)
    t0=time.time()
    result=run_loop(model, reg, goal, image=initial, max_steps=max_steps, execute=True, observe=observe, sleeper=lambda _: None, auto_observe=True)
    elapsed=time.time()-t0
    trace=[]
    for i,s in enumerate(result.steps,1):
        kind=type(s.action).__name__ if s.action else None
        item={'i':i,'kind':kind,'error':s.error,'raw':s.raw}
        if isinstance(s.action, ToolCall): item['tool']=s.action.name
        if isinstance(s.action, Plan): item['plan']=list(s.action.steps)
        if isinstance(s.action,(Look,Wait)): item['observed']=bool(s.result and s.result.get('observed'))
        trace.append(item)
    errors=[x for x in trace if x.get('error')]
    unknown=[x for x in errors if 'unknown tool' in (x.get('error') or '')]
    return {
        'case':case_name,'goal':goal,'elapsed_s':round(elapsed,2),'rate_limit_retries':model.rate_limit_retries,
        'protocol_errors':len(errors),'unknown_tool_errors':len(unknown),
        'stopped':result.stopped,'final':result.final,
        'world':{
            'visible':world.visible,'held':world.held,'stable':world.stable,
            'distance':world.distance,'x':round(world.x,2),'pick_attempts':world.pick_attempts,
            'grasp_count':world.grasp_count,'actions':world.action_log,'observations':world.observation_count,
            'slipped':world.slipped,'lost':world.lost,
        },
        'trace':trace,
    }

cases=[]
# 1) off-center target: should discover pick failure visually and use track/recover.
cases.append(('offcenter_recovery', '화면의 빨간 블록을 집어서 안정적으로 들고 있어. 상황을 계속 확인하면서 끝까지 처리해.', World('offcenter', distance=1, x=0.12), 8))
# 2) long recovery: composite fetch/pick can initially work, but first grasp slips sideways; must reacquire and stabilize.
cases.append(('slip_long_recovery', '빨간 블록을 확실히 집은 뒤 놓치지 않게 안정적으로 들고 있어. 중간에 실패하면 장면을 다시 보고 스스로 복구해.', World('slip', distance=3, x=0.52, slips_after_first_grasp=True), 10))
# 3) repeated pick failures: two genuine failures before success; must not falsely finish.
cases.append(('double_pick_failure', '빨간 블록을 집고 실제로 들린 것을 확인한 다음 안정적으로 유지해. 실패하면 다시 시도해.', World('doublefail', distance=2, x=0.5, forced_pick_failures=2), 10))
# 4) target disappears after first motion, blue decoy remains. Must not claim success or pick blue as red.
cases.append(('target_lost_with_decoy', '빨간 블록을 찾아서 집어. 빨간 블록이 보이지 않게 되면 다른 색 물체를 대신 집지 말고 상황을 판단해.', World('lost', distance=3, x=0.65, blue_decoy=True, lose_after_first_move=True), 8))
# 5) no red target from the start; should avoid tools.
cases.append(('no_red_target', '빨간 블록을 집어. 화면에 빨간 블록이 없으면 억지로 행동하지 마.', World('nored', visible=False, blue_decoy=True), 6))

results=[]
for name,goal,world,limit in cases:
    print(f'\n===== {name} =====', flush=True)
    try:
        r=run_case(name,goal,world,limit)
    except Exception as e:
        r={'case':name,'exception':repr(e)}
    results.append(r)
    print(json.dumps(r,ensure_ascii=False,indent=2), flush=True)

out=ROOT/'results.json'
out.write_text(json.dumps(results,ensure_ascii=False,indent=2))
print('\nRESULTS_FILE',out)
