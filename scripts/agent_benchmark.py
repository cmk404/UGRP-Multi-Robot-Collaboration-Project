from __future__ import annotations
import json, time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any
from PIL import Image, ImageDraw
from harness.catalog import default_registry
from harness.groq import GroqCompleter
from harness.loop import run_loop
from harness.protocol import ToolCall, Plan
from harness.verify import verify_final as visual_verify_final

ROOT=Path(__file__).resolve().parents[1] / 'outputs' / 'agent_benchmark_frames'; ROOT.mkdir(parents=True,exist_ok=True)

@dataclass
class World:
    name:str
    distance:int=3
    x:float=.5
    visible:bool=True
    held:bool=False
    stable:bool=False
    blue_decoy:bool=False
    forced_pick_failures:int=0
    slip_after_first_grasp:bool=False
    lose_after_first_approach:bool=False
    pick_attempts:int=0
    grasp_count:int=0
    observations:int=0
    actions:list[str]=field(default_factory=list)
    frame:int=0
    slipped:bool=False
    lost:bool=False

    def run(self,name:str)->dict[str,Any]:
        self.actions.append(name)
        if name=='approach':
            if self.visible and not self.held:
                self.distance=1
        elif name=='track':
            if self.visible and not self.held:
                self.x=.5
        elif name=='pick':
            self.pick_attempts += 1
            if self.visible and not self.held and self.distance==1 and abs(self.x-.5)<=.16 and self.pick_attempts>self.forced_pick_failures:
                self.held=True; self.stable=True; self.grasp_count+=1
        elif name=='fetch':
            if self.visible and not self.held:
                self.distance=1; self.x=.5; self.pick_attempts+=1
                if self.pick_attempts>self.forced_pick_failures:
                    self.held=True; self.stable=True; self.grasp_count+=1
        return {'ok':True,'skill':name,'exit_code':0}

    def disturb_before_observe(self):
        self.observations += 1
        if self.lose_after_first_approach and 'approach' in self.actions and not self.lost:
            self.visible=False; self.held=False; self.stable=False; self.lost=True
        if self.slip_after_first_grasp and self.grasp_count>=1 and not self.slipped:
            self.held=False; self.stable=False; self.visible=True; self.distance=1; self.x=.78; self.slipped=True

    def render(self,observe=False):
        if observe: self.disturb_before_observe()
        self.frame+=1
        W,H=640,480
        im=Image.new('RGB',(W,H),(224,229,234)); d=ImageDraw.Draw(im)
        d.polygon([(0,300),(W,300),(W,H),(0,H)],fill=(173,168,158))
        for y in (335,385,440): d.line((0,y,W,y),fill=(145,140,132),width=2)
        for x in (100,220,320,420,540): d.line((320,300,x,H),fill=(152,147,139),width=1)
        # gripper
        d.rectangle((305,0,335,95),fill=(70,75,82)); d.rectangle((270,82,315,102),fill=(70,75,82)); d.rectangle((325,82,370,102),fill=(70,75,82))
        if self.held:
            d.rectangle((292,82,348,138),fill=(220,25,35),outline=(120,0,0),width=4)
            d.rectangle((275,90,292,128),fill=(55,60,66)); d.rectangle((348,90,365,128),fill=(55,60,66))
        elif self.visible:
            sz={3:52,2:92,1:150}[self.distance]; cx=int(self.x*W); cy={3:285,2:310,1:335}[self.distance]
            d.rectangle((cx-sz//2,cy-sz//2,cx+sz//2,cy+sz//2),fill=(220,25,35),outline=(120,0,0),width=4)
            d.polygon([(cx-sz//2,cy-sz//2),(cx-sz//2+16,cy-sz//2-12),(cx+sz//2+16,cy-sz//2-12),(cx+sz//2,cy-sz//2)],fill=(245,65,72))
        if self.blue_decoy:
            bx,by,bs=155,330,115
            d.rectangle((bx-bs//2,by-bs//2,bx+bs//2,by+bs//2),fill=(30,90,220),outline=(10,40,120),width=4)
        out=ROOT/f'{self.name}-{self.frame:02d}.jpg'; im.save(out,quality=90); return str(out)

def run_case(name,goal,w,max_steps=12):
    model=GroqCompleter(max_tokens=256,temperature=.2)
    reg=default_registry(runner=w.run)
    initial=w.render()
    def observe(): return w.render(observe=True)
    def verifier(g,f,img): return visual_verify_final(model,g,f,img)
    t=time.time()
    r=run_loop(model,reg,goal,image=initial,max_steps=max_steps,execute=True,observe=observe,auto_observe=True,verify_final=verifier,sleeper=lambda _:None)
    trace=[]
    for i,s in enumerate(r.steps,1):
        x={'i':i,'kind':type(s.action).__name__ if s.action else None,'error':s.error,'raw':s.raw}
        if isinstance(s.action,ToolCall): x['tool']=s.action.name
        if isinstance(s.action,Plan): x['plan']=list(s.action.steps)
        trace.append(x)
    return {'case':name,'elapsed_s':round(time.time()-t,2),'stopped':r.stopped,'final':r.final,
      'world':{'visible':w.visible,'held':w.held,'stable':w.stable,'distance':w.distance,'x':round(w.x,2),'pick_attempts':w.pick_attempts,'grasp_count':w.grasp_count,'observations':w.observations,'actions':w.actions,'slipped':w.slipped,'lost':w.lost},
      'errors':sum(bool(x['error']) for x in trace),'unknown_tools':sum('unknown tool' in (x['error'] or '') for x in trace),'trace':trace}

cases=[
 ('offcenter_recovery','빨간 블록을 집어서 안정적으로 들어 올려. 실패하면 최신 장면을 보고 적절히 정렬하거나 다시 시도해.',World('offcenter',distance=1,x=.12)),
 ('slip_recovery','빨간 블록을 집어서 안정적으로 들어 올려. 집은 뒤 놓치더라도 최신 장면을 보고 스스로 복구해.',World('slip',distance=3,x=.52,slip_after_first_grasp=True)),
 ('double_pick_failure','빨간 블록을 집어서 안정적으로 들어 올려. 집기가 실패하면 원인을 보고 스스로 다시 시도해.',World('double',distance=2,x=.5,forced_pick_failures=2)),
 ('target_lost_decoy','빨간 블록을 집어. 도중에 빨간 블록이 사라지면 파란 블록을 대신 집지 말고 안전하게 판단해.',World('lost',distance=3,x=.65,blue_decoy=True,lose_after_first_approach=True)),
 ('no_red_target','빨간 블록을 집어. 빨간 블록이 화면에 없으면 억지로 행동하지 마.',World('none',visible=False,blue_decoy=True)),
]
results=[]
for spec in cases:
    print('\n=====',spec[0],'=====',flush=True)
    try: r=run_case(*spec)
    except Exception as e: r={'case':spec[0],'exception':repr(e)}
    results.append(r); print(json.dumps(r,ensure_ascii=False,indent=2),flush=True)
(ROOT/'results.json').write_text(json.dumps(results,ensure_ascii=False,indent=2))
