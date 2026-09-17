"""Audited two-LLM skill barrier; receives RGB and issued commands only."""
from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
import io
import json
from urllib.request import urlopen

from harness.camera_skill_actor import build_skill_request, validate_skill_reply, pair_skill_ready
from harness.gemini_proxy import GeminiProxyCompleter
from harness.research_execution_recovery import request_with_recovery


class CameraSkillGate:
    def __init__(self, output, *, model='gemini-3.8-flash', timeout=30.):
        self.output=output
        self.previous={r:None for r in ('r1','r3')}
        self.inbox={r:[] for r in self.previous}
        self.calls=[]
        self.events=[]
        self.completers={}
        self.wires={r:0 for r in self.previous}
        for rid in self.previous:
            (output/rid).mkdir(parents=True)
            def audited_open(req,*,timeout,_rid=rid):
                self.wires[_rid]+=1
                path=output/_rid/f'wire-{self.wires[_rid]:03d}.json'
                path.write_bytes(req.data)
                with urlopen(req,timeout=timeout) as response: data=response.read()
                path.with_name(path.stem+'-response.json').write_bytes(data)
                return io.BytesIO(data)
            self.completers[rid]=GeminiProxyCompleter(model=model,max_tokens=600,
                timeout=timeout,reasoning_effort='none',http_open=audited_open)

    def decide(self, skill, frames, own_commands):
        index=len(self.events)
        def invoke(rid):
            records=[]
            def request(attempt,retry):
                request_id=f'{rid}-skill-{index:03d}-{skill}-a{attempt}'
                req=build_skill_request(rid,skill,request_id=request_id,
                    own_rgb=frames[rid]['own_bytes'],top_rgb=frames[rid]['top_bytes'],
                    previous=self.previous[rid],own_commands=own_commands[rid],
                    peer_claims=self.inbox[rid][-4:],retry=retry)
                path=f'{rid}/{index:03d}-a{attempt}-request.json'
                (self.output/path).write_text(json.dumps(req,ensure_ascii=False,indent=2)+'\n')
                return req
            def record(row):
                row.update(robot_id=rid,event=index,skill=skill,wire_index=self.wires[rid],
                    request=f'{rid}/{index:03d}-a{row["attempt"]}-request.json')
                records.append(row)
            reply,stop=request_with_recovery(self.completers[rid],request,
                lambda raw,req:validate_skill_reply(raw,req,skill),record,
                can_request=lambda:sum((c.get('usage') or {}).get('prompt_tokens',0) for c in self.calls)<300000)
            return reply,stop,records
        with ThreadPoolExecutor(max_workers=2) as pool:
            futures={r:pool.submit(invoke,r) for r in self.previous}
            batch={r:f.result() for r,f in futures.items()}
        replies={r:v[0] for r,v in batch.items()}
        for rid,(reply,stop,records) in batch.items():
            self.calls.extend(records)
            self.previous[rid]=(frames[rid]['own_bytes'],frames[rid]['top_bytes'])
        ready=pair_skill_ready(replies,skill)
        event=dict(index=index,skill=skill,ready=ready,replies=replies,
            stops={r:v[1] for r,v in batch.items()},own_commands=own_commands,
            images={r:dict(own=frames[r]['own_rgb'],top=frames[r]['shared_top_rgb']) for r in frames})
        self.events.append(event)
        for rid,reply in replies.items():
            if reply:
                peer='r3' if rid=='r1' else 'r1'
                self.inbox[peer].append(dict(from_robot=rid,skill=skill,message=reply['message']))
        (self.output/'gate.json').write_text(json.dumps(dict(calls=self.calls,events=self.events),ensure_ascii=False,indent=2)+'\n')
        print(json.dumps(dict(llm_stage=skill,ready=ready,replies=replies)),flush=True)
        return ready
