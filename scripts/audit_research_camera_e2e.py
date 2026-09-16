"""Offline audit of trusted local research-camera trials; no simulator access.

Run only on trusted project records: this replays actor code from the recorded
local Git commit. It is not a sandbox for untrusted repositories or records.
"""
import argparse
import base64
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import types

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from harness.gemini_proxy import _to_gemini_multi_image_messages


def recorded_module(sha, filename):
    source=subprocess.check_output(['git','show',sha+':'+filename],text=True,cwd=ROOT)
    module=types.ModuleType('recorded_'+Path(filename).stem)
    exec(compile(source,filename,'exec'),module.__dict__)
    return module


def audit(path):
    report=json.loads((path/'result.json').read_text())
    # Reconstruct using the exact committed, trusted project actor implementation.
    module=recorded_module(report['source_sha'],'harness/research_camera_actor.py')
    recovered='issued_commands' in report
    if recovered:
        recovery=recorded_module(report['source_sha'],'harness/research_execution_recovery.py')
    previous={r:None for r in ('r1','r3')};counts={r:0 for r in previous}
    calls=sorted(report['calls'],key=lambda c:(c['turn'],c['robot_id'],c.get('attempt',0)))
    rows={row['turn']:row for row in report['turns']}
    audited_images=0
    for call in calls:
        rid=call['robot_id'];turn=call['turn'];counts[rid]+=1
        inputs=json.loads((path/call['inputs'] if recovered else path/rid/f'{turn:03d}-inputs.json').read_text())
        request=json.loads((path/call['request'] if recovered else path/rid/f'{turn:03d}-request.json').read_text())
        allowed={'phase','camera','roles','own_history','inbox','previous','communication'}
        if recovered:
            allowed|={'request_id','retry','agreement','own_proposals','local_drive_steps'}
            expected_history=[{k:v for k,v in cmd.items() if k not in ('robot_id','turn')}
                for cmd in report['issued_commands'] if cmd['robot_id']==rid and cmd['turn']<turn][-16:]
            prior_calls=[c for c in calls if c['robot_id']==rid and c['turn']<turn]
            previous[rid]=json.loads((path/prior_calls[-1]['inputs']).read_text())['camera'] if prior_calls else None
            agreement=recovery.RoleAgreement()
            for row in report['turns']:
                if row['turn']<turn and row['phase']=='NEGOTIATE':agreement.receive(row['replies'],row['turn'])
            assert inputs['agreement']==agreement.context()
            assert inputs['own_proposals']==agreement.history[rid][-4:]
            assert inputs['roles']==(agreement.committed['roles'] if agreement.committed else None)
            assert inputs['request_id']==call['request_id']==f'{rid}-{turn:03d}-{inputs["phase"]}-a{call["attempt"]}'
            assert inputs['local_drive_steps']==report['config']['local_drive_steps']
            assert call['wire_index']==counts[rid]
            if call['attempt']==0:assert inputs['retry'] is None
            else:
                prior=next(c for c in calls if c['robot_id']==rid and c['turn']==turn and c['attempt']==call['attempt']-1)
                assert 'reply' not in prior and prior['retryable']
                expected_hint=({'kind':'reply_schema','detail':prior['error'].split(': ',1)[1],
                    'previous_response':prior.get('raw_response','')} if prior['error_kind']=='reply_schema' else
                    {'kind':prior['error_kind'],'detail':'Previous inference attempt failed; return a fresh reply for this request.'})
                assert inputs['retry']==expected_hint
                earlier=json.loads((path/prior['inputs']).read_text())
                assert {k:v for k,v in inputs.items() if k not in ('request_id','retry')}=={k:v for k,v in earlier.items() if k not in ('request_id','retry')}
            if 'reply' in call:
                assert module.validate_reply(call['raw_response'],inputs['phase'],request_id=inputs['request_id'],agreement=inputs['agreement'])==call['reply']
                if turn in rows:assert rows[turn]['replies'][rid]==call['reply']
        else:
            expected_history=[row['issued'][rid] for row in report['turns'] if row['turn']<turn and rid in row['issued']][-16:]
        assert set(inputs)==allowed
        assert inputs['own_history']==expected_history
        assert inputs['previous']==previous[rid]
        messages=[m for m in report['messages'] if m['recipient']==rid and m['turn']<turn][-4:]
        assert inputs['inbox']==messages
        for message in messages:
            assert message['sender']!=rid and message['source']=='peer_claim'
            assert message['text']==rows[message['turn']]['replies'][message['sender']]['message']
        generated=module.build_request(rid,**inputs)
        assert generated==request
        wire=json.loads((path/rid/f'wire-{counts[rid]:03d}.json').read_text())
        assert wire=={'model':report['config']['model'],'messages':_to_gemini_multi_image_messages(request['messages'],request['images']),
                      'temperature':.2,'max_tokens':900,'reasoning_effort':'none'}
        for pair in (inputs['camera'],inputs['previous']):
            if pair is None:continue
            for label,item in pair['images'].items():
                assert label in ('own_rgb','top_rgb')
                raw=base64.b64decode(item['jpeg_base64'])
                assert raw==(path/item['ref']).read_bytes()
                assert hashlib.sha256(raw).hexdigest()==item['sha256']
                audited_images+=1
        if 'raw_response' in call:
            response=json.loads((path/rid/f'wire-{counts[rid]:03d}-response.json').read_text())
            assert response['choices'][0]['message']['content']==call['raw_response']
        previous[rid]=inputs['camera']
    local_steps=0
    if recovered:
        # Replayed adapter imports must match the recorded source as well.
        for dependency in ('harness/camera_beam_features.py','harness/camera_motion_identity.py'):
            assert subprocess.check_output(['git','show',report['source_sha']+':'+dependency],cwd=ROOT)==(ROOT/dependency).read_bytes()
        visual=recorded_module(report['source_sha'],'harness/research_visual_lease.py')
        for window in report.get('settling_windows',[]):
            check=visual.VisualStillness((path/window['before']).read_bytes())
            assert 1<=len(window['frames'])<=10
            for i,frame in enumerate(window['frames']):
                verdict=check.update((path/frame['ref']).read_bytes())
                assert verdict==frame['decision']
                if i+1<len(window['frames']):assert not verdict['ready']
            if not verdict['ready']:
                assert not any(c['robot_id']==window['robot_id'] and c['turn']==window['turn'] for c in report['issued_commands'])
        commands=report['issued_commands']
        assert len({c['command_id'] for c in commands})==len(commands)
        for batch in report['local_batches']:
            def pixels(ref):
                raw=(path/ref['ref']).read_bytes()
                assert hashlib.sha256(raw).hexdigest()==ref['sha256']
                return raw
            lease=visual.VisualDriveLease(pixels(batch['before_top']),batch['action'],max_steps=batch['max_steps'])
            indices=[]
            assert batch['action']==rows[batch['turn']]['replies'][batch['robot_id']]['action']
            assert 1<=len(batch['steps'])<=batch['max_steps']<=10
            for i,step in enumerate(batch['steps']):
                decision=lease.after_step(pixels(step['top']))
                assert decision==step['decision']
                if i+1<len(batch['steps']):assert decision['renew']
                position=next(j for j,c in enumerate(commands) if c['command_id']==step['command_id'])
                indices.append(position);cmd=commands[position]
                assert cmd['robot_id']==batch['robot_id'] and cmd['phase']=='PREPARE'
                assert cmd['turn']==batch['turn'] and cmd['action']==batch['action'] and cmd['duration_s']==.2
                local_steps+=1
            assert indices==list(range(indices[0],indices[0]+len(indices)))
            assert not batch['steps'][-1]['decision']['renew']
        agreement=recovery.RoleAgreement()
        for row in report['turns']:
            if row['phase']=='NEGOTIATE':agreement.receive(row['replies'],row['turn'])
        assert agreement.events==report['agreement_events']
    for relative,sha in report['files'].items():
        assert hashlib.sha256((path/relative).read_bytes()).hexdigest()==sha,relative
    assert report['invariants_initial']==report['invariants_final']
    assert not report['evaluation']['any_weld']
    assert not report['cleanup_errors']
    assert counts==report['wire_requests']
    return {'source_sha':report['source_sha'],'trial':str(path.resolve()),'wire_requests':sum(counts.values()),
        'image_checks':audited_images,'manifest_files':len(report['files']),'exact_request_derivation':True,
        'exact_wire_payload':True,'own_history_provenance':True,'peer_message_provenance':True,
        'local_rgb_decisions_replayed':local_steps,'retry_and_role_version_audited':recovered,
        'no_referee_input_in_reconstructed_requests':True,'camera_and_geometry_unchanged':True,'weld_off':True,
        'scope':'trusted local call graph + exact payload audit, not process isolation or proof that visual claims are correct'}


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('trial',type=Path);p.add_argument('--output',type=Path)
    a=p.parse_args();result=audit(a.trial)
    if a.output:a.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
