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


def audit(path):
    report=json.loads((path/'result.json').read_text())
    # Reconstruct using the exact committed, trusted project actor implementation.
    source=subprocess.check_output(['git','show',report['source_sha']+':harness/research_camera_actor.py'],text=True,cwd=ROOT)
    module=types.ModuleType('recorded_actor');exec(compile(source,'recorded_actor','exec'),module.__dict__)
    previous={r:None for r in ('r1','r3')};counts={r:0 for r in previous}
    calls=sorted(report['calls'],key=lambda c:(c['turn'],c['robot_id']))
    rows={row['turn']:row for row in report['turns']}
    audited_images=0
    for call in calls:
        rid=call['robot_id'];turn=call['turn'];counts[rid]+=1
        inputs=json.loads((path/rid/f'{turn:03d}-inputs.json').read_text())
        request=json.loads((path/rid/f'{turn:03d}-request.json').read_text())
        assert set(inputs)=={'phase','camera','roles','own_history','inbox','previous','communication'}
        expected_history=[row['issued'][rid] for row in report['turns'] if row['turn']<turn and rid in row['issued']][-16:]
        assert inputs['own_history']==expected_history
        assert inputs['previous']==previous[rid]
        messages=[m for m in report['messages'] if m['recipient']==rid and m['turn']<turn][-4:]
        assert inputs['inbox']==messages
        for message in messages:
            assert message['sender']!=rid and message['source']=='peer_claim'
            assert message['text']==rows[message['turn']]['replies'][message['sender']]['message']
        generated=module.build_request(rid,inputs['phase'],inputs['camera'],roles=inputs['roles'],
            own_history=inputs['own_history'],inbox=inputs['inbox'],previous=inputs['previous'],communication=inputs['communication'])
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
    for relative,sha in report['files'].items():
        assert hashlib.sha256((path/relative).read_bytes()).hexdigest()==sha,relative
    assert report['invariants_initial']==report['invariants_final']
    assert not report['evaluation']['any_weld']
    assert not report['cleanup_errors']
    assert counts==report['wire_requests']
    return {'source_sha':report['source_sha'],'trial':str(path.resolve()),'wire_requests':sum(counts.values()),
        'image_checks':audited_images,'manifest_files':len(report['files']),'exact_request_derivation':True,
        'exact_wire_payload':True,'own_history_provenance':True,'peer_message_provenance':True,
        'no_referee_input_in_reconstructed_requests':True,'camera_and_geometry_unchanged':True,'weld_off':True,
        'scope':'trusted local call graph + exact payload audit, not process isolation or proof that visual claims are correct'}


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('trial',type=Path);p.add_argument('--output',type=Path)
    a=p.parse_args();result=audit(a.trial)
    if a.output:a.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
