import pytest
from harness.carry_input_history import window_indices, wire_request, decode_request


def frame(role=0):
    return {'own_rgb': b'own', 'top_rgb': b'top', 'context': [0., 0., 1., 0., role, 0., 0., 0.]}


def test_causal_window_and_cold_start():
    assert window_indices(0, 4) == [0, 0, 0, 0]
    assert window_indices(2, 4) == [0, 0, 1, 2]
    assert window_indices(7, 4) == [4, 5, 6, 7]
    assert window_indices(7, 1) == [7]


def test_wire_roundtrip_and_boundary():
    fs = [frame() for _ in range(4)]
    assert decode_request(wire_request(fs, 4), 4) == fs
    fs[0]['context'][4] = 1.
    with pytest.raises(ValueError, match='boundary'):
        wire_request(fs, 4)
    for forbidden in ('qpos', 'teacher_action', 'success', 'contact', 'robot_pose'):
        value = wire_request([frame()], 1)
        value['frames'][0][forbidden] = 1
        with pytest.raises(ValueError, match='unapproved'):
            decode_request(value, 1)
    with pytest.raises(ValueError):
        decode_request(wire_request([frame()], 1), 4)


@pytest.mark.parametrize('mismatch', [None, 'seed', 'steps', 'batch_size', 'dataset_sha256', 'history', 'model', 'verification'])
def test_reused_model_must_match_fixed_protocol(tmp_path, mismatch):
    import json
    from scripts.run_carry_input_ablation import sha, validate_training
    root = tmp_path/'model'; (root/'act').mkdir(parents=True)
    (root/'act/model.safetensors').write_bytes(b'checkpoint-fixture')
    adapter = {'kind': 'carry_input_ablation', 'size': 128, 'history': 4}
    protocol = {'dataset_sha256': 'data-hash', 'training': {'steps': 8000, 'batch': 32}}
    report = {'complete': True, 'dataset_sha256': 'data-hash', 'seed': 20260921,
              'steps': 8000, 'batch_size': 32, 'adapter': adapter,
              'model_sha256': sha(root/'act/model.safetensors'),
              'initial_cache_verification': {'probes': [1]},
              'selected_cache_verification': {'train': {}, 'development': {}}}
    if mismatch in ('seed', 'steps', 'batch_size'):
        report[mismatch] += 1
    elif mismatch == 'dataset_sha256': report[mismatch] = 'other-data'
    elif mismatch == 'history': adapter['history'] = 1
    elif mismatch == 'model': (root/'act/model.safetensors').write_bytes(b'different')
    elif mismatch == 'verification': del report['selected_cache_verification']
    (root/'report.json').write_text(json.dumps(report))
    (root/'act/adapter.json').write_text(json.dumps(adapter))
    args = (root, {'size': 128, 'history': 4}, 20260921, protocol)
    if mismatch is None: assert validate_training(*args) == report
    else:
        with pytest.raises(ValueError): validate_training(*args)


def test_runtime_preserves_robot_windows_and_executed_pause_context(tmp_path,monkeypatch):
    import json
    from types import SimpleNamespace
    import scripts.dispatch_act_carry as runtime
    import harness.carry_input_client as clients
    (tmp_path/'adapter.json').write_text(json.dumps({'kind':'carry_input_ablation'}))
    class Client:
        history=4
        last_request_sha256='fixture'
        calls=0
        closed=False
        def __init__(self,*args):pass
        def predict(self,frames):
            index,robot=divmod(self.calls,2);self.calls+=1
            assert all(f['own_rgb']==('r'+str(robot)).encode() for f in frames)
            if index==1:assert frames[-1]['context'][-3:]==[0.,0.,0.]
            done=index>=2 or (index==0 and robot==0)
            return {'action':{'forward':.02,'left':0.,'turn':0.},'done':done,'stop_score':float(done)}
        def close(self):Client.closed=True
    monkeypatch.setattr(clients,'InputCarryClient',Client)
    monkeypatch.setattr(runtime,'OwnHoldContinuity',lambda rgb:SimpleNamespace(observe=lambda rgb:{'held_estimate':True}))
    monkeypatch.setattr(runtime,'authorize_pair',lambda *args:{'phase':'GO'})
    class Pair:
        calls=[]
        now=0.
        bindings=SimpleNamespace(pair={'r1':'r0','r3':'r1'},static_map={'docks':{'dock_a':{'slots':{'beam':{'center_m':[1.,1.]}}}}},plan={'dock':'dock_a'},tasks={'beam':{'route':'north'}},committed={'plan_hash':'abc'})
        def __init__(self):
            self.io=SimpleNamespace(capture=self.capture)
            self.tracked=[]
            def observe(rgb):
                self.tracked.append(rgb)
                return {'center':[.5,.5]}
            self.carried_beam=SimpleNamespace(observe=observe)
        def time(self):return self.now
        def capture(self,label):
            return {r:{'own_bytes':r.encode(),'top_bytes':b'top','own_rgb':{'path':r},'shared_top_rgb':{'path':'top'},'frame_id':int(self.now*100)} for r in ('r0','r1')}
        def drive_mecanum(self,actions,dt):
            if self.now==0:assert all(v==0 for a in actions.values() for v in a.values())
            self.now+=dt
    pair=Pair();runtime.carry(pair,None,tmp_path,10)
    assert Client.closed and len(pair.calls)==5
    assert pair.calls[0]['inputs']['r1']['history']==[pair.calls[0]['inputs']['r1']['history'][0]]*4
    assert pair.calls[-1]['ready_count']==3
    assert pair.tracked==[b'top']*6  # Anchor plus every frame, including done.
    assert all(r['release_tracking']['image']=={'path':'top'} for r in pair.calls)
    assert all('release_tracking' not in i for r in pair.calls for i in r['inputs'].values())
