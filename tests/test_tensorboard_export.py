"""Run with requirements-observability.txt; core validation also runs offline."""
import hashlib
import json
from pathlib import Path
import threading
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest

from scripts.tensorboard_tools.export import Source, inside, redact, sample_indices
from scripts.tensorboard_tools.media import media_registry, make_server


def put(root, name, value):
    p=root/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(value));return p


def test_sample_includes_boundaries_without_inventing_frames():
    assert sample_indices(100,3)=={0,50,99}
    assert sample_indices(1,8)=={0}
    assert sample_indices(0,8)==set()
    assert sample_indices(8,0)==set()
    assert sample_indices(8,1)=={7}


def test_refuses_external_or_hash_mismatched_image(tmp_path):
    src=tmp_path/'source';src.mkdir();(tmp_path/'private.jpg').write_bytes(b'private')
    (src/'link.jpg').symlink_to(tmp_path/'private.jpg')
    (src/'own.jpg').write_bytes(b'image')
    reader=Source(src)
    assert inside(src,'../private.jpg') is None
    assert reader.image({'path':'link.jpg'}) is None
    assert reader.image({'path':'own.jpg','sha256':'incorrect'}) is None
    assert len(reader.warnings)==2


def test_redaction_keeps_usage_but_removes_credentials():
    result=redact({'Authorization':'Bearer private','input_tokens':10,'nested':'{"api_key":"private"}'})
    assert 'private' not in json.dumps(result)
    assert result['input_tokens']==10


def test_incomplete_json_is_recorded_as_warning(tmp_path):
    (tmp_path/'result.json').write_text('{')
    source=Source(tmp_path)
    assert source.read('result.json') is None
    assert source.warnings
    with pytest.raises(ValueError): source.read('result.json',required=True)


@pytest.fixture
def export_api():
    pytest.importorskip('tensorboard')
    pytest.importorskip('PIL')
    from scripts.tensorboard_tools.export import convert
    from tensorboard.backend.event_processing.event_accumulator import EventAccumulator
    return convert,EventAccumulator


def test_training_roundtrip_preserves_steps_and_labels_export_time(tmp_path,export_api):
    convert,EA=export_api
    src=tmp_path/'source';src.mkdir()
    report=put(src,'report.json',{'progress':[{'step':1,'loss':2.,'elapsed_s':4.},
        {'step':500,'loss':.2,'elapsed_s':8.,'development':{'selection_score':.5}}],
        'source_sha':'abc','complete':True,'seed':18})
    before=report.read_bytes()
    manifest=convert(src,tmp_path/'export')
    ea=EA(str(tmp_path/'export')).Reload()
    assert [(x.step,round(x.value,2)) for x in ea.Scalars('training/loss')]==[(1,2.),(500,.2)]
    assert all(x.wall_time==manifest['exported_at_s'] for x in ea.Scalars('training/loss'))
    assert manifest['source_files']['report.json']['sha256']==hashlib.sha256(before).hexdigest()
    assert report.read_bytes()==before
    assert 'text' in ea.PluginTagToContent('text') or 'provenance/source' in ea.PluginTagToContent('text')
    with pytest.raises(FileExistsError): convert(src,tmp_path/'export')


def test_deployment_objective_keeps_loss_parts_and_failed_selection(tmp_path,export_api):
    convert,EA=export_api
    src=tmp_path/'source';src.mkdir()
    put(src,'report.json',{'progress':[{'step':500,'loss':.6,
        'loss_components':{'act_total':.1,'weighted_deployed_done':.5},
        'selection_eligible':False}], 'complete':True,
        'deployed_done_objective':{'weight':1.,'runtime_score_threshold':.65},
        'first_batch_deployed_done':{'action_head_done_gradient_norm':.2},
        'selected_checkpoint_eligible':False})
    convert(src,tmp_path/'export')
    ea=EA(str(tmp_path/'export')).Reload()
    assert ea.Scalars('training/loss_components/act_total')[0].value==pytest.approx(.1)
    assert ea.Scalars('training/loss_components/weighted_deployed_done')[0].value==pytest.approx(.5)
    assert [(x.step,x.value) for x in ea.Scalars('development/checkpoint_eligible')]==[(500,0.)]
    assert 'evaluation/reported_success' not in ea.Tags()['scalars']
    text=ea.Tensors('training/deployed_objective')[0].tensor_proto.string_val[0].decode()
    assert 'selected_checkpoint_eligible' in text and 'false' in text.lower()


def test_failed_evaluation_remains_failed_despite_done_claim(tmp_path,export_api):
    convert,EA=export_api;src=tmp_path/'source';src.mkdir()
    put(src,'result.json',{'success':False,'stop_reason':'RGB_goal_confirmed','protocol_complete':True,
        'cost_usd':None,'wall_s':0,'scope':'approach only'})
    manifest=convert(src,tmp_path/'export')
    ea=EA(str(tmp_path/'export')).Reload()
    assert ea.Scalars('evaluation/reported_success')[0].value==0
    assert ea.Scalars('claims/protocol_complete')[0].value==1
    assert ea.Scalars('result/wall_s')[0].value==0
    assert 'result/cost_usd' not in ea.Tags()['scalars']
    assert manifest['metadata']['success_source_field']=='success'


@pytest.mark.parametrize('terminal_score',[.05,.9])
def test_offline_termination_audit_is_recomputed_and_never_physical_success(tmp_path,export_api,terminal_score):
    from scripts.audit_carry_termination import audit
    convert,EA=export_api;src=tmp_path/'audit';src.mkdir()
    predictions=[{'id':f'development:{slot}:{i}',
                  'prediction':[0,0,0,terminal_score if i else 0.],
                  'target':[0,0,0,float(i)]} for slot in ('r1','r3') for i in range(2)]
    p=put(src,'predictions.json',predictions)
    report=audit(predictions);report['predictions_sha256']=hashlib.sha256(p.read_bytes()).hexdigest()
    put(src,'termination-audit.json',report)
    manifest=convert(src,tmp_path/'export');ea=EA(str(tmp_path/'export')).Reload()
    assert ea.Scalars('offline/missed_terminal_episodes')[0].value==int(terminal_score<.65)
    assert ea.Scalars('offline/termination_pass')[0].value==int(terminal_score>=.65)
    assert manifest['metadata']['outcome']==('offline_pass' if terminal_score>=.65 else 'offline_fail')
    assert 'evaluation/reported_success' not in ea.Tags()['scalars']
    assert manifest['metadata']['success_source_field'] is None
    report['missed_terminal_episodes']=999;put(src,'termination-audit.json',report)
    with pytest.raises(ValueError,match='saved predictions'):convert(src,tmp_path/'bad-count')
    p.write_text('[]')
    with pytest.raises(ValueError,match='hash mismatch'):convert(src,tmp_path/'bad-hash')


def test_missing_success_is_not_zero(tmp_path,export_api):
    convert,EA=export_api;src=tmp_path/'source';src.mkdir()
    put(src,'result.json',{'protocol_complete':True})
    convert(src,tmp_path/'export')
    assert 'evaluation/reported_success' not in EA(str(tmp_path/'export')).Reload().Tags()['scalars']


def test_dispatch_plan_receipts_keep_agreement_separate_from_transport(tmp_path,export_api):
    convert,EA=export_api;src=tmp_path/'source';src.mkdir()
    put(src,'result.json',{'plan_committed':True,'protocol_complete':False,'physical_success':False,'llm_calls':3})
    put(src,'skill-bindings.json',{'solo_robot':'r2'})
    put(src,'team/team.json',{'calls':[{'latency_ms':1000},{'latency_ms':2000},{'latency_ms':500}]})
    put(src,'issued-commands.json',{'r1':[{'stage':'SETUP'},{'action':{'kind':'drive'}}],
        'r2':[{'action':{'kind':'wait'}}],'r3':[]})
    manifest=convert(src,tmp_path/'export')
    ea=EA(str(tmp_path/'export')).Reload()
    assert ea.Scalars('claims/plan_committed')[0].value==1
    assert ea.Scalars('claims/protocol_complete')[0].value==0
    assert ea.Scalars('evaluation/reported_success')[0].value==0
    assert ea.Scalars('result/model_latency_s')[0].value==3.5
    assert ea.Scalars('result/recorded_raw_commands')[0].value==2
    assert 'excludes' in manifest['metadata']['command_count_scope']
    assert 'team/team.json' in manifest['source_files']


def test_postrun_concurrency_audit_keeps_original_and_checks_every_hash(tmp_path,export_api):
    convert,EA=export_api;src=tmp_path/'source';src.mkdir()
    original=put(src,'result.json',{'physical_success':True,'evaluation':{'concurrent_transport':{'simultaneous_loaded_motion_s':0}}}).read_bytes()
    put(src,'issued-commands.json',{});(src/'referee-only.jsonl').write_text('{}\n')
    put(src,'concurrency-audit.json',{'source_files_sha256':{n:hashlib.sha256((src/n).read_bytes()).hexdigest()
        for n in ('result.json','issued-commands.json','referee-only.jsonl')},'concurrent_transport':{'simultaneous_loaded_motion_s':2.5}})
    m=convert(src,tmp_path/'export')
    assert EA(str(tmp_path/'export')).Reload().Scalars('evaluation/simultaneous_loaded_motion_s')[0].value==2.5
    assert (src/'result.json').read_bytes()==original
    assert 'concurrency-audit.json' in m['source_files']
    (src/'referee-only.jsonl').write_text('{"changed":true}\n')
    with pytest.raises(ValueError,match='source hash mismatch'):convert(src,tmp_path/'bad-export')


def test_images_and_sim_time_are_recoverable(tmp_path,export_api):
    convert,EA=export_api
    from PIL import Image
    src=tmp_path/'source';src.mkdir();Image.new('RGB',(8,6),'red').save(src/'image.png')
    put(src,'result.json',{'success':True,'policy':'rule'})
    put(src,'turns.json',[{'observed_at_sim_s':3.5,'action':'stop','observation':{'range_m':.28},
        'images':{'own_rgb':{'path':'image.png'},'shared_top_rgb':{'path':'image.png'}}}])
    manifest=convert(src,tmp_path/'export')
    ea=EA(str(tmp_path/'export')).Reload()
    assert ea.Scalars('execution/sim_time_s')[0].value==3.5
    assert ea.Images('observations/r2/own')[0].width==8
    assert manifest['counts']['images']==2


def test_multi_object_request_binding_uses_request_id(tmp_path,export_api):
    convert,EA=export_api;src=tmp_path/'source';src.mkdir()
    put(src,'result.json',{'transport_success':False})
    put(src,'actor-static-task.json',{})
    put(src,'runtime/request-B.json',{'task_id':'correct-task'})
    put(src,'turn-000.json',{'at_s':4.,'replies':{'r1':{'request_id':'request-B','status':'UNCERTAIN','confidence':0.}}})
    convert(src,tmp_path/'export',max_images=0)
    ea=EA(str(tmp_path/'export')).Reload()
    assert b'correct-task' in ea.Tensors('decisions/r1')[0].tensor_proto.string_val[0]
    assert ea.Scalars('model_confidence_not_success/r1')[0].value==0


def test_act_slot_maps_to_physical_robot(tmp_path,export_api):
    convert,EA=export_api;src=tmp_path/'source';src.mkdir()
    put(src,'result.json',{'physical_success':False,'config':{'carry_act_model':'model-18/act'}})
    put(src,'pair-decisions.json',[{'kind':'act_carry','sim_time_s':7.,
        'inputs':{'r1':{'physical_robot_id':'r3'}},'decisions':{'r1':{'done':True,'action':{'forward':.1}}}}])
    manifest=convert(src,tmp_path/'export',max_images=0)
    ea=EA(str(tmp_path/'export')).Reload()
    assert ea.Scalars('claims/r3/done')[0].value==1
    assert manifest['metadata']['act_carry_decision_rows']==1


def test_dispatch_counts_local_responses_commands_and_latency(tmp_path,export_api):
    convert,EA=export_api;src=tmp_path/'source';src.mkdir()
    put(src,'result.json',{'physical_success':False,'llm_calls':2})
    put(src,'pair-decisions.json',[{'kind':'act_carry',
        'inputs':{'r1':{'inference_wall_s':.04,'wire_sha256':'a'*64},
                  'r3':{'inference_wall_s':.06,'wire_sha256':'b'*64}},
        'decisions':{'r1':{'done':False},'r3':{'done':True}}}])
    put(src,'issued-commands.json',{'r1':[
        {'stage':'SETUP','issued_servo_targets':{'1':2000}},
        {'stage':'TRANSIT','action':{'kind':'mecanum','forward':.1}}],
        'r3':[{'stage':'GRASP','issued_servo_targets':{'1':1500}}]})
    manifest=convert(src,tmp_path/'export',max_images=0)
    ea=EA(str(tmp_path/'export')).Reload()
    assert [x.value for x in ea.Scalars('result/model_calls')]==[4]
    assert [x.value for x in ea.Scalars('result/commands')]==[2]
    assert [x.value for x in ea.Scalars('execution/model_latency_s')]==pytest.approx([.04,.06])
    assert [x.step for x in ea.Scalars('execution/model_latency_s')]==[0,1]
    assert manifest['metadata']['completed_act_responses']==2
    assert 'issued-commands.json' in manifest['source_files']


def test_dispatch_counts_stale_act_predictions_without_issuing_them(tmp_path,export_api):
    convert,EA=export_api;src=tmp_path/'source';src.mkdir()
    put(src,'result.json',{'physical_success':False,'llm_calls':0})
    put(src,'pair-decisions.json',[
        {'kind':'act_stale_capture','frame_id':1,'observed_at_s':1.},
        {'kind':'act_stale_prediction','index':0,'received_at_s':2.,
         'inputs':{'r1':{'physical_robot_id':'r1','inference_wall_s':.08,'wire_sha256':'a'*64},
                   'r3':{'physical_robot_id':'r3','inference_wall_s':.09,'wire_sha256':'b'*64}},
         'decisions':{'r1':{'done':True,'action':{'forward':.1}},
                      'r3':{'done':False,'action':{'forward':.1}}}},
        {'kind':'act_carry','index':0,'sim_time_s':2.2,
         'inputs':{'r1':{'physical_robot_id':'r1','inference_wall_s':.04,'wire_sha256':'c'*64},
                   'r3':{'physical_robot_id':'r3','inference_wall_s':.05,'wire_sha256':'d'*64}},
         'decisions':{'r1':{'done':False,'action':{'forward':.02}},
                      'r3':{'done':False,'action':{'forward':.02}}}}])
    manifest=convert(src,tmp_path/'export',max_images=0)
    ea=EA(str(tmp_path/'export')).Reload()
    assert ea.Scalars('result/model_calls')[0].value==4
    assert [x.value for x in ea.Scalars('execution/model_latency_s')]==pytest.approx([.08,.09,.04,.05])
    assert [x.value for x in ea.Scalars('execution/act_stale_prediction')]==[1,1,0,0]
    assert [x.value for x in ea.Scalars('claims/r1/done')]==[0]
    assert manifest['metadata']['accepted_act_responses']==2
    assert manifest['metadata']['stale_act_responses']==2
    assert manifest['metadata']['completed_act_responses']==4


def test_dispatch_counts_partial_inference_error_as_attempt(tmp_path,export_api):
    convert,EA=export_api;src=tmp_path/'source';src.mkdir()
    put(src,'result.json',{'physical_success':False,'llm_calls':0})
    put(src,'pair-decisions.json',[
        {'kind':'act_inference_error','index':0,'observed_at_s':1.,
         'received_at_s':1.2,'error':'worker exited',
         'inputs':{'r1':{'physical_robot_id':'r1','inference_wall_s':.03,
                         'wire_sha256':'a'*64,'response_received':True},
                   'r3':{'physical_robot_id':'r3','inference_wall_s':.05,
                         'wire_sha256':'b'*64,'response_received':False}},
         'decisions':{'r1':{'done':False}}}])
    manifest=convert(src,tmp_path/'export',max_images=0)
    ea=EA(str(tmp_path/'export')).Reload()
    assert ea.Scalars('result/model_calls')[0].value==2
    assert [x.value for x in ea.Scalars('execution/act_request_outcome')]==[2,2]
    assert manifest['metadata']['completed_act_responses']==1
    assert manifest['metadata']['attempted_act_requests']==2
    assert manifest['metadata']['act_request_verification_complete'] is True
    rows=json.loads((src/'pair-decisions.json').read_text())
    rows[0]['inputs']['r3'].pop('wire_sha256')
    put(src,'pair-decisions.json',rows)
    incomplete=convert(src,tmp_path/'export-incomplete',max_images=0)
    assert incomplete['metadata']['unverified_error_act_attempts']==1
    assert incomplete['metadata']['act_request_verification_complete'] is False
    assert EA(str(tmp_path/'export-incomplete')).Reload().Scalars('result/model_calls')[0].value==1


def test_dispatch_owner_abort_keeps_unconfirmed_slots_out_of_model_calls(tmp_path,export_api):
    convert,EA=export_api;src=tmp_path/'source';src.mkdir()
    put(src,'result.json',{'physical_success':False,'llm_calls':0,'model_calls':2})
    put(src,'pair-decisions.json',[{'kind':'act_inference_error','index':0,
        'discard_reason':'owner_aborted','worker_completion':'timeout',
        'owner_error':'RuntimeError: box lost','unconfirmed_slots':['r3'],
        'inputs':{'r1':{'physical_robot_id':'r1','wire_sha256':'a'*64,
                        'response_received':True,'inference_wall_s':.04}},
        'decisions':{'r1':{'done':False}}}])
    manifest=convert(src,tmp_path/'export',max_images=0)
    ea=EA(str(tmp_path/'export')).Reload()
    assert ea.Scalars('result/model_calls')[0].value==1
    assert ea.Scalars('execution/act_unconfirmed_slots')[0].value==1
    assert manifest['metadata']['reported_model_calls']==2
    assert manifest['metadata']['unconfirmed_act_slots']==1
    assert manifest['metadata']['act_request_verification_complete'] is False
    assert ea.Tags()['tensors'].count('inference_errors/unconfirmed_slots')==1


def coverage_fixture(tmp_path):
    src=tmp_path/'source';src.mkdir()
    put(src,'result.json',{'physical_success':False,'llm_calls':0})
    decisions=put(src,'pair-decisions.json',[{'kind':'act_carry',
        'inputs':{'r1':{'wire_sha256':'a'*64},'r3':{'wire_sha256':'b'*64}},
        'decisions':{'r1':{'done':False},'r3':{'done':False}}}])
    tag='act-carry-1-attempt-0'
    files={}
    for slot in ('r1','r3','top'):
        path=f'rgb/{tag}-{slot}.jpg';data=f'{slot}-orphan'.encode()
        target=src/path;target.parent.mkdir(exist_ok=True);target.write_bytes(data)
        files[slot]={'path':path,'sha256':hashlib.sha256(data).hexdigest()}
    audit={'schema':'ugrp.act_request_coverage_audit.v1','source_raw':str(src.resolve()),
           'pair_decisions_sha256':hashlib.sha256(decisions.read_bytes()).hexdigest(),
           'recorded_wire_request_rows':2,'possible_unlogged_slots':{'min':0,'max':2},
           'worker_receipt_independently_verified':False,
           'orphan_capture_groups':[{'tag':tag,'files':files}]}
    sidecar=put(tmp_path,'coverage.json',audit)
    return src,sidecar


def test_external_coverage_marks_unknown_without_inventing_model_calls(tmp_path,export_api):
    convert,EA=export_api
    src,sidecar=coverage_fixture(tmp_path)
    original=(src/'pair-decisions.json').read_bytes()
    manifest=convert(src,tmp_path/'export',max_images=0,coverage_audit=sidecar)
    ea=EA(str(tmp_path/'export')).Reload()
    meta=manifest['metadata']
    assert meta['attempted_act_requests']==2
    assert meta['act_request_verification_complete'] is False
    assert (meta['act_possible_unlogged_slots_min'],meta['act_possible_unlogged_slots_max'])==(0,2)
    assert (meta['act_possible_request_slot_total_min'],meta['act_possible_request_slot_total_max'])==(2,4)
    assert ea.Scalars('result/model_calls')[0].value==2
    assert ea.Scalars('execution/act_possible_unlogged_slots_max')[0].value==2
    assert 'inference_errors/coverage_audit' in ea.Tags()['tensors']
    assert manifest['external_coverage_audit']['sha256']==hashlib.sha256(sidecar.read_bytes()).hexdigest()
    assert (src/'pair-decisions.json').read_bytes()==original
    uncorrected=convert(src,tmp_path/'uncorrected',max_images=0)
    assert uncorrected['metadata']['act_request_verification_complete'] is True


@pytest.mark.parametrize('damage', ['missing','source','decisions','count','bounds','image','referenced','receipt'])
def test_external_coverage_mismatch_fails_before_export(tmp_path,export_api,damage):
    convert,_=export_api
    src,sidecar=coverage_fixture(tmp_path)
    audit=json.loads(sidecar.read_text())
    if damage=='missing':sidecar.unlink()
    elif damage=='source':audit['source_raw']=str(tmp_path/'other')
    elif damage=='decisions':audit['pair_decisions_sha256']='0'*64
    elif damage=='count':audit['recorded_wire_request_rows']=3
    elif damage=='bounds':audit['possible_unlogged_slots']['max']=1
    elif damage=='image':audit['orphan_capture_groups'][0]['files']['top']['sha256']='0'*64
    elif damage=='receipt':audit['worker_receipt_independently_verified']=True
    elif damage=='referenced':
        rows=json.loads((src/'pair-decisions.json').read_text())
        rows[0]['inputs']['r1']['images']={'own':audit['orphan_capture_groups'][0]['files']['r1']}
        put(src,'pair-decisions.json',rows)
        audit['pair_decisions_sha256']=hashlib.sha256((src/'pair-decisions.json').read_bytes()).hexdigest()
    if damage!='missing':sidecar.write_text(json.dumps(audit))
    with pytest.raises(ValueError,match='Coverage audit'):
        convert(src,tmp_path/'export',max_images=0,coverage_audit=sidecar)
    assert not (tmp_path/'export').exists()


def test_coverage_cli_maps_one_audit_among_multiple_sources(tmp_path,export_api,monkeypatch):
    _,EA=export_api
    from scripts.tensorboard_tools.export import main
    src,sidecar=coverage_fixture(tmp_path)
    other=tmp_path/'other';other.mkdir()
    put(other,'result.json',{'physical_success':False})
    out=tmp_path/'collection'
    monkeypatch.setattr('sys.argv',['export','--source',str(src),'--source',str(other),
                                   '--coverage-audit',str(sidecar),'--output',str(out),
                                   '--max-images','0'])
    assert main()==0
    collection=json.loads((out/'collection.json').read_text())
    assert len(collection['exported'])==2 and collection['failed']==[]
    manifests={json.loads((out/item['name']/'manifest.json').read_text())['source']:
               json.loads((out/item['name']/'manifest.json').read_text())
               for item in collection['exported']}
    assert manifests[str(src)]['metadata']['act_request_verification_complete'] is False
    assert 'external_coverage_audit' not in manifests[str(other)]
    assert EA(str(out/collection['exported'][0]['name'])).Reload().Scalars('result/model_calls')[0].value==2
    monkeypatch.setattr('sys.argv',['export','--source',str(other),
                                   '--coverage-audit',str(sidecar),'--output',str(tmp_path/'rejected')])
    with pytest.raises(SystemExit):main()


def test_coverage_is_not_silently_ignored_by_communication_early_return(tmp_path,export_api):
    convert,_=export_api
    from scripts.tensorboard_tools.rgb_communication import RUN_SCHEMA
    src,sidecar=coverage_fixture(tmp_path)
    result=json.loads((src/'result.json').read_text())
    result.update(schema_version=RUN_SCHEMA,evidence_kind='live_llm')
    put(src,'result.json',result)
    with pytest.raises(ValueError,match='Coverage audit applies only to ACT dispatch'):
        convert(src,tmp_path/'export',max_images=0,coverage_audit=sidecar)
    assert json.loads((tmp_path/'export'/'manifest.json').read_text())['complete'] is False


def test_configured_act_does_not_invent_responses_or_latency(tmp_path,export_api):
    convert,EA=export_api;src=tmp_path/'source';src.mkdir()
    put(src,'result.json',{'physical_success':False,'llm_calls':0,
                         'config':{'carry_act_model':'model/act'}})
    put(src,'pair-decisions.json',[])
    convert(src,tmp_path/'export',max_images=0)
    ea=EA(str(tmp_path/'export')).Reload()
    assert ea.Scalars('result/model_calls')[0].value==0
    assert 'execution/model_latency_s' not in ea.Tags()['scalars']


def test_explicit_dispatch_total_is_not_counted_twice(tmp_path,export_api):
    convert,EA=export_api;src=tmp_path/'source';src.mkdir()
    put(src,'result.json',{'model_calls':3,'commands':7,'llm_calls':1})
    put(src,'pair-decisions.json',[{'kind':'act_carry',
        'inputs':{'r1':{'wire_sha256':'a'*64},'r3':{'wire_sha256':'b'*64}},
        'decisions':{'r1':{'done':False},'r3':{'done':False}}}])
    manifest=convert(src,tmp_path/'export',max_images=0)
    ea=EA(str(tmp_path/'export')).Reload()
    assert ea.Scalars('result/model_calls')[0].value==3
    assert manifest['metadata']['reported_model_calls']==3
    assert manifest['metadata']['act_request_verification_complete'] is True
    assert ea.Scalars('result/commands')[0].value==7


def test_invalid_source_leaves_no_events(tmp_path,export_api):
    convert,_=export_api;src=tmp_path/'source';src.mkdir()
    put(src,'report.json',{'progress':[{'step':5,'loss':1},{'step':4,'loss':.5}]})
    with pytest.raises(ValueError): convert(src,tmp_path/'export')
    assert not list((tmp_path/'export').glob('*tfevents*'))
    assert json.loads((tmp_path/'export/manifest.json').read_text())['complete'] is False
    with pytest.raises(ValueError): convert(src,src/'inside')


def test_live_source_change_is_not_published(tmp_path,export_api,monkeypatch):
    convert,_=export_api
    from scripts.tensorboard_tools import export as module
    src=tmp_path/'source';src.mkdir();put(src,'result.json',{'success':True})
    original=module.Writer.hparams
    def mutate(self,*args):
        original(self,*args);put(src,'result.json',{'success':False})
    monkeypatch.setattr(module.Writer,'hparams',mutate)
    with pytest.raises(ValueError,match='Source changed'):convert(src,tmp_path/'export')
    assert not list((tmp_path/'export').glob('*tfevents*'))


def test_hparams_declares_shared_metric_schema(tmp_path,export_api):
    convert,EA=export_api
    from tensorboard.plugins.hparams import metadata
    src=tmp_path/'source';src.mkdir();put(src,'result.json',{'success':False})
    convert(src,tmp_path/'export')
    ea=EA(str(tmp_path/'export')).Reload()
    content=ea.PluginTagToContent('hparams')[metadata.EXPERIMENT_TAG]
    experiment=metadata.parse_experiment_plugin_data(content)
    assert 'result/cost_usd' in {m.name.tag for m in experiment.metric_infos}
    assert 'training/final_loss' in {m.name.tag for m in experiment.metric_infos}


def test_media_registry_ranges_and_changed_video(tmp_path):
    src=tmp_path/'source';src.mkdir();video=src/'execution.mp4';video.write_bytes(bytes(range(100)))
    st=video.stat();ident='a'*20
    put(tmp_path/'export','manifest.json',{'schema':'ugrp.tensorboard-export.v1','complete':True,
        'source':str(src),'videos':[{'id':ident,'path':str(video),'size':100,'mtime_ns':st.st_mtime_ns}]})
    server=make_server(tmp_path/'export',0);thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    url=f'http://127.0.0.1:{server.server_port}'
    try:
        with urlopen(Request(url+'/raw/'+ident,headers={'Range':'bytes=20-29'})) as r:
            assert r.status==206 and r.read()==bytes(range(20,30))
        with urlopen(url+'/video/'+ident) as r: assert '<video controls' in r.read().decode()
        for target,headers,code in [('/raw/'+ident,{'Host':'evil.invalid'},403),('/raw/../private',{},404)]:
            with pytest.raises(HTTPError) as e: urlopen(Request(url+target,headers=headers))
            assert e.value.code==code
        video.write_bytes(b'changed')
        with pytest.raises(HTTPError) as e: urlopen(url+'/raw/'+ident)
        assert e.value.code==409
    finally: server.shutdown();server.server_close();thread.join()


def test_unfinished_manifest_and_external_media_ignored(tmp_path):
    put(tmp_path,'manifest.json',{'schema':'ugrp.tensorboard-export.v1','complete':False,'source':str(tmp_path)})
    assert media_registry(tmp_path)=={}


def test_console_latency_and_session_completion_are_not_physical_success(tmp_path, export_api):
    convert, EA = export_api
    src = tmp_path / 'source'; src.mkdir()
    put(src, 'result.json', {'operator_session_complete': True, 'protocol_complete': False,
                           'model_latency_s': 2.5, 'scope': 'interactive_simulation'})
    convert(src, tmp_path / 'export')
    ea = EA(str(tmp_path / 'export')).Reload()
    assert ea.Scalars('claims/operator_session_complete')[0].value == 1
    assert ea.Scalars('claims/protocol_complete')[0].value == 0
    assert ea.Scalars('result/model_latency_s')[0].value == 2.5
    assert 'evaluation/reported_success' not in ea.Tags()['scalars']


def test_media_discovers_later_completed_export_without_restart(tmp_path):
    exports=tmp_path/'export';exports.mkdir()
    server=make_server(exports,0)
    thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    ident='b'*20;url=f'http://127.0.0.1:{server.server_port}/raw/{ident}'
    try:
        with pytest.raises(HTTPError) as error:urlopen(url)
        assert error.value.code==404
        source=tmp_path/'source';source.mkdir();video=source/'execution.mp4';video.write_bytes(b'new-video')
        st=video.stat()
        record={'schema':'ugrp.tensorboard-export.v1','complete':False,'source':str(source),
            'videos':[{'id':ident,'path':str(video),'size':st.st_size,'mtime_ns':st.st_mtime_ns}]}
        put(exports/'later','manifest.json',record)
        with pytest.raises(HTTPError) as error:urlopen(url)
        assert error.value.code==404
        record['complete']=True;put(exports/'later','manifest.json',record)
        with urlopen(url) as response:assert response.read()==b'new-video'
    finally:server.shutdown();server.server_close();thread.join()


def test_cloud_setup_failure_is_not_robot_failure(tmp_path, export_api):
    convert, EA = export_api
    src = tmp_path/'source'; src.mkdir()
    put(src, 'run.json', {'status':'failed', 'exit_code':1, 'started_at_unix':10., 'finished_at_unix':12., 'source_sha':'abc'})
    put(src, 'result/recovery-status.json', {'phase':'setup', 'error':'GPU absent'})
    manifest = convert(src, tmp_path/'export')
    events = EA(str(tmp_path/'export')).Reload()
    assert events.Scalars('process/exit_code')[0].value == 1
    assert events.Scalars('result/wall_s')[0].value == 2
    assert 'evaluation/reported_success' not in events.Tags()['scalars']
    assert manifest['metadata']['outcome'] == 'process_exit_1'
    assert 'result/recovery-status.json' in manifest['source_files']


def test_running_cloud_job_is_not_exported_as_complete(tmp_path, export_api):
    convert, _ = export_api
    src = tmp_path/'source'; src.mkdir()
    put(src, 'run.json', {'status':'running', 'started_at_unix':10.})
    with pytest.raises(ValueError, match='terminal process evidence'):
        convert(src, tmp_path/'export')
    assert not list((tmp_path/'export').glob('events.*'))


def test_gpu_probe_reports_devices_without_robot_success(tmp_path, export_api):
    convert, EA = export_api
    src = tmp_path/'source'; src.mkdir()
    put(src, 'gpu-inventory.json', {'torch':{'available':True, 'count':2}, 'internet_http_status':200})
    manifest = convert(src, tmp_path/'export')
    events = EA(str(tmp_path/'export')).Reload()
    assert events.Scalars('hardware/gpu_count')[0].value == 2
    assert events.Scalars('hardware/internet_http_status')[0].value == 200
    assert 'evaluation/reported_success' not in events.Tags()['scalars']
    assert manifest['metadata']['outcome'] == 'gpu_available'
