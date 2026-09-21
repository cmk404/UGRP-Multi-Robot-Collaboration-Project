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


def test_missing_success_is_not_zero(tmp_path,export_api):
    convert,EA=export_api;src=tmp_path/'source';src.mkdir()
    put(src,'result.json',{'protocol_complete':True})
    convert(src,tmp_path/'export')
    assert 'evaluation/reported_success' not in EA(str(tmp_path/'export')).Reload().Tags()['scalars']


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
