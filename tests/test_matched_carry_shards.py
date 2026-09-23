import json
import pytest
from scripts.carry_failure_metrics import schedule
from scripts.combine_matched_carry_shards import combine


def fixture(tmp_path):
    p={'expected_source_sha':'frozen','test':[{'id':'a'},{'id':'b'}],'controls':{},'grasp':'g','stages':'s',
       'asset_sha256':{'model':'hash'},'conditions':{'RGB':{},'ACT':{'model':'m'},'hybrid':{'model':'m','stop_mode':'rgb_refined'}}}
    protocol=tmp_path/'full.json';protocol.write_text(json.dumps(p));reports=[]
    for n,names in enumerate((['RGB','ACT'],['hybrid'])):
        sub={**p,'conditions':{k:p['conditions'][k] for k in names}};jobs=schedule(sub,names)
        verdict={'whole_success':True,'failure_kind':None,'robot_result_available':True,'carry_entered':True,
                 'act_carry_entered':False,'beam_success':True}
        r={'source_sha':'frozen','protocol_sha256':str(n),'protocol':sub,'complete':True,'jobs':jobs,
           'runs':[{**j,'outcome':verdict} for j in jobs]}
        path=tmp_path/f'{n}.json';path.write_text(json.dumps(r));reports.append(path)
    return protocol,reports


def test_complete_disjoint_shards_match_one_declared_suite(tmp_path):
    protocol,reports=fixture(tmp_path);r=combine(protocol,reports)
    assert r['complete'] and len(r['runs'])==6 and len(r['shards'])==2
    assert all(c['attempted']==2 for c in r['failure_estimates']['conditions'].values())
    with pytest.raises(ValueError,match='missing planned'):combine(protocol,reports[:1])
    with pytest.raises(ValueError,match='duplicate'):combine(protocol,reports+reports[:1])


@pytest.mark.parametrize('change',['source','model','test','missing_run'])
def test_incompatible_or_incomplete_shard_cannot_be_combined(tmp_path,change):
    protocol,reports=fixture(tmp_path);p=reports[0];r=json.loads(p.read_text())
    if change=='source':r['source_sha']='other'
    elif change=='model':r['protocol']['asset_sha256']['model']='changed'
    elif change=='test':r['protocol']['test'][0]['id']='exposed'
    else:r['runs'].pop()
    p.write_text(json.dumps(r))
    with pytest.raises(ValueError):combine(protocol,reports)


def test_trial_partition_preserves_complete_plan_and_order(tmp_path):
    protocol,reports=fixture(tmp_path);p=json.loads(protocol.read_text())
    jobs=schedule(p,list(p['conditions']))
    verdict=json.loads(reports[0].read_text())['runs'][0]['outcome']
    for n,path in enumerate(reports):
        ids=[j['trial_id'] for j in jobs[n::2]]
        sub={**p,'trial_ids':ids};part=schedule(sub,list(sub['conditions']))
        path.write_text(json.dumps({'source_sha':'frozen','protocol_sha256':str(n),
            'protocol':sub,'complete':True,'jobs':part,'runs':[{**j,'outcome':verdict} for j in part]}))
    result=combine(protocol,reports)
    assert [r['trial_id'] for r in result['runs']]==[j['trial_id'] for j in jobs]
    assert result['complete']


@pytest.mark.parametrize('ids',[[],['missing'],['RGB--a--r0','RGB--a--r0'],[3],'RGB--a--r0'])
def test_invalid_trial_partition_is_rejected(tmp_path,ids):
    protocol,_=fixture(tmp_path);p=json.loads(protocol.read_text());p['trial_ids']=ids
    with pytest.raises(ValueError,match='trial_ids'):schedule(p,list(p['conditions']))


def test_explicit_pending_handoff_preserves_all_attempts_and_requires_full_coverage(tmp_path):
    protocol,reports=fixture(tmp_path);path=reports[0];r=json.loads(path.read_text())
    transferred=r['runs'].pop();r['complete']=False;path.write_text(json.dumps(r))
    full=json.loads(protocol.read_text());sub={**full,'trial_ids':[transferred['trial_id']]}
    extra=tmp_path/'transferred.json';extra.write_text(json.dumps({'source_sha':'frozen',
        'protocol_sha256':'extra','protocol':sub,'complete':True,
        'jobs':schedule(sub,list(sub['conditions'])),'runs':[transferred]}))
    with pytest.raises(ValueError,match='incomplete'):combine(protocol,reports+[extra])
    with pytest.raises(ValueError,match='missing planned'):combine(protocol,reports,allow_partial_shards=True)
    result=combine(protocol,reports+[extra],allow_partial_shards=True)
    assert result['complete'] and len(result['runs'])==6
    assert result['shards'][0]['pending_trial_ids']==[transferred['trial_id']]
    with pytest.raises(ValueError,match='duplicate'):combine(protocol,reports+[extra,extra],allow_partial_shards=True)
