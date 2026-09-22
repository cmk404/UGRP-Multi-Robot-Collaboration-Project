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
