import json
import pytest
from scripts.assess_research_cohort import assess
from scripts.carry_failure_metrics import outcome


def fixture(tmp_path):
    root=tmp_path/'trial';root.mkdir()
    result={'source_sha':'frozen','physical_success':True,'protocol_complete':True,'error':None,
            'phase':'FINISHED','obstacle_contact_steps':0,'wall_s':10.,
            'evaluation':{'weld_steps':0,'robot_robot_contact_samples':0,'cargo':{'beam':{'physical_success':True}}}}
    (root/'result.json').write_text(json.dumps(result))
    (root/'pair-decisions.json').write_text('[{"kind":"carry"}]')
    for name in ['scene.xml','issued-commands.json','evaluation-only.json','execution.mp4']:
        (root/name).write_text('fixture')
    (root/'issued-commands.json').write_text('{}')
    (root/'evaluation-only.json').write_text('{}')
    job={'trial_id':'RGB--case--r0','case':{'id':'case'},'condition':'RGB'}
    row={**job,'output':str(root),'exit_code':0,'timed_out':False,'outcome':outcome(root,0)}
    report=tmp_path/'report.json';report.write_text(json.dumps({'source_sha':'frozen','protocol_sha256':'protocol','jobs':[job],'runs':[row]}))
    return root,report


def test_only_complete_physical_evidence_qualifies(tmp_path):
    root,report=fixture(tmp_path)
    assert assess(report)['conditions']['RGB']['qualified_for_declared_cases']
    (root/'execution.mp4').unlink()
    assert not assess(report)['conditions']['RGB']['qualified_for_declared_cases']


def test_changed_result_or_missing_collision_measurement_cannot_pass(tmp_path):
    root,report=fixture(tmp_path)
    p=root/'result.json';r=json.loads(p.read_text());r['evaluation'].pop('robot_robot_contact_samples');p.write_text(json.dumps(r))
    assert not assess(report)['conditions']['RGB']['qualified_for_declared_cases']


def test_unattempted_trial_is_not_admitted_or_counted_as_failure(tmp_path):
    _,report=fixture(tmp_path);r=json.loads(report.read_text());r['runs']=[];report.write_text(json.dumps(r))
    s=assess(report);assert not s['complete']
    c=s['conditions']['RGB'];assert c['failures']==0 and c['pending']==1
    assert not c['qualified_for_declared_cases']


@pytest.mark.parametrize('artifact',['result.json','issued-commands.json','pair-decisions.json','evaluation-only.json'])
def test_missing_or_invalid_raw_json_is_reported_without_aborting_assessment(tmp_path,artifact):
    root,report=fixture(tmp_path)
    (root/artifact).unlink()
    assert not assess(report)['conditions']['RGB']['qualified_for_declared_cases']
    (root/artifact).write_text('interrupted JSON')
    assert not assess(report)['conditions']['RGB']['qualified_for_declared_cases']
