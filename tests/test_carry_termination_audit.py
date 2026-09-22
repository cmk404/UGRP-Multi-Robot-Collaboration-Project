import pytest
from scripts.audit_carry_termination import audit


def rows():
    return [{'id':f'episode:{s}:{i}','target':[0,0,0,float(i==3)],
             'prediction':[0,0,0,float(i==3)]} for s in ['r1','r3'] for i in range(4)]


def test_single_false_done_is_episode_failure_even_when_partner_says_continue():
    r=rows();r[1]['prediction'][3]=.7
    result=audit(r)
    assert result['premature_pair_hold_episodes']==1
    assert not result['offline_termination_pass']
    assert result['results'][0]['robots']['r1']['first_stop_remaining_commands']==2


def test_offline_pass_is_not_a_robot_success_or_promotion():
    result=audit(rows())
    assert result['offline_termination_pass'] and not result['eligible_for_default']
    with pytest.raises(ValueError,match='both model slots'):audit(rows()[:4])


def test_near_terminal_negatives_remain_negative_and_receive_their_own_group():
    from scripts.carry_training_objective import sampling_groups,selection
    data=[{'id':f'e:{s}:{i}','done':i==11,'action':[float(i<11),0,0,float(i==11)]}
          for s in ['r1','r3'] for i in range(12)]
    groups=sampling_groups(data)
    assert groups[:3]==[0]*3 and groups[3:11]==[4]*8 and groups[11]==3
    assert sampling_groups(data,'legacy')[:11]==[0]*11
    pred=[r['action'][:] for r in data];pred[10][3]=.7
    metric=selection(data,pred,.02,.03)
    assert metric['selection_score']==1.02 and not metric['offline_termination_pass']
