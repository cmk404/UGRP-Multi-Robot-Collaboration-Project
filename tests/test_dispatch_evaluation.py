from harness.dispatch_evaluation import carry_clearance


def test_placement_does_not_hide_a_drop_and_release_is_excluded():
    plan = {'tasks': [{'object': 'beam', 'participants': ['r1', 'r3']}]}
    history = {'r1': [{'stage': 'TRANSIT', 'issued_at_s': 1.,
                      'action': {'duration_s': .4}}]}
    def sample(t, contact):
        return {'sim_time_s': t, 'cargo': {'beam': {'floor_contact': contact}}}
    rows = [sample(0., True)] + [sample(t, False) for t in [1., 1.1, 1.2, 1.3, 1.4]]
    rows += [sample(1.5, True)]  # Commanded lowering after transport is allowed.
    assert carry_clearance(rows, history, plan)['beam']['sampled_continuous_clearance']
    rows[3]['cargo']['beam']['floor_contact'] = True
    result = carry_clearance(rows, history, plan)['beam']
    assert not result['sampled_continuous_clearance']
    assert result['first_floor_contact_s'] == 1.2


def test_missing_transport_or_sparse_evaluation_cannot_pass():
    plan = {'tasks': [{'object': 'beam', 'participants': ['r1', 'r3']}]}
    assert not carry_clearance([], {}, plan)['beam']['sampled_continuous_clearance']
    history = {'r3': [{'stage': 'TRANSIT', 'issued_at_s': 1., 'duration_s': 1.}]}
    rows = [{'sim_time_s': t, 'cargo': {'beam': {'floor_contact': False}}}
            for t in [1., 2.]]
    assert not carry_clearance(rows, history, plan)['beam']['sampled_continuous_clearance']
