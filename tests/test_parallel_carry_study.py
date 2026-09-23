import copy
import pytest
from scripts.run_parallel_carry_study import validate_split_cases


def test_renaming_a_teacher_case_does_not_make_it_held_out():
    case={'variant':'open','seed':11,'dock':'dock_a','offset':[0,0,0]}
    p={'teacher_cases':[{**case,'id':'teacher'}],
       'reuse_parallel_case':{**case,'offset':[-.01,.004,0]},
       'test_cases':[{**case,'id':'test','offset':[.015,-.008,0]}]}
    validate_split_cases(p)
    for duplicate in [p['teacher_cases'][0],p['reuse_parallel_case']]:
        broken=copy.deepcopy(p);broken['test_cases']=[{**duplicate,'id':'new-name'}]
        with pytest.raises(ValueError,match='overlap'):validate_split_cases(broken)
