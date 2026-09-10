"""Fail-closed checks for the image-only jaw-sweep topology gate."""
import copy

import numpy as np

from harness.camera_sweep_trial import evaluate_sweep_topology


SIZE=[100,100]
TRANSITION={"from_pulse":1500,"to_pulse":2000,"image_size":SIZE}
BEAM={"image_size":SIZE,"corners4":[[.40,.47],[.60,.47],[.60,.53],[.40,.53]]}


def mask_pair():
    mask=np.zeros((100,100),dtype=bool)
    mask[40:61,35:48]=True
    mask[40:61,53:66]=True
    return mask


def candidate(mask=None,axis=(1.,0.),threshold=18):
    return {"threshold":threshold,"supporters":5,"axis":np.asarray(axis,dtype=float),
            "mask":mask_pair() if mask is None else mask}


def evaluate(candidates=(None,),transition=TRANSITION,old=BEAM,new=BEAM,endpoint=(.5,.5)):
    candidates=tuple(candidate() if item is None else item for item in candidates)
    return evaluate_sweep_topology(candidates,copy.deepcopy(transition),copy.deepcopy(old),
                                   copy.deepcopy(new),list(endpoint))


def test_valid_supported_candidate_has_two_external_sides_and_positive_overlap():
    result=evaluate()
    assert result["passed"] and result["candidate_count"]==1
    summary=result["candidates"][0]
    assert summary["left_pixels"]>0 and summary["right_pixels"]>0
    assert min(summary["bracket_margins_px"])>0
    assert summary["inside_distance_px"]>0
    assert summary["intersection_area_px2"]>0
    assert "mask" not in summary


def test_stale_or_wrong_direction_transition_fails_closed():
    assert not evaluate_sweep_topology((candidate(),),None,BEAM,BEAM,[.5,.5])["passed"]
    wrong={**TRANSITION,"from_pulse":2000,"to_pulse":1500}
    result=evaluate(transition=wrong)
    assert not result["passed"] and "1500-to-2000" in result["reason"]


def test_subtracting_both_beam_polygons_can_remove_one_apparent_side():
    mask=np.zeros((100,100),dtype=bool)
    mask[47:54,40:61]=True  # wholly explained by the old/current shaft polygons
    mask[40:61,65:68]=True
    result=evaluate((candidate(mask=mask),))
    assert not result["passed"]
    assert "beam subtraction removed one side" in result["candidates"][0]["reasons"]


def test_every_supported_candidate_must_agree():
    one_sided=np.zeros((100,100),dtype=bool);one_sided[40:61,53:66]=True
    result=evaluate((candidate(threshold=18),candidate(one_sided,threshold=19)))
    assert not result["passed"] and result["candidate_count"]==2
    assert result["candidates"][0]["passed"]
    assert not result["candidates"][1]["passed"]


def test_border_endpoint_and_border_beam_are_rejected():
    assert not evaluate(endpoint=(0.,.5))["passed"]
    border={**BEAM,"corners4":[[0.,.47],[.60,.47],[.60,.53],[0.,.53]]}
    assert not evaluate(old=border)["passed"]


def test_nonfinite_endpoint_or_candidate_axis_is_rejected():
    assert not evaluate(endpoint=(float("nan"),.5))["passed"]
    result=evaluate((candidate(axis=(float("inf"),0.)),))
    assert not result["passed"]
    assert "invalid supported candidate geometry" in result["candidates"][0]["reasons"]
