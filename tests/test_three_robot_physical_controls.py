from pathlib import Path
import base64
import copy
from unittest import mock
import cv2
import numpy as np
import pytest
from harness.camera_recovery_student import fit_recovery_model, predict_recovery
from tests.test_camera_recovery_student import _views, _training
from harness.visual_box_skill import VisualBoxSkill
from tests.test_visual_box_skill import observation, _Tracker, cyan_jpeg
from harness.solo_box_transport import SoloBoxTransport


def test_background_transfer_preserves_own_camera_novelty_and_learned_coordinates():
    model=fit_recovery_model(*_views(),_training())
    scoped=copy.deepcopy(model);scoped['constant_background_top_band']=[6,19]
    own,top=_views((2,-2,2))
    before=predict_recovery(model,own,top)
    img=cv2.imdecode(np.frombuffer(top,np.uint8),cv2.IMREAD_COLOR)
    img[170:,:]=255
    changed=cv2.imencode('.png',img)[1].tobytes()
    after=predict_recovery(scoped,own,changed)
    assert after['observable']
    assert after['delta_pulses']==before['delta_pulses']
    novel=cv2.imencode('.png',np.full((144,192,3),255,np.uint8))[1].tobytes()
    assert not predict_recovery(scoped,novel,changed)['observable']
    # A varying feature outside the allowed region must never be masked.
    key='components' if 'components' in scoped else 'pca_components'
    scoped[key][0][-1]=.01
    with pytest.raises(ValueError,match='learned visual direction'):
        predict_recovery(scoped,own,changed)


def test_sequential_probe_rejects_saved_detached_box():
    skill=VisualBoxSkill(perception_mode='fiducial',attachment_home_reference='previous_endpoint')
    skill.phase='verify_lift';skill.tracker=_Tracker({'visible':False})
    pose={'1':1500,'3':757,'4':1746,'5':2221,'6':1500}
    root=Path('tests/fixtures/visual_attachment/detached')
    for i,(name,pan) in enumerate((('anchor',1500),('left',1560),('right',1440),('home',1500))):
        result=skill.decide(observation(i+1,i,{**pose,'6':pan},(root/(name+'.jpg')).read_bytes()))
    assert result=={'kind':'finish','reason':'VISUAL_ATTACHMENT_UNCONFIRMED'}
    assert not skill.held


def test_sequential_probe_allows_bounded_compliant_shift_but_not_failed_side():
    pose={'1':1500,'3':757,'4':1746,'5':2221,'6':1500}
    for shifts,passed in (((0,3,6,9),True),((0,35,6,9),False)):
        skill=VisualBoxSkill(perception_mode='fiducial',attachment_home_reference='previous_endpoint')
        skill.phase='verify_lift';skill.tracker=_Tracker({'visible':False})
        for i,(shift,pan) in enumerate(zip(shifts,(1500,1560,1440,1500))):
            result=skill.decide(observation(i+1,i,{**pose,'6':pan},cyan_jpeg(x=220+shift)))
        assert skill.held is passed
        assert (result['kind']=='finish') is not passed


def test_solo_uses_selected_region_and_requires_full_width_twice():
    skill=SoloBoxTransport('far_magenta');skill.initialized=True;skill.box.phase='carry'
    skill.target={'center':[.7,.9],'bounds':[.6,.85,.2,.1]}
    def features(x,w=.03):
        return {'zones':[], 'boxes':[{'center':[x+w/2,.9],'bounds':[x,.88,w,.02]}]}
    with mock.patch.object(skill.box,'decide',return_value={'kind':'wait','duration':.1}):
        with mock.patch('harness.solo_box_transport.solo_top_features',return_value=features(.59)):
            action,_=skill.decide({},b'');assert action['kind']=='drive';assert action['fwd']>0
        with mock.patch('harness.solo_box_transport.solo_top_features',return_value=features(.62)):
            action,_=skill.decide({},b'');assert skill.phase=='carry';assert action['fwd']==0
            action,_=skill.decide({},b'');assert skill.phase=='release'
