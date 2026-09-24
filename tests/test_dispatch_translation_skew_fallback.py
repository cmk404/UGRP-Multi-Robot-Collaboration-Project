"""Current-RGB corroboration gates for shaded carried-shaft centerlines."""

import numpy as np
import pytest

import harness.dispatch_translation_skew as skew


def _measure(monkeypatch, values):
    monkeypatch.setattr(skew, 'decode', lambda _: np.zeros((1, 1, 3), np.uint8))
    requested=[]

    def centerline(_frame, _hsv, _beam, saturation):
        requested.append(saturation)
        if saturation not in values:
            raise ValueError('insufficient pixels')
        return values[saturation], {
            'skew_px':values[saturation], 'min_saturation':saturation,
            'uses_issued_motion':False, 'supported_sections':50,
        }

    monkeypatch.setattr(skew, '_centerline', centerline)
    return requested


def test_shaded_shaft_uses_current_low_cut_consensus_only_with_high_cut_confirmation(monkeypatch):
    values={s:.02 for s in range(105, 131, 5)}
    values.update({150:-2., 155:.05, 160:.09, 165:2.8})
    requested=_measure(monkeypatch, values)
    value,evidence=skew.translation_skew(b'current-rgb', {})
    assert value == .02
    assert evidence['method']=='current RGB cross-section line with low/high contrast corroboration'
    assert evidence['consensus_saturations']==list(range(105, 131, 5))
    assert evidence['corroborating_high_saturations']==[155,160]
    assert evidence['uses_issued_motion'] is False
    assert requested==list(range(150,191,5))+list(range(105,131,5))


def test_shaded_shaft_rejects_low_cut_line_without_high_cut_confirmation(monkeypatch):
    values={s:0. for s in range(105, 131, 5)}
    values.update({150:3., 155:3.2, 160:3.4})
    _measure(monkeypatch, values)
    with pytest.raises(ValueError, match='lacks current RGB support across contrast cuts'):
        skew.translation_skew(b'current-rgb', {})


def test_shaded_shaft_keeps_four_cut_and_contrast_span_requirements(monkeypatch):
    values={105:0.,110:0.,115:0.,150:0.}
    _measure(monkeypatch, values)
    with pytest.raises(ValueError, match='lacks current RGB support across contrast cuts'):
        skew.translation_skew(b'current-rgb', {})


def test_valid_high_cut_measurement_does_not_enter_fallback(monkeypatch):
    values={150:.1,155:.2,160:.3,165:.4}
    requested=_measure(monkeypatch, values)
    value,evidence=skew.translation_skew(b'current-rgb', {})
    assert value==pytest.approx(.25)
    assert evidence['method']=='current RGB cross-section line with contrast consensus'
    assert evidence['consensus_saturations']==[150,155,160,165]
    assert requested==list(range(150,191,5))

