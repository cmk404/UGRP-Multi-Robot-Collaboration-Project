import hashlib
import json
import math
from pathlib import Path

from harness.known_map_navigation import KnownMapNavigator
from sim.authored_navigation_map import load_map


def test_unchanged_real_start_images_localize_before_probing():
    root=Path(__file__).resolve().parents[1]
    fixture=root/'tests/fixtures/known_map'
    source=json.loads((fixture/'source.json').read_text())
    images={}
    for kind in ('own','top'):
        name=f'start-{kind}.jpg'; images[kind]=(fixture/name).read_bytes()
        assert hashlib.sha256(images[kind]).hexdigest()==source['files'][name]
    data=load_map(root/'maps/navigation/open.json')
    decision=KnownMapNavigator(data,'r1').decide(images['own'],images['top'],0)
    assert decision['status']=='calibrating_forward'
    assert decision['diagnostics']['localization']['ok']
    assert math.dist(decision['diagnostics']['position_estimate_m'], data['zones']['start']['center_m']) < data['zones']['start']['radius_m']
