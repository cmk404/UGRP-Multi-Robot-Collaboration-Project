import copy
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from sim.authored_navigation_map import augment_map_xml, load_map, map_sha256, validate_map
from sim.adaptive_warehouse import TerrainObservation, plan_local_path


ROOT = Path(__file__).resolve().parents[1]
MAPS = ROOT / "maps" / "navigation"


@pytest.mark.parametrize("name,count", [("open", 0), ("slalom", 2), ("narrow", 2)])
def test_supplied_maps_validate(name, count):
    authored = load_map(MAPS / f"{name}.json")
    assert authored["map_id"] == name
    assert len(authored["obstacles"]) == count
    assert len(map_sha256(authored)) == 64


def test_digest_is_canonical_and_deterministic():
    authored = load_map(MAPS / "slalom.json")
    reordered = {key: authored[key] for key in reversed(authored)}
    assert map_sha256(authored) == map_sha256(reordered)
    changed = copy.deepcopy(authored)
    changed["obstacles"][0]["height_m"] += 0.001
    assert map_sha256(authored) != map_sha256(changed)


@pytest.mark.parametrize("name,route_exists", [("open", True), ("slalom", True), ("narrow", False)])
def test_declared_footprint_has_expected_route_topology(name, route_exists):
    authored = load_map(MAPS / f"{name}.json")
    terrain = tuple(TerrainObservation(
        item["id"], item["kind"], tuple(item["center_m"]), tuple(item["half_extents_m"]),
        item["height_m"], item["traversable"], item["cost_multiplier"],
    ) for item in authored["obstacles"])
    clearance = (authored["footprint"]["unloaded_radius_m"]
                 + authored["footprint"]["safety_margin_m"])
    kwargs = {
        "footprint_xy": (clearance, clearance),
        "resolution_m": authored["grid_resolution_m"],
        "bounds": tuple(authored["bounds_m"]),
    }
    start = authored["zones"]["start"]["center_m"]
    goal = authored["zones"]["goal"]["center_m"]
    if route_exists:
        assert plan_local_path(start, goal, terrain, **kwargs)[-1] == tuple(goal)
    else:
        with pytest.raises(ValueError, match="no collision-free"):
            plan_local_path(start, goal, terrain, **kwargs)


def test_xml_obstacle_boxes_exactly_match_map_bounding_boxes():
    authored = load_map(MAPS / "slalom.json")
    result = ET.fromstring(augment_map_xml("<mujoco><worldbody/></mujoco>", authored))
    geoms = {geom.get("name"): geom for geom in result.find("worldbody").findall("geom")}
    for obstacle in authored["obstacles"]:
        geom = geoms[f"known_map_{obstacle['id']}"]
        assert geom.get("type") == "box"
        pos = [float(item) for item in geom.get("pos").split()]
        size = [float(item) for item in geom.get("size").split()]
        assert pos[:2] == obstacle["center_m"]
        assert size[:2] == obstacle["half_extents_m"]
        assert math.isclose(pos[2], obstacle["height_m"] / 2)
        assert math.isclose(size[2], obstacle["height_m"] / 2)
        assert geom.get("contype") == "1"
    assert geoms["known_map_zone_start"].get("contype") == "0"
    assert geoms["known_map_zone_goal"].get("conaffinity") == "0"
    assert not result.findall(".//camera")


def test_augmentation_preserves_existing_camera_and_robot():
    source = '<mujoco><worldbody><camera name="cctv_top"/><body name="robot"/></worldbody></mujoco>'
    result = ET.fromstring(augment_map_xml(source, load_map(MAPS / "open.json")))
    assert result.find(".//camera").get("name") == "cctv_top"
    assert result.find(".//body").get("name") == "robot"


@pytest.mark.parametrize("mutation", [
    lambda value: value.update(schema="wrong"),
    lambda value: value.update(bounds_m=[0, 0, -3.1, -0.9]),
    lambda value: value.update(grid_resolution_m=0),
    lambda value: value.update(version=True),
    lambda value: value["top_camera"].update(quaternion_wxyz=[0, 0, 0, 0]),
    lambda value: value["footprint"].update(safety_margin_m=-0.1),
    lambda value: value["zones"]["goal"].update(center_m=[99, 99]),
])
def test_malformed_map_is_rejected(mutation):
    authored = load_map(MAPS / "open.json")
    mutation(authored)
    with pytest.raises(ValueError):
        validate_map(authored)


def test_duplicate_obstacle_ids_and_out_of_bounds_obstacles_are_rejected():
    authored = load_map(MAPS / "slalom.json")
    authored["obstacles"][1]["id"] = authored["obstacles"][0]["id"]
    with pytest.raises(ValueError, match="duplicate obstacle"):
        validate_map(authored)
    authored = load_map(MAPS / "slalom.json")
    authored["obstacles"][0]["center_m"] = [-0.84, -2.55]
    with pytest.raises(ValueError, match="outside bounds"):
        validate_map(authored)
    authored = load_map(MAPS / "slalom.json")
    authored["obstacles"][0]["center_m"] = authored["zones"]["start"]["center_m"]
    with pytest.raises(ValueError, match="overlaps zones.start"):
        validate_map(authored)


def test_nonfinite_values_and_dynamic_pose_fields_are_rejected():
    authored = load_map(MAPS / "open.json")
    authored["grid_resolution_m"] = math.nan
    with pytest.raises(ValueError, match="finite"):
        validate_map(authored)
    authored = load_map(MAPS / "open.json")
    authored["robot_pose"] = [0, 0, 0]
    with pytest.raises(ValueError, match="invalid fields"):
        validate_map(authored)


def test_duplicate_json_keys_are_rejected(tmp_path):
    path = tmp_path / "duplicate.json"
    path.write_text('{"schema":"a","schema":"b"}', encoding="utf-8")
    with pytest.raises(ValueError, match="duplicate JSON key"):
        load_map(path)


def test_xml_name_collision_is_rejected():
    authored = load_map(MAPS / "slalom.json")
    xml = '<mujoco><worldbody><geom name="known_map_wall1"/></worldbody></mujoco>'
    with pytest.raises(ValueError, match="already contains"):
        augment_map_xml(xml, authored)
