import xml.etree.ElementTree as ET

from sim.warehouse_arena import camera_team_arena_for_seed
from sim.warehouse_mission import add_warehouse_mission_xml


def _root():
    root = ET.Element("mujoco")
    ET.SubElement(root, "asset")
    ET.SubElement(root, "worldbody")
    ET.SubElement(root, "equality")
    return root


def _primary_geom(root, spec):
    return root.find(
        f"worldbody/body[@name='{spec.body_name}']/geom[@name='{spec.body_name}_geom']"
    )


def test_default_camera_team_boxes_have_real_physics_without_tag_assets_or_plates():
    specs = camera_team_arena_for_seed(46).cargo_specs
    root = _root()
    add_warehouse_mission_xml(root, specs=specs)

    for spec in specs:
        body = root.find(f"worldbody/body[@name='{spec.body_name}']")
        assert body is not None
        assert all("_tag_" not in geom.get("name", "") for geom in body.findall("geom"))
        geom = _primary_geom(root, spec)
        assert geom is not None
        assert geom.get("type") == "box"
        assert tuple(float(value) for value in geom.get("size").split()) == tuple(
            value / 2.0 for value in spec.dimensions_m
        )
        assert float(geom.get("mass")) == spec.mass_kg
        assert geom.get("friction") == "1.2 .20 .010"
        assert geom.get("contype") == "1"
        assert geom.get("conaffinity") == "3"

    asset_names = {node.get("name", "") for node in root.find("asset")}
    assert all(not any(spec.cargo_id in name for spec in specs) for name in asset_names)


def test_legacy_small_box_fiducials_are_explicit_opt_in_and_do_not_change_box_physics():
    specs = camera_team_arena_for_seed(46).cargo_specs
    markerless = _root()
    legacy = _root()
    add_warehouse_mission_xml(markerless, specs=specs)
    add_warehouse_mission_xml(legacy, specs=specs, include_small_box_fiducials=True)

    for spec in specs:
        plain_geom = _primary_geom(markerless, spec)
        tagged_geom = _primary_geom(legacy, spec)
        assert tagged_geom.attrib == plain_geom.attrib
        body = legacy.find(f"worldbody/body[@name='{spec.body_name}']")
        tags = [geom for geom in body.findall("geom") if "_tag_" in geom.get("name", "")]
        assert len(tags) == 2
        assert all(tag.get("mass") == "0" for tag in tags)
        assert all(tag.get("contype") == "0" and tag.get("conaffinity") == "0" for tag in tags)

    asset_names = {node.get("name", "") for node in legacy.find("asset")}
    for spec in specs:
        assert f"warehouse_tag_{spec.cargo_id}" in asset_names
        assert f"warehouse_tag_mat_{spec.cargo_id}" in asset_names
