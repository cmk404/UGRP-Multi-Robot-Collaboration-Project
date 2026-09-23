"""Shared research scene XML transforms; preserve robot bodies and cameras."""
import hashlib
import xml.etree.ElementTree as ET
from sim.authored_navigation_map import augment_map_xml

def sha(data):
    return hashlib.sha256(data).hexdigest()

def course_xml(xml: str, authored_map: dict):
    """Keep the exact robot bodies; replace legacy task props with this course."""
    root = ET.fromstring(xml)
    world = root.find('worldbody')
    robots = {node.get('name'): ET.tostring(node) for node in world.findall('body')
              if node.get('name', '').endswith('__robot')}
    if len(robots) != 3:
        raise ValueError('expected the three unchanged robot bodies')
    for node in list(world):
        if node.tag == 'geom' and node.get('name') != 'floor':
            world.remove(node)
        elif node.tag == 'body' and node.get('name') not in robots:
            # Schema placeholders remain for existing reset/controller APIs.
            # These legacy task props are not part of the unloaded map course.
            for body in node.iter('body'):
                body.set('gravcomp', '1')
            for geom in node.iter('geom'):
                geom.attrib.update(contype='0', conaffinity='0', rgba='0 0 0 0', group='5')
    xml = augment_map_xml(ET.tostring(root, encoding='unicode'), authored_map)
    final = ET.fromstring(xml).find('worldbody')
    after = {node.get('name'): ET.tostring(node) for node in final.findall('body')
             if node.get('name') in robots}
    if robots != after:
        raise ValueError('map construction changed robot body or camera geometry')
    return xml, {name: sha(data) for name, data in robots.items()}


def plain_beam_xml(xml):
    from sim.cooperative_payload import BEAM_BODY_NAME, BEAM_GEOM_NAME, BEAM_CARRIER_IDS, BEAM_WIDTH_M, BEAM_LENGTH_M, BEAM_HEIGHT_M, BEAM_MASS_KG
    PLAIN_BEAM_MASS_KG = BEAM_MASS_KG + .008 * len(BEAM_CARRIER_IDS)
    root = ET.fromstring(xml)
    beam = root.find(f".//body[@name='{BEAM_BODY_NAME}']")
    if beam is None:
        raise RuntimeError("beam body missing from generated model")
    geom = beam.find(f"geom[@name='{BEAM_GEOM_NAME}']")
    if geom is None:
        raise RuntimeError("main beam geom missing from generated model")
    geom.set(
        "size",
        f"{BEAM_WIDTH_M / 2:.6f} {BEAM_LENGTH_M / 2:.6f} {BEAM_HEIGHT_M / 2:.6f}",
    )
    # Preserve the prior fixture's complete 0.196 kg physical mass while
    # consolidating it into the one uniform box (0.180 kg bar + 2x0.008 kg).
    geom.set("mass", f"{PLAIN_BEAM_MASS_KG:.6f}")
    for rid in BEAM_CARRIER_IDS:
        anchor = beam.find(f"body[@name='{BEAM_BODY_NAME}_{rid}_endpoint']")
        if anchor is None:
            raise RuntimeError(f"{rid} weld coordinate anchor missing")
        for child in list(anchor):
            if child.tag in {"geom", "site"}:
                anchor.remove(child)
    return ET.tostring(root, encoding="unicode")
