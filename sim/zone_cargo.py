"""Zone cargo catalogue: solo, pair and trio items as reusable MuJoCo bodies.

2026-09-25 user request: more diverse cargo so that role allocation and teaming
matter. Items one MasterPi carries alone, items that need exactly two (a long
beam, a heavy crate) and one that needs three (a triangular frame).

How many robots an item needs is a *physical* property of the SIM robot, not a
rule: the pair/trio masses and lengths are set from the robot model's measured
single-gripper payload capacity and from its gripper geometry (see
``experiments/2026-09-25-zone-cargo-catalogue``). ``required_carriers`` below is
the catalogue claim; the probe results are the evidence. Masses are SIM stress
values relative to the SIM robot, not claims about the real platform.

Static specification (this module's ``CATALOGUE``) is kept apart from runtime
instances (``CargoInstance``: body name, joint name, pose). Every handle is
built from the production box cross-section (40 mm grip width, 32 mm tall,
same friction and finger contact pairs), so the calibrated box grasp (IK at
0.155 m radius, 24 mm grasp height, gripper pulse 1500) applies unchanged.
No weld/equality constraint is ever added for cargo.
"""
from __future__ import annotations

import hashlib
import json
import math
import xml.etree.ElementTree as ET
from dataclasses import asdict, dataclass, field

SCHEMA = 'ugrp.zone_cargo.v1'
CATALOGUE_VERSION = 1
# Calibrated box grasp (scripts/zone_teacher.py): grip point this far ahead of
# the robot base, gripper facing it, at this height above the floor.
GRASP_RADIUS_M = .155
GRASP_Z_M = .024
# Production box material; all cargo geoms reuse it so the local_contact_fine
# finger/box pairs can be mirrored unchanged (same friction, solref, solimp).
BOX_FRICTION = (1.2, .2, .01)
# Box cross-section used for every handle: grip width (across the jaws) and
# height. Jaws open to 61 mm and close to 29 mm; 40 mm gives 6.6 N squeeze per
# jaw at the closed pulse (1200 N/m x 5.5 mm), identical to the box.
HANDLE_WIDTH_M, HANDLE_HEIGHT_M = .040, .032
HANDLE_RGBA = '.06 .06 .06 1'
# Measured with scripts/probe_zone_cargo.py --sweep cal_block (GT teacher, weld
# OFF, local_contact_fine): one MasterPi lifts, holds 8 s, carries 0.5 m and
# places a box-shaped block of 0.66 kg; at 0.68 kg it tips forward about its
# front axle (38-41 deg) before the block leaves the floor. No arm servo
# saturates first, and finger friction (mu 3.4, 6.6 N squeeze) is not the limit.
MEASURED_SINGLE_ROBOT_CAPACITY_KG = (.66, .68)
# Pair/trio lugs are taller than the box (same width, same grasp height): under
# the local_contact_fine profile a held load creeps down through the jaws at
# about 0.9 mm/s per kg (measured), so heavy items get more vertical margin.
# 56 mm lugs touched the wrist collision geom at the grasp pose; 46 mm do not.
LUG_HEIGHT_M = .046
# TOP cameras: 2.5 m high, 55 deg vertical FOV, 960x720 -> pixels per metre on the floor.
TOP_PX_PER_M = 360/math.tan(math.radians(27.5))/2.5


@dataclass(frozen=True)
class Part:
    """One geom in the object frame (origin on the floor under the centroid)."""
    name: str
    shape: str                      # 'box' | 'cylinder'
    center: tuple                   # (x, y, z) metres
    size: tuple                     # box half extents / (radius, half height)
    yaw: float = 0.
    rgba: str = ''
    collision: bool = True
    role: str = 'body'              # 'body' | 'handle' | 'marker'

    def volume(self):
        if not self.collision:
            return 0.
        if self.shape == 'box':
            return 8*self.size[0]*self.size[1]*self.size[2]
        return math.pi*self.size[0]**2*2*self.size[1]


@dataclass(frozen=True)
class Grasp:
    """A grasp role: grip point and the heading the robot faces to take it."""
    role: str
    grip_xyz: tuple                 # object frame, metres
    approach_yaw: float             # object frame; the jaws close across this heading
    geom: str                       # part name the jaws close on

    def approach_base(self):
        """Robot base pose (x, y, yaw) in the object frame for this grasp."""
        x, y, _ = self.grip_xyz
        return (round(x-GRASP_RADIUS_M*math.cos(self.approach_yaw), 6),
                round(y-GRASP_RADIUS_M*math.sin(self.approach_yaw), 6), self.approach_yaw)


@dataclass(frozen=True)
class CargoKind:
    kind: str
    label: str
    tier: str                       # 'solo' | 'pair' | 'trio'
    required_carriers: int
    mass_kg: float
    colour: str
    rgba: str
    parts: tuple
    grasps: tuple
    formations: tuple               # allowed role sets
    landing_half_extents_m: tuple   # object-frame rectangle that must lie in the target area
    landing_yaw_symmetry_deg: float  # yaw equivalence (0: any yaw)
    physics_basis: str
    shape: str = ''                 # TOP-RGB identification cue (not colour alone)
    markings: str = ''
    notes: str = ''
    extra: dict = field(default_factory=dict)


def _handle(name, x, y, yaw, length=.06, height=LUG_HEIGHT_M, rgba=HANDLE_RGBA):
    """A lug with the box grip width, its long axis along ``yaw``, standing on the floor."""
    return Part(name, 'box', (x, y, height/2), (length/2, HANDLE_WIDTH_M/2, height/2),
                yaw=yaw, rgba=rgba, role='handle')


def _band(name, x, y, yaw, half_len, half_w, top):
    """Massless, non-colliding paint band marking a grasp point from above."""
    return Part(name, 'box', (x, y, top+.0006), (half_len, half_w, .0006), yaw=yaw,
                rgba=HANDLE_RGBA, collision=False, role='marker')


def _build():
    kinds = {}
    # --- solo ---------------------------------------------------------------
    # can: the jaws close across its 38 mm diameter from any heading. 50 mm tall:
    # a 70 mm can touched the wrist collision geom at the grasp pose (dev run).
    kinds['can'] = CargoKind(
        'can', 'metal can', 'solo', 1, .080, 'violet', '.52 .22 .92 1',
        parts=(Part('shell', 'cylinder', (0., 0., .025), (.019, .025), rgba='.52 .22 .92 1'),),
        grasps=(Grasp('any', (0., 0., GRASP_Z_M), 0., 'shell'),),
        formations=(('any',),), landing_half_extents_m=(.019, .019), landing_yaw_symmetry_deg=0.,
        physics_basis='80 g, well inside one gripper payload; round, so any approach heading works',
        shape='small disc from TOP (38 mm)', markings='none')
    # tile: 12 mm tall; grasped at 7 mm with the same IK (reach verified 6-14 mm).
    kinds['tile'] = CargoKind(
        'tile', 'low tile', 'solo', 1, .025, 'magenta', '.86 .16 .58 1',
        parts=(Part('slab', 'box', (0., 0., .006), (.030, .020, .006), rgba='.86 .16 .58 1'),),
        grasps=(Grasp('west', (0., 0., .007), 0., 'slab'), Grasp('east', (0., 0., .007), math.pi, 'slab')),
        formations=(('west',), ('east',)), landing_half_extents_m=(.030, .020), landing_yaw_symmetry_deg=180.,
        physics_basis='25 g, low (12 mm): grasped 7 mm above the floor',
        shape='small 60x40 mm rectangle, flat', markings='none')
    # --- pair ---------------------------------------------------------------
    # long beam: the jaws always close perpendicular to the reach direction and
    # the arm has no wrist roll, so a beam can only be gripped with the robot on
    # its long axis, i.e. within ~0.09 m of an end. One robot then holds a
    # 0.60 m lever: it can only tip it up with the far end on the floor.
    beam_len = .60
    grip_x = beam_len/2 - .03
    kinds['long_beam'] = CargoKind(
        'long_beam', 'long beam', 'pair', 2, .300, 'lime', '.55 .78 .12 1',
        parts=(Part('bar', 'box', (0., 0., HANDLE_HEIGHT_M/2), (beam_len/2, HANDLE_WIDTH_M/2, HANDLE_HEIGHT_M/2),
                    rgba='.55 .78 .12 1', role='handle'),
               _band('band_neg', -grip_x, 0., 0., .018, .0205, HANDLE_HEIGHT_M),
               _band('band_pos', grip_x, 0., 0., .018, .0205, HANDLE_HEIGHT_M)),
        grasps=(Grasp('end_neg', (-grip_x, 0., GRASP_Z_M), 0., 'bar'),
                Grasp('end_pos', (grip_x, 0., GRASP_Z_M), math.pi, 'bar')),
        formations=(('end_neg', 'end_pos'),), landing_half_extents_m=(beam_len/2, HANDLE_WIDTH_M/2),
        landing_yaw_symmetry_deg=180.,
        physics_basis=('0.60 m: grasp only from the long axis (jaws close across the reach, no wrist roll) '
                       'within 0.09 m of an end; single-end lift tips the beam about the far end'),
        shape='long bar, aspect 15:1 (600x40 mm)', markings='black grip bands 36 mm wide at both ends')
    # heavy crate: 100 mm wide body (wider than the 61 mm jaw opening), so it is
    # only gripped at its two lugs; mass above one robot's measured capacity.
    kinds['heavy_crate'] = CargoKind(
        'heavy_crate', 'heavy crate', 'pair', 2, .900, 'pink', '1.0 .58 .72 1',
        parts=(Part('box', 'box', (0., 0., .03), (.07, .05, .03), rgba='1.0 .58 .72 1'),
               _handle('lug_west', -.09, 0., 0.), _handle('lug_east', .09, 0., 0.)),
        grasps=(Grasp('west', (-.10, 0., GRASP_Z_M), 0., 'lug_west'),
                Grasp('east', (.10, 0., GRASP_Z_M), math.pi, 'lug_east')),
        formations=(('west', 'east'),), landing_half_extents_m=(.12, .05), landing_yaw_symmetry_deg=180.,
        physics_basis='mass above one robot measured lift capacity; body wider than the jaw opening',
        shape='rectangle 140x100 mm with two black end lugs (240x100 mm overall, aspect 2.4)',
        markings='black lugs at both short ends')
    # --- trio ---------------------------------------------------------------
    # triangular frame: three edge bars, a lug at each vertex pointing out.
    radius = .20
    verts = [(radius*math.cos(a), radius*math.sin(a), a) for a in (0., 2*math.pi/3, 4*math.pi/3)]
    parts = []
    for i in range(3):
        (x0, y0, _), (x1, y1, _) = verts[i], verts[(i+1) % 3]
        length = math.hypot(x1-x0, y1-y0)
        parts.append(Part(f'edge{i}', 'box', ((x0+x1)/2, (y0+y1)/2, .012), (length/2+.012, .015, .012),
                          yaw=math.atan2(y1-y0, x1-x0), rgba='.95 .88 .66 1'))
    grasps = []
    for i, (x, y, a) in enumerate(verts):
        c = (radius+.02)/radius
        parts.append(_handle(f'lug{i}', x*c, y*c, a))
        g = (radius+.03)/radius
        grasps.append(Grasp(f'v{i}', (round(x*g, 6), round(y*g, 6), GRASP_Z_M), a+math.pi, f'lug{i}'))
    kinds['tri_frame'] = CargoKind(
        'tri_frame', 'triangular frame', 'trio', 3, 1.500, 'cream', '.95 .88 .66 1',
        parts=tuple(parts), grasps=tuple(grasps), formations=(('v0', 'v1', 'v2'),),
        landing_half_extents_m=(.27, .27), landing_yaw_symmetry_deg=120.,
        physics_basis=('three vertex lugs; mass above what two robots lift clear, and a two-lug hold '
                       'leaves the third vertex on the floor'),
        shape='open triangle outline (side 0.35 m) with a black lug at each vertex',
        markings='black vertex lugs', extra={'vertex_radius_m': radius})
    return kinds


CATALOGUE = _build()
# Probe-only: the production box shape at any mass, to measure one robot's
# payload capacity with the calibrated grasp. Never part of a zone episode.
CALIBRATION = {'cal_block': CargoKind(
    'cal_block', 'calibration block (box shape)', 'calibration', 1, .030, 'grey', '.50 .50 .55 1',
    parts=(Part('block', 'box', (0., 0., .016), (.017, .020, .016), rgba='.50 .50 .55 1', role='handle'),),
    grasps=(Grasp('west', (0., 0., GRASP_Z_M), 0., 'block'),), formations=(('west',),),
    landing_half_extents_m=(.017, .020), landing_yaw_symmetry_deg=180.,
    physics_basis='probe-only payload calibration')}
# The production box stays the zone box (sim.zone_arena); listed here for the table.
EXISTING_SOLO = {'box': {'label': 'dispatch box (existing, 4 paints)', 'mass_kg': .030,
                         'dims_m': [.034, .040, .032], 'required_carriers': 1, 'source': 'sim.zone_arena'}}


def kind(name):
    if name in CATALOGUE:
        return CATALOGUE[name]
    if name in CALIBRATION:
        return CALIBRATION[name]
    raise ValueError(f'unknown cargo kind: {name}')


def with_mass(spec, mass_kg):
    """A copy of a kind with another total mass (calibration sweeps only)."""
    from dataclasses import replace
    return replace(spec, mass_kg=float(mass_kg))


# --------------------------------------------------------------------------
# Derived static properties

def part_masses(spec):
    vols = [p.volume() for p in spec.parts]
    total = sum(vols)
    return [spec.mass_kg*v/total for v in vols]


def _part_inertia(part, mass):
    if part.shape == 'box':
        a, b, c = (2*s for s in part.size)
        local = [mass*(b*b+c*c)/12, mass*(a*a+c*c)/12, mass*(a*a+b*b)/12]
    else:
        r, h = part.size[0], 2*part.size[1]
        local = [mass*(3*r*r+h*h)/12, mass*(3*r*r+h*h)/12, mass*r*r/2]
    c, s = math.cos(part.yaw), math.sin(part.yaw)
    rot = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    return [[sum(rot[i][k]*local[k]*rot[j][k] for k in range(3)) for j in range(3)] for i in range(3)]


def mass_properties(spec):
    """Centre of mass and inertia about it (object frame), from the collision parts."""
    masses = part_masses(spec)
    total = sum(masses)
    com = [sum(m*p.center[i] for m, p in zip(masses, spec.parts))/total for i in range(3)]
    inertia = [[0.]*3 for _ in range(3)]
    for m, p in zip(masses, spec.parts):
        if not m:
            continue
        own = _part_inertia(p, m)
        d = [p.center[i]-com[i] for i in range(3)]
        dd = sum(v*v for v in d)
        for i in range(3):
            for j in range(3):
                inertia[i][j] += own[i][j] + m*((dd if i == j else 0.) - d[i]*d[j])
    return {'mass_kg': round(total, 6), 'com_m': [round(v, 6) for v in com],
            'inertia_kgm2': [[round(v, 9) for v in row] for row in inertia]}


def bounding_box(spec):
    """Axis-aligned object-frame extents of the collision parts (x, y, z)."""
    lo, hi = [math.inf]*3, [-math.inf]*3
    for p in spec.parts:
        if not p.collision:
            continue
        if p.shape == 'box':
            c, s = abs(math.cos(p.yaw)), abs(math.sin(p.yaw))
            hx, hy = c*p.size[0]+s*p.size[1], s*p.size[0]+c*p.size[1]
            hz = p.size[2]
        else:
            hx = hy = p.size[0]
            hz = p.size[1]
        for i, h in enumerate((hx, hy, hz)):
            lo[i] = min(lo[i], p.center[i]-h)
            hi[i] = max(hi[i], p.center[i]+h)
    return [round(h-l, 4) for l, h in zip(lo, hi)]


def spec_record(spec):
    """JSON-safe static description, including the derived fields."""
    value = asdict(spec)
    value['parts'] = [asdict(p) for p in spec.parts]
    value['grasps'] = [{**asdict(g), 'approach_base_xyyaw': list(g.approach_base())} for g in spec.grasps]
    value['formations'] = [list(f) for f in spec.formations]
    value['mass_properties'] = mass_properties(spec)
    value['bounding_box_m'] = bounding_box(spec)
    value['visual'] = visual_spec(spec)
    value['contact'] = {'material': 'production box (friction %s)' % ' '.join(map(str, BOX_FRICTION)),
                        'finger_pairs': 'mirrored dispatch_box_geom pairs under the scene contact profile',
                        'weld': 'none'}
    return value


def visual_spec(spec):
    """How the kind appears in TOP RGB (for later detector support; robots get RGB only)."""
    import colorsys
    r, g, b = (float(v) for v in spec.rgba.split()[:3])
    h, sat, val = colorsys.rgb_to_hsv(r, g, b)
    bx, by, _ = bounding_box(spec)
    return {'colour': spec.colour, 'rgba': spec.rgba, 'shape': spec.shape, 'markings': spec.markings,
            'top_footprint_mm': [round(bx*1000), round(by*1000)],
            'top_footprint_px': [round(bx*TOP_PX_PER_M, 1), round(by*TOP_PX_PER_M, 1)],
            'aspect': round(max(bx, by)/min(bx, by), 2),
            'unlit_hsv_opencv': [round(h*179), round(sat*255), round(val*255)]}


def catalogue_record():
    value = {'schema': SCHEMA, 'version': CATALOGUE_VERSION,
             'grasp_convention': {'radius_m': GRASP_RADIUS_M, 'grasp_z_m': GRASP_Z_M,
                                  'handle_width_m': HANDLE_WIDTH_M, 'handle_height_m': HANDLE_HEIGHT_M},
             'kinds': {k: spec_record(v) for k, v in CATALOGUE.items()},
             'existing_solo': EXISTING_SOLO}
    value['sha256'] = hashlib.sha256(json.dumps(value, sort_keys=True).encode()).hexdigest()
    return value


# --------------------------------------------------------------------------
# Runtime instances (never part of the static spec)

@dataclass(frozen=True)
class CargoInstance:
    item_id: str
    kind: str
    pose: tuple                     # (x, y, yaw) world, setup only
    mass_kg: float | None = None    # calibration override; None = catalogue mass

    @property
    def body(self):
        return 'cargo_' + self.item_id

    @property
    def joint(self):
        return self.body + '_free'

    def spec(self):
        base = kind(self.kind)
        return base if self.mass_kg is None else with_mass(base, self.mass_kg)

    def geom(self, part):
        return f'{self.body}__{part}'


def instances(items):
    """Validate [{'item_id', 'kind', 'pose': [x, y, yaw]}] into CargoInstances."""
    out, seen = [], set()
    for item in items or ():
        iid = str(item['item_id'])
        if not iid.replace('_', '').isalnum() or iid in seen:
            raise ValueError('cargo item ids must be unique alphanumerics')
        kind(item['kind'])
        pose = tuple(float(v) for v in item['pose'])
        if len(pose) != 3:
            raise ValueError('cargo pose is [x, y, yaw]')
        seen.add(iid)
        out.append(CargoInstance(iid, item['kind'], pose, item.get('mass_kg')))
    return out


def world_grasps(instance, pose=None):
    """Grip points and approach base poses in the world for a cargo pose (teacher only)."""
    x0, y0, yaw0 = instance.pose if pose is None else pose
    c, s = math.cos(yaw0), math.sin(yaw0)
    out = {}
    for g in instance.spec().grasps:
        gx, gy, gz = g.grip_xyz
        bx, by, byaw = g.approach_base()
        out[g.role] = {'grip_xyz': (x0+c*gx-s*gy, y0+s*gx+c*gy, gz),
                       'base_xyyaw': (x0+c*bx-s*by, y0+s*bx+c*by, _wrap(yaw0+byaw)),
                       'geom': instance.geom(g.geom)}
    return out


def _wrap(a):
    return (a+math.pi) % (2*math.pi) - math.pi


def body_element(instance):
    spec = instance.spec()
    x, y, yaw = instance.pose
    body = ET.Element('body', name=instance.body, pos=f'{x} {y} .0005',
                      quat=f'{math.cos(yaw/2)} 0 0 {math.sin(yaw/2)}')
    ET.SubElement(body, 'freejoint', name=instance.joint)
    for part, mass in zip(spec.parts, part_masses(spec)):
        attrs = {'name': instance.geom(part.name), 'type': part.shape,
                 'pos': ' '.join(f'{v:.6f}' for v in part.center),
                 'size': ' '.join(f'{v:.6f}' for v in part.size),
                 'rgba': part.rgba or spec.rgba}
        if part.yaw:
            # quaternion, independent of the scene's compiler angle unit
            attrs['quat'] = f'{math.cos(part.yaw/2):.9f} 0 0 {math.sin(part.yaw/2):.9f}'
        if part.collision:
            attrs.update(mass=f'{mass:.9f}', contype='1', conaffinity='3',
                         friction=' '.join(map(str, BOX_FRICTION)), group='0')
        else:
            attrs.update(mass='0', contype='0', conaffinity='0', group='1')
        ET.SubElement(body, 'geom', attrs)
    return body


def add_cargo_xml(xml, items, *, mirror_pairs):
    """Append cargo bodies; mirror the box's finger contact pairs onto their geoms.

    ``mirror_pairs`` must be True when a contact profile declared finger/box
    pairs (local_contact_fine): cargo then gets exactly the box's pairs.
    """
    root = ET.fromstring(xml)
    world = root.find('worldbody')
    geoms = []
    for inst in items:
        body = body_element(inst)
        world.append(body)
        geoms += [g.get('name') for g in body.iter('geom') if g.get('contype') != '0']
    mirrored = 0
    if mirror_pairs:
        contact = root.find('contact')
        source = [p for p in (contact.findall('pair') if contact is not None else [])
                  if p.get('geom2') == 'dispatch_box_geom']
        if not source:
            raise ValueError('contact profile declares no finger/box pairs to mirror')
        for name in geoms:
            for pair in source:
                contact.append(ET.Element('pair', {**pair.attrib, 'geom2': name}))
                mirrored += 1
    if any(eq.get('active') != 'false' for eq in root.findall('equality/weld')):
        raise ValueError('weld assistance must be OFF')
    out = ET.tostring(root, encoding='unicode')
    return out, {'items': [{'item_id': i.item_id, 'kind': i.kind, 'body': i.body,
                            'mass_kg': i.spec().mass_kg} for i in items],
                 'finger_contact_pairs': mirrored, 'weld': 'off',
                 'catalogue_sha256': catalogue_record()['sha256']}


def place(world, items):
    """Setup-only placement of cargo freejoints (before a trial starts)."""
    import mujoco
    for inst in items:
        jid = mujoco.mj_name2id(world.model, mujoco.mjtObj.mjOBJ_JOINT, inst.joint)
        if jid < 0:
            raise ValueError(f'missing cargo joint: {inst.joint}')
        q, v = int(world.model.jnt_qposadr[jid]), int(world.model.jnt_dofadr[jid])
        x, y, yaw = inst.pose
        world.data.qpos[q:q+7] = [x, y, .0005, math.cos(yaw/2), 0, 0, math.sin(yaw/2)]
        world.data.qvel[v:v+6] = 0
    mujoco.mj_forward(world.model, world.data)
