"""Declarative scenario configs for the Korean-dialogue zone study: loader and
validator (package E-config).

Every robot-facing input of a trial is a function of the SCENARIO CONFIG and the
static map FILE (package A, ``harness.zone_study_inputs``). This module owns the
configs themselves (``configs/zone_study_scenarios/*.json``) and the checks that
keep them inside the study contract. It builds no scene, opens no MuJoCo window,
calls no model and writes no map: ``sim.zone_cargo`` is imported read-only for
the cargo mass / carrier table and ``maps/zones/*.json`` is only ever read.

Config layout -- the whole point is the public/private split:

* **public part** (``public_part``): ``schema``, ``scenario_id``, ``map_id``,
  ``landmark_detail``, ``seeds``, ``orders``. This is the part package A turns
  into the order sheet a robot receives on every call. It names only the
  static map, the item kind/count, how many robots the item needs, the
  destination zone and the COARSE pickup bay slot an item was put at during
  setup. It carries no metric coordinate at all (``_public_numbers_are_ints``).
  The DESCRIPTIVE design note moved to the private section as
  ``design_notes_ko`` (2026-09-26 review finding 17): it named the hidden event
  kind, its target and, in s6, the solution. The descriptive ``scenario_id``
  also stays evaluation-side; the robot-facing order sheet carries package A's
  opaque ``scenario_ref`` instead.
* **private part** (``private_part``, the scenario's ``eval`` key): the setup the
  scene builder consumes (arena variant, contact profile, weld off, exact
  placement poses) plus the hidden events with their SIM-time triggers, the
  per-trial budget and the discovery rule each event is supposed to satisfy.
  Nothing here may ever reach a model: ``validate`` proves the private section
  still trips ``harness.zone_study_contract.forbidden_key_hits``.

What ``validate`` proves (``Report.checks``):

``public_no_private_info``
    The public part carries no evaluation-only key, no non-ASCII key, no metric
    number and none of the private ids; the private part is still rejected by the
    payload validator (positive control).
``order_sheet_private_independent``
    The order sheet built from the config WITH its private section is byte-identical
    (SHA-256 of the canonical JSON) to the sheet built from the public part alone,
    and to the sheet built after the private section is mutated.
``hidden_events_private_only``
    Hidden events exist only under ``eval``; their triggers are SIM-time only,
    their ids are unique and ordered, every target exists in the public map or in
    the declared placements, and every discovery rule is own-camera.
``cargo_matches_sim``
    ``required_robots`` and the mass of every kind agree with
    ``sim.zone_cargo.CATALOGUE`` / ``EXISTING_SOLO`` (and package A's frozen
    ``REQUIRED_ROBOTS`` copy agrees with the same source), a kind heavier than one
    robot's measured lift capacity needs at least two carriers, and no scenario
    overrides a catalogue mass.
``placements_match_public_slots``
    Every declared pose lies inside the coarse slot its order names, the
    footprints do not overlap each other or a wall, and they stay inside the
    pickup region.
``leader_rotation``
    The rotating ``leader_ko`` leader (package A's ``leader_for_seed``) covers
    r1, r2 and r3 over the scenario's seeds.

Frames: ``pose_m`` is ``[x, y, yaw]`` in the map's world frame (metres, x east,
y north, yaw radians), the same setup-only convention as
``sim.zone_cargo.CargoInstance.pose``.

Limits: these are declarative checks over config, map JSON and the cargo
catalogue. Whether a placement is physically reachable, whether a blocked door is
really invisible from the far side and whether a trial actually completes are
SIM questions this module does not answer and must not be reported as verified.
"""
from __future__ import annotations

import copy
import json
import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path

from harness.zone_map_schematic import MAP_DIR, digest, load_map, map_bundle, pickup_bays
from harness.zone_study_contract import (ROBOTS, ZONE_IDS, ContractViolation, forbidden_key_hits,
                                         leader_for_seed, non_ascii_keys)
from harness.zone_study_inputs import (REQUIRED_ROBOTS, SCENARIO_SCHEMA, OrderSheetSource, order_sheet,
                                       validate_scenario)
from sim.zone_cargo import CATALOGUE, EXISTING_SOLO, MEASURED_SINGLE_ROBOT_CAPACITY_KG, bounding_box

ROOT = Path(__file__).resolve().parents[1]
SCENARIO_DIR = ROOT / 'configs' / 'zone_study_scenarios'
PRIVATE_SCHEMA = 'ugrp.zone_study_scenario_private.v1'
PRIVATE_KEY = 'eval'
PUBLIC_KEYS = ('schema', 'scenario_id', 'map_id', 'landmark_detail', 'seeds', 'orders')
PRIVATE_KEYS = ('schema', 'setup', 'hidden_events', 'budget', 'tests_ko', 'notes_ko', 'design_notes_ko')
SETUP_KEYS = ('arena_variant', 'map_file_sha256', 'contact_profile', 'weld', 'robot_spawns', 'placements')
PLACEMENT_KEYS = ('item_id', 'kind', 'order_id', 'slot', 'pose_m', 'notes_ko')
EVENT_KEYS = ('event_id', 'kind', 'trigger', 'target', 'discovery', 'notes_ko')
BUDGET_KEYS = ('sim_seconds', 'http_attempts_per_trial', 'http_attempts_per_actor', 'output_tokens_per_call')
EVENT_KINDS = ('passage_blocked', 'passage_cleared', 'item_moved', 'item_dropped', 'robot_hold',
               'obstruction_added')
TRIGGER_KINDS = ('sim_time',)
# Every discovery rule is own-camera: the host never announces a hidden event and
# never wakes a robot because one fired.
DISCOVERY_KINDS = ('own_camera_near_anchor', 'own_camera_self', 'own_camera_any')
SPAWNS_DEFAULT = 'arena_default'
WELD_REQUIRED = 'off'
# Footprint margins for the static placement check (metres). The robot drives a
# 0.39 m chassis and reaches 0.195 m ahead, so items are kept a chassis width
# apart and a reach away from a wall. These are layout sanity bounds, not a
# reachability proof.
ITEM_CLEARANCE_M = .20
WALL_CLEARANCE_M = .20
SLOT_MARGIN_M = .05
# A loaded robot's turning envelope (sim.zone_arena: 0.39 m, which is why the
# one-robot door is 0.50 m). A "blocked" passage must leave less than this.
LOADED_ROBOT_WIDTH_M = .39
CARGO_KINDS = tuple(CATALOGUE)
BOX_KINDS = tuple(k for k in REQUIRED_ROBOTS if k not in CATALOGUE)


class ScenarioError(ContractViolation):
    """A scenario config that breaks the study contract or the cargo catalogue."""


# ---------------------------------------------------------------------------
# Cargo catalogue (read-only from sim.zone_cargo)

def cargo_table() -> dict:
    """kind -> {required_carriers, mass_kg, footprint_m, source} straight from the SIM catalogue."""
    table = {}
    for kind, spec in CATALOGUE.items():
        extents = bounding_box(spec)
        table[kind] = {'required_carriers': spec.required_carriers, 'mass_kg': spec.mass_kg,
                       'footprint_m': [extents[0], extents[1]], 'tier': spec.tier,
                       'source': 'sim.zone_cargo.CATALOGUE'}
    box = EXISTING_SOLO['box']
    for kind in BOX_KINDS:
        table[kind] = {'required_carriers': box['required_carriers'], 'mass_kg': box['mass_kg'],
                       'footprint_m': [box['dims_m'][0], box['dims_m'][1]], 'tier': 'solo',
                       'source': 'sim.zone_cargo.EXISTING_SOLO[box]'}
    return table


def half_footprint(kind: str, yaw: float = 0.) -> tuple[float, float]:
    """Axis-aligned half extents of a kind's footprint at ``yaw`` (radians)."""
    fx, fy = cargo_table()[kind]['footprint_m']
    hx, hy = fx / 2., fy / 2.
    c, s = abs(math.cos(float(yaw))), abs(math.sin(float(yaw)))
    return c * hx + s * hy, s * hx + c * hy


def _rect(center: Sequence[float], half: Sequence[float], grow: float = 0.) -> tuple:
    return (float(center[0]) - float(half[0]) - grow, float(center[0]) + float(half[0]) + grow,
            float(center[1]) - float(half[1]) - grow, float(center[1]) + float(half[1]) + grow)


def _overlaps(a: tuple, b: tuple) -> bool:
    return a[0] < b[1] and b[0] < a[1] and a[2] < b[3] and b[2] < a[3]


def _inside(inner: tuple, outer: tuple) -> bool:
    return inner[0] >= outer[0] and inner[1] <= outer[1] and inner[2] >= outer[2] and inner[3] <= outer[3]


# ---------------------------------------------------------------------------
# Loading and the public/private split

def scenario_ids(*, directory: Path | str = SCENARIO_DIR) -> tuple[str, ...]:
    return tuple(sorted(p.stem for p in Path(directory).glob('*.json')))


def scenario_path(scenario_id: str, *, directory: Path | str = SCENARIO_DIR) -> Path:
    if not isinstance(scenario_id, str) or not scenario_id or '/' in scenario_id \
            or scenario_id.startswith('.'):
        raise ScenarioError(f'bad scenario_id: {scenario_id!r}')
    return Path(directory) / (scenario_id + '.json')


def load(scenario_id: str, *, directory: Path | str = SCENARIO_DIR) -> dict:
    """Read one scenario config. No map access, no simulator, no live state."""
    path = scenario_path(scenario_id, directory=directory)
    scenario = json.loads(path.read_text())
    if scenario.get('scenario_id') != scenario_id:
        raise ScenarioError(f'{path.name}: scenario_id {scenario.get("scenario_id")!r} differs from the '
                            f'file name')
    return scenario


def load_all(*, directory: Path | str = SCENARIO_DIR) -> dict:
    return {sid: load(sid, directory=directory) for sid in scenario_ids(directory=directory)}


def public_part(scenario: Mapping) -> dict:
    """The part that becomes the robot-facing order sheet. Never the hidden events."""
    return copy.deepcopy({k: scenario[k] for k in PUBLIC_KEYS if k in scenario})


def private_part(scenario: Mapping) -> dict:
    """Setup placements, hidden events, budget. Evaluation and scene builder only."""
    return copy.deepcopy(dict(scenario.get(PRIVATE_KEY) or {}))


def bundle_for(scenario: Mapping, *, maps_dir: Path | str = MAP_DIR, schematic: bool = False) -> dict:
    """The map bundle a scenario pins: map file hash + public projection hash (read-only)."""
    return map_bundle(scenario['map_id'], maps_dir=maps_dir,
                      landmark_detail=scenario.get('landmark_detail', 'full'), schematic=schematic)


def leader_rotation(seeds: Sequence[int]) -> dict:
    """seed -> the robot that doubles as leader in ``leader_ko`` (package A's rotation)."""
    return {int(seed): leader_for_seed('leader_ko', int(seed)) for seed in seeds}


# ---------------------------------------------------------------------------
# Report

@dataclass
class Report:
    """One scenario's validation result: per-check problems plus the run manifest."""

    scenario_id: str
    checks: dict = field(default_factory=dict)
    manifest: dict = field(default_factory=dict)

    @property
    def problems(self) -> list[str]:
        return [f'{name}: {problem}' for name, problems in self.checks.items() for problem in problems]

    @property
    def ok(self) -> bool:
        return not self.problems

    def raise_for_problems(self) -> Report:
        if not self.ok:
            raise ScenarioError(f'{self.scenario_id} is not a valid study scenario: '
                                + '; '.join(self.problems))
        return self


CHECK_NAMES = ('schema', 'public_no_private_info', 'order_sheet_private_independent',
               'hidden_events_private_only', 'cargo_matches_sim', 'placements_match_public_slots',
               'leader_rotation')


# ---------------------------------------------------------------------------
# Checks

def _numbers(value, out=None) -> list:
    out = [] if out is None else out
    if isinstance(value, bool):
        return out
    if isinstance(value, (int, float)):
        out.append(value)
    elif isinstance(value, Mapping):
        for item in value.values():
            _numbers(item, out)
    elif isinstance(value, Sequence) and not isinstance(value, (str, bytes)):
        for item in value:
            _numbers(item, out)
    return out


def _strings(value, out=None) -> list:
    out = [] if out is None else out
    if isinstance(value, str):
        out.append(value)
    elif isinstance(value, Mapping):
        for key, item in value.items():
            out.append(key)
            _strings(item, out)
    elif isinstance(value, Sequence) and not isinstance(value, (str, bytes)):
        for item in value:
            _strings(item, out)
    return out


def private_ids(private: Mapping) -> tuple[str, ...]:
    """The private-only tokens that must never appear in the public part."""
    ids = [str(event.get('event_id')) for event in private.get('hidden_events') or ()
           if isinstance(event, Mapping) and event.get('event_id')]
    for event in private.get('hidden_events') or ():
        target = event.get('target') if isinstance(event, Mapping) else None
        if isinstance(target, Mapping) and isinstance(target.get('obstacle'), Mapping):
            obstacle_id = target['obstacle'].get('obstacle_id')
            if obstacle_id:
                ids.append(str(obstacle_id))
    return tuple(dict.fromkeys(ids))


def check_public_no_private_info(scenario: Mapping) -> list[str]:
    """The public part carries no evaluation-only key, no coordinate and no private id."""
    public, private = public_part(scenario), private_part(scenario)
    out = list(forbidden_key_hits(public))
    out += [f'non-ASCII key in the public part: {hit}' for hit in non_ascii_keys(public)]
    extra = sorted(set(scenario) - set(PUBLIC_KEYS) - {PRIVATE_KEY})
    if extra:
        out.append(f'unknown top-level key(s): {extra}')
    floats = [n for n in _numbers(public) if not isinstance(n, int)]
    if floats:
        out.append(f'the public part must carry no metric number (only counts and seeds), found: {floats}')
    tokens = _strings(public)
    leaked = sorted(token for token in private_ids(private)
                    if any(token in text for text in tokens))
    if leaked:
        out.append(f'private id(s) named in the public part: {leaked}')
    if not private:
        return out
    # Positive control: the private section is exactly what a robot payload may
    # never contain, so package A's payload check must still reject it by key.
    hits = ' '.join(forbidden_key_hits(private))
    if 'hidden_events' not in hits:
        out.append('forbidden_key_hits no longer flags hidden_events: the schedule would pass a robot '
                   'payload check')
    if (private.get('setup') or {}).get('placements') and 'pose_m' not in hits:
        out.append('forbidden_key_hits no longer flags the setup pose_m keys: the exact placements would '
                   'pass a robot payload check')
    return out


def check_order_sheet_private_independent(scenario: Mapping, bundle: Mapping) -> list[str]:
    """The order sheet must be identical with, without and after mutating the private section."""
    out: list[str] = []
    try:
        full = digest(order_sheet(scenario, bundle))
        public_only = digest(order_sheet(public_part(scenario), bundle))
    except ContractViolation as error:
        return [f'the order sheet could not be built: {error}']
    if full != public_only:
        out.append(f'the order sheet changes when the private section is dropped '
                   f'({full[:16]} vs {public_only[:16]})')
    mutated = copy.deepcopy(dict(scenario))
    mutated[PRIVATE_KEY] = {'schema': PRIVATE_SCHEMA,
                            'hidden_events': [{'event_id': 'probe_event', 'kind': 'item_dropped',
                                               'trigger': {'kind': 'sim_time', 'at_sim_s': 123.5},
                                               'target': {'item_id': 'probe_item'},
                                               'discovery': {'kind': 'own_camera_self'}}]}
    if digest(order_sheet(mutated, bundle)) != full:
        out.append('the order sheet changes when the private section is replaced')
    source_full, source_public = OrderSheetSource(scenario, bundle), OrderSheetSource(public_part(scenario),
                                                                                     bundle)
    if source_full.sha256 != source_public.sha256:
        out.append('OrderSheetSource disagrees with/without the private section')
    source_full.assert_unchanged()
    if source_public.hidden_events():
        out.append('the public part alone must yield no hidden event')
    return out


def _passages(public_map: Mapping) -> dict:
    return {p['id']: p for p in public_map.get('passages', ())}


def free_gaps_m(passage: Mapping, obstacle: Mapping) -> list[float] | None:
    """Free widths left on each side of ``obstacle`` across ``passage`` (None: not in the opening).

    The opening of a door/corridor runs across its ``axis``, so an ``axis: x``
    passage opens along y. A blocking obstacle must sit in the opening on the
    other axis as well, otherwise it is next to the door, not in it.
    """
    index = 1 if passage.get('axis', 'x') == 'x' else 0
    other = 1 - index
    centre, half = passage['center_m'], passage['half_extents_m']
    o_centre, o_half = obstacle['center_m'], obstacle['half_extents_m']
    if abs(float(o_centre[other]) - float(centre[other])) >= float(half[other]) + float(o_half[other]):
        return None
    low, high = float(centre[index]) - float(half[index]), float(centre[index]) + float(half[index])
    o_low, o_high = float(o_centre[index]) - float(o_half[index]), float(o_centre[index]) + float(o_half[index])
    if o_high <= low or o_low >= high:
        return None
    return [max(0., o_low - low), max(0., high - o_high)]


def _location_refs(public_map: Mapping) -> frozenset[str]:
    refs = {b['bay_id'] for b in public_map.get('pickup_bays', ())}
    refs |= {s['slot_id'] for b in public_map.get('pickup_bays', ()) for s in b['slots']}
    refs |= {s['slot_id'] for slots in public_map.get('zone_slots', {}).values() for s in slots}
    return frozenset(refs | set(ZONE_IDS) | {'pickup'})


def check_hidden_events_private_only(scenario: Mapping, bundle: Mapping) -> list[str]:
    """Hidden events live under ``eval`` only, fire on SIM time and are found by own camera."""
    out: list[str] = []
    public = public_part(scenario)
    if 'hidden_events' in set(_strings(public)):
        out.append('the public part names hidden_events')
    private = private_part(scenario)
    unknown = sorted(set(private) - set(PRIVATE_KEYS))
    if unknown:
        out.append(f'unknown private key(s): {unknown}')
    if private and private.get('schema') != PRIVATE_SCHEMA:
        out.append(f'the private section needs schema {PRIVATE_SCHEMA}, got {private.get("schema")!r}')
    events = private.get('hidden_events')
    if events is None:
        return out + ['the private section must declare hidden_events (an empty list is fine)']
    if not isinstance(events, Sequence) or isinstance(events, str):
        return out + ['hidden_events must be a list']
    passages, refs = _passages(bundle['public_map']), _location_refs(bundle['public_map'])
    walls = [(w['id'], w['center_m'], w['half_extents_m']) for w in bundle['public_map']['walls']]
    bounds = bundle['public_map']['bounds_m']
    arena = (float(bounds[0]), float(bounds[1]), float(bounds[2]), float(bounds[3]))
    kinds = {p.get('item_id'): p.get('kind') for p in (private.get('setup') or {}).get('placements') or ()
             if isinstance(p, Mapping)}
    seen, last = set(), -1.
    for index, event in enumerate(events):
        label = f'hidden_events[{index}]'
        if not isinstance(event, Mapping):
            out.append(f'{label} must be an object')
            continue
        unknown = sorted(set(event) - set(EVENT_KEYS))
        if unknown:
            out.append(f'{label} has unknown key(s): {unknown}')
        event_id = event.get('event_id')
        if not isinstance(event_id, str) or not event_id:
            out.append(f'{label} needs an event_id')
        elif event_id in seen:
            out.append(f'{label}: duplicate event_id {event_id}')
        else:
            seen.add(event_id)
        kind = event.get('kind')
        if kind not in EVENT_KINDS:
            out.append(f'{label}: kind must be one of {EVENT_KINDS}, got {kind!r}')
        trigger = event.get('trigger')
        if not isinstance(trigger, Mapping) or trigger.get('kind') not in TRIGGER_KINDS \
                or set(trigger) - {'kind', 'at_sim_s'}:
            out.append(f'{label}: trigger must be {{"kind": "sim_time", "at_sim_s": <seconds>}}')
        else:
            at = trigger.get('at_sim_s')
            if isinstance(at, bool) or not isinstance(at, (int, float)) or at < 0:
                out.append(f'{label}: at_sim_s must be a non-negative number of SIM seconds')
            else:
                if at < last:
                    out.append(f'{label}: hidden events must be ordered by at_sim_s')
                last = float(at)
        out += _event_target(label, kind, event.get('target'), passages, kinds, walls, arena)
        out += _event_discovery(label, kind, event.get('discovery'), passages, refs)
    return out


def _event_target(label: str, kind: str, target, passages, kinds, walls, arena) -> list[str]:
    if not isinstance(target, Mapping):
        return [f'{label}: target must be an object']
    out: list[str] = []
    if kind in ('passage_blocked', 'passage_cleared'):
        if set(target) - {'passage', 'obstacle'}:
            out.append(f'{label}: a passage event targets {{"passage", "obstacle"}}')
        passage = target.get('passage')
        if passage not in passages:
            return out + [f'{label}: passage {passage!r} is not in the map (known: {sorted(passages)})']
        obstacle = target.get('obstacle')
        if kind == 'passage_cleared':
            return out
        if not isinstance(obstacle, Mapping) or set(obstacle) - {'obstacle_id', 'center_m',
                                                                 'half_extents_m', 'height_m'}:
            return out + [f'{label}: passage_blocked needs an obstacle '
                          f'{{obstacle_id, center_m, half_extents_m, height_m}}']
        try:
            rect = _rect(obstacle['center_m'], obstacle['half_extents_m'])
            gaps = free_gaps_m(passages[passage], obstacle)
        except (KeyError, TypeError, IndexError):
            return out + [f'{label}: the obstacle needs center_m [x, y] and half_extents_m [hx, hy]']
        if not _inside(rect, arena):
            out.append(f'{label}: the obstacle is outside the arena bounds')
        if gaps is None:
            out.append(f'{label}: the obstacle does not sit in the opening of {passage}, so it blocks '
                       f'nothing')
        elif max(gaps) >= LOADED_ROBOT_WIDTH_M:
            out.append(f'{label}: {passage} still leaves {max(gaps):.3f} m free, at least the '
                       f'{LOADED_ROBOT_WIDTH_M} m a loaded robot needs: this is not a blockage')
    elif kind == 'obstruction_added':
        if set(target) - {'obstacle'} or not isinstance(target.get('obstacle'), Mapping):
            out.append(f'{label}: obstruction_added targets {{"obstacle": ...}}')
    elif kind == 'item_moved':
        if set(target) - {'item_id', 'to_pose_m'}:
            out.append(f'{label}: item_moved targets {{"item_id", "to_pose_m"}}')
        item_id = target.get('item_id')
        if item_id not in kinds:
            out.append(f'{label}: item {item_id!r} is not a declared placement')
        pose = target.get('to_pose_m')
        if not isinstance(pose, Sequence) or isinstance(pose, str) or len(pose) != 3 \
                or any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in pose):
            out.append(f'{label}: to_pose_m must be [x, y, yaw]')
        elif item_id in kinds:
            rect = _rect(pose[:2], half_footprint(kinds[item_id], pose[2]))
            if not _inside(rect, arena):
                out.append(f'{label}: to_pose_m is outside the arena bounds')
            for wall_id, centre, half in walls:
                if _overlaps(rect, _rect(centre, half)):
                    out.append(f'{label}: to_pose_m puts {item_id} inside {wall_id}')
    elif kind == 'item_dropped':
        if set(target) - {'item_id'}:
            out.append(f'{label}: item_dropped targets {{"item_id"}}')
        if target.get('item_id') not in kinds:
            out.append(f'{label}: item {target.get("item_id")!r} is not a declared placement')
    elif kind == 'robot_hold':
        if set(target) - {'robot_id', 'duration_s'}:
            out.append(f'{label}: robot_hold targets {{"robot_id", "duration_s"}}')
        if target.get('robot_id') not in ROBOTS:
            out.append(f'{label}: robot_id must be one of {ROBOTS}')
        duration = target.get('duration_s')
        if isinstance(duration, bool) or not isinstance(duration, (int, float)) or duration <= 0:
            out.append(f'{label}: duration_s must be a positive number of SIM seconds')
    return out


def _event_discovery(label: str, kind: str, discovery, passages, refs) -> list[str]:
    if not isinstance(discovery, Mapping):
        return [f'{label}: discovery must be an object; the host never announces a hidden event']
    out: list[str] = []
    if set(discovery) - {'kind', 'anchor', 'radius_m'}:
        out.append(f'{label}: discovery takes {{"kind", "anchor", "radius_m"}} only')
    if discovery.get('kind') not in DISCOVERY_KINDS:
        return out + [f'{label}: discovery kind must be one of {DISCOVERY_KINDS}, '
                      f'got {discovery.get("kind")!r}']
    if discovery['kind'] == 'own_camera_near_anchor':
        anchor = discovery.get('anchor')
        if anchor not in passages and anchor not in refs:
            out.append(f'{label}: discovery anchor {anchor!r} is not a map passage or location ref')
        radius = discovery.get('radius_m')
        if isinstance(radius, bool) or not isinstance(radius, (int, float)) or radius <= 0:
            out.append(f'{label}: own_camera_near_anchor needs a positive radius_m')
    elif 'anchor' in discovery or 'radius_m' in discovery:
        out.append(f'{label}: {discovery["kind"]} takes no anchor or radius_m')
    if kind in ('passage_blocked', 'passage_cleared') and discovery.get('kind') != 'own_camera_near_anchor':
        out.append(f'{label}: a passage event must be discovered near the passage itself')
    return out


def check_cargo_matches_sim(scenario: Mapping) -> list[str]:
    """Item kinds, carrier counts and masses must agree with ``sim.zone_cargo``."""
    out: list[str] = []
    table = cargo_table()
    capacity = MEASURED_SINGLE_ROBOT_CAPACITY_KG[0]
    for kind, carriers in REQUIRED_ROBOTS.items():
        if kind not in table:
            out.append(f'package A knows kind {kind!r}, the SIM catalogue does not')
        elif table[kind]['required_carriers'] != carriers:
            out.append(f'{kind}: package A says {carriers} robots, sim.zone_cargo says '
                       f'{table[kind]["required_carriers"]}')
    for kind, entry in table.items():
        if entry['mass_kg'] > capacity and entry['required_carriers'] < 2:
            out.append(f'{kind}: {entry["mass_kg"]} kg is above one robot measured capacity '
                       f'({capacity} kg) but the catalogue asks for {entry["required_carriers"]} carrier')
    for order in scenario.get('orders') or ():
        if not isinstance(order, Mapping):
            out.append('each order must be an object')
            continue
        order_id, kind = order.get('order_id'), order.get('kind')
        if kind not in table:
            out.append(f'{order_id}: unknown item kind {kind!r} (known: {sorted(table)})')
            continue
        entry = table[kind]
        required = order.get('required_robots', entry['required_carriers'])
        if required != entry['required_carriers']:
            out.append(f'{order_id}: {kind} needs {entry["required_carriers"]} robots '
                       f'(sim.zone_cargo, {entry["mass_kg"]} kg), the config says {required!r}')
        if required > len(ROBOTS):
            out.append(f'{order_id}: {kind} needs more robots than the team has')
        if 'mass_kg' in order:
            out.append(f'{order_id}: a scenario may not override the catalogue mass')
    for placement in (private_part(scenario).get('setup') or {}).get('placements') or ():
        if not isinstance(placement, Mapping):
            continue
        if placement.get('kind') not in table:
            out.append(f'placement {placement.get("item_id")!r}: unknown kind {placement.get("kind")!r}')
        if 'mass_kg' in placement:
            out.append(f'placement {placement.get("item_id")!r}: a scenario may not override the '
                       f'catalogue mass')
    return out


def check_placements_match_public_slots(scenario: Mapping, bundle: Mapping, *,
                                       maps_dir: Path | str = MAP_DIR) -> list[str]:
    """Exact private poses must sit inside the coarse public slot, clear of walls and each other."""
    out: list[str] = []
    setup = private_part(scenario).get('setup')
    if not isinstance(setup, Mapping):
        return ['the private section must declare a setup object']
    unknown = sorted(set(setup) - set(SETUP_KEYS))
    if unknown:
        out.append(f'unknown setup key(s): {unknown}')
    if setup.get('weld') != WELD_REQUIRED:
        out.append(f'setup.weld must be {WELD_REQUIRED!r}: weld assistance stays OFF')
    from sim.zone_arena import VARIANTS
    variant = setup.get('arena_variant')
    if variant not in VARIANTS:
        out.append(f'arena_variant must be a sim.zone_arena variant {tuple(VARIANTS)}, got {variant!r}')
    base = bundle['public_map'].get('base_map_id', bundle['map_id'])
    if variant is not None and variant != base:
        out.append(f'arena_variant {variant!r} is not the physical map behind {bundle["map_id"]} ({base})')
    from sim.dispatch_contact_profile import PROFILES
    from sim.zone_cargo_contact import CARGO_PROFILES
    if setup.get('contact_profile') not in tuple(PROFILES) + tuple(CARGO_PROFILES):
        out.append(f'contact_profile must be one of {tuple(PROFILES) + tuple(CARGO_PROFILES)}, '
                   f'got {setup.get("contact_profile")!r}')
    if setup.get('map_file_sha256') != bundle['map_file_sha256']:
        out.append(f'setup.map_file_sha256 {str(setup.get("map_file_sha256"))[:16]} does not match the map '
                   f'file {bundle["map_file_sha256"][:16]}: the map changed or the pin is stale')
    spawns = setup.get('robot_spawns', SPAWNS_DEFAULT)
    if spawns != SPAWNS_DEFAULT:
        if not isinstance(spawns, Mapping) or set(spawns) != set(ROBOTS):
            out.append(f'robot_spawns must be {SPAWNS_DEFAULT!r} or a pose per robot {ROBOTS}')
    placements = setup.get('placements')
    if not isinstance(placements, Sequence) or isinstance(placements, str) or not placements:
        return out + ['setup.placements must be a non-empty list']

    data, _ = load_map(bundle['map_id'], maps_dir=maps_dir)
    slots = {s['slot_id']: s for bay in pickup_bays(data) for s in bay['slots']}
    bays = {bay['bay_id']: bay for bay in pickup_bays(data)}
    pickup = bundle['public_map']['regions']['pickup']
    pickup_rect = _rect(pickup['center_m'], pickup['half_extents_m'])
    walls = [(w['id'], w['center_m'], w['half_extents_m']) for w in bundle['public_map']['walls']]
    orders = {o['order_id']: o for o in scenario.get('orders') or () if isinstance(o, Mapping)}
    table, seen, rects = cargo_table(), set(), []
    per_order: dict[str, list[str]] = {}
    for index, placement in enumerate(placements):
        label = f'placements[{index}]'
        if not isinstance(placement, Mapping):
            out.append(f'{label} must be an object')
            continue
        unknown = sorted(set(placement) - set(PLACEMENT_KEYS))
        if unknown:
            out.append(f'{label} has unknown key(s): {unknown}')
        item_id, kind = placement.get('item_id'), placement.get('kind')
        if not isinstance(item_id, str) or not item_id:
            out.append(f'{label} needs an item_id')
        elif item_id in seen:
            out.append(f'{label}: duplicate item_id {item_id}')
        else:
            seen.add(item_id)
        if kind not in table:
            continue
        pose = placement.get('pose_m')
        if not isinstance(pose, Sequence) or isinstance(pose, str) or len(pose) != 3 \
                or any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in pose):
            out.append(f'{label}: pose_m must be [x, y, yaw] in metres and radians')
            continue
        half = half_footprint(kind, pose[2])
        rect = _rect(pose[:2], half)
        if not _inside(rect, pickup_rect):
            out.append(f'{label}: {item_id} does not fit inside the pickup region')
        for wall_id, center, wall_half in walls:
            if _overlaps(rect, _rect(center, wall_half, WALL_CLEARANCE_M)):
                out.append(f'{label}: {item_id} is within {WALL_CLEARANCE_M} m of {wall_id}')
        for other_id, other in rects:
            if _overlaps(_rect(pose[:2], half, ITEM_CLEARANCE_M / 2.), other):
                out.append(f'{label}: {item_id} is within {ITEM_CLEARANCE_M} m of {other_id}')
        rects.append((item_id, _rect(pose[:2], half, ITEM_CLEARANCE_M / 2.)))
        order_id = placement.get('order_id')
        if order_id is None:
            continue
        order = orders.get(order_id)
        if order is None:
            out.append(f'{label}: order_id {order_id!r} is not in the public part')
            continue
        per_order.setdefault(order_id, []).append(item_id)
        if order.get('kind') != kind:
            out.append(f'{label}: {item_id} is a {kind}, order {order_id} asks for {order.get("kind")!r}')
        location = order.get('initial_location') or {}
        slot_id, bay_id = placement.get('slot'), location.get('pickup_bay')
        if slot_id != location.get('slot'):
            out.append(f'{label}: slot {slot_id!r} differs from the public initial_location '
                       f'{location.get("slot")!r} of {order_id}')
        target = slots.get(slot_id) if slot_id else bays.get(bay_id)
        if target is None:
            out.append(f'{label}: neither slot {slot_id!r} nor bay {bay_id!r} is in the map')
        elif not _inside(rect, _rect(target['center_m'], target['half_extents_m'], -SLOT_MARGIN_M)):
            out.append(f'{label}: the pose of {item_id} is not inside {slot_id or bay_id} '
                       f'(the public part promised that slot)')
    for order_id, order in orders.items():
        got = per_order.get(order_id, [])
        if len(got) != order.get('count'):
            out.append(f'{order_id}: the public part asks for {order.get("count")} item(s), the private '
                       f'setup places {len(got)}')
        if order.get('identity') == 'specific_item' and sorted(got) != sorted(order.get('item_ids') or ()):
            out.append(f'{order_id}: a specific_item order needs exactly its item_ids placed, got {got}')
    return out


def check_leader_rotation(scenario: Mapping) -> list[str]:
    """The rotating ``leader_ko`` leader must cover r1, r2 and r3 over the seeds."""
    seeds = scenario.get('seeds')
    if not isinstance(seeds, Sequence) or isinstance(seeds, str) or not seeds:
        return ['seeds must be a non-empty list of ints']
    rotation = leader_rotation(seeds)
    missing = sorted(set(ROBOTS) - set(rotation.values()))
    if missing:
        return [f'the seeds {list(seeds)} never make {missing} the leader '
                f'(leader_ko rotates r1/r2/r3 as robots[seed % 3]); rotation={rotation}']
    return []


# ---------------------------------------------------------------------------
# Validation and manifest

def validate(scenario: Mapping, *, maps_dir: Path | str = MAP_DIR, bundle: Mapping | None = None) -> Report:
    """Run every study check over one scenario config (never raises on a bad config)."""
    scenario_id = scenario.get('scenario_id') if isinstance(scenario, Mapping) else None
    report = Report(str(scenario_id), {name: [] for name in CHECK_NAMES})
    if not isinstance(scenario, Mapping):
        report.checks['schema'].append('scenario config must be an object')
        return report
    if scenario.get('schema') != SCENARIO_SCHEMA:
        report.checks['schema'].append(f'schema must be {SCENARIO_SCHEMA}, got {scenario.get("schema")!r}')
        return report
    try:
        bundle = bundle_for(scenario, maps_dir=maps_dir) if bundle is None else bundle
    except (OSError, ValueError) as error:
        report.checks['schema'].append(f'the pinned map could not be read: {error}')
        return report
    if scenario.get('landmark_detail', 'full') != 'none' and not bundle['has_landmarks']:
        report.checks['schema'].append(f'{bundle["map_id"]} has no AprilTag landmarks, so own-camera '
                                       f'localisation has nothing to anchor on')
    try:
        validate_scenario(scenario, map_bundle=bundle)
    except ContractViolation as error:
        report.checks['schema'].append(str(error))
        return report
    report.checks['public_no_private_info'] = check_public_no_private_info(scenario)
    report.checks['order_sheet_private_independent'] = check_order_sheet_private_independent(scenario,
                                                                                            bundle)
    report.checks['hidden_events_private_only'] = check_hidden_events_private_only(scenario, bundle)
    report.checks['cargo_matches_sim'] = check_cargo_matches_sim(scenario)
    report.checks['placements_match_public_slots'] = check_placements_match_public_slots(
        scenario, bundle, maps_dir=maps_dir)
    report.checks['leader_rotation'] = check_leader_rotation(scenario)
    report.manifest = manifest(scenario, bundle) if report.ok else {}
    return report


def validate_all(*, directory: Path | str = SCENARIO_DIR, maps_dir: Path | str = MAP_DIR) -> dict:
    return {sid: validate(scenario, maps_dir=maps_dir)
            for sid, scenario in load_all(directory=directory).items()}


def manifest(scenario: Mapping, bundle: Mapping) -> dict:
    """What a run bundle records about a scenario (docs/execution_versioning.md)."""
    private = private_part(scenario)
    setup = private.get('setup') or {}
    source = OrderSheetSource(scenario, bundle)
    events = private.get('hidden_events') or []
    return {'scenario_schema': SCENARIO_SCHEMA, 'private_schema': PRIVATE_SCHEMA,
            'scenario_id': scenario['scenario_id'], 'scenario_sha256': digest(scenario),
            'public_sha256': digest(public_part(scenario)), 'private_sha256': digest(private),
            'order_sheet_sha256': source.sha256, 'orders': len(scenario['orders']),
            'map': {'map_id': bundle['map_id'], 'map_file': bundle['map_file'],
                    'map_file_sha256': bundle['map_file_sha256'],
                    'public_map_sha256': bundle['public_map_sha256'],
                    'base_map': bundle.get('base_map'), 'has_landmarks': bundle['has_landmarks'],
                    'landmark_detail': bundle['landmark_detail']},
            'arena_variant': setup.get('arena_variant'), 'contact_profile': setup.get('contact_profile'),
            'weld': setup.get('weld'), 'placements': len(setup.get('placements') or ()),
            'hidden_events': [{'event_id': e.get('event_id'), 'kind': e.get('kind'),
                               'at_sim_s': (e.get('trigger') or {}).get('at_sim_s'),
                               'discovery': (e.get('discovery') or {}).get('kind')} for e in events],
            'seeds': list(scenario['seeds']), 'leader_rotation': leader_rotation(scenario['seeds']),
            'budget': dict(private.get('budget') or {}),
            'cargo_source': 'sim.zone_cargo', 'cargo_kinds': sorted({o['kind'] for o in scenario['orders']}),
            'verified': ['config schema', 'public/private split', 'order sheet independence',
                         'hidden event schema', 'cargo carriers and mass', 'placement geometry',
                         'leader rotation'],
            'not_verified': ['physical reachability of a placement', 'whether a hidden event is really '
                             'invisible outside its discovery radius', 'trial completion']}


def scenario_table() -> list[dict]:
    """One row per scenario: what it tests, the map it pins and its hidden events (docs)."""
    rows = []
    for sid, scenario in load_all().items():
        private = private_part(scenario)
        rows.append({'scenario_id': sid, 'map_id': scenario['map_id'],
                     'orders': len(scenario['orders']), 'seeds': list(scenario['seeds']),
                     'leader_rotation': leader_rotation(scenario['seeds']),
                     'hidden_events': [e.get('kind') for e in private.get('hidden_events') or ()],
                     'tests_ko': private.get('tests_ko', '')})
    return rows


def main(argv: Sequence[str] | None = None) -> int:
    """``python -m harness.zone_study_scenarios`` -- validate every config, print the report."""
    import argparse
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('scenario_id', nargs='*', help='default: every config in configs/zone_study_scenarios')
    parser.add_argument('--manifest', action='store_true', help='print the run manifest of each scenario')
    args = parser.parse_args(argv)
    reports = ({sid: validate(load(sid)) for sid in args.scenario_id} if args.scenario_id
               else validate_all())
    failures = 0
    for sid, report in sorted(reports.items()):
        print(f'{"ok  " if report.ok else "FAIL"} {sid}')
        for problem in report.problems:
            print(f'       {problem}')
        failures += 0 if report.ok else 1
        if args.manifest and report.ok:
            print(json.dumps(report.manifest, ensure_ascii=False, indent=2, sort_keys=True))
    print(f'{len(reports) - failures}/{len(reports)} scenario config(s) valid')
    return 1 if failures else 0


__all__ = ['SCENARIO_DIR', 'PRIVATE_SCHEMA', 'PRIVATE_KEY', 'PUBLIC_KEYS', 'PRIVATE_KEYS', 'EVENT_KINDS',
           'TRIGGER_KINDS', 'DISCOVERY_KINDS', 'CARGO_KINDS', 'BOX_KINDS', 'ScenarioError', 'Report',
           'CHECK_NAMES', 'cargo_table', 'half_footprint', 'scenario_ids', 'scenario_path', 'load',
           'load_all', 'public_part', 'private_part', 'private_ids', 'bundle_for', 'leader_rotation',
           'validate', 'validate_all', 'manifest', 'scenario_table', 'main']


if __name__ == '__main__':
    raise SystemExit(main())
