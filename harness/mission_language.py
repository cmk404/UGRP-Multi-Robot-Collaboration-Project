"""Deterministic natural-language compiler for warehouse transfer missions."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import re
from typing import Mapping


ZONE_REGISTRY: dict[str, dict[str, object]] = {
    "A": {
        "name": "checkpoint_a",
        "color": "blue",
        "aliases": ("a", "a구역", "a 구역", "에이 구역", "zone a", "area a", "파란 구역", "파란색 구역", "blue zone", "출발 구역", "출발지"),
    },
    "B": {
        "name": "checkpoint_b",
        "color": "green",
        "aliases": ("b", "b구역", "b 구역", "비 구역", "zone b", "area b", "초록 구역", "녹색 구역", "green zone", "도착 구역", "도착지"),
    },
    "C": {
        "name": "checkpoint_c",
        "color": "yellow",
        "aliases": ("c", "c구역", "c 구역", "씨 구역", "zone c", "area c", "노란 구역", "노란색 구역", "yellow zone", "대기 구역", "중간 구역"),
    },
}

TYPE_ALIASES = {
    "plank": ("목재", "판재", "나무판", "긴 판", "plank", "timber", "wood"),
    "pipe": ("파이프", "관", "원통", "pipe", "cylinder"),
    "crate": ("상자", "박스", "화물상자", "crate", "box"),
}


@dataclass(frozen=True)
class ZoneRef:
    id: str
    name: str
    color: str


@dataclass(frozen=True)
class CargoSelector:
    mode: str
    object_type: str | None = None
    cargo_color: str | None = None


@dataclass(frozen=True)
class TransferMission:
    kind: str
    mission_id: str
    source: ZoneRef
    destination: ZoneRef
    route: tuple[ZoneRef, ...]
    selector: CargoSelector
    raw: str

    def as_dict(self) -> dict[str, object]:
        value = asdict(self)
        value["route"] = list(value["route"])
        return value


def _normal(text: object) -> str:
    return " ".join(str(text or "").strip().lower().split())


def _contains_alias(text: str, alias: str) -> bool:
    """Match short Korean/English nouns without firing inside another word."""
    value = str(alias).lower()
    if len(value) == 1 or value.isascii():
        return bool(re.search(
            rf"(?<![0-9a-z가-힣]){re.escape(value)}(?![0-9a-z가-힣])",
            text,
        ))
    return value in text


def _zone_mentions(text: str, registry: Mapping[str, Mapping[str, object]]) -> list[tuple[int, str]]:
    found: list[tuple[int, str]] = []
    for zone_id, entry in registry.items():
        aliases = tuple(str(v).lower() for v in (entry.get("aliases") or ()))
        # Single Latin labels require a word boundary; longer Korean/English
        # aliases may be matched literally.
        for alias in aliases:
            if len(alias) == 1 and alias.isascii():
                for match in re.finditer(
                    rf"(?<![a-z0-9]){re.escape(alias)}(?![a-z0-9])", text
                ):
                    found.append((match.start(), zone_id))
            else:
                start = 0
                while True:
                    index = text.find(alias, start)
                    if index < 0:
                        break
                    found.append((index, zone_id))
                    start = index + max(1, len(alias))
    return sorted(set(found))


def normalize_zone(token: object, *, registry: Mapping[str, Mapping[str, object]] = ZONE_REGISTRY) -> ZoneRef:
    text = _normal(token)
    mentions = _zone_mentions(text, registry)
    ids = list(dict.fromkeys(zone_id for _index, zone_id in mentions))
    if len(ids) != 1:
        raise ValueError("AMBIGUOUS_ZONE")
    zone_id = ids[0]
    entry = registry[zone_id]
    return ZoneRef(zone_id, str(entry.get("name") or zone_id), str(entry.get("color") or ""))


def parse_transfer_command(
    text: object,
    *,
    zone_registry: Mapping[str, Mapping[str, object]] = ZONE_REGISTRY,
) -> TransferMission:
    raw = " ".join(str(text or "").strip().split())
    low = raw.lower()
    has_cargo = any(token in low for token in ("짐", "화물", "물건", "물체", "cargo", "payload", "load")) or any(
        _contains_alias(low, alias) for aliases in TYPE_ALIASES.values() for alias in aliases
    )
    has_transfer = any(token in low for token in ("옮", "운반", "이동", "보내", "transfer", "transport", "move", "deliver"))
    if not (has_cargo and has_transfer):
        raise ValueError("NOT_TRANSFER_MISSION")

    mentions = _zone_mentions(low, zone_registry)
    ordered = list(dict.fromkeys(zone_id for _index, zone_id in mentions))
    if len(ordered) < 2:
        if len({index for index, _zone_id in mentions}) >= 2 and len(ordered) == 1:
            raise ValueError("INVALID_ROUTE")
        # Source/destination wording is a stable A/B alias even if the operator
        # omits the letters.
        if any(k in low for k in ("출발", "로딩", "loading")) and any(
            k in low for k in ("도착", "배송", "delivery")
        ):
            ordered = ["A", "B"]
        else:
            raise ValueError("AMBIGUOUS_ZONE")
    source_id, destination_id = ordered[0], ordered[-1]
    if source_id == destination_id:
        raise ValueError("INVALID_ROUTE")
    zone_order = ("A", "B", "C")
    start_index = zone_order.index(source_id)
    end_index = zone_order.index(destination_id)
    step = 1 if end_index > start_index else -1
    route_ids = tuple(zone_order[index] for index in range(start_index, end_index + step, step))
    if len(ordered) > 2 and tuple(ordered) != route_ids:
        raise ValueError("INVALID_ROUTE")

    object_type = None
    for canonical, aliases in TYPE_ALIASES.items():
        if any(_contains_alias(low, alias) for alias in aliases):
            object_type = canonical
            break
    selector_mode = "type" if object_type else "all"
    selector = CargoSelector(selector_mode, object_type=object_type)
    canonical = f"warehouse:{'>'.join(route_ids)}:{selector.mode}:{object_type or '*'}"
    digest = hashlib.sha256(canonical.encode("utf-8")).hexdigest()[:10]
    source_entry = zone_registry[source_id]
    destination_entry = zone_registry[destination_id]
    route = tuple(
        ZoneRef(
            zone_id,
            str(zone_registry[zone_id].get("name")),
            str(zone_registry[zone_id].get("color")),
        )
        for zone_id in route_ids
    )
    return TransferMission(
        kind="WAREHOUSE_TRANSFER",
        mission_id=f"warehouse_{source_id.lower()}_to_{destination_id.lower()}_{digest}",
        source=ZoneRef(source_id, str(source_entry.get("name")), str(source_entry.get("color"))),
        destination=ZoneRef(destination_id, str(destination_entry.get("name")), str(destination_entry.get("color"))),
        route=route,
        selector=selector,
        raw=raw,
    )


def maybe_parse_transfer_command(text: object) -> TransferMission | None:
    try:
        return parse_transfer_command(text)
    except ValueError:
        return None


__all__ = [
    "ZONE_REGISTRY", "TYPE_ALIASES", "ZoneRef", "CargoSelector",
    "TransferMission", "normalize_zone", "parse_transfer_command",
    "maybe_parse_transfer_command",
]
