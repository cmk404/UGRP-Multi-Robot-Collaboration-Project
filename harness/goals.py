"""Deterministic goal predicates over transferable WorldState."""

from __future__ import annotations

import re
from dataclasses import dataclass

from .state import WorldState


@dataclass(frozen=True)
class GoalSpec:
    kind: str  # SCAN | SEARCH | GRASP | LIFT | PLACE | GENERIC
    raw: str
    subject: str = "UNKNOWN"  # RED_BLOCK | UNKNOWN
    target: str = "UNKNOWN"   # YELLOW_BLOCK | BLUE_BLOCK | UNKNOWN


def infer_goal(text: str) -> GoalSpec:
    t = text.lower()

    def mentioned_color(segment: str) -> str:
        if "빨간" in segment or "red" in segment:
            return "RED_BLOCK"
        if "파란" in segment or "파랑" in segment or "blue" in segment:
            return "BLUE_BLOCK"
        if "노란" in segment or "yellow" in segment:
            return "YELLOW_BLOCK"
        return "UNKNOWN"

    # Peer handoffs and chat replies often prefix the command with a status
    # sentence ("노란 블럭이 바닥층에 놓였어. 이제 파란 블럭을 노란 블럭 위에
    # 올려 줘"). Source/destination are read from the last sentence that names
    # a colour, otherwise the "에 놓" of the status sentence splits the text
    # and the real source colour is lost. Verbs are still matched on the whole.
    colour_tokens = ("빨간", "red", "파란", "파랑", "blue", "노란", "yellow")
    # A TEAM turn message may end with what the *next* robot should do
    # ("완료를 확인하면 send_peer_message로 R1에게 다음 단계(파란 블럭을 …)를
    # 넘겨"). That sentence names colours but is a delegation, not this robot's
    # own goal, so it is only used when no other colour sentence exists.
    delegation_markers = ("send_peer_message", "에게", "넘겨", "넘기", "다음 단계", "팀에 알려")
    clause = t
    sentences = [part.strip() for part in re.split(r"[.!?。\n]+", t) if part.strip()]
    if len(sentences) > 1:
        coloured = [part for part in sentences if any(token in part for token in colour_tokens)]
        own = [part for part in coloured if not any(marker in part for marker in delegation_markers)]
        candidates = own or coloured
        if candidates:
            clause = candidates[-1]

    # For placement commands, preserve source and destination separately.  In
    # Korean, the destination normally appears immediately before "위에";
    # pronouns such as "그거" deliberately leave the source UNKNOWN so a later
    # stateful executive can resolve it to the object currently held.
    split_markers = ("위에", "위로", "에 올", "에 놓", "onto", " on ")
    split_at = None
    for marker in split_markers:
        idx = clause.find(marker)
        if idx >= 0 and (split_at is None or idx < split_at):
            split_at = idx
    if split_at is not None:
        before, after = clause[:split_at], clause[split_at:]
        if clause[split_at:].startswith(("위에", "위로", "에 올", "에 놓")):
            mentioned_before = []
            for token, color in (
                ("빨간", "RED_BLOCK"), ("red", "RED_BLOCK"),
                ("파란", "BLUE_BLOCK"), ("파랑", "BLUE_BLOCK"), ("blue", "BLUE_BLOCK"),
                ("노란", "YELLOW_BLOCK"), ("yellow", "YELLOW_BLOCK"),
            ):
                if token in before:
                    mentioned_before.append(color)
            # A conversational pronoun can come before *or after* the destination
            # phrase (e.g. "파란 블럭 위에 올려볼래? 그거").  With only one
            # named colour, that colour is the destination and the source is the
            # already-held object.  Two named colours remain explicit source/dest.
            mentioned_before = list(dict.fromkeys(mentioned_before))
            has_pronoun = any(token in t for token in ("그거", "그것", "이거", "it", "that"))
            if len(mentioned_before) >= 2:
                subject, target = mentioned_before[0], mentioned_before[-1]
            elif mentioned_before:
                # In Korean ``파란 블럭 위에 올려봐`` names the *support*
                # object, not the carried source.  The source is intentionally
                # omitted and must be resolved from grasp state by the executive.
                # Requiring an explicit pronoun here made this exact natural
                # follow-up fall out of the deterministic PLACE fast path.
                subject, target = "UNKNOWN", mentioned_before[-1]
            else:
                subject, target = mentioned_color(before), mentioned_color(after)
        else:
            subject = mentioned_color(before)
            target = mentioned_color(after)
    else:
        subject = mentioned_color(clause)
        target = "UNKNOWN"

    scan_words = ("주변", "공간", "맵", "지도", "3d map", "3d맵", "scan", "스캔")
    scan_verbs = ("인식", "파악", "살펴", "훑", "갱신", "만들", "스캔", "scan", "map")
    if any(k in t for k in scan_words) and any(k in t for k in scan_verbs):
        return GoalSpec("SCAN", text, subject, target)

    search_verbs = ("찾아", "찾기", "검색", "탐색", "search", "find", "locate")
    if subject != "UNKNOWN" and any(k in t for k in search_verbs):
        return GoalSpec("SEARCH", text, subject, target)

    if any(k in t for k in ("옮겨", "옮기", "가져다", "놓아", "놓고", "놓아줘", "올려줘", "올려 놓", "올려놓", "얹어", "위에", "배달", "place", "move to", "deliver")):
        # If a sentence mentions two colours but the simple split missed the
        # target, take the last distinct colour as the destination.
        if target == "UNKNOWN":
            mentions=[]
            for token,color in (("빨간","RED_BLOCK"),("red","RED_BLOCK"),("파란","BLUE_BLOCK"),("파랑","BLUE_BLOCK"),("blue","BLUE_BLOCK"),("노란","YELLOW_BLOCK"),("yellow","YELLOW_BLOCK")):
                idx=clause.find(token)
                if idx>=0: mentions.append((idx,color))
            mentions.sort()
            if mentions:
                subject=mentions[0][1]
                for _,color in reversed(mentions):
                    if color != subject:
                        target=color; break
        return GoalSpec("PLACE", text, subject, target)
    if any(k in t for k in ("들어줘", "들어 올", "들어올", "lift", "들고 있어", "들고있")):
        return GoalSpec("LIFT", text, subject, target)
    if any(k in t for k in ("집어", "집고", "집어줘", "잡아", "grasp", "pick up", "pickup")):
        return GoalSpec("GRASP", text, subject, target)
    return GoalSpec("GENERIC", text, subject, target)


def goal_status(goal: GoalSpec, state: WorldState) -> str:
    """Return ACHIEVED, NOT_ACHIEVED, UNKNOWN or UNSUPPORTED."""
    if goal.kind == "SCAN":
        if state.last_action.name == "scan_world" and state.last_action.outcome_status == "ACHIEVED":
            return "ACHIEVED"
        if state.last_action.name == "scan_world" and state.last_action.outcome_status == "NOT_ACHIEVED":
            return "NOT_ACHIEVED"
        return "UNKNOWN"
    if goal.kind == "SEARCH":
        if state.target.visible is True:
            return "ACHIEVED"
        if state.last_action.name == "search" and state.last_action.outcome_status == "NOT_ACHIEVED":
            return "NOT_ACHIEVED"
        return "UNKNOWN"
    if goal.kind == "GRASP":
        expected_color = {
            "RED_BLOCK": "red", "BLUE_BLOCK": "blue", "YELLOW_BLOCK": "yellow",
        }.get(goal.subject)
        held_color_matches = (
            expected_color is None or state.grasp.held_object_color == expected_color
        )
        # The current REAL gripper has no positive force/contact sensor. Its
        # public pick contract therefore defines PROBABLE_HELD|HELD as successful
        # grasp completion. Requiring HELD here made a valid REAL/SIM pick
        # impossible to finish without an extra planner call/evidence source.
        if state.grasp.state in {"PROBABLE_HELD", "HELD"} and held_color_matches:
            return "ACHIEVED"
        if state.grasp.state == "EMPTY":
            return "NOT_ACHIEVED"
        return "UNKNOWN"
    if goal.kind == "LIFT":
        if state.grasp.state == "HELD":
            return "ACHIEVED"
        if state.grasp.state == "EMPTY":
            return "NOT_ACHIEVED"
        return "UNKNOWN"
    if goal.kind == "PLACE":
        if state.task.phase == "PLACED":
            return "ACHIEVED"
        if state.last_action.name in {"place", "place_on_red", "place_on_yellow", "place_on_blue", "stack_on", "stage_base"} and state.last_action.outcome_status == "NOT_ACHIEVED":
            return "NOT_ACHIEVED"
        return "UNKNOWN"
    return "UNKNOWN"


def final_is_non_success(text: str) -> bool:
    """Conservative lexical classifier for failure/uncertainty reports.

    It does not prove success. It only allows the agent to safely stop without
    falsely claiming completion when the sensor goal predicate is unresolved.
    """
    t = text.lower()
    markers = (
        "실패", "못했", "못 했", "못함", "못 했어", "못했어", "안 됐", "안됐",
        "확인할 수 없", "확인이 안", "확실하지", "불확실", "판단할 수 없",
        "중단", "멈췄", "찾지 못", "보이지 않", "도움이 필요", "재시도 필요",
        "failed", "failure", "couldn't", "could not", "unable", "uncertain",
        "cannot verify", "not verified", "not complete", "stopped", "abort",
    )
    return any(marker in t for marker in markers)
