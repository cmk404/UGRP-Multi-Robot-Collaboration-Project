"""Load executable Python actions from one swappable file."""

from __future__ import annotations

import importlib.util
import os
from pathlib import Path
from typing import Any

from .registry import Parameter, Registry, Tool


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ACTIONS_PATH = ROOT / "scripts" / "robot_actions.py"


def resolve_actions_path(path: str | Path | None = None) -> Path:
    if path:
        return Path(path).expanduser().resolve()
    env = os.environ.get("UGRP_ACTIONS")
    if env:
        return Path(env).expanduser().resolve()
    return DEFAULT_ACTIONS_PATH


def load_actions_module(path: str | Path | None = None):
    target = resolve_actions_path(path)
    spec = importlib.util.spec_from_file_location("ugrp_robot_actions", target)
    if spec is None or spec.loader is None:
        raise FileNotFoundError(target)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def action_map(module=None, *, path: str | Path | None = None) -> dict[str, str]:
    mod = module or load_actions_module(path)
    raw = getattr(mod, "ACTIONS", None)
    if not isinstance(raw, dict) or not raw:
        location = getattr(mod, "__file__", path or DEFAULT_ACTIONS_PATH)
        raise ValueError(f"{location} must define a non-empty ACTIONS dict")
    return {str(name): str(description) for name, description in raw.items()}


SKILLS = action_map()
SKILLS_SCRIPT = DEFAULT_ACTIONS_PATH
PLAN_SKIP = frozenset(
    {
        "look", "확인", "보기", "장면", "wait", "대기", "기다림", "확인하기",
    }
)
PLAN_ALIASES = {
    "approach": "approach",
    "다가감": "approach",
    "다가가기": "approach",
    "pick": "pick",
    "집기": "pick",
    # pick.py already lifts into the carry pose, so old carry wording maps to pick.
    "carry": "pick",
    "들기": "pick",
    "들고 있기": "pick",
    # REAL no longer exposes the legacy composite fetch tool. If a backend
    # still provides fetch, the exact-name check above keeps it; otherwise old
    # wording resolves to guarded pick and the executive inserts approach when needed.
    "fetch": "pick",
    "가져오기": "pick",
    "track": "track",
    "따라가기": "track",
    "앞으로": "move_forward",
    "전진": "move_forward",
    "뒤로": "move_backward",
    "후진": "move_backward",
    "왼쪽 이동": "move_left",
    "왼쪽으로 이동": "move_left",
    "좌측 이동": "move_left",
    "strafe_left": "move_left",
    "strafe left": "move_left",
    "오른쪽 이동": "move_right",
    "오른쪽으로 이동": "move_right",
    "우측 이동": "move_right",
    "strafe_right": "move_right",
    "strafe right": "move_right",
    "왼쪽 회전": "turn_left",
    "좌회전": "turn_left",
    "오른쪽 회전": "turn_right",
    "우회전": "turn_right",
    "정지": "stop_motion",
}


def resolve_plan_step(step: str, tool_names) -> str | None:
    """Map a plan step to a tool name, or None to skip look/wait words."""
    names = set(tool_names)
    text = " ".join(step.strip().split())
    if not text or text in PLAN_SKIP or text.lower() in PLAN_SKIP:
        return None
    if text in names:
        return text
    mapped = PLAN_ALIASES.get(text) or PLAN_ALIASES.get(text.lower())
    if mapped in names:
        return mapped
    return text


def _action_parameters(module, name: str) -> tuple[Parameter, ...]:
    """Load optional typed parameters declared by an actions module.

    Legacy action files do not need ACTION_PARAMETERS and remain zero-arg.
    Parameter ranges remain enforced by the action adapter itself; this layer
    exposes only the primitive type/default/description to the model.
    """
    raw_all = getattr(module, "ACTION_PARAMETERS", {})
    if not isinstance(raw_all, dict):
        return ()
    raw = raw_all.get(name) or {}
    if not isinstance(raw, dict):
        return ()
    type_map = {"bool": bool, "int": int, "float": float, "str": str}
    out: list[Parameter] = []
    for param_name, spec in raw.items():
        if not isinstance(spec, dict):
            raise ValueError(f"ACTION_PARAMETERS[{name!r}][{param_name!r}] must be a dict")
        py_type = type_map.get(str(spec.get("type", "")))
        if py_type is None:
            raise ValueError(f"unsupported action parameter type for {name}.{param_name}")
        required = bool(spec.get("required", False))
        default = None if required else spec.get("default")
        out.append(Parameter(
            name=str(param_name),
            type=py_type,
            required=required,
            default=default,
            description=str(spec.get("description") or ""),
        ))
    return tuple(out)


def default_registry(*, runner=None, actions_path: str | Path | None = None) -> Registry:
    module = load_actions_module(actions_path)
    skills = action_map(module)
    run = runner or getattr(module, "run")
    contracts = getattr(module, "CONTRACTS", {})
    if not isinstance(contracts, dict):
        contracts = {}
    cancel_current = getattr(module, "cancel_current", None)
    # A caller-supplied runner is normally a test/fake executor. Do not silently
    # bypass it through the module's production program fastpath.
    program_runner = getattr(module, "run_program", None) if runner is None else None
    registry = Registry(
        cancel_current=cancel_current if callable(cancel_current) else None,
        program_runner=program_runner if callable(program_runner) else None,
    )
    for name, description in skills.items():
        parameters = _action_parameters(module, name)
        registry.register(
            Tool(
                name=name,
                description=description,
                handler=_skill_handler(name, run),
                parameters=parameters,
                contract=dict(contracts.get(name) or {}),
            )
        )
    return registry


def tools_payload(registry: Registry, *, source: str | Path | None = None) -> dict[str, Any]:
    path = resolve_actions_path(source)
    try:
        rel = str(path.relative_to(ROOT))
    except ValueError:
        rel = str(path)
    tools = []
    for name in registry.names():
        tool = registry.get(name)
        tools.append(
            {
                "name": tool.name,
                "description": tool.description,
                "source": rel,
                "language": "python",
                "contract": tool.contract,
                "parameters": [
                    {
                        "name": param.name,
                        "type": param.type.__name__,
                        "required": param.required,
                        "default": param.default,
                        "description": param.description,
                    }
                    for param in tool.parameters
                ],
            }
        )
    return {"file": rel, "tools": tools}


def _skill_handler(name: str, runner):
    def handler(**kwargs) -> dict[str, Any]:
        value = runner(name, **kwargs) if kwargs else runner(name)
        # Best-effort observability for the neutral TEAM view. The message bus
        # never decides an action; it only mirrors what each independent agent
        # actually attempted/completed.
        try:
            from .team_bus import record_event
            nested = value if isinstance(value, dict) else {}
            # target_color lets peers see which block this robot is working on
            # (role evidence) without exposing simulator truth.
            color = kwargs.get("target_color") or nested.get("target_color")
            record_event(
                os.environ.get("UGRP_ROBOT_ID", "r1"),
                "tool",
                tool=name,
                ok=nested.get("ok") is not False,
                reason=str(nested.get("reason") or "")[:240],
                target_color=str(color).lower() if color else None,
            )
        except Exception:
            pass
        return value

    handler.__name__ = name
    handler.__annotations__ = {"return": dict}
    return handler
