"""Allowlisted Python tools that a model may call."""

from __future__ import annotations

import inspect
from dataclasses import dataclass, field
from typing import Any, Callable, get_args, get_origin, get_type_hints


ALLOWED_TYPES = {bool, int, float, str}


class ToolError(ValueError):
    """Raised when a tool name or argument is rejected."""


@dataclass(frozen=True)
class Parameter:
    name: str
    type: type
    required: bool
    default: Any = None
    description: str = ""


@dataclass(frozen=True)
class Tool:
    name: str
    description: str
    handler: Callable[..., Any]
    parameters: tuple[Parameter, ...] = ()
    contract: dict[str, Any] = field(default_factory=dict)


@dataclass
class Registry:
    tools: dict[str, Tool] = field(default_factory=dict)
    # Optional cancellation hook owned by the same actions module whose run()
    # handlers are registered below. Keeping this exact module instance matters:
    # a separately imported robot_actions module would not see the live child.
    cancel_current: Callable[[], Any] | None = None
    # Hidden execution fastpath for deterministic public-tool programs. This is
    # deliberately NOT part of ``tools`` / prompt_schema(), so the model still
    # sees the exact same low-level allowlist and safety contracts.
    program_runner: Callable[[list[tuple[str, dict[str, Any]]]], list[dict[str, Any]]] | None = None

    def register(self, tool: Tool) -> Tool:
        if tool.name in self.tools:
            raise ToolError(f"duplicate tool: {tool.name}")
        self.tools[tool.name] = tool
        return tool

    def tool(
        self,
        name: str | None = None,
        *,
        description: str,
    ) -> Callable[[Callable[..., Any]], Callable[..., Any]]:
        def decorator(fn: Callable[..., Any]) -> Callable[..., Any]:
            self.register(
                Tool(
                    name=name or fn.__name__,
                    description=description,
                    handler=fn,
                    parameters=parameters_from_signature(fn),
                )
            )
            return fn

        return decorator

    def get(self, name: str) -> Tool:
        try:
            return self.tools[name]
        except KeyError as exc:
            raise ToolError(f"unknown tool: {name}") from exc

    def names(self) -> list[str]:
        return sorted(self.tools)

    def prompt_schema(self) -> str:
        """Compact but complete planner-facing tool catalog.

        Contracts are immutable definitions, so each is sent once here. Optional
        argument prose is intentionally omitted: type/default plus the handler's
        validator are sufficient, while required-argument descriptions remain
        available when they carry identity/enum semantics.
        """
        lines = []
        for name in self.names():
            tool = self.tools[name]
            parts = [f"- {tool.name}: {tool.description}"]
            if tool.contract:
                pre = tool.contract.get("preconditions") or []
                post = tool.contract.get("expected_postconditions") or []
                if pre:
                    parts.append("pre=" + ",".join(map(str, pre)))
                if post:
                    parts.append("post=" + ",".join(map(str, post)))
            if not tool.parameters:
                parts.append("args={}")
            else:
                args = []
                for param in tool.parameters:
                    if param.required:
                        token = f"{param.name}:{param.type.__name__}!"
                        if param.description:
                            token += f"({param.description})"
                    else:
                        token = f"{param.name}:{param.type.__name__}={param.default!r}"
                        if param.description:
                            # Keep only the first clause carrying the useful
                            # enum/range; drop explanatory prose duplicated by
                            # the tool description/validator.
                            brief = param.description.split(". ", 1)[0].rstrip(".")
                            if brief:
                                token += f"({brief})"
                    args.append(token)
                parts.append("args=" + ",".join(args))
            lines.append(" | ".join(parts))
        return "\n".join(lines)


def parameters_from_signature(fn: Callable[..., Any]) -> tuple[Parameter, ...]:
    signature = inspect.signature(fn)
    hints = get_type_hints(fn)
    params: list[Parameter] = []
    for name, param in signature.parameters.items():
        if name in {"self", "cls"}:
            continue
        if param.kind in {inspect.Parameter.VAR_POSITIONAL, inspect.Parameter.VAR_KEYWORD}:
            raise ToolError(f"{fn.__name__} may not use *args/**kwargs")
        annotation = hints.get(name, param.annotation)
        py_type = _unwrap_type(annotation)
        if py_type not in ALLOWED_TYPES:
            raise ToolError(f"{fn.__name__}.{name} must be bool|int|float|str")
        required = param.default is inspect.Parameter.empty
        params.append(
            Parameter(
                name=name,
                type=py_type,
                required=required,
                default=None if required else param.default,
            )
        )
    return tuple(params)


def _unwrap_type(annotation: Any) -> type | None:
    if annotation is inspect.Parameter.empty:
        return None
    origin = get_origin(annotation)
    if origin is None:
        return annotation if isinstance(annotation, type) else None
    args = [arg for arg in get_args(annotation) if arg is not type(None)]
    if len(args) == 1 and isinstance(args[0], type):
        return args[0]
    return None


def validate_args(tool: Tool, raw: dict[str, Any] | None) -> dict[str, Any]:
    incoming = {} if raw is None else raw
    if not isinstance(incoming, dict):
        raise ToolError(f"{tool.name}: args must be an object")
    unknown = sorted(set(incoming) - {param.name for param in tool.parameters})
    if unknown:
        raise ToolError(f"{tool.name}: unexpected args: {', '.join(unknown)}")
    validated: dict[str, Any] = {}
    for param in tool.parameters:
        if param.name not in incoming:
            if param.required:
                raise ToolError(f"{tool.name}: missing arg {param.name}")
            validated[param.name] = param.default
            continue
        value = incoming[param.name]
        if param.type is bool and isinstance(value, bool):
            validated[param.name] = value
        elif param.type is int and isinstance(value, int) and not isinstance(value, bool):
            validated[param.name] = value
        elif param.type is float and isinstance(value, (int, float)) and not isinstance(value, bool):
            validated[param.name] = float(value)
        elif param.type is str and isinstance(value, str):
            validated[param.name] = value
        else:
            raise ToolError(f"{tool.name}: {param.name} must be {param.type.__name__}")
    return validated
