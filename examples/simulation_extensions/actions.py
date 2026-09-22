"""Named actions translate to one validated raw command; no world access."""


def nudge(params):
    if set(params) - {"power", "seconds"}:
        raise ValueError("nudge accepts only power and seconds")
    return {"kind": "drive", "forward": params.get("power", .08),
            "turn": 0, "duration_s": params.get("seconds", .2)}
