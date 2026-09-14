"""Dispatch fixed image-only student artifacts without simulator dependencies."""
from __future__ import annotations

from typing import Any

from harness.camera_teacher_student import SCHEMA, predict_correction


def predict_student(model: dict[str, Any], own_jpeg: bytes, top_jpeg: bytes,
                    max_step: int = 25) -> dict[str, Any]:
    if model.get("schema") == SCHEMA:
        return predict_correction(model, own_jpeg, top_jpeg, max_step=max_step)
    from harness.camera_recovery_student import predict_recovery
    return predict_recovery(model, own_jpeg, top_jpeg, max_step=max_step)
