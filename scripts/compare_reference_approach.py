#!/usr/bin/env python3
"""Train official ACT and existing kernel on identical teacher-case splits."""
from __future__ import annotations

import argparse
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from harness.reference_approach_data import action_chunks, load_teacher, sha256, split_cases


def write(path, value):
    path.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n")


def compare(teacher: Path, out: Path, protocol: dict):
    import numpy as np
    import torch
    from harness.camera_approach_student import fit_approach_model, predict_approach
    from harness.reference_act import IMAGE_KEYS, RGBAct, UPSTREAM_SHA, image_tensor, make_policy
    import lerobot.policies.act.modeling_act as upstream_act

    torch.set_num_threads(protocol["threads"])
    torch.use_deterministic_algorithms(True)
    out.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    source_sha = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    if subprocess.check_output(["git", "status", "--porcelain", "--untracked-files=no"], cwd=ROOT):
        raise ValueError("commit execution code before running comparison")
    trajectories, references, source = load_teacher(teacher)
    train_cases, test_cases = split_cases(source["successful_cases"], protocol["heldout_cases"])
    direct = json.loads(importlib.metadata.distribution("lerobot").read_text("direct_url.json") or "{}")
    if direct.get("vcs_info", {}).get("commit_id") != UPSTREAM_SHA:
        raise ValueError("install the pinned upstream LeRobot commit")
    report = {"source_sha": source_sha, "upstream_sha": UPSTREAM_SHA, "protocol": protocol,
              "upstream_modeling_act_sha256": sha256(Path(upstream_act.__file__)),
              "teacher_source_sha": source["source_sha"], "teacher_root": str(teacher),
              "input_hashes": {n: sha256(teacher / n) for n in
                               ("report.json", "actor_samples.json", "privileged_labels.json")},
              "train_cases": train_cases, "heldout_cases": test_cases,
              "excluded_teacher_cases": source["excluded_cases"], "robots": {}, "complete": False,
              "environment": {"python": sys.version, "platform": platform.platform(),
                              **{n: importlib.metadata.version(n) for n in
                                 ("torch", "torchvision", "lerobot", "numpy", "opencv-python-headless")}},
              "scope": "offline held-out teacher predictions, not physical task success",
              "external_model_calls": 0, "external_model_tokens": 0}
    write(out / "report.json", report)
    for rid in ("r1", "r3"):
        torch.manual_seed(protocol["seed"])
        np.random.seed(protocol["seed"])
        robot_dir = out / rid
        robot_dir.mkdir()
        train_rows, actions, padding = action_chunks(trajectories[rid], train_cases, 4)
        test_rows = [row for case in test_cases for row in trajectories[rid][case]]
        ref_own, ref_top = references[rid]
        anchor = {"sample_id": f"goal-anchor:{rid}", "case_id": "goal-anchor",
                  "own_jpeg": ref_own, "top_jpeg": ref_top, "forward": 0.0, "stop": True}
        # Kernel adds exactly this same reference/label internally.
        act_rows = [*train_rows, anchor]
        actions.append([[0., 1.]] * 4)
        padding.append([False, True, True, True])
        rows_manifest = [{"sample_id": row["sample_id"], "case_id": row["case_id"],
                          "own_sha256": __import__("hashlib").sha256(row["own_jpeg"]).hexdigest(),
                          "top_sha256": __import__("hashlib").sha256(row["top_jpeg"]).hexdigest()}
                         for row in act_rows]
        write(robot_dir / "training-inputs.json", rows_manifest)
        print(json.dumps({"robot": rid, "event": "kernel_training", "samples": len(train_rows)}), flush=True)
        tick = time.monotonic()
        kernel = fit_approach_model(ref_own, ref_top, train_rows, domain_samples=train_rows)
        kernel_s = time.monotonic() - tick
        write(robot_dir / "kernel.json", kernel)
        images = {key: torch.stack([image_tensor(row[field]) for row in act_rows])
                  for key, field in zip(IMAGE_KEYS, ("own_jpeg", "top_jpeg"))}
        target = torch.tensor(actions, dtype=torch.float32)
        padded = torch.tensor(padding, dtype=torch.bool)
        policy = make_policy()
        optimizer = torch.optim.AdamW(policy.parameters(), lr=protocol["learning_rate"], weight_decay=1e-4)
        losses = []
        tick = time.monotonic()
        for step in range(protocol["steps"]):
            indices = torch.randint(len(act_rows), (protocol["batch_size"],))
            batch = {key: tensor[indices] for key, tensor in images.items()}
            batch.update(action=target[indices], action_is_pad=padded[indices])
            policy.train()
            loss, metrics = policy(batch)
            if not torch.isfinite(loss):
                raise ValueError("nonfinite ACT training loss")
            optimizer.zero_grad(set_to_none=True)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(policy.parameters(), 1.0)
            optimizer.step()
            if step % 50 == 0 or step + 1 == protocol["steps"]:
                event = {"step": step + 1, "loss": float(loss.detach()), **metrics,
                         "elapsed_s": time.monotonic() - tick}
                losses.append(event)
                print(json.dumps({"robot": rid, "event": "act_training", **event}), flush=True)
                write(robot_dir / "training-progress.json", losses)
        act_s = time.monotonic() - tick
        actor = RGBAct(policy)
        actor.save(robot_dir / "act")
        restored = RGBAct.load(robot_dir / "act")
        probe = test_rows[0]
        original = actor.predict(probe["own_jpeg"], probe["top_jpeg"])
        reloaded = restored.predict(probe["own_jpeg"], probe["top_jpeg"])
        if original != reloaded:
            raise ValueError("checkpoint read-back changed predictions")
        evaluations = {}
        for name, predict in (("kernel", lambda own, top: predict_approach(kernel, own, top)),
                              ("act", restored.predict)):
            records = []
            for row in test_rows:
                begin = time.monotonic()
                decision = predict(row["own_jpeg"], row["top_jpeg"])
                records.append({"sample_id": row["sample_id"], "case_id": row["case_id"],
                                "target_forward": row["forward"], "target_stop": row["stop"],
                                "decision": decision, "inference_wall_s": time.monotonic() - begin})
            write(robot_dir / f"{name}-heldout.json", records)
            n = len(records)
            moving = sum(not r["target_stop"] for r in records)
            stopped = n - moving
            evaluations[name] = {
                "samples": n, "moving_samples": moving, "stop_samples": stopped,
                "forward_mae": float(np.mean([abs(r["decision"]["forward"] - r["target_forward"]) for r in records])),
                "unsupported_samples": sum(not r["decision"]["ok"] for r in records),
                "false_ready": sum(r["decision"]["ready"] and not r["target_stop"] for r in records),
                "missed_ready": sum(not r["decision"]["ready"] and r["target_stop"] for r in records),
                "mean_inference_wall_s": float(np.mean([r["inference_wall_s"] for r in records])),
            }
        report["robots"][rid] = {"training_samples": len(train_rows), "goal_anchors": 1,
                                  "kernel_training_s": kernel_s, "act_training_s": act_s,
                                  "trainable_parameters": sum(p.numel() for p in policy.parameters()),
                                  "checkpoint_readback_identical": True, "heldout": evaluations}
        write(out / "report.json", report)
    report.update(complete=True, wall_elapsed_s=time.monotonic() - started)
    write(out / 'approach-skill.json', {
        'schema': 'ugrp.rgb_short_approach_skill.v1',
        'runtime_inputs': ['own_rgb', 'fixed_top_rgb'],
        'models': {r: {'path': f'{r}/kernel.json', 'sha256': sha256(out / r / 'kernel.json')}
                   for r in ('r1', 'r3')},
        'scope': 'reference comparison kernel refitted on training cases only',
    })
    report["artifacts"] = {str(p.relative_to(out)): sha256(p) for p in sorted(out.rglob("*"))
                           if p.is_file() and p.name != "report.json"}
    write(out / "report.json", report)
    return report


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--teacher-dir", required=True, type=Path)
    p.add_argument("--protocol", required=True, type=Path)
    p.add_argument("--out-dir", required=True, type=Path)
    args = p.parse_args()
    protocol = json.loads(args.protocol.read_text())
    if any(type(protocol[k]) is not int or protocol[k] < 1 for k in ("threads", "steps", "batch_size")):
        p.error("threads, steps and batch_size must be positive integers")
    compare(args.teacher_dir.resolve(), args.out_dir.resolve(), protocol)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
