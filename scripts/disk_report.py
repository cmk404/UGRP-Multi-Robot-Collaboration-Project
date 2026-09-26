#!/usr/bin/env python3
"""Read-only disk usage report for the UGRP project (docs/disk_management.md).

Measures the primary checkout (outputs/ by folder and file kind, tracked
experiments/, .git), every registered worktree split into tracked checkout /
ignored outputs / caches / other ignored / untracked, virtual environments,
agent state folders, raw-retention tiers of outputs/, and the proposed budget.

Sizes are allocated bytes (st_blocks), hard links counted once. APFS clones are
counted in full, so totals can exceed the physical usage. Nothing is written
except the optional --json file; git runs with --no-optional-locks.
"""

from __future__ import annotations

import argparse
import collections
import datetime as dt
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scripts.agent_worktree import HEAVY_SUFFIXES, infer_owner, is_cache, read_marker  # noqa: E402

GIB = 2**30
SECTIONS = ("fs", "primary", "outputs", "worktrees", "experiments", "runtimes", "agents", "retention", "budget")
# Proposed budget (GiB); the user confirms the numbers (docs/disk_management.md).
BUDGET_GIB = {"primary_outputs": 40.0, "worktrees": 10.0, "primary_other": 6.0, "runtimes": 2.0}
EXPERIMENT_RAW_BUDGET_GIB = 2.0
KINDS = {
    "image": {"jpg", "jpeg", "png", "gif", "webp", "bmp", "tif", "tiff"},
    "video": {"mp4", "mov", "m4v", "avi", "webm", "mkv"},
    "archive": {"zip", "gz", "tgz", "xz", "bz2", "7z", "tar"},
    "json": {"json", "jsonl"},
    "model": {"safetensors", "pt", "pth", "ckpt", "onnx", "bin", "npz", "npy", "mjb"},
}
INFRA = re.compile(r"^(tensorboard.*|agent-locks|models|dispatch-models|model-release.*|simulation-ci|"
                   r"storage-cleanup.*|repository-audit.*|retired-offload.*|\.quickstart.*)$")
DEV_TOKENS = {"dev", "diag", "diagnostic", "pilot", "probe", "smoke", "debug", "sweep", "tune", "trial",
              "check", "preview", "scratch", "tmp", "draft", "sanity", "calib", "calibration"}
TEST_TOKENS = {"test", "final", "holdout", "heldout", "cohort", "qualification", "frozen", "matrix"}


def kind_of(name: str) -> str:
    low = name.lower()
    ext = low.rsplit(".", 1)[-1] if "." in low else ""
    for kind, exts in KINDS.items():
        if ext in exts:
            return kind
    return "other"


class Walker:
    """Allocated-size walker that counts each inode once per report."""

    def __init__(self) -> None:
        self.seen: set[tuple[int, int]] = set()

    def usage(self, path: Path, kinds: collections.Counter | None = None) -> tuple[int, int]:
        files = size = 0
        try:
            st = os.lstat(path)
        except OSError:
            return 0, 0
        if not os.path.isdir(path) or os.path.islink(path):
            return self._count(st, path.name, kinds)
        stack = [str(path)]
        while stack:
            current = stack.pop()
            try:
                entries = list(os.scandir(current))
            except OSError:
                continue
            for entry in entries:
                try:
                    st = entry.stat(follow_symlinks=False)
                except OSError:
                    continue
                if entry.is_dir(follow_symlinks=False):
                    stack.append(entry.path)
                    continue
                f, s = self._count(st, entry.name, kinds)
                files += f
                size += s
        return files, size

    def _count(self, st: os.stat_result, name: str, kinds: collections.Counter | None) -> tuple[int, int]:
        key = (st.st_dev, st.st_ino)
        if key in self.seen:
            return 0, 0
        self.seen.add(key)
        alloc = st.st_blocks * 512
        if kinds is not None:
            kinds[kind_of(name)] += alloc
            kinds[f"{kind_of(name)}#files"] += 1
        return 1, alloc


def git(cwd: Path, *args: str, check: bool = False) -> str:
    result = subprocess.run(["git", "--no-optional-locks", "-C", str(cwd), *args], capture_output=True, text=True)
    if check and result.returncode != 0:
        raise RuntimeError(result.stderr.strip())
    return result.stdout if result.returncode == 0 else ""


def worktree_rows(primary: Path) -> list[dict]:
    rows = []
    for block in git(primary, "worktree", "list", "--porcelain", check=True).strip().split("\n\n"):
        row: dict = {"branch": None, "detached": False}
        for line in block.splitlines():
            key, _, value = line.partition(" ")
            if key == "worktree":
                row["path"] = Path(value)
            elif key == "HEAD":
                row["head"] = value
            elif key == "branch":
                row["branch"] = value.removeprefix("refs/heads/")
            elif key == "detached":
                row["detached"] = True
        if "path" in row:
            rows.append(row)
    return rows


def status_entries(path: Path) -> list[tuple[str, str]]:
    raw = git(path, "status", "--porcelain=v1", "-z", "--ignored=traditional", "--untracked-files=normal")
    items, out, index = raw.split("\0"), [], 0
    while index < len(items):
        item = items[index]
        index += 1
        if item:
            if item[0] in "RC":
                index += 1
            out.append((item[:2], item[3:]))
    return out


def section_fs(primary: Path) -> dict:
    usage = shutil.disk_usage(primary)
    return {"path": str(primary), "total": usage.total, "used": usage.used, "free": usage.free}


def section_primary(primary: Path, walker: Walker) -> dict:
    entries = {}
    for entry in sorted(primary.iterdir()):
        if entry.name == "outputs":
            continue
        if entry.is_symlink():
            entries[entry.name] = {"symlink_to": os.readlink(entry), "bytes": 0}
            continue
        files, size = walker.usage(entry)
        entries[entry.name] = {"files": files, "bytes": size}
    return {"entries": entries, "bytes": sum(e["bytes"] for e in entries.values())}


def section_outputs(primary: Path, walker: Walker, top: int) -> dict:
    outputs = primary / "outputs"
    kinds: collections.Counter = collections.Counter()
    children = []
    if outputs.is_dir():
        for entry in outputs.iterdir():
            files, size = walker.usage(entry, kinds)
            children.append({"name": entry.name, "files": files, "bytes": size})
    children.sort(key=lambda row: -row["bytes"])
    return {"bytes": sum(c["bytes"] for c in children), "entries": len(children), "top": children[:top],
            "all": children, "by_kind": {k: v for k, v in kinds.items() if not k.endswith("#files")},
            "files_by_kind": {k[:-6]: v for k, v in kinds.items() if k.endswith("#files")}}


def pr_states(primary: Path) -> dict[str, list[dict]] | None:
    """PRs by head branch via gh (read-only); None when gh is unavailable."""
    if not shutil.which("gh"):
        return None
    try:
        result = subprocess.run(["gh", "pr", "list", "--state", "all", "--limit", "500", "--json",
                                 "number,headRefName,state,headRefOid"], cwd=primary, capture_output=True,
                                text=True, timeout=60)
    except (OSError, subprocess.TimeoutExpired):
        return None
    if result.returncode != 0:
        return None
    states: dict[str, list[dict]] = collections.defaultdict(list)
    for item in json.loads(result.stdout or "[]"):
        states[item["headRefName"]].append(item)
    return states


def section_worktrees(primary: Path, walker: Walker, base: str, prs: dict | None) -> dict:
    rows = []
    for row in worktree_rows(primary):
        path = row["path"]
        if path.resolve() == primary.resolve():
            continue
        record = {"path": str(path), "branch": row["branch"], "head": row.get("head"), "exists": path.is_dir()}
        if not path.is_dir():
            rows.append(record)
            continue
        row["marker"] = read_marker(path)
        record["owner"], record["owner_source"] = infer_owner(row)
        record["merged"] = subprocess.run(["git", "-C", str(primary), "merge-base", "--is-ancestor",
                                           row.get("head", ""), base], capture_output=True).returncode == 0
        if prs is not None:
            items = prs.get(row["branch"] or "", [])
            record["prs"] = [f"#{item['number']}:{item['state']}" for item in items]
            record["open_pr"] = any(item["state"] == "OPEN" for item in items)
            record["retire_candidate"] = record["merged"] and not record["open_pr"]
        else:
            record["retire_candidate"] = None  # PR state unknown
        record["sparse"] = git(path, "config", "--get", "core.sparseCheckout").strip().lower() == "true"
        split = collections.Counter()
        for code, rel in status_entries(path):
            _, size = walker.usage(path / rel.rstrip("/"))
            if code == "??":
                split["untracked"] += size
            elif rel.startswith("outputs"):
                split["outputs"] += size
            elif is_cache(rel):
                split["caches"] += size
            else:
                split["other_ignored"] += size
        _, rest = walker.usage(path)  # entries above are already counted once
        record.update({"tracked": rest, **{k: split.get(k, 0) for k in
                                            ("outputs", "caches", "other_ignored", "untracked")}})
        record["bytes"] = rest + sum(split.values())
        rows.append(record)
    groups: dict[str, collections.Counter] = collections.defaultdict(collections.Counter)
    for record in rows:
        if not record.get("exists"):
            continue
        for key in (f"owner:{record['owner']}", f"parent:{parent_group(record['path'])}", "all"):
            group = groups[key]
            group["count"] += 1
            group["merged"] += int(record["merged"])
            group["sparse"] += int(record["sparse"])
            for field in ("bytes", "tracked", "outputs", "caches", "other_ignored", "untracked"):
                group[field] += record[field]
    return {"base": base, "rows": rows, "groups": {k: dict(v) for k, v in groups.items()}}


def parent_group(path: str) -> str:
    if "/.codex/worktrees/" in path:
        return "~/.codex/worktrees"
    return str(Path(path).parent)


def section_experiments(primary: Path, ref: str, top: int) -> dict:
    by_ext: collections.Counter = collections.Counter()
    count: collections.Counter = collections.Counter()
    by_dir: collections.Counter = collections.Counter()
    files = []
    for record in git(primary, "ls-tree", "-r", "-l", "-z", ref, "--", "experiments").split("\0"):
        if not record:
            continue
        meta, path = record.split("\t", 1)
        size = int(meta.split()[3]) if meta.split()[3].isdigit() else 0
        ext = path.rsplit(".", 1)[-1].lower() if "." in path.rsplit("/", 1)[-1] else "(none)"
        by_ext[ext] += size
        count[ext] += 1
        by_dir[path.split("/")[1] if path.count("/") >= 2 else "(top)"] += size
        files.append((size, path))
    files.sort(reverse=True)
    heavy = sum(size for ext, size in by_ext.items() if ext in HEAVY_SUFFIXES)
    return {"ref": ref, "bytes": sum(by_ext.values()), "heavy_media_bytes": heavy,
            "by_ext": [{"ext": e, "bytes": b, "files": count[e]} for e, b in by_ext.most_common()],
            "largest_files": [{"path": p, "bytes": s} for s, p in files[:top]],
            "largest_dirs": [{"dir": d, "bytes": b} for d, b in by_dir.most_common(top)]}


def section_runtimes(primary: Path, walker: Walker) -> dict:
    """Environment folders outside the checkout that the primary's .venv* links point to."""
    roots: dict[str, int] = {}
    for entry in sorted(primary.iterdir()):
        if entry.name.startswith(".venv") and entry.is_symlink():
            target = entry.resolve()
            parent = target.parent
            if target.is_dir() and not str(target).startswith(str(primary)) and str(parent) not in roots:
                roots[str(parent)] = walker.usage(parent)[1]
    return {"roots": roots, "bytes": sum(roots.values())}


def section_agents(home: Path, walker: Walker) -> dict:
    rows = {}
    for name in (".codex", ".kiro", ".claude"):
        folder = home / name
        if not folder.is_dir():
            continue
        parts = {}
        for entry in folder.iterdir():
            if name == ".codex" and entry.name == "worktrees":
                continue  # measured with the worktrees
            parts[entry.name] = walker.usage(entry)[1]
        rows[name] = {"bytes": sum(parts.values()),
                      "largest": sorted(parts.items(), key=lambda kv: -kv[1])[:5]}
    return rows


def tokens(name: str) -> set[str]:
    return {t for t in re.split(r"[-_.]+", name.lower()) if t}


def tier_of(name: str) -> str | None:
    if name == "retired-worktrees":
        return "retired-worktrees"
    if INFRA.match(name):
        return "infrastructure"
    parts = tokens(name)
    if "archive" in name.lower() or "archives" in parts:
        return "archive"
    if parts & DEV_TOKENS:
        return "dev-diagnostic"
    if parts & TEST_TOKENS:
        return "test-cohort"
    return None


def record_references(primary: Path, ref: str) -> set[str]:
    out = subprocess.run(["git", "-C", str(primary), "grep", "-h", "-o", "-I", "-E",
                          r"outputs/[A-Za-z0-9._-]+(/[A-Za-z0-9._-]+)?", ref, "--",
                          "experiments", "docs", "configs"], capture_output=True, text=True).stdout
    refs = set()
    for line in out.splitlines():
        parts = line.split("outputs/", 1)[-1].split("/")
        refs.add(parts[0])
        if len(parts) > 1:
            refs.add("/".join(parts[:2]))
    return refs


def age_days(folder: Path, name: str, now: dt.datetime) -> float | None:
    """Age from a YYYYMMDD token in the name, else from the folder mtime."""
    match = re.search(r"(20\d{2})(\d{2})(\d{2})", name)
    try:
        if match:
            day = dt.datetime(int(match.group(1)), int(match.group(2)), int(match.group(3)), tzinfo=now.tzinfo)
        else:
            day = dt.datetime.fromtimestamp(folder.stat().st_mtime, tz=now.tzinfo)
    except (ValueError, OSError):
        return None
    return round((now - day).total_seconds() / 86400, 1)


def section_retention(primary: Path, outputs: dict, ref: str) -> dict:
    """Classify outputs/ folders by name into raw-retention tiers (heuristic, see docs)."""
    refs = record_references(primary, ref)
    now = dt.datetime.now().astimezone()
    rows = []
    for child in outputs["all"]:
        name = child["name"]
        tier = tier_of(name)
        folder = primary / "outputs" / name
        age = age_days(folder, name, now)
        if tier is None and folder.is_dir() and not folder.is_symlink():
            sub = []
            for entry in folder.iterdir():
                sub_tier = tier_of(entry.name)
                if sub_tier in ("dev-diagnostic", "test-cohort"):
                    # outputs/ was already walked once; measure the subfolder with a fresh walker.
                    sub.append((entry.name, sub_tier, Walker().usage(entry)[1]))
            classified = sum(size for _, _, size in sub)
            for sub_name, sub_tier, size in sub:
                rows.append({"path": f"{name}/{sub_name}", "tier": sub_tier, "bytes": size, "age_days": age,
                             "record_reference": name in refs or f"{name}/{sub_name}" in refs})
            rest = child["bytes"] - classified
            if rest > 0:
                rows.append({"path": name + ("/(rest)" if sub else ""), "tier": "unclassified", "bytes": rest,
                             "age_days": age, "record_reference": name in refs})
            continue
        rows.append({"path": name, "tier": tier or "unclassified", "bytes": child["bytes"], "age_days": age,
                     "record_reference": name in refs})
    totals: collections.Counter = collections.Counter()
    by_age: collections.Counter = collections.Counter()
    for row in rows:
        totals[row["tier"]] += row["bytes"]
        age = row["age_days"]
        by_age["unknown" if age is None else "<=3d" if age <= 3 else "<=7d" if age <= 7 else ">7d"] += row["bytes"]
    over = [c for c in outputs["all"] if c["bytes"] > EXPERIMENT_RAW_BUDGET_GIB * GIB
            and tier_of(c["name"]) not in ("infrastructure", "retired-worktrees")]
    return {"totals": dict(totals), "by_age": dict(by_age), "rows": sorted(rows, key=lambda r: -r["bytes"]),
            "over_experiment_budget": over, "experiment_raw_budget_gib": EXPERIMENT_RAW_BUDGET_GIB}


def section_budget(report: dict) -> dict:
    primary_other = report.get("primary", {}).get("bytes", 0)
    measured = {
        "primary_outputs": report.get("outputs", {}).get("bytes", 0),
        "worktrees": report.get("worktrees", {}).get("groups", {}).get("all", {}).get("bytes", 0),
        "primary_other": primary_other,
        "runtimes": report.get("runtimes", {}).get("bytes", 0),
    }
    rows = {k: {"bytes": v, "budget_bytes": int(BUDGET_GIB[k] * GIB), "over": v > BUDGET_GIB[k] * GIB}
            for k, v in measured.items()}
    total = sum(measured.values())
    return {"rows": rows, "project_bytes": total, "project_budget_bytes": int(sum(BUDGET_GIB.values()) * GIB),
            "agent_state_bytes": sum(v["bytes"] for v in report.get("agents", {}).values())}


def fmt(value: int) -> str:
    return f"{value / GIB:7.2f} GiB"


def print_text(report: dict, top: int) -> None:
    print(f"UGRP disk report {report['generated_at']} (allocated bytes; APFS clones counted in full)")
    if "fs" in report:
        fs = report["fs"]
        print(f"\n[filesystem] free {fmt(fs['free'])} of {fmt(fs['total'])}")
    if "primary" in report:
        print(f"\n[primary checkout, without outputs/] {fmt(report['primary']['bytes'])}")
        for name, row in sorted(report["primary"]["entries"].items(), key=lambda kv: -kv[1]["bytes"])[:8]:
            print(f"  {fmt(row['bytes'])}  {name}")
    if "outputs" in report:
        out = report["outputs"]
        print(f"\n[primary outputs/] {fmt(out['bytes'])} in {out['entries']} entries; by kind: " +
              ", ".join(f"{k} {v / GIB:.2f}" for k, v in sorted(out["by_kind"].items(), key=lambda kv: -kv[1])))
        for row in out["top"][:top]:
            print(f"  {fmt(row['bytes'])}  {row['name']}")
    if "worktrees" in report:
        wt = report["worktrees"]
        print(f"\n[worktrees] (merged = ancestor of {wt['base']})")
        for key, g in sorted(wt["groups"].items()):
            print(f"  {key:40s} n={g['count']:3d} merged={g['merged']:3d} sparse={g['sparse']:3d} "
                  f"total {fmt(g['bytes'])} tracked {fmt(g['tracked'])} outputs {fmt(g['outputs'])} "
                  f"caches {fmt(g['caches'])}")
        candidates = [r for r in wt["rows"] if r.get("merged") and r.get("retire_candidate") is not False]
        if candidates:
            known = all(r.get("retire_candidate") is not None for r in candidates)
            print("  HEAD in base" + (" and no open PR" if known else " (PR state unknown; check before retiring)") +
                  " -> owner may retire with scripts/agent_worktree.py retire <path>:")
            for row in sorted(candidates, key=lambda r: -r["bytes"]):
                line = (f"    {fmt(row['bytes'])} outputs {fmt(row['outputs'])}  {row['path']} "
                        f"{' '.join(row.get('prs', []))}")
                print(line.rstrip())
        held = [r for r in wt["rows"] if r.get("merged") and r.get("retire_candidate") is False]
        if held:
            print(f"  HEAD in base but PR still open (not candidates): {len(held)}")
    if "experiments" in report:
        ex = report["experiments"]
        print(f"\n[tracked experiments/ at {ex['ref']}] {fmt(ex['bytes'])}; heavy media {fmt(ex['heavy_media_bytes'])}")
        for row in ex["by_ext"][:8]:
            print(f"  {fmt(row['bytes'])} {row['files']:5d}  .{row['ext']}")
    if "runtimes" in report:
        print(f"\n[virtual environments] {fmt(report['runtimes']['bytes'])}")
    if "agents" in report:
        for name, row in report["agents"].items():
            print(f"[agent state {name}] {fmt(row['bytes'])}")
    if "retention" in report:
        ret = report["retention"]
        print("\n[outputs/ retention tiers] " + ", ".join(f"{k} {v / GIB:.2f}" for k, v in
                                                     sorted(ret["totals"].items(), key=lambda kv: -kv[1])))
        print("  by age: " + ", ".join(f"{k} {v / GIB:.2f}" for k, v in sorted(ret["by_age"].items())))
        over = ret["over_experiment_budget"]
        print(f"  entries over {ret['experiment_raw_budget_gib']:g} GiB: " +
              (", ".join(f"{r['name']} {r['bytes'] / GIB:.2f}" for r in over[:top]) or "none"))
    if "budget" in report:
        b = report["budget"]
        print(f"\n[budget] project {fmt(b['project_bytes'])} / {fmt(b['project_budget_bytes'])}; "
              f"agent state (outside budget) {fmt(b['agent_state_bytes'])}")
        for key, row in b["rows"].items():
            print(f"  {key:16s} {fmt(row['bytes'])} / {fmt(row['budget_bytes'])}{'  OVER' if row['over'] else ''}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n", 1)[0])
    parser.add_argument("--primary", type=Path, help="primary checkout (default: this repository's)")
    parser.add_argument("--home", type=Path, default=Path.home())
    parser.add_argument("--ref", default="origin/main", help="tree for experiments/ and record references")
    parser.add_argument("--sections", default=",".join(SECTIONS))
    parser.add_argument("--top", type=int, default=15)
    parser.add_argument("--no-pr-status", action="store_true", help="skip the read-only `gh pr list` lookup")
    parser.add_argument("--json", type=Path, help="also write the full report as JSON")
    args = parser.parse_args(argv)
    if args.primary:
        primary = args.primary.resolve()
    else:
        common = git(ROOT, "rev-parse", "--path-format=absolute", "--git-common-dir", check=True).strip()
        primary = Path(common).parent.resolve()
    wanted = [s for s in args.sections.split(",") if s]
    unknown = set(wanted) - set(SECTIONS)
    if unknown:
        parser.error(f"unknown sections: {sorted(unknown)}")
    ref = args.ref if git(primary, "rev-parse", "--verify", "--quiet", args.ref).strip() else "HEAD"
    walker = Walker()
    report: dict = {"schema": "ugrp.disk-report.v1", "generated_at": dt.datetime.now().astimezone().isoformat()}
    if "fs" in wanted:
        report["fs"] = section_fs(primary)
    if "outputs" in wanted or "retention" in wanted or "budget" in wanted:
        report["outputs"] = section_outputs(primary, walker, args.top)
    if "primary" in wanted or "budget" in wanted:
        report["primary"] = section_primary(primary, walker)
    if "worktrees" in wanted or "budget" in wanted:
        prs = None if args.no_pr_status else pr_states(primary)
        report["worktrees"] = section_worktrees(primary, walker, ref, prs)
        report["worktrees"]["pr_status"] = prs is not None
    if "experiments" in wanted:
        report["experiments"] = section_experiments(primary, ref, args.top)
    if "runtimes" in wanted or "budget" in wanted:
        report["runtimes"] = section_runtimes(primary, walker)
    if "agents" in wanted:
        report["agents"] = section_agents(args.home, walker)
    if "retention" in wanted:
        report["retention"] = section_retention(primary, report["outputs"], ref)
    if "budget" in wanted:
        report["budget"] = section_budget(report)
    print_text(report, args.top)
    if args.json:
        slim = dict(report)
        if "outputs" in slim:
            slim["outputs"] = {k: v for k, v in slim["outputs"].items() if k != "all"}
        args.json.write_text(json.dumps(slim, ensure_ascii=False, indent=1, default=str) + "\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
