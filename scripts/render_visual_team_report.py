"""Render an evidence-linked Korean review page for a visual-team cohort."""

from __future__ import annotations

import argparse
import html
import json
import os
import re
from pathlib import Path
from urllib.parse import quote


PHASE_ORDER = ("approach", "lower", "navigate", "release", "verify_release", "recovery")
PHASE_KO = {
    "approach": "접근", "lower": "집게 하강", "navigate": "목적지 이동",
    "release": "놓기", "verify_release": "배치 확인", "recovery": "복구",
    "attachment_left": "파지 확인: 왼쪽",
    "attachment_right": "파지 확인: 오른쪽",
    "attachment_home": "파지 확인: 중앙 복귀",
}


def _json(path: Path):
    try:
        return json.loads(path.read_text())
    except (OSError, json.JSONDecodeError):
        return None


def _rows(path: Path) -> list[dict]:
    rows = []
    try:
        for line in path.read_text().splitlines():
            try:
                value = json.loads(line)
            except json.JSONDecodeError:
                continue
            if isinstance(value, dict):
                rows.append(value)
    except OSError:
        pass
    return rows


def _manifest_runs(root: Path) -> list[Path]:
    for name in ("cohort-manifest.json", "manifest.json"):
        data = _json(root / name)
        if not isinstance(data, dict) or not isinstance(data.get("runs"), list):
            continue
        result = []
        for item in data["runs"]:
            relative = item.get("path") if isinstance(item, dict) else item
            if isinstance(relative, str):
                result.append(root / relative)
        return result
    return []


def discover_runs(root: Path) -> list[Path]:
    planned = _manifest_runs(root)
    if planned:
        return planned
    markers = ("source-manifest.json", "result.json", "control.jsonl",
               "llm-decisions.jsonl", "motion-1x.mp4")
    return [child for child in sorted(root.iterdir()) if child.is_dir() and (
        any((child / marker).exists() for marker in markers) or (child / "inputs").is_dir()
    )]


def _phase_family(phase: object) -> str | None:
    value = str(phase or "").lower()
    # Check the specific verification state before its "release" suffix.
    for family in ("verify_release", "recovery", "approach", "lower", "navigate", "release"):
        if family in value:
            return family
    return None


def _infer_group(path: Path, result: dict | None, rows: list[dict]) -> str:
    active = result.get("active_robots") if isinstance(result, dict) else None
    if isinstance(active, list):
        return "solo1" if len(active) == 1 else "team3"
    robots = {str(row.get("robot_id")) for row in rows if row.get("robot_id")}
    if len(robots) >= 2 or re.search(r"(?:team|robots?[-_]?3|3robots?)", path.name, re.I):
        return "team3"
    return "solo1"


def _seed(path: Path, result: dict | None):
    if isinstance(result, dict) and isinstance(result.get("seed"), int):
        return result["seed"]
    match = re.search(r"(?:seed[-_]?)?(\d+)", path.name, re.I)
    return int(match.group(1)) if match else None


def _llm_disposition(row: dict) -> str:
    value = row.get("disposition")
    if isinstance(value, str) and value:
        return value
    return "legacy_unverified" if isinstance(row.get("decision"), dict) else "request"


def _response_model(row: dict | None):
    audit = row.get("audit") if isinstance(row, dict) else None
    value = audit.get("response_model") if isinstance(audit, dict) else None
    return value if isinstance(value, str) and value else None


def _audit_counts(rows: list[dict]) -> dict[str, int]:
    """Count only explicit recovery/veto evidence; never infer it from failure."""
    recovery = placement_veto = grip_rechecks = 0
    requests = {row.get("call_id"): row for row in rows if row.get("event") == "llm_request"}
    for row in rows:
        disposition = str(row.get("disposition") or "").lower()
        action = row.get("decision", {}).get("action", {}) if isinstance(row.get("decision"), dict) else {}
        if disposition == "accepted":
            if action.get("kind") == "check_grip": grip_rechecks += 1
            request = requests.get(row.get("call_id"), {})
            if action.get("kind") == "approach" and request.get("skill_state") == "released": recovery += 1
        if disposition.startswith("rejected") and (row.get("fresh_placement") is not None or
                str(row.get("error", "")).startswith(("SAFE_INSIDE_", "CAMERA_INSIDE_"))):
            placement_veto += 1
    return {"recovery": recovery, "placement_veto": placement_veto, "grip_rechecks": grip_rechecks}


def summarize_run(path: Path, planned: dict | None = None, lifecycle: str | None = None) -> dict:
    result = _json(path / "result.json")
    config = _json(path / "run-config.json")
    rows = _rows(path / "control.jsonl")
    llm_rows = _rows(path / "llm-decisions.jsonl")
    message_rows = _rows(path / "messages.jsonl")
    active = result.get("active_robots", []) if isinstance(result, dict) else []
    robot_ids = list(active) if isinstance(active, list) else []
    robot_ids += sorted({str(row["robot_id"]) for row in rows if row.get("robot_id")})
    robot_ids = list(dict.fromkeys(robot_ids))
    decisions = {}
    llm_decisions = {}
    llm_latest_responses = {}
    llm_dispositions = {}
    response_models = {}
    audit_counts = {}
    jumps = {}
    for rid in robot_ids:
        own = [row for row in rows if str(row.get("robot_id")) == rid]
        decisions[rid] = own[-1] if own else None
        own_llm = [row for row in llm_rows if str(row.get("robot_id")) == rid]
        responses = [row for row in own_llm if row.get("event") == "llm_result" or
                     isinstance(row.get("decision"), dict) or row.get("error")]
        accepted = [row for row in responses if _llm_disposition(row) == "accepted"]
        llm_decisions[rid] = accepted[-1] if accepted else None
        llm_latest_responses[rid] = responses[-1] if responses else None
        counts = {}
        for row in responses:
            disposition = _llm_disposition(row)
            counts[disposition] = counts.get(disposition, 0) + 1
        llm_dispositions[rid] = counts
        # A terminal proxy/error row commonly has no provider model. Preserve
        # the newest model value that was actually confirmed by response audit.
        response_models[rid] = next(
            (model for model in map(_response_model, reversed(responses)) if model),
            None,
        )
        audit_counts[rid] = _audit_counts(own_llm)
        found = {}
        for row in own:
            family = _phase_family(row.get("phase"))
            if family and family not in found:
                found[family] = {"time": row.get("time"), "step": row.get("step")}
        jumps[rid] = found
    outcomes = result.get("outcomes", {}) if isinstance(result, dict) else {}
    complete = isinstance(result, dict)
    communication = result.get("communication") if complete else None
    if communication not in {"none", "status", "natural"} and isinstance(config, dict):
        communication = config.get("communication")
    if communication not in {"none", "status", "natural"} and isinstance(planned, dict):
        communication = planned.get("communication")
    if communication not in {"none", "status", "natural"}:
        communication = "none" if not message_rows else "unknown"
    latency_values = []
    for row in llm_rows:
        candidates = [row.get("wall_latency_ms"), row.get("latency_ms")]
        inference = row.get("inference_error")
        if isinstance(inference, dict):
            candidates.append(inference.get("latency_ms"))
        audit = row.get("audit")
        if isinstance(audit, dict):
            candidates.extend((audit.get("wall_latency_ms"), audit.get("latency_ms")))
        latency_values.extend(float(value) for value in candidates
                              if isinstance(value, (int, float)) and value >= 0)
    return {
        "name": path.name,
        "path": str(path),
        "seed": _seed(path, result),
        "group": _infer_group(path, result, rows),
        "complete": complete,
        "lifecycle": "complete" if complete else (lifecycle or "pending"),
        "attempted": complete or lifecycle == "running",
        "success": result.get("success") if complete else None,
        "active_robots": robot_ids,
        "outcomes": outcomes if isinstance(outcomes, dict) else {},
        "elapsed_sim_s": result.get("elapsed_sim_s") if complete else None,
        "peer_penetration_gt2mm_s": result.get("peer_penetration_gt2mm_s") if complete else None,
        "obstacle_penetration_gt2mm_s": result.get("obstacle_penetration_gt2mm_s") if complete else None,
        "concurrent_drive_s": result.get("concurrent_drive_s") if complete else None,
        "concurrent_cargo_motion_s": result.get("concurrent_cargo_motion_s") if complete else None,
        "source_hash": result.get("source_hash") if complete else None,
        "physics": result.get("physics") if complete else None,
        "controller": result.get("controller") if complete else None,
        "model": result.get("model") if complete else None,
        "llm_calls": result.get("llm_calls") if complete else None,
        "messages": result.get("messages") if complete else None,
        "communication_mode": communication,
        "message_rows": message_rows,
        "wall_latency_ms": latency_values,
        "error": result.get("error") if complete else None,
        "video": str(path / "motion-1x.mp4") if (path / "motion-1x.mp4").exists() else None,
        "decisions": decisions,
        "llm_decisions": llm_decisions,
        "llm_latest_responses": llm_latest_responses,
        "llm_dispositions": llm_dispositions,
        "response_models": response_models,
        "audit_counts": audit_counts,
        "phase_jumps": jumps,
    }


def _link(target: str | None, output: Path) -> str | None:
    if not target:
        return None
    relative = os.path.relpath(target, output.parent).replace(os.sep, "/")
    return quote(relative, safe="/.-_")


def _fmt(value, digits=2):
    if isinstance(value, (int, float)):
        return f"{value:.{digits}f}"
    return "—"


def _message_text(content: object) -> str:
    if isinstance(content, dict):
        observed = str(content.get("observed", ""))
        intent = str(content.get("intent", "unknown"))
        request = str(content.get("request", "none"))
        return f"{observed} / 의도 {intent} / 요청 {request}"
    return str(content)


def _message_transcript(run: dict, video_id: str) -> str:
    messages = run["message_rows"]
    if not messages:
        return '<p class="muted">기록된 실제 방송 메시지 0건</p>'
    items = []
    for row in messages:
        sent = row.get("sent_at")
        if not isinstance(sent, (int, float)):
            continue
        sender = html.escape(str(row.get("sender", row.get("robot_id", "—"))))
        content = html.escape(_message_text(row.get("content", "")))
        items.append(f'<li><button onclick="seekVideo(\'{video_id}\',{float(sent):.6f})">{float(sent):.2f}s</button> '
                     f'<strong>{sender}</strong> 방송: {content}</li>')
    return '<ol class="transcript">' + ''.join(items) + '</ol>' if items else '<p class="muted">유효한 전송 기록 0건</p>'


def _run_card(run: dict, output: Path) -> str:
    status = ("미시작" if run["lifecycle"] == "not_attempted" else "진행 중" if not run["complete"]
              else "성공" if run["success"] else "실패")
    status_class = "pending" if not run["complete"] else ("pass" if run["success"] else "fail")
    video = _link(run["video"], output)
    video_id = "video-" + re.sub(r"[^a-zA-Z0-9_-]", "-", run["name"])
    media = (f'<video id="{video_id}" controls preload="metadata" src="{video}"></video>' if video else
             '<div class="missing">1× 실제 렌더 영상 없음</div>')
    robots = []
    for rid in run["active_robots"]:
        decision = run["decisions"].get(rid)
        llm_row = run["llm_decisions"].get(rid)
        latest_llm_row = run["llm_latest_responses"].get(rid)
        image = None
        if isinstance(decision, dict) and isinstance(decision.get("step"), int):
            candidate = Path(run["path"]) / "inputs" / rid / f'{decision["step"]:04d}-wrist.jpg'
            if candidate.exists():
                image = _link(str(candidate), output)
        outcome = run["outcomes"].get(rid, {}) if isinstance(run["outcomes"], dict) else {}
        gates = outcome.get("gates", {}) if isinstance(outcome, dict) else {}
        gate_html = " ".join(
            f'<span class="gate {"ok" if value else "no"}">{html.escape(str(key))}</span>'
            for key, value in gates.items()
        ) or "<span class=muted>gate 없음</span>"
        jumps = run["phase_jumps"].get(rid, {})
        jump_html = " ".join(
            f'<button onclick="const v=document.getElementById(\'{video_id}\');if(v){{v.currentTime={float(jumps[family].get("time") or 0):.6f};v.play()}}">{PHASE_KO[family]} {_fmt(jumps[family].get("time"), 2)}s</button>'
            for family in PHASE_ORDER if family in jumps
        ) or "—"
        thumb = (f'<a href="{image}"><img src="{image}" alt="{rid} latest wrist"></a>' if image else
                 '<div class="thumb missing">결정 JPEG 없음</div>')
        llm_decision = llm_row.get("decision") if isinstance(llm_row, dict) else None
        reason = None
        if isinstance(llm_decision, dict):
            reason = llm_decision.get("private_reason", llm_decision.get("reason"))
        if reason is None and isinstance(llm_row, dict):
            reason = llm_row.get("private_reason")
        latest_disposition = _llm_disposition(latest_llm_row) if isinstance(latest_llm_row, dict) else None
        accepted_id = llm_row.get("decision_id") if isinstance(llm_row, dict) else None
        response_model = run["response_models"].get(rid)
        disposition_counts = run["llm_dispositions"].get(rid, {})
        audit_counts = run["audit_counts"].get(rid, {})
        phase_label = PHASE_KO.get(str(decision.get("phase")), str(decision.get("phase", "—"))) if isinstance(decision,dict) else "—"
        robots.append(f'''<article class="robot"><h4>{html.escape(rid)}</h4>{thumb}
          <p>최근 제어 기록: {_fmt(decision.get("time"),2) if isinstance(decision,dict) else "—"}s · 단계 {html.escape(phase_label)}</p>
          <p class="jumps">최초 단계: {jump_html}</p><p>{gate_html}</p>
          <p><strong>비공개 Gemini 판단 이유:</strong> {html.escape(str(reason)) if reason is not None else "—"}</p>
          <p>최근 적용 판단: {html.escape(str(accepted_id)) if accepted_id else "없음"}<br>
          최근 응답 처리: {html.escape(str(latest_disposition)) if latest_disposition else "—"}<br>
          최근 확인된 실제 응답 모델: {html.escape(str(response_model)) if response_model else "—"}<br>
          응답 처리 집계: {html.escape(json.dumps(disposition_counts,ensure_ascii=False,sort_keys=True))}<br>
          재집기 복구 {audit_counts.get("recovery",0)}회 · 파지 재확인 {audit_counts.get("grip_rechecks",0)}회 · 배치 거부 {audit_counts.get("placement_veto",0)}회</p>
          <p>판정: {html.escape(str(outcome.get("reason","—"))) if isinstance(outcome,dict) else "—"}</p></article>''')
    metrics = (
        f'경과 {_fmt(run["elapsed_sim_s"])}s · peer 침투 {_fmt(run["peer_penetration_gt2mm_s"])}s · '
        f'장벽 침투 {_fmt(run["obstacle_penetration_gt2mm_s"])}s · 동시 주행 {_fmt(run["concurrent_drive_s"])}s · '
        f'동시 cargo 이동 {_fmt(run["concurrent_cargo_motion_s"])}s'
    )
    transcript = _message_transcript(run, video_id)
    return f'''<section class="run {status_class}"><header><div><span class="badge">{status}</span>
      <h3>{html.escape(run["name"])}</h3></div><strong>seed {run["seed"] if run["seed"] is not None else "—"}</strong></header>
      {media}<p class="metrics">통신 모드 <strong>{html.escape(run["communication_mode"])}</strong> · {metrics}</p>
      <details class="messages"><summary>실제 방송 메시지 {len(run["message_rows"])}건</summary>{transcript}</details>
      <div class="robots">{"".join(robots) or '<p class="missing">robot log 없음</p>'}</div>
      <details><summary>실행 및 재현 정보</summary><pre>{html.escape(json.dumps({"controller":run["controller"],"requested_model":run["model"],"actual_response_models":run["response_models"],"llm_calls":run["llm_calls"],"llm_dispositions":run["llm_dispositions"],"audit_counts":run["audit_counts"],"messages":run["messages"],"source_hash":run["source_hash"],"physics":run["physics"],"error":run["error"]},ensure_ascii=False,indent=2))}</pre></details></section>'''


def render(root: Path, output: Path) -> dict:
    manifest = next((_json(root / name) for name in ("cohort-manifest.json", "manifest.json")
                     if isinstance(_json(root / name), dict)), None)
    planned_items = {str(item.get("path")): item for item in manifest.get("runs", [])
                     if isinstance(item, dict) and isinstance(item.get("path"), str)} if manifest else {}
    cohort_status = _json(root / "cohort-status.json")
    blocked = isinstance(cohort_status, dict) and cohort_status.get("status") == "infrastructure_blocked"
    started = set(cohort_status.get("started_runs", [])) if blocked and isinstance(cohort_status.get("started_runs"), list) else set()
    runs = []
    for path in discover_runs(root):
        planned = planned_items.get(path.name)
        has_activity = path.exists() and any((path / marker).exists() for marker in
            ("result.json", "control.jsonl", "llm-decisions.jsonl", "motion-1x.mp4"))
        lifecycle = ("not_attempted" if blocked and path.name not in started and not has_activity
                     else "running" if has_activity else "pending")
        runs.append(summarize_run(path, planned, lifecycle))
    runs.sort(key=lambda run: (run["group"], run["success"] is not True,
                               run["seed"] if run["seed"] is not None else 10**9, run["name"]))
    summary = {
        "cohort_root": str(root),
        "planned_runs": len(runs),
        "finished_runs": sum(run["complete"] for run in runs),
        "successful_runs": sum(run["success"] is True for run in runs),
        "unfinished_runs": sum(not run["complete"] for run in runs),
        "planned_seeds": sorted({run["seed"] for run in runs if run["seed"] is not None}),
        "groups": {group: [run for run in runs if run["group"] == group]
                   for group in ("solo1", "team3")},
    }
    modes = {}
    for run in runs:
        aggregate = modes.setdefault(run["communication_mode"], {"runs": 0, "completed_runs": 0,
            "attempted_runs": 0, "not_attempted_runs": 0, "successful_runs": 0,
            "successful_robot_deliveries": 0, "llm_calls": 0, "broadcasts": 0, "wall_latency_ms": []})
        aggregate["runs"] += 1
        aggregate["completed_runs"] += run["complete"]
        aggregate["attempted_runs"] += run["attempted"]
        aggregate["not_attempted_runs"] += run["lifecycle"] == "not_attempted"
        aggregate["successful_runs"] += run["success"] is True
        aggregate["successful_robot_deliveries"] += sum(
            outcome.get("success") is True for outcome in run["outcomes"].values() if isinstance(outcome, dict))
        if isinstance(run["llm_calls"], dict):
            aggregate["llm_calls"] += sum(value for value in run["llm_calls"].values() if isinstance(value, int))
        elif isinstance(run["llm_calls"], int):
            aggregate["llm_calls"] += run["llm_calls"]
        aggregate["broadcasts"] += len(run["message_rows"])
        aggregate["wall_latency_ms"].extend(run["wall_latency_ms"])
    summary["communication_modes"] = modes
    output.parent.mkdir(parents=True, exist_ok=True)
    (output.parent / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2))
    sections = []
    for group, title in (("solo1", "Solo 1대"), ("team3", "Team 3대")):
        cards = "".join(_run_card(run, output) for run in summary["groups"][group])
        sections.append(f'<h2>{title}</h2>{cards or "<p class=missing>계획된 run 없음</p>"}')
    try:
        supplement = (root / "supplement.html").read_text()
    except OSError:
        supplement = ""
    mode_rows = ''.join(
        f'<tr><td>{html.escape(mode)}</td><td>{item["successful_runs"]}/{item["completed_runs"]}</td>'
        f'<td>{item["successful_robot_deliveries"]}</td><td>{item["llm_calls"]}</td><td>{item["broadcasts"]}</td>'
        f'<td>{_fmt(sum(item["wall_latency_ms"])/len(item["wall_latency_ms"]),1) if item["wall_latency_ms"] else "—"}</td></tr>'
        for mode, item in sorted(modes.items()))
    mode_table = ('<table><thead><tr><th>통신 모드</th><th>성공 run</th><th>성공 배달</th><th>LLM 호출</th>'
                  '<th>실제 방송</th><th>기록된 평균 wall latency (ms)</th></tr></thead><tbody>' + mode_rows + '</tbody></table>')
    page = f'''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Gemini RGB Team 검토</title><style>
body{{margin:auto;max-width:1180px;padding:24px;background:#101318;color:#eef2f7;font:15px system-ui}}h1,h2,h3,h4{{margin:.3em 0}}.notice,.run{{background:#191f27;border:1px solid #303946;border-radius:14px;padding:16px;margin:14px 0}}.run header{{display:flex;justify-content:space-between;align-items:center}}video{{width:100%;max-height:540px;background:#050607;border-radius:10px;margin-top:10px}}.robots{{display:grid;grid-template-columns:repeat(auto-fit,minmax(240px,1fr));gap:10px}}.robot{{background:#11161d;padding:10px;border-radius:10px}}img,.thumb{{width:100%;aspect-ratio:4/3;object-fit:cover;border-radius:8px}}.badge,.gate,.jumps button,.transcript button{{display:inline-block;padding:3px 7px;border:0;border-radius:99px;background:#384250;color:#eef2f7;margin:2px;cursor:pointer}}.pass .badge,.gate.ok{{background:#176b48}}.fail .badge,.gate.no{{background:#8a3038}}.pending .badge{{background:#85651c}}.missing,.muted{{color:#9eabb9;padding:20px;text-align:center}}.metrics{{color:#c5d0dc}}pre{{white-space:pre-wrap;overflow-wrap:anywhere}}table{{width:100%;border-collapse:collapse;margin-top:12px}}th,td{{padding:7px;border-bottom:1px solid #384250;text-align:left}}.transcript li{{margin:6px 0}}
</style><script>function seekVideo(id,t){{const v=document.getElementById(id);if(v){{v.currentTime=t;v.play();}}}}</script><body><h1>Gemini 독립 RGB 운반 검토</h1><div class="notice"><strong>계획 {summary['planned_runs']} · 완료 {summary['finished_runs']} · 성공 {summary['successful_runs']} · 실패 {summary['finished_runs']-summary['successful_runs']} · 미완료 {summary['unfinished_runs']} · 계획 seed {html.escape(', '.join(map(str,summary['planned_seeds'])) or '—')}</strong>
<p>각 로봇은 자기 RGB 카메라 2개(고정 nav, wrist)와 자기 actuator 상태를 사용합니다. controller, 요청 model, 실제 응답 model, LLM 호출 수와 메시지 수는 저장된 result와 LLM audit 값을 구분해 표시합니다. 판단 이유는 accepted 응답만 표시하며 rejected/discarded 응답은 처리 집계에 남깁니다. cargo와 목표 zone은 operator가 지정했습니다. Zone 한 변은 0.82m이고 물리 marker/크기 prior를 사용합니다. 이 결과만으로 communication 효과를 입증하지 않습니다.</p>
<p>성공·실패는 referee의 물리 gate 판정입니다. Gemini 응답 모델과 응답 적용 여부는 별도 audit이며 성공을 뜻하지 않습니다. 비공개 판단 이유와 실제 방송은 구분하며, 방송은 messages.jsonl에 기록된 항목만 표시합니다. 방송 시간 버튼은 해당 영상 시점으로 이동합니다.</p>{mode_table}</div>{supplement}{''.join(sections)}</body></html>'''
    output.write_text(page)
    return summary


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("cohort_root", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    root = args.cohort_root.resolve()
    output = (args.output.resolve() if args.output else root / "index.html")
    summary = render(root, output)
    print(json.dumps({key: summary[key] for key in
                      ("planned_runs", "finished_runs", "successful_runs", "unfinished_runs")},
                     ensure_ascii=False))


if __name__ == "__main__":
    main()
