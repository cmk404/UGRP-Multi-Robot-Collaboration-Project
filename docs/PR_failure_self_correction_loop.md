# PR Proposal: Add Failure Diagnosis Memory and Self-Correction Loop to the Embodied Agent

**Status:** Proposed  
**Date:** 2026-08-30  
**Scope:** `harness/` runtime agent architecture  
**Primary goal:** Make repeated manipulation failures diagnosable, comparable across attempts, and actionable by the planner instead of collapsing into a shallow `failure_code -> stop` flow.

---

## 1. Summary

The current UGRP agent is strong at deterministic execution safety, but weak at recognizing *why* it is failing across repeated physical attempts.

For common manipulation goals, the runtime often behaves as a deterministic executive rather than a reflective agent:

```text
camera/state
   ↓
TaskExecutive
   ↓
search → track → approach → pick → carry
                         ↓
                       failure
                         ↓
                failure_code only
                         ↓
                        stop
```

This PR proposes a lightweight, explicit **Failure Memory + Failure Diagnosis + Recovery Planning** layer that preserves deterministic safety checks while giving the planner enough structured history to detect repeated failure patterns and change strategy.

Target loop:

```text
Observe
  ↓
Execute
  ↓
Verify
  ↓
Diagnose failure
  ↓
Compare with prior attempts
  ↓
Generate ranked hypotheses
  ↓
Choose bounded recovery
  ↓
Retry
  ↓
Compare outcome and update belief
```

This is intentionally **not** a proposal to remove `TaskExecutive` or weaken safety preconditions. The deterministic executive should remain the authority for whether an action is allowed. The new layer should operate above it to decide *what to try next and why*.

---

## 2. Problem Statement

### 2.1 The planner often never gets a chance to reason about manipulation failure

In `harness/loop.py`, when structured execution is active, `TaskExecutive.plan_calls()` precompiles a tool queue. While `structured_queue` is non-empty, the loop executes the queued tool directly instead of asking the LLM for a new decision.

Relevant behavior:

- `UGRP_STRUCTURED_RED_FASTPATH` is a legacy deterministic baseline/debug switch and defaults to disabled; set it to `1` explicitly when reproducing that baseline.
- Known goals can become sequences such as:
  - `search`
  - `track`
  - `approach`
  - `pick`
  - `carry`
- The LLM is bypassed while that queue is active.

As a result, the system may look like an agent externally while its critical manipulation path is effectively a state machine.

### 2.2 Runtime memory is too short to detect repeated failure patterns

In `harness/loop.py`, `auto_observe` mode truncates the conversational trace and exposes only a small recent tail to the model.

Conceptually:

```python
recent = messages[mission_index + 1:][-3:]
```

The explicit `world_state` passed to the planner also exposes only the **current/last action failure**, not a history of attempts.

Current public planner state includes fields such as:

```text
last_action.name
last_action.failure_code
last_action.required_state
last_action.recommended_recovery
```

This means the planner can know:

> "pick failed"

but cannot reliably know:

> "pick failed three times in the same geometric direction, after two retracks, with no change in outcome"

That distinction is necessary for meaningful self-correction.

### 2.3 Failure representation is too shallow

`harness/state.py` currently models action failure mainly with:

```text
failure_code
required_state
recommended_recovery
```

This is sufficient for precondition gating, but insufficient for diagnosis.

For example, `GRASP_NOT_ACQUIRED` does not encode whether:

- the target was laterally offset from the gripper center,
- one finger contacted before the other,
- the object was pushed instead of enclosed,
- the block rotated during approach,
- the gripper closed above or beside the target,
- the approach depth was too shallow/deep,
- a previous recovery already tried retracking and failed,
- the same error pattern repeated over multiple attempts.

Without this information, the agent cannot distinguish geometry, perception, control, grasp, or sensing failure modes.

### 2.4 The current structured path explicitly terminates on failed grasp acquisition

In `harness/loop.py`, `GRASP_NOT_ACQUIRED` on `pick`/`carry` can clear the structured queue and mark the attempt failed.

The existing safety motivation is valid: a destructive grasp can move or rotate the object, so blindly restarting the whole acquisition pipeline may be unsafe and may invalidate experimental initial conditions.

However, the practical consequence is that the agent does not currently have a bounded self-correction phase in the same user turn.

The correct fix is **not** to remove the stop rule blindly. The fix is to insert an explicit diagnostic/recovery layer that can distinguish:

1. failures where autonomous retry is unsafe and must stop,
2. failures where read-only diagnosis is safe,
3. failures where a small bounded recovery is safe,
4. failures that require an episode reset or operator confirmation.

---

## 3. Design Goals

This PR should satisfy the following goals.

### G1. Preserve deterministic safety

`TaskExecutive.check()` remains authoritative for action preconditions.

The LLM must never be allowed to bypass hard safety state such as:

- object already carried,
- empty gripper when placement is requested,
- missing pregrasp handoff,
- unconfirmed target visibility for destructive pick,
- identity mismatch between held object and requested object.

### G2. Make repeated failures explicit state

The runtime must preserve enough attempt history for the planner to answer:

- How many times has this skill failed?
- Is it the same failure code?
- Did the scene change?
- Did the chosen recovery change anything?
- Is the error directional/systematic?
- Has this recovery already been tried?

### G3. Separate evidence from hypotheses

Observed facts and inferred causes must be separate.

Example:

```json
{
  "observed": {
    "target_offset_x": -0.043,
    "object_motion": "pushed_left"
  },
  "hypotheses": [
    {
      "cause": "grasp_center_bias",
      "confidence": 0.82
    }
  ]
}
```

The system must not promote a hypothesis into sensor truth.

### G4. Prefer bounded recovery over blind repetition

The system should explicitly reject repeated identical actions that have already failed without meaningful state change.

### G5. Remain useful in both SIM and REAL

The schema should support richer SIM evidence where available, but must not depend on simulator-only ground truth.

REAL mode may initially populate fewer diagnostic fields and still benefit from attempt history.

---

## 4. Proposed Architecture

### 4.1 Add `FailureMemory` to `WorldState`

Add a structured history in `harness/state.py`.

Suggested data model:

```python
@dataclass
class FailureObservation:
    target_offset_x: float | None = None
    target_offset_y: float | None = None
    target_visible: bool | None = None
    target_centered: bool | None = None
    target_range_class: str | None = None
    object_motion: str | None = None
    left_contact: bool | None = None
    right_contact: bool | None = None
    grasp_verified: bool | None = None
    visual_hold: bool | None = None


@dataclass
class FailureHypothesis:
    cause: str
    confidence: float
    evidence: list[str] = field(default_factory=list)


@dataclass
class FailureAttempt:
    attempt_id: int
    skill: str
    failure_code: str | None
    reason: str | None
    observation: FailureObservation
    recovery_used: str | None = None
    state_changed: bool | None = None


@dataclass
class FailureMemory:
    attempts: list[FailureAttempt] = field(default_factory=list)
    repeated_failure_count: int = 0
    dominant_failure_code: str | None = None
    hypotheses: list[FailureHypothesis] = field(default_factory=list)
    previous_recoveries: list[str] = field(default_factory=list)
    recommended_recovery: str | None = None
```

Then add:

```python
failure_memory: FailureMemory = field(default_factory=FailureMemory)
```

to `WorldState`.

The exact schema can be simplified during implementation. The important requirement is to preserve **attempt-level history**, not only the last failure.

---

### 4.2 Add a `FailureAnalyzer`

Add a new module, preferably:

```text
harness/failure_analysis.py
```

Responsibilities:

1. Consume post-action evidence and `WorldState`.
2. Produce normalized failure observations.
3. Compare the new attempt against prior attempts.
4. Maintain repeated failure counts.
5. Produce ranked hypotheses using deterministic rules first.
6. Recommend one of a small set of bounded recovery classes.

Initial implementation should be mostly deterministic.

Example rule set:

```text
IF failure_code == GRASP_NOT_ACQUIRED
AND same code occurred >= 2 times
AND lateral target offset has same sign
THEN hypothesis += grasp_center_bias

IF failure_code == GRASP_NOT_ACQUIRED
AND object_motion == pushed_left
AND right_contact == true
AND left_contact == false
THEN hypothesis += right_side_early_contact

IF track succeeds repeatedly
AND approach succeeds
AND pick fails with stable target geometry
THEN reduce confidence in perception failure
AND increase confidence in grasp geometry/calibration failure

IF the same recovery has already been tried without state change
THEN do not recommend it again
```

The first version does not need perfect physical inference. It needs to be more informative than a single flat failure code.

---

### 4.3 Expose compact failure state to the planner

Extend `_planner_state_public()` in `harness/loop.py`.

Do **not** dump the entire trace.

Expose a compact summary such as:

```json
{
  "failure_memory": {
    "attempts": 3,
    "same_failure_count": 3,
    "dominant_failure_code": "GRASP_NOT_ACQUIRED",
    "last_two": [
      {
        "skill": "pick",
        "failure_code": "GRASP_NOT_ACQUIRED",
        "recovery_used": "track",
        "state_changed": false
      }
    ],
    "hypotheses": [
      {
        "cause": "grasp_center_bias",
        "confidence": 0.82
      }
    ],
    "previous_recoveries": ["track", "approach"],
    "recommended_recovery": "lateral_realign"
  }
}
```

The planner should be able to see accumulated failure structure even though conversational history remains truncated.

---

### 4.4 Add an explicit diagnostic stage after a failed destructive action

Do not immediately reduce every `GRASP_NOT_ACQUIRED` to `structured_failed=True`.

Replace the current binary handling with a bounded state machine:

```text
PICK_FAILED
   ↓
DIAGNOSE
   ↓
classify retry safety
   ├─ unsafe → STOP
   ├─ needs reset → REQUEST/RESET EPISODE
   ├─ read-only diagnosis only → OBSERVE/ANALYZE
   └─ bounded recovery safe → RECOVER → RETRY
```

Suggested policy enum:

```python
class RecoverySafety(Enum):
    STOP = "STOP"
    OBSERVE_ONLY = "OBSERVE_ONLY"
    BOUNDED_RECOVERY = "BOUNDED_RECOVERY"
    RESET_REQUIRED = "RESET_REQUIRED"
```

This preserves the original rationale for avoiding destructive blind retries.

---

### 4.5 Distinguish retry from repetition

A retry should only happen if at least one of these is true:

- the planned recovery differs from the previous one,
- target geometry changed meaningfully,
- confidence in a diagnosis increased based on new evidence,
- an episode reset restored the initial condition,
- the executive explicitly authorizes the new bounded recovery.

The loop should stop or escalate if:

- the same failure repeats N times,
- the same recovery was tried without measurable change,
- no new evidence is available,
- the only remaining action is to repeat the same destructive primitive.

Recommended initial bound:

```text
max autonomous destructive retries per user turn: 2
max identical recovery attempts: 1
max diagnosis observations after a failed grasp: 2
```

These values should be constants/configurable, not hardcoded throughout the loop.

---

## 5. Planner Prompt Changes

The current prompt already contains the useful instruction:

> Never blindly repeat a tool when the fresh scene shows no progress; diagnose geometry or choose another relevant tool.

The architecture currently does not give the model enough structured memory to follow that instruction reliably.

After `failure_memory` is implemented, extend the prompt with rules such as:

```text
When failure_memory.same_failure_count >= 2, explicitly compare the last attempts before selecting another action.
Do not repeat a recovery listed in previous_recoveries unless the world state changed materially.
Treat hypotheses as uncertain explanations, not sensor facts.
Prefer the recommended bounded recovery when it is consistent with current sensor state and executive preconditions.
If no safe recovery exists, report the diagnosed failure instead of repeating motion.
```

The prompt change should come **after** state support exists. Prompt-only changes are not sufficient.

---

## 6. Proposed File Changes

### Required

#### `harness/state.py`

- Add failure-memory dataclasses.
- Add `failure_memory` to `WorldState`.
- Add methods to record and reset failure attempts.
- Ensure `StateEstimator.reset()` clears failure history at a true episode boundary.
- Decide whether user-turn boundaries preserve or reset failure history.

Recommended default:

- preserve within a user turn,
- reset on explicit simulation episode reset,
- optionally preserve a compressed summary across consecutive follow-up commands in the same physical scene.

#### `harness/failure_analysis.py` (new)

- Normalize failure evidence.
- Compare attempts.
- Generate deterministic hypotheses.
- Recommend recovery class/action.

#### `harness/loop.py`

- Invoke `FailureAnalyzer` after structured tool failure.
- Update planner-visible state.
- Replace unconditional structured termination on `GRASP_NOT_ACQUIRED` with bounded diagnostic policy.
- Prevent identical blind retries.
- Keep existing safety gate behavior intact.

#### `harness/protocol.py`

No required protocol extension for V1 if recovery is expressed using existing allowlisted tools.

If future diagnostic actions need a first-class representation, add that separately rather than overloading `look`.

### Likely

#### `harness/executive.py`

Add a method that classifies whether a recovery is safe after a destructive failure.

Possible interface:

```python
def recovery_policy(
    self,
    failed_skill: str,
    state: WorldState,
    failure_memory: FailureMemory,
) -> RecoveryDecision:
    ...
```

The executive, not the LLM, should remain the final authority on whether a proposed recovery can execute.

#### `harness/execution_trace.py`

Persist diagnosis metadata so offline analysis can verify whether the agent actually adapted.

#### `harness/trace_analysis.py`

Add metrics for:

- repeated failure count,
- recovery diversity,
- state change after recovery,
- blind-repeat rate,
- successful recovery rate,
- dominant failure hypotheses.

---

## 7. Example Desired Runtime Behavior

### Current behavior

```text
User: 빨간 블럭 집어봐

search → track → approach → pick
pick: GRASP_NOT_ACQUIRED
STOP
```

A repeated user request may run the same sequence again without understanding the previous miss.

### Desired behavior

```text
Attempt 1
pick → GRASP_NOT_ACQUIRED
Observation: target left of gripper center after close
Recovery: retrack

Attempt 2
pick → GRASP_NOT_ACQUIRED
Observation: same left-offset pattern
FailureMemory: same_failure_count=2
Hypothesis: systematic grasp-center bias (0.74)
Recovery: bounded lateral realignment

Attempt 3
pick → ACHIEVED
FailureMemory marks recovery as effective
```

If the object is clearly displaced into an unsafe or unknown geometry:

```text
pick → GRASP_NOT_ACQUIRED
object moved significantly
reliable reacquisition unavailable
RecoverySafety=STOP

Final:
"집기를 시도했지만 물체가 밀려 초기 기하가 바뀌었어. 같은 동작을 반복하면 오히려 상태를 더 망가뜨릴 수 있어서 중단했어. 현재 관측상 파지 중심 오차 가능성이 가장 높아."
```

This is preferable to either blind retry or an unexplained stop.

---

## 8. Acceptance Criteria

The PR is complete when all of the following are true.

### State and memory

- [ ] `WorldState` stores more than the last failure.
- [ ] At least the last 3 failure attempts can be summarized to the planner.
- [ ] Failure memory resets correctly on true episode reset.
- [ ] Observed evidence is distinct from inferred hypotheses.

### Runtime behavior

- [ ] Two identical failed picks are detectable as a repeated pattern.
- [ ] The same failed recovery is not blindly repeated when state did not change.
- [ ] `GRASP_NOT_ACQUIRED` no longer always collapses immediately to a terminal stop; it first passes through recovery-policy classification.
- [ ] Unsafe destructive retry remains blocked.
- [ ] Deterministic executive preconditions cannot be bypassed by the planner.

### Planner cognition

- [ ] `_planner_state_public()` includes a compact failure summary.
- [ ] The planner can name the difference between first failure and repeated systematic failure.
- [ ] The planner can see which recoveries were already tried.

### Traceability

- [ ] Execution traces record diagnosis and selected recovery.
- [ ] Offline analysis can determine whether the recovery changed state.

### Tests

Add tests for at least:

1. repeated `GRASP_NOT_ACQUIRED` increments the same-failure counter,
2. different failure codes do not incorrectly merge,
3. simulator episode reset clears failure memory,
4. identical recovery without state change is rejected,
5. bounded recovery can be proposed after a safe diagnosable miss,
6. unsafe changed geometry still terminates,
7. executive safety preconditions remain unchanged,
8. planner public state does not expose simulator-only ground truth.

---

## 9. Suggested Initial Test Scenario

Use a deterministic SIM case where grasp center is deliberately biased.

Example experiment:

1. Start with a red block in a known reachable pose.
2. Add a fixed lateral pick offset large enough to cause acquisition failure.
3. Run the task.
4. Confirm first failure is recorded.
5. Repeat after a permitted recovery.
6. Confirm the same directional failure is recognized.
7. Confirm hypothesis confidence rises for systematic center bias.
8. Apply a bounded opposite correction.
9. Confirm either:
   - grasp succeeds, or
   - diagnosis updates based on new evidence instead of repeating the same explanation blindly.

This test is more valuable than testing only that a `failure_code` is emitted.

---

## 10. Non-Goals

This PR should **not** attempt to solve all manipulation intelligence at once.

Out of scope for V1:

- unrestricted autonomous trial-and-error,
- online reinforcement learning during user interaction,
- changing actuator calibration automatically without bounds,
- using simulator ground truth in the real planner,
- removing deterministic preconditions,
- allowing unlimited grasp retries,
- replacing the existing trace system.

The goal is specifically to add the missing cognitive bridge between:

```text
"the action failed"
```

and

```text
"this is likely why it failed, this is what we already tried, and this is the next safe different thing to try"
```

---

## 11. Implementation Order

Recommended implementation sequence for the next agent:

1. Add `FailureMemory` schema to `harness/state.py`.
2. Add unit tests for attempt accumulation/reset.
3. Create `harness/failure_analysis.py` with a minimal deterministic rule engine.
4. Record failed tool outcomes from `run_loop()`.
5. Expose compact failure state via `_planner_state_public()`.
6. Add executive recovery-safety classification.
7. Replace direct `GRASP_NOT_ACQUIRED -> structured_failed` with diagnosis/recovery policy.
8. Add blind-repeat prevention.
9. Persist diagnosis into execution traces.
10. Add a biased-grasp simulation regression test.
11. Only then adjust `system_prompt()` to make the LLM consume the new state explicitly.

---

## 12. Review Notes for the Implementing Agent

Before changing behavior, read:

- `agent.md`
- `README.md`
- `ROADMAP.md`
- `docs/decision_log.md`
- `harness/loop.py`
- `harness/state.py`
- `harness/executive.py`
- `harness/protocol.py`
- `harness/execution_trace.py`
- `harness/trace_analysis.py`

Do not weaken the current REAL-world safety rationale simply to increase apparent autonomy.

The key architectural principle is:

> **Deterministic executive owns safety; failure memory owns continuity; diagnosis owns explanation; planner owns strategy selection within allowed recoveries.**

That separation should remain clear in code.
