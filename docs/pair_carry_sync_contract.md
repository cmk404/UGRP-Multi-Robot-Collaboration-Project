# Pair carry synchronization contract

`PairCarrySync` is a bounded, in-process execution barrier for one task and a
fixed set of robot identities. It is not a planner, message bus, distributed
lock, or consensus protocol. The caller authenticates the source identity and
delivers reports; the barrier only checks the identity string against its fixed
participant set.

All timestamps must be finite seconds from the same monotonic clock domain.
`observed_at_s` is when the source frame was observed and must not be later than
`received_at_s`, when the barrier received the report. `authorize(now_s)` uses
observation age to enforce `report_ttl_s`; a report already older than the TTL
when received is rejected. Calls also cannot move the local clock backwards.
Wall-clock timestamps, unsynchronized
robot clocks, and delayed conversion between clock domains violate this
contract.

Each participant reports a strictly increasing sequence and a previously unseen
non-empty frame ID. Reports must match the current task's plan version and
epoch. Two fresh positive reports authorize `GO`. A valid negative report or an
expired/missing report invalidates a prior `GO`: the next authorization check
enters `HOLD`, clears readiness, and advances the epoch. Explicit `hold()` does
the same; repeated calls while already held are idempotent. Returning to `GO`
requires a new report from every participant in the new epoch. `update_plan()`
accepts only a higher version and also advances the epoch. `ABORT` is terminal.

`authorize()`, `hold()`, `abort()`, and `update_plan()` return a JSON-compatible
decision with `phase`, `epoch`, and `reason`. `events` is an append-only,
JSON-serializable diagnostic list. Every event includes `event`, `timestamp_s`,
`task_id`, `plan_version`, and `epoch`; report events also include the participant
and relevant report fields. Rejected input is recorded but does not change the
barrier state.

A `GO` decision means only that the local execution preconditions were satisfied
at that authorization check. A command issued after `GO` is not evidence that a
robot moved, grasped, or remained connected. Observed motion must come from the
allowed camera observations and be reported separately. This module has no
physical safety guarantee and does not provide network fault tolerance or
distributed consensus.

`SharedResourceLedger` is similarly local. Each required participant must reserve
the same `(resource_id, task_id, plan_version)` before it can become occupied.
An unoccupied reservation expires, but an occupied resource never expires. It
can be removed only after explicit release acknowledgments from every required
participant with the exact owning task and plan version. Other tasks and stale
versions cannot occupy, release, or replace it. Durable ownership, crash recovery,
authentication, and network replication remain responsibilities of the caller.
