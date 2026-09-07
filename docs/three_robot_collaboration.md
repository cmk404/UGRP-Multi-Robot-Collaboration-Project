# UGRP 3× Robot Collaboration Architecture

## Scope

This is the user-approved three-MasterPi collaboration platform layer. It does **not** redefine the official research specification or claim calibrated SIM↔REAL parity.

## Identity and session isolation

- Robot IDs are fixed to `r1`, `r2`, `r3`.
- Each robot owns a separate ChatState / planner history / cancel state / camera cache.
- SIM session ports: R1 `8082`, R2 `8084`, R3 `8085`.
- REAL session ports: R1 `8083`, R2 `8086`, R3 `8087`.
- The public R1 UI proxies the other sessions through `/sim/r2`, `/sim/r3`, `/real/r1`, `/real/r2`, `/real/r3`.

The isolation is intentional: changing tabs must never make one agent inherit another robot's conversation, cancellation, camera evidence, or actuator target.

## SIM shared world

`sim.multi_masterpi_production.MultiMasterPiProductionV2` owns one MuJoCo model/data containing all three physical robot bodies. Every robot body, joint, actuator, and camera is namespaced.

The GPU worker receives `robot_id` with every command and routes that action only to the selected robot. The physics world is shared, so robots can collide and perceive one another. Controller execution is serialized deliberately because the migrated REAL controller stack still contains process-global compatibility shims and MuJoCo model/data mutation is not thread-safe.

Worker contract:

`masterpi-v2-three-robot-shared-world-v2`

Old single-robot GPU workers are rejected by the bridge instead of being silently treated as 3× compatible.

## Coordination model

The three agents are peers; TEAM is **not** a central planner.

`harness.team_bus` stores only coordination evidence and delivery state:

- one operator/team goal,
- the shared TEAM chat transcript,
- peer messages and proposed actions / required partners,
- ACK/NACK records,
- pending/claimed/completed wake-delivery records,
- recent action events.

TEAM is a neutral room/router, not an action planner. An operator message with `@R1`, `@R2`, or `@R3` wakes only the mentioned peer; `@everyone`/`@TEAM`/`@ALL`, or ordinary room text without a mention, gives all three independent peers a turn. A peer can explicitly hand work to another peer with `send_peer_message`, which is mirrored into the shared room and wakes the addressed idle agent. Plain final bot replies are display-only and do not recursively wake more bots, preventing reply storms.

Each R1/R2/R3 planner receives its own ID plus the current team goal, TEAM transcript, and visible peer messages as planner context, then decides locally. In an active multi-step turn, that context is refreshed immediately before every VLM decision; if an already-running robot consumes a newly queued room message, the redundant wake ticket is absorbed instead of starting a second overlapping turn. A per-robot turn lock is the final concurrency guard. None of these mechanisms selects the robot's next physical action.

## TEAM UI

TEAM is presented as a shared bot-chat room **only on the TEAM tab**. R1/R2/R3 tabs render only their own private robot chat and robot controls; the TEAM transcript is not rendered there. TEAM auto-wake turns are also ephemeral to each robot's private conversation history: the addressed planner may use the message for that decision, but the TEAM prompt/reply is stored only in the shared TEAM transcript.

It includes:

- one common chat timeline with operator, R1, R2, and R3 identities,
- `@R1` / `@R2` / `@R3` / `@everyone` / `@TEAM` mention routing,
- R1/R2/R3 camera previews and connection/state summaries,
- `QUEUED` / `THINKING` wake status,
- the shared goal and recent robot/team events.

In SIM, an addressed bot receives an ordinary independent agent turn and may act only if its own planner judges the message to be an action request. Conversational room messages remain conversation-only. In REAL, automatic TEAM wakeups are always `chat_only` and `execute=False`; physical commands still require an explicit REAL robot turn. TEAM intentionally exposes no central actuator executor.

## REAL safety policy

R1 is currently the only verified physical endpoint (`ugrp1`). R2 and R3 remain unconfigured until their actual endpoints are positively identified.

Unconfigured REAL slots run local placeholder sessions with:

- host `offline-r2.invalid` / `offline-r3.invalid`,
- no camera,
- `execute=False`,
- `robot_configured=False`.

The `.invalid` hostname is deliberate: an unconfigured slot must never fall back to R1 or accidentally resolve to another machine. To attach R2/R3 later, configure their real host and enable flags explicitly; do not infer them from historical names.

## Regression checks

`tests/test_multi_robot_platform.py` covers:

- peer goal/message/ACK sharing,
- TEAM mention routing and no-mention group delivery,
- idle-agent wake delivery and active-turn wake absorption,
- peer handoff mirroring into the room and recipient-only wakeups,
- SIM wakeups being eligible for an independent agent turn while REAL auto-wake remains chat-only,
- per-robot bridge frame namespace without overwriting shared world state,
- selected `UGRP_ROBOT_ID` propagation through SIM transport,
- inclusion of the multi-robot world in the GPU bundle,
- non-resolving dry-run REAL placeholders.

`tests/test_harness.py` verifies both live peer-context refresh on successive planner calls and the per-robot overlapping-turn rejection. `tests/test_sim_ui_consistency.py` locks the shared TEAM chat/mention/wake-status browser contract.

A physical REAL actuation test is intentionally excluded from automated regression.
