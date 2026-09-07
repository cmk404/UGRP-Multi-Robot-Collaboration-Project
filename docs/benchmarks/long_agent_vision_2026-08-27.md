# Long-agent vision benchmark — 2026-08-27

Live Groq `qwen/qwen3.6-27b` + current UGRP `auto_observe=True` loop. Robot execution is replaced by a stateful synthetic world whose next camera frame depends on the tool actually chosen. This tests agent behavior, not motor control.

| Case | Result | Tool actions | Observations | Rate-limit retries | Final state |
|---|---:|---:|---:|---:|---|
| `offcenter_recovery` | **PASS** | 5 | 5 | 1 | held=true, stable=true |
| `slip_long_recovery` | **PARTIAL** | 10 | 10 | 15 | held=false, stable=false |
| `double_pick_failure` | **PASS** | 6 | 6 | 8 | held=true, stable=true |
| `target_lost_with_decoy` | **PASS** | 1 | 1 | 2 | held=false, stable=false |
| `no_red_target` | **PASS** | 0 | 0 | 0 | held=false, stable=false |

## Findings

- **Long traces do work.** The model maintained goal-directed behavior across up to 13 model decisions / 10 tool actions in the slip case, and across repeated failures it did not invent tools.
- **Recovery works when the scene makes the failure legible.** In `offcenter_recovery`, the first pick failed; the agent noticed the block remained on the floor, used `track`, retried `pick`, then used `carry`, reaching the true success state.
- **Repeated failure recovery also works.** In `double_pick_failure`, two pick attempts were forced to fail. The agent noticed both failures, eventually used `track`, succeeded on the third pick, and reached `held=true, stable=true`.
- **Safety / task identity was preserved.** When the red target disappeared and a salient blue decoy remained, it stopped rather than substituting the blue object. With no red object initially, it used no tools.
- **Hard recovery is still weak.** In `slip_long_recovery`, after a successful grasp the block slipped to the right. The agent correctly noticed repeated failure and eventually stopped instead of falsely claiming success, but repeatedly said it would adjust the arm while issuing `pick` instead of `track`. It did not solve the recoverable geometry within the tool budget.
- **Planning is not fully reliable.** Multi-tool goals often produced a plan, but `double_pick_failure` began directly with `approach`. The runtime still remained coherent because every action received a fresh post-action frame.
- **Current Groq quota is a practical long-horizon bottleneck.** Vision calls use about 2.2k prompt tokens each and the current organization limit observed was 8,000 TPM. The longest successful recovery needed multiple 429 retries; `double_pick_failure` took 8 retries and `slip_long_recovery` 15. A production long-running agent needs built-in retry/backoff and/or less frequent VLM calls / a cheaper perception path.

## Current verdict

**The current system is already an agent loop, not just one-shot VLM tool calling.** It can observe, act, re-observe, detect some failures, revise its course, preserve the target identity, and continue for many steps. However, it is **not yet robust enough to call a dependable long-horizon autonomous robot agent**: difficult geometry can lead to repetitive ineffective actions, planning compliance is imperfect, and API TPM pressure grows quickly with repeated visual observations.

## Next engineering targets

1. Add structured progress state (`goal`, `attempts`, `last_effect`, `failure_reason`, `next_verification`) instead of relying only on conversational history.
2. Add loop-level anti-repeat logic: after the same tool fails twice with materially similar observations, require a different tool or explicit stop/replan.
3. Make tool results semantic (`success`, `target_visible`, `grasp_verified`, detector geometry) rather than only returning `exit_code=0`.
4. Add built-in Groq 429 retry/backoff and compact/summarized history for long runs.
5. Keep this benchmark as a regression suite and require all recoverable cases to reach the true simulator state, not merely a plausible final sentence.
