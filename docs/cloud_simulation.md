# UGRP simulation compute architecture

Updated: 2026-09-02 (cloud GPU providers retired)

## Decision (2026-09-02)

**MuJoCo for this project is CPU-bound, not GPU-bound.** The 3-robot MasterPi
world runs at a few hundred µs per physics step; camera rendering is offscreen
software/EGL at 640×480 and is not the bottleneck either. Every cloud GPU we
tried (Colab T4, Lightning T4/L4, Azure A10) added 30–120 ms of WAN round trip
per command/frame, cost money or quota, and needed its own recovery/cost-guard
machinery, without making the simulator faster than a laptop CPU.

Therefore:

- **Mac (Apple M3, `.venv-sim-worker-mac`) is the only automatic remote worker.**
  `scripts/mac_worker_recover.py` starts it over Tailscale SSH; it connects to
  the bridge over the authenticated WebSocket (`wss://…:8443`).
- **Oracle CPU worker (`ugrp-sim-worker.service`) is the manual fallback**
  (disabled while `UGRP_SIM_GPU_ONLY=1`; see Trade-offs below).
- **Azure, Colab and Lightning are retired.** The Azure PAYG subscription and
  the `ugrp-a10-sim` VM no longer exist; `ugrp-azure-idle-controller.timer` is
  disabled; `.env.gpu` lists only `UGRP_GPU_PROVIDERS='mac'` with
  `UGRP_ALLOW_AZURE_WORKER=0`, `UGRP_ALLOW_COLAB_WORKER=0`. Their recover
  scripts (`azure_worker_recover.py`, `colab_worker_recover.py`,
  `lightning_worker_recover.py`, `deploy_lightning_sim.py`, `cloud/lightning/`)
  stay in the tree as reference only and are never invoked automatically.
- Kaggle/Modal notes from the earlier revision are no longer relevant to the
  live simulator and are dropped; batch RL training, if ever needed, is a
  separate decision.

The simulator protocol is unchanged: one provider-neutral authenticated
WebSocket worker; a connecting worker is parked until the failover controller
grants authority, except that the **same process** reconnecting after a socket
blip resumes authority immediately (bridge fix 2026-09-02).

## Runtime architecture

```text
Browser (tailnet)  ──►  Oracle :8082/8084/8085  SIM coworker UI (r1/r2/r3)
                          │
                          ▼
                        Oracle :8091  simulation bridge / command queue / authority
                          │   token-authenticated control endpoints
                          │   (/command, /remote/authority, /sim/speed)
                          ├── local HTTP worker :8092 (Oracle CPU, manual fallback)
                          └── WebSocket :8093 → tailscale serve :8443
                                └── Mac M3 MuJoCo worker  (only automatic provider)

Oracle :8083/8086/8087    REAL coworker UI (ugrp1/2/3)  — unrelated to the SIM compute path
```

## Automatic recovery (`ugrp-sim-failover.service`)

1. Poll bridge health every second.
2. If a remote worker is connected but its `provider` is not in the configured
   set (`mac`) or its worker contract is incompatible, revoke authority and
   ignore it.
3. If no allowed remote worker is connected for 3 s **and** a browser touched
   SIM within the last 75 s, start `ugrp-sim-gpu-recover.service`, which runs
   `gpu_worker_recover.py` → `mac_worker_recover.py`.
4. When the worker is connected, parked, and nothing is inflight, grant
   authority; the bridge resets the publication epoch and requests a sync frame.

`.env.gpu` is re-read on every provider probe, so provider changes take effect
within ~15 s without restarting services.

## Trade-offs the user may still want to revisit

| Option | Latency (Mac browser ↔ sim) | Availability | Notes |
| --- | --- | --- | --- |
| **Mac worker via Oracle bridge (current)** | 2 WAN hops per command/frame, ~40–80 ms + Tailscale relay jitter | depends on Mac awake + Tailscale | UI/bridge/coworker stay on Oracle; nothing to install on Mac beyond the worker venv |
| Oracle CPU worker (`UGRP_SIM_GPU_ONLY=0`) | 1 WAN hop | always on | ~2–3× slower physics/render than M3; fine for regression, slow for interactive 3-robot use |
| Full local stack on Mac (bridge + coworker + worker) | 0 WAN hops | offline-capable | needs Groq keys on Mac, separate ports from REAL stack, and a decision to leave Oracle as REAL-only |

## Security

- Bridge control endpoints (`POST /command`, `/remote/authority`, `/sim/speed`)
  and worker channels require `X-UGRP-Sim-Token`. Callers read it via
  `sim/bridge_client.py` (env `UGRP_SIM_TOKEN`, else `.sim_bridge_token`).
  Loopback binding is not treated as an authentication boundary.
- Request bodies are capped (16 MiB worker, 64 KiB control); the command queue
  is bounded (`UGRP_SIM_MAX_QUEUE_DEPTH`, default 16 → HTTP 429
  `SIM_QUEUE_FULL`); MJPEG stream fan-out is bounded (`UGRP_SIM_MAX_STREAMS`).
- Secrets (`.env.gpu`, `.sim_bridge_token`, `.sim_worker_env`, `.groq_keys`,
  `.webui_secret_key`) are mode 0600 on both Oracle and the Mac mirror and are
  git-ignored.
- Human-facing UIs remain tailnet-only behind `tailscale serve`.

## Previous revisions

The 2026-08-29 revision of this file described Lightning as preferred GPU,
Colab as opt-in and Kaggle/Modal as future options. That plan was superseded
by the Azure A10 experiment (08-29 → 09-01, see `docs/decision_log.md`) and
then by this CPU-only decision. Historical details remain in the decision log.
