# REAL execution recorder

Status: MasterPi platform/debug tooling. This is not a research result.

Every Oracle-launched `scripts/red_block/*` REAL skill now receives a unique trace ID. Recording is enabled only in that remote child process; SIM and ordinary unit tests remain no-op unless the trace environment variables are explicitly set.

Local traces are collected under:

`outputs/real_traces/<trace_id>/`

Each trace contains, when available:

- `result.json`: skill/script, argv, immutable package hash, wall-clock start/end/duration, exit code and copy status;
- `events.jsonl`: timestamped camera-frame metadata, red/blue/yellow detections, commanded servo PWM state, chassis commands/wheel values, STOPs and hardware probe telemetry;
- `frames/*.jpg`: throttled camera samples (default 4 Hz) plus forced important/debug frames;
- `stderr.log`: remote failure text when present;
- `failure-debug.jpg`: the existing failure debug image when a failed skill produced one.

Camera metadata is emitted for every frame read. JPEG encoding is throttled to avoid changing controller timing materially. `save_debug_frame()` forces an additional important JPEG. Detection rows contain the frame sequence when the detector receives the original captured frame.

The recorder is intentionally best-effort: trace I/O or JPEG failures are swallowed and cannot turn into actuator failures.

This trace schema is designed as the input to the next stage: REAL failure corpus/replay and SIM calibration. Hidden simulator state is not added to REAL traces.
