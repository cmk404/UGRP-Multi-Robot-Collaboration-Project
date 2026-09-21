# Pair capture speed without changing actor inputs

The ACT repair revalidation spends substantial time in rendering. Each pair
observation previously rendered TOP, all three robot cameras, and a standalone
overview JPEG. The pair only consumes TOP and its two cameras. Continuous video
has a separate capture path.

`--efficient-capture` omits the third robot's unused camera and the standalone
overview for pair observations. Default behavior, planning/identity captures,
consumed camera settings and JPEG bytes, physics, control timing, and continuous
video remain unchanged. No image estimate or simulator state is added to control.

Validation protocol: `scripts/benchmark_dispatch_capture.py --output <new-dir>`
runs eight counterbalanced capture pairs on one frozen open scene, alternating
two participant allocations. It records source SHA, environment, camera/physics
invariants, every actor image hash, unchanged physical-state hashes and simulation
time, and individual/median wall times. This measures capture throughput only;
it does not establish an end-to-end speedup or new model success rate.

Execution source is committed before measurement. Raw files remain local under
`outputs/render-speed/`; measured results and source SHA are recorded after the
finite benchmark completes. A complete mission must separately verify behavior.
