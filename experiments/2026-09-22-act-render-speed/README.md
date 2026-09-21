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

Measured at source `4beb670e1f8ac0faab82c095474863bbb097e92f`: all eight pairs preserved the actor JPEG hashes and physical state. Capture median was 0.748671 seconds for five images and 0.471948 seconds for three images (1.586x throughput, 36.96% less capture time). Other Mac jobs were active, so paired alternating timings are recorded individually. These are not whole-mission wall times.

Next validation: the original ACT seed18/open-minus successful mission, original calibration and plan, 900 carry decisions, 2400-second wall cap, 4 fps video, with only `--efficient-capture` enabled. Compare full actor request hashes, ACT commands, and physical success against the repaired reference. Run this one job serially after the current revalidation finishes; preserve both full outputs.
