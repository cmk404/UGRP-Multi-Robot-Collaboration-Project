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

For a new repeat/revalidation protocol, `controls.efficient_capture: true`
forwards the same option to both teacher and ACT trials. Omission preserves old
protocol behavior. Completed or active protocols are never rewritten in place.

Execution source is committed before measurement. Raw files remain local under
`outputs/render-speed/`; measured results and source SHA are recorded after the
finite benchmark completes. A complete mission must separately verify behavior.

Measured at source `4beb670e1f8ac0faab82c095474863bbb097e92f`: all eight pairs preserved the actor JPEG hashes and physical state. Capture median was 0.748671 seconds for five images and 0.471948 seconds for three images (1.586x throughput, 36.96% less capture time). Other Mac jobs were active, so paired alternating timings are recorded individually. These are not whole-mission wall times.

Full-mission validation completed at source `074f1ec640661508c37bbb9289fe2b9ea814a420`: the original ACT seed18/open-minus mission succeeded in 437.187 seconds with efficient capture and 4 fps video. All 104 ACT decision rounds, 208 request hashes, 883 issued commands, and 2,177 common JPEG files are identical to the repaired reference. Scene XML also matches. Only unused pair camera/overview files are absent; no extra or changed images were found.

The repaired reference took 814.808 seconds while another simulation was active; the optimized validation ran alone. These whole-mission times describe the observed runs, not a controlled estimate of the render-only effect. The counterbalanced capture benchmark above is the isolated speed measurement. This validation establishes reference behavior preservation, not a new model's generalization or retraining.

`full-mission.json` links the exact command, source SHA, source hashes, alignment audits and verified TensorBoard snapshot. The 159.75-second source video is 960x720 at 4 fps; its final frame was visually reviewed. Raw artifacts remain local.
