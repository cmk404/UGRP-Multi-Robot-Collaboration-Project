# Parallel visual-distribution expansion

The eight original train/development demonstrations all have zero overlap between the beam and box TRANSIT windows. The corrected parallel scene changes the model-visible background. This study holds the 128px/history4 architecture, training seed and 8000-update budget fixed and expands demonstrations instead.

Finite sequence: two new RGB demonstrations, combine the successful pickup-overlap reference and original data with whole-episode train/development separation, train one ACT, and run all nine paired physical evaluations. Test configurations are excluded from the added demonstrations. Every success/failure is retained; no action correction uses teacher or referee information. Results remain local and completed trials are exported to the common TensorBoard.

The saved baseline was trained on a Tesla T4 (PyTorch 2.11 CUDA); the expanded model trains on Mac CPU. Architecture, seed, step budget and deployment cases are controlled, but this is a practical existing-versus-expanded checkpoint comparison, not a causal data-only ablation. Three held-out spawn perturbations in the open map are not evidence of broad unseen-map generalization.

Completed additions: train-plus and development-zero both physically succeeded (0 failures / 2 teacher trials), with strict sampled concurrent loaded motion of 6.0s and 5.8s, respectively. Sampled video review shows concurrent carry and final placement. Dataset admission and image hashes passed; 8 train and 3 development episodes were admitted. New training is running under db0a6a7; physical evaluation results are not yet available at this checkpoint. See launch.json and the local study.json for exact stage state.
