# Parallel visual-distribution expansion

The eight original train/development demonstrations all have zero overlap between the beam and box TRANSIT windows. The corrected parallel scene changes the model-visible background. This study holds the 128px/history4 architecture, training seed and 8000-update budget fixed and expands demonstrations instead.

Finite sequence: two new RGB demonstrations, combine the successful pickup-overlap reference and original data with whole-episode train/development separation, train one ACT, and run all nine paired physical evaluations. Test configurations are excluded from the added demonstrations. Every success/failure is retained; no action correction uses teacher or referee information. Results remain local and completed trials are exported to the common TensorBoard.
