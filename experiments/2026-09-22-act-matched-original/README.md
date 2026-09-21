# Matched original-calibration diagnostic

New 128px/h1 seed20260921 ACT failed at final visual placement after 543 rounds (471.940s). The beam stayed off the floor during commanded transit, but ended outside its slot. Both actors claimed arrival; 263 earlier rounds had one-sided arrival claims.

The original ACT reference succeeds under the same calibration, scene XML, pre-carry commands and first carry RGB bytes (alignment.json). Thus original-vs-native calibration is not sufficient to explain this model failure. Architecture/context, training seed and checkpoint differ between models; those effects are not individually isolated. No student actions were corrected from teacher or referee state.

Failure and video exported to the common TensorBoard; event reload, video Range and native UI verified. This is one diagnostic, not a failure-rate estimate. Raw remains local.
