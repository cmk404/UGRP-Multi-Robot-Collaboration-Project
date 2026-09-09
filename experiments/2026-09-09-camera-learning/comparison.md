# Camera-policy comparison

| Mode / seed | Calls | Grasp | Lift m | Hold s | Input tokens | Median call ms | Open commands | Longest repeat r1/r3 |
|---|---:|---|---:|---:|---:|---:|---:|---|
| baseline / 11 | 60 | False | -4.9e-05 | 0.0 | 160020 | 7658 | 21 | 16/6 |
| memory / 11 | 60 | False | -4.9e-05 | 0.0 | 167620 | 4912 | 4 | 1/3 |
| temporal / 11 | 60 | False | -4.9e-05 | 0.0 | 293308 | 5151 | 6 | 2/5 |
| learned / 11 | 60 | False | -4.9e-05 | 0.0 | 350552 | 4854 | 3 | 2/3 |

Exploratory bounded cohort: no statistical significance claim. Actual billing is unknown.
Identical frames and repeated commands are symptoms, not causal proof. Physics pauses during inference;
SIM success and call latency do not establish real-time hardware performance. Raw artifacts remain local.
