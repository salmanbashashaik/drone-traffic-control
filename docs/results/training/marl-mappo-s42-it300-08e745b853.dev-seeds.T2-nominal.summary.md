# Evaluation of `marl-mappo-s42-it300-08e745b853` (profile: nominal)

| suite | variant | eps | completion % | aborted | min sep (m) | viol. pair-steps | eps w/ viol. | eps inside d_eff | interventions | infeasible | fallbacks | mean steps | max p95 step (ms) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| shipped | reference | 3 | 100.00 | 0 | 0.84 | 22 | 2 | 3 | 0 | 0 | 0 | 371.00 | 0.47 |
| shipped | unshielded_marl | 3 | 100.00 | 0 | 1.14 | 22 | 2 | 2 | 0 | 0 | 0 | 391.00 | 0.83 |
| shipped | shielded_marl | 3 | 100.00 | 0 | 3.79 | 16 | 1 | 1 | 112 | 0 | 0 | 397.00 | 36.84 |
| shipped | shielded_reference | 3 | 100.00 | 0 | 3.78 | 16 | 1 | 1 | 119 | 0 | 0 | 386.00 | 43.84 |
| heldout | reference | 30 | 100.00 | 0 | 1.59 | 23 | 8 | 10 | 0 | 0 | 0 | 257.60 | 0.93 |
| heldout | unshielded_marl | 30 | 77.78 | 0 | 2.88 | 8 | 4 | 4 | 0 | 0 | 0 | 472.50 | 1.05 |
| heldout | shielded_marl | 30 | 78.89 | 0 | 7.66 | 0 | 0 | 0 | 150 | 0 | 0 | 471.13 | 34.69 |
| heldout | shielded_reference | 30 | 100.00 | 0 | 7.51 | 0 | 0 | 0 | 723 | 0 | 0 | 259.10 | 42.41 |

Measured on seeded simulation only. Not a guarantee; zero violations here do not establish zero risk.
