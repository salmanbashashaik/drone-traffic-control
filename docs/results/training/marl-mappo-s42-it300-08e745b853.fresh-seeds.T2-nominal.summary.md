# Evaluation of `marl-mappo-s42-it300-08e745b853` (profile: nominal)

| suite | variant | eps | completion % | aborted | min sep (m) | viol. pair-steps | eps w/ viol. | eps inside d_eff | interventions | infeasible | fallbacks | mean steps | max p95 step (ms) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| heldout | reference | 50 | 100.00 | 0 | 0.60 | 46 | 19 | 22 | 0 | 0 | 0 | 233.98 | 0.61 |
| heldout | unshielded_marl | 50 | 75.17 | 1 | 2.59 | 8 | 3 | 3 | 0 | 0 | 0 | 459.14 | 1.28 |
| heldout | shielded_marl | 50 | 77.18 | 0 | 7.79 | 0 | 0 | 0 | 306 | 0 | 0 | 447.32 | 22.94 |
| heldout | shielded_reference | 50 | 100.00 | 0 | 8.46 | 0 | 0 | 0 | 1497 | 0 | 0 | 234.56 | 22.73 |

Measured on seeded simulation only. Not a guarantee; zero violations here do not establish zero risk.
