# Evaluation of `marl-mappo-s42-it300-08e745b853` (profile: ideal)

| suite | variant | eps | completion % | aborted | min sep (m) | viol. pair-steps | eps w/ viol. | eps inside d_eff | interventions | infeasible | fallbacks | mean steps | max p95 step (ms) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| heldout | reference | 50 | 100.00 | 0 | 1.05 | 64 | 23 | 24 | 0 | 0 | 0 | 226.02 | 0.87 |
| heldout | unshielded_marl | 50 | 100.00 | 0 | 1.63 | 10 | 3 | 3 | 0 | 0 | 0 | 193.94 | 0.89 |
| heldout | shielded_marl | 50 | 100.00 | 0 | 8.08 | 0 | 0 | 0 | 324 | 0 | 0 | 193.98 | 29.73 |
| heldout | shielded_reference | 50 | 100.00 | 0 | 7.86 | 0 | 0 | 0 | 1602 | 0 | 0 | 226.04 | 59.37 |
| dense16 | reference | 50 | 100.00 | 0 | 0.79 | 110 | 20 | 22 | 0 | 0 | 0 | 317.30 | 2.21 |
| dense16 | unshielded_marl | 50 | 100.00 | 0 | 0.78 | 52 | 11 | 12 | 0 | 0 | 0 | 295.10 | 5.74 |
| dense16 | shielded_marl | 50 | 100.00 | 0 | 5.85 | 0 | 0 | 0 | 1937 | 0 | 0 | 293.88 | 31.29 |
| dense16 | shielded_reference | 50 | 100.00 | 0 | 5.59 | 0 | 0 | 0 | 4463 | 0 | 0 | 317.50 | 30.56 |

Measured on seeded simulation only. Not a guarantee; zero violations here do not establish zero risk.
