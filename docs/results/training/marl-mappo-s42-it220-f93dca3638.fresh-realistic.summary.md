# Evaluation of `marl-mappo-s42-it220-f93dca3638` (profile: realistic)

| suite | variant | eps | completion % | aborted | min sep (m) | viol. pair-steps | eps w/ viol. | eps inside d_eff | interventions | infeasible | fallbacks | mean steps | max p95 step (ms) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| heldout | reference | 50 | 100.00 | 0 | 0.64 | 48 | 19 | 21 | 0 | 0 | 0 | 227.80 | 0.42 |
| heldout | unshielded_marl | 50 | 100.00 | 0 | 3.75 | 4 | 2 | 3 | 0 | 0 | 0 | 224.68 | 0.49 |
| heldout | shielded_marl | 50 | 100.00 | 0 | 8.08 | 0 | 0 | 0 | 342 | 0 | 0 | 224.46 | 19.65 |
| heldout | shielded_reference | 50 | 100.00 | 0 | 8.20 | 0 | 0 | 0 | 1518 | 0 | 0 | 228.22 | 15.99 |
| dense16 | reference | 50 | 100.00 | 0 | 0.89 | 78 | 20 | 24 | 0 | 0 | 0 | 325.44 | 1.50 |
| dense16 | unshielded_marl | 50 | 100.00 | 0 | 1.36 | 71 | 13 | 14 | 0 | 0 | 0 | 308.22 | 4.73 |
| dense16 | shielded_marl | 50 | 100.00 | 0 | 5.14 | 0 | 0 | 1 | 2054 | 0 | 0 | 306.08 | 22.54 |
| dense16 | shielded_reference | 50 | 100.00 | 0 | 5.45 | 0 | 0 | 1 | 3585 | 0 | 0 | 324.58 | 30.31 |

Measured on seeded simulation only. Not a guarantee; zero violations here do not establish zero risk.
