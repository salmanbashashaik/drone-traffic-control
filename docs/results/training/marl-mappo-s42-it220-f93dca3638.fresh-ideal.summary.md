# Evaluation of `marl-mappo-s42-it220-f93dca3638` (profile: ideal)

| suite | variant | eps | completion % | aborted | min sep (m) | viol. pair-steps | eps w/ viol. | eps inside d_eff | interventions | infeasible | fallbacks | mean steps | max p95 step (ms) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| heldout | reference | 50 | 100.00 | 0 | 1.05 | 64 | 23 | 24 | 0 | 0 | 0 | 226.02 | 0.54 |
| heldout | unshielded_marl | 50 | 100.00 | 0 | 4.34 | 3 | 2 | 2 | 0 | 0 | 0 | 196.80 | 0.38 |
| heldout | shielded_marl | 50 | 100.00 | 0 | 8.29 | 0 | 0 | 0 | 360 | 0 | 0 | 196.78 | 18.20 |
| heldout | shielded_reference | 50 | 100.00 | 0 | 7.86 | 0 | 0 | 0 | 1602 | 0 | 0 | 226.04 | 16.33 |

Measured on seeded simulation only. Not a guarantee; zero violations here do not establish zero risk.
