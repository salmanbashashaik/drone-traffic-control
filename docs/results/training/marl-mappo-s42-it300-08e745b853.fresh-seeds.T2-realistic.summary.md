# Evaluation of `marl-mappo-s42-it300-08e745b853` (profile: realistic)

| suite | variant | eps | completion % | aborted | min sep (m) | viol. pair-steps | eps w/ viol. | eps inside d_eff | interventions | infeasible | fallbacks | mean steps | max p95 step (ms) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| heldout | reference | 50 | 100.00 | 0 | 0.64 | 48 | 19 | 21 | 0 | 0 | 0 | 227.80 | 0.62 |
| heldout | unshielded_marl | 50 | 83.22 | 1 | 2.53 | 5 | 3 | 3 | 0 | 0 | 0 | 451.96 | 0.74 |
| heldout | shielded_marl | 50 | 83.89 | 0 | 6.97 | 0 | 0 | 0 | 272 | 0 | 0 | 451.78 | 23.46 |
| heldout | shielded_reference | 50 | 100.00 | 0 | 8.20 | 0 | 0 | 0 | 1518 | 0 | 0 | 228.22 | 23.70 |

Measured on seeded simulation only. Not a guarantee; zero violations here do not establish zero risk.
