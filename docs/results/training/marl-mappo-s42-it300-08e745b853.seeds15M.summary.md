# Evaluation of `marl-mappo-s42-it300-08e745b853` (profile: ideal)

| suite | variant | eps | completion % | aborted | min sep (m) | viol. pair-steps | eps w/ viol. | eps inside d_eff | interventions | infeasible | fallbacks | mean steps | max p95 step (ms) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| heldout | reference | 50 | 100.00 | 0 | 0.62 | 51 | 19 | 20 | 0 | 0 | 0 | 260.08 | 0.79 |
| heldout | unshielded_marl | 50 | 99.33 | 0 | 1.93 | 8 | 2 | 2 | 0 | 0 | 0 | 241.12 | 0.98 |
| heldout | shielded_marl | 50 | 99.33 | 0 | 7.94 | 0 | 0 | 0 | 204 | 0 | 0 | 241.48 | 34.82 |
| heldout | shielded_reference | 50 | 100.00 | 0 | 7.17 | 0 | 0 | 0 | 1327 | 0 | 0 | 259.50 | 23.89 |

Measured on seeded simulation only. Not a guarantee; zero violations here do not establish zero risk.
