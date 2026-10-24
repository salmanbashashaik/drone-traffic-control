# Evaluation of `marl-mappo-s42-it300-08e745b853` (profile: ideal)

| suite | variant | eps | completion % | aborted | min sep (m) | viol. pair-steps | eps w/ viol. | eps inside d_eff | interventions | infeasible | fallbacks | mean steps | max p95 step (ms) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| swap | reference | 50 | 100.00 | 0 | 0.05 | 338 | 50 | 50 | 0 | 0 | 0 | 226.44 | 4.95 |
| swap | unshielded_marl | 50 | 100.00 | 0 | 0.33 | 344 | 50 | 50 | 0 | 0 | 0 | 210.08 | 2.56 |
| swap | shielded_marl | 50 | 100.00 | 0 | 6.05 | 0 | 0 | 0 | 2222 | 0 | 0 | 209.44 | 33.81 |
| swap | shielded_reference | 50 | 100.00 | 0 | 5.81 | 0 | 0 | 0 | 3185 | 0 | 0 | 226.48 | 39.02 |

Measured on seeded simulation only. Not a guarantee; zero violations here do not establish zero risk.
