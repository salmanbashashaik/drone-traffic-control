# Evaluation of `marl-mappo-s42-it300-08e745b853` (profile: ideal)

| suite | variant | eps | completion % | aborted | min sep (m) | viol. pair-steps | eps w/ viol. | eps inside d_eff | interventions | infeasible | fallbacks | mean steps | max p95 step (ms) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| six | reference | 50 | 100.00 | 0 | 0.94 | 61 | 24 | 24 | 0 | 0 | 0 | 280.38 | 1.13 |
| six | unshielded_marl | 50 | 100.00 | 0 | 2.80 | 13 | 4 | 4 | 0 | 0 | 0 | 254.80 | 1.83 |
| six | shielded_marl | 50 | 100.00 | 0 | 7.81 | 0 | 0 | 0 | 486 | 0 | 0 | 256.44 | 27.39 |
| six | shielded_reference | 50 | 100.00 | 0 | 8.04 | 0 | 0 | 0 | 2036 | 0 | 0 | 281.60 | 34.58 |

Measured on seeded simulation only. Not a guarantee; zero violations here do not establish zero risk.
