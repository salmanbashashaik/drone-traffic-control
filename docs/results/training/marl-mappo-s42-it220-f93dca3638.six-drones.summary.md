# Evaluation of `marl-mappo-s42-it220-f93dca3638` (profile: ideal)

| suite | variant | eps | completion % | aborted | min sep (m) | viol. pair-steps | eps w/ viol. | eps inside d_eff | interventions | infeasible | fallbacks | mean steps | max p95 step (ms) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| six | reference | 50 | 100.00 | 0 | 0.94 | 61 | 24 | 24 | 0 | 0 | 0 | 280.38 | 1.28 |
| six | unshielded_marl | 50 | 100.00 | 0 | 0.80 | 6 | 3 | 5 | 0 | 0 | 0 | 246.58 | 2.84 |
| six | shielded_marl | 50 | 100.00 | 0 | 8.00 | 0 | 0 | 0 | 558 | 0 | 0 | 246.60 | 26.45 |
| six | shielded_reference | 50 | 100.00 | 0 | 8.04 | 0 | 0 | 0 | 2036 | 0 | 0 | 281.60 | 28.82 |

Measured on seeded simulation only. Not a guarantee; zero violations here do not establish zero risk.
