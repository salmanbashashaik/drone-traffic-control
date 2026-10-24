# Evaluation of `marl-mappo-s42-it300-8f5c910be5` (profile: ideal)

| suite | variant | eps | completion % | aborted | min sep (m) | viol. pair-steps | eps w/ viol. | eps inside d_eff | interventions | infeasible | fallbacks | mean steps | max p95 step (ms) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| heldout | reference | 50 | 100.00 | 0 | 0.62 | 51 | 19 | 20 | 0 | 0 | 0 | 260.08 | 0.60 |
| heldout | unshielded_marl | 50 | 100.00 | 0 | 3.35 | 8 | 3 | 3 | 0 | 0 | 0 | 223.98 | 0.55 |
| heldout | shielded_marl | 50 | 100.00 | 0 | 7.71 | 0 | 0 | 0 | 213 | 0 | 0 | 224.02 | 24.52 |
| heldout | shielded_reference | 50 | 100.00 | 0 | 7.17 | 0 | 0 | 0 | 1327 | 0 | 0 | 259.50 | 34.09 |

Measured on seeded simulation only. Not a guarantee; zero violations here do not establish zero risk.
