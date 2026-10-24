# Evaluation of `marl-mappo-s42-it300-08e745b853` (profile: ideal)

| suite | variant | eps | completion % | aborted | min sep (m) | viol. pair-steps | eps w/ viol. | eps inside d_eff | interventions | infeasible | fallbacks | mean steps | max p95 step (ms) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| shipped | reference | 3 | 100.00 | 0 | 0.00 | 27 | 3 | 3 | 0 | 0 | 0 | 295.33 | 0.38 |
| shipped | unshielded_marl | 3 | 100.00 | 0 | 1.13 | 26 | 3 | 3 | 0 | 0 | 0 | 362.00 | 0.52 |
| shipped | shielded_marl | 3 | 100.00 | 0 | 3.51 | 13 | 1 | 1 | 110 | 0 | 0 | 366.33 | 20.36 |
| shipped | shielded_reference | 3 | 100.00 | 0 | 3.74 | 13 | 1 | 1 | 114 | 0 | 0 | 305.67 | 21.27 |
| heldout | reference | 50 | 100.00 | 0 | 0.82 | 56 | 21 | 21 | 0 | 0 | 0 | 246.26 | 0.52 |
| heldout | unshielded_marl | 50 | 100.00 | 0 | 2.43 | 6 | 2 | 3 | 0 | 0 | 0 | 229.58 | 0.77 |
| heldout | shielded_marl | 50 | 100.00 | 0 | 7.93 | 0 | 0 | 0 | 243 | 0 | 0 | 229.06 | 25.43 |
| heldout | shielded_reference | 50 | 100.00 | 0 | 8.13 | 0 | 0 | 0 | 1634 | 0 | 0 | 245.78 | 23.43 |
| dense16 | reference | 50 | 100.00 | 0 | 0.81 | 105 | 17 | 19 | 0 | 0 | 0 | 321.74 | 2.53 |
| dense16 | unshielded_marl | 50 | 100.00 | 0 | 0.86 | 52 | 15 | 17 | 0 | 0 | 0 | 300.92 | 3.18 |
| dense16 | shielded_marl | 50 | 100.00 | 0 | 5.66 | 0 | 0 | 0 | 2172 | 0 | 0 | 300.86 | 43.51 |
| dense16 | shielded_reference | 50 | 100.00 | 0 | 5.77 | 0 | 0 | 0 | 5121 | 0 | 0 | 323.50 | 36.13 |

Measured on seeded simulation only. Not a guarantee; zero violations here do not establish zero risk.
