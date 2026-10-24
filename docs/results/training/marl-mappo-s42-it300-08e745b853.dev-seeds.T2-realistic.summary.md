# Evaluation of `marl-mappo-s42-it300-08e745b853` (profile: realistic)

| suite | variant | eps | completion % | aborted | min sep (m) | viol. pair-steps | eps w/ viol. | eps inside d_eff | interventions | infeasible | fallbacks | mean steps | max p95 step (ms) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| shipped | reference | 3 | 100.00 | 0 | 0.84 | 24 | 2 | 3 | 0 | 0 | 0 | 340.33 | 0.35 |
| shipped | unshielded_marl | 3 | 100.00 | 0 | 0.86 | 22 | 2 | 2 | 0 | 0 | 0 | 394.00 | 0.59 |
| shipped | shielded_marl | 3 | 100.00 | 0 | 2.81 | 16 | 1 | 1 | 114 | 0 | 0 | 399.67 | 29.99 |
| shipped | shielded_reference | 3 | 100.00 | 0 | 2.80 | 16 | 1 | 1 | 126 | 0 | 0 | 339.00 | 55.78 |
| heldout | reference | 30 | 100.00 | 0 | 1.54 | 24 | 9 | 10 | 0 | 0 | 0 | 243.83 | 0.73 |
| heldout | unshielded_marl | 30 | 84.44 | 0 | 3.47 | 5 | 3 | 4 | 0 | 0 | 0 | 445.37 | 1.35 |
| heldout | shielded_marl | 30 | 84.44 | 0 | 7.10 | 0 | 0 | 0 | 174 | 0 | 0 | 444.50 | 35.54 |
| heldout | shielded_reference | 30 | 100.00 | 0 | 7.08 | 0 | 0 | 0 | 708 | 0 | 0 | 244.53 | 41.32 |

Measured on seeded simulation only. Not a guarantee; zero violations here do not establish zero risk.
