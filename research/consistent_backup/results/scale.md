Latency per filter step: 5 warm-up steps, then 20 timed closed-loop steps from a dense mid-encounter state; limit 10 min per configuration.

| N | Filter | Pairs within 20 m at start | Timed steps | p50 / p95 (ms) | Mean (ms) | Min sep (m) | Fallbacks | Wall (s) |
|---|---|---|---|---|---|---|---|---|
| 6 | main | 10 | 20/20 | 37 / 38 | 37 | 6.580 | 0 | 2 |
| 6 | cbk-lite | 10 | 20/20 | 21 / 24 | 21 | 6.988 | 0 | 1 |
| 6 | cbk | 10 | 20/20 | 49 / 53 | 50 | 6.958 | 0 | 2 |
| 16 | main | 27 | 20/20 | 53 / 57 | 53 | 5.198 | 0 | 2 |
| 16 | cbk-lite | 27 | 20/20 | 66 / 77 | 67 | 4.636 | 0 | 3 |
| 16 | cbk | 27 | 20/20 | 273 / 296 | 279 | 5.223 | 0 | 8 |
| 64 | main | 56 | 20/20 | 366 / 388 | 366 | 5.615 | 0 | 10 |
| 64 | cbk-lite | 56 | 20/20 | 1290 / 1664 | 1310 | 5.647 | 0 | 33 |
| 64 | cbk | 56 | 20/20 | 4276 / 4776 | 4407 | 5.520 | 0 | 112 |

Decentralised per-drone estimate (ESTIMATE, not a measurement of a decentralised filter): own rollouts + k pair margin matrices + a one-drone QP with 5k rows; selection and messaging not included.

| Library | Options | Own rollouts (ms) | One pair margin (ms) | QP k=6 (ms) | Per drone, k=6 (ms) | QP k=15 (ms) | Per drone, k=15 (ms) |
|---|---|---|---|---|---|---|---|
| full | 54 | 11.7 | 1.62 | 3.0 | 24 | 3.3 | 39 |
| lite | 12 | 11.3 | 0.10 | 2.6 | 15 | 3.0 | 16 |
