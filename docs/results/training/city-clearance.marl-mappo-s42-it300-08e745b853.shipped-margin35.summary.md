# Building clearance of flown tracks: `marl-mappo-s42-it300-08e745b853` (shipped)

26 buildings per layout, each at least 35 m from every straight start-goal route. Measurement only: the controller does not see buildings, and the filter can move a drone further off its straight route than the margin.

| scenario | variant | layouts | min clearance (m) | 5th pct | median | layouts where a track enters a building |
|---|---|---|---|---|---|---|
| crossing-90 | shielded_marl | 20 | 1.2 | 5.2 | 29.3 | 0 |
| crossing-90 | unshielded_marl | 20 | 2.1 | 6.7 | 34.2 | 0 |
| head-on-40 | shielded_marl | 20 | 0.0 | 0.0 | 20.6 | 3 |
| head-on-40 | unshielded_marl | 20 | 3.2 | 5.7 | 32.6 | 0 |
| sandwich-3 | shielded_marl | 20 | 0.0 | 0.0 | 30.6 | 2 |
| sandwich-3 | unshielded_marl | 20 | 0.0 | 2.2 | 26.4 | 1 |
