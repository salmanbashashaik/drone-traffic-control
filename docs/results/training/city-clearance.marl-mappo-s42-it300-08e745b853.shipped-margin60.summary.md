# Building clearance of flown tracks: `marl-mappo-s42-it300-08e745b853` (shipped)

26 buildings per layout, each at least 60 m from every straight start-goal route. Measurement only: the controller does not see buildings, and the filter can move a drone further off its straight route than the margin.

| scenario | variant | layouts | min clearance (m) | 5th pct | median | layouts where a track enters a building |
|---|---|---|---|---|---|---|
| crossing-90 | shielded_marl | 20 | 26.1 | 26.4 | 42.9 | 0 |
| crossing-90 | unshielded_marl | 20 | 31.2 | 35.8 | 51.4 | 0 |
| head-on-40 | shielded_marl | 20 | 20.6 | 24.6 | 37.7 | 0 |
| head-on-40 | unshielded_marl | 20 | 32.9 | 34.7 | 47.5 | 0 |
| sandwich-3 | shielded_marl | 20 | 22.7 | 26.3 | 43.9 | 0 |
| sandwich-3 | unshielded_marl | 20 | 29.6 | 32.5 | 46.3 | 0 |
