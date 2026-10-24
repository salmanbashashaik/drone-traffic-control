# Building clearance of flown tracks: `marl-mappo-s42-it300-08e745b853`

50 scenarios (seeds 12000000+, 2-4 drones), 26 buildings each, placed >= 35 m from every straight start-goal route. Measurement only; the controller does not see buildings.

| variant | episodes | buildings placed (min) | min clearance (m) | 5th pct | median | episodes with a track inside a building |
|---|---|---|---|---|---|---|
| unshielded_marl | 50 | 26 | 21.8 | 30.7 | 39.2 | 0 |
| shielded_marl | 50 | 26 | 21.8 | 30.7 | 39.2 | 0 |
