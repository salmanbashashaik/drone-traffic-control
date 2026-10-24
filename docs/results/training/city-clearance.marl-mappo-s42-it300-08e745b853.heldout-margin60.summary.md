# Building clearance of flown tracks: `marl-mappo-s42-it300-08e745b853` (heldout)

26 buildings per layout, each at least 60 m from every straight start-goal route. Measurement only: the controller does not see buildings, and the filter can move a drone further off its straight route than the margin.

| scenario | variant | layouts | min clearance (m) | 5th pct | median | layouts where a track enters a building |
|---|---|---|---|---|---|---|
| heldout (all) | shielded_marl | 50 | 47.7 | 49.7 | 63.8 | 0 |
| heldout (all) | unshielded_marl | 50 | 47.7 | 49.7 | 64.1 | 0 |
