Rows: 2336; commits: f01961701893.

Risk target per pair-hour in each heading. Absorb: runs with a margin deficit (cycles, largest deficit m). Wind at t0: runs whose first cycle raised the wind alarm (prior). `absorb` and `margin` are listed but do not count as a warning before LoS.

## Target 1e-05 per pair-hour (K1 = 6.31, K2 = 6.63)

### In-limit conditions (recoverable and admissible encounters only)

| condition | mode | runs | LoS | runs with alarm | LoS, monitor silent | alarms | wind at t0 | absorb | runs saturated | margin p50 (m) | GNSS / lost-command term p50 (m) | min sep (m) | min sep from clearance (m) | certificate violations: runs (cycles, deepest m) | fallback runs |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| gps_sps/calm | adaptive | 39 | 0 | 39 | 0 | headroom 22, margin 39, wall 30 | 0 | 0 (0, 0.00) | 0 | 48.57 | 46.76 / 24.65 | 32.20 | 32.20 | 4 (9, 0.03) | 0 |
| gps_sps/calm | baseline | 39 | 35 | - | - | - | - | - | 6 | 0.5 | - | 0.09 | 0.09 | 8 (28, 1.23) | 0 |
| gps_sps/calm | baseline_lag | 39 | 35 | - | - | - | - | - | 6 | 0.5 | - | 0.09 | 0.09 | 8 (28, 1.23) | 0 |
| gps_sps/india | adaptive | 22 | 0 | 22 | 0 | margin 22, wall 12 | 22 | 0 (0, 0.00) | 0 | 48.57 | 44.6 / 24.65 | 40.55 | 40.55 | 4 (8, 0.04) | 0 |
| gps_sps/india | baseline | 22 | 1 | - | - | - | - | - | 0 | 0.5 | - | 2.54 | 2.54 | 1 (1, 0.00) | 0 |
| gps_sps/india | baseline_lag | 22 | 1 | - | - | - | - | - | 0 | 0.5 | - | 2.54 | 2.54 | 1 (1, 0.00) | 0 |
| rtk_f9p/calm | adaptive | 45 | 0 | 31 | 0 | headroom 25, wall 15 | 0 | 0 (0, 0.00) | 0 | 24.89 | 0.28 / 24.47 | 27.56 | 27.56 | 0 (0, 0.00) | 0 |
| rtk_f9p/calm | baseline | 45 | 17 | - | - | - | - | - | 20 | 0.5 | - | 3.84 | 3.84 | 27 (222, 1.65) | 6 |
| rtk_f9p/calm | baseline_lag | 45 | 17 | - | - | - | - | - | 20 | 0.5 | - | 3.68 | 3.68 | 27 (221, 1.80) | 6 |
| rtk_f9p/india | adaptive | 43 | 0 | 12 | 0 | absorb 11, nis 3, wall 1 | 43 | 11 (15, 0.13) | 0 | 34.68 | 10.07 / 24.47 | 36.59 | 36.59 | 14 (20, 0.03) | 0 |
| rtk_f9p/india | baseline | 43 | 1 | - | - | - | - | - | 1 | 0.5 | - | 4.79 | 4.79 | 6 (29, 0.70) | 0 |
| rtk_f9p/india | baseline_lag | 43 | 1 | - | - | - | - | - | 1 | 0.5 | - | 4.79 | 4.79 | 6 (29, 0.70) | 0 |
| sbas_gagan/calm | adaptive | 40 | 0 | 40 | 0 | headroom 22, lag 1, margin 40, wall 24 | 0 | 0 (0, 0.00) | 0 | 48.57 | 30.62 / 24.65 | 43.40 | 43.40 | 6 (10, 0.02) | 0 |
| sbas_gagan/calm | baseline | 40 | 33 | - | - | - | - | - | 6 | 0.5 | - | 0.28 | 0.28 | 12 (51, 1.58) | 1 |
| sbas_gagan/calm | baseline_lag | 40 | 33 | - | - | - | - | - | 6 | 0.5 | - | 0.28 | 0.28 | 12 (51, 1.58) | 1 |
| sbas_gagan/india | adaptive | 29 | 0 | 29 | 0 | margin 29, wall 16 | 29 | 0 (0, 0.00) | 0 | 48.57 | 31.96 / 24.65 | 45.69 | 45.69 | 8 (20, 0.06) | 0 |
| sbas_gagan/india | baseline | 29 | 2 | - | - | - | - | - | 0 | 0.5 | - | 2.54 | 2.54 | 1 (3, 0.11) | 0 |
| sbas_gagan/india | baseline_lag | 29 | 2 | - | - | - | - | - | 0 | 0.5 | - | 2.54 | 2.54 | 1 (3, 0.11) | 0 |

Excluded: 6 adaptive runs not recoverable (main, ground truth, no delay); 64 recoverable but not admissible at clearance (separation below 5 m + requested margin along the backup).

### Beyond-limit conditions (adaptive)

| case | runs | alarm before any LoS | LoS | runs with alarm | LoS, monitor silent | alarms | wind at t0 | absorb | first alarm (s, median) | first lag alarm (s, median) |
|---|---|---|---|---|---|---|---|---|---|---|
| B1_squall_11mps | 16 | 16 | 0 | 16 | 0 | headroom 14, margin 16, saturation 4, wall 9, wind 16 | 16 | 0 (0, 0.00) | 2.0 | - |
| B2_gust_front_8mps | 16 | 16 | 0 | 16 | 0 | margin 16, nis 16, saturation 11, wall 16, wind 13 | 16 | 0 (0, 0.00) | 9.6 | - |
| B3_gnss_underreports_5x | 16 | 16 | 0 | 16 | 0 | lag 1, margin 16, nis 16, saturation 12, wall 14 | 16 | 0 (0, 0.00) | 2.0 | 6.3 |
| B4_downlink_out_3s | 16 | 16 | 0 | 16 | 0 | link 16, margin 16, saturation 3, wall 10 | 16 | 0 (0, 0.00) | 9.3 | - |
| B5_uplink_out_1.5s | 16 | 16 | 0 | 16 | 0 | link 16, margin 16, saturation 2 | 16 | 0 (0, 0.00) | 9.4 | - |
| B6_lag_2x | 16 | 14 | 0 | 16 | 0 | lag 9, margin 16, nis 13, saturation 1, wall 9 | 16 | 0 (0, 0.00) | 7.4 | 7.3 |
| B6_lag_2x_rtk | 16 | 16 | 0 | 16 | 0 | absorb 3, lag 10, nis 16 | 16 | 3 (6, 0.01) | 8.0 | 9.2 |
| B7_gps_worst_site | 16 | 14 | 0 | 16 | 0 | margin 16, saturation 1, wall 14 | 16 | 0 (0, 0.00) | 7.7 | - |
| B8_bias_5sigma | 16 | 13 | 0 | 16 | 0 | margin 16, wall 13 | 16 | 0 (0, 0.00) | 12.0 | - |
| B8_bias_5sigma_rtk | 16 | 0 | 0 | 5 | 0 | absorb 5 | 16 | 5 (6, 0.08) | - | - |

### Takeoff tracks (adaptive, track starts 15 s before the run's t = 0, at hover)

| condition | runs | LoS | runs with alarm | LoS, monitor silent | alarms | wind at t0 | absorb | alarm in the last 5 s before the encounter | margin p50 (m) | min sep (m) | min sep from clearance (m) | certificate violations: runs (cycles, deepest m) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| takeoff/gps_sps/india | 43 | 0 | 43 | 0 | lag 2, margin 43, saturation 2, wall 35 | 43 | 0 (0, 0.00) | 7 | 48.57 | 31.37 | 31.37 | 11 (36, 0.07) |
| takeoff/rtk_f9p/india | 45 | 0 | 20 | 0 | absorb 15, lag 3, nis 2, wall 9 | 45 | 15 (28, 0.15) | 6 | 34.64 | 36.89 | 36.89 | 13 (27, 0.03) |
| takeoff/sbas_gagan/india | 43 | 0 | 43 | 0 | lag 2, margin 43, saturation 1, wall 31 | 43 | 0 (0, 0.00) | 5 | 48.57 | 43.65 | 43.65 | 9 (33, 0.05) |

### Corridor capacity per GNSS grade (India wind, adaptive, in-limit runs; reference corridor 100 m x 30 m, square lane grid at s = 5 m + margin, lanes at least the margin from the walls)

| grade | margin p50 of runs (m) | lane spacing s (m) | lanes | drones per km of corridor |
|---|---|---|---|---|
| rtk_f9p | 34.68 | 39.7 | 0 | 0 |
| sbas_gagan | 48.57 | 53.6 | 0 | 0 |
| gps_sps | 48.57 | 53.6 | 0 | 0 |

## Target 1e-07 per pair-hour (K1 = 6.99, K2 = 7.30)

### In-limit conditions (recoverable and admissible encounters only)

| condition | mode | runs | LoS | runs with alarm | LoS, monitor silent | alarms | wind at t0 | absorb | runs saturated | margin p50 (m) | GNSS / lost-command term p50 (m) | min sep (m) | min sep from clearance (m) | certificate violations: runs (cycles, deepest m) | fallback runs |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| gps_sps/calm | adaptive | 39 | 0 | 39 | 0 | headroom 22, margin 39, wall 30 | 0 | 0 (0, 0.00) | 0 | 48.57 | 51.78 / 24.72 | 32.20 | 32.20 | 4 (9, 0.03) | 0 |
| gps_sps/calm | baseline | 39 | 35 | - | - | - | - | - | 6 | 0.5 | - | 0.09 | 0.09 | 8 (28, 1.23) | 0 |
| gps_sps/calm | baseline_lag | 39 | 35 | - | - | - | - | - | 6 | 0.5 | - | 0.09 | 0.09 | 8 (28, 1.23) | 0 |
| gps_sps/india | adaptive | 14 | 0 | 14 | 0 | margin 14, wall 7 | 14 | 0 (0, 0.00) | 0 | 48.57 | 46.43 / 24.72 | 44.28 | 44.28 | 2 (2, 0.01) | 0 |
| gps_sps/india | baseline | 14 | 1 | - | - | - | - | - | 0 | 0.5 | - | 4.99 | 4.99 | 0 (0, 0.00) | 0 |
| gps_sps/india | baseline_lag | 14 | 1 | - | - | - | - | - | 0 | 0.5 | - | 4.99 | 4.99 | 0 (0, 0.00) | 0 |
| rtk_f9p/calm | adaptive | 45 | 0 | 31 | 0 | headroom 25, wall 15 | 0 | 0 (0, 0.00) | 0 | 24.97 | 0.31 / 24.52 | 27.64 | 27.64 | 0 (0, 0.00) | 0 |
| rtk_f9p/calm | baseline | 45 | 17 | - | - | - | - | - | 20 | 0.5 | - | 3.84 | 3.84 | 27 (222, 1.65) | 6 |
| rtk_f9p/calm | baseline_lag | 45 | 17 | - | - | - | - | - | 20 | 0.5 | - | 3.68 | 3.68 | 27 (221, 1.80) | 6 |
| rtk_f9p/india | adaptive | 43 | 0 | 19 | 0 | absorb 15, nis 6, wall 1 | 43 | 15 (17, 0.13) | 0 | 35.83 | 11.17 / 24.52 | 37.60 | 37.60 | 15 (27, 0.03) | 0 |
| rtk_f9p/india | baseline | 43 | 1 | - | - | - | - | - | 1 | 0.5 | - | 4.86 | 4.86 | 5 (32, 0.63) | 0 |
| rtk_f9p/india | baseline_lag | 43 | 1 | - | - | - | - | - | 1 | 0.5 | - | 4.86 | 4.86 | 5 (32, 0.63) | 0 |
| sbas_gagan/calm | adaptive | 39 | 0 | 39 | 0 | headroom 22, lag 1, margin 39, wall 24 | 0 | 0 (0, 0.00) | 0 | 48.57 | 33.91 / 24.72 | 43.40 | 43.40 | 6 (10, 0.02) | 0 |
| sbas_gagan/calm | baseline | 39 | 32 | - | - | - | - | - | 6 | 0.5 | - | 0.28 | 0.28 | 12 (51, 1.58) | 1 |
| sbas_gagan/calm | baseline_lag | 39 | 32 | - | - | - | - | - | 6 | 0.5 | - | 0.28 | 0.28 | 12 (51, 1.58) | 1 |
| sbas_gagan/india | adaptive | 23 | 0 | 23 | 0 | margin 23, wall 9 | 23 | 0 (0, 0.00) | 0 | 48.57 | 35.28 / 24.72 | 45.88 | 45.88 | 4 (6, 0.04) | 0 |
| sbas_gagan/india | baseline | 23 | 2 | - | - | - | - | - | 0 | 0.5 | - | 2.49 | 2.49 | 1 (1, 0.04) | 0 |
| sbas_gagan/india | baseline_lag | 23 | 2 | - | - | - | - | - | 0 | 0.5 | - | 2.49 | 2.49 | 1 (1, 0.04) | 0 |

Excluded: 6 adaptive runs not recoverable (main, ground truth, no delay); 79 recoverable but not admissible at clearance (separation below 5 m + requested margin along the backup).

### Beyond-limit conditions (adaptive)

| case | runs | alarm before any LoS | LoS | runs with alarm | LoS, monitor silent | alarms | wind at t0 | absorb | first alarm (s, median) | first lag alarm (s, median) |
|---|---|---|---|---|---|---|---|---|---|---|
| B1_squall_11mps | 16 | 16 | 0 | 16 | 0 | headroom 13, margin 16, saturation 6, wall 9, wind 16 | 16 | 0 (0, 0.00) | 2.0 | - |
| B2_gust_front_8mps | 16 | 16 | 0 | 16 | 0 | margin 16, nis 16, saturation 12, wall 16, wind 13 | 16 | 0 (0, 0.00) | 9.6 | - |
| B3_gnss_underreports_5x | 16 | 16 | 0 | 16 | 0 | margin 16, nis 16, saturation 11, wall 13 | 16 | 0 (0, 0.00) | 2.0 | - |
| B4_downlink_out_3s | 16 | 16 | 0 | 16 | 0 | link 16, margin 16, saturation 2, wall 9 | 16 | 0 (0, 0.00) | 9.3 | - |
| B5_uplink_out_1.5s | 16 | 16 | 0 | 16 | 0 | link 16, margin 16, saturation 3 | 16 | 0 (0, 0.00) | 9.4 | - |
| B6_lag_2x | 16 | 14 | 0 | 16 | 0 | lag 10, margin 16, nis 13, saturation 2, wall 9 | 16 | 0 (0, 0.00) | 7.2 | 6.9 |
| B6_lag_2x_rtk | 16 | 16 | 0 | 16 | 0 | absorb 2, lag 8, nis 16 | 16 | 2 (3, 0.06) | 8.9 | 8.9 |
| B7_gps_worst_site | 16 | 14 | 0 | 16 | 0 | margin 16, saturation 2, wall 14 | 16 | 0 (0, 0.00) | 7.4 | - |
| B8_bias_5sigma | 16 | 13 | 0 | 16 | 0 | margin 16, wall 13 | 16 | 0 (0, 0.00) | 11.9 | - |
| B8_bias_5sigma_rtk | 16 | 0 | 0 | 5 | 0 | absorb 5 | 16 | 5 (6, 0.11) | - | - |

### Takeoff tracks (adaptive, track starts 15 s before the run's t = 0, at hover)

| condition | runs | LoS | runs with alarm | LoS, monitor silent | alarms | wind at t0 | absorb | alarm in the last 5 s before the encounter | margin p50 (m) | min sep (m) | min sep from clearance (m) | certificate violations: runs (cycles, deepest m) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| takeoff/gps_sps/india | 43 | 0 | 43 | 0 | lag 2, margin 43, wall 35 | 43 | 0 (0, 0.00) | 9 | 48.57 | 32.29 | 32.29 | 13 (41, 0.10) |
| takeoff/rtk_f9p/india | 45 | 0 | 17 | 0 | absorb 15, lag 2, nis 2, saturation 1, wall 5 | 45 | 15 (24, 0.14) | 7 | 35.81 | 37.86 | 37.86 | 15 (31, 0.06) |
| takeoff/sbas_gagan/india | 43 | 0 | 43 | 0 | lag 2, margin 43, wall 28 | 43 | 0 (0, 0.00) | 5 | 48.57 | 43.82 | 43.82 | 15 (44, 0.05) |

### Corridor capacity per GNSS grade (India wind, adaptive, in-limit runs; reference corridor 100 m x 30 m, square lane grid at s = 5 m + margin, lanes at least the margin from the walls)

| grade | margin p50 of runs (m) | lane spacing s (m) | lanes | drones per km of corridor |
|---|---|---|---|---|
| rtk_f9p | 35.83 | 40.8 | 0 | 0 |
| sbas_gagan | 48.57 | 53.6 | 0 | 0 |
| gps_sps | 48.57 | 53.6 | 0 | 0 |

