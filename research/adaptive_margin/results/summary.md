Rows: 2336; commits: e06d9414dd95.

Risk target per pair-hour in each heading. Absorb: runs with a margin deficit (cycles, largest deficit m). Wind at t0: runs whose first cycle raised the wind alarm (prior). `absorb` and `margin` are listed but do not count as a warning before LoS.

## Target 1e-05 per pair-hour (K1 = 6.31, K2 = 6.63)

### In-limit conditions (recoverable and admissible encounters only)

| condition | mode | runs | LoS | runs with alarm | LoS, monitor silent | alarms | wind at t0 | absorb | runs saturated | margin p50 (m) | GNSS / lost-command term p50 (m) | min sep (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| gps_sps/calm | adaptive | 43 | 0 | 41 | 0 | absorb 3, headroom 24, lag 1, margin 38, wall 33 | 0 | 3 (15, 0.59) | 0 | 48.57 | 46.76 / 7.56 | 32.20 |
| gps_sps/calm | baseline | 43 | 39 | - | - | - | - | - | 7 | 0.5 | - | 0.09 |
| gps_sps/calm | baseline_lag | 43 | 39 | - | - | - | - | - | 7 | 0.5 | - | 0.09 |
| gps_sps/india | adaptive | 37 | 0 | 34 | 0 | absorb 3, lag 1, margin 33, saturation 1, wall 23 | 37 | 3 (3, 0.05) | 1 | 48.57 | 45.21 / 7.67 | 37.28 |
| gps_sps/india | baseline | 37 | 5 | - | - | - | - | - | 0 | 0.5 | - | 1.84 |
| gps_sps/india | baseline_lag | 37 | 5 | - | - | - | - | - | 0 | 0.5 | - | 1.84 |
| rtk_f9p/calm | adaptive | 47 | 0 | 29 | 0 | headroom 27, lag 1, wall 4 | 0 | 0 (0, 0.00) | 0 | 8.03 | 0.29 / 7.55 | 12.83 |
| rtk_f9p/calm | baseline | 47 | 18 | - | - | - | - | - | 22 | 0.5 | - | 3.51 |
| rtk_f9p/calm | baseline_lag | 47 | 18 | - | - | - | - | - | 22 | 0.5 | - | 3.51 |
| rtk_f9p/india | adaptive | 45 | 0 | 11 | 0 | absorb 8, nis 5 | 45 | 8 (15, 0.11) | 0 | 18.07 | 10.26 / 7.63 | 22.81 |
| rtk_f9p/india | baseline | 45 | 1 | - | - | - | - | - | 1 | 0.5 | - | 4.79 |
| rtk_f9p/india | baseline_lag | 45 | 1 | - | - | - | - | - | 1 | 0.5 | - | 4.79 |
| sbas_gagan/calm | adaptive | 44 | 0 | 37 | 0 | absorb 18, headroom 24, lag 1, saturation 1, wall 26 | 0 | 18 (227, 0.91) | 1 | 39.44 | 31.8 / 7.55 | 31.17 |
| sbas_gagan/calm | baseline | 44 | 37 | - | - | - | - | - | 7 | 0.5 | - | 0.28 |
| sbas_gagan/calm | baseline_lag | 44 | 37 | - | - | - | - | - | 7 | 0.5 | - | 0.28 |
| sbas_gagan/india | adaptive | 43 | 0 | 25 | 0 | absorb 20, saturation 1, wall 15 | 43 | 20 (125, 0.39) | 1 | 40.75 | 32.74 / 7.67 | 33.98 |
| sbas_gagan/india | baseline | 43 | 10 | - | - | - | - | - | 1 | 0.5 | - | 1.50 |
| sbas_gagan/india | baseline_lag | 43 | 10 | - | - | - | - | - | 1 | 0.5 | - | 1.50 |

Excluded: 6 adaptive runs not recoverable (main, ground truth, no delay); 23 recoverable but not admissible at clearance (separation below 5 m + requested margin along the backup).

### Beyond-limit conditions (adaptive)

| case | runs | alarm before any LoS | LoS | runs with alarm | LoS, monitor silent | alarms | wind at t0 | absorb | first alarm (s, median) | first lag alarm (s, median) |
|---|---|---|---|---|---|---|---|---|---|---|
| B1_squall_11mps | 16 | 16 | 0 | 16 | 0 | headroom 14, margin 16, saturation 4, wall 9, wind 16 | 16 | 0 (0, 0.00) | 2.0 | - |
| B2_gust_front_8mps | 16 | 16 | 0 | 16 | 0 | absorb 16, lag 1, nis 16, saturation 11, wall 14, wind 13 | 16 | 16 (124, 0.35) | 9.6 | 10.1 |
| B3_gnss_underreports_5x | 16 | 16 | 0 | 16 | 0 | absorb 14, nis 16, saturation 10, wall 13 | 16 | 14 (162, 1.21) | 2.0 | - |
| B4_downlink_out_3s | 16 | 16 | 0 | 16 | 0 | absorb 16, link 16, wall 10 | 16 | 16 (138, 1.13) | 9.3 | - |
| B5_uplink_out_1.5s | 16 | 16 | 0 | 16 | 0 | absorb 12, link 16, saturation 3 | 16 | 12 (50, 0.13) | 9.4 | - |
| B6_lag_2x | 16 | 16 | 0 | 16 | 0 | absorb 6, lag 13, nis 10, saturation 2, wall 9 | 16 | 6 (84, 0.44) | 9.1 | 8.0 |
| B6_lag_2x_rtk | 16 | 16 | 0 | 16 | 0 | absorb 3, lag 5, nis 16 | 16 | 3 (10, 0.10) | 10.4 | 10.2 |
| B7_gps_worst_site | 16 | 14 | 0 | 16 | 0 | margin 16, saturation 1, wall 14 | 16 | 0 (0, 0.00) | 7.7 | - |
| B8_bias_5sigma | 16 | 13 | 0 | 13 | 0 | absorb 8, saturation 1, wall 13 | 16 | 8 (40, 0.44) | 13.5 | - |
| B8_bias_5sigma_rtk | 16 | 0 | 0 | 3 | 0 | absorb 3 | 16 | 3 (4, 0.10) | - | - |

### Takeoff tracks (adaptive, track starts 15 s before the run's t = 0, at hover)

| condition | runs | LoS | runs with alarm | LoS, monitor silent | alarms | wind at t0 | absorb | alarm in the last 5 s before the encounter | margin p50 (m) |
|---|---|---|---|---|---|---|---|---|---|
| takeoff/gps_sps/india | 44 | 0 | 42 | 0 | absorb 4, lag 2, margin 42, saturation 3, wall 36 | 44 | 4 (27, 0.55) | 8 | 48.57 |
| takeoff/rtk_f9p/india | 46 | 0 | 17 | 0 | absorb 14, lag 3, nis 3 | 46 | 14 (18, 0.15) | 7 | 17.8 |
| takeoff/sbas_gagan/india | 45 | 0 | 39 | 0 | absorb 27, lag 3, saturation 3, wall 27 | 45 | 27 (186, 0.60) | 19 | 40.02 |

### Corridor capacity per GNSS grade (India wind, adaptive, in-limit runs; reference corridor 100 m x 30 m, square lane grid at s = 5 m + margin, lanes at least the margin from the walls)

| grade | margin p50 of runs (m) | lane spacing s (m) | lanes | drones per km of corridor |
|---|---|---|---|---|
| rtk_f9p | 18.07 | 23.1 | 0 | 0 |
| sbas_gagan | 40.75 | 45.8 | 0 | 0 |
| gps_sps | 48.57 | 53.6 | 0 | 0 |

## Target 1e-07 per pair-hour (K1 = 6.99, K2 = 7.30)

### In-limit conditions (recoverable and admissible encounters only)

| condition | mode | runs | LoS | runs with alarm | LoS, monitor silent | alarms | wind at t0 | absorb | runs saturated | margin p50 (m) | GNSS / lost-command term p50 (m) | min sep (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| gps_sps/calm | adaptive | 42 | 0 | 42 | 0 | headroom 23, lag 1, margin 42, wall 32 | 0 | 0 (0, 0.00) | 0 | 48.57 | 51.78 / 7.57 | 32.20 |
| gps_sps/calm | baseline | 42 | 38 | - | - | - | - | - | 6 | 0.5 | - | 0.09 |
| gps_sps/calm | baseline_lag | 42 | 38 | - | - | - | - | - | 6 | 0.5 | - | 0.09 |
| gps_sps/india | adaptive | 29 | 0 | 29 | 0 | margin 29, saturation 1, wall 17 | 29 | 0 (0, 0.00) | 1 | 48.57 | 49.2 / 7.69 | 37.05 |
| gps_sps/india | baseline | 29 | 3 | - | - | - | - | - | 0 | 0.5 | - | 2.49 |
| gps_sps/india | baseline_lag | 29 | 3 | - | - | - | - | - | 0 | 0.5 | - | 2.49 |
| rtk_f9p/calm | adaptive | 47 | 0 | 29 | 0 | headroom 27, lag 1, wall 4 | 0 | 0 (0, 0.00) | 0 | 8.06 | 0.32 / 7.56 | 12.91 |
| rtk_f9p/calm | baseline | 47 | 18 | - | - | - | - | - | 22 | 0.5 | - | 3.51 |
| rtk_f9p/calm | baseline_lag | 47 | 18 | - | - | - | - | - | 22 | 0.5 | - | 3.51 |
| rtk_f9p/india | adaptive | 44 | 0 | 14 | 0 | absorb 9, nis 7, saturation 1 | 44 | 9 (17, 0.13) | 1 | 19.22 | 11.47 / 7.65 | 23.91 |
| rtk_f9p/india | baseline | 44 | 1 | - | - | - | - | - | 1 | 0.5 | - | 4.86 |
| rtk_f9p/india | baseline_lag | 44 | 1 | - | - | - | - | - | 1 | 0.5 | - | 4.86 |
| sbas_gagan/calm | adaptive | 43 | 0 | 35 | 0 | absorb 16, headroom 24, saturation 2, wall 26 | 0 | 16 (241, 1.08) | 2 | 42.23 | 34.51 / 7.56 | 34.17 |
| sbas_gagan/calm | baseline | 43 | 36 | - | - | - | - | - | 7 | 0.5 | - | 0.28 |
| sbas_gagan/calm | baseline_lag | 43 | 36 | - | - | - | - | - | 7 | 0.5 | - | 0.28 |
| sbas_gagan/india | adaptive | 43 | 0 | 28 | 0 | absorb 21, saturation 1, wall 20 | 43 | 21 (94, 0.23) | 1 | 43.94 | 36.17 / 7.69 | 36.99 |
| sbas_gagan/india | baseline | 43 | 11 | - | - | - | - | - | 0 | 0.5 | - | 1.39 |
| sbas_gagan/india | baseline_lag | 43 | 11 | - | - | - | - | - | 0 | 0.5 | - | 1.39 |

Excluded: 6 adaptive runs not recoverable (main, ground truth, no delay); 34 recoverable but not admissible at clearance (separation below 5 m + requested margin along the backup).

### Beyond-limit conditions (adaptive)

| case | runs | alarm before any LoS | LoS | runs with alarm | LoS, monitor silent | alarms | wind at t0 | absorb | first alarm (s, median) | first lag alarm (s, median) |
|---|---|---|---|---|---|---|---|---|---|---|
| B1_squall_11mps | 16 | 16 | 0 | 16 | 0 | headroom 13, margin 16, saturation 6, wall 9, wind 16 | 16 | 0 (0, 0.00) | 2.0 | - |
| B2_gust_front_8mps | 16 | 16 | 0 | 16 | 0 | absorb 16, lag 1, nis 16, saturation 11, wall 14, wind 13 | 16 | 16 (112, 0.90) | 9.5 | 9.2 |
| B3_gnss_underreports_5x | 16 | 16 | 0 | 16 | 0 | absorb 15, nis 16, saturation 9, wall 12 | 16 | 15 (174, 0.79) | 2.0 | - |
| B4_downlink_out_3s | 16 | 16 | 0 | 16 | 0 | absorb 16, link 16, margin 9, wall 9 | 16 | 16 (126, 0.66) | 9.3 | - |
| B5_uplink_out_1.5s | 16 | 16 | 0 | 16 | 0 | absorb 8, link 16, saturation 1 | 16 | 8 (28, 0.09) | 9.4 | - |
| B6_lag_2x | 16 | 12 | 0 | 12 | 0 | absorb 8, lag 7, nis 10, saturation 3, wall 8 | 16 | 8 (75, 0.51) | 7.3 | 7.2 |
| B6_lag_2x_rtk | 16 | 16 | 0 | 16 | 0 | absorb 4, lag 8, nis 16 | 16 | 4 (5, 0.05) | 10.6 | 10.6 |
| B7_gps_worst_site | 16 | 14 | 0 | 16 | 0 | margin 16, saturation 2, wall 14 | 16 | 0 (0, 0.00) | 7.4 | - |
| B8_bias_5sigma | 16 | 13 | 0 | 13 | 0 | absorb 8, wall 13 | 16 | 8 (49, 0.32) | 13.1 | - |
| B8_bias_5sigma_rtk | 16 | 0 | 0 | 3 | 0 | absorb 3 | 16 | 3 (5, 0.12) | - | - |

### Takeoff tracks (adaptive, track starts 15 s before the run's t = 0, at hover)

| condition | runs | LoS | runs with alarm | LoS, monitor silent | alarms | wind at t0 | absorb | alarm in the last 5 s before the encounter | margin p50 (m) |
|---|---|---|---|---|---|---|---|---|---|
| takeoff/gps_sps/india | 43 | 0 | 43 | 0 | lag 2, margin 43, wall 35 | 43 | 0 (0, 0.00) | 9 | 48.57 |
| takeoff/rtk_f9p/india | 45 | 0 | 17 | 0 | absorb 14, lag 2, nis 2, saturation 1 | 45 | 14 (20, 0.12) | 10 | 18.95 |
| takeoff/sbas_gagan/india | 44 | 0 | 35 | 0 | absorb 22, lag 2, saturation 4, wall 27 | 44 | 22 (157, 0.59) | 14 | 43.36 |

### Corridor capacity per GNSS grade (India wind, adaptive, in-limit runs; reference corridor 100 m x 30 m, square lane grid at s = 5 m + margin, lanes at least the margin from the walls)

| grade | margin p50 of runs (m) | lane spacing s (m) | lanes | drones per km of corridor |
|---|---|---|---|---|
| rtk_f9p | 19.22 | 24.2 | 0 | 0 |
| sbas_gagan | 43.94 | 48.9 | 0 | 0 |
| gps_sps | 48.57 | 53.6 | 0 | 0 |

