Rows: 1168; commits: 486f3b4bd66c.

Absorb: runs with a margin deficit (cycles, largest deficit m). Wind at t0: runs whose first cycle raised the wind alarm (prior).

### In-limit conditions (recoverable encounters only)

| condition | mode | runs | LoS | runs with alarm | LoS, monitor silent | alarms | wind at t0 | absorb | runs saturated | margin p50 (m) | min sep (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| gps_sps/calm | adaptive | 47 | 0 | 45 | 0 | absorb 17, headroom 43, lag 1, saturation 6, wall 34 | 0 | 17 (183, 16.37) | 6 | 38.95 | 9.68 |
| gps_sps/calm | baseline | 47 | 43 | - | - | - | - | - | 10 | 0.5 | 0.09 |
| gps_sps/calm | baseline_lag | 47 | 43 | - | - | - | - | - | 10 | 0.5 | 0.09 |
| gps_sps/india | adaptive | 47 | 0 | 46 | 0 | absorb 18, headroom 46, lag 6, nis 1, saturation 4, wall 40 | 47 | 18 (189, 14.04) | 4 | 38.95 | 9.68 |
| gps_sps/india | baseline | 47 | 10 | - | - | - | - | - | 3 | 0.5 | 1.22 |
| gps_sps/india | baseline_lag | 47 | 10 | - | - | - | - | - | 3 | 0.5 | 1.22 |
| rtk_f9p/calm | adaptive | 47 | 0 | 43 | 0 | absorb 4, headroom 43, lag 1, saturation 3 | 0 | 4 (22, 3.16) | 3 | 1.34 | 6.52 |
| rtk_f9p/calm | baseline | 47 | 17 | - | - | - | - | - | 20 | 0.5 | 1.23 |
| rtk_f9p/calm | baseline_lag | 47 | 17 | - | - | - | - | - | 20 | 0.5 | 1.23 |
| rtk_f9p/india | adaptive | 47 | 0 | 46 | 0 | absorb 2, headroom 46, lag 6, nis 4, wall 1 | 47 | 2 (6, 0.54) | 0 | 1.43 | 6.60 |
| rtk_f9p/india | baseline | 47 | 2 | - | - | - | - | - | 3 | 0.5 | 4.62 |
| rtk_f9p/india | baseline_lag | 47 | 2 | - | - | - | - | - | 3 | 0.5 | 4.62 |
| sbas_gagan/calm | adaptive | 47 | 0 | 43 | 0 | absorb 13, headroom 43, lag 1, saturation 2, wall 23 | 0 | 13 (93, 4.89) | 2 | 23.26 | 9.68 |
| sbas_gagan/calm | baseline | 47 | 39 | - | - | - | - | - | 11 | 0.5 | 0.28 |
| sbas_gagan/calm | baseline_lag | 47 | 39 | - | - | - | - | - | 11 | 0.5 | 0.28 |
| sbas_gagan/india | adaptive | 47 | 0 | 46 | 0 | absorb 13, headroom 46, lag 6, nis 1, saturation 2, wall 27 | 47 | 13 (97, 3.14) | 2 | 23.28 | 9.69 |
| sbas_gagan/india | baseline | 47 | 11 | - | - | - | - | - | 3 | 0.5 | 1.23 |
| sbas_gagan/india | baseline_lag | 47 | 11 | - | - | - | - | - | 3 | 0.5 | 1.23 |

Scenarios excluded as not recoverable (main, ground truth, no delay): 1.

### Beyond-limit conditions (adaptive)

| case | runs | alarm before any LoS | LoS | runs with alarm | LoS, monitor silent | alarms | wind at t0 | absorb | first alarm (s, median) | first lag alarm (s, median) |
|---|---|---|---|---|---|---|---|---|---|---|
| B1_squall_11mps | 16 | 16 | 0 | 16 | 0 | headroom 16, lag 15, saturation 2, wall 4, wind 16 | 16 | 0 (0, 0.00) | 2.0 | 2.2 |
| B2_gust_front_8mps | 16 | 16 | 0 | 16 | 0 | absorb 14, headroom 15, lag 10, nis 16, saturation 9, wall 14, wind 9 | 16 | 14 (82, 1.91) | 9.6 | 9.6 |
| B3_gnss_underreports_5x | 16 | 16 | 0 | 16 | 0 | absorb 9, headroom 16, lag 2, nis 16, saturation 2, wall 7 | 16 | 9 (35, 0.92) | 2.0 | 2.3 |
| B4_downlink_out_3s | 16 | 16 | 0 | 16 | 0 | absorb 2, headroom 15, link 16, wall 9 | 16 | 2 (5, 0.54) | 9.9 | - |
| B5_uplink_out_1.5s | 16 | 16 | 0 | 16 | 0 | absorb 15, headroom 15, lag 3, link 16, nis 16, saturation 12, wall 14 | 16 | 15 (231, 3.63) | 9.4 | 14.3 |
| B6_lag_2x | 16 | 16 | 0 | 16 | 0 | absorb 6, headroom 15, lag 16, nis 15, saturation 2, wall 13 | 16 | 6 (30, 0.70) | 9.9 | 10.5 |
| B6_lag_2x_rtk | 16 | 14 | 0 | 16 | 0 | headroom 15, lag 14, nis 12 | 16 | 0 (0, 0.00) | 11.4 | 11.5 |
| B7_gps_worst_site | 16 | 16 | 0 | 16 | 0 | headroom 15, lag 2, margin 16, wall 16 | 16 | 0 (0, 0.00) | 2.0 | 8.3 |
| B8_bias_5sigma | 16 | 6 | 2 | 16 | 1 | absorb 3, headroom 15, wall 6 | 16 | 3 (13, 1.69) | 16.4 | - |
| B8_bias_5sigma_rtk | 16 | 0 | 0 | 15 | 0 | headroom 15 | 16 | 0 (0, 0.00) | - | - |

### Takeoff tracks (adaptive, track starts 15 s before the run's t = 0, at hover)

| condition | runs | LoS | runs with alarm | LoS, monitor silent | alarms | wind at t0 | absorb | alarm in the last 5 s before the encounter | margin p50 (m) |
|---|---|---|---|---|---|---|---|---|---|
| takeoff/gps_sps/india | 47 | 0 | 47 | 0 | absorb 20, headroom 46, lag 9, nis 1, saturation 18, wall 43 | 47 | 20 (166, 4.02) | 17 | 39.13 |
| takeoff/rtk_f9p/india | 47 | 0 | 47 | 0 | absorb 2, headroom 47, lag 11, nis 8, saturation 15, wall 6 | 47 | 2 (13, 1.96) | 4 | 4.22 |
| takeoff/sbas_gagan/india | 47 | 0 | 46 | 0 | absorb 11, headroom 46, lag 8, nis 1, saturation 15, wall 28 | 47 | 11 (78, 2.11) | 11 | 23.53 |

### Corridor capacity per GNSS grade (India wind, adaptive, in-limit runs; reference corridor 100 m x 30 m, square lane grid at s = 5 m + margin)

| grade | margin p50 of runs (m) | lane spacing s (m) | lanes | drones per km of corridor |
|---|---|---|---|---|
| rtk_f9p | 1.43 | 6.4 | 80 | 12400 |
| sbas_gagan | 23.28 | 28.3 | 8 | 280 |
| gps_sps | 38.95 | 44.0 | 3 | 66 |
