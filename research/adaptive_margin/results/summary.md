### In-limit conditions (recoverable encounters only)

| condition | mode | runs | LoS | LoS, monitor silent | runs with alarm | alarms | margin p50 (m) | min sep (m) |
|---|---|---|---|---|---|---|---|---|
| gps_sps/calm | adaptive | 47 | 0 | 0 | 19 | wall 19 | 15.76 | 9.59 |
| gps_sps/calm | baseline | 47 | 37 | - | - | - | 0.5 | 0.65 |
| gps_sps/india | adaptive | 47 | 0 | 0 | 28 | margin 3, wall 10, wind 27 | 15.84 | 9.69 |
| gps_sps/india | baseline | 47 | 0 | - | - | - | 0.5 | 5.74 |
| rtk_f9p/calm | adaptive | 47 | 0 | 0 | 2 | wall 2 | 1.4 | 6.47 |
| rtk_f9p/calm | baseline | 47 | 7 | - | - | - | 0.5 | 2.53 |
| rtk_f9p/india | adaptive | 47 | 0 | 0 | 29 | margin 1, wall 4, wind 28 | 2.72 | 6.73 |
| rtk_f9p/india | baseline | 47 | 0 | - | - | - | 0.5 | 5.97 |
| sbas_gagan/calm | adaptive | 47 | 0 | 0 | 16 | wall 16 | 12.86 | 9.59 |
| sbas_gagan/calm | baseline | 47 | 37 | - | - | - | 0.5 | 1.48 |
| sbas_gagan/india | adaptive | 47 | 0 | 0 | 28 | margin 3, wall 7, wind 27 | 12.95 | 9.69 |
| sbas_gagan/india | baseline | 47 | 0 | - | - | - | 0.5 | 5.84 |

Scenarios excluded as not recoverable (main, ground truth, no delay): 1.

### Beyond-limit conditions (adaptive)

| case | runs | runs with alarm | alarm before any LoS | LoS | LoS, monitor silent | first alarm (s, median) | alarms |
|---|---|---|---|---|---|---|---|
| B1_squall_11mps | 16 | 16 | 16 | 0 | 0 | 2.0 | authority 16, margin 16, saturation 14, wall 9, wind 16 |
| B2_gust_front_8mps | 16 | 16 | 16 | 0 | 0 | 2.6 | margin 1, nis 16, saturation 3, wall 5, wind 14 |
| B3_gnss_underreports_5x | 16 | 16 | 16 | 0 | 0 | 2.0 | margin 1, nis 16, wind 10 |
| B4_downlink_out_3s | 16 | 16 | 16 | 0 | 0 | 2.6 | link 16, margin 1, wall 4, wind 9 |
| B5_uplink_out_1.5s | 16 | 16 | 16 | 0 | 0 | 2.6 | link 16, margin 1, nis 16, wall 2, wind 16 |
| B6_lag_2x | 16 | 11 | 11 | 0 | 0 | 2.0 | margin 1, wind 11 |
| B7_gps_worst_site | 16 | 16 | 16 | 0 | 0 | 2.0 | margin 16, wall 15, wind 9 |
