# Command link protocol notes (design only)

Status: design notes, nothing here is implemented. The PX4 SITL bridge (`PX4_SETUP.md`) uses plain MAVLink offboard
setpoints through MAVSDK. These notes list what a filter-to-vehicle link needs beyond that, based on what the SITL runs showed.
A change to any message here that crosses `shared/contracts.py` is a contract version change and goes to the founder.

"CBK" is read here as **committed backup**: the backup manoeuvre the backup-set barrier certified for a drone at a step.
If the founder meant something else by CBK, section 4 needs a rewrite.

## 1. What the SITL runs showed

| Observation | Number | Source |
|---|---|---|
| Loop latency, telemetry snapshot to last setpoint sent | p95 7.2 ms (headon2), 20.7 ms (cross3); max 33.9 ms | `docs/results/px4/*/summary.json` |
| Telemetry age when the step reads it | p95 16 to 19 ms, max 24.7 ms | same |
| Step period | 100 ms, no catch-up; a step over 500 ms late aborts | `px4_bridge.py` |
| Realised acceleration vs commanded | median 84 to 87 %, p10 50 % | `PX4_SETUP.md` §7 |
| PX4 offboard-loss timeout `COM_OF_LOSS_T` | 1.0 s | manifests |
| Failsafe after the last *fresh* setpoint | 1.35 s, because MAVSDK re-sent the stale setpoint until the process exited | `PX4_SETUP.md` §11 |
| Failsafe action without RC | RTL at 31 m, filter out of the loop | same |

The main finding: the vehicle cannot tell a fresh command from a re-sent one. MAVLink offboard setpoints carry no
sequence number and no expiry, and MAVSDK keeps streaming the last one. A bridge that hangs without exiting keeps a stale command alive with no time limit.

## 2. Per-command acknowledgement

- The vehicle side acknowledges each command with `(drone_id, seq, status, t_rx_vehicle)`. `status` is one of
  `applied`, `expired`, `out_of_order`, `rejected_limits`, `not_in_mode`.
- The filter side tracks the last acknowledged `seq` per drone. Missing ACKs for more than `k_ack` steps (proposal: 3 steps, 300 ms)
  mark the drone **unconfirmed**. The filter then treats that drone as flying its committed backup (section 4), not its last command, in every pair it is in.
- ACKs go on the telemetry stream that already runs at 10 Hz or faster, so they add no extra message rate. A setpoint stream without ACK
  can still be used, but then the filter can only assume the worst case of section 4 for every drone.
- MAVLink has `COMMAND_ACK` for commands, not for `SET_POSITION_TARGET_LOCAL_NED` streams. This needs a custom message
  or a companion process on the vehicle that echoes the applied `seq`.

## 3. Validity windows and sequence numbers

Each command carries:

| Field | Meaning |
|---|---|
| `seq` | per-drone counter, +1 every step, never reused in a session |
| `t_valid_from`, `t_valid_until` | vehicle-clock window in which the command may be applied |
| `session_id` | changes on every bridge restart, so an old `seq` is never mistaken for a new one |

Vehicle rules:

1. Apply a command only if `seq` is greater than the last applied `seq` of the same session. Drop it otherwise (`out_of_order`).
2. Apply it only inside its window. After `t_valid_until` with no newer command, switch to the committed backup (section 4), not to the last command and not to RTL.
3. A re-sent command keeps its `seq` and window, so re-sending can never extend its life. This closes the 1.35 s vs 1.0 s gap seen in SITL.

Proposal for the window: `t_valid_until = t_sent + 2 × step = 200 ms`. One lost command is then tolerated, and a second loss
hands the drone to its backup. Clock offset between ground and vehicle must be estimated (for example MAVLink `TIMESYNC`) and
counted in the window. The 200 ms is a design proposal, not a threshold; choosing it is a founder decision.

## 4. CBK: the committed backup broadcast

The backup-set barrier already computes, for each pair, a backup trajectory: both drones fly the evasive manoeuvre
(`_evasive_direction`, latched frame) for 66 to 109 steps, and the QP keeps that trajectory outside `d_eff`. So at every step the filter
holds a certified "what to do if commands stop" for each drone. Today that exists only inside the filter. CBK makes it a message.

Each step, per drone, the filter sends with the setpoint:

| Field | Meaning |
|---|---|
| `seq`, `session_id` | as in section 3; the CBK belongs to this command |
| `backup_accel_profile` | the backup acceleration for the drone, as the latched frame plus the per-step clipped acceleration, or a compact form (escape direction + world-box limits) the vehicle can re-run |
| `horizon_steps` | rollout length used (66 to 109) |
| `t_commit` | time from which the backup applies if no newer valid command arrives |
| `rollout_capped` | true when the rollout hit the cap, so the backup is weaker than certified |

Vehicle rule: when the current command expires (section 3, rule 2), fly the last received CBK from `t_commit`, then hold.

Open issues that block CBK from being a safety claim:

- **BL-03.** The backup is per pair today. A drone with several neighbours has a different escape against each, so there is no single
  backup to broadcast. CBK needs the per-drone backup of BL-03 first. Until then a CBK is only valid for drones in at most one conflict pair.
- **All drones must switch together.** The barrier certifies the pair when *both* drones fly the backup. A drone that loses link while its neighbour keeps
  receiving nominal commands breaks the assumption. The filter side must therefore also switch the neighbours of an unconfirmed drone to their
  CBK (section 2), and the vehicles need a way to learn that, for example a short drone-to-drone broadcast of `(drone_id, seq, mode)`.
- **Tracking.** The backup assumes the plant flies the acceleration exactly (`lag_s = 0`). PX4 realises a median 84 to 87 % in SITL. The backup needs the same
  lag and tracking margin as the nominal path.
- **Vertical limits.** The evasive vertical component is up to 3.5 m/s², but PX4's default vertical speed limits are 3 m/s up and 1.5 m/s down.

## 5. Latency budget (one 100 ms step)

| Segment | Measured in SITL | Budget proposal | Note |
|---|---|---|---|
| Telemetry age at read (vehicle estimate to bridge) | p95 16 to 19 ms | 30 ms | local UDP; a radio link adds its own latency |
| Bridge loop (convert, policy, filter, send) | p95 7 to 21 ms, max 34 ms (2 to 3 drones) | 40 ms | the filter grows with N; at N = 64 `apply_filter` p95 is 2136 ms (DECISIONS.md), which does not fit |
| Command uplink (bridge to vehicle) | not measured (local UDP) | 20 ms | |
| Vehicle response to a new setpoint | 0.5 to 1 s to steady tracking (whole-phase vs last-1 s gap in `PX4_SETUP.md` §7) | modelled in `lag_s`, not in the step | today `lag_s = 0`, a mismatch |
| Total, telemetry to setpoint applied | ≈ 25 to 45 ms in SITL | ≤ 100 ms | |
| Command validity | n/a | 200 ms (section 3) | |
| Offboard-loss failsafe | 1.0 s after the last packet, 1.35 s after the last fresh command | replaced by the validity window + CBK | RTL today |

Rules:

- The telemetry age plus loop latency is the age of the state the filter acts on. It should be logged per step (the bridge already logs both)
  and added to the filter's state uncertainty when it exceeds the budget.
- A step over budget sends no command, so the vehicle's validity window decides what happens, instead of a late command arriving with stale state.
- The bridge already aborts and lands at 500 ms of stale telemetry or step delay. With validity windows and CBK, that abort becomes a fall-back to CBK.
