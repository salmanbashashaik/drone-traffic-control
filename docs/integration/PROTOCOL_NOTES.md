# Command link protocol notes (design only)

Status: design notes, nothing here is implemented. The PX4 SITL bridge (`PX4_SETUP.md`) uses plain MAVLink offboard
setpoints through MAVSDK. These notes list what a filter-to-vehicle link needs beyond that, based on what the SITL runs showed.
A change to any message here that crosses `shared/contracts.py` is a contract version change and goes to the founder.

"CBK" is the consistent backup of P3-SA-02 (`research/consistent-backup`, `docs/formulation/P3-SA-02-consistent-backup.md`):
one backup manoeuvre per drone, chosen jointly with its neighbours (CSA 2021, TBC 2022). It is research, not on main, and its
recommendation is "adapt, do not adopt". Section 4 designs its broadcast message so the link ADR can include it.

## 1. What the SITL runs showed

| Observation | Number | Source |
|---|---|---|
| Loop latency, telemetry snapshot to last setpoint sent | p95 7.5 ms (headon2), 10.3 ms (cross3); max 37.9 ms | `docs/results/px4/*/summary.json` |
| Telemetry age when the step reads it | p95 16 to 19 ms, max 20.4 ms (filter-on runs) | same |
| Step period | 100 ms, no catch-up; a step over 500 ms late aborts | `px4_bridge.py` |
| Realised acceleration vs commanded | median 82 to 86 %, p10 47 to 51 % | `PX4_SETUP.md` §7 |
| PX4 offboard-loss timeout `COM_OF_LOSS_T` | 1.0 s | manifests |
| Failsafe after the last *fresh* setpoint | 1.27 s: MAVSDK re-sent the stale setpoint until the process exited (0.16 s), then PX4's 1.0 s timeout ran | `PX4_SETUP.md` §11 |
| Failsafe action without RC | RTL at 31 m, filter out of the loop | same |

The main finding: the vehicle cannot tell a fresh command from a re-sent one. MAVLink offboard setpoints carry no
sequence number and no expiry, and MAVSDK keeps streaming the last one. A bridge that hangs without exiting keeps a stale command alive with no time limit.

## 2. Per-command acknowledgement

- The vehicle side acknowledges each command with `(drone_id, seq, status, t_rx_vehicle)`. `status` is one of
  `applied`, `expired`, `out_of_order`, `rejected_limits`, `not_in_mode`.
- The filter side tracks the last acknowledged `seq` per drone. Missing ACKs for more than `k_ack` steps (proposal: 3 steps, 300 ms)
  mark the drone **unconfirmed**. The filter then treats that drone as flying its backup (section 4), not its last command, in every pair it is in.
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
2. Apply it only inside its window. After `t_valid_until` with no newer command, switch to its backup (section 4), not to the last command and not to RTL.
3. A re-sent command keeps its `seq` and window, so re-sending can never extend its life. This closes the 1.27 s vs 1.0 s gap seen in SITL.

Proposal for the window: `t_valid_until = t_sent + 2 × step = 200 ms`. One lost command is then tolerated, and a second loss
hands the drone to its backup. Clock offset between ground and vehicle must be estimated (for example MAVLink `TIMESYNC`) and
counted in the window. The 200 ms is a design proposal, not a threshold; choosing it is a founder decision.

## 4. CBK: the backup broadcast

In CBK each drone owns one backup `π_i` that depends only on its own state: candidate 0 brakes to hover; candidates 1 to 52
accelerate at 3.5 m/s² along one of the 26 world-frame sign vectors for 1 s or 2 s, then brake (P3-SA-02 §2.2). The latch holds, per drone,
the current candidate and its manoeuvre clock (the TBC clock τ0). "Every drone flies its backup" is one joint action that satisfies every
pair row and the world box at once (CSA Theorem 1). In the centralised filter one process makes all choices and needs no message. A
decentralised filter, or an onboard layer that must fly the backup after link loss, needs each drone's choice on the link. TBC Theorem 2 needs every
agent to know the others' current manoeuvre (P3-SA-02 §5).

Message `BackupBroadcast`, one per drone per step:

| Field | Type | Meaning |
|---|---|---|
| `drone_id` | str | sender |
| `session_id`, `seq` | as section 3 | ties the broadcast to the command of the same step |
| `candidate` | int 0..52 | chosen backup candidate (P3-SA-02 §2.2 library) |
| `clock_steps` | int | manoeuvre steps already flown (τ0) |
| `t_state` | vehicle time | time of the state the rollout starts from |
| `library_version` | str | candidate library and horizon rules; a receiver with another version rejects the message |

It carries no trajectory: each candidate is a function of the drone's own state, so any receiver with that state and the library can
re-run it (world-box clipping and speed cap included). The message is a few bytes per drone per step.

Who sends it:

- **Centralised (today's architecture):** the server sends each drone its own `(candidate, clock_steps)` next to the setpoint, and the
  drone ACKs both (section 2). After link loss (command window expired, section 3) the drone flies that candidate from that clock, then
  hovers, instead of RTL. Its neighbours, if still linked, stay under the filter, whose rows now treat the lost drone as flying
  its committed backup (section 2). Each neighbour's own backup stays a feasible choice for those rows, because "every drone flies its
  backup" satisfies every row at once (CSA Theorem 1, P3-SA-02 §5).
- **Decentralised:** each drone broadcasts its choice to its neighbours, in the fixed order of CSA Algorithm 1. Each drone selects after the
  drones before it in the order. The step then takes the ordering depth times (selection time + one message delay), which is not measured (P3-SA-02 §6.3).

Missing or late broadcast: default the sender to candidate 0 (brake), which every receiver can compute from the observed state. P3-SA-02 §5 proposes this rule, but it is untested.

Open issues before CBK can be a safety claim:

- CBK itself fails 3 of the 4 acceptance-bar items today (P3-SA-02 §6.1). In particular, 2 of 20 noisy vertical runs enter `d_eff`, and the
  full library takes 273 ms per step at N = 16. Building the message is useful only after the adapted CBK is accepted.
- Delay and loss of the broadcast are not covered by CSA or TBC as used here. A stale `clock_steps` shifts the receiver's rollout by the delay.
  The receiver must advance it by `(now − t_state) / dt`, and the barrier needs a margin for the clock uncertainty.
- Tracking. Backups assume the plant flies 3.5 m/s² exactly. CBK's horizon adds a 5-step lag allowance (P3-SA-02 §2.2), but
  the bridge runs main's filter with `lag_s = 0`, which has no lag term. PX4 realises a median 82 to 86 % in SITL, and its default vertical
  speed limits (3 m/s up, 1.5 m/s down) cap vertical candidates. An onboard backup must be flown by the autopilot's own controller,
  which needs a PX4-side mode or companion process that can execute a candidate.
- Contract: adding `BackupBroadcast` to `shared/contracts.py` is a contract version change (founder), and P3-SA-02 §9 question 7 asks whether to add it now.

## 5. Latency budget (one 100 ms step)

| Segment | Measured in SITL | Budget proposal | Note |
|---|---|---|---|
| Telemetry age at read (vehicle estimate to bridge) | p95 16 to 19 ms | 30 ms | local UDP; a radio link adds its own latency |
| Bridge loop (convert, policy, filter, send) | p95 7 to 10 ms, max 38 ms (2 to 3 drones) | 40 ms | the filter grows with N; at N = 64 `apply_filter` p95 is 2136 ms (DECISIONS.md), which does not fit |
| Command uplink (bridge to vehicle) | not measured (local UDP) | 20 ms | |
| Vehicle response to a new setpoint | 0.5 to 1 s to steady tracking (whole-phase vs last-1 s gap in `PX4_SETUP.md` §7) | modelled in `lag_s`, not in the step | today `lag_s = 0`, a mismatch |
| Total, telemetry to setpoint applied | ≈ 25 to 30 ms at p95 in SITL, under 60 ms in the worst step | ≤ 100 ms | |
| Command validity | n/a | 200 ms (section 3) | |
| Offboard-loss failsafe | 1.0 s after the last packet, 1.27 s after the last fresh command | replaced by the validity window + CBK | RTL today |

Rules:

- The telemetry age plus loop latency is the age of the state the filter acts on. It should be logged per step (the bridge already logs both)
  and added to the filter's state uncertainty when it exceeds the budget.
- A step over budget sends no command, so the vehicle's validity window decides what happens, instead of a late command arriving with stale state.
- The bridge already aborts and lands at 500 ms of stale telemetry or step delay. With validity windows and CBK, that abort would become a switch to each drone's backup.
