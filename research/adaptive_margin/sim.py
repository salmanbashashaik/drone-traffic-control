"""Closed loop for P3-SA-03: plant, links, ground station and scenarios.

The plant runs at H = 10 ms ticks: first-order actuator lag, airspeed drag
D (w - v) with w the wind (mean wind plus per-drone Dryden-like gusts), the
airframe's tilt authority, speed cap, and PX4's offboard-loss braking. Each
drone is one registered airframe (``envelope.FLEET``). Telemetry (GNSS
position and velocity, IMU acceleration, and the echo of the applied
commands) goes down at 10 Hz; thrust commands go up once per 100 ms filter
cycle and play out at a fixed delay (P3-SA-03 §2.2, §4).

The filter's output u is an acceleration in the contract's drag-free model
(v' = u); the station turns it into a thrust command. Three ground
stations share the plant and every random draw:

- ``adaptive``: trackers, prediction to the next command's playout,
  adaptive margin raised no faster than the barrier absorbs, envelope
  monitor, main's ``apply_filter`` on the predicted states with the backup
  rollout flown through the lag, drag and thrust limit (``rollout.py``);
  thrust u - D (w_hat - v_hat).
- ``baseline``: main as it is, on the latest raw measurement, margin 0.5 m,
  no prediction; thrust u + D v_route, the trim that holds the planned
  velocity in still air (it knows no wind).
- ``baseline_lag``: as ``baseline`` with ``lag_s`` = the declared tau.

Every scenario is flown at the cleared cruise speed (``cleared``): each
drone's planned speed is capped where its thrust, after the drag at the
forecast wind, still gives the braking the margin assumes (review B3).

src/ is not modified; the filter is called as any caller would, with its
backup rollout replaced by ``rollout.backup_constraints`` in adaptive mode.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
from shared.contracts import (
    MAX_SPEED_MPS,
    SAFE_SEPARATION_M,
    WORLD_XY_MAX_M,
    WORLD_XY_MIN_M,
    WORLD_Z_MAX_M,
    WORLD_Z_MIN_M,
    DroneState,
    MARLAction,
)
from utm_services.safety import filter as F

from research.adaptive_margin import envelope as E
from research.adaptive_margin import rollout
from research.adaptive_margin.margin import (
    CYCLE,
    P_TICKS,
    TARGETS,
    U_MAX,
    Pred,
    Risk,
    absorbable,
    margin,
    monitor,
    risk,
)
from research.adaptive_margin.tracker import H, CommandLog, Tracker, kin_step

T_C_S = 12.0                     # time to the scenario's conflict point
EPISODE_S = T_C_S + 10.0
Z_CENTRE_M = 115.0
_LO = np.array([WORLD_XY_MIN_M, WORLD_XY_MIN_M, WORLD_Z_MIN_M])
_HI = np.array([WORLD_XY_MAX_M, WORLD_XY_MAX_M, WORLD_Z_MAX_M])


@dataclass(frozen=True)
class Cond:
    """Operating condition of one run. Defaults are in-model."""

    name: str
    gnss: E.Gnss
    wind_mps: float = 0.0            # mean wind U, horizontal, random direction
    w20_mps: float = 0.0             # Dryden turbulence class (0: no gusts)
    white_factor: float = 0.0        # >0: GNSS white noise at this multiple of sigma, no bias (B3)
    gust_step_mps: float = 0.0       # step added to drone 0's wind at OUTAGE_S (B2)
    down_out_s: float = 0.0          # downlink outage length from OUTAGE_S (B4)
    up_out_s: float = 0.0            # uplink outage length from OUTAGE_S (B5)
    tau_true_s: float = E.TAU_ACT_S  # plant actuator lag (B6)
    bias_sigma: float = 0.0          # >0: GNSS bias of this Mahalanobis length, random direction (B8)
    preroll_s: float = 0.0           # >0: the track starts this long earlier, at hover (takeoff runs)
    forecast: tuple[float, float] | None = None   # (U, W20) the clearance used, if not the true wind


OUTAGE_S = T_C_S - 3.0
# Track convergence: no clearance before the track has converged. The wind
# prior at the track's start is the full forecast speed in an unknown
# direction, so the monitor is read from WARMUP_S on (20 telemetry
# messages, two NIS windows); alarms of the first cycle are reported apart.
WARMUP_S = 2.0
# Takeoff runs: hover, then accelerate along the route to the cruise velocity.
PREROLL_S, HOVER_S, ROUTE_ACCEL_MPS2 = 15.0, 3.0, 2.0

CALM, INDIA = dict(wind_mps=0.0), dict(wind_mps=E.INDIA_MEAN_100M_MPS, w20_mps=E.W20_LIGHT_MPS)
NOMINAL = [Cond(f"{g.name}/{w}", g, **kw) for g in (E.RTK, E.SBAS, E.SPS)
           for w, kw in (("calm", CALM), ("india", INDIA))]
BEYOND = [
    Cond("B1_squall_11mps", E.SBAS, wind_mps=E.SQUALL_PEAK_MPS, w20_mps=E.W20_MODERATE_MPS,
         forecast=(E.INDIA_MEAN_100M_MPS, E.W20_LIGHT_MPS)),
    Cond("B2_gust_front_8mps", E.SBAS, gust_step_mps=E.SQUALL_RISE_MPS, **INDIA),
    Cond("B3_gnss_underreports_5x", E.SBAS, white_factor=5.0, **INDIA),
    Cond("B4_downlink_out_3s", E.SBAS, down_out_s=3.0, **INDIA),
    Cond("B5_uplink_out_1.5s", E.SBAS, up_out_s=1.5, **INDIA),
    Cond("B6_lag_2x", E.SBAS, tau_true_s=2.0 * E.TAU_ACT_S, **INDIA),
    Cond("B6_lag_2x_rtk", E.RTK, tau_true_s=2.0 * E.TAU_ACT_S, **INDIA),
    Cond("B7_gps_worst_site", E.SPS_WORST, **INDIA),
    Cond("B8_bias_5sigma", E.SBAS, bias_sigma=5.0, **INDIA),
    Cond("B8_bias_5sigma_rtk", E.RTK, bias_sigma=5.0, **INDIA),
]
TAKEOFF = [Cond(f"takeoff/{g.name}/india", g, preroll_s=PREROLL_S, **INDIA) for g in (E.RTK, E.SBAS, E.SPS)]
FAMILIES = ("headon2", "cross2", "climb2", "ring4", "ring6", "random")


def scenario(family: str, seed: int) -> tuple[np.ndarray, np.ndarray]:
    """Drones that meet near one point at T_C_S if nobody deviates: (p0, v0)."""
    rng = np.random.default_rng([FAMILIES.index(family), seed])
    centre = np.array([rng.uniform(400, 600), rng.uniform(400, 600), Z_CENTRE_M])
    th0 = rng.uniform(0, 2 * math.pi)
    climb = np.zeros(6)
    if family == "headon2":
        th = [th0, th0 + math.pi]
    elif family == "cross2":
        th = [th0, th0 + math.pi / 2]
    elif family == "climb2":
        th = [th0, th0 + rng.uniform(math.pi / 2, math.pi)]
        climb[1] = math.radians(15.0) * rng.choice([-1.0, 1.0])
    elif family.startswith("ring"):
        n = int(family[4:])
        th = [th0 + 2 * math.pi * k / n for k in range(n)]
    else:
        th = list(rng.uniform(0, 2 * math.pi, rng.integers(3, 7)))
    n = len(th)
    s = rng.uniform(5.0, 15.0, n)
    v0 = np.stack([s * np.cos(th) * np.cos(climb[:n]), s * np.sin(th) * np.cos(climb[:n]),
                   s * np.sin(climb[:n])], axis=1)
    off = rng.normal(0.0, [2.0, 2.0, 1.0], (n, 3))
    return centre + off - v0 * T_C_S, v0


def _fleet(seed: int, n: int) -> list[E.Airframe]:
    """The run's airframes: the first draw of the run's generator, as in ``episode``."""
    return [E.FLEET[k] for k in np.random.default_rng([seed, 7]).integers(0, len(E.FLEET), n)]


def cleared_speed(af: E.Airframe, cond: Cond, rk: Risk) -> float:
    """Highest cruise speed at which the airframe keeps the braking the margin assumes (review B3).

    The stopping time T (``margin.stopping_s``) assumes each drone can brake
    at u_max |v| / v_max on top of cancelling its drag. With airspeed at most
    W + v, the thrust left is a_phys - D (W + v), so the cleared speed solves
    a_phys - D (W + v) = u_max v / v_max, with W the forecast mean wind plus
    K2 sigma of its gusts. An operating rule: the station plans no faster.
    The ``headroom`` alarm checks the same inequality on the estimates.
    """
    U, w20 = cond.forecast or (cond.wind_mps, cond.w20_mps)
    sigma_u = E.dryden(w20, Z_CENTRE_M)[0] if w20 > 0.0 and U > 0.0 else 0.0
    D = af.drag_per_s
    v = (af.a_phys_mps2 - D * (U + rk.k2 * sigma_u)) / (D + U_MAX / MAX_SPEED_MPS)
    assert v > 0.0, f"{af.name} cannot hold the braking in {cond.name}"
    return min(MAX_SPEED_MPS, v)


def cleared(p0: np.ndarray, v0: np.ndarray, cond: Cond, rk: Risk, seed: int):
    """The scenario at the cleared speeds, meeting at the same point and time: (p0, v0, v_clr per drone)."""
    vc = np.array([cleared_speed(af, cond, rk) for af in _fleet(seed, len(p0))])
    v1 = v0 * np.minimum(1.0, vc / np.linalg.norm(v0, axis=1))[:, None]
    return p0 + (v0 - v1) * T_C_S, v1, vc


def recoverable(p0: np.ndarray, v0: np.ndarray) -> float:
    """Min separation of main on ground truth, no disturbance, no delay (fuzz dynamics)."""
    from research.consistent_backup import harness
    # Main's own backup rollout: an adaptive episode leaves its drones in CTX (re-review R1).
    rollout.CTX.clear()
    # The harness caps speed at exactly MAX_SPEED_MPS, which rounding can put a
    # hair above the contract's limit; cap a hair below, as kin_step's callers do.
    harness.MAX_SPEED_MPS = MAX_SPEED_MPS * (1.0 - 1e-9)
    return harness.kinematic("main", p0, v0, int(EPISODE_S / 0.1))["min_sep"]


def _state(aid: str, p: np.ndarray, v: np.ndarray, tick: int) -> DroneState:
    """DroneState from an estimate, clipped into the contract's box and speed limit."""
    p = np.clip(p, _LO, _HI)
    s = float(np.linalg.norm(v))
    v = v * min(1.0, MAX_SPEED_MPS * (1.0 - 1e-9) / max(s, 1e-9))
    return DroneState(agent_id=aid, position_m=tuple(map(float, p)), velocity_mps=tuple(map(float, v)),
                      goal_m=(900.0, 900.0, 100.0), timestamp_ms=tick * 10)


def _authority(c: np.ndarray, af: E.Airframe) -> tuple[np.ndarray, bool]:
    """Clip a thrust command to the airframe: horizontal disc of radius a_phys, |c_z| <= a_phys.

    shortcut: vertical authority taken equal to the horizontal; the spec
    pages give no climb acceleration. Revisit with thrust-to-weight data.
    """
    a = af.a_phys_mps2
    h = float(np.linalg.norm(c[:2]))
    out = c.copy()
    if h > a:
        out[:2] *= a / h
    out[2] = min(max(c[2], -a), a)
    return out, bool(h > a or abs(c[2]) > a)


def _route(v0: np.ndarray, preroll_s: float):
    """Planned velocity and acceleration at time t (s from the track's start), and the start position offset.

    Without a preroll the route is v0 throughout. With one, the drone hovers
    for HOVER_S, accelerates along v0 at ROUTE_ACCEL_MPS2, then cruises,
    reaching the scenario's state (p0, v0) at t = preroll_s.
    """
    if preroll_s == 0.0:
        return (lambda t: v0), (lambda t: np.zeros(3)), np.zeros(3)
    t_acc = float(np.linalg.norm(v0)) / ROUTE_ACCEL_MPS2
    acc = v0 / t_acc

    def vel(t: float) -> np.ndarray:
        return v0 * min(max((t - HOVER_S) / t_acc, 0.0), 1.0)

    def accel(t: float) -> np.ndarray:
        return acc if HOVER_S <= t < HOVER_S + t_acc else np.zeros(3)

    return vel, accel, -v0 * (preroll_s - HOVER_S - t_acc / 2.0)


@dataclass
class _Out:
    min_sep: float = math.inf
    los_s: float | None = None
    alarms: dict[str, float] = field(default_factory=dict)
    alarms_t0: list[str] = field(default_factory=list)
    margins: list[float] = field(default_factory=list)
    requested: list[float] = field(default_factory=list)
    gnss: list[float] = field(default_factory=list)
    lost: list[float] = field(default_factory=list)
    admissible: bool | None = None
    t0_req: float | None = None
    t0_star: float | None = None
    alarm_cycles: int = 0
    absorb_cycles: int = 0
    absorb_max: float = 0.0
    saturated_cycles: int = 0
    fallbacks: int = 0
    solver_errors: int = 0
    cap_pairs: int = 0


def episode(p0: np.ndarray, v0: np.ndarray, cond: Cond, mode: str, seed: int, target: float = TARGETS[0]) -> dict:
    """One closed-loop run; ``mode`` is "adaptive", "baseline" or "baseline_lag"; ``target`` risk per pair-hour."""
    rng = np.random.default_rng([seed, 7])
    n = len(p0)
    ids = [f"d{i}" for i in range(n)]
    fleet = [E.FLEET[k] for k in rng.integers(0, len(E.FLEET), n)]
    rk = risk(target)
    pairs = [(ids[i], ids[j]) for i in range(n) for j in range(i + 1, n)]
    rollout.CTX.clear()   # baseline modes run main's own backup rollout
    D = np.array([af.drag_per_s for af in fleet])
    g = cond.gnss
    sig_p = np.array([g.sigma_h_m, g.sigma_h_m, g.sigma_v_m])
    routes = [_route(v0[i], cond.preroll_s) for i in range(n)]
    p = np.array([p0[i] + routes[i][2] for i in range(n)], dtype=float)
    v = np.array([routes[i][0](0.0) for i in range(n)], dtype=float)
    # Wind: mean U in a random horizontal direction, plus independent per-drone gusts.
    phi_w = rng.uniform(0, 2 * math.pi)
    W = cond.wind_mps * np.array([math.cos(phi_w), math.sin(phi_w), 0.0])
    if cond.w20_mps > 0.0 and cond.wind_mps > 0.0:
        su, sw, lu, lw = E.dryden(cond.w20_mps, Z_CENTRE_M)
        sig_g, tau_g = np.array([su, su, sw]), np.array([lu, lu, lw]) / cond.wind_mps
    else:
        sig_g, tau_g = np.zeros(3), np.ones(3)
    gust = rng.normal(0.0, sig_g, (n, 3))
    e_g = np.exp(-H / tau_g)
    step_dir = np.array([v0[0, 0], v0[0, 1], 0.0]) / max(float(np.linalg.norm(v0[0, :2])), 1e-9)
    # GNSS: constant per-episode bias (in model), a fixed 5-sigma one (B8), or white noise (B3).
    if cond.white_factor:
        bias = np.zeros((n, 3))
    elif cond.bias_sigma:
        u = rng.normal(size=(n, 3))
        bias = cond.bias_sigma * sig_p * u / np.linalg.norm(u, axis=1, keepdims=True)
    else:
        bias = rng.normal(0.0, sig_p, (n, 3))
    t0_tick = round(cond.preroll_s / H)
    out_from, down_to, up_to = (t0_tick + round(OUTAGE_S / H), t0_tick + round((OUTAGE_S + cond.down_out_s) / H),
                                t0_tick + round((OUTAGE_S + cond.up_out_s) / H))

    def wind(t: int) -> np.ndarray:
        w = W + gust
        if cond.gust_step_mps and t >= out_from:
            w[0] = w[0] + cond.gust_step_mps * step_dir
        return w

    def measure(i: int, w_i: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        noise = rng.normal(0.0, sig_p * cond.white_factor) if cond.white_factor else bias[i]
        z = np.concatenate([p[i] + noise, v[i] + rng.normal(0.0, g.sigma_vel_mps, 3)])
        return z, a[i] + D[i] * (w_i - v[i]) + rng.normal(0.0, E.ACC_NOISE_MPS2, 3)

    # The drone starts trimmed: thrust cancels the drag at its velocity.
    a = -D[:, None] * (wind(0) - v)
    c_now = a.copy()
    # Ground station's wind knowledge: forecast U and turbulence class (P3-SA-03 §4.3).
    q = np.maximum(2.0 * sig_g**2 / tau_g, 1e-6)
    p_ww0 = np.maximum(cond.wind_mps**2 + sig_g**2, 1e-2)
    z0 = [measure(i, wind(0)[i])[0] for i in range(n)]
    trackers = [Tracker(g, q, E.TAU_ACT_S, D[i], 0, z0[i], c_now[i], p_ww0) for i in range(n)]
    logs = [CommandLog(c_now[i].copy()) for i in range(n)]
    latest = [(0, z) for z in z0]
    echo: list[list[tuple[int, np.ndarray]]] = [[] for _ in range(n)]
    last_rx = np.zeros(n, dtype=int)
    down: list[tuple] = []                                     # (arrival, drone, t_m, z, a_meas, echo)
    up: dict[int, list[tuple[int, np.ndarray]]] = {}           # playout tick -> (drone, c)
    latch: dict = {}
    m_prev = 0.0
    out = _Out()

    def alarm(name: str, t: int) -> None:
        out.alarms.setdefault(name, round(t * H, 2))

    def latency_ticks() -> tuple[bool, int]:
        d = E.LTE_AIR_ONE_WAY_MEDIAN_S * math.exp(E.LTE_AIR_SIGMA_LN * rng.standard_normal())
        return rng.random() < E.LINK_LOSS_3GPP, max(1, math.ceil(d / H - 1e-9))

    for t in range(t0_tick + round(EPISODE_S / H)):
        w_now = wind(t)
        if t % CYCLE == 0:
            if t > 0:
                for i in range(n):
                    lost, dt = latency_ticks()
                    z, am = measure(i, w_now[i])
                    if not lost and not out_from <= t < down_to:
                        down.append((t + dt, i, t, z, am, list(echo[i])))
            # --- ground station ---
            ready = sorted((m for m in down if m[0] <= t), key=lambda m: m[2])
            down = [m for m in down if m[0] > t]
            for _, i, tm, z, am, ech in ready:
                if mode == "adaptive":
                    logs[i].confirm(ech, tm)
                    trackers[i].update(tm, z, logs[i], am)
                elif tm > latest[i][0]:
                    latest[i] = (tm, z)
            t_play = (t + P_TICKS) * H
            acts = [MARLAction(agent_id=aid, proposed_accel_mps2=tuple(map(float, routes[i][1](t_play))),
                               timestamp_ms=t * 10) for i, aid in enumerate(ids)]
            if mode == "adaptive":
                preds = {}
                for i, aid in enumerate(ids):
                    x, P, a0 = trackers[i].predict(t + P_TICKS, logs[i], floor=True)
                    preds[aid] = Pred(x, P, trackers[i], logs[i], fleet[i], a0)
                    rollout.CTX[aid] = (a0, D[i], x[6:], fleet[i].a_phys_mps2)
                states = [_state(aid, pr.x[:3], pr.x[3:6], t) for aid, pr in preds.items()]
                # The backup each pair would fly now, through the lag (review B1, B4).
                traj = rollout.pair_rollout({s.agent_id: s for s in states}, pairs,
                                            F.FilterConfig(margin_m=m_prev), latch)[0]
                res = margin(preds, rk, {pr: traj[:, q] for q, pr in enumerate(pairs)})
                # Raise the margin no faster than the barrier absorbs (P3-SA-03 §3.9).
                m_star = absorbable(traj)
                m_app = min(res.margin_m, max(m_star, m_prev))
                deficit = res.margin_m - m_app
                if deficit > 1e-9:
                    res.alarms.add("absorb")
                monitor(preds, res, t + P_TICKS, rk, m_app)
                cfg = F.FilterConfig(margin_m=m_app)
                m_prev = m_app
                if t == 0:
                    out.alarms_t0 = sorted(res.alarms)
                if t * H >= WARMUP_S and out.admissible is None:
                    # Admissible (review B2): at clearance the states satisfy the margin the pair needs.
                    out.admissible, out.t0_req, out.t0_star = m_star >= res.requested_m, res.requested_m, m_star
                if t * H >= WARMUP_S:
                    out.margins.append(m_app)
                    out.requested.append(res.requested_m)
                    out.gnss.append(res.gnss_m)
                    out.lost.append(res.lost_m)
                    if deficit > 1e-9:
                        out.absorb_cycles += 1
                        out.absorb_max = max(out.absorb_max, deficit)
            else:
                res = None
                states = [_state(aid, latest[i][1][:3], latest[i][1][3:], t) for i, aid in enumerate(ids)]
                cfg = F.FilterConfig(lag_s=E.TAU_ACT_S if mode == "baseline_lag" else 0.0)
            safe, diag = F.apply_filter(states, acts, cfg, latch=latch)
            out.fallbacks += int(diag.fallback_used)
            out.solver_errors += int(not diag.qp_feasible)
            out.cap_pairs += len(diag.backup_rollout_cap_pairs)
            by_id = {s.agent_id: np.array(s.safe_accel_mps2) for s in safe}
            saturated = False
            for i, aid in enumerate(ids):
                u = by_id[aid]
                if res is not None:
                    c, sat = _authority(u - preds[aid].drag, fleet[i])
                    if sat:
                        res.alarms.add("saturation")
                    logs[i].plan(t + P_TICKS, c)
                else:
                    c, sat = _authority(u + D[i] * routes[i][0](t_play), fleet[i])
                saturated |= sat
                lost, dt = latency_ticks()
                if not lost and dt <= P_TICKS and not out_from <= t < up_to:
                    up.setdefault(t + P_TICKS, []).append((i, c))
            if t * H >= WARMUP_S:
                out.saturated_cycles += saturated
                if res is not None:
                    for name in res.alarms:
                        alarm(name, t)
                    out.alarm_cycles += bool(res.alarms)
        # --- plant ---
        for i, c in up.pop(t, []):
            c_now[i] = c
            echo[i].append((t, c))
            last_rx[i] = t
        for i in range(n):
            if (t - last_rx[i]) * H > E.OFFBOARD_LOSS_S:   # PX4 offboard loss: brake, autopilot trims its own drag
                c_now[i] = _authority(np.clip(-v[i] / 0.1, -U_MAX, U_MAX) - D[i] * (w_now[i] - v[i]), fleet[i])[0]
                echo[i].append((t, c_now[i].copy()))   # the drone reports what it applies (review N1)
            p[i], v[i], a[i] = kin_step(p[i], v[i], a[i], c_now[i], w_now[i], D[i], cond.tau_true_s)
        gust = gust * e_g + sig_g * np.sqrt(1.0 - e_g**2) * rng.standard_normal((n, 3))
        sep = min(float(np.linalg.norm(p[i] - p[j])) for i in range(n) for j in range(i + 1, n))
        out.min_sep = min(out.min_sep, sep)
        if sep < SAFE_SEPARATION_M and out.los_s is None:
            out.los_s = round((t + 1) * H, 2)
    m = np.array(out.margins) if out.margins else np.zeros(1)

    def p50(xs: list[float]):
        return round(float(np.median(xs)), 2) if xs else None

    return {
        "min_sep": round(out.min_sep, 3), "los_s": out.los_s, "alarms": out.alarms,
        "first_alarm_s": min(out.alarms.values(), default=None), "alarm_cycles": out.alarm_cycles,
        "alarms_t0": out.alarms_t0 if mode == "adaptive" else None,
        "margin_p50": round(float(np.median(m)), 2), "margin_max": round(float(m.max()), 2),
        "margin_req_max": round(max(out.requested, default=0.0), 2),
        "gnss_p50": p50(out.gnss), "lost_p50": p50(out.lost), "target": target,
        "admissible": out.admissible, "m_req_t0": out.t0_req and round(out.t0_req, 2),
        "m_star_t0": out.t0_star and round(out.t0_star, 2),
        "absorb_cycles": out.absorb_cycles, "absorb_max_m": round(out.absorb_max, 3),
        "saturated_cycles": out.saturated_cycles,
        "fallbacks": out.fallbacks, "solver_errors": out.solver_errors, "cap_pairs": out.cap_pairs,
        "uplink_lost": int(sum(lg.lost for lg in logs)) if mode == "adaptive" else None,
        "fleet": [af.name for af in fleet], "t_encounter_s": cond.preroll_s + T_C_S,
    }
