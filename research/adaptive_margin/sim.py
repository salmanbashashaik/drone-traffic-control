"""Closed loop for P3-SA-03: plant, links, ground station and scenarios.

The plant runs at H = 10 ms ticks: first-order actuator lag, wind
disturbance (mean wind plus per-drone Dryden-like gusts), speed cap, and
PX4's offboard-loss braking. Telemetry (GNSS position and velocity, plus the
echo of the applied commands) goes down at 10 Hz; commands go up once per
100 ms filter cycle and play out at a fixed delay (P3-SA-03 §2.2, §4).

Two ground stations share the plant and every random draw:

- ``adaptive``: trackers, equivalent-delay prediction, adaptive margin,
  envelope monitor, main's ``apply_filter`` on the predicted states, and
  disturbance feedforward.
- ``baseline``: main as it is, on the latest raw measurement, margin 0.5 m,
  no prediction and no feedforward.

src/ is not modified; the filter is called as any caller would.
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
from research.adaptive_margin.margin import U_MAX, Pred, margin, monitor
from research.adaptive_margin.tracker import H, CommandLog, Tracker, kin_step

CYCLE = 10                       # ticks per 100 ms filter cycle
T_C_S = 12.0                     # time to the scenario's conflict point
EPISODE_S = T_C_S + 10.0
Z_CENTRE_M = 115.0
# Uplink playout delay: the 99.9 % quantile of the one-way latency, rounded up
# to the tick. A command arriving later is dropped (counted as lost).
P_TICKS = math.ceil(
    E.LTE_AIR_ONE_WAY_MEDIAN_S * math.exp(E.LTE_AIR_SIGMA_LN * 3.090232) / H - 1e-9
)
A_TICKS = round(E.TAU_ACT_S / H)
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


OUTAGE_S = T_C_S - 3.0
# Entry check: a drone joins only once its tracker has converged; the wind
# prior at t = 0 is the full forecast speed in an unknown direction, so the
# monitor is read from WARMUP_S on (20 telemetry messages, two NIS windows).
WARMUP_S = 2.0

CALM, INDIA = dict(wind_mps=0.0), dict(wind_mps=E.INDIA_MEAN_100M_MPS, w20_mps=E.W20_LIGHT_MPS)
NOMINAL = [Cond(f"{g.name}/{w}", g, **kw) for g in (E.RTK, E.SBAS, E.SPS)
           for w, kw in (("calm", CALM), ("india", INDIA))]
BEYOND = [
    Cond("B1_squall_11mps", E.SBAS, wind_mps=E.SQUALL_PEAK_MPS, w20_mps=E.W20_MODERATE_MPS),
    Cond("B2_gust_front_8mps", E.SBAS, gust_step_mps=E.SQUALL_RISE_MPS, **INDIA),
    Cond("B3_gnss_underreports_5x", E.SBAS, white_factor=5.0, **INDIA),
    Cond("B4_downlink_out_3s", E.SBAS, down_out_s=3.0, **INDIA),
    Cond("B5_uplink_out_1.5s", E.SBAS, up_out_s=1.5, **INDIA),
    Cond("B6_lag_2x", E.SBAS, tau_true_s=2.0 * E.TAU_ACT_S, **INDIA),
    Cond("B7_gps_worst_site", E.SPS_WORST, **INDIA),
]
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


def recoverable(p0: np.ndarray, v0: np.ndarray) -> float:
    """Min separation of main on ground truth, no disturbance, no delay (fuzz dynamics)."""
    from research.consistent_backup import harness
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


@dataclass
class _Out:
    min_sep: float = math.inf
    los_s: float | None = None
    alarms: dict[str, float] = field(default_factory=dict)
    margins: list[float] = field(default_factory=list)
    alarm_cycles: int = 0
    fallbacks: int = 0
    solver_errors: int = 0
    cap_pairs: int = 0


def episode(p0: np.ndarray, v0: np.ndarray, cond: Cond, mode: str, seed: int) -> dict:
    """One closed-loop run; ``mode`` is "adaptive" or "baseline"."""
    rng = np.random.default_rng([seed, 7])
    n = len(p0)
    ids = [f"d{i}" for i in range(n)]
    g = cond.gnss
    sig_p = np.array([g.sigma_h_m, g.sigma_h_m, g.sigma_v_m])
    p, v, a = p0.astype(float).copy(), v0.astype(float).copy(), np.zeros((n, 3))
    c_now = np.zeros((n, 3))
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
    D = E.DRAG_PER_S
    # GNSS: constant per-episode bias (in model), or white noise (B3).
    bias = rng.normal(0.0, sig_p, (n, 3)) if cond.white_factor == 0.0 else np.zeros((n, 3))

    def measure(i: int) -> np.ndarray:
        noise = rng.normal(0.0, sig_p * cond.white_factor) if cond.white_factor else bias[i]
        return np.concatenate([p[i] + noise, v[i] + rng.normal(0.0, g.sigma_vel_mps, 3)])

    # Ground station's wind knowledge: forecast U and turbulence class (P3-SA-03 §4.2).
    q = np.maximum(2.0 * (D * sig_g) ** 2 / tau_g, 1e-6)
    p_dd0 = np.maximum((D * cond.wind_mps) ** 2 + (D * sig_g) ** 2, 1e-4)
    z0 = [measure(i) for i in range(n)]
    trackers = [Tracker(g, q, E.TAU_ACT_S, 0, z0[i], np.zeros(3), p_dd0) for i in range(n)]
    logs = [CommandLog(np.zeros(3)) for _ in range(n)]
    latest = [(0, z) for z in z0]
    echo: list[list[tuple[int, np.ndarray]]] = [[] for _ in range(n)]
    last_rx = np.zeros(n, dtype=int)
    down: list[tuple[int, int, int, np.ndarray, list]] = []   # (arrival, drone, t_m, z, echo)
    up: dict[int, list[tuple[int, np.ndarray]]] = {}           # playout tick -> (drone, c)
    latch: dict = {}
    out = _Out()
    out_from, down_to, up_to = (round(OUTAGE_S / H), round((OUTAGE_S + cond.down_out_s) / H),
                                round((OUTAGE_S + cond.up_out_s) / H))

    def alarm(name: str, t: int) -> None:
        out.alarms.setdefault(name, round(t * H, 2))

    def latency_ticks() -> tuple[bool, int]:
        d = E.LTE_AIR_ONE_WAY_MEDIAN_S * math.exp(E.LTE_AIR_SIGMA_LN * rng.standard_normal())
        return rng.random() < E.LINK_LOSS_3GPP, max(1, math.ceil(d / H - 1e-9))

    for t in range(round(EPISODE_S / H)):
        if t % CYCLE == 0:
            if t > 0:
                for i in range(n):
                    lost, dt = latency_ticks()
                    z = measure(i)
                    if not lost and not out_from <= t < down_to:
                        down.append((t + dt, i, t, z, list(echo[i])))
            # --- ground station ---
            ready = sorted((m for m in down if m[0] <= t), key=lambda m: m[2])
            down = [m for m in down if m[0] > t]
            for _, i, tm, z, ech in ready:
                if mode == "adaptive":
                    logs[i].confirm(ech, tm)
                    trackers[i].update(tm, z, logs[i])
                elif tm > latest[i][0]:
                    latest[i] = (tm, z)
            acts = [MARLAction(agent_id=aid, proposed_accel_mps2=(0.0, 0.0, 0.0), timestamp_ms=t * 10) for aid in ids]
            if mode == "adaptive":
                preds = {}
                for i, aid in enumerate(ids):
                    x, P, _ = trackers[i].predict_delay(t + P_TICKS, logs[i], A_TICKS)
                    preds[aid] = Pred(x, P, trackers[i], logs[i])
                res = margin(preds, t + P_TICKS + A_TICKS)
                monitor(preds, res, t)
                states = [_state(aid, pr.x[:3], pr.x[3:6], t) for aid, pr in preds.items()]
                cfg = F.FilterConfig(margin_m=res.margin_m)
                if t * H >= WARMUP_S:
                    out.margins.append(res.margin_m)
            else:
                res = None
                states = [_state(aid, latest[i][1][:3], latest[i][1][3:], t) for i, aid in enumerate(ids)]
                cfg = F.FilterConfig()
            safe, diag = F.apply_filter(states, acts, cfg, latch=latch)
            out.fallbacks += int(diag.fallback_used)
            out.solver_errors += int(not diag.qp_feasible)
            out.cap_pairs += len(diag.backup_rollout_cap_pairs)
            by_id = {s.agent_id: np.array(s.safe_accel_mps2) for s in safe}
            for i, aid in enumerate(ids):
                c = by_id[aid]
                if res is not None:
                    c = c - preds[aid].x[6:]
                    if np.any(np.abs(c) > E.A_PHYS_MPS2):
                        res.alarms.add("saturation")
                    c = np.clip(c, -E.A_PHYS_MPS2, E.A_PHYS_MPS2)
                    logs[i].plan(t + P_TICKS, c)
                lost, dt = latency_ticks()
                if not lost and dt <= P_TICKS and not out_from <= t < up_to:
                    up.setdefault(t + P_TICKS, []).append((i, c))
            if res is not None and t * H >= WARMUP_S:
                for name in res.alarms:
                    alarm(name, t)
                out.alarm_cycles += bool(res.alarms)
        # --- plant ---
        for i, c in up.pop(t, []):
            c_now[i] = c
            echo[i].append((t, c))
            last_rx[i] = t
        d = D * (W + gust)
        if cond.gust_step_mps and t >= out_from:
            d[0] += D * cond.gust_step_mps * step_dir
        for i in range(n):
            if (t - last_rx[i]) * H > E.OFFBOARD_LOSS_S:   # PX4 offboard loss: brake
                c_now[i] = np.clip(-v[i] / 0.1, -U_MAX, U_MAX)
            p[i], v[i], a[i] = kin_step(p[i], v[i], a[i], c_now[i], d[i], cond.tau_true_s)
        gust = gust * e_g + sig_g * np.sqrt(1.0 - e_g**2) * rng.standard_normal((n, 3))
        sep = min(float(np.linalg.norm(p[i] - p[j])) for i in range(n) for j in range(i + 1, n))
        out.min_sep = min(out.min_sep, sep)
        if sep < SAFE_SEPARATION_M and out.los_s is None:
            out.los_s = round((t + 1) * H, 2)
    m = np.array(out.margins) if out.margins else np.zeros(1)
    return {
        "min_sep": round(out.min_sep, 3), "los_s": out.los_s, "alarms": out.alarms,
        "first_alarm_s": min(out.alarms.values(), default=None), "alarm_cycles": out.alarm_cycles,
        "margin_p50": round(float(np.median(m)), 2), "margin_max": round(float(m.max()), 2),
        "fallbacks": out.fallbacks, "solver_errors": out.solver_errors, "cap_pairs": out.cap_pairs,
        "uplink_lost": int(sum(lg.lost for lg in logs)) if mode == "adaptive" else None,
    }
