"""Reviewer probes for PR #12 head: walls, corners, vertical LOS, determinism."""
import logging
import sys

import numpy as np
from shared.contracts import MAX_SPEED_MPS, DroneState, MARLAction
from utm_services.safety import filter as F

logging.disable(logging.CRITICAL)
DT, D_EFF = 0.1, 5.5
LO = np.array([0.0, 0.0, 30.0])
HI = np.array([1000.0, 1000.0, 200.0])


def sim(p, v, nominal, steps=200, cfg=None, wind=(0, 0, 0)):
    """Closed loop like fuzz.py. nominal(k, p, v) -> (n,3). Returns stats."""
    cfg = cfg or F.FilterConfig(step_deadline=False)
    latch = {}  # one filter memory per run, like the episode runner
    p, v = np.array(p, float), np.array(v, float)
    n = len(p)
    ids = [f"d{i}" for i in range(n)]
    st = dict(min_sep=1e9, out=0.0, fb=0, infeas=0, cap=0, rollout=0, exc=None, lat=[])
    for k in range(steps):
        try:
            states = [DroneState(agent_id=ids[i], position_m=tuple(p[i]), velocity_mps=tuple(v[i]),
                                 goal_m=(500.0, 500.0, 100.0), timestamp_ms=k * 100) for i in range(n)]
        except Exception as e:  # noqa: BLE001
            st["exc"] = f"state@{k}: {str(e).splitlines()[-1][:80]}"
            break
        nom = np.clip(nominal(k, p, v), -3.5, 3.5)
        acts = [MARLAction(agent_id=ids[i], proposed_accel_mps2=tuple(nom[i]), timestamp_ms=k * 100) for i in range(n)]
        safe, diag = F.apply_filter(states, acts, cfg, latch=latch)
        st["fb"] += diag.fallback_used
        st["infeas"] += not diag.qp_feasible
        st["cap"] += len(diag.backup_rollout_cap_pairs)
        st["rollout"] = max([st["rollout"], *diag.backup_rollout_steps])
        a = np.array([s.safe_accel_mps2 for s in sorted(safe, key=lambda s: s.agent_id)])
        st["lat"].append(a.copy())
        vn = v + (a + np.array(wind)) * DT
        sp = np.linalg.norm(vn, axis=1)
        vn = np.where((sp > MAX_SPEED_MPS)[:, None], vn * (MAX_SPEED_MPS / sp * (1 - 1e-12))[:, None], vn)
        p = p + 0.5 * (v + vn) * DT
        v = vn
        st["out"] = max(st["out"], float(np.max(LO - p)), float(np.max(p - HI)))
        if n > 1:
            st["min_sep"] = min(st["min_sep"], min(np.linalg.norm(p[i] - p[j]) for i in range(n) for j in range(i + 1, n)))
    return st


def show(name, st):
    print(f"{name:52s} min_sep={st['min_sep']:8.3f} out_by={st['out']:7.3f} fb={st['fb']} infeas={st['infeas']} "
          f"cap={st['cap']} rollout={st['rollout']} exc={st['exc']}")


def push(direction):
    d = np.array(direction, float)
    return lambda k, p, v: np.tile(d * 3.5 / np.max(np.abs(d)), (len(p), 1))


section = sys.argv[1] if len(sys.argv) > 1 else "all"

if section in ("all", "limits"):
    print("== _world_accel_limits sweep: low <= high, on-bound, corners")
    cfg = F.FilterConfig(step_deadline=False)
    worst = 1e9
    rng = np.random.default_rng(0)
    for _ in range(200000):
        pos = rng.uniform(LO, HI)
        # bias some samples onto walls / corners exactly
        mask = rng.random(3) < 0.3
        pos = np.where(mask, np.where(rng.random(3) < 0.5, LO, HI), pos)
        vel = rng.normal(size=3)
        vel *= rng.uniform(0, 15) / np.linalg.norm(vel)
        lo, hi = F._world_accel_limits(pos, vel, cfg)
        worst = min(worst, float(np.min(hi - lo)))
    print(f"min(high-low) over 200k samples incl. walls/corners: {worst:.3f}")
    for m in (0.5, 5.0):
        lo, hi = F._world_accel_limits(np.array([500, 500, 115.0]), np.zeros(3), F.FilterConfig(step_deadline=False, margin_m=m))
        print(f"margin_m={m}: z limits ({lo[2]:.2f}, {hi[2]:.2f})")
    # FilterConfig rejects margins above 48.57 m at the default barrier offset.
    for m in (50.0, 90.0):
        try:
            F.FilterConfig(step_deadline=False, margin_m=m)
        except ValueError as e:
            print(f"margin_m={m}: rejected ({e})")
        else:
            raise AssertionError(f"FilterConfig accepted margin_m={m}")

if section in ("all", "walls"):
    print("== single drone, nominal full accel toward wall(s), max speed toward wall")
    for name, p0, d in [
        ("floor z=30 (on bound) v=-15", (500, 500, 30.0), (0, 0, -1)),
        ("floor z=45 v=-15", (500, 500, 45.0), (0, 0, -1)),
        ("ceiling z=200 (on bound) v=+15", (500, 500, 200.0), (0, 0, 1)),
        ("x=0 on bound v=-15x", (0.0, 500, 100), (-1, 0, 0)),
        ("corner x=0,y=0,z=30 diag", (0.0, 0.0, 30.0), (-1, -1, -1)),
        ("corner x=1000,y=1000,z=200 diag", (1000.0, 1000.0, 200.0), (1, 1, 1)),
        ("near corner x=20,y=20,z=50 diag", (20.0, 20.0, 50.0), (-1, -1, -1)),
    ]:
        dv = np.array(d, float)
        v0 = dv / np.linalg.norm(dv) * 15 * 0.9999
        show(name, sim([p0], [v0], push(d), steps=150))
    print("  with constant wind 1 m/s^2 toward floor (disturbance profile):")
    show("floor z=45 v=-15 wind -1z", sim([(500, 500, 45.0)], [(0, 0, -14.99)], push((0, 0, -1)), 150, wind=(0, 0, -1.0)))

if section in ("all", "squeeze"):
    print("== pair squeezed against walls (closed loop, nominal = hold velocity)")
    zero = lambda k, p, v: np.zeros((len(p), 3))
    cases = [
        ("floor: j hover z=31, i descends 15 from 60", [(500, 500, 60.0), (500, 500, 31.0)], [(0, 0, -14.99), (0, 0, 0)]),
        ("floor: j falls -15 at 40, i descends 15 at 75", [(500, 500, 75.0), (500, 500, 40.0)], [(0, 0, -14.99), (0, 0, -14.99)]),
        ("ceiling: j hover z=199, i climbs 15 from 170", [(500, 500, 170.0), (500, 500, 199.0)], [(0, 0, 14.99), (0, 0, 0)]),
        ("corner: j at (1,1,31), i diag 15 from (21,21,51)", [(21.0, 21.0, 51.0), (1.0, 1.0, 31.0)], [(-8.66, -8.66, -8.66), (0, 0, 0)]),
        ("corner: j on (0,0,30), i diag from (20,20,50)", [(20.0, 20.0, 50.0), (0.0, 0.0, 30.0)], [(-8.66, -8.66, -8.66), (0, 0, 0)]),
        ("x-wall + floor: i near x=1, j near z=31 oblique", [(1.0, 500, 60.0), (25.0, 500, 31.0)], [(0, 0, -10), (-10, 0, 0)]),
        ("x-wall: both fleeing into walls head-on in x", [(2.0, 500, 100.0), (30.0, 500, 100.0)], [(10, 0, 0), (-10, 0, 0)]),
    ]
    for name, p0, v0 in cases:
        show(name, sim(p0, v0, zero, 200))
        show(name + " [braking]", sim(p0, v0, zero, 200, cfg=F.FilterConfig(step_deadline=False, barrier="braking")))

if section in ("all", "vertical"):
    print("== vertical LOS cases")
    zero = lambda k, p, v: np.zeros((len(p), 3))
    for ang in (90.0, 89.9, 89.0):
        th = np.radians(ang)
        n = np.array([np.cos(th), 0.0, np.sin(th)])
        p0 = [np.array([500, 500, 100.0]) - 40 * n, np.array([500, 500, 100.0]) + 40 * n]
        show(f"head-on along {ang} deg elev, 10 m/s each", sim(p0, [10 * n, -10 * n], zero, 150))
    # LOS passes through vertical mid-encounter: horizontal overtake with 3 m vertical gap
    for gap in (0.0, 1.0, 3.0, 5.0):
        st = sim([(480, 500, 100.0 + gap), (500, 500, 100.0)], [(10, 0, 0), (0, 0, 0)], zero, 120)
        show(f"overtake through vertical, gap {gap} m", st)
    # direction determinism / fixedness: lateral() along a trajectory crossing vertical
    print("  _lateral as LOS sweeps through vertical (x from -0.2 to 0.2, z=1):")
    for x in (-0.2, -0.05, -1e-6, -1e-10, 0.0, 1e-10, 1e-6, 0.05, 0.2):
        n = np.array([x, 0.0, 1.0]); n /= np.linalg.norm(n)
        print(f"    n_x={x:+.0e} lateral={np.round(F._lateral(n), 3)}  antisym_err={np.abs(F._lateral(-n) + F._lateral(n)).max():.1e}")
    print("  _escape_axes sign flip near diagonal offset (near-vertical n):")
    for e in (-1e-6, 1e-6):
        n = np.array([0.05 + e, -0.05, 1.0]); n /= np.linalg.norm(n)
        print(f"    n={np.round(n, 7)} L={np.round(F._escape_axes(n)[0], 3)}")
    print("  3 drones on one vertical line (10 m apart), outer two converge at 10 m/s:")
    for bar in ("backup", "braking"):
        st = sim([(500, 500, 70.0), (500, 500, 100.0), (500, 500, 130.0)], [(0, 0, 10), (0, 0, 0), (0, 0, -10)], zero, 150,
                 cfg=F.FilterConfig(step_deadline=False, barrier=bar))
        show(f"vertical stack 3 {bar}", st)
        st = sim([(500, 500, 60.0), (500, 500, 85.0), (500, 500, 110.0), (500, 500, 135.0)],
                 [(0, 0, 10), (0, 0, 0), (0, 0, 0), (0, 0, -10)], zero, 150, cfg=F.FilterConfig(step_deadline=False, barrier=bar))
        show(f"vertical stack 4 {bar}", st)

if section in ("all", "det"):
    print("== determinism: same vertical stack twice, compare actions bitwise")
    zero = lambda k, p, v: np.zeros((len(p), 3))
    runs = [sim([(500, 500, 70.0), (500, 500, 100.0), (500, 500, 130.0)], [(0, 0, 10), (0, 0, 0), (0, 0, -10)], zero, 120)
            for _ in range(2)]
    same = all(np.array_equal(a, b) for a, b in zip(runs[0]["lat"], runs[1]["lat"]))
    print("  bitwise identical:", same)
