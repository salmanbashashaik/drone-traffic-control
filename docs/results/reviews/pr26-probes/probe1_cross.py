"""Attack 1: with one cluster forced to miss, do all cross-cluster rows still hold, and does the kept cluster's
solution stay bit-identical? Random scenes of several nearby groups."""
import numpy as np
from shared.contracts import DroneState, MARLAction
from utm_services.safety import filter as F
F._warm_up()
rng = np.random.default_rng(7)
cfg_off = F.FilterConfig(step_deadline=False)
worst = -np.inf; checked = 0; scenes = 0; identical = True; nonbox = 0
for trial in range(300):
    groups = rng.integers(2, 5)
    states, actions = [], []
    for g in range(groups):
        c = np.array([rng.uniform(100, 900), rng.uniform(100, 900), rng.uniform(40, 190)])
        k = rng.integers(2, 6)
        for m in range(k):
            d = rng.normal(size=3); d /= np.linalg.norm(d)
            p = c + rng.uniform(6, 30) * d
            v = -rng.uniform(3, 15) * d + rng.normal(scale=2.0, size=3)
            v *= min(1.0, 14.9 / np.linalg.norm(v))
            p = np.clip(p, [1, 1, 31], [999, 999, 199])
            aid = f"g{g}d{m}"
            states.append(DroneState(agent_id=aid, position_m=tuple(p), velocity_mps=tuple(v), goal_m=(900.0, 900.0, 100.0), timestamp_ms=0))
            actions.append(MARLAction(agent_id=aid, proposed_accel_mps2=tuple(rng.uniform(-3.5, 3.5, 3)), timestamp_ms=0))
    smap = {s.agent_id: s for s in states}; amap = {a.agent_id: a for a in actions}
    ids = sorted(smap); pairs = [(a, b) for i, a in enumerate(ids) for b in ids[i+1:]]
    # the clusters the filter would form (first stage)
    frames = F._update_latches(smap, pairs, cfg_off, {})
    backup = F._backup_constraints(smap, pairs, cfg_off, frames)
    problems = [p for p in F._independent_problems(smap, amap, pairs, cfg_off, None, backup, frames) if len(p[1])]
    if len(problems) < 2:
        continue
    scenes += 1
    base, _ = F.apply_filter(states, actions, cfg_off, latch={}); base = {a.agent_id: a for a in base}
    victim = set(problems[rng.integers(len(problems))][0])
    real = F._solve_one_qp
    def miss_victim(states_, *args, **kwargs):
        return F._DEADLINE if victim.intersection(states_) else real(states_, *args, **kwargs)
    F._solve_one_qp = miss_victim
    try:
        F._STEP_BUDGET_S = 5.0
        safe, diag = F.apply_filter(states, actions, F.FilterConfig(), latch={})
    finally:
        F._solve_one_qp = real; F._STEP_BUDGET_S = 0.1
    safe = {a.agent_id: a for a in safe}
    assert set(diag.deadline_fallback_agents) == victim, (diag.deadline_fallback_agents, victim)
    acc = {aid: np.array(safe[aid].safe_accel_mps2) for aid in ids}
    # every dropped (cross-cluster or free) row must hold with the mixed accelerations; kept rows inside the victim are the backup's business
    for k, (a, b) in enumerate(pairs):
        same = any({a, b} <= set(agents) for agents, _ in problems)
        if same:
            continue
        resid = float(backup[0][k] @ (acc[a] - acc[b])) - backup[1][k]
        worst = max(worst, -resid); checked += 1
    for aid in ids:
        if aid in victim: continue
        if safe[aid].safe_accel_mps2 != base[aid].safe_accel_mps2:
            identical = False
    # evasive inside box?
    for aid in victim:
        st = smap[aid]
        lo, hi = F._world_accel_limits(np.array(st.position_m), np.array(st.velocity_mps), cfg_off)
        if np.any(acc[aid] < lo - 1e-12) or np.any(acc[aid] > hi + 1e-12): nonbox += 1
print(f"scenes with >=2 clusters: {scenes}; cross-cluster rows checked: {checked}; worst violation (m/s^2, positive = violated): {worst:.3e}")
print(f"kept clusters bit-identical to the no-deadline run: {identical}; evasive outside world-box limits: {nonbox}")
