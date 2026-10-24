"""Reviewer probe: pairs and stacks squeezed against the floor, ceiling and a corner."""
import sys

sys.argv = ["x", "none"]  # probe.py runs no section of its own on import
from probe import F, np, show, sim  # noqa: E402

zero = lambda k, p, v: np.zeros((len(p), 3))
for top in (70.0, 75.0, 85.0):
    for bar in ("backup","braking"):
        show(f"floor squeeze: j hover 31, i -15 from {top} {bar}", sim([(500,500,top),(500,500,31.0)],[(0,0,-14.99),(0,0,0)],zero,200,cfg=F.FilterConfig(step_deadline=False, barrier=bar)))
        show(f"ceil squeeze: j hover 199, i +15 from {230-top} {bar}", sim([(500,500,230-top),(500,500,199.0)],[(0,0,14.99),(0,0,0)],zero,200,cfg=F.FilterConfig(step_deadline=False, barrier=bar)))
        show(f"corner squeeze: j (0.5,0.5,30.5), i diag from {top-30}-off {bar}", sim([(top-30,top-30,top-10),(0.5,0.5,30.5)],[(-8.65,-8.65,-8.65),(0,0,0)],zero,200,cfg=F.FilterConfig(step_deadline=False, barrier=bar)))
# 3 drones: stack against floor
for bar in ("backup","braking"):
    show(f"floor stack 3: 31/45 hover, top -15 from 90 {bar}", sim([(500,500,90.0),(500,500,45.0),(500,500,31.0)],[(0,0,-14.99),(0,0,0),(0,0,0)],zero,200,cfg=F.FilterConfig(step_deadline=False, barrier=bar)))
    show(f"wind -1z pair floor, i -10 from 80 {bar}", sim([(500,500,80.0),(500,500,31.0)],[(0,0,-10),(0,0,0)],zero,200,cfg=F.FilterConfig(step_deadline=False, barrier=bar),wind=(0,0,-1.0)))
