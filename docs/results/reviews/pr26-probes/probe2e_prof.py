"""PR 26 review probe, see docs/results/reviews/pr26-review-2026-10-24.md. Run from this directory with the project installed."""
import cProfile, pstats, logging, io
from common import ring
from utm_services.safety import filter as F
logging.disable(logging.CRITICAL)
F._warm_up()
sa, aa = ring((300, 500, 100), 50.0, 12.0, 64, "a")
smap = {s.agent_id: s for s in sa}; ids = sorted(smap)
pairs = [(a, b) for i, a in enumerate(ids) for b in ids[i+1:]]
pr = cProfile.Profile(); pr.enable(); F._update_latches(smap, pairs, F.FilterConfig(), {}); pr.disable()
s = io.StringIO(); pstats.Stats(pr, stream=s).sort_stats("cumulative").print_stats(12); print("\n".join(s.getvalue().splitlines()[4:22]))
