"""Re-run the CBK cases that hit an OSQP solver error in the first evaluation (P3-SA-02 §6.2).

    PYTHONPATH="src;.;scripts/verify" python -m research.consistent_backup.rerun_fallbacks

One process. Prints one line per run; every other run is unchanged by the
solver retry, because the retry only acts where both earlier attempts failed.
"""

from __future__ import annotations

from research.consistent_backup.evaluate import jobs, run

CASES = {"sandwich floor", "sandwich ceiling", "sandwich vertical"}

if __name__ == "__main__":
    for job in jobs(["cbk"]):
        name, case, kind, kw = job
        if case in CASES or (case == "vnoise sigma=1.5" and kw["seed"] == 14):
            r = run(job)
            print({k: v for k, v in r.items() if k != "_latency_ms"}, kw.get("seed"))
