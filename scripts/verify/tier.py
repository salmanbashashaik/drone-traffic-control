"""Run a verification tier and report each step's result and wall time.

    python scripts/verify/tier.py quick [--out DIR] [--procs N]
    python scripts/verify/tier.py full  [--out DIR] [--procs N]

quick: fuzz.py --tier quick, checked bit for bit against the reference with compare_results.py.
full:  fuzz.py --tier full, checked the same way, then every script of the README table at the
       README command, up to --procs at a time (det.py twice, in separate processes, outputs compared).

Each step writes its output to DIR/<step>.log (default: ./verify_<tier>, gitignored). A rerun with the
same DIR resumes the fuzz run (fuzz.py skips seeds already in its JSONL). Exits 1 if any step fails.
Verification is change-scoped (CLAUDE.md): run quick while iterating, full once per PR.
"""
import argparse
import os
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
# The README table, longest first (assumed) so the pool ends together.
SCRIPTS = [
    ("inv2", ["inv2.py", "3000"]), ("invariance", ["invariance.py"]), ("horizon", ["horizon.py"]),
    ("optbound", ["optbound.py", "sandwich", "40"]), ("probe", ["probe.py"]), ("squeeze2", ["squeeze2.py"]),
    ("vnoise", ["vnoise.py"]), ("hnoise", ["hnoise.py"]), ("vsweep", ["vsweep.py"]), ("vertical", ["vertical.py"]),
    ("floor", ["floor.py"]), ("targeted", ["targeted.py"]), ("headon", ["headon.py"]),
    ("recov", ["recov.py", "2704", "252"]), ("det_a", ["det.py"]), ("det_b", ["det.py"]),
]


def step(name, args, out, env):
    t0 = time.perf_counter()
    with open(os.path.join(out, f"{name}.log"), "w") as log:
        code = subprocess.call([sys.executable, *args], cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT)
    wall = time.perf_counter() - t0
    print(f"{name}: {'ok' if code == 0 else f'FAILED (exit {code})'} in {wall:.0f} s", flush=True)
    return code


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("tier", choices=("quick", "full"))
    ap.add_argument("--out", help="log directory (default ./verify_<tier>)")
    ap.add_argument("--procs", type=int, default=os.cpu_count())
    a = ap.parse_args(argv)
    out = os.path.abspath(a.out or f"verify_{a.tier}")
    os.makedirs(out, exist_ok=True)
    env = {**os.environ, "PYTHONPATH": os.pathsep.join(["src", ".", os.environ.get("PYTHONPATH", "")]),
           "PYTHONIOENCODING": "utf-8"}
    env.pop("HARD", None)  # the tiers set the mode per run
    t0 = time.perf_counter()
    results = os.path.join(out, "fuzz.jsonl")
    failed = step("fuzz", [os.path.join(HERE, "fuzz.py"), "--tier", a.tier, "--out", results,
                           "5000", "backup", "100", str(a.procs)], out, env) != 0
    failed |= step("compare", [os.path.join(HERE, "compare_results.py"), results], out, env) != 0
    if a.tier == "full":
        with ThreadPoolExecutor(a.procs) as pool:
            codes = list(pool.map(lambda s: step(s[0], [os.path.join(HERE, s[1][0]), *s[1][1:]], out, env), SCRIPTS))
        failed |= any(codes)
        with open(os.path.join(out, "det_a.log")) as fa, open(os.path.join(out, "det_b.log")) as fb:
            same = fa.read() == fb.read()
        print(f"det: {'identical' if same else 'DIFFERENT'} across two processes")
        failed |= not same
    print(f"tier {a.tier}: {'FAILED' if failed else 'ok'} in {time.perf_counter() - t0:.0f} s; logs in {out}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
