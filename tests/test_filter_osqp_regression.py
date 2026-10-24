"""PR #12 review: OSQP non-convergence on HARD fuzz seeds 864 and 977.

Both hit OSQP's iteration limit in the first solve (864 at t = 2.0 s, 977 at
t = 0, six drones), which dropped the step to the evasive fallback. The
encounters come from the reviewer's fuzzer, scripts/verify/fuzz.py, in HARD
mode (shorter time to approach).
"""

import importlib.util
import logging
from pathlib import Path

import pytest

FUZZ = Path(__file__).resolve().parents[1] / "scripts" / "verify" / "fuzz.py"


def _hard_fuzz(monkeypatch):
    monkeypatch.setenv("HARD", "1")
    spec = importlib.util.spec_from_file_location("verify_fuzz_hard", FUZZ)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    logging.disable(logging.NOTSET)  # fuzz.py disables logging on import
    return module


@pytest.mark.parametrize("seed", [864, 977])
@pytest.mark.slow
def test_hard_fuzz_seed_solves_without_fallback(seed, monkeypatch) -> None:
    result = _hard_fuzz(monkeypatch).run((seed, "backup", 100))

    assert result["exc"] is None
    assert result["solver_err_steps"] == 0
    assert result["fallback_steps"] == 0
