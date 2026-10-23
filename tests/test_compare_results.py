"""scripts/verify/compare_results.py: exact and tolerance checks of fuzz results files."""
import importlib.util
import pathlib

_PATH = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "verify" / "compare_results.py"
_spec = importlib.util.spec_from_file_location("compare_results", _PATH)
cr = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(cr)


def _row(seed, min_sep, **kw):
    row = {"aborts": 0, "cap_hits": 0, "exc": None, "fallback_steps": 0, "first_inside": None, "max_rollout": 66,
           "min_sep": min_sep, "mode": "hard", "n": 2, "seed": seed, "solver_err_steps": 0}
    return {("hard", seed): {**row, **kw}}


def test_exact_needs_identical_rows():
    ref = {**_row(1, 6.0), **_row(2, 5.4, first_inside=40)}
    assert cr.compare(_row(1, 6.0), ref) == ([], 0.0)
    assert len(cr.compare(_row(1, 6.0 + 1e-15), ref)[0]) == 1
    assert len(cr.compare(_row(2, 5.4, first_inside=41), ref)[0]) == 1
    assert len(cr.compare(_row(3, 6.0), ref)[0]) == 1  # not in the reference


def test_tolerance_allows_small_min_sep_change_only():
    ref = {**_row(1, 6.0), **_row(2, 5.50005)}
    assert cr.compare(_row(1, 6.0 + 5e-5), ref, 1e-4)[0] == []
    assert len(cr.compare(_row(1, 6.0 + 2e-4), ref, 1e-4)[0]) == 1
    assert len(cr.compare(_row(1, 6.0, fallback_steps=1), ref, 1e-4)[0]) == 1
    # Within tolerance, but the seed moves inside d_eff.
    assert len(cr.compare(_row(2, 5.49999), ref, 1e-4)[0]) == 1
    assert len(cr.compare(_row(1, float("nan")), ref, 1e-4)[0]) == 1
