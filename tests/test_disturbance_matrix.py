"""scripts/disturbance_matrix.py: the BL-06 baseline table (a measurement, not a safety test)."""

import pytest

from scripts.disturbance_matrix import CASES, T2_PROFILES, main


@pytest.mark.slow
def test_matrix_writes_one_row_per_case_profile_and_seed(tmp_path) -> None:
    assert main(["--seeds", "1", "--out", str(tmp_path)]) == 0

    [report] = tmp_path.glob("bl06-baseline-*.md")
    rows = [line for line in report.read_text(encoding="utf-8").splitlines() if line.startswith("| ") and "---" not in line]
    assert len(rows) - 1 == len(CASES) * len(T2_PROFILES)  # minus the header row
    # The symmetric no-wind crossing is always in the table, for both profiles.
    assert sum(r.startswith("| crossing-90 no wind |") for r in rows) == len(T2_PROFILES)
