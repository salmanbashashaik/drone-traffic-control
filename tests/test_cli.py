"""`utm-run` CLI: run a scenario file, write the run artifacts, optional replay plot."""

import json
from pathlib import Path

import pytest
from shared.contracts import ScenarioConfig

from utm_services.evaluation.cli import main

SCENARIOS = sorted((Path(__file__).resolve().parents[1] / "scenarios").glob("*.json"))


def test_shipped_scenarios_are_valid_configs() -> None:
    assert SCENARIOS
    for path in SCENARIOS:
        ScenarioConfig.model_validate_json(path.read_text())


@pytest.mark.parametrize("variant", ["reference", "shielded_marl"])
def test_run_writes_manifest_summary_and_one_record_per_step(tmp_path, variant) -> None:
    scenario = SCENARIOS[0]
    assert main([str(scenario), "--variant", variant, "--out", str(tmp_path)]) == 0

    manifest = json.loads((tmp_path / "manifest.json").read_text())
    summary = json.loads((tmp_path / "summary.json").read_text())
    records = (tmp_path / "records.jsonl").read_text().splitlines()

    assert manifest["scenario_id"] == ScenarioConfig.model_validate_json(scenario.read_text()).scenario_id
    assert manifest["controller_variant"] == variant
    assert len(records) == summary["steps"]
    assert json.loads(records[0])["timestamp_ms"] == 0


def test_profile_selects_the_disturbance_profile(tmp_path) -> None:
    assert main([str(SCENARIOS[0]), "--profile", "realistic", "--out", str(tmp_path)]) == 0

    manifest = json.loads((tmp_path / "manifest.json").read_text())
    assert manifest["disturbance_profile"] == "realistic"
    assert manifest["disturbances"]["obs_pos_sigma_m"] == 1.5


def test_plot_writes_replay_png(tmp_path) -> None:
    pytest.importorskip("matplotlib")
    assert main([str(SCENARIOS[0]), "--out", str(tmp_path), "--plot"]) == 0

    assert (tmp_path / "replay.png").read_bytes()[:8] == b"\x89PNG\r\n\x1a\n"


def test_bad_scenario_file_exits_nonzero(tmp_path, capsys) -> None:
    bad = tmp_path / "bad.json"
    bad.write_text('{"scenario_id": "x"}')

    assert main([str(bad), "--out", str(tmp_path / "out")]) == 2
    assert "invalid scenario" in capsys.readouterr().err
