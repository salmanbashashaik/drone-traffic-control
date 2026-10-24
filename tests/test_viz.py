"""The viewers of the decision log: utm-viz-cesium (CZML + HTML) and utm-viz-rerun (.rrd)."""

import functools
import json
import re
import threading
import urllib.error
import urllib.request
from pathlib import Path

import numpy as np
import pytest

from utm_services.evaluation import cli, viz_cesium
from utm_services.evaluation.decision_log import read_decision_log

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def runs(tmp_path_factory) -> dict[str, Path]:
    out = tmp_path_factory.mktemp("cross4")
    for variant in ("shielded_marl", "unshielded_marl"):
        assert cli.main([str(ROOT / "scenarios" / "demo" / "cross4.json"), "--variant", variant,
                         "--out", str(out / variant), "--decision-log"]) == 0
    return {"on": out / "shielded_marl" / "decisions.jsonl", "off": out / "unshielded_marl" / "decisions.jsonl"}


def test_world_to_globe_keeps_distances_and_up() -> None:
    rng = np.random.default_rng(3)
    world = rng.uniform([0, 0, 30], [1000, 1000, 200], (50, 3))
    ecef = viz_cesium.enu_to_ecef(viz_cesium.BENGALURU, world)
    d_world = np.linalg.norm(world[:, None] - world[None], axis=2)
    d_ecef = np.linalg.norm(ecef[:, None] - ecef[None], axis=2)
    assert np.abs(d_world - d_ecef).max() < 1e-6
    ground, above = viz_cesium.enu_to_ecef(viz_cesium.BENGALURU, np.array([[500.0, 500.0, 0.0], [500.0, 500.0, 100.0]]))
    lat, lon = np.radians(viz_cesium.BENGALURU[:2])
    normal = [np.cos(lat) * np.cos(lon), np.cos(lat) * np.sin(lon), np.sin(lat)]  # the ellipsoid's up at the origin
    assert np.allclose(above - ground, 100.0 * np.array(normal), atol=1e-6)


def test_cesium_build_has_every_drone_both_arrows_and_no_key(runs, tmp_path, monkeypatch) -> None:
    monkeypatch.setenv("GOOGLE_MAPS_API_KEY", "sentinel-google-key")
    monkeypatch.setenv("CESIUM_ION_TOKEN", "sentinel-ion-token")
    out = tmp_path / "cesium"
    manifest = viz_cesium.build([runs["on"], runs["off"]], out, viz_cesium.BENGALURU, "auto")
    assert manifest["origin"]["lat_deg"] == viz_cesium.BENGALURU[0] and set(manifest["runs"]) == {"on", "off"}
    header, steps = read_decision_log(runs["on"])
    ids = {p["id"] for p in json.loads((out / "on.czml").read_text())}
    for aid in header["agent_ids"]:
        assert {f"on/{aid}", f"on/{aid}/proposed", f"on/{aid}/safe"} <= ids
    assert manifest["runs"]["on"]["min_sep_m"] == min(s["min_sep_m"] for s in steps if s["min_sep_m"] is not None)
    for f in out.iterdir():
        assert "sentinel" not in f.read_text(encoding="utf-8"), f"a key was written to {f.name}"


def test_cesium_refuses_runs_of_different_scripts(runs, tmp_path) -> None:
    lines = runs["off"].read_text().splitlines()
    header = json.loads(lines[0]) | {"script_sha256": "0" * 64}
    other = tmp_path / "other.jsonl"
    other.write_text("\n".join([json.dumps(header), *lines[1:]]) + "\n")
    with pytest.raises(ValueError, match="different scripts"):
        viz_cesium.build([runs["on"], other], tmp_path / "cesium", viz_cesium.BENGALURU, "auto")


def test_cesium_refuses_a_toggle_without_script_hashes(runs, tmp_path) -> None:
    logs = []
    for tag in ("on", "off"):
        lines = runs[tag].read_text().splitlines()
        logs.append(tmp_path / f"{tag}.jsonl")
        logs[-1].write_text("\n".join([json.dumps(json.loads(lines[0]) | {"script_sha256": None}), *lines[1:]]) + "\n")
    with pytest.raises(ValueError, match="no script_sha256"):
        viz_cesium.build(logs, tmp_path / "cesium", viz_cesium.BENGALURU, "auto")


def test_cesium_page_escapes_log_fields(runs, tmp_path) -> None:
    lines = runs["on"].read_text().splitlines()
    log = tmp_path / "x.jsonl"
    log.write_text("\n".join([json.dumps(json.loads(lines[0]) | {"episode_id": "</script><b>x"}), *lines[1:]]) + "\n")
    viz_cesium.build([log], tmp_path / "cesium", viz_cesium.BENGALURU, "auto")
    assert "</script><b>" not in (tmp_path / "cesium" / "index.html").read_text(encoding="utf-8")


def _get(url: str, host: str | None = None) -> tuple[int, str, dict]:
    req = urllib.request.Request(url, headers={"Host": host} if host else {})
    try:
        with urllib.request.urlopen(req) as r:
            return r.status, r.read().decode(), dict(r.headers)
    except urllib.error.HTTPError as e:
        return e.code, "", {}


def test_keys_are_served_from_the_environment_to_our_page_only(runs, tmp_path, monkeypatch) -> None:
    monkeypatch.setenv("CESIUM_ION_TOKEN", "sentinel-ion-token")
    monkeypatch.delenv("GOOGLE_MAPS_API_KEY", raising=False)
    out = tmp_path / "cesium"
    viz_cesium.build([runs["on"]], out, viz_cesium.BENGALURU, "auto")
    before = {f: f.read_bytes() for f in out.iterdir()}
    server = viz_cesium._Server(("127.0.0.1", 0), functools.partial(viz_cesium._Handler, directory=str(out)))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{server.server_address[1]}"
    try:
        status, page, _ = _get(base + "/")
        name = re.search(r'<script src="(keys-[^"]+\.js)">', page).group(1)
        status, body, headers = _get(f"{base}/{name}")
        assert status == 200 and headers["Cache-Control"] == "no-store"
        assert '"ion": "sentinel-ion-token"' in body and '"google": ""' in body
        assert _get(base + "/keys.js")[0] == 404  # a guessable path that any web page could load
        assert _get(f"{base}/{name}", host="evil.example")[0] == 403  # DNS rebinding
        assert _get(base + "/", host="evil.example")[0] == 403
    finally:
        server.shutdown()
        server.server_close()
    assert {f: f.read_bytes() for f in out.iterdir()} == before  # nothing written


def test_rerun_recording_is_written(runs, tmp_path) -> None:
    pytest.importorskip("rerun")
    from utm_services.evaluation.viz_rerun import log_to_rerun

    log_to_rerun(runs["on"], tmp_path / "x.rrd")
    assert (tmp_path / "x.rrd").stat().st_size > 10_000
