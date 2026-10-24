"""`utm-px4-demo`: fly a PX4 SITL encounter guided live by the safety filter, then land.

    utm-px4-demo headon2 [--filter off] [--out DIR]
    utm-px4-demo cross3  [--filter off] [--out DIR]

Starts headless PX4 SITL (scripts/px4/sitl.sh, WSL2 Ubuntu; see docs/integration/PX4_SETUP.md),
takes every vehicle off, flies the encounter at 10 Hz, lands and stops SITL. Writes
``manifest.json``, ``summary.json``, ``records.jsonl`` (the utm-run format) and
``replay3d.html`` to ``--out``. ``--filter off`` flies each drone at its own altitude
(10 m apart), so the unfiltered encounter cannot collide; the summary then also shows the
horizontal separation. ``--stop-after S`` is the link-loss test: the bridge process exits
S seconds into the encounter without landing, and PX4's offboard-loss failsafe takes over.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import time
from dataclasses import asdict
from datetime import UTC, datetime
from importlib.metadata import version
from pathlib import Path

from shared.contracts import SAFE_SEPARATION_M

from utm_services.integration.px4_bridge import (
    ALT_OFFSET_M,
    TELEMETRY_STALE_MS,
    WORLD_ORIGIN_XY,
    Encounter,
    Px4Fleet,
    check_shared_origin,
    run_encounter,
    sitl_height,
    summarize,
    warm_up_filter,
)
from utm_services.safety.filter import FilterConfig
from utm_services.training.viewer3d import build_payload, write_viewer

SITL_SH = Path(__file__).resolve().parents[3] / "scripts" / "px4" / "sitl.sh"
PX4_FLOAT_PARAMS = ("COM_OF_LOSS_T", "MPC_XY_VEL_P_ACC", "MPC_ACC_HOR", "MPC_ACC_HOR_MAX", "MPC_XY_VEL_MAX",
                    "MPC_Z_VEL_MAX_UP", "MPC_Z_VEL_MAX_DN", "MPC_TILTMAX_AIR")
PX4_INT_PARAMS = ("COM_OBL_RC_ACT",)
CRUISE_Z = 40.0      # our altitude for the filtered encounters (20 m above the SITL ground)
FILTER_OFF_STEP_M = 10.0


def encounter(name: str, filter_on: bool) -> Encounter:
    """headon2: two drones 80 m apart on the x axis, swapped goals. cross3: three drones on a
    40 m circle, 120 degrees apart, each flying to the opposite point (all meet at the centre)."""
    if name == "headon2":
        spawns = ((-40.0, 0.0), (40.0, 0.0))
    elif name == "cross3":
        spawns = tuple((round(40.0 * math.cos(a), 3), round(40.0 * math.sin(a), 3))
                       for a in (0.0, 2 * math.pi / 3, 4 * math.pi / 3))
    else:
        raise ValueError(f"unknown encounter '{name}'")
    z = tuple(CRUISE_Z + (0.0 if filter_on else FILTER_OFF_STEP_M * i) for i in range(len(spawns)))
    goals = tuple((WORLD_ORIGIN_XY[0] - e, WORLD_ORIGIN_XY[1] - n, z[i]) for i, (e, n) in enumerate(spawns))
    return Encounter(name, tuple("abc"[: len(spawns)]), spawns, z, goals)


def _px4_tag() -> str | None:
    px4_dir = os.environ.get("PX4_DIR", str(Path.home() / "px4-v1.17.0"))
    try:
        return subprocess.run(["git", "-C", px4_dir, "describe", "--tags"], capture_output=True, text=True,
                              check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return None


def _write(out: Path, manifest: dict, records: list[dict], d_eff: float) -> dict:
    summary = summarize(records, d_eff)
    out.mkdir(parents=True, exist_ok=True)
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2, default=str))
    (out / "summary.json").write_text(json.dumps(summary, indent=2))
    with (out / "records.jsonl").open("w") as f:
        for rec in records:
            f.write(json.dumps(rec) + "\n")
    if records:
        ids = manifest["agent_ids"]
        goals = dict(zip(ids, manifest["goals_m"], strict=True))
        note = (f"PX4 SITL {manifest['px4_version']} x500 quadrotors in Gazebo Harmonic, positions from the PX4 EKF. "
                f"Filter {'on' if manifest['filter'] else 'off'}; end: {manifest['end_reason']}.")
        write_viewer(out / "replay3d.html", build_payload(records, ids, goals, [], summary, manifest["episode_id"], note))
    return summary


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="utm-px4-demo", description=__doc__.splitlines()[0])
    parser.add_argument("scenario", choices=("headon2", "cross3"))
    parser.add_argument("--filter", choices=("on", "off"), default="on")
    parser.add_argument("--out", type=Path, help="output directory (default: runs/px4/<scenario>-filter-<on|off>)")
    parser.add_argument("--no-launch", action="store_true", help="use SITL that is already running with these spawns")
    parser.add_argument("--stop-after", type=float, metavar="S",
                        help="link-loss test: exit S s into the encounter without landing")
    args = parser.parse_args(argv)

    filter_on = args.filter == "on"
    enc = encounter(args.scenario, filter_on)
    filter_config = FilterConfig()
    out = args.out or Path("runs") / "px4" / f"{args.scenario}-filter-{args.filter}"
    if not args.no_launch:
        subprocess.run(["bash", str(SITL_SH), "start", *(f"{e},{n}" for e, n in enc.spawns_en)], check=True)

    warm_up_s = warm_up_filter(filter_config) if filter_on else 0.0
    fleet = Px4Fleet(len(enc.agent_ids))
    try:
        fleet.wait(lambda: all(fleet.ned(i)[2] < TELEMETRY_STALE_MS and math.isfinite(fleet.ned(i)[0][0])
                               for i in range(len(enc.agent_ids))), 60, "position telemetry from every vehicle")
        origin_errors = check_shared_origin(enc.spawns_en, fleet.gps_origins())
        manifest = {
            "episode_id": f"{enc.scenario_id}-px4sitl-{'shielded_marl' if filter_on else 'unshielded_marl'}",
            "scenario_id": enc.scenario_id,
            "controller_variant": "shielded_marl" if filter_on else "unshielded_marl",
            "policy_id": "reference-standin",
            "filter": vars(filter_config) if filter_on else None,
            "backend": "px4-sitl-gazebo",
            "px4_version": _px4_tag(),
            "mavsdk_version": version("mavsdk"),
            "px4_params": fleet.params(PX4_FLOAT_PARAMS, PX4_INT_PARAMS),
            "encounter": asdict(enc),
            "agent_ids": list(enc.agent_ids),
            "goals_m": [list(g) for g in enc.goals_m],
            "frame": {"world_origin_xy": WORLD_ORIGIN_XY, "alt_offset_m": ALT_OFFSET_M,
                      "rule": "x = X0 + spawn_e + east, y = Y0 + spawn_n + north, z = -down + alt_offset"},
            "origin_check_errors_m": [round(e, 3) for e in origin_errors],
            "setpoint_mapping": "velocity v_meas + a_safe*dt (|v| <= 15 m/s) + a_safe acceleration feedforward",
            "loop_latency_boundary": "telemetry snapshot to last setpoint sent, time.perf_counter",
            "d_safe_m": SAFE_SEPARATION_M,
            "filter_warm_up_s": round(warm_up_s, 2),
            "started_utc": datetime.now(UTC).isoformat(timespec="seconds"),
        }
        print(f"origin check errors (m): {manifest['origin_check_errors_m']}; params {manifest['px4_params']}")
        fleet.take_off([sitl_height(z) for z in enc.start_z])
        fleet.start_offboard()
        t0 = time.monotonic()
        records, reason = run_encounter(fleet, enc, filter_on, filter_config, args.stop_after)
        last_send_unix = time.time() - (time.monotonic() - t0 - records[-1]["wall_time_s"]) if records else math.nan
        manifest["end_reason"], manifest["encounter_s"] = reason, round(time.monotonic() - t0, 2)
        print(f"end: {reason} after {manifest['encounter_s']} s, {len(records)} steps")
        if reason == "stopped":
            summary = _write(out, manifest, records, SAFE_SEPARATION_M + filter_config.margin_m)
            print(json.dumps(summary, indent=2))
            print(f"link-loss test: last setpoint at unix {last_send_unix:.3f}; bridge exits now without "
                  "landing; PX4 failsafe in charge")
            sys.stdout.flush()
            os._exit(3)  # no cleanup, no offboard stop: the link just disappears
        fleet.land()
    except BaseException:
        if any(v.telemetry.armed() for v in fleet.vehicles):
            print("error during the flight: landing every vehicle", file=sys.stderr)
            fleet.land()
        raise
    finally:
        fleet.close()
        if not args.no_launch:
            subprocess.run(["bash", str(SITL_SH), "stop"], check=False)

    summary = _write(out, manifest, records, SAFE_SEPARATION_M + filter_config.margin_m)
    print(json.dumps(summary, indent=2))
    return 0 if reason == "completed" else 1


if __name__ == "__main__":
    sys.exit(main())
