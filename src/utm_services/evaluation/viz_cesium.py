"""`utm-viz-cesium`: decision logs of the same script, filter on and off, on a CesiumJS globe.

    utm-viz-cesium runs/x-on [runs/x-off] --out runs/x-cesium [--origin LAT,LON[,H]] [--serve]

Writes ``on.czml``, ``off.czml``, ``index.html`` and ``manifest.json`` to ``--out``. The world
frame (metres, z up) is placed with its point (500, 500, 0) at a fixed origin (default: Cubbon
Park, Bengaluru, at the WGS84 ellipsoid) through an exact east-north-up rotation, so distances
on the globe equal the logged ones. Drones (billboards) with trails, a sphere of radius d_eff/2
coloured by the distance to the nearest neighbour, the proposed (blue) and safe (green)
acceleration arrows, a click panel per drone, the timeline (half speed by default) and a toggle
between the two runs. Every value comes from the decision logs.

Basemap: Cesium World Terrain + OSM Buildings when CESIUM_ION_TOKEN is set, Google
Photorealistic 3D Tiles with ``--basemap google`` and GOOGLE_MAPS_API_KEY (no high-resolution
Google coverage at Bengaluru or Hyderabad on 2026-10-24), otherwise OpenStreetMap imagery on the
ellipsoid. Keys are read from the environment only by ``--serve``, which answers them on
127.0.0.1 from memory, at a random path that only its own ``index.html`` names: no key is ever
written to disk, and no other web page can load them.
"""

from __future__ import annotations

import argparse
import functools
import html
import http.server
import json
import math
import os
import secrets
import sys
from datetime import datetime, timedelta, timezone
from pathlib import Path
from urllib.parse import quote

import numpy as np

from utm_services.evaluation.decision_log import read_decision_log, separation_rgb
from utm_services.evaluation.viz_rerun import ACCEL_SCALE_M, PROPOSED_RGB, SAFE_RGB

CESIUM = "1.123.0"
EPOCH = datetime(2026, 1, 1, tzinfo=timezone.utc)  # simulation t = 0; the date means nothing
BENGALURU = (12.9763, 77.5929, 0.0)  # Cubbon Park; height of the world's z = 0 above the ellipsoid
WORLD_ORIGIN_XY_M = (500.0, 500.0)
TRAIL_S = 15.0
# A quadcopter seen from above, white so the billboard colour tints it.
DRONE_SVG = "data:image/svg+xml," + quote(
    "<svg xmlns='http://www.w3.org/2000/svg' width='32' height='32' viewBox='0 0 32 32'>"
    "<g stroke='white' stroke-width='3'><line x1='8' y1='8' x2='24' y2='24'/><line x1='24' y1='8' x2='8' y2='24'/>"
    "</g><g fill='none' stroke='white' stroke-width='2'><circle cx='7' cy='7' r='5'/><circle cx='25' cy='7' r='5'/>"
    "<circle cx='7' cy='25' r='5'/><circle cx='25' cy='25' r='5'/></g><circle cx='16' cy='16' r='4' fill='white'/></svg>")


def enu_to_ecef(origin: tuple[float, float, float], xyz: np.ndarray) -> np.ndarray:
    """World points (N, 3) to Earth-fixed metres: (500, 500, 0) sits at ``origin`` (lat, lon, h), x east, y north, z up."""
    a, f = 6378137.0, 1 / 298.257223563
    e2 = f * (2 - f)
    lat, lon, h = np.radians(origin[0]), np.radians(origin[1]), origin[2]
    n = a / np.sqrt(1 - e2 * np.sin(lat) ** 2)
    base = np.array([(n + h) * np.cos(lat) * np.cos(lon), (n + h) * np.cos(lat) * np.sin(lon), (n * (1 - e2) + h) * np.sin(lat)])
    east = np.array([-np.sin(lon), np.cos(lon), 0.0])
    north = np.array([-np.sin(lat) * np.cos(lon), -np.sin(lat) * np.sin(lon), np.cos(lat)])
    up = np.array([np.cos(lat) * np.cos(lon), np.cos(lat) * np.sin(lon), np.sin(lat)])
    local = np.asarray(xyz, dtype=float) - [*WORLD_ORIGIN_XY_M, 0.0]
    return base + local[:, :1] * east + local[:, 1:2] * north + local[:, 2:3] * up


def _iso(t_s: float) -> str:
    return (EPOCH + timedelta(seconds=t_s)).isoformat().replace("+00:00", "Z")


def _row(t_s: float, values) -> list[float]:
    return [round(t_s, 3), *(round(float(v), 3) for v in values)]


def _panel(t_s: float, d: dict) -> str:
    def vec(v):
        return "(" + ", ".join(f"{c:.2f}" for c in v) + ")"
    near = d["nearest"]
    latch = ", ".join(f"{html.escape(x['with'])} (turn {vec(x['side'])}, up {x['up']:+.0f})" for x in d["latch"]) or "none"
    rows = [("t", f"{t_s:.1f} s"), ("position (m)", vec(d["pos"])), ("velocity (m/s)", vec(d["vel"])),
            ("speed (m/s)", f"{np.linalg.norm(d['vel']):.2f}"), ("proposed accel (m/s²)", vec(d["proposed"])),
            ("safe accel (m/s²)", vec(d["safe"])), ("filter intervened", "yes" if d["intervened"] else "no"),
            ("fallback used", "yes" if d["fallback_used"] else "no"),
            ("deadline fallback", "not on main" if d["deadline_fallback"] is None else str(d["deadline_fallback"])),
            ("conflict cluster", "none" if d["cluster"] is None else str(d["cluster"])), ("latched with", latch),
            ("nearest", "none" if near is None else f"{html.escape(near['id'])} at {near['dist_m']:.2f} m")]
    return ("<table>"
            + "".join(f"<tr><th style='text-align:left'>{k}</th><td>{v}</td></tr>" for k, v in rows) + "</table>")


def run_czml(header: dict, steps: list[dict], origin: tuple[float, float, float], end_s: float) -> list[dict]:
    """CZML packets of one decision log; ids are prefixed with ``on/`` or ``off/``."""
    tag = "on" if header["filter_on"] else "off"
    d_eff = header["d_eff_m"]
    track: dict[str, dict[str, list]] = {}
    for step in steps:
        t = step["t_ms"] / 1000.0
        for aid, d in step["drones"].items():
            s = track.setdefault(aid, {k: [] for k in ("t", "pos", "proposed", "safe", "rgb", "label", "panel")})
            s["t"].append(t)
            s["pos"].append(d["pos"])
            s["proposed"].append(np.add(d["pos"], np.multiply(d["proposed"], ACCEL_SCALE_M)))
            s["safe"].append(np.add(d["pos"], np.multiply(d["safe"], ACCEL_SCALE_M)))
            s["rgb"].append(separation_rgb(d["nearest"]["dist_m"] if d["nearest"] else None, d_eff))
            s["label"].append(aid + (" *" if d["intervened"] else "") + (" FALLBACK" if d["fallback_used"] else ""))
            s["panel"].append(_panel(t, d))
    packets = [{"id": "document", "name": f"{header['episode_id']} filter {tag}", "version": "1.0",
                "clock": {"interval": f"{_iso(0)}/{_iso(end_s)}", "currentTime": _iso(0), "multiplier": 0.5,
                          "range": "LOOP_STOP", "step": "SYSTEM_CLOCK_MULTIPLIER"}}]
    epoch = _iso(0)
    for aid, s in track.items():
        times = s["t"]
        # A step's value holds until the next step; the last one holds for one step.
        bounds = [_iso(t) for t in times] + [_iso(times[-1] + header["dt_s"])]
        available = f"{bounds[0]}/{bounds[-1]}"

        def sampled(points):
            ecef = enu_to_ecef(origin, np.array(points))
            return {"epoch": epoch, "referenceFrame": "FIXED", "interpolationAlgorithm": "LINEAR",
                    "forwardExtrapolationType": "HOLD", "forwardExtrapolationDuration": header["dt_s"],
                    "cartesian": [x for t, p in zip(times, ecef) for x in _row(t, p)]}

        def held(values, key):
            runs, start = [], 0
            for k in range(1, len(values) + 1):
                if k == len(values) or values[k] != values[start]:
                    runs.append({"interval": f"{bounds[start]}/{bounds[k]}", key: values[start]})
                    start = k
            return runs

        rgba = {"epoch": epoch, "rgba": [x for t, c in zip(times, s["rgb"]) for x in _row(t, (*c, 255))]}
        sphere = {"epoch": epoch, "rgba": [x for t, c in zip(times, s["rgb"]) for x in _row(t, (*c, 70))]}
        drone = f"{tag}/{aid}"
        packets.append({
            "id": drone, "name": aid, "availability": available, "position": sampled(s["pos"]),
            "description": held(s["panel"], "string"),
            "billboard": {"image": DRONE_SVG, "scale": 0.8, "color": rgba, "disableDepthTestDistance": 1e9},
            "label": {"text": held(s["label"], "string"), "font": "13px sans-serif", "pixelOffset": {"cartesian2": [0, -22]},
                      "disableDepthTestDistance": 1e9,
                      "fillColor": {"rgba": [255, 255, 255, 255]}, "outlineColor": {"rgba": [0, 0, 0, 255]},
                      "outlineWidth": 3, "style": "FILL_AND_OUTLINE"},
            "ellipsoid": {"radii": {"cartesian": [d_eff / 2] * 3}, "material": {"solidColor": {"color": sphere}},
                          "outline": True, "outlineColor": rgba, "slicePartitions": 12, "stackPartitions": 12},
            "path": {"leadTime": 0, "trailTime": TRAIL_S, "width": 2, "resolution": 0.1,
                     "material": {"solidColor": {"color": {"rgba": [255, 255, 255, 160]}}}},
        })
        for kind, colour in (("proposed", PROPOSED_RGB), ("safe", SAFE_RGB)):
            packets.append({"id": f"{drone}/{kind}-tip", "availability": available, "position": sampled(s[kind])})
            packets.append({"id": f"{drone}/{kind}", "name": f"{aid} {kind} acceleration", "availability": available,
                            "polyline": {"positions": {"references": [f"{drone}#position", f"{drone}/{kind}-tip#position"]},
                                         "width": 9, "arcType": "NONE",
                                         "material": {"polylineArrow": {"color": {"rgba": [*colour, 255]}}}}})
    return packets


def build(logs: list[Path], out: Path, origin: tuple[float, float, float], basemap: str) -> dict:
    runs = [read_decision_log(p) for p in logs]
    tags = ["on" if h["filter_on"] else "off" for h, _ in runs]
    if len(set(tags)) != len(tags):
        raise ValueError("give at most one run with the filter on and one with it off")
    if len(runs) == 2 and not all(h["script_sha256"] for h, _ in runs):
        raise ValueError("a log has no script_sha256: cannot show that both runs fly the same script")
    if len({h["script_sha256"] for h, _ in runs}) != 1:
        raise ValueError("the runs come from different scripts (script_sha256 differs): no on/off toggle")
    end_s = max(s[-1]["t_ms"] / 1000.0 + h["dt_s"] for h, s in runs if s)
    out.mkdir(parents=True, exist_ok=True)
    points = np.array([d["pos"] for _, steps in runs for step in steps for d in step["drones"].values()])
    lo, hi = points.min(axis=0), points.max(axis=0)
    centre = enu_to_ecef(origin, ((lo + hi) / 2)[None])[0]
    manifest = {
        "format": "utm-viz-cesium/1", "cesium": CESIUM, "basemap": basemap,
        "origin": {"lat_deg": origin[0], "lon_deg": origin[1], "height_m": origin[2],
                   "meaning": f"world point {WORLD_ORIGIN_XY_M + (0.0,)} m; x east, y north, z up, exact ENU rotation"},
        "epoch": _iso(0), "epoch_meaning": "simulation t = 0",
        "camera": {"target_ecef_m": centre.tolist(), "range_m": float(np.ptp(points[:, :2], axis=0).max()) * 1.4 + 60.0},
        "runs": {},
    }
    for tag, (header, steps), log in zip(tags, runs, logs):
        (out / f"{tag}.czml").write_text(json.dumps(run_czml(header, steps, origin, end_s), separators=(",", ":")))
        manifest["runs"][tag] = {k: header[k] for k in ("episode_id", "scenario_id", "backend", "truth_source",
                                                        "scenario_sha256", "script_sha256", "replay_check", "d_eff_m")}
        manifest["runs"][tag]["log"] = f"{log.parent.name}/{log.name}"  # no local path (user name) in a shared page
        manifest["runs"][tag]["min_sep_m"] = min((s["min_sep_m"] for s in steps if s["min_sep_m"] is not None), default=None)
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2))
    meta = json.dumps(manifest).replace("<", "\\u003c")  # no "</script>" from a log field can end the script
    (out / "index.html").write_text(PAGE.replace("__CESIUM__", CESIUM).replace("__META__", meta), encoding="utf-8")
    return manifest


class _Handler(http.server.SimpleHTTPRequestHandler):
    """Serves --out. Keys only at the server's random ``keys_path``, named only in the index.html it serves,
    so another web page cannot load them with a script tag (it cannot read our index.html across origins)."""

    def do_GET(self) -> None:
        port = self.server.server_address[1]
        if self.headers.get("Host") not in (f"127.0.0.1:{port}", f"localhost:{port}"):  # DNS rebinding
            return self.send_error(403)
        path = self.path.split("?")[0]
        if path == self.server.keys_path:
            keys = {"google": os.environ.get("GOOGLE_MAPS_API_KEY", ""), "ion": os.environ.get("CESIUM_ION_TOKEN", "")}
            return self._send(f"window.UTM_KEYS = {json.dumps(keys)};".encode(), "text/javascript")
        if path in ("/", "/index.html"):
            page = Path(self.directory, "index.html").read_text(encoding="utf-8")
            page = page.replace('<script src="keys.js">', f'<script src="{self.server.keys_path[1:]}">')
            return self._send(page.encode(), "text/html; charset=utf-8")
        return super().do_GET()

    def _send(self, body: bytes, content_type: str) -> None:
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


class _Server(http.server.ThreadingHTTPServer):
    allow_reuse_address = sys.platform != "win32"  # on Windows it lets a second server share the port silently

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.keys_path = f"/keys-{secrets.token_urlsafe(16)}.js"


def serve(out: Path, port: int) -> None:
    handler = functools.partial(_Handler, directory=str(out))
    with _Server(("127.0.0.1", port), handler) as httpd:
        print(f"http://127.0.0.1:{port}/  (Ctrl+C stops)", flush=True)
        httpd.serve_forever()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="utm-viz-cesium", description=__doc__.splitlines()[0])
    parser.add_argument("logs", type=Path, nargs="+", help="decisions.jsonl or its run directory: filter on, and off")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--origin", default=",".join(map(str, BENGALURU)), help="LAT,LON[,H] of world (500, 500, 0); H = ground height above the ellipsoid for terrain "
                             "basemaps; write --origin=-12.9,77 for a negative latitude")
    parser.add_argument("--basemap", choices=["auto", "google", "ion", "osm"], default="auto",
                        help="auto: ion if CESIUM_ION_TOKEN is set, else osm")
    parser.add_argument("--serve", action="store_true", help="serve --out on 127.0.0.1 with keys from the environment")
    parser.add_argument("--port", type=int, default=8002)
    args = parser.parse_args(argv)
    if len(args.logs) > 2:
        parser.error("give one or two logs")
    try:
        origin = [float(v) for v in args.origin.split(",")]
    except ValueError:
        origin = []
    if (len(origin) not in (2, 3) or not all(map(math.isfinite, origin))
            or not (-90 <= origin[0] <= 90 and -180 <= origin[1] <= 180)):
        parser.error("--origin is LAT,LON or LAT,LON,H in degrees and metres")
    origin = (origin + [0.0])[:3]
    logs = [p / "decisions.jsonl" if p.is_dir() else p for p in args.logs]
    try:
        build(logs, args.out, tuple(origin), args.basemap)
    except ValueError as exc:
        print(f"utm-viz-cesium: {exc}", file=sys.stderr)
        return 2
    print(args.out / "index.html")
    if args.serve:
        serve(args.out, args.port)
    return 0


PAGE = """<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>UTM decision replay</title>
<script src="https://cdn.jsdelivr.net/npm/cesium@__CESIUM__/Build/Cesium/Cesium.js"></script>
<link href="https://cdn.jsdelivr.net/npm/cesium@__CESIUM__/Build/Cesium/Widgets/widgets.css" rel="stylesheet">
<script src="keys.js"></script>
<style>
html, body, #globe { margin: 0; width: 100%; height: 100%; overflow: hidden; background: #000; font: 13px sans-serif; }
#panel { position: absolute; top: 8px; left: 8px; max-width: 360px; padding: 10px 12px; color: #eee;
         background: rgba(20, 22, 28, 0.85); border-radius: 6px; line-height: 1.45; }
#panel h1 { font-size: 15px; margin: 0 0 4px; } #panel .small { color: #aaa; font-size: 12px; }
#panel button { margin: 2px 4px 2px 0; } .key { display: inline-block; width: 14px; height: 4px; margin-right: 4px; vertical-align: middle; }
</style></head>
<body><div id="globe"></div>
<div id="panel"><h1>UTM decision replay</h1>
<div class="small">Simulation replay, not live data. Every value comes from the decision log.</div>
<div id="runs"></div>
<div><b>Show:</b> <label><input type="radio" name="run" value="on" checked> filter on</label>
<label><input type="radio" name="run" value="off"> filter off</label></div>
<div><b>Speed:</b> <button data-x="0.1">0.1×</button><button data-x="0.25">0.25×</button><button data-x="0.5">0.5×</button><button data-x="1">1×</button></div>
<div class="small"><span class="key" style="background:rgb(70,130,255)"></span>proposed accel
<span class="key" style="background:rgb(40,200,90)"></span>safe accel (2 m per m/s²).<br>
Sphere radius d_eff/2: two spheres touch at d_eff. Colour: red inside d_eff, yellow at 2 d_eff, green from 4 d_eff.
<b>*</b> = filter intervened. Click a drone for its panel.</div>
<div class="small" id="basemap"></div></div>
<script>
const META = __META__;
const KEYS = window.UTM_KEYS || {};
(async () => {
  let mode = META.basemap === "auto" ? (KEYS.ion ? "ion" : "osm") : META.basemap;
  let note = "";
  if (mode === "google" && !KEYS.google) { mode = "osm"; note = " (GOOGLE_MAPS_API_KEY not set)"; }
  if (mode === "ion" && !KEYS.ion) { mode = "osm"; note = " (CESIUM_ION_TOKEN not set)"; }
  if (KEYS.ion) Cesium.Ion.defaultAccessToken = KEYS.ion;
  const osm = new Cesium.ImageryLayer(new Cesium.OpenStreetMapImageryProvider({
    url: "https://tile.openstreetmap.org/", credit: new Cesium.Credit("© OpenStreetMap contributors", true) }));
  const viewer = new Cesium.Viewer("globe", {
    baseLayer: mode === "ion" ? undefined : (mode === "osm" ? osm : false), baseLayerPicker: false, geocoder: false,
    homeButton: false, sceneModePicker: false, navigationHelpButton: false, shouldAnimate: true });
  const text = { osm: "OpenStreetMap imagery on the WGS84 ellipsoid, no terrain" + note,
                 ion: "Cesium World Terrain + Cesium OSM Buildings",
                 google: "3D buildings, terrain and imagery: Google Photorealistic 3D Tiles. Drones and overlays: this replay." };
  document.getElementById("basemap").textContent = "Basemap: " + text[mode] + ". Origin " + META.origin.lat_deg + ", "
    + META.origin.lon_deg + "; world z = 0 at " + META.origin.height_m + " m above the ellipsoid."
    + (mode !== "osm" && META.origin.height_m === 0
       ? " Warning: the ground here is above the ellipsoid, so the drones may be underground; pass --origin LAT,LON,H." : "");
  if (mode === "ion") {
    viewer.scene.setTerrain(Cesium.Terrain.fromWorldTerrain());
    viewer.scene.primitives.add(await Cesium.createOsmBuildingsAsync());
  } else if (mode === "google") {
    Cesium.GoogleMaps.defaultApiKey = KEYS.google;
    viewer.scene.globe.show = false;  // Google tiles carry their own terrain; no mixing with the globe
    const tiles = await Cesium.createGooglePhotorealistic3DTileset(undefined, { showCreditsOnScreen: true });
    tiles.showCreditsOnScreen = true;
    viewer.scene.primitives.add(tiles);
  }
  const sources = {};
  for (const tag of Object.keys(META.runs)) {
    sources[tag] = await viewer.dataSources.add(Cesium.CzmlDataSource.load(tag + ".czml"));
    const r = META.runs[tag];
    const div = document.createElement("div"), small = document.createElement("div");
    div.textContent = "filter " + tag + ": " + r.episode_id + ", min sep "
      + (r.min_sep_m === null ? "n/a" : r.min_sep_m.toFixed(2) + " m");
    small.className = "small";
    small.textContent = r.backend + " (" + r.truth_source + "), script " + (r.script_sha256 || "none").slice(0, 12) + "…";
    div.append(small); document.getElementById("runs").append(div);
  }
  const first = sources.on || sources.off;
  viewer.clock.startTime = first.clock.startTime; viewer.clock.stopTime = first.clock.stopTime;
  viewer.clock.currentTime = first.clock.currentTime; viewer.clock.multiplier = first.clock.multiplier;
  viewer.clock.clockRange = Cesium.ClockRange.LOOP_STOP;
  viewer.timeline.zoomTo(viewer.clock.startTime, viewer.clock.stopTime);
  const show = (tag) => { for (const t in sources) sources[t].show = t === tag; viewer.selectedEntity = undefined; };
  document.querySelectorAll("input[name=run]").forEach((el) => {
    el.disabled = !sources[el.value]; el.onchange = () => show(el.value); });
  show(sources.on ? "on" : "off");
  if (!sources.on) document.querySelector("input[value=off]").checked = true;
  document.querySelectorAll("button[data-x]").forEach((b) => b.onclick = () => viewer.clock.multiplier = Number(b.dataset.x));
  const c = META.camera;
  viewer.camera.lookAt(new Cesium.Cartesian3(...c.target_ecef_m),
                       new Cesium.HeadingPitchRange(Cesium.Math.toRadians(20), Cesium.Math.toRadians(-35), c.range_m));
  viewer.camera.lookAtTransform(Cesium.Matrix4.IDENTITY);
  window.UTM_VIEWER = viewer;  // for the browser console and scripted screenshots
})();
</script></body></html>
"""

if __name__ == "__main__":
    sys.exit(main())
