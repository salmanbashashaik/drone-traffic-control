# Quickstart: run the MARL drone simulation and look at it in 3D

This gets a new person from a fresh clone to a 3D, rotatable replay of trained drones flying through a city, and shows how to evaluate and retrain the policy. Everything here is simulation only. The learned policy is a nominal action proposer, not a safety controller; read [`MARL_TRAINING.md`](./MARL_TRAINING.md) for what the results do and do not show.

Sections 2, 3, 5 and 6 were run from a fresh clone of the pushed branch in a brand-new virtual environment on macOS (Apple silicon, Python 3.11). Section 4 (PyBullet) was run in my working environment only, because compiling PyBullet takes several minutes and I did not repeat it in a clean one. Linux and Windows steps that I could not run myself are marked **untested**.

## 1. Requirements

- **Python 3.11 or newer.** Check with `python3 --version`. On macOS the system `python3` can be older (3.9 on the machine this was written on); use the one you installed, for example `python3.11` or `python3.12` (`brew install python@3.11`).
- Git, and about 1.5 GB of disk for the Python packages (most of it PyTorch).
- A browser (Chrome, Safari, Firefox, Edge) for the 3D viewer. No GPU needed, no internet needed after install.

## 2. Install

```bash
git clone https://github.com/salmanbashashaik/drone-traffic-control.git
cd drone-traffic-control
git checkout salman/marl-safety        # until PR #20 is merged; after that, stay on main

python3.11 -m venv .venv               # use python3.12 if that is what you have
source .venv/bin/activate              # Windows PowerShell: .\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -e ".[dev,train,plot]"
```

What the extras are: `dev` = pytest/ruff, `train` = PyTorch (only needed to train; running a trained policy needs only numpy), `plot` = matplotlib (2D plots). Check the install:

```bash
utm-replay --help
python -m pytest tests/test_contracts.py tests/test_reference_policy.py tests/test_simulation_env.py -q
```

## 3. See the drones (the quick way, about 15 seconds)

Fastest: **random drones, random start and destination points, every run**:

```bash
utm-replay random:6 --artifact artifacts/policies/marl-mappo-s42-it300-08e745b853 \
    --variant shielded_marl --html --out runs/viz/random6
```

Change `6` to any number of drones. It prints the seed it picked so you can reproduce the run (`heldout:<seed>:<n>`). Then open `runs/viz/random6/replay3d.html`. A fixed, repeatable example:

```bash
utm-replay heldout:18000044:6 \
    --artifact artifacts/policies/marl-mappo-s42-it300-08e745b853 \
    --variant shielded_marl --html --out runs/viz/six-shielded
```

Then open `runs/viz/six-shielded/replay3d.html` (macOS: `open runs/viz/six-shielded/replay3d.html`; Linux: `xdg-open ...` **untested**; Windows: `start ...` **untested**; or just double-click the file).

What you get in the output folder:

| File | What it is |
|---|---|
| `replay3d.html` | the interactive 3D viewer (one self-contained file; works offline) |
| `replay.png` | top-down tracks, minimum drone separation, minimum distance to buildings |
| `replay.gif` | animated top-down view |

The command also prints a result line (drones arrived, minimum separation, violation pair-steps, filter interventions, clearance to buildings). Lines starting with `filter:` are the safety filter's own log messages and are normal.

### Using the 3D viewer

- **Drag** to orbit, **right-drag** (or Shift-drag) to pan, **wheel** to zoom.
- Keys: `1` iso, `2` top, `3` side, `4` front, `F` follow the next drone, `R` reset, **Space** play/pause.
- Bottom bar: time scrubber, speed (0.25x to 4x), follow-a-drone menu, toggles for trails, halos, labels, **see-through buildings** and the 1 km airspace box, and a drone-size slider.
- Right panel: each drone's speed and altitude. A halo is **amber** while the safety filter is changing that drone's command and **red** when two drones are closer than 5 m.
- Drones are drawn about 20 times their real size and level (the simulator has no tilt). Buildings are scenery: the policy and the filter do not see them.

### The head-on test: two drones swap places

`swap:` builds a deliberate collision course: **d0 starts where d1 wants to go and d1 starts where d0 wants to go**, 25 m apart, at the same altitude and at rest, plus 4 other drones with random routes (6 in total).

```bash
utm-replay swap:24900003:6 --artifact artifacts/policies/marl-mappo-s42-it300-08e745b853 \
    --variant unshielded_marl --html --out runs/viz/swap-unshielded
utm-replay swap:24900003:6 --artifact artifacts/policies/marl-mappo-s42-it300-08e745b853 \
    --variant shielded_marl --html --out runs/viz/swap-shielded
```

Open both `replay3d.html` files. Without the filter, d0 and d1 only drift 1.2 m and 0.7 m off their straight lines and pass through each other (closest 1.70 m, 7 violation pair-steps in this episode). With the filter, it acts on exactly those two drones for about 1.8 s (0.9 to 2.7 s), they sidestep 5.2 m and 2.4 m, and the closest approach is 6.68 m; the other four drones are untouched. In the viewer, click **Follow d0** and drag the time slider to about 2.3 s: both drones have an amber halo and "filter active" in the right panel. The command also prints an `avoidance:` block (how far each drone strayed from its straight route and when the filter acted).

Forms: `swap:<seed>` (repeatable), `swap:random` (new random scenario every run, seed printed), `swap:<seed|random>:<n>:<gap>` (for example `swap:random:6:20` for 20 m, any gap you like; 20 to 30 m is the range tested). Seed `24900003` was picked from 20 probe seeds as a clear example, so it is not a typical episode.

### Compare with and without the safety filter

Run the same scenario with `--variant unshielded_marl` (learned policy alone) and `--variant shielded_marl` (policy plus the HOCBF safety filter), then open both pages:

```bash
utm-replay heldout:18000044:6 --artifact artifacts/policies/marl-mappo-s42-it300-08e745b853 \
    --variant unshielded_marl --html --out runs/viz/six-unshielded
```

In that scenario the unshielded run ends with 6 violation pair-steps (closest 3.83 m, below the 5 m limit) and the shielded run has none (closest 7.81 m). It is the worst of 50 6-drone episodes, picked to show the filter working; it is not typical.

### Choosing a scenario

The first argument of `utm-replay` is one of:

- `swap:<seed|random>[:<n>[:<gap>]]`: the mirrored head-on above (default 6 drones, 25 m).
- `random:<n>`: `n` drones with new random starts and destinations every run (add `:0.3` for fewer forced head-ons, default conflict is 0.8).
- `heldout:<seed>:<n>`: a generated scenario with `n` drones (1 or more; the policy was trained on 2 to 4, so 6 or more is outside its training), for example `heldout:12000008:4` or `heldout:18000044:6`. The same seed always gives the same scenario.
- `dense16:<seed>`: 16 drones with random starts and goals.
- a path to a `ScenarioConfig` JSON file, for example `scenarios/head-on-40.json` (two drones flying at each other).

Other useful options: `--buildings N` (default 26, `0` for none), `--profile nominal|realistic` (wind, noise, delay and dropout; the T1-trained artifact does badly there, see below), and no `--artifact` to fly the hand-written reference controller instead.

### Which policy file?

| `--artifact` | Trained on | Use it for |
|---|---|---|
| `artifacts/policies/marl-mappo-s42-it300-08e745b853` | ideal dynamics (T1) | the default for everything above |
| `artifacts/policies/marl-mappo-s42-it220-f93dca3638` | simulated wind/noise/delay/dropout (T2 "nominal") | runs with `--profile nominal` or `--profile realistic` |

Loading checks the weight hash and the observation/action settings and refuses a mismatched or altered artifact.

## 4. Fly it in a physics simulator (optional, harder to install)

`--pybullet` runs the same policy in a rigid-body quadrotor simulator (PyBullet, Crazyflie model); the policy was trained on a simpler point-mass model, so this is a transfer test.

```bash
# macOS on Apple silicon (the two workarounds were needed on macOS 26; they are harmless otherwise)
CFLAGS="-Dfdopen=fdopen" python -m pip install "pybullet>=3.2.7,<4" "gymnasium>=1.0,<2" "transforms3d>=0.4,<1"
python -m pip install --no-deps --ignore-requires-python \
    "gym-pybullet-drones @ git+https://github.com/utiasDSL/gym-pybullet-drones.git@7ebad1e"

utm-replay heldout:12000008:4 --artifact artifacts/policies/marl-mappo-s42-it300-08e745b853 \
    --variant shielded_marl --pybullet --snapshots 6 --out runs/viz/pybullet
```

This section is optional; nothing else depends on it, and the simulation in sections 3, 5 and 6 does not need PyBullet. `--snapshots N` renders the 3D world every N steps to `scene_start.png`, `scene_mid.png`, `scene_end.png` and `scene.gif`. `--gui` opens a live PyBullet window instead (add `--speed 0.6`; close the window to exit). The window is slow, and on some machines it was not usable, so the browser viewer above is the supported way to look around. Linux: `python -m pip install -e ".[physics]"` then the `gym-pybullet-drones` line (**untested**). Windows has no PyBullet wheels.

## 5. Measure it (evaluation)

```bash
utm-eval-policy artifacts/policies/marl-mappo-s42-it300-08e745b853 \
    --suite six --episodes 50 --out runs/eval/six
```

It flies four variants on identical scenarios: `reference` (hand-written controller), `unshielded_marl`, `shielded_marl`, and `shielded_reference` (reference plus filter, as a baseline). It writes `summary.md` (the table), `episodes.jsonl` (one row per episode) and `manifest.json`. Suites: `heldout` (2 to 4 drones), `six` (6 drones), `swap` (6 drones, two of them swapping places 25 m apart; `--swap-gap` changes the distance), `dense16` (16 drones), `shipped` (the three scenarios in `scenarios/`). Time: a few seconds per episode, so 50 episodes of `six` takes roughly 10 to 15 minutes; use `--episodes 3` to try it. Other options: `--profile`, `--seed-base`.

To measure how close the drones came to the buildings over many scenarios: `python -m utm_services.training.city_eval <artifact> --episodes 50 --out runs/eval/city`.

## 6. Train a policy (optional)

```bash
utm-train --out runs/train/my-run --iterations 300 --workers 4
```

About 2 minutes on a laptop CPU for 300 iterations. The same seed gives the same weights. Outputs in `runs/train/my-run/`: `train_log.jsonl`, `config.json`, `checkpoint.pt`, and `artifact/` (the best checkpoint chosen on validation seeds, ready for `--artifact`) and `artifact_last/`. Add `--profile nominal` to train under simulated disturbances, `--resume` to continue, `--help` for all settings. `runs/` is git-ignored.

A quick check that training runs end to end (about 10 seconds):

```bash
utm-train --out runs/train/smoke --iterations 3 --episodes-per-iter 4 --bc-episodes 8 --bc-epochs 2 --eval-every 3 --val-episodes 4
```

## 7. What the numbers mean

Read [`MARL_TRAINING.md`](./MARL_TRAINING.md) before quoting any result. In short: the learned policy alone still loses 5 m separation in a few percent of episodes; the safety filter is what removes violations in these runs; zero violations in a finite set of simulated seeds is not evidence of zero risk; buildings are not avoided by the controller; and the policy was trained on ideal dynamics only (the T2 artifact on simulated disturbances with placeholder parameters).

## 8. Troubleshooting

| Symptom | Fix |
|---|---|
| `requires a different Python` / syntax errors | the interpreter is older than 3.11; create the venv with `python3.11` or `python3.12` |
| `utm-replay: command not found` | activate the venv (`source .venv/bin/activate`) or run `python -m utm_services.training.replay ...` |
| `No module named 'torch'` | you skipped the `train` extra; it is only needed for `utm-train` |
| `ModuleNotFoundError: matplotlib` | install the `plot` extra: `python -m pip install -e ".[plot]"` |
| `ArtifactError ... normalisation fingerprint` | the artifact was made by a different version of `policy/learned.py`; retrain or use the shipped artifacts |
| `ModuleNotFoundError: gym_pybullet_drones` | only needed for `--pybullet`; see section 4 |
| 3D page is blank | use a current browser with WebGL; check that the file is the one from this run (`ls -la replay3d.html` should be about 0.7 MB) |
| many `filter: barrier violated ...` lines | normal log output of the safety filter when it is working on a conflict |
