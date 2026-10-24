# MAPPO training loop, numpy policy runtime and two trained nominal-policy artifacts

Adds a custom single-process MAPPO trainer (`training/mappo.py`, `rollout.py`, `scenarios.py`), a torch-free actor runtime with artifact save/load checks (`policy/learned.py`), a matched evaluator and replay renderer, two frozen actors (T1-trained, T2-nominal-trained), per-episode evaluation rows and docs. No filter, contract or simulator code changes. The committed numbers recompute exactly from the episode rows and the T1 artifact reproduces bit-identically; the problems are in how the evidence is presented and in artifact provenance.

Watch for: (1) the "fresh seeds, evaluated once" claim is false, since seeds 12,000,000+ were used for at least four evaluation runs including the decision not to ship the shield-in-loop policy (confirmed); (2) the shield-in-loop "no benefit" conclusion compares a 150-iteration run to a 300-iteration run and has no log, artifact or commit behind it (confirmed); (3) the T2 artifact's `metadata.json` says "Trained and validated on T1 (ideal) dynamics only" (confirmed); (4) the dev-seed numbers behind the T1-vs-T2 narrative have no rows or manifest and cannot be recomputed (confirmed).

**Verdict**: NEEDS_CHANGES

## High-level view

The trainer has no correctness bug I could find, but a few small training-loop issues are listed below. The artifact checks are weaker than the doc suggests: the normalisation check compares description strings rather than the numeric constants, and the hash sits next to the weights it protects, so it guards against accidents rather than substitution. The T2 artifact's metadata carries a hard-coded T1-only limitation that contradicts its own `disturbance_tier_trained`.

Where the weakness lies is the presentation. The fresh seed range was reused across artifacts and decisions, one design conclusion rests on a confounded comparison, the dev-seed evidence has no rows, and two safety phrases imply more than the data shows. No tests accompany any of the new code.

<details>
<summary>Issues (11)</summary>

1. **"Evaluated once" is false** (confirmed, MEDIUM) — seeds 12,000,000–12,000,049 were used for the it150 shield-in-loop run, T1 it300, T2 it220 ideal and T2 it220 realistic. Reword `MARL_TRAINING.md:53`, `DECISIONS.md:41`, and `scenarios.py:14` to say what was done, or draw new seeds for any further decision.
2. **Shield-in-loop "no benefit" is confounded** (confirmed, MEDIUM) — it150 vs it300 differs in length (curriculum and LR schedule scale with `iterations`), one seed, no train log, artifact, rows or commit for it150, and the decision was taken on the reused fresh seeds. Retrain at 300 iterations or soften to "inconclusive" (`MARL_TRAINING.md:87`, `DECISIONS.md:44`).
3. **T2 artifact metadata contradicts itself** (confirmed, MEDIUM) — `mappo.py:370` hard-codes "Trained and validated on T1 (ideal) dynamics only"; the it220 `metadata.json` carries it next to `disturbance_tier_trained: T2 profile 'nominal'`. Derive the limitation from `cfg.profile` and regenerate or hand-correct the shipped metadata (the weight hash does not cover metadata, so the fix does not change the artifact id).
4. **Dev-seed evidence not recomputable** (confirmed, MEDIUM) — the dev-seed summaries (78%/84% completion, shipped-scenario numbers) have no `episodes.jsonl` or manifest, and the it150 summary has none either. Commit the rows, or label those figures as unverifiable summaries.
5. **T1→T2 "removed the completion gap" compares different seed sets** (confirmed, LOW) — the T1 policy under T2 was measured on dev seeds (30 episodes); the T2 policy on fresh seeds. Run the T1 policy under nominal/realistic on the same fresh seeds, or drop the causal phrasing at `MARL_TRAINING.md:86`.
6. **Fresh T2 evaluations have no manifest** (confirmed, LOW) — only the T1 fresh run has `manifest.json` (commit, filter config, seed base). Add manifests for the two T2 runs.
7. **Plot scenario chosen post hoc and not labelled** (confirmed, LOW) — `heldout:12000008:4` is the worst unshielded case of 50 (1.63 m, the global minimum, and one of only three violating episodes) and the lowest shielded minimum (8.08 m). In that scenario the shielded reference reaches 8.68 m, above the learned policy. Say it is the worst case, not a typical one.
8. **Two phrases imply safety inside the tested envelope** (confirmed, LOW) — "nothing here shows … safe *outside* the simulated conditions" (`MARL_TRAINING.md:3`) and "Safety in these runs comes from the filter" (`:96`); also "Shielded separation still held" (`:86`). Replace with measured-separation wording. The table also shows one shielded dense16 episode inside d_eff; mention it in the prose.
9. **Normalisation check is textual** (confirmed, LOW) — `learned.py:203` compares `OBS_NORMALIZATION` strings; `NEIGHBOUR_GATE_M`, `_GOAL_SCALE_M`, `_SHIFT`/`_SCALE` can change without the strings or `OBS_NORM_VERSION` changing, and old artifacts would load. Build the strings from the constants or add a numeric fingerprint to metadata. Also sanitise `weights_file` (`:214` joins an unvalidated metadata path), and note the hash is integrity-only.
10. **Small training-loop issues** (likely/confirmed, LOW) — T2 training treats a sampled action as executed even when the env dropped the command (`env.py:280`); BC labels come from the stateless PD `compute_reference_action` while the evaluated "reference" is the PID `ReferencePolicy`, and BC jobs ignore `profile` (`mappo.py:235`); resume duplicates `train_log.jsonl` rows after the last checkpoint (`mappo.py:431`) and uses `torch.load(weights_only=False)` (`:399`). Document or fix.
11. **No tests** (confirmed, LOW) — no test touches GAE, `normalize_obs`, rollout masking, or the loader's refusal paths (missing metadata, altered weights, bound mismatch); `torch` is not in the `dev` extra. Add numpy-only tests for the loader and `normalize_obs`, and a small GAE test behind `importorskip("torch")`.

</details>

<details>
<summary>Details</summary>

### Fresh seeds reused across artifacts and decisions

`MARL_TRAINING.md:53` titles the fresh-seed section "evaluated once after the design was frozen", `DECISIONS.md:41` says "The frozen design was evaluated once", and `scenarios.py:14` documents `FINAL_SEED_BASE` as "run once". The committed files show otherwise. Four evaluations used seeds 12,000,000–12,000,049: the it150 shield-in-loop policy (`…it150….fresh-ideal.summary.md`), T1 it300 (`fresh-seeds`), T2 it220 ideal and T2 it220 realistic. The it150 summary's reference row (min sep 1.05, 64 violation pair-steps, 23 episodes, 226.02 mean steps) is identical to the other runs' reference row, which confirms it is the same 50 scenarios. The decision to leave it150 out of the shipped set was then taken from those numbers ("No benefit on fresh seeds"). The T2 policy was also designed after seeing the T1 result. The numbers are not wrong, but the seed range is a test set that has been consulted several times, and the doc's framing of it as a single untouched look overstates the evidence. The dense16 suite is similarly the only place where the policy is far outside its trained team size, and it uses the same seeds.

### Shield-in-loop comparison

The it150 config has `shield: true` and `iterations: 150`. The curriculum ramp is `curriculum_frac * iterations` and the LR decays over `iterations`, so a 150-iteration run is a different schedule, not a truncated T1 run. The comparison the doc makes (14 violation pair-steps vs 10, 498 interventions vs 324) is also 3 vs 5 violating episodes out of 50, which is well inside what seed-to-seed variation could produce, and the doc itself says run-to-run variation across training seeds is not measured. Only the summary table survives: there is no `train_log.jsonl`, no artifact, no episode rows and no recorded commit for it150, and its config predates the `profile` field. The conclusion belongs as "not tested fairly", not "no benefit".

### Artifact provenance

Recorded commits `6f4c636` (T1) and `1b650e7` (T2) both exist on this branch with `src_dirty: false`, and the recorded `weights_sha256` matches the recomputed hash for both artifacts. I reran the T1 training at HEAD in a scratch directory (300 iterations, 4 workers, about 97 s) and got `weights_sha256 = 08e745b853…`, identical to the committed artifact, so the doc's reproducibility claim holds for T1. The T2 rerun was not done.

The one inconsistency is `limitations` in `_metadata` (`mappo.py:370`), which is written for every run regardless of `cfg.profile`. The T2 artifact therefore ships with "Trained and validated on T1 (ideal) dynamics only; no claim about disturbances (T2+)", directly beside `disturbance_tier_trained: T2 profile 'nominal'` and `hyperparameters.profile: nominal`. Anyone reading the metadata alone cannot tell which is true.

### Evidence that cannot be recomputed

The fresh-seed aggregates match the summaries and rows exactly (I recomputed all 20 suite/variant groups across the three `episodes.jsonl` files, including the four min-separation deltas in the doc). The claims that have no rows are the dev-seed ones: the 78%/84% completion drop, the shipped-scenario `sandwich-3` equality at 13 violation pair-steps, and the it150 numbers. The causal story in the doc (T1 policy fails under disturbance, T2 training fixes it) compares a dev-seed 30-episode measurement of one policy with a fresh-seed 50-episode measurement of another, and only the T1 fresh run has a manifest.

### Plot scenario

The three PNGs are for seed 12,000,008 with 4 drones. In the T1 fresh rows that seed has the lowest unshielded minimum (1.63 m, equal to the suite minimum) and the lowest shielded minimum (8.08 m), and it accounts for 4 of the 10 unshielded violation pair-steps. The reference there is 1.64 m, not its worst (1.05 m at 12,000,011). Replaying it at HEAD reproduces 1.63 m / 4 violation pair-steps unshielded and 8.08 m / 44 interventions shielded, so the figures are honest; they just show a failure case, which the doc presents as "the same fresh-seed scenario flown three ways". Using the worst case to show the filter working is the better choice than a typical case, but it should say so.

### Safety wording

Three phrases leak a stronger claim than the data. "Nothing here shows that it or the filter is safe outside the simulated conditions" implies it was shown safe inside them. "Safety in these runs comes from the filter" attributes a property to the run rather than a measurement. "Shielded separation still held (0 violations)" is on 30 dev episodes. Separately, the results table shows one shielded dense16 episode inside d_eff under the realistic profile (also true for the shielded reference), which is not mentioned in the prose even though the surrounding text emphasises "0 violations".

### Loader checks

The normalisation check at `learned.py:203` compares the `OBS_NORMALIZATION` dict of prose strings against the metadata copy; the real parameters are the module constants (`NEIGHBOUR_GATE_M = 250`, `_GOAL_SCALE_M = 20`, `_SHIFT`, `_SCALE`) and the string "250 m" is written separately from the constant. A change to a constant that leaves the strings alone loads every existing artifact with silently different inputs. And `weights_file` is read from metadata and joined to the directory unchecked, while the hash is stored in the same metadata file, so the check detects corruption or a mixed-up directory, not a deliberate swap of both files.

### Training-loop issues

GAE terminal and truncation handling and slot/alive masking are correct; no bug found. The remaining items are minor. Under T2 profiles the environment drops a command with probability 0.05 or 0.1 (`env.py:280`), but the rollout stores the sampled action as if it had been executed, so policy-gradient credit is noisy for those steps (likely; only affects T2 training efficiency). Behaviour-cloning labels come from `compute_reference_action`, the stateless PD law, while the `reference` and `shielded_reference` variants evaluated in the tables use `ReferencePolicy`, which has an integral term; the docs call both "the reference". The BC jobs (`mappo.py:235`) carry no `profile`, so the T2 run is warm-started from ideal dynamics. Resume saves a checkpoint only every `eval_every` iterations while `train_log.jsonl` is appended every iteration (`mappo.py:431`), so resuming after a crash duplicates log rows for the iterations after the last checkpoint, and `torch.load(..., weights_only=False)` at `:399` unpickles arbitrary objects from the checkpoint file.

### Test coverage

The diff adds no tests and does not touch CI. Not tested: GAE terminal and truncation handling, slot/alive masking, `normalize_obs` (including the neighbour gate and the arcsinh goal scaling), the loader's refusal paths, `save_policy_artifact`/`load` round trip, and determinism of the trainer. Torch is only in the optional `train` extra, so trainer tests would need `importorskip`; the runtime and loader tests would not.

</details>

<details>
<summary>File map</summary>

- `src/utm_services/training/mappo.py` — MAPPO trainer, BC warm start, validation/checkpoint selection, artifact metadata, CLI
- `src/utm_services/training/rollout.py` — torch-free episode rollouts, slot/alive arrays, BC labels
- `src/utm_services/training/scenarios.py` — procedural scenario family and seed ranges
- `src/utm_services/training/evaluate.py` — matched evaluator, aggregate table, manifest
- `src/utm_services/training/replay.py` — PNG/GIF replay renderer
- `src/utm_services/policy/learned.py` — numpy actor, `obs-norm-v3`, artifact save/load and checks
- `artifacts/policies/*` — two frozen actors and metadata
- `docs/results/training/*` — configs, train logs, summaries, episode rows, one manifest
- `docs/training/MARL_TRAINING.md`, `docs/training/figures/*` — usage, results, plots
- `docs/formulation/P1-SA-01-method.md` §9, `docs/DECISIONS.md` — deviations and decisions
- `pyproject.toml` — `train` extra, three console scripts

Full diff: `git diff origin/main...HEAD` on `salman/marl-safety`.

</details>
