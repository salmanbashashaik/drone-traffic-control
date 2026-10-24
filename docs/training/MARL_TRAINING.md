# MARL training loop (P2-SA-01)

What this is: a MAPPO training loop for the **nominal** policy, a numpy runtime that loads the resulting artifact, a matched evaluator, and a replay renderer. It is simulation-only. The learned policy proposes accelerations; it is not a safety controller. Every number below is a measured separation or completion count from seeded simulation; none of it is a guarantee, and none of it says the policy or the filter is safe.

Method choices and their deviations from the draft: `docs/formulation/P1-SA-01-method.md` §9. Reward: `P1-SA-01-reward.md` (`P1-SA-01-reward-v1`, implemented in `simulation/env.py`). Training reward is not a safety metric; every number below is a measured separation or completion quantity.

## Run it

```
pip install -e ".[dev,train,plot]"                       # torch==2.5.1 only for training; the runtime needs numpy

# train (about 2 min on a laptop; seeded; bit-identical weights on rerun)
utm-train --out runs/train/my-run --iterations 300 --workers 4              # T1 (ideal) dynamics
utm-train --out runs/train/my-t2  --iterations 300 --workers 4 --profile nominal   # T2-nominal

# evaluate: reference / unshielded / shielded / shielded-reference on identical scenarios
utm-eval-policy artifacts/policies/marl-mappo-s42-it220-f93dca3638 \
    --suite heldout --suite dense16 --episodes 50 --seed-base 12000000 --profile realistic --out runs/eval/x

# watch it: replay.png (tracks + separation) and replay.gif (animated top-down)
utm-replay heldout:12000008:4 --artifact artifacts/policies/marl-mappo-s42-it300-08e745b853 \
    --variant shielded_marl --out runs/viz/x
```

Load a policy in code: `LearnedPolicy.load(dir)` (`policy/learned.py`). It satisfies the existing `PolicyFn` protocol, so `run_episode(config, "unshielded_marl", policy=..., policy_id=policy.artifact_id)` works unchanged. Loading refuses a missing `metadata.json`, an altered weight file (SHA-256), a different observation normalisation, or different world bounds than `shared/contracts.py`.

## What was built

| Piece | File |
|---|---|
| MAPPO trainer (shared actor, centralised critic, BC warm start, validation-seed checkpoint selection, resume) | `training/mappo.py` |
| Torch-free episode rollouts on the unmodified `DroneEnv` | `training/rollout.py` |
| Procedural scenarios; disjoint train / validation / development-test / final seed ranges | `training/scenarios.py` |
| Numpy actor runtime, observation normalisation (`obs-norm-v3`), artifact save/load | `policy/learned.py` |
| Matched evaluator and aggregate table | `training/evaluate.py` |
| Replay renderer | `training/replay.py` |
| Frozen artifacts | `artifacts/policies/<id>/{actor.npz,metadata.json}` |
| Training logs, configs, evaluation summaries and per-episode rows | `docs/results/training/` |

## Artifacts

| Artifact | Trained on | Selected at | Source commit |
|---|---|---|---|
| `marl-mappo-s42-it300-08e745b853` | T1 (ideal) | iteration 300 (validation) | `6f4c636`, clean tree |
| `marl-mappo-s42-it220-f93dca3638` | T2 `nominal` profile (placeholder parameters) | iteration 220 (validation) | `1b650e7`, clean tree |

Both: network 18-256-256-3 tanh, training seed 42, 2–4 agents per episode, behaviour-cloning warm start from the reference controller, 2.7M (T1) and 3.0M (T2) agent-steps. Rerunning the T1 config on the same machine gave the same weight hash, also across an unrelated code edit.

## Results

Variants on identical scenarios: `reference` (PD controller, no filter), `unshielded_marl` (learned policy, no filter), `shielded_marl` (learned policy + HOCBF filter, default `FilterConfig`), `shielded_reference` (reference + filter, a baseline for context). "Viol." is separation-violation pair-steps below 5 m (`INTERFACES.md`). All runs completed 0 aborts, 0 infeasible steps and 0 fallbacks unless a column says otherwise.

### Fresh seeds (12,000,000+)

These seeds were not used for training, checkpoint selection, or any preprocessing change. They were consulted more than once after the design was frozen: for the T1 policy, for the T2-nominal policy under two profiles, and for deciding not to ship the first shield-in-loop run. Treat them as a repeatedly used test set, not a single untouched look.

Policy `…it300-08e745b853`, ideal (T1) dynamics, 50 episodes per suite. Held-out = 2–4 drones, conflict 0.8, trained team size. Dense16 = 16 drones with random starts and goals, outside the trained team size.

| Suite | Variant | Completion | Min sep (m) | Viol. | Episodes with viol. | Interventions |
|---|---|---|---|---|---|---|
| held-out | reference | 100% | 1.05 | 64 | 23 | 0 |
| held-out | unshielded learned | 100% | 1.63 | 10 | 3 | 0 |
| held-out | shielded learned | 100% | 8.08 | 0 | 0 | 324 |
| held-out | shielded reference | 100% | 7.86 | 0 | 0 | 1602 |
| dense16 | reference | 100% | 0.79 | 110 | 20 | 0 |
| dense16 | unshielded learned | 100% | 0.78 | 52 | 11 | 0 |
| dense16 | shielded learned | 100% | 5.85 | 0 | 0 | 1937 |
| dense16 | shielded reference | 100% | 5.59 | 0 | 0 | 4463 |

Policy `…it220-f93dca3638` (trained on T2-nominal), 50 episodes:

| Evaluated under | Suite | Variant | Completion | Min sep (m) | Viol. | Episodes inside d_eff |
|---|---|---|---|---|---|---|
| ideal | held-out | unshielded learned | 100% | 4.34 | 3 | 2 |
| ideal | held-out | shielded learned | 100% | 8.29 | 0 | 0 |
| realistic (not trained on) | held-out | unshielded learned | 100% | 3.75 | 4 | 3 |
| realistic (not trained on) | held-out | shielded learned | 100% | 8.08 | 0 | 0 |
| realistic (not trained on) | dense16 | unshielded learned | 100% | 1.36 | 71 | 14 |
| realistic (not trained on) | dense16 | shielded learned | 100% | 5.14 | 0 | 1 |

One shielded dense16 episode (T2-realistic) ended up inside d_eff (5.5 m) although no pair went below 5 m; the shielded reference did the same on that suite. Zero violation pair-steps there does not mean the filter held d_eff throughout.

Same 50 fresh seeds, **T1-trained** policy (`…it300-08e745b853`) under the T2 profiles it was not trained on, against the T2-nominal-trained policy above (held-out suite, 50 episodes each):

| Profile | Policy | Variant | Completion | Aborted | Min sep (m) | Viol. |
|---|---|---|---|---|---|---|
| nominal | T1-trained | unshielded | 75.2% | 1 | 2.59 | 8 |
| nominal | T1-trained | shielded | 77.2% | 0 | 7.79 | 0 |
| realistic | T1-trained | unshielded | 83.2% | 1 | 2.53 | 5 |
| realistic | T1-trained | shielded | 83.9% | 0 | 6.97 | 0 |
| realistic | T2-nominal-trained | unshielded | 100% | 0 | 3.75 | 4 |
| realistic | T2-nominal-trained | shielded | 100% | 0 | 8.08 | 0 |

In the same runs the reference controller completes 100% (it has an integral term). Rows: `…fresh-seeds.T2-*.episodes.jsonl`, `…fresh-ideal`/`…fresh-realistic` for the T2-trained policy.

Full tables including the reference variants: `docs/results/training/*.summary.md`.

### Development seeds (not clean held-out)

Seeds 9,000,000+ and the three shipped scenarios were used to diagnose failures, and two preprocessing changes came from them. They are reported as development evidence only. The fresh-seed results above show the same pattern (held-out, shielded learned: 0 violations in both).

- Shipped scenarios (3 episodes; rows in `…dev-seeds.ideal.episodes.jsonl`): the shielded learned policy is the same as the shielded reference on `sandwich-3` (13 violation pair-steps; that scenario is documented as physically unrecoverable and the filter does not fully hold it).
- **T1-trained policy under T2 disturbances:** the table above, on the same fresh seeds, is the comparison. Earlier development-seed runs (30 episodes) gave the same picture (78% / 84%, rows in `…dev-seeds.T2-*.episodes.jsonl`). Training under T2-nominal removed the completion gap in these runs. Why is not isolated; a stateless policy cannot cancel a steady wind, which the reference's integral term does. That explanation is a hypothesis, and each configuration has one training seed.
- **Shield in the training loop: inconclusive.** A first 150-iteration run was evaluated on the fresh seeds (14 unshielded violation pair-steps in 5 episodes against 10 in 3 for the T1 run; 498 interventions against 324). It is not a fair comparison: 150 vs 300 iterations changes the curriculum and learning-rate schedule, it is one seed, and the evidence files of that run were not kept, so it is not shipped and not cited as a result. A full-length 300-iteration shield-in-loop run is the proper test.

### What the plots show

`docs/training/figures/` has one fresh-seed scenario (`heldout:12000008:4`) flown three ways. I picked it as the worst unshielded case of the 50 (1.63 m, the suite minimum; 4 of the 10 unshielded violation pair-steps), to show the filter working, so it is not typical. In that scenario the shielded reference reaches a larger minimum separation (8.68 m) than the shielded learned policy (8.08 m). The reference (1.64 m) and the unshielded learned policy (1.63 m) both lose separation; the unshielded plot shows two drones crossing at about 2.7 s. With the filter the minimum is 8.08 m. The learned policy alone does not solve this conflict; the filter does.

## What these results do not show

- Everything is on the project's point-mass simulator with perfect dynamics (T1) or the T2 profiles, whose parameters are placeholders (`simulation/env.py`). No real telemetry, no hardware, no PyBullet or other dynamics were used.
- Zero violations in a finite set of seeds does not establish zero risk. In these runs the filter is the component that changes the outcome; its limits are in `docs/formulation/P2-SA-02-lateral-escape.md` and `docs/DECISIONS.md` (open items BL-03 to BL-12 apply unchanged).
- The unshielded learned policy still violates separation in 3/50 held-out and 11/50 dense16 episodes. It is not a safe controller.
- Unshielded, the learned policy has fewer violation pair-steps than the reference (10 vs 64 held-out, 52 vs 110 dense16) and completes as often on these suites. Shielded, its minimum separation is within 0.35 m of the shielded reference in either direction across the four comparisons above (+0.22, +0.26, −0.12, −0.31 m), so no separation advantage is claimed. It needs fewer filter interventions (324 vs 1602 held-out), which is a count, not a safety measure.
- The T2-trained policy was evaluated on a profile milder than any real flight and on a profile harsher than it was trained on, both simulated. It says nothing about other disturbance types, larger teams in training, obstacles, or geofences (the observation has no geofence field).
- Model selection used 60 validation episodes at one conflict level; a tie went to the later checkpoint. Selection noise is not quantified. One training seed per configuration; run-to-run variation across training seeds is not measured.
- Step latency figures in the summaries include the filter and the Python loop on a laptop, not an edge device (P4).

## Known gaps

- R3 (abort penalty) is not implemented. One abort occurred, in the T1-trained policy under T2 profiles (unshielded, fresh seeds); none in the shipped T2-trained policy's runs.
- Only one training seed per configuration; a multi-seed study is needed before comparing configurations.
- `dense16` generator in `evaluate.py` copies the one in `tests/test_random_traffic.py`; keep them in sync.
- The `shipped` suite has 3 episodes; treat it as a smoke check.
- A PyBullet backend run of the trained policy was not done.
- Training details that were reviewed and left as they are: behaviour cloning labels come from the stateless PD law (`compute_reference_action`), while the evaluated "reference" is the PID `ReferencePolicy`; the behaviour-cloning rollouts always use ideal dynamics, so the T2 run is warm-started from T1 conditions; under T2 profiles a sampled action is stored as executed even when the environment drops that command, which makes policy-gradient credit noisier on those steps; the checkpoint loader uses `weights_only=False` and must only be given our own checkpoints.
- The artifact checks (weight hash, normalisation fingerprint, bounds) detect corruption and mixed-up directories. Both values live in the metadata file next to the weights, so they do not stop someone who replaces both files.
- Reproducibility was checked by retraining the T1 policy (identical hash, repeated by an independent reviewer). The T2 policy was not retrained to check.
- No tests accompany the new code. Checks were scripted runs: the fast training path and `run_episode` agree on integer outcomes (and to 1e-13 m on minimum separation), and the loader refuses a missing `metadata.json`, altered weights, a missing normalisation fingerprint, an unexpected `weights_file` and mismatched world bounds.
