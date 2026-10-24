# Project instructions

## Decision authority
Decide yourself, log it in docs/DECISIONS.md, and continue:
- implementation details, refactors, test structure, tooling, performance work that reproduces the per-seed results file exactly
- choices that follow an existing rule in this file or the spec

Stop and ask the founder only for:
- changing a safety claim, threshold, acceptance bar or the spec
- contract version changes
- any result that gets worse on a safety metric
- merging, force-pushing, deleting
- architecture decisions (ADRs)

Every safety PR gets an automatic adversarial review by a fresh subagent before it is reported as ready.

## Verification
Verification is change-scoped: run the quick tier and targeted probes while iterating; run the full tier once per PR before merge; reviewers never rerun a tier that already ran on the same commit.
- quick tier: `python scripts/verify/tier.py quick`; full tier: `python scripts/verify/tier.py full`; `invariance.py` only on demand: `python scripts/verify/tier.py ondemand` (see `scripts/verify/README.md`)
- `pytest` runs the fast suite in parallel; `pytest -m slow` runs the slow suite (CI runs it on PRs to main)
