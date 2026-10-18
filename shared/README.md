# shared/ — Shared Types & Contracts

Files here are used by all three workstreams. **No one person owns this directory** — any change requires a PR with all three owners as reviewers.

---

## What goes here

| File | What it is | Who creates it |
|------|------------|----------------|
| `schemas.py` | `DroneState`, `MARLAction`, `SafeAction` dataclasses | Salman + Anas (ticket A-03 / S-04) |

---

## Rules

1. No business logic here — only data types.
2. No imports from any workstream directory (salman/, anas/, aman/).
3. All types must be JSON-serializable.
4. Any field rename or type change = PR + all three owners approve + `INTERFACES.md` version bumped.

---

## Type definitions (to be implemented in schemas.py)

See `INTERFACES.md` for the complete field-by-field spec for each type:
- `DroneState` → § STATE-SCHEMA
- `MARLAction` → § MARL-ACTION  
- `SafeAction` → § CBF-OUTPUT
