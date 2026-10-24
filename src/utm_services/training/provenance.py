"""Where this code came from (torch-free, so evaluation and replay can record it)."""

from __future__ import annotations

import subprocess
from pathlib import Path
from typing import Any


def git_state() -> dict[str, Any]:
    try:
        root = Path(__file__).resolve().parents[3]
        sha = subprocess.run(["git", "rev-parse", "HEAD"], cwd=root, capture_output=True, text=True, check=True).stdout.strip()
        dirty = bool(subprocess.run(["git", "status", "--porcelain", "--", "src", "shared"], cwd=root,
                                    capture_output=True, text=True, check=True).stdout.strip())
        return {"commit": sha, "src_dirty": dirty}
    except Exception:  # noqa: BLE001 - metadata only
        return {"commit": None, "src_dirty": None}
