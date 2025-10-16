#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Shared plan utilities for the beta mission scripts.
Separated from beta_main.py so that dry-run tooling can reuse the
parsing logic without importing djitellopy (which requires hardware).
"""
from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Iterable, List, Sequence, Tuple

# Plans live next to the flight scripts
BASE_DIR = Path(__file__).resolve().parent
PLAN_DIR = BASE_DIR / "plans"
PLAN_DIR.mkdir(exist_ok=True)


def _vec(a: Sequence[float], b: Sequence[float]) -> Tuple[float, float]:
    return (b[0] - a[0], b[1] - a[1])


def _dot(u: Sequence[float], v: Sequence[float]) -> float:
    return u[0] * v[0] + u[1] * v[1]


def _cross2(u: Sequence[float], v: Sequence[float]) -> float:
    return u[0] * v[1] - u[1] * v[0]


def _norm(u: Sequence[float]) -> float:
    return math.hypot(u[0], u[1])


def _angle_unsigned(u: Sequence[float], v: Sequence[float]) -> float:
    mu, mv = _norm(u), _norm(v)
    if mu == 0 or mv == 0:
        return 0.0
    d = _dot(u, v) / (mu * mv)
    d = max(-1.0, min(1.0, d))
    return math.degrees(math.acos(d))


def _angle_signed(u: Sequence[float], v: Sequence[float]) -> float:
    a = _angle_unsigned(u, v)
    c = _cross2(u, v)
    if c > 0:
        return a
    if c < 0:
        return -a
    return 0.0


def _initial_heading_from_pos(pos0: Sequence[float], pos1: Sequence[float]) -> float:
    return _angle_signed((1.0, 0.0), _vec(pos0, pos1))


def load_plan(json_path: str | Path) -> Tuple[List[Tuple[int, int]], dict]:
    """
    Load beta_waypoint JSON and return [(turn_signed_deg, dist_cm), ...], meta dict.
    """
    path = Path(json_path)
    with path.open("r", encoding="utf-8") as f:
        data = json.load(f)

    wp = data.get("wp", [])
    pos = data.get("pos", None)
    meta = data.get("meta", {})
    if not wp:
        raise ValueError("JSON has no 'wp' array.")

    dists: List[int] = []
    for i, w in enumerate(wp):
        if "dist_cm" not in w:
            raise ValueError(f"dist_cm missing for segment {i}")
        dists.append(int(round(w["dist_cm"])))

    has_signed = any("turn_signed_deg" in w for w in wp)
    turns = [0] * len(dists)
    if has_signed:
        for i in range(len(dists)):
            turns[i] = int(round(wp[i].get("turn_signed_deg", 0)))
        if pos and len(pos) >= 2 and abs(turns[0]) < 1:
            turns[0] = int(round(_initial_heading_from_pos(pos[0], pos[1])))
    elif pos and len(pos) >= 2:
        turns[0] = int(round(_initial_heading_from_pos(pos[0], pos[1])))
        for i in range(1, len(dists)):
            if i < len(pos) - 1:
                turns[i] = int(
                    round(_angle_signed(_vec(pos[i - 1], pos[i]), _vec(pos[i], pos[i + 1])))
                )
            else:
                turns[i] = 0
    else:
        for i in range(len(dists)):
            turns[i] = int(round(wp[i].get("angle_deg", 0)))

    segs = [(turns[i], dists[i]) for i in range(len(dists))]
    return segs, meta


def find_latest_beta_waypoint_json() -> Path | None:
    """
    Find the most recently modified beta_waypoint JSON in the plans directory.
    """
    cands: List[Path] = []
    cands += list(PLAN_DIR.glob("beta_waypoint.json"))
    cands += list(PLAN_DIR.glob("beta_waypoint_*.json"))
    cands += list(PLAN_DIR.glob("beta_waypoint-*.json"))
    if not cands:
        return None
    return max(cands, key=lambda p: p.stat().st_mtime)


__all__ = ["PLAN_DIR", "load_plan", "find_latest_beta_waypoint_json"]
