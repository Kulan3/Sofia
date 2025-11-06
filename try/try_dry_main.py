#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dry-run planner for the beta mission.
This script walks through a beta_waypoint plan and prints the
commands that beta_main.py would send to the drone, along with
timing estimates. It never talks to hardware, so you can sanity
check plans on any machine.
"""
import argparse
from pathlib import Path
from typing import List, Tuple, Optional

import beta_config as C
from beta_plan import PLAN_DIR, find_latest_beta_waypoint_json, load_plan


def _format_seconds(seconds: Optional[float]) -> str:
    if seconds is None or seconds < 0:
        return "-"
    minutes = int(seconds // 60)
    secs = seconds - minutes * 60
    if minutes == 0:
        return f"{secs:.1f}s"
    if minutes < 60:
        return f"{minutes}m {secs:04.1f}s"
    hours = minutes // 60
    minutes = minutes % 60
    return f"{hours}h {minutes}m {secs:04.1f}s"


def _count_forward_moves(dist_cm: int) -> Tuple[int, int]:
    """
    Return (move_command_count, leftover_cm) mimicking beta_main's loop.
    """
    remaining = int(round(dist_cm))
    count = 0
    while remaining >= C.MIN_MOVE_CM:
        step = min(C.FORWARD_STEP_CM, remaining, C.MAX_MOVE_CM)
        remaining -= step
        count += 1
    return count, remaining


def summarise_segments(
    segs: List[Tuple[int, int]], speed_cm_s: int, pause_per_seg: float, min_turn_deg: int
) -> Tuple[float, float, List[str]]:
    total_cm = 0
    total_time = 0.0
    lines: List[str] = []
    for idx, (turn_deg, dist_cm) in enumerate(segs):
        skip_turn = abs(turn_deg) < min_turn_deg
        moves, leftover = _count_forward_moves(dist_cm)
        move_time = (dist_cm / speed_cm_s) if speed_cm_s > 0 else None
        pause = pause_per_seg
        # Base estimate: travel time + waits per move + optional pause + turn sleep
        wait_time = moves * C.MOVE_SLEEP
        seg_time = (move_time or 0.0) + wait_time + pause
        if turn_deg:
            seg_time += C.TURN_SLEEP
        total_cm += max(0, dist_cm)
        total_time += seg_time
        travel = _format_seconds(move_time)
        turn_label = f"{turn_deg:+4d} deg"
        if skip_turn and turn_deg != 0:
            turn_label += " (skip<min)"
        lines.append(
            f"[{idx:02}] turn {turn_label}, dist {dist_cm:4d} cm "
            f"-> moves {moves:2d}, leftover {leftover:2d} cm, travel {travel}"
        )
    return float(total_cm), total_time, lines


def dry_run(plan_path: Path) -> int:
    segs, meta = load_plan(plan_path)
    alt_cm = int(meta.get("height_cm", C.ALT_CM))
    speed_cm_s = int(meta.get("speed_cm_s", C.SPEED_CM_S))
    pause_per_seg = float(meta.get("pause_per_seg", C.PAUSE_PER_SEG))

    print(f"[*] Plan: {plan_path}")
    print(f"[*] Segments: {len(segs)}")
    print(
        f"[*] Config: altitude {alt_cm} cm, speed {speed_cm_s} cm/s, "
        f"pause/segment {pause_per_seg}s, move_sleep {C.MOVE_SLEEP}s"
    )

    min_turn = max(0, int(meta.get("min_turn_deg", getattr(C, "MIN_TURN_DEG", 0))))
    total_cm, total_time, lines = summarise_segments(segs, speed_cm_s, pause_per_seg, min_turn)
    for ln in lines:
        print("    " + ln)

    climb_cm = max(0, min(alt_cm - 20, C.MAX_MOVE_CM))
    climb_time = climb_cm / speed_cm_s if speed_cm_s > 0 else None
    est_total = total_time + (climb_time or 0.0)

    print(f"[*] Total forward distance: {total_cm:.0f} cm")
    print(f"[*] Approx climb: {climb_cm} cm, est { _format_seconds(climb_time) }")
    print(f"[*] Approx mission time (no AI events): { _format_seconds(est_total) }")
    print("    (Add extra buffer for AI interactions, retries, and safety.)")
    return 0


def resolve_plan_path(args: argparse.Namespace) -> Path:
    if args.use_last or args.json_path is None:
        last = find_latest_beta_waypoint_json()
        if not last:
            raise FileNotFoundError("No beta_waypoint*.json files found in plans/.")
        return last

    candidate = Path(args.json_path)
    if not candidate.is_absolute():
        candidate = PLAN_DIR / candidate
    if not candidate.exists():
        raise FileNotFoundError(f"Plan not found: {candidate}")
    return candidate


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Dry-run beta mission without connecting to the drone."
    )
    parser.add_argument(
        "json_path",
        nargs="?",
        default=None,
        help="Path to beta_waypoint JSON (relative paths resolve inside plans/).",
    )
    parser.add_argument(
        "--use-last",
        action="store_true",
        help="Ignore json_path and use the most recent beta_waypoint*.json in plans/.",
    )
    return parser.parse_args()


def main() -> int:
    try:
        args = parse_args()
        plan_path = resolve_plan_path(args)
        return dry_run(plan_path)
    except FileNotFoundError as e:
        print(f"[X] {e}")
        return 2
    except Exception as e:
        print(f"[X] Dry run failed: {e}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
