#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import annotations

import argparse
import logging
import os
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Callable, Dict, Optional, Tuple

from djitellopy import Tello

import beta_config as C
from beta_detect import FireDetector, FireDetection
from beta_plan import PLAN_DIR, find_latest_beta_waypoint_json, load_plan

# --------------------------------------------------------------------------- #
# Logging helpers
# --------------------------------------------------------------------------- #

LOGGER: Optional[logging.Logger] = None


def init_logging(log_path: Optional[str]) -> None:
    global LOGGER
    LOGGER = logging.getLogger("flight")
    LOGGER.setLevel(logging.INFO)
    LOGGER.handlers.clear()

    console = logging.StreamHandler(sys.stdout)
    console.setLevel(logging.INFO)
    console.setFormatter(logging.Formatter("[%(asctime)s] %(message)s", datefmt="%H:%M:%S"))
    LOGGER.addHandler(console)

    handlers = [console]
    if log_path:
        os.makedirs(os.path.dirname(log_path) or ".", exist_ok=True)
        file_handler = logging.FileHandler(log_path, encoding="utf-8")
        file_handler.setLevel(logging.INFO)
        file_handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
        LOGGER.addHandler(file_handler)
        handlers.append(file_handler)
        LOGGER.info("Logging to %s", log_path)

    sdk_logger = logging.getLogger("djitellopy")
    sdk_logger.setLevel(logging.INFO)
    sdk_logger.handlers.clear()
    for handler in handlers:
        sdk_logger.addHandler(handler)
    sdk_logger.propagate = False


def log_i(msg: str) -> None:
    print("[*] {}".format(msg))
    if LOGGER:
        LOGGER.info(msg)


def log_w(msg: str) -> None:
    print("[!] {}".format(msg))
    if LOGGER:
        LOGGER.warning(msg)


def log_e(msg: str) -> None:
    print("[X] {}".format(msg))
    if LOGGER:
        LOGGER.error(msg)


# --------------------------------------------------------------------------- #
# Command wrappers and telemetry helpers
# --------------------------------------------------------------------------- #

def _is_imu_error(exc: Exception) -> bool:
    msg = str(exc).lower()
    return "no valid imu" in msg or "not joystick" in msg


def _is_timeout_error(exc: Exception) -> bool:
    msg = str(exc).lower()
    return "did not receive a response" in msg or "max retries exceeded" in msg


def _needs_command_recover(exc: Exception) -> bool:
    msg = str(exc).lower()
    if not msg:
        return False
    return (
        "unknown command" in msg
        or "invalid continuation byte" in msg
        or "utf-8 codec" in msg
        or "utf8" in msg
        or "did not receive a response" in msg
        or "aborting command" in msg
    )


def _state_height_cm(t: Tello) -> Optional[float]:
    try:
        data = t.get_current_state()
        if not data:
            return None
        height = data.get("h")
        if height is None:
            return None
        return float(height)
    except Exception:
        return None


def _capture_command_snapshot(t: Optional[Tello], label: str) -> Optional[Dict[str, Optional[float]]]:
    if not isinstance(t, Tello):
        return None
    if label in ("takeoff", "land", "move_up", "move_down"):
        return {"h": _state_height_cm(t)}
    return None


def _command_effect_seen(
    t: Optional[Tello],
    label: str,
    before: Optional[Dict[str, Optional[float]]],
    args,
) -> bool:
    if not isinstance(t, Tello) or before is None:
        return False
    tol = float(getattr(C, "COMMAND_SUCCESS_TOL_CM", 10.0))
    after = _capture_command_snapshot(t, label)
    if not after:
        return False
    if label == "takeoff":
        height = after.get("h")
        return height is not None and height >= float(getattr(C, "TAKEOFF_SUCCESS_HEIGHT_CM", 30.0))
    if label == "land":
        height = after.get("h")
        return height is not None and height <= tol
    if label in ("move_up", "move_down"):
        before_h = before.get("h")
        after_h = after.get("h")
        if before_h is None or after_h is None:
            return False
        target = float(args[0]) if args else 0.0
        delta = after_h - before_h
        if label == "move_up":
            return delta >= max(0.0, target - tol)
        return delta <= -max(0.0, target - tol)
    return False


def _command_pad_seconds(label: str, args) -> float:
    cfg = getattr(C, "COMMAND_TIMEOUT_PAD", {})
    entry = cfg.get(label)
    if entry is None:
        return 0.0
    if isinstance(entry, (int, float)):
        return float(entry)
    base = float(entry.get("base", 0.0) or 0.0)
    per_cm = float(entry.get("per_cm", 0.0) or 0.0)
    dist_cm = 0.0
    if args:
        try:
            dist_cm = float(args[0])
        except Exception:
            dist_cm = 0.0
    return max(0.0, base + per_cm * abs(dist_cm))


def ensure_command_mode(t: Tello, log_recover: bool = True) -> bool:
    try:
        resp = t.send_command_with_return("command")
        if isinstance(resp, str) and resp.lower() not in ("ok", "ack", "ready"):
            log_w("Re-entered command mode, response: {}".format(resp))
        if log_recover:
            log_w("Sent 'command' to re-sync SDK mode.")
        time.sleep(0.2)
        return True
    except Exception as exc:
        if log_recover:
            log_w("command-mode recovery failed: {}".format(exc))
        return False


def try_cmd(fn, *args, retries=C.RETRIES, sleep=C.RETRY_SLEEP, label: str = "cmd"):
    attempts = 0
    base_attempts = retries + 1
    extra_imu = max(0, getattr(C, "IMU_RECOVER_MAX", 0))
    max_attempts = base_attempts
    t_obj = getattr(fn, "__self__", None)

    while True:
        snapshot = _capture_command_snapshot(t_obj, label)
        timeout_override_applied = False
        original_timeout = None
        pad_seconds = _command_pad_seconds(label, args)
        if pad_seconds > 0 and isinstance(t_obj, Tello):
            try:
                original_timeout = t_obj.RESPONSE_TIMEOUT
                t_obj.RESPONSE_TIMEOUT = original_timeout + pad_seconds
                timeout_override_applied = True
            except Exception:
                timeout_override_applied = False

        try:
            return fn(*args)
        except Exception as exc:
            attempts += 1
            imu_err = _is_imu_error(exc)
            timeout_err = _is_timeout_error(exc)
            if timeout_override_applied and isinstance(t_obj, Tello):
                t_obj.RESPONSE_TIMEOUT = original_timeout
                timeout_override_applied = False
            if timeout_err and _command_effect_seen(t_obj, label, snapshot, args):
                log_w("{} timeout but state indicates completion; skipping retries.".format(label))
                return None
            if _needs_command_recover(exc):
                t_obj = getattr(fn, "__self__", None)
                if isinstance(t_obj, Tello) and ensure_command_mode(t_obj):
                    log_w("{} retrying after command-mode recovery ({}/{})".format(label, attempts, max_attempts - 1))
                    time.sleep(sleep)
                    if attempts < max_attempts:
                        continue
            if imu_err and max_attempts == base_attempts:
                max_attempts += extra_imu
            if attempts < max_attempts:
                wait = getattr(C, "IMU_RECOVER_SLEEP", sleep) if imu_err else sleep
                reason = " (IMU not ready)" if imu_err else ""
                log_w("{} failed ({}); retry {}/{}{}".format(label, exc, attempts, max_attempts - 1, reason))
                time.sleep(wait)
            else:
                log_e("{} failed after {} tries: {}".format(label, max_attempts, exc))
                raise
        finally:
            if timeout_override_applied and isinstance(t_obj, Tello):
                t_obj.RESPONSE_TIMEOUT = original_timeout


# --------------------------------------------------------------------------- #
# Flight helpers
# --------------------------------------------------------------------------- #

def _normalize_yaw(deg: float) -> float:
    while deg > 180.0:
        deg -= 360.0
    while deg <= -180.0:
        deg += 360.0
    return deg


def get_current_yaw(t: Tello) -> Optional[float]:
    try:
        data = t.get_current_state()
        if not data:
            return None
        yaw = data.get("yaw")
        if yaw is None:
            return None
        return float(yaw)
    except Exception as exc:
        log_w("get_current_state error: {}".format(exc))
        return None


def correct_heading_if_needed(t: Tello, expected_yaw: float) -> float:
    tol = float(getattr(C, "DRIFT_HEADING_TOL_DEG", 0))
    if tol <= 0:
        return expected_yaw
    actual = get_current_yaw(t)
    if actual is None:
        return expected_yaw
    diff = _normalize_yaw(actual - expected_yaw)
    if abs(diff) < tol:
        return expected_yaw
    max_c = float(getattr(C, "DRIFT_CORRECT_MAX_DEG", tol))
    correction = max(-max_c, min(max_c, diff))
    log_w("Heading drift: actual {:.1f}°, expected {:.1f}° -> correcting {:.1f}°".format(actual, expected_yaw, -correction))
    rotate_signed_deg(t, -correction)
    updated = get_current_yaw(t)
    return _normalize_yaw(updated) if updated is not None else _normalize_yaw(expected_yaw - correction)


def fallback_rc_descent(t: Tello) -> None:
    log_w("Attempting RC descent fallback.")
    try:
        t.send_rc_control(0, 0, -20, 0)
        time.sleep(2.5)
        t.send_rc_control(0, 0, 0, 0)
    except Exception as exc:
        log_e("RC descent fallback failed: {}; issuing emergency stop.".format(exc))
        try:
            t.emergency()
        except Exception as ee:
            log_e("Emergency command failed: {}".format(ee))


def safe_land(t: Tello) -> None:
    try:
        try_cmd(t.land, label="land")
        return
    except Exception as exc:
        log_w("land error: {}".format(exc))
    if not ensure_command_mode(t, log_recover=True):
        log_w("Unable to re-enter command mode before RC descent.")
    fallback_rc_descent(t)


def rotate_signed_deg(t: Tello, ang_deg: float) -> None:
    a = int(round(ang_deg))
    if a == 0:
        return
    min_turn = max(0, getattr(C, "MIN_TURN_DEG", 0))
    if abs(a) < min_turn:
        log_w("turn {:+d} deg below MIN_TURN_DEG ({}); skipping.".format(a, min_turn))
        return

    chunk_cfg = getattr(C, "TURN_CHUNK_DEG", 0)
    step_limit = abs(int(chunk_cfg)) if chunk_cfg else 0
    remaining = abs(a)
    fn = t.rotate_counter_clockwise if a > 0 else t.rotate_clockwise
    label = "rotate_ccw" if a > 0 else "rotate_cw"

    try:
        t.send_rc_control(0, 0, 0, 0)
    except Exception:
        pass
    time.sleep(C.TURN_SLEEP)

    chunk_idx = 0
    while remaining > 0:
        step = remaining if step_limit <= 0 else min(remaining, step_limit)
        chunk_idx += 1
        if step_limit > 0 and remaining > step:
            direction = "CCW" if a > 0 else "CW"
            log_i("    turn chunk {}: {} {} deg (rem {})".format(chunk_idx, direction, step, remaining - step))
        try:
            try_cmd(fn, step, label=label)
        except Exception as exc:
            if _is_imu_error(exc):
                fallback_deg = step if a > 0 else -step
                if rc_yaw_fallback(t, fallback_deg):
                    log_w("{} chunk used RC fallback for {:+d} deg.".format(label, fallback_deg))
                else:
                    log_w("{} chunk aborted due to IMU error; skipping remaining turn.".format(label))
                    return
            else:
                raise
        remaining -= step
        time.sleep(C.TURN_SLEEP)


def rc_yaw_fallback(t: Tello, step_deg: int) -> bool:
    speed = abs(int(getattr(C, "RC_YAW_SPEED", 0)))
    rate = float(getattr(C, "RC_YAW_DEG_PER_SEC", 0) or 0)
    if speed <= 0 or rate <= 0 or step_deg == 0:
        return False
    yaw_cmd = speed if step_deg > 0 else -speed
    duration = max(0.1, abs(step_deg) / rate)
    log_w("    rc yaw fallback: rc 0 0 0 {} for {:.2f}s".format(yaw_cmd, duration))
    success = False
    try:
        t.send_rc_control(0, 0, 0, yaw_cmd)
        time.sleep(duration)
        success = True
    except Exception as exc:
        log_e("RC yaw fallback failed: {}".format(exc))
    finally:
        try:
            t.send_rc_control(0, 0, 0, 0)
        except Exception:
            pass
        time.sleep(getattr(C, "RC_YAW_RECOVER_PAUSE", 0.2))
    return success


def approach_once(t: Tello, det: FireDetection) -> float:
    if det is None or not det.has_fire:
        return 0.0
    yaw_applied = 0.0
    if abs(det.dx) > C.CENTER_TOL_PX:
        step = int(C.APPROACH_YAW_STEP)
        yaw = -step if det.dx > 0 else step
        if yaw > 0:
            t.rotate_counter_clockwise(yaw)
        else:
            t.rotate_clockwise(-yaw)
        time.sleep(C.TURN_SLEEP)
        yaw_applied += yaw
        return yaw_applied
    if det.area_frac < C.NEAR_AREA_FRAC:
        t.move_forward(int(C.APPROACH_FWD_CM))
        time.sleep(C.MOVE_SLEEP)
    return yaw_applied


# --------------------------------------------------------------------------- #
# Detection policies
# --------------------------------------------------------------------------- #

def do_policy_v1_hold_until_lost(
    t: Tello,
    detector: FireDetector,
    frame_supplier: Callable[[], Optional[object]],
    initial_det: Optional[FireDetection] = None,
) -> None:
    log_i("V1: FIRE detected -> approaching; resume after target is lost.")
    t.send_rc_control(0, 0, 0, 0)
    t.hover()
    time.sleep(0.1)
    last_seen = time.time()
    current = initial_det if (initial_det and initial_det.has_fire) else None
    if current:
        last_seen = current.ts
    while True:
        if current is None:
            current = detector.infer(frame_supplier())
        if current.has_fire:
            last_seen = time.time()
            try:
                approach_once(t, current)
            except Exception as exc:
                log_w("approach_once error: {}".format(exc))
        else:
            if (time.time() - last_seen) * 1000.0 > C.FIRE_LOST_MS:
                log_i("V1: Fire lost -> resuming mission.")
                break
        time.sleep(0.05)
        current = None


def do_policy_v2_approach_then_hold(
    t: Tello,
    detector: FireDetector,
    frame_supplier: Callable[[], Optional[object]],
    initial_det: Optional[FireDetection] = None,
) -> None:
    log_i("V2: FIRE detected -> approach then dwell {:.1f}s.".format(C.HOLD_SECS))
    t.send_rc_control(0, 0, 0, 0)
    t.hover()
    time.sleep(0.1)
    last_seen = None
    steps = 0
    current = initial_det if (initial_det and initial_det.has_fire) else None
    if current:
        last_seen = current.ts
    while True:
        if current is None:
            current = detector.infer(frame_supplier())
        if current.has_fire:
            last_seen = time.time()
            if steps < C.APPROACH_MAX_STEPS:
                try:
                    approach_once(t, current)
                    steps += 1
                except Exception as exc:
                    log_w("approach_once error: {}".format(exc))
        else:
            if last_seen is None:
                log_i("V2: Fire not re-detected; resuming.")
                break
        if last_seen is not None and (time.time() - last_seen) >= C.HOLD_SECS:
            log_i("V2: Hold complete -> resuming mission.")
            break
        time.sleep(0.05)
        current = None


# --------------------------------------------------------------------------- #
# Main mission flow
# --------------------------------------------------------------------------- #

def main(json_path: str, mode_version: int, ai_enabled: bool, show_video: bool) -> int:
    segs, meta = load_plan(json_path)
    alt_cm = int(meta.get("height_cm", C.ALT_CM))
    speed_cm_s = int(meta.get("speed_cm_s", C.SPEED_CM_S))
    log_i("Altitude {} cm, speed {} cm/s, AI={} policy={}".format(alt_cm, speed_cm_s, "ON" if ai_enabled else "OFF", mode_version if ai_enabled else "-"))

    t = Tello()
    try:
        timeout_override = float(getattr(C, "RESPONSE_TIMEOUT_S", t.RESPONSE_TIMEOUT))
        if timeout_override > 0:
            t.RESPONSE_TIMEOUT = timeout_override
    except Exception:
        pass
    try:
        retry_override = int(getattr(C, "COMMAND_RETRY_COUNT", t.retry_count))
        if retry_override > 0:
            t.retry_count = retry_override
    except Exception:
        pass

    ok = False
    for attempt in range(1, C.CONNECT_RETRIES + 1):
        try:
            log_i("Connecting (attempt {}/{})...".format(attempt, C.CONNECT_RETRIES))
            t.connect()
            battery = t.get_battery()
            log_i("Battery {}%".format(battery))
            ok = True
            break
        except Exception as exc:
            log_w("connect error: {}".format(exc))
            time.sleep(C.CONNECT_BACKOFF)
    if not ok:
        log_e("Unable to connect. Check TELLO Wi-Fi / power / close other Tello apps.")
        return 3

    expected_yaw = get_current_yaw(t)
    expected_yaw = _normalize_yaw(expected_yaw) if expected_yaw is not None else 0.0

    need_detector = ai_enabled or show_video
    frame_supplier: Optional[Callable[[], Optional[object]]] = None
    frame_read = None
    stream_active = False

    if need_detector:
        try:
            t.streamon()
            stream_active = True
            frame_read = t.get_frame_read(with_queue=False, max_queue_len=0)

            def _supply_frame(fr=frame_read):
                return fr.frame

            frame_supplier = _supply_frame
        except Exception as exc:
            log_w("streamon error - disabling AI/show-video: {}".format(exc))
            need_detector = False
            ai_enabled = False
            show_video = False

    try:
        t.set_speed(speed_cm_s)
    except Exception as exc:
        log_w("set_speed error: {}".format(exc))

    log_i("Takeoff.")
    try:
        try_cmd(t.takeoff, label="takeoff")
    except Exception:
        t.end()
        return 5
    time.sleep(0.5)

    stabilize = max(0.0, getattr(C, "IMU_STABILIZE_SECS", 0.0))
    if stabilize > 0:
        log_i("Hovering for IMU stabilization ({:.1f}s).".format(stabilize))
        time.sleep(stabilize)

    climb = max(0, min(alt_cm - 20, C.MAX_MOVE_CM))
    if climb > 0:
        climb_step = max(0, getattr(C, "CLIMB_CHUNK_CM", 0))
        remaining = climb
        chunk_idx = 0
        while remaining > 0:
            step = remaining if climb_step <= 0 else min(remaining, climb_step)
            chunk_idx += 1
            if climb_step > 0 and remaining > step:
                log_i("    climb chunk {}: {} cm (rem {})".format(chunk_idx, step, remaining - step))
            try:
                try_cmd(t.move_up, step, label="move_up")
            except Exception as exc:
                log_w("move_up({}) failed; continuing without additional climb: {}".format(step, exc))
                break
            remaining -= step
            time.sleep(C.MOVE_SLEEP)

    detector: Optional[FireDetector] = None
    if need_detector and frame_supplier is not None:
        detector = FireDetector(enable_model=ai_enabled, show_video=show_video)
        if ai_enabled:
            detector.start_async(frame_supplier)
    elif need_detector:
        log_w("Stream active but frame source unavailable; live preview disabled.")
        show_video = False
        ai_enabled = False
        detector = None

    aborted = False
    try:
        for idx, (turn_deg, dist_cm) in enumerate(segs):
            try:
                battery = t.get_battery()
                if battery is not None and battery <= C.LOW_BATT_RTH:
                    log_e("Low battery {}% - stop mission".format(battery))
                    aborted = True
                    break
            except Exception as exc:
                log_w("get_battery error: {}".format(exc))

            if turn_deg:
                log_i("[{}] turn {:+d} deg".format(idx, turn_deg))
                rotate_signed_deg(t, turn_deg)
                expected_yaw = _normalize_yaw(expected_yaw + turn_deg)
                yaw_after_turn = get_current_yaw(t)
                if yaw_after_turn is not None:
                    expected_yaw = _normalize_yaw(yaw_after_turn)

            remaining = int(round(dist_cm))
            log_i("[{}] forward total {} cm".format(idx, remaining))
            while remaining >= C.MIN_MOVE_CM and not aborted:
                step = min(C.FORWARD_STEP_CM, remaining, C.MAX_MOVE_CM)
                try:
                    try_cmd(t.move_forward, step, label="move_forward")
                except Exception as exc:
                    log_w("move_forward({}) failed; skipping remaining distance: {}".format(step, exc))
                    break
                remaining -= step
                time.sleep(C.MOVE_SLEEP)
                expected_yaw = correct_heading_if_needed(t, expected_yaw)

                if detector:
                    if ai_enabled:
                        det_snapshot = detector.get_latest_detection(max_age=0.7)
                        if det_snapshot and det_snapshot.has_fire:
                            detector.pause_async()
                            try:
                                if mode_version == 1:
                                    do_policy_v1_hold_until_lost(t, detector, frame_supplier, det_snapshot)
                                elif mode_version == 2:
                                    do_policy_v2_approach_then_hold(t, detector, frame_supplier, det_snapshot)
                            finally:
                                detector.resume_async()
                            yaw_after_policy = get_current_yaw(t)
                            if yaw_after_policy is not None:
                                expected_yaw = _normalize_yaw(yaw_after_policy)
                    elif show_video and frame_supplier is not None:
                        frame = frame_supplier()
                        if frame is not None:
                            detector.infer(frame)

            if C.PAUSE_PER_SEG > 0:
                time.sleep(C.PAUSE_PER_SEG)

        if not aborted:
            log_i("Mission complete. Landing.")
    except KeyboardInterrupt:
        log_e("KeyboardInterrupt - landing")
    except Exception as exc:
        log_e("Runtime error: {} - landing".format(exc))
    finally:
        safe_land(t)
        if detector:
            detector.close()
        if stream_active:
            try:
                t.streamoff()
            except Exception:
                pass
        t.end()
        log_i("Done.")
    return 0


# --------------------------------------------------------------------------- #
# CLI entrypoint
# --------------------------------------------------------------------------- #

def _resolve_plan_path(args) -> Path:
    if args.use_last or args.json_path is None:
        last = find_latest_beta_waypoint_json()
        if last:
            print("[*] Using latest plan: {}".format(last))
            return last
        print("[X] No beta_waypoint*.json found in plans/ and no path provided.")
        raise SystemExit(2)
    candidate = Path(args.json_path)
    if not candidate.is_absolute():
        candidate = PLAN_DIR / candidate
    if not candidate.exists():
        print("[X] JSON not found: {}".format(candidate))
        raise SystemExit(2)
    return candidate


def cli() -> int:
    parser = argparse.ArgumentParser(description="Fly Tello plan with optional fire detection")
    parser.add_argument(
        "json_path",
        nargs="?",
        default=None,
        help="Path to beta_waypoint JSON (relative names are looked up in plans/).",
    )
    parser.add_argument(
        "--mode",
        choices=["1", "2"],
        default=None,
        help="AI policy: 1=resume when lost, 2=approach+hold. Ignored if AI is off.",
    )
    parser.add_argument(
        "--ai",
        choices=["auto", "on", "off"],
        default="auto",
        help="Force AI on/off, or use config (default: auto).",
    )
    parser.add_argument(
        "--log",
        dest="log_path",
        default=None,
        help="Optional log file path. Use \"\" to auto-generate in logs/.",
    )
    parser.add_argument(
        "--use-last",
        action="store_true",
        help="Ignore json_path and use the newest beta_waypoint file in plans/.",
    )
    parser.add_argument(
        "--show-video",
        action="store_true",
        help="Force live preview window even if AI is off.",
    )

    args = parser.parse_args()
    plan_path = _resolve_plan_path(args)

    log_path = args.log_path
    if log_path == "":
        log_path = "logs/flight_{}.log".format(datetime.now().strftime("%Y%m%d_%H%M%S"))
    init_logging(log_path)

    if args.ai == "auto":
        ai_enabled = bool(C.ENABLE_AI)
    else:
        ai_enabled = args.ai == "on"

    policy = int(C.VERSION)
    if args.mode is not None:
        policy = int(args.mode)
    policy_internal = policy if ai_enabled else 0

    return main(str(plan_path), policy_internal, ai_enabled, args.show_video)


if __name__ == "__main__":
    raise SystemExit(cli())
