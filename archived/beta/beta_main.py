#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Waypoint runner with target engagement behaviour."""
from __future__ import annotations

import argparse
import logging
import math
import os
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple

from djitellopy import Tello

import beta_config as C
from beta_detect import FireDetector, FireDetection
from beta_plan import PLAN_DIR, find_latest_beta_waypoint_json, load_plan

LOGGER: Optional[logging.Logger] = None
AI_LOGGER: Optional[logging.Logger] = None


def init_logging(log_path: Optional[str]) -> None:
    """Configure mission and AI loggers."""
    global LOGGER, AI_LOGGER

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

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    if log_path:
        base_dir = os.path.dirname(log_path) or "."
        base_name = os.path.basename(log_path)
        if base_name.startswith("flight_"):
            ai_name = base_name.replace("flight_", "flight_ai_", 1)
        else:
            root, ext = os.path.splitext(base_name)
            ai_name = f"{root}_ai{ext or '.log'}"
        ai_path = os.path.join(base_dir, ai_name)
    else:
        os.makedirs("logs", exist_ok=True)
        ai_path = os.path.join("logs", f"flight_ai_{timestamp}.log")

    AI_LOGGER = logging.getLogger("flight.ai")
    AI_LOGGER.setLevel(logging.INFO)
    AI_LOGGER.handlers.clear()
    ai_handler = logging.FileHandler(ai_path, encoding="utf-8")
    ai_handler.setFormatter(logging.Formatter("%(asctime)s %(message)s"))
    AI_LOGGER.addHandler(ai_handler)
    AI_LOGGER.propagate = False


def log_i(msg: str) -> None:
    print(f"[*] {msg}")
    if LOGGER:
        LOGGER.info(msg)


def log_w(msg: str) -> None:
    print(f"[!] {msg}")
    if LOGGER:
        LOGGER.warning(msg)


def log_e(msg: str) -> None:
    print(f"[X] {msg}")
    if LOGGER:
        LOGGER.error(msg)


def log_ai(msg: str) -> None:
    if AI_LOGGER:
        AI_LOGGER.info(msg)


def set_detector_status(detector: Optional[FireDetector], message: str) -> None:
    """Update detector overlay/status text if available."""
    if detector and hasattr(detector, "set_status"):
        try:
            detector.set_status(message)
        except Exception:
            pass


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
        state = t.get_current_state()
        if not state:
            return None
        h = state.get("h")
        if h is None:
            return None
        return float(h)
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


def _normalize_yaw(deg: float) -> float:
    while deg > 180.0:
        deg -= 360.0
    while deg <= -180.0:
        deg += 360.0
    return deg


def get_current_yaw(t: Tello) -> Optional[float]:
    try:
        state = t.get_current_state()
        if not state:
            return None
        yaw = state.get("yaw")
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
    log_w("Heading drift: actual {:.1f}deg, expected {:.1f}deg -> correcting {:.1f}deg".format(actual, expected_yaw, -correction))
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


def _estimate_distance_m(detection: FireDetection, spec: Dict[str, Any]) -> Optional[float]:
    if detection.bbox is None:
        return None
    real_width = float(spec.get("real_width_m", 0.0) or 0.0)
    if real_width <= 0:
        return None
    x1, _, x2, _ = detection.bbox
    box_width_px = float(max(1, x2 - x1))
    focal_px = (C.FRAME_W / 2.0) / math.tan(math.radians(C.H_FOV_DEG / 2.0))
    return (real_width * focal_px) / box_width_px

def _pixels_to_yaw_deg(dx: float) -> float:
    if not C.FRAME_W or not C.H_FOV_DEG:
        return 0.0
    half_width = C.FRAME_W / 2.0
    if half_width <= 0:
        return 0.0
    return (dx / half_width) * (C.H_FOV_DEG / 2.0)


def _pixels_to_vertical_cm(dy: float, distance_m: Optional[float]) -> int:
    if abs(dy) < 1e-3:
        return 0
    if C.FRAME_H and C.V_FOV_DEG and distance_m and distance_m > 0:
        half_height = C.FRAME_H / 2.0
        if half_height > 0:
            angle_deg = (dy / half_height) * (C.V_FOV_DEG / 2.0)
            delta_m = math.tan(math.radians(angle_deg)) * distance_m
            return int(round(delta_m * 100.0))
    scale = float(C.VERTICAL_STEP_CM) / float(max(1, C.VERTICAL_TOL_PX))
    return int(round(dy * scale))


def _pixels_to_strafe_cm(dx: float, distance_m: Optional[float]) -> int:
    if abs(dx) < 1e-3:
        return 0
    if C.FRAME_W and C.H_FOV_DEG and distance_m and distance_m > 0:
        half_width = C.FRAME_W / 2.0
        if half_width > 0:
            angle_deg = (dx / half_width) * (C.H_FOV_DEG / 2.0)
            lateral_m = math.tan(math.radians(angle_deg)) * distance_m
            return int(round(lateral_m * 100.0))
    if not C.FRAME_W:
        return 0
    half_width = C.FRAME_W / 2.0
    if half_width <= 0:
        return 0
    ratio = dx / half_width
    baseline = max(C.MIN_MOVE_CM, int(getattr(C, "APPROACH_STRAFE_CM", C.MIN_MOVE_CM)))
    return int(round(ratio * baseline))






def engage_target(
    t: Tello,
    detector: FireDetector,
    frame_supplier: Callable[[], Optional[Any]],
    initial_det: FireDetection,
) -> None:
    if frame_supplier is None:
        log_ai("No frame supplier; skipping engagement")
        return

    label = (initial_det.label or "fire").lower()
    spec = C.TARGET_SPECS.get(label, C.TARGET_SPECS.get("fire"))
    if not spec:
        log_ai("No spec for target '{}' ; skipping engagement".format(label))
        return

    desired_distance_m = float(spec.get("approach_distance_m", 0.3) or 0.3)
    distance_tol_m = C.APPROACH_DISTANCE_TOL_CM / 100.0
    log_ai("Engaging target '{}' (conf={:.2f})".format(label, initial_det.conf))

    def pump_preview() -> None:
        if detector is None or not getattr(detector, "show_video", False) or frame_supplier is None:
            return
        try:
            frame = frame_supplier()
        except Exception:
            frame = None
        if frame is not None:
            try:
                detector.infer(frame)
            except Exception:
                pass

    def fetch_detection(max_wait_s: float = 0.6, require_target: bool = True) -> Optional[FireDetection]:
        deadline = time.time() + max(0.0, max_wait_s)
        det_local: Optional[FireDetection] = None
        while True:
            pump_preview()
            det_local = detector.get_latest_detection(max_age=0.5)
            if det_local and det_local.has_fire and (det_local.label == label or det_local.label is None):
                return det_local
            if not require_target:
                return det_local
            if time.time() >= deadline:
                return None
            time.sleep(0.05)

    initial_yaw = get_current_yaw(t)
    total_forward_cm = 0

    plan_det = initial_det if initial_det.has_fire and (initial_det.label == label or initial_det.label is None) else None
    if plan_det is None or plan_det.bbox is None:
        plan_det = fetch_detection(max_wait_s=1.0, require_target=True)
    if plan_det is None or not plan_det.has_fire or plan_det.bbox is None:
        log_ai("Unable to confirm target for planning; aborting engagement")
        return

    def yaw_to_center(det: FireDetection) -> None:
        dx = det.dx
        yaw_step = int(round(_pixels_to_yaw_deg(dx)))
        if abs(yaw_step) >= max(1, int(getattr(C, "MIN_TURN_DEG", 1))):
            log_ai("Yaw to center target by {:+d} deg".format(yaw_step))
            set_detector_status(detector, "Yaw {:+d} deg".format(yaw_step))
            if yaw_step > 0:
                try_cmd(t.rotate_clockwise, yaw_step, label="rotate_cw")
            else:
                try_cmd(t.rotate_counter_clockwise, -yaw_step, label="rotate_ccw")
            time.sleep(C.TURN_SLEEP)

    yaw_to_center(plan_det)

    distance_tol_cm = int(round(distance_tol_m * 100.0))
    approach_success = False
    target_visible_on_finish = False
    start_time = time.time()
    lost_grace = float(getattr(C, "APPROACH_LOST_MS", getattr(C, "FIRE_LOST_MS", 400))) / 1000.0
    last_seen = time.time()
    last_det = plan_det
    remaining_forward_cm = 0
    initial_distance = _estimate_distance_m(plan_det, spec)
    if initial_distance is not None:
        remaining_forward_cm = max(0, int(round(max(0.0, initial_distance - desired_distance_m) * 100.0)))

    while time.time() - start_time < float(getattr(C, "APPROACH_TIMEOUT_S", 30)):
        det = fetch_detection(max_wait_s=0.5, require_target=False)
        now = time.time()
        if det and det.has_fire and (det.label == label or det.label is None):
            last_det = det
            last_seen = now
            yaw_to_center(det)
            distance_m = _estimate_distance_m(det, spec)
            if distance_m is None:
                continue
            delta_m = distance_m - desired_distance_m
            remaining_forward_cm = max(0, int(round(max(0.0, delta_m) * 100.0)))
            if abs(delta_m) <= distance_tol_m:
                log_ai("Reached stand-off distance at {:.2f} m".format(distance_m))
                approach_success = True
                target_visible_on_finish = True
                break
            step_cm = max(C.MIN_MOVE_CM, min(C.MAX_MOVE_CM, remaining_forward_cm))
            step_limit = int(getattr(C, "FORWARD_APPROACH_STEP_CM", 30))
            if step_limit > 0:
                step_cm = max(C.MIN_MOVE_CM, min(step_cm, step_limit))
            log_ai("Forward {} cm towards target (distance {:.2f} m)".format(step_cm, distance_m))
            set_detector_status(detector, "Forward {} cm".format(step_cm))
            try_cmd(t.move_forward, step_cm, label="move_forward")
            total_forward_cm += step_cm
            time.sleep(C.MOVE_SLEEP)
            continue
        else:
            if now - last_seen <= lost_grace:
                pump_preview()
                continue
            if remaining_forward_cm > distance_tol_cm:
                step_limit = int(getattr(C, "FORWARD_APPROACH_STEP_CM", 30))
                step_cm = max(C.MIN_MOVE_CM, remaining_forward_cm if step_limit <= 0 else min(remaining_forward_cm, step_limit))
                log_ai("Blind forward {} cm (resume toward standoff)".format(step_cm))
                set_detector_status(detector, "Forward {} cm (blind)".format(step_cm))
                try_cmd(t.move_forward, step_cm, label="move_forward")
                total_forward_cm += step_cm
                remaining_forward_cm = max(0, remaining_forward_cm - step_cm)
                time.sleep(C.MOVE_SLEEP)
                if remaining_forward_cm <= distance_tol_cm:
                    approach_success = True
                    target_visible_on_finish = False
                    break
                continue
            log_ai("Target lost during approach")
            break
        pump_preview()

    final_visible = target_visible_on_finish or (last_det is not None and (time.time() - last_seen) <= lost_grace)
    if approach_success and final_visible:
        log_ai("Holding on target until lost")
        set_detector_status(detector, "Holding target")
        hold_grace = float(getattr(C, "FIRE_LOST_MS", 400)) / 1000.0
        last_visible = last_seen
        last_keepalive = time.time()
        while True:
            pump_preview()
            time.sleep(0.2)
            now = time.time()
            if now - last_keepalive > 8.0:
                try:
                    t.send_command_without_return("command")
                except Exception:
                    pass
                last_keepalive = now
            det = fetch_detection(max_wait_s=0.2, require_target=False)
            if det and det.has_fire and (det.label == label or det.label is None):
                last_visible = now
                continue
            if now - last_visible > hold_grace:
                log_ai("Target lost; exiting hold")
                break
    elif approach_success:
        log_ai("Standoff reached but target not visible; skipping hold")
    else:
        log_ai("Unable to reach desired distance; skipping hold")

    if total_forward_cm > 0:
        log_ai("Moving back {} cm to resume route".format(total_forward_cm))
        set_detector_status(detector, "Retreat {} cm".format(total_forward_cm))
        try_cmd(t.move_back, total_forward_cm, label="move_back")
        time.sleep(C.MOVE_SLEEP)

    if initial_yaw is not None:
        current_yaw = get_current_yaw(t)
        if current_yaw is not None:
            yaw_error = _normalize_yaw(current_yaw - initial_yaw)
            correction = _normalize_yaw(-yaw_error)
            if abs(correction) >= max(1, int(getattr(C, "MIN_TURN_DEG", 1))):
                log_ai("Restoring heading by {:+.1f} deg".format(correction))
                set_detector_status(detector, "Restore heading")
                rotate_signed_deg(t, correction)

    set_detector_status(detector, "Returning to route")

def main(json_path: str, show_video: bool) -> int:
    segs, meta = load_plan(json_path)
    alt_cm = int(meta.get("height_cm", C.ALT_CM))
    speed_cm_s = int(meta.get("speed_cm_s", C.SPEED_CM_S))
    log_i("Altitude {} cm, speed {} cm/s".format(alt_cm, speed_cm_s))

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

    frame_supplier: Optional[Callable[[], Optional[Any]]] = None
    detector: Optional[FireDetector] = None
    stream_active = False

    try:
        t.streamon()
        stream_active = True
        frame_read = t.get_frame_read(with_queue=False, max_queue_len=0)

        def _supply_frame(fr=frame_read):
            return fr.frame

        frame_supplier = _supply_frame
        detector = FireDetector(enable_model=True, show_video=show_video)
        detector.start_async(frame_supplier)
    except Exception as exc:
        log_w("streamon error - disabling detection/preview: {}".format(exc))
        frame_supplier = None
        detector = None

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
        idx = 0
        while remaining > 0:
            step = remaining if climb_step <= 0 else min(remaining, climb_step)
            idx += 1
            if climb_step > 0 and remaining > step:
                log_i("    climb chunk {}: {} cm (rem {})".format(idx, step, remaining - step))
            try:
                try_cmd(t.move_up, step, label="move_up")
            except Exception as exc:
                log_w("move_up({}) failed; continuing without additional climb: {}".format(step, exc))
                break
            remaining -= step
            time.sleep(C.MOVE_SLEEP)

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
                yaw_after = get_current_yaw(t)
                if yaw_after is not None:
                    expected_yaw = _normalize_yaw(yaw_after)

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
                    det_snapshot = detector.get_latest_detection(max_age=0.3)
                    if det_snapshot and det_snapshot.has_fire:
                        label_detected = (det_snapshot.label or "fire").lower()
                        if label_detected not in C.TARGET_SPECS and "fire" not in C.TARGET_SPECS:
                            continue
                        log_i("Target '{}' detected; engaging".format(label_detected))
                        log_ai("Detected '{}' conf={:.2f}".format(label_detected, det_snapshot.conf))
                        engage_target(t, detector, frame_supplier, det_snapshot)
                        yaw_after = get_current_yaw(t)
                        if yaw_after is not None:
                            expected_yaw = _normalize_yaw(yaw_after)

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


def _resolve_plan_path(args) -> Path:
    if args.use_last or args.json_path is None:
        last = find_latest_beta_waypoint_json()
        if last:
            print(f"[*] Using latest plan: {last}")
            return last
        print("[X] No beta_waypoint*.json found in plans/ and no path provided.")
        raise SystemExit(2)
    candidate = Path(args.json_path)
    if not candidate.is_absolute():
        candidate = PLAN_DIR / candidate
    if not candidate.exists():
        print(f"[X] JSON not found: {candidate}")
        raise SystemExit(2)
    return candidate


def cli() -> int:
    parser = argparse.ArgumentParser(description="Fly plan with target engagement")
    parser.add_argument(
        "json_path",
        nargs="?",
        default=None,
        help="Path to beta_waypoint JSON (relative names are looked up in plans/).",
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
        help="Show live preview window.",
    )

    args = parser.parse_args()
    plan_path = _resolve_plan_path(args)

    log_path = args.log_path
    if log_path == "":
        os.makedirs("logs", exist_ok=True)
        log_path = os.path.join("logs", f"flight_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log")
    init_logging(log_path)

    return main(str(plan_path), args.show_video)


if __name__ == "__main__":
    raise SystemExit(cli())
