from __future__ import annotations

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time, sys, argparse, os, logging
from datetime import datetime
from pathlib import Path
from djitellopy import Tello

import beta_config as C
from beta_detect import FireDetector, approach_once, FireDetection
from beta_plan import PLAN_DIR, load_plan, find_latest_beta_waypoint_json

def _classes_text(self):
    c = getattr(C, "DETECT_CLASSES", None)
    if c is None:
        return ""
    if isinstance(c, (list, tuple, set)):
        return ",".join(map(str, c))
    return str(c)

def _annotate_and_show(self, frame_bgr, det: FireDetection):
    # Crosshair
    cx, cy = int(C.FRAME_W/2), int(C.FRAME_H/2)
    cv2.drawMarker(frame_bgr, (cx, cy), (0, 255, 255), markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)

    # Box + label
    if det.bbox is not None:
        x1, y1, x2, y2 = map(int, det.bbox)
        cv2.rectangle(frame_bgr, (x1, y1), (x2, y2), (0,0,255), 2)
        cls_text = self._classes_text()
        label = f"{cls_text or 'fire'} {det.conf:.2f} | dx={det.dx:.0f} dy={det.dy:.0f} area={det.area_frac:.2f}"
    else:
        label = f"no {self._classes_text() or 'fire'}"
    cv2.putText(frame_bgr, label, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (40,40,40), 2)

# ------------- Logging -------------
LOGGER = None
def init_logging(log_path: str | None):
    global LOGGER
    LOGGER = logging.getLogger("flight")
    LOGGER.setLevel(logging.INFO)
    LOGGER.handlers.clear()
    ch = logging.StreamHandler(sys.stdout)
    ch.setLevel(logging.INFO)
    ch.setFormatter(logging.Formatter("[%(asctime)s] %(message)s", datefmt="%H:%M:%S"))
    LOGGER.addHandler(ch)
    attached_handlers = [ch]
    if log_path:
        os.makedirs(os.path.dirname(log_path) or ".", exist_ok=True)
        fh = logging.FileHandler(log_path, encoding="utf-8")
        fh.setLevel(logging.INFO)
        fh.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
        LOGGER.addHandler(fh)
        attached_handlers.append(fh)
        LOGGER.info(f"Logging to {log_path}")
    # Mirror handlers on djitellopy logger so SDK messages reach file too.
    sdk_logger = logging.getLogger("djitellopy")
    sdk_logger.setLevel(logging.INFO)
    sdk_logger.handlers.clear()
    for h in attached_handlers:
        sdk_logger.addHandler(h)
    sdk_logger.propagate = False

def log_i(msg): print(f"[*] {msg}"); LOGGER and LOGGER.info(msg)
def log_w(msg): print(f"[!] {msg}"); LOGGER and LOGGER.warning(msg)
def log_e(msg): print(f"[X] {msg}"); LOGGER and LOGGER.error(msg)

# ------------- SDK wrappers -------------
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


def _state_height_cm(t: Tello) -> float | None:
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


def _capture_command_snapshot(t: Tello | None, label: str):
    if not isinstance(t, Tello):
        return None
    if label in ("takeoff", "land", "move_up", "move_down"):
        return {"h": _state_height_cm(t)}
    return None


def _command_effect_seen(
    t: Tello | None,
    label: str,
    before: dict | None,
    args,
) -> bool:
    if not isinstance(t, Tello) or before is None:
        return False
    tol = float(getattr(C, "COMMAND_SUCCESS_TOL_CM", 10.0))
    now = _capture_command_snapshot(t, label)
    if not now:
        return False
    if label == "takeoff":
        h_now = now.get("h")
        if h_now is None:
            return False
        return h_now >= float(getattr(C, "TAKEOFF_SUCCESS_HEIGHT_CM", 30.0))
    if label == "land":
        h_now = now.get("h")
        if h_now is None:
            return False
        return h_now <= tol
    if label in ("move_up", "move_down"):
        h_before = before.get("h")
        h_now = now.get("h")
        if h_before is None or h_now is None:
            return False
        target = float(args[0]) if args else 0.0
        delta = h_now - h_before
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
    pad = base + per_cm * abs(dist_cm)
    return max(0.0, pad)


def ensure_command_mode(t: Tello, *, log_recover: bool = True) -> bool:
    try:
        resp = t.send_command_with_return("command")
        if isinstance(resp, str):
            resp_l = resp.lower()
            if resp_l not in ("ok", "ack", "ready"):
                log_w(f"Re-entered command mode, response: {resp}")
        if log_recover:
            log_w("Sent 'command' to re-sync SDK mode.")
        time.sleep(0.2)
        return True
    except Exception as e:
        if log_recover:
            log_w(f"command-mode recovery failed: {e}")
        return False


def try_cmd(fn, *args, retries=C.RETRIES, sleep=C.RETRY_SLEEP, label="cmd"):
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
            result = fn(*args)
            return result
        except Exception as e:
            attempts += 1
            imu_err = _is_imu_error(e)
            timeout_err = _is_timeout_error(e)
            if timeout_override_applied and isinstance(t_obj, Tello):
                t_obj.RESPONSE_TIMEOUT = original_timeout
                timeout_override_applied = False
            if timeout_err and _command_effect_seen(t_obj, label, snapshot, args):
                log_w(f"{label} timeout but state indicates completion; skipping retries.")
                return None
            if _needs_command_recover(e):
                t_obj = getattr(fn, "__self__", None)
                if isinstance(t_obj, Tello):
                    recovered = ensure_command_mode(t_obj)
                    if recovered:
                        log_w(f"{label} retrying after command-mode recovery ({attempts}/{max_attempts-1}).")
                        time.sleep(sleep)
                        if attempts < max_attempts:
                            continue
                # fall through if not recovered
            if imu_err and max_attempts == base_attempts:
                max_attempts += extra_imu
            if attempts < max_attempts:
                wait = getattr(C, "IMU_RECOVER_SLEEP", sleep) if imu_err else sleep
                reason = " (IMU not ready)" if imu_err else ""
                log_w(f"{label} failed ({e}); retry {attempts}/{max_attempts-1}{reason}.")
                time.sleep(wait)
            else:
                log_e(f"{label} failed after {max_attempts} tries: {e}")
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


def get_current_yaw(t: Tello) -> float | None:
    try:
        state = t.get_current_state()
        if not state:
            return None
        yaw = state.get("yaw")
        if yaw is None:
            return None
        return float(yaw)
    except Exception as e:
        log_w(f"get_current_state error: {e}")
        return None


def correct_heading_if_needed(t: Tello, expected_yaw: float) -> float:
    tol = float(getattr(C, "DRIFT_HEADING_TOL_DEG", 0) or 0)
    if tol <= 0:
        return expected_yaw
    actual = get_current_yaw(t)
    if actual is None:
        return expected_yaw
    diff = _normalize_yaw(actual - expected_yaw)
    if abs(diff) < tol:
        return expected_yaw
    max_c = float(getattr(C, "DRIFT_CORRECT_MAX_DEG", tol) or tol)
    step = max(-max_c, min(max_c, diff))
    log_w(f"Heading drift detected: actual {actual:+.1f}°, expected {expected_yaw:+.1f}° -> correcting {-step:+.1f}°")
    rotate_signed_deg(t, -step)
    updated = get_current_yaw(t)
    if updated is not None:
        return _normalize_yaw(updated)
    return _normalize_yaw(expected_yaw - step)


def fallback_rc_descent(t: Tello):
    log_w("Attempting RC descent fallback.")
    try:
        t.send_rc_control(0, 0, -20, 0)
        time.sleep(2.5)
        t.send_rc_control(0, 0, 0, 0)
    except Exception as e:
        log_e(f"RC descent fallback failed: {e}; issuing emergency stop.")
        try:
            t.emergency()
        except Exception as ee:
            log_e(f"Emergency command failed: {ee}")


def safe_land(t: Tello):
    try:
        try_cmd(t.land, label="land")
        return
    except Exception as e:
        log_w(f"land error: {e}")
    if not ensure_command_mode(t, log_recover=True):
        log_w("Unable to re-enter command mode before RC descent.")
    fallback_rc_descent(t)


def rotate_signed_deg(t: Tello, ang_deg: float):
    a = int(round(ang_deg))
    if a == 0:
        return
    min_turn = max(0, getattr(C, "MIN_TURN_DEG", 0))
    if abs(a) < min_turn:
        log_w(f"turn {a:+d} deg below MIN_TURN_DEG ({min_turn}); skipping.")
        return

    chunk_cfg = getattr(C, "TURN_CHUNK_DEG", 0)
    step_limit = abs(int(chunk_cfg)) if chunk_cfg else 0
    remaining = abs(a)
    fn = t.rotate_counter_clockwise if a > 0 else t.rotate_clockwise
    label = "rotate_ccw" if a > 0 else "rotate_cw"

    # Stop any residual RC commands before turning
    try:
        t.send_rc_control(0, 0, 0, 0)
    except Exception:
        pass
    time.sleep(C.TURN_SLEEP)

    chunk_idx = 0
    while remaining > 0:
        step = remaining if step_limit <= 0 else min(remaining, step_limit)
        chunk_idx += 1
        direction = "CCW" if a > 0 else "CW"
        if step_limit > 0:
            log_i(f"    turn chunk {chunk_idx}: {direction} {step} deg (rem {remaining-step})")
        try:
            try_cmd(fn, step, label=label)
        except Exception as e:
            if _is_imu_error(e):
                fallback_deg = step if a > 0 else -step
                if rc_yaw_fallback(t, fallback_deg):
                    log_w(f"{label} chunk used RC fallback for {fallback_deg:+d} deg.")
                else:
                    log_w(f"{label} chunk aborted due to IMU error; skipping remaining turn.")
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
    log_w(f"    rc yaw fallback: rc 0 0 0 {yaw_cmd} for {duration:.2f}s")
    success = False
    try:
        t.send_rc_control(0, 0, 0, yaw_cmd)
        time.sleep(duration)
        success = True
    except Exception as e:
        log_e(f"RC yaw fallback failed: {e}")
    finally:
        try:
            t.send_rc_control(0, 0, 0, 0)
        except Exception:
            pass
        time.sleep(getattr(C, "RC_YAW_RECOVER_PAUSE", 0.2))
    return success


# ------------- Detection policies -------------
def do_policy_v1_hold_until_lost(
    t: Tello,
    det: FireDetector,
    frame_supplier,
    initial_det: FireDetection | None = None,
):
    log_i("V1: FIRE detected -> approaching; resume after target is lost.")
    t.send_rc_control(0,0,0,0); t.hover(); time.sleep(0.1)
    last_seen = time.time()
    current = initial_det if (initial_det and initial_det.has_fire) else None
    if current:
        last_seen = current.ts
    while True:
        if current is None:
            frame = frame_supplier() if frame_supplier else None
            current = det.infer(frame)
        if current.has_fire:
            last_seen = time.time()
            try:
                approach_once(t, current)
            except Exception as e:
                log_w(f"approach_once error: {e}")
        else:
            if (time.time() - last_seen) * 1000.0 > C.FIRE_LOST_MS:
                log_i("V1: Fire lost -> resuming mission.")
                break
        time.sleep(0.05)
        current = None


def do_policy_v2_approach_then_hold(
    t: Tello,
    det: FireDetector,
    frame_supplier,
    initial_det: FireDetection | None = None,
):
    log_i(f"V2: FIRE detected -> approach then dwell {C.HOLD_SECS:.1f}s.")
    t.send_rc_control(0,0,0,0); t.hover(); time.sleep(0.1)
    last_seen = None
    steps = 0
    current = initial_det if (initial_det and initial_det.has_fire) else None
    if current:
        last_seen = current.ts
    while True:
        if current is None:
            frame = frame_supplier() if frame_supplier else None
            current = det.infer(frame)
        if current.has_fire:
            last_seen = time.time()
            if steps < C.APPROACH_MAX_STEPS:
                try:
                    approach_once(t, current)
                    steps += 1
                except Exception as e:
                    log_w(f"approach_once error: {e}")
        else:
            if last_seen is None:
                log_i("V2: Fire not re-detected; resuming.")
                break
        if last_seen is not None and (time.time() - last_seen) >= C.HOLD_SECS:
            log_i("V2: Hold complete -> resuming mission.")
            break
        time.sleep(0.05)
        current = None


def main(json_path, mode_version: int, ai_enabled: bool, show_video: bool):
    segs, meta = load_plan(json_path)
    ALT_CM = int(meta.get("height_cm", C.ALT_CM))
    SPEED_CM_S = int(meta.get("speed_cm_s", C.SPEED_CM_S))
    policy_label = mode_version if ai_enabled else '-'
    log_i(f"Altitude {ALT_CM} cm, speed {SPEED_CM_S} cm/s, AI={'ON' if ai_enabled else 'OFF'} policy={policy_label}")

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
            log_i(f"Connecting (attempt {attempt}/{C.CONNECT_RETRIES})...")
            t.connect()
            b = t.get_battery()
            log_i(f"Battery {b}%")
            ok = True
            break
        except Exception as e:
            log_w(f"connect error: {e}")
            time.sleep(C.CONNECT_BACKOFF)
    if not ok:
        log_e("Unable to connect. Check TELLO Wi-Fi / power / close other Tello apps.")
        return 3

    expected_yaw = get_current_yaw(t)
    if expected_yaw is None:
        expected_yaw = 0.0
    else:
        expected_yaw = _normalize_yaw(expected_yaw)

    need_detector = ai_enabled or show_video
    stream_active = False
    frame_supplier = None
    frame_read = None
    if need_detector:
        try:
            t.streamon()
            stream_active = True
            if hasattr(t, "get_frame_read"):
                frame_read = t.get_frame_read()

                def _supply_frame(fr=frame_read):
                    return fr.frame

                frame_supplier = _supply_frame
        except Exception as e:
            log_w(f"streamon error - disabling AI/show-video: {e}")
            if ai_enabled:
                ai_enabled = False
            if show_video:
                show_video = False
            need_detector = False
            frame_supplier = None

    try:
        t.set_speed(SPEED_CM_S)
    except Exception as e:
        log_w(f"set_speed error: {e}")

    log_i("Takeoff.")
    try:
        t.takeoff()
    except Exception as e:
        log_e(f"takeoff failed: {e}")
        t.end()
        return 5
    time.sleep(0.5)

    stabilize = max(0.0, getattr(C, "IMU_STABILIZE_SECS", 0.0))
    if stabilize > 0:
        log_i(f"Hovering for IMU stabilization ({stabilize:.1f}s).")
        time.sleep(stabilize)

    climb = max(0, min(ALT_CM - 20, C.MAX_MOVE_CM))
    if climb > 0:
        climb_step = max(0, getattr(C, "CLIMB_CHUNK_CM", 0))
        remaining_climb = climb
        chunk_idx = 0
        while remaining_climb > 0:
            step = remaining_climb if climb_step <= 0 else min(remaining_climb, climb_step)
            chunk_idx += 1
            if climb_step > 0 and remaining_climb > step:
                log_i(f"    climb chunk {chunk_idx}: {step} cm (rem {remaining_climb - step})")
            try:
                try_cmd(t.move_up, step, label="move_up")
            except Exception as e:
                log_w(f"move_up({step}) failed; continuing without additional climb: {e}")
                break
            remaining_climb -= step
            time.sleep(C.MOVE_SLEEP)

    detector = None
    if need_detector and frame_supplier:
        detector = FireDetector(enable_model=ai_enabled, show_video=show_video if show_video else None)
        if ai_enabled:
            detector.start_async(frame_supplier)
    elif need_detector:
        log_w("Stream active but frame source unavailable; live preview/AI disabled.")
        if ai_enabled:
            ai_enabled = False
        if show_video:
            show_video = False
        need_detector = False

    aborted = False
    try:
        for i, (turn_deg, dist_cm) in enumerate(segs):
            try:
                b = t.get_battery()
                if b is not None and b <= C.LOW_BATT_RTH:
                    log_e(f"Low battery {b}% - stop mission")
                    aborted = True
                    break
            except Exception as e:
                log_w(f"get_battery error: {e}")

            if turn_deg:
                log_i(f"[{i}] turn {turn_deg:+d} deg")
                rotate_signed_deg(t, turn_deg)
                expected_yaw = _normalize_yaw(expected_yaw + turn_deg)
                actual_after_turn = get_current_yaw(t)
                if actual_after_turn is not None:
                    expected_yaw = _normalize_yaw(actual_after_turn)

            remaining = int(round(dist_cm))
            log_i(f"[{i}] forward total {remaining} cm")
            while remaining >= C.MIN_MOVE_CM and not aborted:
                step = min(C.FORWARD_STEP_CM, remaining, C.MAX_MOVE_CM)
                try:
                    try_cmd(t.move_forward, step, label="move_forward")
                except Exception as e:
                    log_w(f"move_forward({step}) failed; skipping remaining distance: {e}")
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
                    elif show_video and frame_supplier:
                        frame = frame_supplier()
                        if frame is not None:
                            detector.infer(frame)

            if C.PAUSE_PER_SEG > 0:
                time.sleep(C.PAUSE_PER_SEG)

        if not aborted:
            log_i("Mission complete. Landing.")
    except KeyboardInterrupt:
        log_e("KeyboardInterrupt - landing")
    except Exception as e:
        log_e(f"Runtime error: {e} - landing")
    finally:
        safe_land(t)
        if detector:
            try:
                detector.close()
            except Exception:
                pass
        if stream_active:
            try:
                t.streamoff()
            except Exception:
                pass
        t.end()
        log_i("Done.")
    return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Fly Tello plan with optional fire detection")
    ap.add_argument("json_path", nargs="?", default=None,
                    help="Path to beta_waypoint JSON (relative names are looked up in plans/). If omitted, uses newest file in plans/.")
    ap.add_argument("--mode", choices=["1","2"], default=None,
                    help="AI policy: 1=resume when lost, 2=approach+hold. If omitted, uses config.VERSION")
    ap.add_argument("--ai", choices=["auto","on","off"], default="auto",
                    help="Force AI on/off, or use config (default: auto)")
    ap.add_argument("--log", dest="log_path", default=None,
                    help="Optional log file (e.g. logs/flight_YYYYMMDD_HHMM.log). Use '' to auto-generate.")
    ap.add_argument("--use-last", action="store_true",
                    help="Ignore json_path and use the newest plan in plans/")
    ap.add_argument("--show-video", action="store_true",
                    help="Force live preview window even if AI is disabled.")

    args = ap.parse_args()

    # decide which plan to use
    if args.use_last or args.json_path is None:
        last = find_latest_beta_waypoint_json()
        if last:
            json_path = last
            print(f"[*] Using latest plan: {json_path}")
        else:
            print("[X] No beta_waypoint*.json found in plans/ and no path provided."); sys.exit(2)
    else:
        p = Path(args.json_path)
        json_path = p if p.is_absolute() else (PLAN_DIR / p)
        if not json_path.exists():
            print(f"[X] JSON not found: {json_path}"); sys.exit(2)

    # logging
    log_path = args.log_path
    if log_path == "":
        log_path = f"logs/flight_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
    init_logging(log_path)

    # AI enable
    ai_enabled = C.ENABLE_AI if args.ai == "auto" else (args.ai == "on")

    # policy
    policy = int(C.VERSION) if args.mode is None else int(args.mode)
    policy_internal = policy if ai_enabled else 0

    sys.exit(main(str(json_path), policy_internal, ai_enabled, args.show_video))
