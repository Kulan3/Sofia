#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import json, time, math, sys, argparse, os, logging
from datetime import datetime
from pathlib import Path
from djitellopy import Tello

import beta_config as C
from beta_detect import FireDetector, approach_once, FireDetection

# ---------------- plans directory ----------------
BASE_DIR = Path(__file__).resolve().parent
PLAN_DIR = BASE_DIR / "plans"
PLAN_DIR.mkdir(exist_ok=True)

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
    if log_path:
        os.makedirs(os.path.dirname(log_path) or ".", exist_ok=True)
        fh = logging.FileHandler(log_path, encoding="utf-8")
        fh.setLevel(logging.INFO)
        fh.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
        LOGGER.addHandler(fh)
        LOGGER.info(f"Logging to {log_path}")

def log_i(msg): print(f"[*] {msg}"); LOGGER and LOGGER.info(msg)
def log_w(msg): print(f"[!] {msg}"); LOGGER and LOGGER.warning(msg)
def log_e(msg): print(f"[X] {msg}"); LOGGER and LOGGER.error(msg)

# ------------- Math helpers -------------
def vec(a, b):  return (b[0]-a[0], b[1]-a[1])
def dot(u, v):  return u[0]*v[0] + u[1]*v[1]
def cross2(u,v):return u[0]*v[1] - u[1]*v[0]
def norm(u):    return math.hypot(u[0], u[1])
def angle_unsigned(u, v):
    mu, mv = norm(u), norm(v)
    if mu == 0 or mv == 0: return 0
    d = dot(u, v) / (mu * mv)
    d = max(-1.0, min(1.0, d))
    return math.degrees(math.acos(d))
def angle_signed(u, v):
    a = angle_unsigned(u, v)
    c = cross2(u, v)
    return a if c > 0 else (-a if c < 0 else 0)
def initial_heading_from_pos(pos0, pos1):
    return angle_signed((1.0,0.0), vec(pos0, pos1))
def clamp_cm(v):
    v = int(round(v))
    if v == 0: return 0
    s = 1 if v > 0 else -1
    m = max(C.MIN_MOVE_CM, min(abs(v), C.MAX_MOVE_CM))
    return s * m

# ------------- SDK wrappers -------------
def try_cmd(fn, *args, retries=C.RETRIES, sleep=C.RETRY_SLEEP, label="cmd"):
    for k in range(retries + 1):
        try:
            return fn(*args)
        except Exception as e:
            if k < retries:
                log_w(f"{label} failed ({e}); retry {k+1}/{retries} …")
                time.sleep(sleep)
            else:
                log_e(f"{label} failed after {retries+1} tries: {e}")
                raise
def rotate_signed_deg(t: Tello, ang_deg: float):
    a = int(round(ang_deg))
    if a == 0: return
    if a > 0:
        try_cmd(t.rotate_counter_clockwise, a, label="rotate_ccw")
    else:
        try_cmd(t.rotate_clockwise, -a, label="rotate_cw")
    time.sleep(C.TURN_SLEEP)

# ------------- beta_waypoint plan loader -------------
def load_plan(json_path):
    with open(json_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    wp   = data.get("wp", [])
    pos  = data.get("pos", None)
    meta = data.get("meta", {})
    if not wp: raise ValueError("JSON has no 'wp' array.")
    dists = []
    for i, w in enumerate(wp):
        if "dist_cm" not in w:
            raise ValueError(f"dist_cm missing for segment {i}")
        dists.append(int(round(w["dist_cm"])))
    has_signed = any("turn_signed_deg" in w for w in wp)
    turns = [0]*len(dists)
    if has_signed:
        for i in range(len(dists)):
            turns[i] = int(round(wp[i].get("turn_signed_deg", 0)))
        if pos and len(pos) >= 2 and abs(turns[0]) < 1:
            turns[0] = int(round(initial_heading_from_pos(pos[0], pos[1])))
    elif pos and len(pos) >= 2:
        turns[0] = int(round(initial_heading_from_pos(pos[0], pos[1])))
        for i in range(1, len(dists)):
            if i < len(pos)-1:
                turns[i] = int(round(angle_signed(vec(pos[i-1], pos[i]), vec(pos[i], pos[i+1]))))
            else:
                turns[i] = 0
    else:
        for i in range(len(dists)):
            turns[i] = int(round(wp[i].get("angle_deg", 0)))
    segs = [(turns[i], dists[i]) for i in range(len(dists))]
    return segs, meta

def find_latest_beta_waypoint_json():
    cands = []
    cands += list(PLAN_DIR.glob("beta_waypoint.json"))
    cands += list(PLAN_DIR.glob("beta_waypoint_*.json"))
    cands += list(PLAN_DIR.glob("beta_waypoint-*.json"))
    if not cands:
        return None
    return max(cands, key=lambda p: p.stat().st_mtime)

# ------------- Detection policies -------------
def do_policy_v1_hold_until_lost(t: Tello, det: FireDetector):
    log_i("V1: FIRE detected → approaching; resume after target is lost.")
    t.send_rc_control(0,0,0,0); t.hover(); time.sleep(0.1)
    last_seen = time.time()
    while True:
        frame = t.get_frame_read().frame if hasattr(t, "get_frame_read") else None
        d = det.infer(frame)
        if d.has_fire:
            last_seen = time.time()
            try: approach_once(t, d)
            except Exception as e: log_w(f"approach_once error: {e}")
        else:
            if (time.time() - last_seen) * 1000.0 > C.FIRE_LOST_MS:
                log_i("V1: Fire lost → resuming mission."); break
        time.sleep(0.05)

def do_policy_v2_approach_then_hold(t: Tello, det: FireDetector):
    log_i(f"V2: FIRE detected → approach then dwell {C.HOLD_SECS:.1f}s.")
    t.send_rc_control(0,0,0,0); t.hover(); time.sleep(0.1)
    last_seen = None; steps = 0
    while True:
        frame = t.get_frame_read().frame if hasattr(t, "get_frame_read") else None
        d = det.infer(frame)
        if d.has_fire:
            last_seen = time.time()
            if steps < C.APPROACH_MAX_STEPS:
                try: approach_once(t, d); steps += 1
                except Exception as e: log_w(f"approach_once error: {e}")
        else:
            if last_seen is None:
                log_i("V2: Fire not re-detected; resuming."); break
        if last_seen is not None and (time.time() - last_seen) >= C.HOLD_SECS:
            log_i("V2: Hold complete → resuming mission."); break
        time.sleep(0.05)

# ------------- Main flight -------------
def main(json_path, mode_version: int, ai_enabled: bool):
    segs, meta = load_plan(json_path)
    ALT_CM     = int(meta.get("height_cm",  C.ALT_CM))
    SPEED_CM_S = int(meta.get("speed_cm_s", C.SPEED_CM_S))
    log_i(f"Altitude {ALT_CM} cm, speed {SPEED_CM_S} cm/s, AI={'ON' if ai_enabled else 'OFF'} policy={mode_version if ai_enabled else '—'}")

    t = Tello()

    ok = False
    for attempt in range(1, C.CONNECT_RETRIES+1):
        try:
            log_i(f"Connecting (attempt {attempt}/{C.CONNECT_RETRIES}) …")
            t.connect()
            b = t.get_battery(); log_i(f"Battery {b}%")
            ok = True; break
        except Exception as e:
            log_w(f"connect error: {e}"); time.sleep(C.CONNECT_BACKOFF)
    if not ok:
        log_e("Unable to connect. Check TELLO Wi-Fi / power / close other Tello apps."); return 3

    if ai_enabled:
        try:
            t.streamon()
        except Exception as e:
            log_w(f"streamon error → continuing without AI: {e}")
            ai_enabled = False

    try:
        t.set_speed(SPEED_CM_S)
    except Exception as e:
        log_w(f"set_speed error: {e}")

    log_i("Takeoff…")
    try:
        t.takeoff()
    except Exception as e:
        log_e(f"takeoff failed: {e}"); t.end(); return 5
    time.sleep(0.5)

    climb = max(0, min(ALT_CM - 20, C.MAX_MOVE_CM))
    if climb > 0:
        try:
            t.move_up(climb); time.sleep(C.MOVE_SLEEP)
        except Exception as e:
            log_w(f"move_up error: {e}")

    detector = FireDetector() if ai_enabled else None

    aborted = False
    try:
        for i, (turn_deg, dist_cm) in enumerate(segs):
            try:
                b = t.get_battery()
                if b is not None and b <= C.LOW_BATT_RTH:
                    log_e(f"Low battery {b}% → stop mission"); aborted = True; break
            except Exception as e:
                log_w(f"get_battery error: {e}")

            if turn_deg:
                log_i(f"[{i}] turn {turn_deg:+d}°")
                rotate_signed_deg(t, turn_deg)

            remaining = int(round(dist_cm))
            log_i(f"[{i}] forward total {remaining} cm")
            while remaining >= C.MIN_MOVE_CM:
                step = min(C.FORWARD_STEP_CM, remaining, C.MAX_MOVE_CM)
                try:
                    t.move_forward(step); time.sleep(C.MOVE_SLEEP)
                except Exception as e:
                    log_w(f"move_forward({step}) error: {e}")
                remaining -= step

                if ai_enabled and detector:
                    frame = t.get_frame_read().frame if hasattr(t, "get_frame_read") else None
                    d = detector.infer(frame)
                    if d.has_fire:
                        if mode_version == 1:
                            do_policy_v1_hold_until_lost(t, detector)
                        elif mode_version == 2:
                            do_policy_v2_approach_then_hold(t, detector)

            if C.PAUSE_PER_SEG > 0:
                time.sleep(C.PAUSE_PER_SEG)

        if not aborted:
            log_i("Mission complete. Landing…")
    except KeyboardInterrupt:
        log_e("KeyboardInterrupt → landing")
    except Exception as e:
        log_e(f"Runtime error: {e} → landing")
    finally:
        try:
            t.land()
        except Exception as e:
            log_w(f"land error: {e}")
        if ai_enabled:
            try: t.streamoff()
            except Exception: pass
        t.end(); log_i("Done.")
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

    sys.exit(main(str(json_path), policy_internal, ai_enabled))
