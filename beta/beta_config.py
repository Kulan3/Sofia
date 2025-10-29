from pathlib import Path

# =========================
# Mission & Drone Defaults
# =========================

# Basic Flight
ALT_CM             = 60      # target height from takeoff (cm)
SPEED_CM_S         = 60      # 10-50 (indoors)
LOW_BATT_RTH       = 20      # % battery threshold (abort at/under this)

# Waypoint execution
PAUSE_PER_SEG      = 0.0     # pause after each segment (s)
MIN_MOVE_CM        = 20      # Tello min distance per move (SDK)
MAX_MOVE_CM        = 500     # Tello max distance per move (SDK)
FORWARD_STEP_CM    = 60      # per sub-step while flying a segment
TURN_SLEEP         = 0.2     # delay after rotate (s)
MOVE_SLEEP         = 0.2     # delay after move (s)
RETRIES            = 2       # per-command retries
RETRY_SLEEP        = 0.2     # delay between retries
TURN_CHUNK_DEG     = 45      # split large turns into <= this many degrees per command (0 disables chunking)
CLIMB_CHUNK_CM     = 40      # split climb after takeoff into <= this many cm (0 disables chunking)
MIN_TURN_DEG       = 2       # ignore turns smaller than this magnitude (deg)
IMU_RECOVER_SLEEP  = 1.5     # wait time when IMU reports not ready (s)
IMU_RECOVER_MAX    = 3       # extra attempts allowed on IMU-not-ready errors (per command)
IMU_STABILIZE_SECS  = 2.0     # wait after takeoff before issuing movement commands (s)
RC_YAW_SPEED       = 60      # fallback rc yaw speed when imu rejects turn commands (0 disables fallback)
RC_YAW_DEG_PER_SEC = 90      # approximate yaw rate produced by RC_YAW_SPEED (deg/s)
RC_YAW_RECOVER_PAUSE = 0.3   # pause after RC yaw fallback before next command (s)

# Connectivity
CONNECT_RETRIES    = 4       # connect attempts
CONNECT_BACKOFF    = 1.0     # seconds between connect attempts
HEALTH_TIMEOUT_S   = 6.0     # seconds to wait for a battery/SDK response after connect
RESPONSE_TIMEOUT_S = 10.0    # SDK command timeout before retry (default 7s)
COMMAND_RETRY_COUNT = 4      # base retry_count for djitellopy (default 3)
COMMAND_TIMEOUT_PAD = {      # extra grace before timeout, optional per-distance scaling
    "takeoff": {"base": 12.0},
    "land": {"base": 6.0},
    "move_up": {"base": 1.0, "per_cm": 0.04},
    "move_down": {"base": 1.0, "per_cm": 0.04},
    "move_forward": {"base": 0.8, "per_cm": 0.03},
    "move_back": {"base": 0.8, "per_cm": 0.03},
    "move_left": {"base": 0.8, "per_cm": 0.03},
    "move_right": {"base": 0.8, "per_cm": 0.03},
}
COMMAND_SUCCESS_TOL_CM = 10   # tolerance when checking state-based completion
TAKEOFF_SUCCESS_HEIGHT_CM = 30  # height considered successful takeoff

# =========================
# Fire Detection Settings
# =========================

# Global AI switch
ENABLE_AI          = True

# Model
_MODEL_DIR         = Path(__file__).resolve().parent
YOLO_MODEL_PATH    = str((_MODEL_DIR / "fire_model.pt").resolve())  # change to your weights if needed
DETECT_CLASSES     = 'backpack'              # None = any class; or ['fire','smoke']
DETECT_CONF        = 0.45              # confidence threshold (0..1)

# Video / geometry
FRAME_W            = 960
FRAME_H            = 720
# Rough FOVs; you can calibrate these
H_FOV_DEG          = 82.0
V_FOV_DEG          = 52.0

# Live preview window (works even if AI off)
SHOW_VIDEO         = True              # show an OpenCV window with stream/overlay

# Debounce / decision
FIRE_PERSIST_MS    = 300               # require detection persist this long to count as 'real'
FIRE_LOST_MS       = 600               # consider 'lost' after no detection for this long

# =========================
# Fire Handling Behavior
# =========================
# VERSION = 1 -> approach while fire present, continue when lost
# VERSION = 2 -> approach fire, hold nearby for HOLD_SECS, then continue
VERSION                   = 2

# After AI episode, restore heading so path stays true to plan
RESTORE_HEADING_AFTER_AI  = True

# Approach parameters (used by both policies)
CENTER_TOL_PX      = 50      # |dx| <= this means centered (pixels)
NEAR_AREA_FRAC     = 0.25    # bbox area fraction considered 'near enough'
APPROACH_YAW_STEP  = 10      # deg per yaw correction
APPROACH_STRAFE_CM = 25      # cm per lateral (not used by default, kept for expansion)
APPROACH_FWD_CM    = 40      # cm per forward step
APPROACH_MAX_STEPS = 15      # failsafe: avoid infinite approach in policy 2
HOLD_SECS          = 6.0     # dwell time near fire (policy 2)

# Optional recording (set path or None)
VIDEO_SAVE_PATH    = None    # e.g. r"C:\Users\nutth\Videos\Drone\Fly_Test.mp4" or None
VIDEO_FPS          = 40
VIDEO_CODEC        = "mp4v"
TELLO_FRAME_RGB    = True    # djitellopy returns RGB frames by default
ASYNC_FRAME_HZ     = 12      # background frame polling rate when stream is on

# Drift mitigation
DRIFT_HEADING_TOL_DEG = 5    # correct heading if |actual-expected| exceeds this
DRIFT_CORRECT_MAX_DEG = 10   # clamp correction magnitude per adjustment
