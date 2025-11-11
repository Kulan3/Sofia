import os
from pathlib import Path

# =========================
# Mission & Drone Defaults
# =========================

# Basic Flight
ALT_CM             = 60      # target height from takeoff (cm)
SPEED_CM_S         = 60      # 10-50 (indoors)
LOW_BATT_RTH       = 10      # % battery threshold (abort at/under this)

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
IMU_RECOVER_SLEEP  = 2.5     # wait time when IMU reports not ready (s)
IMU_RECOVER_MAX    = 4       # extra attempts allowed on IMU-not-ready errors (per command)
IMU_STABILIZE_SECS  = 3.0     # wait after takeoff before issuing movement commands (s)
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
YOLO_MODEL_PATH    = str((_MODEL_DIR / "best.pt").resolve())  # change to your weights if needed
DETECT_CLASSES     = ['fire']              # None = any class; or ['fire','smoke']
DETECT_CONF        = 0.40              # confidence threshold (0..1)

# Video / geometry
FRAME_W            = 960
FRAME_H            = 720
# Rough FOVs; you can calibrate these
H_FOV_DEG          = 82.0
V_FOV_DEG          = 52.0

# Live preview window (works even if AI off)
SHOW_VIDEO         = True              # show an OpenCV window with stream/overlay

# Debounce / decision
FIRE_PERSIST_MS    = 200               # require detection persist this long to count as 'real'
FIRE_LOST_MS       = 2000               # consider 'lost' after no detection for this long

# =========================
# Fire Handling Behavior
# =========================
# Approach parameters (used by both policies)
CENTER_TOL_PX      = 40      # |dx| <= this means centered (pixels)
VERTICAL_TOL_PX    = 40      # |dy| <= this means centered vertically (pixels)
NEAR_AREA_FRAC     = 0.25    # bbox area fraction considered 'near enough'
APPROACH_YAW_STEP  = 10      # deg per yaw correction
APPROACH_STRAFE_CM = 25      # reserved for future lateral moves
APPROACH_FWD_CM    = 40      # cm per forward step (legacy)
APPROACH_MAX_STEPS = 15      # failsafe loop limit for old policies
HOLD_SECS          = 6.0     # dwell time when aligned on target
VERTICAL_STEP_CM   = 20      # cm per vertical correction during approach
FORWARD_APPROACH_STEP_CM = 60  # cm per forward move while closing distance
APPROACH_DISTANCE_TOL_CM = 5   # acceptable +/- distance band
APPROACH_TIMEOUT_S       = 60  # give up approaching after this many seconds
APPROACH_LOST_MS         = 3000  # grace period without detection during approach

# Optional recording (set path or None)
_DEFAULT_VIDEO_DIR = Path(__file__).resolve().parent / "videos"
_VIDEO_DIR_ENV     = os.environ.get("TELLO_VIDEO_DIR")
if _VIDEO_DIR_ENV:
    base_video_dir = Path(_VIDEO_DIR_ENV).expanduser()
else:
    base_video_dir = _DEFAULT_VIDEO_DIR
VIDEO_SAVE_PATH    = str((base_video_dir / "").resolve())
VIDEO_CODEC        = "mp4v"
VIDEO_FPS          = 60
TELLO_FRAME_RGB    = True   # djitellopy delivers BGR frames; set True only if frames are already RGB
ASYNC_FRAME_HZ     = 12      # background frame polling rate when stream is on

# Drift mitigation
DRIFT_HEADING_TOL_DEG = 5    # correct heading if |actual-expected| exceeds this
DRIFT_CORRECT_MAX_DEG = 10   # clamp correction magnitude per adjustment

# Target specifications: real width (meters) and desired standoff distance (meters)
TARGET_SPECS = {
    "fire": {
        "real_width_m": 0.066,
        "approach_distance_m": 0.70,
    }
}
