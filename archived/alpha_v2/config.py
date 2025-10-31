# =========================
# Mission & Drone Defaults
# =========================

# Flight
ALT_CM             = 90      # target altitude above takeoff (cm)
SPEED_CM_S         = 30      # 10–50 safe indoors
LOW_BATT_RTH       = 20      # % battery threshold (abort at/under this)

# Waypoint execution
PAUSE_PER_SEG      = 0.0     # pause after each segment (s)
MIN_MOVE_CM        = 20      # Tello min distance per move
MAX_MOVE_CM        = 500     # Tello max distance per move
FORWARD_STEP_CM    = 60      # how far to move per sub-step while flying a segment
TURN_SLEEP         = 0.2     # delay after rotate (s)
MOVE_SLEEP         = 0.2     # delay after move (s)
RETRIES            = 2       # per-command retries
RETRY_SLEEP        = 0.2     # delay between retries

# Connectivity
CONNECT_RETRIES    = 4       # connect attempts
CONNECT_BACKOFF    = 1.0     # seconds between connect attempts
HEALTH_TIMEOUT_S   = 6.0     # seconds to wait for a battery/SDK response after connect

# =========================
# Fire Detection Settings
# =========================

# Global AI switch (set False to completely disable detection logic)
ENABLE_AI          = True

# Model
YOLO_MODEL_PATH    = "fire_model.pt"  # change to your trained weights path
DETECT_CLASSES     = None             # None = allow whatever model predicts; or list like ['fire','smoke']
DETECT_CONF        = 0.45             # confidence threshold (0..1)

# Video / geometry (default)
FRAME_W            = 960
FRAME_H            = 720
# Rough FOVs; adjust if you calibrate (degrees)
H_FOV_DEG          = 82.0
V_FOV_DEG          = 52.0

# Debounce / decision
FIRE_PERSIST_MS    = 300      # require detection persist this long to count as “real”
FIRE_LOST_MS       = 600      # consider “lost” after no detection for this long

# =========================
# Fire Handling Behavior
# =========================
# VERSION = 1 -> pause/approach while fire present, continue when lost
# VERSION = 2 -> approach fire, hold nearby for HOLD_SECS, then continue
VERSION            = 1

# Approach (used by both policies)
CENTER_TOL_PX      = 50      # how close to center (in pixels) counts as centered
NEAR_AREA_FRAC     = 0.25    # bbox area fraction of frame that counts as “near enough”
APPROACH_YAW_STEP  = 10      # deg per correction
APPROACH_STRAFE_CM = 25      # cm per lateral correction
APPROACH_FWD_CM    = 40      # cm per forward step
APPROACH_MAX_STEPS = 15      # fail-safe to avoid infinite approach in policy 2
HOLD_SECS          = 6.0     # how long to hold near fire before continuing (policy 2)

# Optional recording (set path or None)
VIDEO_SAVE_PATH    = None    # e.g. r"C:\Users\nutth\Videos\Drone\Fly_Test.mp4" or None
VIDEO_FPS          = 40
VIDEO_CODEC        = "mp4v"
