# =========================
# Mission & Drone Defaults
# =========================

# Flight
ALT_CM             = 90      # target altitude above takeoff (cm)
SPEED_CM_S         = 30      # 10–50 safe indoors
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

# Connectivity
CONNECT_RETRIES    = 4       # connect attempts
CONNECT_BACKOFF    = 1.0     # seconds between connect attempts
HEALTH_TIMEOUT_S   = 6.0     # seconds to wait for a battery/SDK response after connect

# =========================
# Fire Detection Settings
# =========================

# Global AI switch
ENABLE_AI          = True

# Model
YOLO_MODEL_PATH    = "fire_model.pt"   # change to your weights
DETECT_CLASSES     = None              # None = any class; or ['fire','smoke']
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
FIRE_PERSIST_MS    = 300               # require detection persist this long to count as “real”
FIRE_LOST_MS       = 600               # consider “lost” after no detection for this long

# =========================
# Fire Handling Behavior
# =========================
# VERSION = 1 -> approach while fire present, continue when lost
# VERSION = 2 -> approach fire, hold nearby for HOLD_SECS, then continue
VERSION                   = 1

# After AI episode, restore heading so path stays true to plan
RESTORE_HEADING_AFTER_AI  = True

# Approach parameters (used by both policies)
CENTER_TOL_PX      = 50      # |dx| <= this means centered (pixels)
NEAR_AREA_FRAC     = 0.25    # bbox area fraction considered “near enough”
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
