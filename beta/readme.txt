
Key scripts
-----------
- `beta_main.py`            Mission runner (waypoints, AI policies, safety fallbacks)
- `beta_detect.py`          YOLO fire detector + video overlay
- `beta_path_gui.py`        Planner UI (writes `plans/beta_waypoint*.json`)
- `dry_main.py`             Offline dry-run of a plan (no hardware)
- `runner.py`               Convenience launcher for common profiles
- `beta_config.py`          All tunables (altitude, retries, timeout pads, AI switches)

Planning a route
----------------
```
python beta_path_gui.py
```
- Free mode: click to drop points, `Z` undo, `Y` redo, `H` return to start.
- Grid mode: press **Gen Grid**, set width/height/cell, toggle sweep with **Grid Dir**.
- **Save JSON** writes `plans/beta_waypoint.json` (enter a new filename to branch plans).

Flying missions
---------------
```
python beta_main.py --use-last                    # newest plan, AI flag from config
python beta_main.py --use-last --ai off           # pure waypoint execution
python beta_main.py --use-last --ai on --mode 1   # AI policy 1 (approach then resume)
python beta_main.py --use-last --ai on --mode 2   # AI policy 2 (approach then dwell)
python beta_main.py plans/custom.json             # explicit plan file
```
Useful flags:
- `--show-video` opens the live preview window (works even with `--ai off`)
- `--log ""` auto-creates `logs/flight_YYYYMMDD_HHMMSS.log`
- `--log logs/test.log` writes to a fixed path

Live preview without AI
-----------------------
```
python beta_main.py --use-last --ai off --show-video --log ""
```
Frames are still annotated, but the AI policies stay dormant.

Safety/timeouts overview
------------------------
- `beta_config.py` exposes `RESPONSE_TIMEOUT_S`, `COMMAND_RETRY_COUNT`, and the
  distance-aware `COMMAND_TIMEOUT_PAD`. Longer moves automatically get extra grace time.
- After a timeout the script inspects telemetry (height, etc.). If the move already finished,
  retries are skipped.
- `Ctrl+C` always funnels into `safe_land()`, which attempts SDK landing then RC descent.

Using the helper launcher
-------------------------
```
python runner.py --help
```
Profiles include dry-run, preview-only, AI policies, and a diagnostic mode that logs sensor
state every loop.

Recommended first-flight flow
-----------------------------
1. **Plan** the route – `python beta_path_gui.py`, save to `plans/`.
2. **Dry run** the JSON – `python dry_main.py --use-last`.
3. **Hover test** indoors – `python beta_main.py --use-last --ai off --show-video --log ""`.
4. **Enable AI** once the link is stable – `python beta_main.py --use-last --ai on --mode 1`.

Troubleshooting check list
--------------------------
- If `streamon` fails, the script logs a warning and keeps flying without AI/preview.
- Missing `ok` responses usually mean weak Wi-Fi. Keep the drone close, reduce interference,
  or adjust the timeout pads in `beta_config.py`.
- Use the latest log in `logs/` to inspect command timings and SDK errors.
- For manual experiments, the detector alone can be exercised via
  `from beta_detect import FireDetector` and calling `infer(frame)`.

Quick folder sanity
-------------------
- `plans/` contains the waypoint JSONs.
- `logs/` gathers flight logs when `--log` is provided.
- YOLO weights live next to the scripts (`fire_model.pt`, etc.) and are referenced from
  `beta_config.py`.
์Requaire Package

pip install --upgrade pip
pip install djitellopy pygame opencv-python ultralytics numpy

FOR EXECUTING
Drone Project - Run Guide

Plan editor (make/inspect paths)
--------------------------------
- Launch the planner GUI (saves to `plans/beta_waypoint.json` by default):
  python beta_path_gui.py

  Tips in GUI:
  - Free mode: click to drop points; Z undo / Y redo; H back to base; L load last JSON.
  - Grid mode: "Gen Grid" -> set width/height/cell; "Grid Dir" toggles X<->Y.
  - "Save JSON" writes plans/beta_waypoint.json.

Flying the plan (beta_main.py)
------------------------------
- Use the newest plan (beta_waypoint*.json under plans/), AI policy from config.py:
  python beta_main.py --use-last

- Use newest plan, force AI ON, policy 1 (approach & resume when lost):
  python beta_main.py --use-last --ai on --mode 1

- Use newest plan, force AI ON, policy 2 (approach & hold then resume):
  python beta_main.py --use-last --ai on --mode 2

- Use newest plan, force AI OFF (ignore --mode, just fly beta_waypoints):
  python beta_main.py --use-last --ai off

- Fly an explicit plan file:
  python beta_main.py plans/beta_waypoint.json

- Auto-create a timestamped log file (PowerShell/CMD empty string):
  python beta_main.py --use-last --log=""

- Write logs to a specific file:
  python beta_main.py --use-last --log logs/flight_test.log

- Respect config.py for AI enable/disable and policy (no CLI overrides for AI):
  python beta_main.py --use-last --ai auto --mode 1
  (With --ai auto, ON/OFF follows ENABLE_AI in config.py. If AI is OFF, --mode is ignored.)

Dry run (no drone required)
---------------------------
- Review timing and distance without connecting to the Tello:
  python dry_main.py --use-last

- Use an explicit plan file:
  python dry_main.py plans/beta_waypoint.json

Live camera view (real-time video)
----------------------------------
Option A - If beta_main.py supports --show-video (live view while flying):
  python beta_main.py --use-last --ai on --mode 2 --show-video
  (Close the window with 'q' or by clicking X.)

Option B - Standalone viewer (no flying) using beta_detect.py (if CLI added):
  python beta_detect.py --show --record out.mp4 --conf 0.5 --model yolov8s.pt

Recommended first-flight flow
-----------------------------
1) Make a plan:
   python beta_path_gui.py
   (Save -> creates plans/beta_waypoint.json)

2) Dry-run the plan:
   python dry_main.py --use-last

3) Test flight with AI OFF:
   python beta_main.py --use-last --ai off --log=""

4) Flight with AI ON, policy 2 (approach+hold):
   python beta_main.py --use-last --ai on --mode 2 --log=""

Notes & gotchas
---------------
- If you see "JSON not found", ensure the planner saved into `plans/` or pass the full path.
- In PowerShell/CMD, an empty string is `""`; in bash, use ''.
- Ctrl+C during flight triggers a safe landing in the script.
- If AI is ON but streamon() fails, the script logs a warning and continues without AI.
- Logs always go to console; add `--log=""` for an auto timestamped file under `logs/`.
- The YOLO model path defaults to `fire_model.pt` located beside these scripts; update `beta_config.py` if you relocate the weights.

Folder expectations
-------------------
- beta_path_gui.py, beta_main.py, beta_detect.py, dry_main.py at project root.
- plans/    (planner writes beta_waypoint.json here)
- logs/     (optional: beta_main.py writes log files here when --log is used)
