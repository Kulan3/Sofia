์Requaire Package

pip install --upgrade pip
pip install djitellopy pygame opencv-python ultralytics numpy

FOR EXECUTING
Drone Project - Run Guide
=========================

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
