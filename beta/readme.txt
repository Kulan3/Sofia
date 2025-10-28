Beta Flight Stack Quickstart  
=============================

Directory layout  
----------------
- `beta_main.py` – mission runner (waypoints, AI policies, safety fallbacks)  
- `beta_detect.py` – YOLO detector + optional live preview/recording  
- `beta_path_gui.py` – waypoint editor (writes files to `plans/`)  
- `dry_main.py` – dry-run tool that prints the command sequence (no hardware)  
- `beta_config.py` – tunables for speed/altitude, retries, timeouts, AI switches  
- `runner.py` – convenience launcher for common scenarios  
- `plans/` – stored waypoint JSONs (created by the planner)  
- `logs/` – command logs when `--log` is supplied

Planning routes  
---------------
```
python beta_path_gui.py
```
- Free mode: click to add points, `Z` undo, `Y` redo, `H` return to origin.  
- Grid mode: choose size/cell spacing via **Gen Grid**, swap sweep direction with **Grid Dir**.  
- **Save JSON** to export into `plans/beta_waypoint.json` (type a new filename to branch plans).

Running missions  
----------------
```
python beta_main.py --use-last                      # newest plan, AI flag from config
python beta_main.py --use-last --ai off             # fly waypoints only
python beta_main.py --use-last --ai on --mode 1     # policy 1: approach, resume
python beta_main.py --use-last --ai on --mode 2     # policy 2: approach, hold
python beta_main.py plans/custom.json               # explicit plan file
```
Useful flags:  
- `--show-video` – keep the preview window open (works even when AI is off)  
- `--log ""` – auto-generate a timestamped log in `logs/`  
- `--log logs/test.log` – write to a fixed path

Preview without AI  
-------------------
```
python beta_main.py --use-last --ai off --show-video --log ""
```
You get the YOLO overlay but the mission logic stays in control.

Dry run (no drone)  
------------------
```
python dry_main.py --use-last
python dry_main.py plans/beta_waypoint.json
```
Prints the command sequence, move counts, and time estimates for inspection.

Helper launcher  
---------------
```
python runner.py --help
```
Profiles include:  
- `ai1` / `ai2` – policy 1 or 2 with AI enabled  
- `plain` – waypoint-only run  
- `preview` – video only, no AI  
- `diag` – AI auto + preview  
- `dry` – dry-run shortcut  
Append extra CLI flags after `--` (e.g. `python runner.py ai1 -- --log logs/foo.log`).

Safety & retries (see `beta_config.py`)  
---------------------------------------
- `RESPONSE_TIMEOUT_S`, `COMMAND_RETRY_COUNT` – base SDK timeout & retries  
- `COMMAND_TIMEOUT_PAD` – adds distance-aware grace time (e.g. long moves wait longer before retrying)  
- State snapshot check: after a timeout the controller inspects telemetry (height, etc.) and skips retries if the move already succeeded.  
- `Ctrl+C` triggers `safe_land()`: tries SDK land, then RC descent, then `emergency` if necessary.

Recommended first-flight flow  
-----------------------------
1. Plan the path – `python beta_path_gui.py`, save to `plans/`.  
2. Dry run – `python dry_main.py --use-last`.  
3. Indoor hover test – `python beta_main.py --use-last --ai off --show-video --log ""`.  
4. Enable AI once the link is stable – `python beta_main.py --use-last --ai on --mode 1`.

Troubleshooting tips  
--------------------
- If `streamon` fails, the script logs a warning and continues without AI/preview.  
- Slow or missing `ok` replies usually mean weak Wi-Fi; keep the drone close, or tweak timeout pads.  
- Check `logs/flight_*.log` after each mission for command timing and SDK errors.  
- For manual experimentation, `FireDetector.infer(frame)` returns offsets/confidence for a given frame.

Compatibility notes  
-------------------
- Tested on Python 3.8+ (no `match`/`|` unions, typing uses `Optional/Tuple` etc.).  
- Requires `djitellopy`, `opencv-python`, `ultralytics`, `numpy`.  
- YOLO weights (`fire_model.pt` or similar) should live beside the scripts and be referenced in `beta_config.py`.
