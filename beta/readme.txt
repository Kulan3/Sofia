Beta Flight Stack Quickstart
============================

Directory layout
----------------
- `beta_main.py` – mission runner (waypoints, target engagement, safety fallbacks)
- `beta_detect.py` – YOLO detector + optional live preview/recording
- `beta_path_gui.py` – waypoint editor (writes files to `plans/`)
- `dry_main.py` – dry-run tool that prints the command sequence (no hardware)
- `beta_config.py` – tunables for speed/altitude, retries, timeouts, target specs
- `runner.py` – convenience launcher for common scenarios
- `plans/` – stored waypoint JSONs (created by the planner)
- `logs/` – command logs (`flight_*.log`) and AI engagement logs (`flight_ai_*.log`)

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
python beta_main.py --use-last                      # newest plan with engagement + preview
python beta_main.py --use-last --show-video         # force preview window
python beta_main.py plans/custom.json               # explicit plan file
```
Useful flags:
- `--show-video` – keep the preview window open.
- `--log ""` – auto-generate a timestamped log in `logs/`.
- `--log logs/test.log` – write to a fixed log path.

Preview-only
------------
```
python beta_detect.py
```
Displays the YOLO overlay without flying. Close with `x`.

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
- `mission` – latest plan, preview + auto logs
- `silent` – latest plan, no preview
- `preview` – latest plan, preview only
- `detect` – run `beta_detect.py`
- `dry` – invoke the dry-run tool
Append extra CLI flags after `--` (e.g. `python runner.py mission -- --log logs/foo.log`).

Target engagement behaviour
---------------------------
When the detector sees a labelled target listed in `TARGET_SPECS`:
1. Estimate the forward distance from the bounding-box width.
2. Rotate, climb, or descend until the target is centered in the camera.
3. Move forward/backward to reach the configured stand-off distance.
4. Hold position while the target remains visible, then resume the waypoint route.
Each engagement is logged to `flight_ai_*.log` with manoeuvres, distances, confidence, and dwell time.

Safety & retries (see `beta_config.py`)
---------------------------------------
- `RESPONSE_TIMEOUT_S`, `COMMAND_RETRY_COUNT` – base SDK timeout & retries.
- `COMMAND_TIMEOUT_PAD` – adds distance-aware grace time (longer moves wait longer before retrying).
- After a timeout the controller inspects telemetry (height, etc.) and skips retries if the move already succeeded.
- `Ctrl+C` triggers `safe_land()`: tries SDK land, then RC descent, then `emergency` if necessary.

Recommended first-flight flow
-----------------------------
1. Plan the path – `python beta_path_gui.py`, save to `plans/`.
2. Dry run – `python dry_main.py --use-last`.
3. Preview the stream – `python beta_detect.py` to confirm video.
4. Indoor hover test – `python beta_main.py --use-last --show-video --log ""`.
5. Full mission – same command outdoors, keeping the drone in view.

Troubleshooting tips
--------------------
- If `streamon` fails, the script logs a warning and keeps flying without detection/preview.
- Missing `ok` replies typically mean weak Wi-Fi; keep the drone close or tweak timeout pads.
- Video requires UDP port 11111 (and telemetry needs 8890) open in your firewall.
- Check `logs/flight_*.log` and `logs/flight_ai_*.log` after each mission for command timing and engagement details.
- For manual experiments, call `beta_detect.FireDetector.infer(frame)` to retrieve offsets/confidence from a frame.

Compatibility notes
-------------------
- Works on Python 3.8+ (no pattern matching or `|` unions).
- Requires `djitellopy`, `opencv-python`, `ultralytics`, `numpy`, and `av`.
- Place YOLO weights next to the scripts and set `YOLO_MODEL_PATH` in `beta_config.py`.
