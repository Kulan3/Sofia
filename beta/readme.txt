Beta Stack Quick Reference
==========================

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
