#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, time, threading
from pathlib import Path
import cv2
import numpy as np
from ultralytics import YOLO
import beta_config as C

class FireDetection:
    """
    Simple container for one-frame detection result.
    """
    def __init__(self, has_fire: bool, dx=0.0, dy=0.0, area_frac=0.0, conf=0.0, bbox=None, ts: float | None = None):
        self.has_fire  = bool(has_fire)
        self.dx        = float(dx)          # + right of center, - left of center
        self.dy        = float(dy)          # + below center, - above center
        self.area_frac = float(area_frac)   # bbox area fraction of frame (0..1)
        self.conf      = float(conf)
        self.bbox      = bbox               # (x1,y1,x2,y2) or None
        self.ts        = float(time.time() if ts is None else ts)

class FireDetector:
    """
    Run YOLO on frames. Can also just show live video (if model disabled).
    - If enable_model=False, we only display the frame (no detection).
    - If show_video=True, an OpenCV window is shown with overlay.
    """
    def __init__(self, enable_model: bool = True, show_video: bool | None = None):
        self.enable_model = bool(enable_model)
        self.show_video   = (C.SHOW_VIDEO if show_video is None else bool(show_video))

        self.model = None
        self.model_path = Path(C.YOLO_MODEL_PATH)
        self._expect_rgb = bool(getattr(C, "TELLO_FRAME_RGB", False))
        if self.enable_model:
            if not self.model_path.exists():
                raise FileNotFoundError(f"YOLO model not found at {self.model_path}")
            self.model = YOLO(str(self.model_path), task="detect")

        self.classes = set(C.DETECT_CLASSES) if C.DETECT_CLASSES else None
        self.window_name = "Tello Live"
        self.last_seen_ts = 0.0
        self._vw = None  # VideoWriter
        self._lock = threading.Lock()
        self._last_detection = FireDetection(False)
        self._async_thread: threading.Thread | None = None
        self._async_stop = threading.Event()
        self._frame_supplier = None
        self._poll_interval = 0.05

    def _ensure_vwriter(self, frame_shape):
        if not C.VIDEO_SAVE_PATH:
            return
        if self._vw is not None:
            return
        os.makedirs(os.path.dirname(C.VIDEO_SAVE_PATH) or ".", exist_ok=True)
        fourcc = cv2.VideoWriter_fourcc(*C.VIDEO_CODEC)
        h, w = frame_shape[:2]
        self._vw = cv2.VideoWriter(C.VIDEO_SAVE_PATH, fourcc, C.VIDEO_FPS, (w, h), True)

    def _annotate_and_show(self, frame_bgr, det: FireDetection):
        # Crosshair
        cx, cy = int(C.FRAME_W/2), int(C.FRAME_H/2)
        cv2.drawMarker(frame_bgr, (cx, cy), (0, 255, 255), markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)

        # Box + label
        if det.bbox is not None:
            x1, y1, x2, y2 = map(int, det.bbox)
            cv2.rectangle(frame_bgr, (x1, y1), (x2, y2), (0,0,255), 2)
            label = f"C.DETECT_CLASSES {det.conf:.2f} | dx={det.dx:.0f} dy={det.dy:.0f} area={det.area_frac:.2f}"
        else:
            label = 'no ' + C.DETECT_CLASSES
        cv2.putText(frame_bgr, label, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (40,40,40), 2)

        if self.show_video:
            cv2.imshow(self.window_name, frame_bgr)
            k = cv2.waitKey(1) & 0xFF
            if k in (27, ord('x'), ord('q')):   # ESC / x / q → close preview
                self.show_video = False
                try:
                    cv2.destroyWindow(self.window_name)
                except:
                    pass

        # Save annotated frame if requested
        if C.VIDEO_SAVE_PATH:
            self._ensure_vwriter(frame_bgr.shape)
            if self._vw is not None:
                self._vw.write(frame_bgr)

    def infer(self, frame) -> FireDetection:
        """
        Process one frame. Returns FireDetection.
        - If model is disabled or frame is None: show-only, returns has_fire=False.
        """
        if frame is None:
            det = FireDetection(False)
            self._update_last(det)
            return det

        if self._expect_rgb and isinstance(frame, np.ndarray) and frame.ndim == 3 and frame.shape[2] == 3:
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        else:
            frame_bgr = frame

        H, W = frame_bgr.shape[:2]
        # If frame size differs from config, don't resize forcibly; compute center from actual size
        cx, cy = W / 2.0, H / 2.0

        if (not self.enable_model) or (self.model is None):
            # show-only path
            det = FireDetection(False)
            self._annotate_and_show(frame_bgr.copy(), det)
            self._update_last(det)
            return det

        # Run YOLO
        results = self.model(frame_bgr, conf=C.DETECT_CONF, verbose=False)
        best = None  # (xyxy, class_name, conf)
        for r in results:
            if r.boxes is None:
                continue
            for b in r.boxes:
                cls_id = int(b.cls[0]) if b.cls is not None else -1
                name = self.model.names.get(cls_id, str(cls_id))
                conf = float(b.conf[0]) if b.conf is not None else 0.0
                if self.classes and (name not in self.classes):
                    continue
                xyxy = b.xyxy[0].tolist()  # [x1,y1,x2,y2]
                if (best is None) or (conf > best[2]):
                    best = (xyxy, name, conf)

        if best:
            x1, y1, x2, y2 = best[0]
            bx = (x1 + x2) * 0.5
            by = (y1 + y2) * 0.5
            dx = bx - cx      # + right, - left
            dy = by - cy      # + down,  - up
            area = max(1.0, (x2 - x1) * (y2 - y1))
            area_frac = area / float(W * H)
            conf = best[2]
            det = FireDetection(True, dx, dy, area_frac, conf, (int(x1), int(y1), int(x2), int(y2)))
            self.last_seen_ts = time.time()
        else:
            det = FireDetection(False)

        self._annotate_and_show(frame_bgr.copy(), det)
        self._update_last(det)
        return det

    # Convenience if some code wants offsets after last infer()
    def bbox_center_offset(self, det: FireDetection | None):
        if not det or not det.has_fire:
            return None
        return (det.dx, det.dy, det.area_frac, det.conf)

    def close(self):
        self.stop_async()
        if self._vw is not None:
            try:
                self._vw.release()
            except:
                pass
            self._vw = None
        try:
            if self.show_video:
                cv2.destroyWindow(self.window_name)
        except:
            pass

    def _update_last(self, det: FireDetection):
        with self._lock:
            self._last_detection = det

    def get_latest_detection(self, max_age: float | None = None) -> FireDetection | None:
        with self._lock:
            det = self._last_detection
        if det is None:
            return None
        if max_age is not None and (time.time() - det.ts) > max_age:
            return None
        return det

    def start_async(self, frame_supplier, poll_interval: float | None = None):
        if frame_supplier is None:
            return
        if self._async_thread and self._async_thread.is_alive():
            return
        hz = float(getattr(C, "ASYNC_FRAME_HZ", 0) or 0)
        base = poll_interval if poll_interval else (1.0 / hz if hz > 0 else 0.05)
        self._poll_interval = max(0.02, min(base, 0.5))
        self._frame_supplier = frame_supplier
        self._async_stop.clear()
        self._async_thread = threading.Thread(target=self._async_loop, name="fire-detector", daemon=True)
        self._async_thread.start()

    def pause_async(self):
        if not self._async_thread:
            return
        self._async_stop.set()
        self._async_thread.join(timeout=1.0)
        self._async_thread = None
        self._async_stop.clear()

    def resume_async(self):
        if self._frame_supplier is None:
            return
        self.start_async(self._frame_supplier, self._poll_interval)

    def stop_async(self):
        self.pause_async()
        self._frame_supplier = None

    def _async_loop(self):
        while not self._async_stop.is_set():
            frame = None
            try:
                frame = self._frame_supplier()
            except Exception:
                frame = None
            if frame is not None:
                try:
                    self.infer(frame)
                except Exception:
                    pass
            time.sleep(self._poll_interval)


def approach_once(tello, det: FireDetection) -> float:
    """
    Do ONE small alignment/approach step.
    Returns the yaw applied this call (degrees), where + = CCW(left), - = CW(right).
    Policy:
      1) If horizontal offset significant → yaw toward target CENTER_TOL_PX by APPROACH_YAW_STEP.
      2) Else if target not near enough (small area) → move forward APPROACH_FWD_CM.
      3) (Optional) could strafe/up-down using APPROACH_STRAFE_CM if you want to expand.
    """
    yaw_applied = 0.0
    if not det or not det.has_fire:
        return yaw_applied

    # 1) Yaw correction
    if abs(det.dx) > C.CENTER_TOL_PX:
        step = int(C.APPROACH_YAW_STEP)
        # dx > 0 means target RIGHT → rotate RIGHT (CW = negative signed yaw)
        yaw = -step if det.dx > 0 else +step
        if yaw > 0:
            tello.rotate_counter_clockwise(yaw)
        else:
            tello.rotate_clockwise(-yaw)
        time.sleep(C.TURN_SLEEP)
        yaw_applied += yaw
        return yaw_applied  # do one thing per call

    # 2) Forward approach if not near enough
    if det.area_frac < C.NEAR_AREA_FRAC:
        tello.move_forward(int(C.APPROACH_FWD_CM))
        time.sleep(C.MOVE_SLEEP)

    return yaw_applied