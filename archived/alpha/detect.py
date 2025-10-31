#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from dataclasses import dataclass
import time
import cv2
from ultralytics import YOLO
from djitellopy import Tello
import config as C

@dataclass
class FireDetection:
    has_fire: bool
    dx: float = 0.0          # + right of center, - left (pixels)
    dy: float = 0.0          # + below center,  - above (pixels)
    area_frac: float = 0.0   # bbox area / frame area (0..1)
    conf: float = 0.0

class FireDetector:
    """
    Simple synchronous detector: call infer(frame) -> FireDetection.
    """
    def __init__(self):
        self.model = YOLO(C.YOLO_MODEL_PATH, task="detect")
        self.cls_whitelist = set(C.DETECT_CLASSES) if C.DETECT_CLASSES else None
        self._last_ok_ts = 0.0  # ms since epoch when last valid detection happened

    def infer(self, frame) -> FireDetection:
        """Run the model on a single frame and return a FireDetection."""
        if frame is None:
            return FireDetection(False)

        h, w = frame.shape[:2]
        results = self.model(frame, conf=C.DETECT_CONF, verbose=False)

        best = None  # (xyxy, name, conf)
        for r in results:
            if r.boxes is None:
                continue
            for b in r.boxes:
                cls_id = int(b.cls[0]) if b.cls is not None else -1
                name = self.model.names.get(cls_id, str(cls_id))
                conf = float(b.conf[0]) if b.conf is not None else 0.0
                if self.cls_whitelist and (name not in self.cls_whitelist):
                    continue
                xyxy = b.xyxy[0].tolist()  # [x1,y1,x2,y2]
                if (best is None) or (conf > best[2]):
                    best = (xyxy, name, conf)

        if not best:
            # decide "present" using persist window
            if (time.time() * 1000.0) - self._last_ok_ts <= C.FIRE_PERSIST_MS:
                # still considered present in the short persist window
                return FireDetection(True, 0.0, 0.0, 0.0, 0.0)
            return FireDetection(False)

        (x1, y1, x2, y2), name, conf = best
        x1, y1, x2, y2 = map(float, (x1, y1, x2, y2))
        cx = 0.5 * (x1 + x2)
        cy = 0.5 * (y1 + y2)
        dx = cx - (w / 2.0)
        dy = cy - (h / 2.0)
        area = max(1.0, (x2 - x1) * (y2 - y1))
        area_frac = area / float(w * h)

        self._last_ok_ts = time.time() * 1000.0
        return FireDetection(True, dx=dx, dy=dy, area_frac=area_frac, conf=conf)

def approach_once(t: Tello, det: FireDetection):
    """
    One small alignment/approach step toward the detected target.
    Uses config thresholds:
      - CENTER_TOL_PX, NEAR_AREA_FRAC
      - APPROACH_YAW_STEP, APPROACH_STRAFE_CM, APPROACH_FWD_CM
      - TURN_SLEEP, MOVE_SLEEP
    Positive dx means target is on the RIGHT side of the frame.
    """
    if not det.has_fire:
        return

    # 1) Yaw toward the target if it’s off-center horizontally
    if abs(det.dx) > C.CENTER_TOL_PX:
        # dx>0 -> target right -> yaw RIGHT (clockwise / negative angle in our convention)
        try:
            a = int(abs(C.APPROACH_YAW_STEP))
            if det.dx > 0:
                t.rotate_clockwise(a)
            else:
                t.rotate_counter_clockwise(a)
            time.sleep(C.TURN_SLEEP)
        except Exception as e:
            print(f"[!] yaw error: {e}")

    # 2) (Optional) strafe if still very off-center
    if abs(det.dx) > (1.5 * C.CENTER_TOL_PX) and C.APPROACH_STRAFE_CM > 0:
        try:
            d = int(C.APPROACH_STRAFE_CM)
            if det.dx > 0:
                t.move_right(d)
            else:
                t.move_left(d)
            time.sleep(C.MOVE_SLEEP)
        except Exception as e:
            print(f"[!] strafe error: {e}")

    # 3) Move forward until “near enough” (by area)
    if det.area_frac < C.NEAR_AREA_FRAC:
        try:
            t.move_forward(int(C.APPROACH_FWD_CM))
            time.sleep(C.MOVE_SLEEP)
        except Exception as e:
            print(f"[!] forward error: {e}")
