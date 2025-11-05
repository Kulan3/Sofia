#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Simplified fire detection helper compatible with Python 3.8.
Only performs YOLO inference and optional frame display/recording.
"""
import os
import sys
import threading
import time
from pathlib import Path
from typing import Optional, Tuple, Callable

JETSON_DIST_PACKAGES = "/usr/lib/python3/dist-packages"
if JETSON_DIST_PACKAGES not in sys.path and os.path.isdir(JETSON_DIST_PACKAGES):
    sys.path.insert(0, JETSON_DIST_PACKAGES)

try:
    import cv2  # type: ignore
    if not hasattr(cv2, "_registerMatType"):
        raise AttributeError("_registerMatType missing")
except AttributeError as exc:
    if "_registerMatType" in str(exc):
        import importlib

        sys.modules.pop("cv2", None)
        cv2 = importlib.import_module("cv2.cv2")  # type: ignore
        if not hasattr(cv2, "_registerMatType"):
            raise
    else:
        raise
except ModuleNotFoundError:
    raise

import numpy as np
from ultralytics import YOLO

import beta_config as C


class FireDetection:
    """Container for a single-frame detection result."""

    def __init__(
        self,
        has_fire: bool,
        dx: float = 0.0,
        dy: float = 0.0,
        area_frac: float = 0.0,
        conf: float = 0.0,
        bbox: Optional[Tuple[int, int, int, int]] = None,
        timestamp: Optional[float] = None,
        label: Optional[str] = None,
    ) -> None:
        self.has_fire = bool(has_fire)
        self.dx = float(dx)
        self.dy = float(dy)
        self.area_frac = float(area_frac)
        self.conf = float(conf)
        self.bbox = bbox
        self.ts = float(time.time() if timestamp is None else timestamp)
        self.label = label


class FireDetector:
    """
    YOLO-based detector with optional live preview/recording.
    No extra decorations are drawn; annotated frames come directly from YOLO.
    """

    def __init__(self, enable_model: bool = True, show_video: Optional[bool] = None) -> None:
        self.enable_model = bool(enable_model)
        self.show_video = C.SHOW_VIDEO if show_video is None else bool(show_video)
        self._expect_rgb = bool(getattr(C, "TELLO_FRAME_RGB", False))
        self.window_name = "Tello Live"
        self._window_created = False
        self._window_failed = False

        self.model = None
        if self.enable_model:
            model_path = Path(C.YOLO_MODEL_PATH)
            if not model_path.exists():
                raise FileNotFoundError("YOLO model not found at {}".format(model_path))
            self.model = YOLO(str(model_path), task="detect")

        self.classes = {str(label).lower() for label in C.DETECT_CLASSES} if C.DETECT_CLASSES else None
        self.last_seen_ts = 0.0

        self._video_writer = None
        self._last_detection = FireDetection(False)
        self._lock = threading.Lock()

        self._async_thread = None  # type: Optional[threading.Thread]
        self._async_stop = threading.Event()
        self._frame_supplier = None  # type: Optional[Callable[[], Optional[np.ndarray]]]
        self._poll_interval = 0.05

        if self.show_video:
            self._try_create_window()

    # ------------------------------------------------------------------ #
    # Internal helpers
    # ------------------------------------------------------------------ #
    def _ensure_vwriter(self, frame_shape) -> None:
        if not C.VIDEO_SAVE_PATH:
            return
        if self._video_writer is not None:
            return
        os.makedirs(os.path.dirname(C.VIDEO_SAVE_PATH) or ".", exist_ok=True)
        fourcc = cv2.VideoWriter_fourcc(*C.VIDEO_CODEC)
        height, width = frame_shape[:2]
        self._video_writer = cv2.VideoWriter(C.VIDEO_SAVE_PATH, fourcc, C.VIDEO_FPS, (width, height), True)

    def _resolve_window_flags(self) -> int:
        flag = getattr(C, "WINDOW_FLAGS", "normal")
        if isinstance(flag, str):
            key = flag.strip().upper()
            mapping = {
                "NORMAL": cv2.WINDOW_NORMAL,
                "AUTOSIZE": cv2.WINDOW_AUTOSIZE,
                "FULLSCREEN": cv2.WINDOW_FULLSCREEN,
                "OPENGL": cv2.WINDOW_OPENGL,
            }
            return mapping.get(key, cv2.WINDOW_NORMAL)
        if isinstance(flag, int):
            return flag
        return cv2.WINDOW_NORMAL

    def _try_create_window(self) -> bool:
        if self._window_created:
            return True
        if self._window_failed or not self.show_video:
            return False
        try:
            if getattr(C, "FORCE_WINDOW_THREAD", False):
                try:
                    cv2.startWindowThread()
                except Exception:
                    pass
            cv2.namedWindow(self.window_name, self._resolve_window_flags())
            resize = getattr(C, "WINDOW_RESIZE", None)
            if isinstance(resize, (tuple, list)) and len(resize) == 2:
                try:
                    width, height = int(resize[0]), int(resize[1])
                    if width > 0 and height > 0:
                        cv2.resizeWindow(self.window_name, width, height)
                except Exception:
                    pass
            self._window_created = True
            return True
        except Exception as exc:
            print(f"[!] Unable to create OpenCV window ({exc}); disabling preview.")
            self.show_video = False
            self._window_failed = True
            return False

    def _display_frame(self, frame_bgr: np.ndarray) -> None:
        if self.show_video and self._try_create_window():
            cv2.imshow(self.window_name, frame_bgr)
            delay = int(max(1, getattr(C, "WAITKEY_DELAY_MS", 1)))
            key = cv2.waitKey(delay) & 0xFF
            if key in (27, ord("x"), ord("q")):
                self.show_video = False
                try:
                    cv2.destroyWindow(self.window_name)
                except Exception:
                    pass
        if C.VIDEO_SAVE_PATH:
            self._ensure_vwriter(frame_bgr.shape)
            if self._video_writer is not None:
                self._video_writer.write(frame_bgr)

    def _update_last(self, det: FireDetection) -> None:
        with self._lock:
            self._last_detection = det

    # ------------------------------------------------------------------ #
    # Public API
    # ------------------------------------------------------------------ #
    def infer(self, frame: Optional[np.ndarray]) -> FireDetection:
        """Run detection on a single frame."""
        if frame is None:
            detection = FireDetection(False)
            self._update_last(detection)
            return detection

        if self._expect_rgb:
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        else:
            frame_bgr = frame.copy()

        annotated_bgr = frame_bgr
        detection = FireDetection(False)

        if not self.enable_model or self.model is None:
            self._display_frame(frame_bgr.copy())
            self._update_last(detection)
            return detection

        results = self.model(frame_bgr, conf=C.DETECT_CONF, verbose=False)
        best = None
        height, width = frame_bgr.shape[:2]
        center_x, center_y = width / 2.0, height / 2.0

        for result in results:
            boxes = getattr(result, "boxes", None)
            if boxes is None:
                continue
            for box in boxes:
                cls_id = int(box.cls[0]) if box.cls is not None else -1
                name = self.model.names.get(cls_id, str(cls_id))
                name_norm = str(name).lower()
                if self.classes and name_norm not in self.classes:
                    continue
                conf = float(box.conf[0]) if box.conf is not None else 0.0
                coords = box.xyxy[0].tolist()
                if best is None or conf > best[2]:
                    best = (coords, name_norm, conf)

        if best is not None:
            x1, y1, x2, y2 = best[0]
            box_center_x = (x1 + x2) * 0.5
            box_center_y = (y1 + y2) * 0.5
            dx = box_center_x - center_x
            dy = box_center_y - center_y
            area = max(1.0, (x2 - x1) * (y2 - y1))
            area_frac = area / float(width * height)
            detection = FireDetection(
                True,
                dx=dx,
                dy=dy,
                area_frac=area_frac,
                conf=best[2],
                bbox=(int(x1), int(y1), int(x2), int(y2)),
                label=best[1],
            )
            self.last_seen_ts = time.time()

        if results:
            try:
                annotated = results[0].plot()  # returns annotated frame (BGR)
                if isinstance(annotated, np.ndarray):
                    annotated_bgr = annotated
            except Exception:
                annotated_bgr = frame_bgr

        self._display_frame(annotated_bgr)
        self._update_last(detection)
        return detection

    def get_latest_detection(self, max_age: Optional[float] = None) -> Optional[FireDetection]:
        with self._lock:
            det = self._last_detection
        if max_age is not None and (time.time() - det.ts) > max_age:
            return None
        return det

    def start_async(self, frame_supplier: Callable[[], Optional[np.ndarray]], poll_interval: Optional[float] = None) -> None:
        if frame_supplier is None:
            return
        if self._async_thread and self._async_thread.is_alive():
            return
        self._frame_supplier = frame_supplier
        if poll_interval is not None:
            self._poll_interval = max(0.02, min(poll_interval, 0.5))
        self._async_stop.clear()
        self._async_thread = threading.Thread(target=self._async_loop, name="fire-detector", daemon=True)
        self._async_thread.start()

    def pause_async(self) -> None:
        if not self._async_thread:
            return
        self._async_stop.set()
        self._async_thread.join(timeout=1.0)
        self._async_thread = None
        self._async_stop.clear()

    def resume_async(self) -> None:
        if not self._frame_supplier:
            return
        self.start_async(self._frame_supplier, self._poll_interval)

    def stop_async(self) -> None:
        self.pause_async()
        self._frame_supplier = None

    def close(self) -> None:
        self.stop_async()
        if self._video_writer is not None:
            try:
                self._video_writer.release()
            except Exception:
                pass
            self._video_writer = None
        if self._window_created:
            try:
                cv2.destroyWindow(self.window_name)
            except Exception:
                pass
            self._window_created = False

    # ------------------------------------------------------------------ #
    # Background polling
    # ------------------------------------------------------------------ #
    def _async_loop(self) -> None:
        while not self._async_stop.is_set():
            frame = None
            if self._frame_supplier is not None:
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


def main() -> int:
    """Standalone preview similar to the sample code."""
    drone = Tello()  # type: ignore
    drone.connect()
    print("Battery:", drone.get_battery())
    drone.streamon()

    frame_reader = drone.get_frame_read(with_queue=False, max_queue_len=0)
    detector = FireDetector(enable_model=True, show_video=True)

    try:
        while True:
            frame = frame_reader.frame
            detector.infer(frame)
    except KeyboardInterrupt:
        pass
    finally:
        detector.close()
        drone.streamoff()
        drone.end()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    from djitellopy import Tello  # avoid dependency

    raise SystemExit(main())
