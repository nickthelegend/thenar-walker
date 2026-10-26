"""The robot's camera, for the brain (Hasami: SAM + Muse). `GET /snap.jpg` on the robot returns one JPEG.

Picamera2 (CSI camera module) when it is installed and a camera answers; otherwise OpenCV on a USB webcam
(THENAR_CAMERA=<index>, default 0); otherwise no camera, and /snap.jpg answers 503 "Setup required: no camera".
A frame is captured only when asked for, so an idle robot spends no CPU on video.
"""
from __future__ import annotations

import logging
import os
import threading

log = logging.getLogger("thenar_pi.camera")


class Camera:
    def __init__(self):
        self.kind = "none"
        self._lock = threading.Lock()
        self._pc = self._cv = None
        if os.environ.get("THENAR_CAMERA", "") == "off":
            return
        try:
            from picamera2 import Picamera2
            pc = Picamera2()
            pc.configure(pc.create_still_configuration(main={"size": (1280, 960), "format": "RGB888"}))
            pc.start()
            self._pc, self.kind = pc, "picamera2"
        except Exception as e:
            log.info("no CSI camera (%s)", type(e).__name__)
        if self._pc is None:
            try:
                import cv2
                cap = cv2.VideoCapture(int(os.environ.get("THENAR_CAMERA", "0") or 0))
                if cap.isOpened():
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
                    self._cv, self.kind = cap, "usb"
                else:
                    cap.release()
            except Exception as e:
                log.info("no USB camera (%s)", type(e).__name__)
        log.info("camera: %s", self.kind)

    def jpeg(self, quality: int = 85) -> bytes | None:
        with self._lock:
            try:
                import cv2
                if self._pc is not None:
                    frame = self._pc.capture_array()   # RGB888 in Picamera2 = BGR byte order for OpenCV
                elif self._cv is not None:
                    for _ in range(2):   # drop the stale buffered frame
                        self._cv.grab()
                    ok, frame = self._cv.read()
                    if not ok:
                        return None
                else:
                    return None
                ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
                return buf.tobytes() if ok else None
            except Exception as e:
                log.warning("camera capture failed: %s", e)
                return None

    def close(self) -> None:
        if self._pc is not None:
            self._pc.stop()
        if self._cv is not None:
            self._cv.release()
