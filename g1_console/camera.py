"""Read the existing official video service; never open or reconfigure a camera device."""
import threading
import time
from collections import deque
from uuid import uuid4
from .camera_rpc import ReadOnlyVideoClient


class OfficialCamera:
    def __init__(self, width=960, fps=5):
        self.width, self.period = width, 1 / fps
        self.source = "unitree-videohub-" + uuid4().hex[:12]
        self.lock, self.stop = threading.Lock(), threading.Event()
        self.frame, self.status, self.error = None, "starting", None
        self.attempts = deque(maxlen=512)
        self.total_attempts = self.total_successes = self.total_timeouts = 0
        self.started = time.monotonic()
        self.thread = threading.Thread(target=self._run, daemon=True, name="g1-camera-readonly")
        self.thread.start()

    def _run(self):
        # DDS is already initialized by UnitreeAdapter on the selected robot interface.
        try:
            import cv2
            import numpy as np
            client = ReadOnlyVideoClient(timeout=.7)
        except Exception as exc:
            with self.lock:
                self.status, self.error = "unavailable", type(exc).__name__
            return
        sequence = 0
        while not self.stop.is_set():
            started, wall = time.monotonic(), time.time()
            result = None
            try:
                # Only API 1001 GetImageSample. No lease, motor, audio or device writes.
                code, data = client.GetImageSample()
                result = code
                if code != 0:
                    raise ValueError(f"SDK_{code}")
                raw = bytes(data)
                if not 4 <= len(raw) <= 4_000_000 or not raw.startswith(b"\xff\xd8"):
                    raise ValueError("INVALID_IMAGE")
                image = cv2.imdecode(np.frombuffer(raw, dtype=np.uint8), cv2.IMREAD_COLOR)
                if image is None:
                    raise ValueError("INVALID_IMAGE")
                source_height, source_width = image.shape[:2]
                if source_width > self.width:
                    image = cv2.resize(image, (self.width, round(source_height * self.width / source_width)), interpolation=cv2.INTER_AREA)
                ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 78])
                if not ok or len(encoded) > 500000:
                    raise ValueError("IMAGE_TOO_LARGE")
                sequence += 1
                metadata = dict(source=self.source, sequence=sequence, request_started_at=wall,
                    received_at=time.time(), width=image.shape[1], height=image.shape[0],
                    source_width=source_width, source_height=source_height,
                    request_ms=round((time.monotonic() - started) * 1000),
                    timestamp_basis="request_started_at", sensor_timestamp=None)
                with self.lock:
                    if not self.stop.is_set():
                        self.frame = (bytes(encoded), started, wall, metadata)
                        self.status, self.error = "receiving", None
            except Exception as exc:
                with self.lock:
                    self.status = "retrying"
                    self.error = str(exc) if isinstance(exc, ValueError) else type(exc).__name__
                result = self.error
            with self.lock:
                result = 0 if self.error is None else result
                self.attempts.append((time.monotonic(), result, (time.monotonic()-started)*1000))
                self.total_attempts += 1
                self.total_successes += int(result == 0)
                self.total_timeouts += int(result == "SDK_3104")
            self.stop.wait(max(.01, self.period - (time.monotonic() - started)) if self.error is None else .8)
        client.close()
        with self.lock:
            self.status = "stopped"

    def latest(self):
        with self.lock:
            return self.frame, self.status, self.error

    def diagnostics(self):
        now = time.monotonic()
        with self.lock:
            rows = [r for r in self.attempts if now-r[0] < 60]
            return dict(transport="official_api_1001_polled", window_s=round(min(60, now-self.started), 1),
                attempts=len(rows), successes=sum(r[1] == 0 for r in rows),
                timeouts=sum(r[1] == "SDK_3104" for r in rows),
                errors=sum(r[1] != 0 for r in rows), total_attempts=self.total_attempts,
                total_successes=self.total_successes, total_timeouts=self.total_timeouts)

    def close(self):
        self.stop.set()
        self.thread.join(timeout=1.5)  # The SDK read itself has a 700 ms deadline.
