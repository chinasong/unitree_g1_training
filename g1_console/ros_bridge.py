"""Optional ROS 2 ingress. ROS imports stay outside the console's core runtime."""
import argparse
import base64
from collections import OrderedDict
import hashlib
from http.cookiejar import CookieJar
import json
import math
import os
import threading
import time
from urllib.request import build_opener, HTTPCookieProcessor, Request
from uuid import uuid4


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec / 1e9


def planar_transform(transform):
    """ROS quaternion xyzw; reject non-planar or invalid transforms."""
    q, t = transform.rotation, transform.translation
    norm = sum(v * v for v in (q.x, q.y, q.z, q.w))
    if not .99 <= norm <= 1.01 or not all(math.isfinite(v) for v in (t.x, t.y, t.z)):
        raise ValueError("invalid TF")
    if 1 - 2 * (q.x * q.x + q.y * q.y) < .98:
        raise ValueError("scan/map TF is not planar")
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return t.x, t.y, yaw


def scan_points(scan, transform):
    """Require near-full sweep, rather than claiming clearance outside a camera FOV."""
    n = len(scan.ranges)
    coverage = abs(scan.angle_increment) * (n - 1)
    if n < 30 or not 6.0 <= coverage <= 6.4 or not .5 <= scan.range_max:
        raise ValueError("need a 360-degree LaserScan")
    x, y, yaw = planar_transform(transform)
    points, valid = [], 0
    for i, value in enumerate(scan.ranges):
        if math.isnan(value) or value == -math.inf:
            continue
        if value < scan.range_min:
            raise ValueError("scan has below-minimum returns; clearance unknown")
        valid += 1
        if i % max(1, math.ceil(n / 1800)):
            continue
        distance = min(value, scan.range_max, 29.0)
        angle = scan.angle_min + i * scan.angle_increment + yaw
        points.append((x + distance * math.cos(angle), y + distance * math.sin(angle)))
    if valid / n < .98 or len(points) < 8:
        raise ValueError("scan has insufficient valid coverage")
    return points, min(coverage, 2 * math.pi)


def occupancy_grid(message):
    info = message.info
    q = info.origin.orientation
    if abs(q.x) + abs(q.y) + abs(q.z) > 1e-5 or abs(abs(q.w) - 1) > 1e-5:
        raise ValueError("rotated map origin is unsupported")
    if not 4 <= info.width <= 160 or not 4 <= info.height <= 160:
        raise ValueError("map must be 4..160 cells per side; provide a local map at adequate resolution")
    payload = dict(frame=message.header.frame_id, width=info.width, height=info.height,
                   resolution=info.resolution, origin=[info.origin.position.x, info.origin.position.y],
                   cells=list(message.data))
    # Content identity is used by the node to advance a monotonic map revision.
    digest = hashlib.sha256(json.dumps(payload, separators=(",", ":")).encode()).hexdigest()
    return payload, digest


class BridgeHTTP:
    def __init__(self, url):
        from urllib.parse import urlparse
        parsed = urlparse(url)
        if parsed.scheme not in ("http", "https") or parsed.path not in ("", "/"):
            raise ValueError("--url must be the console HTTP(S) origin")
        self.url = url.rstrip("/")
        self.headers = {"Content-Type": "application/json", "X-G1-Intent": "operator"}
        token = os.getenv("G1_API_TOKEN")
        self.http = build_opener(HTTPCookieProcessor(CookieJar()))
        if token:
            self.headers["Authorization"] = "Bearer " + token
        else:
            if parsed.hostname not in ("localhost", "127.0.0.1", "::1"):
                raise ValueError("remote console requires G1_API_TOKEN")
            self.http.open(self.url + "/api/session", timeout=2).close()
        self.pending = OrderedDict()
        self.lock, self.wake = threading.Lock(), threading.Event()
        self.closed = False
        self.last_error = 0.
        self.thread = threading.Thread(target=self.run, daemon=True, name="g1-sensor-http")
        self.thread.start()

    def submit(self, kind, data):
        with self.lock:
            self.pending[kind] = data  # Latest only; never replay a stale queue.
        self.wake.set()

    def run(self):
        while not self.closed:
            self.wake.wait(.1)
            self.wake.clear()
            while not self.closed:
                with self.lock:
                    item = self.pending.popitem(last=False) if self.pending else None
                if not item:
                    break
                kind, payload = item
                try:
                    request = Request(self.url + "/api/" + kind, json.dumps(payload, allow_nan=False).encode(), self.headers)
                    self.http.open(request, timeout=.4).close()
                except Exception as exc:
                    if time.monotonic() - self.last_error > 3:
                        # Never print request payloads, credentials or camera frames.
                        print(f"Bridge ingress rejected: {getattr(exc, 'code', type(exc).__name__)}", flush=True)
                        self.last_error = time.monotonic()

    def close(self):
        self.closed = True
        self.wake.set()
        self.thread.join(timeout=1)


def main():
    parser = argparse.ArgumentParser(description="G1 ROS 2 → 身体控制台；只读取传感器，不发布电机或 cmd_vel")
    parser.add_argument("--url", default="http://127.0.0.1:8765")
    parser.add_argument("--scan", default="/scan")
    parser.add_argument("--map", default="/map")
    parser.add_argument("--image", default="/camera/color/image_raw")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--base-frame", default="base_link")
    args = parser.parse_args()
    # Use the ROS distribution's Python interpreter (Humble commonly uses 3.10).
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from sensor_msgs.msg import LaserScan, Image
    from nav_msgs.msg import OccupancyGrid
    from tf2_ros import Buffer, TransformListener
    from cv_bridge import CvBridge
    import cv2

    class SensorBridge(Node):
        def __init__(self):
            super().__init__("g1_body_console_sensor_bridge")
            if self.get_parameter("use_sim_time").value:
                raise ValueError("real ingress requires wall-clock ROS timestamps")
            self.http = BridgeHTTP(args.url)
            self.tf = Buffer()
            self.listener = TransformListener(self.tf, self)
            self.cv = CvBridge()
            self.source, self.seq, self.camera_seq = "orin-" + uuid4().hex[:12], 0, 0
            self.grid, self.digest, self.revision = None, None, 0
            self.last_scan, self.last_image, self.last_warning = 0., 0., 0.
            self.create_subscription(LaserScan, args.scan, self.scan, qos_profile_sensor_data)
            map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(OccupancyGrid, args.map, self.map, map_qos)
            self.create_subscription(Image, args.image, self.image, qos_profile_sensor_data)

        def warn(self, exc):
            if time.monotonic() - self.last_warning > 3:
                self.get_logger().warning(f"等待有效传感器 / TF：{type(exc).__name__}: {str(exc)[:140]}")
                self.last_warning = time.monotonic()

        def map(self, message):
            try:
                grid, digest = occupancy_grid(message)
                if grid["frame"] != args.map_frame:
                    raise ValueError("map frame differs from --map-frame")
                if digest != self.digest:
                    self.revision += 1
                    self.grid, self.digest = dict(grid, revision=self.revision), digest
            except Exception as exc:
                self.grid = None
                self.warn(exc)

        def scan(self, message):
            if time.monotonic() - self.last_scan < .12:
                return
            self.last_scan = time.monotonic()
            try:
                captured = stamp_seconds(message.header.stamp)
                if not -.05 <= time.time() - captured <= .3 or not self.grid:
                    raise ValueError("stale scan or missing map")
                stamp = Time.from_msg(message.header.stamp)
                pose_tf = self.tf.lookup_transform(args.map_frame, args.base_frame, stamp)
                scan_tf = self.tf.lookup_transform(args.base_frame, message.header.frame_id, stamp)
                # Lookup at scan time must succeed; latest/stale TF is never substituted.
                x, y, yaw = planar_transform(pose_tf.transform)
                points, coverage = scan_points(message, scan_tf.transform)
                self.seq += 1
                self.http.submit("observation", dict(source=self.source, frame=args.map_frame,
                    sequence=self.seq, captured_at=captured, pose=dict(x=x, y=y, yaw=yaw),
                    scan=points, coverage_radians=coverage, grid=self.grid))
            except Exception as exc:
                self.warn(exc)

        def image(self, message):
            if time.monotonic() - self.last_image < .33:
                return
            self.last_image = time.monotonic()
            try:
                captured = stamp_seconds(message.header.stamp)
                if not -.05 <= time.time() - captured <= .3:
                    raise ValueError("stale camera timestamp")
                image = self.cv.imgmsg_to_cv2(message, desired_encoding="bgr8")
                h, w = image.shape[:2]
                if max(h, w) > 640:
                    image = cv2.resize(image, (round(w * 640 / max(h, w)), round(h * 640 / max(h, w))))
                ok, jpeg = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 75])
                if not ok or len(jpeg) > 500000:
                    raise ValueError("JPEG encoding failed")
                self.camera_seq += 1
                self.http.submit("camera", dict(source=self.source, sequence=self.camera_seq,
                    captured_at=captured, jpeg_base64=base64.b64encode(jpeg).decode()))
            except Exception as exc:
                self.warn(exc)

    rclpy.init()
    node = None
    try:
        node = SensorBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.http.close()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
