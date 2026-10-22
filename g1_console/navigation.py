"""Conservative 2-D navigation on an externally localized, surveyed level floor."""
import heapq
import math
import time
from .config import ConsoleError
from .contracts import Observation


class World:
    def __init__(self):
        self.observation = None
        self.grid = None
        self.received = 0.0
        self.sequence = -1
        self.source = None
        self.camera = None
        self.camera_at = 0.0
        self.camera_captured_at = 0.0
        self.camera_source = None
        self.camera_sequence = -1
        self.camera_metadata = {}

    def update(self, observation: Observation, now=None):
        now = time.time() if now is None else now
        if not -0.1 <= now - observation.captured_at <= 0.6:
            raise ConsoleError("STALE_OBSERVATION", "观察时间过期或来自未来；请同步 Orin 与传感器时钟")
        if self.source is not None and observation.source != self.source:
            raise ConsoleError("SOURCE_CONFLICT", "已有活动感知源；断开机器人后才能切换")
        if observation.sequence <= self.sequence:
            raise ConsoleError("REPLAYED_OBSERVATION", "拒绝重复或倒序观察")
        if observation.grid:
            if observation.grid.frame != observation.frame:
                raise ConsoleError("FRAME_MISMATCH", "定位和地图必须在同一坐标系")
            if self.grid and observation.grid.frame == self.grid.frame and observation.grid.revision < self.grid.revision:
                raise ConsoleError("OLD_MAP", "拒绝倒序地图版本")
        elif self.grid and self.grid.frame != observation.frame:
            raise ConsoleError("FRAME_MISMATCH", "定位坐标系与现有地图不一致")
        if any(math.hypot(x, y) > 30 for x, y in observation.scan):
            raise ConsoleError("SCAN_RANGE", "扫描点超出 30 米输入范围", 422)
        self.source, self.sequence = observation.source, observation.sequence
        self.observation, self.received = observation, time.monotonic()
        if observation.grid:
            self.grid = observation.grid

    def fresh(self, ttl):
        return (self.observation is not None and time.monotonic() - self.received < ttl
                and time.time() - self.observation.captured_at < ttl)

    def snapshot(self):
        obs = self.observation
        return dict(source=self.source, age_ms=round((time.monotonic() - self.received) * 1000) if obs else None,
                    frame=obs.frame if obs else None, pose=obs.pose.model_dump() if obs else None,
                    scan=obs.scan if obs else [], grid=self.grid.model_dump() if self.grid else None,
                    camera_age_ms=round(self.camera_age() * 1000) if self.camera else None)

    def camera_age(self):
        return max(time.monotonic() - self.camera_at, time.time() - self.camera_captured_at)


def inflate(grid, radius):
    """Unknown and off-map space are occupied; inflate a circular body footprint."""
    width, height, resolution = grid.width, grid.height, grid.resolution
    pad = math.ceil(radius / resolution)
    offsets = [(dx, dy) for dx in range(-pad, pad + 1) for dy in range(-pad, pad + 1)
               if math.hypot(dx, dy) * resolution <= radius + resolution * 0.71]
    blocked = set()
    for y in range(height):
        for x in range(width):
            if grid.cells[y * width + x] < 0 or grid.cells[y * width + x] >= 50:
                blocked.update((x + dx, y + dy) for dx, dy in offsets)
            if min(x + 0.5, y + 0.5, width - x - 0.5, height - y - 0.5) * resolution < radius:
                blocked.add((x, y))
    return blocked


def plan(grid, start, goal, radius):
    def cell(p):
        return (math.floor((p[0] - grid.origin[0]) / grid.resolution),
                math.floor((p[1] - grid.origin[1]) / grid.resolution))

    s, g = cell(start), cell(goal)
    blocked = inflate(grid, radius)

    def valid(p):
        return 0 <= p[0] < grid.width and 0 <= p[1] < grid.height and p not in blocked

    if not valid(s) or not valid(g):
        raise ConsoleError("UNSAFE_GOAL", "起点或目标位于障碍物、未知区域或身体净空不足的区域")
    frontier, costs, came = [(0, s)], {s: 0}, {}
    while frontier:
        _, current = heapq.heappop(frontier)
        if current == g:
            path = [g]
            while path[-1] != s:
                path.append(came[path[-1]])
            path.reverse()
            return [(grid.origin[0] + (x + 0.5) * grid.resolution,
                     grid.origin[1] + (y + 0.5) * grid.resolution) for x, y in path]
        x, y = current
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, -1), (1, -1), (-1, 1)):
            nxt = x + dx, y + dy
            if not valid(nxt) or (dx and dy and (not valid((x + dx, y)) or not valid((x, y + dy)))):
                continue
            candidate = costs[current] + math.hypot(dx, dy)
            if candidate < costs.get(nxt, math.inf):
                costs[nxt], came[nxt] = candidate, current
                heapq.heappush(frontier, (candidate + math.dist(nxt, g), nxt))
    raise ConsoleError("NO_PATH", "已知可通行区域内没有满足身体净空的路线")


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def local_clear(scan, vx, vy, omega, radius):
    """Check swept local footprint plus braking margin, rather than front ray only."""
    speed = math.hypot(vx, vy)
    for x, y in scan:
        if math.hypot(x, y) < radius + 0.10 and (speed > 0 or abs(omega) > 0):
            return False
        if speed > 0:
            along, across = (x * vx + y * vy) / speed, abs(x * vy - y * vx) / speed
            if -radius < along < radius + 0.18 + speed * 1.0 and across < radius + 0.08:
                return False
    return True
