"""Analytic room + kinematic demo. Not a dynamics simulation or real-robot proof."""
import math
import time
from .contracts import Grid, Observation, Pose

WIDTH = 60
RESOLUTION = .1
CELLS = [100 if x in (0, 59) or y in (0, 59) or (27 <= x <= 32 and 27 <= y <= 32)
         or (40 <= x <= 46 and 17 <= y <= 22) else 0 for y in range(WIDTH) for x in range(WIDTH)]


def observe(pose, sequence, include_grid=False):
    x, y, yaw = pose
    points = []
    for i in range(120):
        angle = i / 120 * 2 * math.pi
        for step in range(1, 91):
            distance = step * .06
            wx, wy = x + math.cos(yaw + angle) * distance, y + math.sin(yaw + angle) * distance
            cx, cy = math.floor((wx + 3) / .1), math.floor((wy + 3) / .1)
            if not (0 <= cx < 60 and 0 <= cy < 60) or CELLS[cy * 60 + cx] >= 50:
                points.append((math.cos(angle) * distance, math.sin(angle) * distance))
                break
    grid = Grid(frame="demo-map", width=60, height=60, resolution=.1, origin=(-3, -3), cells=CELLS, revision=1) if include_grid else None
    return Observation(source="synthetic-demo", frame="demo-map", sequence=sequence, captured_at=time.time(),
                       pose=Pose(x=x, y=y, yaw=yaw), scan=points, coverage_radians=2 * math.pi, grid=grid)
