"""Explore the classroom: build a 2D grid of what the drone sees, stream it to the map and fly back to take-off.

At each stop the drone takes three photos (left, ahead, right), adds them to the grid and picks the
next move on the grid: right only when a whole step is clear there, otherwise ahead, otherwise left,
always towards floor it has not seen yet. A fresh photo confirms the move before it flies. When no
reachable floor is left unseen, it returns to the take-off pad along an A* path over seen floor.

Run with --synthetic it flies a stand-in drone inside the measured classroom (docs/classroom.md),
seeing by casting rays instead of taking photos, so the logic can be checked without a drone.
"""

import argparse
import heapq
import json
import math
import socket
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

from drone_interface import DroneInterface, TelloDrone, UnityDrone
from find_free_space import (
    DRONE_RADIUS_CM, MAX_FORWARD_CM, MAX_RANGE_CM, MIN_FORWARD_CM, RAY_STEP_CM, SAFETY_MARGIN_CM, STEP_LIMIT_CM,
    STOP_OBSTACLE, STOP_RANGE, SWEEP_STEP_DEG, allowed_distances, drone_safe_area, free_polygon_from_drone,
)
from free_space_fast import FreeSpace, analyze_frame
from image_io import write_image

CALIBRATION_FILE = Path("calibration/camera_calibration.npz")
FLIGHTS_DIR = Path("flights")
OUTPUT_DIR = Path("explorer")
MAP_PORT = 5005

# The same size as the map's voxels, so the brain and Unity carve the room at the same resolution.
CELL_CM = 10.0
# 12 x 12 m around take-off: the grid must not depend on the room it explores, and this holds any classroom.
GRID_HALF_SIZE_CM = 600.0
UNKNOWN, FREE, OBSTACLE = 0, 1, 2

# Just under the 54.5 degree field of view, so the three photos of a stop leave no gap between them.
SWEEP_TURN_DEG = 54
# Candidate headings for the next move, relative to the heading the stop started with.
HEADING_STEP_DEG = 5
MAX_HEADING_DEG = 80
# Headings beyond this from straight ahead count as a turn to the right or to the left.
SIDE_SECTOR_DEG = 30
# Less new floor than one 60 cm tile is not worth a stop; it also ignores stray unknown cells along walls.
MIN_NEW_CELLS = 36
# Rays of the "what would a photo from here see" estimate. It only needs to count cells roughly,
# and it runs for every candidate heading and frontier spot.
NEW_AREA_RAY_STEP_DEG = 2.5
# A safety cap: the measured classroom takes about 15 stops, so reaching this means the planner is looping.
MAX_STOPS = 100
MAX_CONFIRMATIONS = 3
# A move or frontier target ending within one 60 cm tile of an earlier stop is skipped: that spot was already
# swept, and returning to it only makes the drone shuttle between two stops.
REVISIT_CM = 60.0
FRONTIER_HEADINGS_DEG = range(0, 360, 45)
# The Tello needs some margin to land and to fly back; below this it returns to take-off at once.
LOW_BATTERY_PCT = 20

# Rough Tello speeds, only used to time the synthetic flight so its recording replays at a real pace.
SYNTHETIC_FORWARD_SPEED_CM_S = 50.0
SYNTHETIC_YAW_SPEED_DEG_S = 90.0
SYNTHETIC_PHOTO_TIME_S = 1.0
# The simulator's take-off height (docs/simulator.md).
SYNTHETIC_HEIGHT_CM = 80.0
# The Tello's camera looks down this much in flight (measured on the tiles in the first flight's photo), which
# brings the blind zone to about 1.25 m instead of the 2 m of a level camera.
SYNTHETIC_CAMERA_TILT_DEG = 11.0
# docs/classroom.md: free floor of the measured classroom and the take-off pad, in the room frame.
CLASSROOM_FLOOR_CM = [(0, 0), (120, 0), (120, 310), (370, 310), (370, 0), (460, 0), (460, 560), (0, 560)]
CLASSROOM_TAKEOFF_CM = (60, 60)

TOP_VIEW_PX_PER_CELL = 4
UNKNOWN_BGR = (150, 150, 150)
GIVEN_UP_BGR = (110, 110, 110)
FREE_BGR = (255, 255, 255)
SAFE_BGR = (200, 240, 200)
OBSTACLE_BGR = (0, 0, 0)
TRAIL_BGR = (255, 120, 0)
DRONE_BGR = (0, 0, 255)
CHOICE_BGR = (0, 170, 0)
ROOM_BGR = (0, 140, 255)


def wrap_deg(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


def direction(heading_deg: float) -> np.ndarray:
    """Unit vector of a heading measured clockwise from +y, as yaw_deg is defined in docs/protocol.md."""
    radians = math.radians(heading_deg)
    return np.array([math.sin(radians), math.cos(radians)])


def camera_fov(camera_matrix: np.ndarray, size: tuple[int, int] = (960, 720)) -> dict[str, float]:
    """Field of view on each side of the optical axis, from the calibrated intrinsics."""
    fx, fy, cx, cy = camera_matrix[0, 0], camera_matrix[1, 1], camera_matrix[0, 2], camera_matrix[1, 2]
    width, height = size
    return {
        "left_deg": math.degrees(math.atan(cx / fx)),
        "right_deg": math.degrees(math.atan((width - cx) / fx)),
        "up_deg": math.degrees(math.atan(cy / fy)),
        "down_deg": math.degrees(math.atan((height - cy) / fy)),
    }


def trusted_rays(free_space: FreeSpace) -> dict[str, np.ndarray]:
    """Cut every ray at the trusted range of docs/protocol.md: the blind zone plus one step."""
    limit = free_space.entry_cm + STEP_LIMIT_CM
    cut = free_space.stop_cm > limit
    return {
        "angle_deg": free_space.angle_deg,
        "entry_cm": free_space.entry_cm,
        "stop_cm": np.where(cut, limit, free_space.stop_cm),
        "kind": np.where(cut, STOP_RANGE, free_space.kind),
    }


class Grid:
    """What the drone knows about the floor, in cells of the map frame. Unseen floor counts as blocked."""

    def __init__(self) -> None:
        size = int(2 * GRID_HALF_SIZE_CM / CELL_CM)
        self.cells = np.full((size, size), UNKNOWN, np.uint8)
        self.trail = np.zeros((size, size), bool)
        self.given_up = np.zeros((size, size), bool)

    def index(self, points_cm: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Return the rows, columns and in-grid mask of points given as (x, y) in cm."""
        points_cm = np.atleast_2d(points_cm)
        columns = np.floor((points_cm[:, 0] + GRID_HALF_SIZE_CM) / CELL_CM).astype(int)
        rows = np.floor((points_cm[:, 1] + GRID_HALF_SIZE_CM) / CELL_CM).astype(int)
        inside = (rows >= 0) & (rows < self.cells.shape[0]) & (columns >= 0) & (columns < self.cells.shape[1])
        return np.clip(rows, 0, self.cells.shape[0] - 1), np.clip(columns, 0, self.cells.shape[1] - 1), inside

    def centre(self, row: int, column: int) -> np.ndarray:
        return (np.array([column, row]) + 0.5) * CELL_CM - GRID_HALF_SIZE_CM

    def add_observation(self, position: np.ndarray, yaw_deg: float, rays: dict[str, np.ndarray]) -> None:
        for angle, entry, stop, kind in zip(rays["angle_deg"], rays["entry_cm"], rays["stop_cm"], rays["kind"]):
            # The blind zone is clear only if the camera saw floor at the entry (occlusion, as in
            # allowed_distances). A ray stopped at its entry saw something closer, so nothing is known.
            if stop <= entry:
                continue
            unit = direction(yaw_deg + angle)
            rows, columns, inside = self.index(position + np.arange(0.0, stop, CELL_CM / 2)[:, None] * unit)
            rows, columns = rows[inside], columns[inside]
            # Obstacles win over free floor: a cell holding a wall is blocked even if a ray grazed it.
            keep = self.cells[rows, columns] != OBSTACLE
            self.cells[rows[keep], columns[keep]] = FREE
            if kind == STOP_OBSTACLE:
                # stop_cm is the last free floor; the obstacle's foot is one ray step further.
                row, column, inside = self.index(position + (stop + RAY_STEP_CM) * unit)
                if inside[0]:
                    self.cells[row[0], column[0]] = OBSTACLE

    def add_trail(self, start: np.ndarray, end: np.ndarray) -> None:
        """Mark the cells the drone flew through; its own body proves them free."""
        length = np.linalg.norm(end - start)
        points = start + np.linspace(0.0, 1.0, max(2, int(length / (CELL_CM / 2)) + 1))[:, None] * (end - start)
        rows, columns, inside = self.index(points)
        self.trail[rows[inside], columns[inside]] = True
        self.cells[rows[inside], columns[inside]] = FREE

    def safe_area(self) -> np.ndarray:
        """Cells where the drone's centre can be: seen free floor eroded by the drone's radius plus margin."""
        radius = int(math.ceil((DRONE_RADIUS_CM + SAFETY_MARGIN_CM) / CELL_CM))
        disc = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * radius + 1, 2 * radius + 1))
        return cv2.erode((self.cells == FREE).astype(np.uint8), disc) > 0

    def passable(self, safe: np.ndarray, position: np.ndarray) -> np.ndarray:
        """Where a path may go: safe cells, the trail, where the drone's centre has already flown without
        hitting anything, and the seen-free floor around the drone, which it may need to cross first."""
        rows, columns, _ = self.index(position)
        radius = int(math.ceil((DRONE_RADIUS_CM + SAFETY_MARGIN_CM) / CELL_CM)) + 1
        near = np.zeros_like(safe)
        cv2.circle(near.view(np.uint8), (int(columns[0]), int(rows[0])), radius, 1, -1)
        return safe | self.trail | (near & (self.cells == FREE))

    def clear_distance(self, safe: np.ndarray, position: np.ndarray, heading_deg: float) -> float:
        """How far the drone can fly straight along a heading, up to one step.

        As in allowed_distances, the floor between the drone and the safe area only has to be seen
        free: the photos see the floor in front, so nothing there can hit the drone, while the safety
        margin cannot be checked yet beside a drone that has just seen a narrow wedge ahead of it.
        From the first safe cell on, the path must stay inside the safe area.
        """
        distances = np.arange(0.0, STEP_LIMIT_CM + 1e-9, CELL_CM / 2)
        rows, columns, inside = self.index(position + distances[:, None] * direction(heading_deg))
        free = inside & (self.cells[rows, columns] == FREE)
        in_safe = inside & safe[rows, columns]
        limit = int(np.argmax(~free)) if not free.all() else len(distances)
        if not in_safe[:limit].any():
            return 0.0
        entered = int(np.argmax(in_safe[:limit]))
        leaves = ~in_safe[entered:limit]
        end = entered + (int(np.argmax(leaves)) if leaves.any() else limit - entered)
        return float(distances[end - 1])

    def unseen_in_view(self, position: np.ndarray, heading_deg: float, reach_cm: float, half_fov_deg: float) -> set[tuple[int, int]]:
        """Unknown cells, not yet given up on, that photos from here would cover, stopping at known obstacles."""
        angles = np.arange(-half_fov_deg, half_fov_deg + 1e-9, NEW_AREA_RAY_STEP_DEG)
        distances = np.arange(0.0, reach_cm, CELL_CM / 2)
        unseen = set()
        for angle in angles:
            rows, columns, inside = self.index(position + distances[:, None] * direction(heading_deg + angle))
            values = np.where(inside, self.cells[rows, columns], OBSTACLE)
            end = int(np.argmax(values == OBSTACLE)) if (values == OBSTACLE).any() else len(values)
            rows, columns = rows[:end], columns[:end]
            unknown = (values[:end] == UNKNOWN) & ~self.given_up[rows, columns]
            unseen.update(zip(rows[unknown].tolist(), columns[unknown].tolist()))
        return unseen

    def new_cells(self, position: np.ndarray, heading_deg: float, reach_cm: float, half_fov_deg: float) -> int:
        return len(self.unseen_in_view(position, heading_deg, reach_cm, half_fov_deg))

    def give_up(self, cells: set[tuple[int, int]]) -> None:
        """Stop counting cells a stop's photos covered without seeing them.

        With a 1-2 m blind zone, floor close to a wall in a narrow arm is never seen: from near, the
        wall hides it; from far, it lies beyond the trusted range. Left unknown, it would draw the
        drone back to the same spot forever.
        """
        for row, column in cells:
            if self.cells[row, column] == UNKNOWN:
                self.given_up[row, column] = True


def segment_is_passable(grid: Grid, passable: np.ndarray, start: np.ndarray, end: np.ndarray) -> bool:
    points = start + np.linspace(0.0, 1.0, max(2, int(np.linalg.norm(end - start) / (CELL_CM / 2)) + 1))[:, None] * (end - start)
    rows, columns, inside = grid.index(points[1:])
    return bool(np.all(inside & passable[rows, columns]))


def astar(passable: np.ndarray, start: tuple[int, int], goal: tuple[int, int]) -> list[tuple[int, int]] | None:
    """Shortest 8-connected path over passable cells."""
    def heuristic(cell: tuple[int, int]) -> float:
        return math.hypot(cell[0] - goal[0], cell[1] - goal[1])

    queue = [(heuristic(start), 0.0, start)]
    parents, costs = {start: None}, {start: 0.0}
    while queue:
        _, cost, cell = heapq.heappop(queue)
        if cell == goal:
            path = [cell]
            while parents[path[-1]] is not None:
                path.append(parents[path[-1]])
            return path[::-1]
        if cost > costs[cell]:
            continue
        for d_row in (-1, 0, 1):
            for d_column in (-1, 0, 1):
                neighbour = (cell[0] + d_row, cell[1] + d_column)
                if neighbour == cell or not (0 <= neighbour[0] < passable.shape[0] and 0 <= neighbour[1] < passable.shape[1]):
                    continue
                if not passable[neighbour]:
                    continue
                new_cost = cost + math.hypot(d_row, d_column)
                if new_cost < costs.get(neighbour, math.inf):
                    costs[neighbour], parents[neighbour] = new_cost, cell
                    heapq.heappush(queue, (new_cost + heuristic(neighbour), new_cost, neighbour))
    return None


class MapLink:
    """Records every map message to flights/<session>.jsonl and streams it to Unity when it can.

    The map is not critical (docs/protocol.md): if Unity is down, the flight goes on and the
    connection is retried now and then, re-sending session_start first as the protocol requires.
    """

    RECONNECT_INTERVAL_S = 5.0
    CONNECT_TIMEOUT_S = 1.0

    def __init__(self, session: str, host: str | None) -> None:
        FLIGHTS_DIR.mkdir(exist_ok=True)
        self.path = FLIGHTS_DIR / f"{session}.jsonl"
        self.log = self.path.open("w", encoding="utf-8")
        self.host = host
        self.connection = None
        self.last_attempt = -math.inf
        self.seq = 0
        self.session_start = ""

    def send(self, message_type: str, t: float, **fields) -> None:
        line = json.dumps({"type": message_type, "seq": self.seq, "t": round(t, 2), **fields}) + "\n"
        self.seq += 1
        if message_type == "session_start":
            self.session_start = line
        self.log.write(line)
        self.log.flush()
        if self.host:
            self._stream(line)

    def _stream(self, line: str) -> None:
        if self.connection is None:
            if time.monotonic() - self.last_attempt < self.RECONNECT_INTERVAL_S:
                return
            self.last_attempt = time.monotonic()
            try:
                self.connection = socket.create_connection((self.host, MAP_PORT), timeout=self.CONNECT_TIMEOUT_S)
                if line != self.session_start:
                    self.connection.sendall(self.session_start.encode("utf-8"))
            except OSError:
                self.connection = None
                return
        try:
            self.connection.sendall(line.encode("utf-8"))
        except OSError:
            self.connection.close()
            self.connection = None

    def close(self) -> None:
        self.log.close()
        if self.connection is not None:
            self.connection.close()


class SyntheticDrone(DroneInterface):
    """A stand-in for the simulator inside the measured classroom: exact moves, and rays instead of photos."""

    def __init__(self, camera_matrix: np.ndarray) -> None:
        super().__init__()
        floor = np.array(CLASSROOM_FLOOR_CM, float) - CLASSROOM_TAKEOFF_CM
        self.floor_contour = floor.astype(np.float32).reshape(-1, 1, 2)
        self.segments = np.stack([floor, np.roll(floor, -1, axis=0)], axis=1)
        self.fov = camera_fov(camera_matrix)
        self.position = np.zeros(2)
        self.yaw_deg = 0.0
        self.clock_s = 0.0

    def _send(self, command: str) -> str:
        name, _, value = command.partition(" ")
        if name in ("command", "streamon", "takeoff", "land"):
            return "ok"
        amount = float(value)
        if name in ("cw", "ccw"):
            self.yaw_deg = wrap_deg(self.yaw_deg + (amount if name == "cw" else -amount))
            self.clock_s += amount / SYNTHETIC_YAW_SPEED_DEG_S
            return "ok"
        if name == "forward":
            self.clock_s += amount / SYNTHETIC_FORWARD_SPEED_CM_S
            unit = direction(self.yaw_deg)
            for distance in np.arange(1.0, amount + 1e-9, 1.0):
                if cv2.pointPolygonTest(self.floor_contour, tuple(map(float, self.position + distance * unit)), True) < DRONE_RADIUS_CM:
                    # Like the simulator, stop at the contact point and report the planning bug.
                    self.position = self.position + (distance - 1.0) * unit
                    return "error collision"
            self.position = self.position + amount * unit
            return "ok"
        return "error unknown command"

    def see(self) -> FreeSpace:
        self.clock_s += SYNTHETIC_PHOTO_TIME_S
        angles = np.arange(-self.fov["left_deg"], self.fov["right_deg"], SWEEP_STEP_DEG)
        angles = np.round(angles / SWEEP_STEP_DEG) * SWEEP_STEP_DEG
        units = np.stack([np.sin(np.radians(self.yaw_deg + angles)), np.cos(np.radians(self.yaw_deg + angles))], axis=1)
        starts, edges = self.segments[:, 0], self.segments[:, 1] - self.segments[:, 0]
        # Ray-segment intersection for every ray against every wall at once: position + t·u = start + s·edge.
        denominator = units[:, None, 0] * edges[None, :, 1] - units[:, None, 1] * edges[None, :, 0]
        offset = starts[None] - self.position
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (offset[..., 0] * edges[None, :, 1] - offset[..., 1] * edges[None, :, 0]) / denominator
            s = (offset[..., 0] * units[:, None, 1] - offset[..., 1] * units[:, None, 0]) / denominator
        hits = np.where((np.abs(denominator) > 1e-9) & (t > 0) & (s >= 0) & (s <= 1), t, np.inf).min(axis=1)

        # A flat floor seen from the flight height through the tilted camera.
        lowest_view = math.radians(self.fov["down_deg"] + SYNTHETIC_CAMERA_TILT_DEG)
        entry = SYNTHETIC_HEIGHT_CM / math.tan(lowest_view) / np.cos(np.radians(angles))
        in_range = hits < MAX_RANGE_CM
        # stop_cm is the last free sample before the wall, as find_free_space.py reports it. A wall inside
        # the blind zone hides all floor, so the pipeline cannot measure it and reports stop_cm = entry_cm.
        stop = np.where(in_range, np.maximum(np.floor(hits / RAY_STEP_CM) * RAY_STEP_CM - RAY_STEP_CM, entry), MAX_RANGE_CM)
        kind = np.where(in_range, STOP_OBSTACLE, STOP_RANGE)
        allowed = allowed_distances(drone_safe_area(free_polygon_from_drone(angles, stop)), angles, entry)
        return FreeSpace(angles, entry, stop, kind, allowed, "", SYNTHETIC_HEIGHT_CM)

    def take_photo(self) -> np.ndarray:
        raise NotImplementedError("The synthetic drone sees by casting rays; use see()")

    def get_state(self) -> dict:
        return {"yaw_deg": self.yaw_deg, "pitch_deg": 0.0, "height_cm": SYNTHETIC_HEIGHT_CM, "battery_pct": 100.0}

    def _disconnect(self) -> None:
        pass


class Explorer:
    """The EXPLORING phase: stops, grid, choice of the next move, and the flight back to take-off."""

    def __init__(self, drone: DroneInterface, see, clock, fov: dict[str, float], map_link: MapLink, room: np.ndarray | None) -> None:
        self.drone, self.see, self.clock, self.fov, self.map = drone, see, clock, fov, map_link
        self.room = room
        self.grid = Grid()
        self.position = np.zeros(2)
        self.yaw_deg = 0.0
        self.state = {}
        self.blind_zone_cm = STEP_LIMIT_CM
        self.stop_positions: list[np.ndarray] = []
        self.stops = 0
        OUTPUT_DIR.mkdir(exist_ok=True)

    def pose(self) -> dict:
        return {"x": round(float(self.position[0]), 1), "y": round(float(self.position[1]), 1), "yaw_deg": round(self.yaw_deg, 1)}

    def read_state(self) -> None:
        self.state = self.drone.get_state()
        self.yaw_deg = self.state["yaw_deg"]

    def command(self, command: str) -> None:
        start = self.pose()
        reply = self.drone.send(command)
        if reply != "ok":
            raise RuntimeError(f"{command}: {reply}")
        if command.startswith("forward"):
            end = self.position + float(command.split()[1]) * direction(self.yaw_deg)
            self.grid.add_trail(self.position, end)
            self.position = end
        self.read_state()
        self.map.send("move", self.clock(), command=command, **{"from": start, "to": self.pose()})

    def turn(self, relative_deg: float) -> None:
        turn = round(wrap_deg(relative_deg))
        if turn:
            self.command(f"cw {turn}" if turn > 0 else f"ccw {-turn}")

    def look(self) -> FreeSpace | None:
        """Take a photo and add it to the grid and the map; a photo without a reference tile is retaken once."""
        free_space = self.see() or self.see()
        if free_space is None:
            return None
        rays = trusted_rays(free_space)
        self.grid.add_observation(self.position, self.yaw_deg, rays)
        self.blind_zone_cm = float(np.median(free_space.entry_cm))
        pose = {**self.pose(), "z": round(self.state["height_cm"], 1), "pitch_deg": round(self.state["pitch_deg"], 1)}
        self.map.send("observation", self.clock(), pose=pose, rays={
            "angle_deg": [float(a) for a in rays["angle_deg"]],
            "entry_cm": [round(float(e)) for e in rays["entry_cm"]],
            "stop_cm": [round(float(s)) for s in rays["stop_cm"]],
            "kind": [int(k) for k in rays["kind"]],
        })
        return free_space

    def sweep(self) -> float:
        """Photos to the left, ahead and to the right; returns the heading the stop is planned from."""
        centre = self.yaw_deg
        self.look()
        self.turn(SWEEP_TURN_DEG)
        self.look()
        self.turn(-2 * SWEEP_TURN_DEG)
        self.look()
        return centre

    def revisits(self, point: np.ndarray) -> bool:
        return any(np.linalg.norm(point - old) < REVISIT_CM for old in self.stop_positions)

    def sweep_half_fov(self) -> float:
        """Half the angle the three photos of a stop cover, which is what the next stop will see."""
        return SWEEP_TURN_DEG + (self.fov["left_deg"] + self.fov["right_deg"]) / 2

    def candidates(self, centre_deg: float) -> list[tuple[float, float]]:
        """Return (relative heading, clear distance) of every heading worth taking, best first.

        Right only when a whole step is clear there, so a narrow gap does not wedge the drone against
        a wall; otherwise ahead, then left, each by the longest clear step and then the smallest turn.
        A tie between equal turns to either side goes to the right, like the rest of the rule.
        """
        safe = self.grid.safe_area()
        reach = self.blind_zone_cm + STEP_LIMIT_CM
        right, ahead, left = [], [], []
        for relative in range(-MAX_HEADING_DEG, MAX_HEADING_DEG + 1, HEADING_STEP_DEG):
            heading = centre_deg + relative
            clear = self.grid.clear_distance(safe, self.position, heading)
            if clear < MIN_FORWARD_CM:
                continue
            arrival = self.position + clear * direction(heading)
            if self.revisits(arrival):
                continue
            new = self.grid.new_cells(arrival, heading, reach, self.sweep_half_fov())
            if new < MIN_NEW_CELLS:
                continue
            if relative > SIDE_SECTOR_DEG:
                if clear >= STEP_LIMIT_CM:
                    right.append((-new, relative, clear))
            elif relative < -SIDE_SECTOR_DEG:
                left.append((-math.floor(clear / CELL_CM), abs(relative), -relative, relative, clear))
            else:
                ahead.append((-math.floor(clear / CELL_CM), abs(relative), -relative, relative, clear))
        return [(r, c) for _, r, c in sorted(right)] + [(r, c) for *_, r, c in sorted(ahead)] + [(r, c) for *_, r, c in sorted(left)]

    def advance(self, centre_deg: float, options: list[tuple[float, float]]) -> bool:
        """Turn to the best option, confirm it with a fresh photo and fly; try the next one if it is not clear.

        The photo limits the step only where it can measure. If it sees floor ahead, the blind zone up
        to that floor is clear by occlusion and allowed_cm rules beyond it. If a wall stands inside its
        blind zone, it cannot tell how far, and the grid, which saw that floor from farther away, rules.
        """
        for relative, clear in options[:MAX_CONFIRMATIONS]:
            self.turn(centre_deg + relative - self.yaw_deg)
            photo = self.look()
            if photo is None:
                continue
            ahead = int(np.argmin(np.abs(photo.angle_deg)))
            if photo.stop_cm[ahead] > photo.entry_cm[ahead]:
                clear = min(clear, max(photo.allowed_cm[ahead], photo.entry_cm[ahead]))
            step = int(min(clear, STEP_LIMIT_CM, MAX_FORWARD_CM))
            if step >= MIN_FORWARD_CM:
                self.command(f"forward {step}")
                return True
        return False

    def fly_path(self, path: list[tuple[int, int]]) -> None:
        """Fly an A* path as straight legs of at most one step. The path only crosses floor already seen
        safe or flown, so the legs need no new photo."""
        passable = self.grid.passable(self.grid.safe_area(), self.position)
        points = [self.grid.centre(*cell) for cell in path]
        index = 0
        while index < len(points) - 1:
            target = index + 1
            while (target + 1 < len(points) and np.linalg.norm(points[target + 1] - self.position) <= STEP_LIMIT_CM
                   and segment_is_passable(self.grid, passable, self.position, points[target + 1])):
                target += 1
            offset = points[target] - self.position
            distance = float(np.linalg.norm(offset))
            index = target
            if distance < MIN_FORWARD_CM:
                continue
            self.turn(math.degrees(math.atan2(offset[0], offset[1])) - self.yaw_deg)
            self.command(f"forward {int(distance)}")

    def go_to_frontier(self) -> bool:
        """Fly to the nearest reachable spot that still faces unseen floor, then face it."""
        safe = self.grid.safe_area()
        reach = self.blind_zone_cm + STEP_LIMIT_CM
        half_fov = self.sweep_half_fov()
        rows, columns = np.nonzero(safe)
        spots = [self.grid.centre(r, c) for r, c in zip(rows[::2], columns[::2])]
        spots.sort(key=lambda spot: np.linalg.norm(spot - self.position))
        start = tuple(int(v[0]) for v in self.grid.index(self.position)[:2])
        for spot in spots:
            if self.revisits(spot):
                continue
            best = max(FRONTIER_HEADINGS_DEG, key=lambda h: self.grid.new_cells(spot, h, reach, half_fov))
            if self.grid.new_cells(spot, best, reach, half_fov) < MIN_NEW_CELLS:
                continue
            path = astar(self.grid.passable(safe, self.position), start, tuple(int(v[0]) for v in self.grid.index(spot)[:2]))
            if path is None:
                continue
            self.fly_path(path)
            self.turn(best - self.yaw_deg)
            return True
        return False

    def return_home(self) -> float | None:
        """Fly back to take-off over safe floor and the trail, which always leads back to the pad.
        Returns the distance left to the pad, or None when there is no path."""
        start = tuple(int(v[0]) for v in self.grid.index(self.position)[:2])
        pad = tuple(int(v[0]) for v in self.grid.index(np.zeros(2))[:2])
        path = astar(self.grid.passable(self.grid.safe_area(), self.position), start, pad)
        if path is None:
            return None
        self.fly_path(path)
        return float(np.linalg.norm(self.position))

    def draw(self, title: str, choice: tuple[float, float] | None = None) -> np.ndarray:
        safe = self.grid.safe_area()
        image = np.zeros((*self.grid.cells.shape, 3), np.uint8)
        image[:] = UNKNOWN_BGR
        image[self.grid.given_up] = GIVEN_UP_BGR
        image[self.grid.cells == FREE] = FREE_BGR
        image[safe] = SAFE_BGR
        image[self.grid.cells == OBSTACLE] = OBSTACLE_BGR
        image[self.grid.trail] = TRAIL_BGR
        # Rows grow downwards in the image, y grows forward in the map.
        image = cv2.resize(image[::-1], None, fx=TOP_VIEW_PX_PER_CELL, fy=TOP_VIEW_PX_PER_CELL, interpolation=cv2.INTER_NEAREST)

        def px(point: np.ndarray) -> tuple[int, int]:
            scale = TOP_VIEW_PX_PER_CELL / CELL_CM
            return round((point[0] + GRID_HALF_SIZE_CM) * scale), round((GRID_HALF_SIZE_CM - point[1]) * scale)

        if self.room is not None:
            cv2.polylines(image, [np.array([px(p) for p in self.room], np.int32)], True, ROOM_BGR, 1)
        cv2.circle(image, px(self.position), 6, DRONE_BGR, -1)
        cv2.arrowedLine(image, px(self.position), px(self.position + 40 * direction(self.yaw_deg)), DRONE_BGR, 2)
        if choice is not None:
            heading, clear = choice
            cv2.arrowedLine(image, px(self.position), px(self.position + clear * direction(heading)), CHOICE_BGR, 3, tipLength=0.1)
        known = np.argwhere(self.grid.cells[::-1] != UNKNOWN)
        if len(known):
            (top, left), (bottom, right) = known.min(axis=0) * TOP_VIEW_PX_PER_CELL, (known.max(axis=0) + 1) * TOP_VIEW_PX_PER_CELL
            pad = 20 * TOP_VIEW_PX_PER_CELL
            image = image[max(top - pad, 0):bottom + pad, max(left - pad, 0):right + pad].copy()
        cv2.putText(image, title, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1, cv2.LINE_AA)
        return image

    def run(self, session: str) -> str:
        camera = {"hfov_deg": round(self.fov["left_deg"] + self.fov["right_deg"], 1),
                  "vfov_up_deg": round(self.fov["up_deg"], 1), "vfov_down_deg": round(self.fov["down_deg"], 1)}
        self.map.send("session_start", self.clock(), version=1, session=session, camera=camera,
                      drone_radius_cm=DRONE_RADIUS_CM, safety_margin_cm=SAFETY_MARGIN_CM)
        self.map.send("state", self.clock(), mode="exploring", person_distance_cm=-1.0)
        for command in ("command", "streamon", "takeoff"):
            reply = self.drone.send(command)
            if reply != "ok":
                raise RuntimeError(f"{command}: {reply}")
        self.read_state()

        reason = "back_at_start"
        while self.stops < MAX_STOPS:
            if self.state["battery_pct"] < LOW_BATTERY_PCT:
                reason = "low_battery"
                break
            self.stops += 1
            self.stop_positions.append(self.position.copy())
            centre = self.sweep()
            # The three photos just covered these cells within reach and still could not see them.
            self.grid.give_up(self.grid.unseen_in_view(self.position, centre, self.blind_zone_cm + STEP_LIMIT_CM, self.sweep_half_fov()))
            options = self.candidates(centre)
            if not options:
                # A dead end: look behind before deciding there is nothing left here.
                self.turn(centre + 180 - self.yaw_deg)
                centre = self.sweep()
                options = self.candidates(centre)
            write_image(OUTPUT_DIR / f"1_grid_stop{self.stops:02d}.png",
                        self.draw(f"stop {self.stops}", (centre + options[0][0], options[0][1]) if options else None))
            if options and self.advance(centre, options):
                continue
            if not self.go_to_frontier():
                break

        home = self.return_home()
        outcome = "no path home" if home is None else f"{home:.0f} cm from take-off"
        write_image(OUTPUT_DIR / "2_final.png", self.draw(f"final: {self.stops} stops, {outcome}"))
        self.map.send("session_end", self.clock(), reason=reason)
        return f"{reason}, {outcome}"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    target = parser.add_mutually_exclusive_group(required=True)
    target.add_argument("--synthetic", action="store_true", help="a stand-in drone in the measured classroom")
    target.add_argument("--sim", metavar="HOST", help="the Unity simulator's address")
    target.add_argument("--tello", action="store_true", help="the real drone, over its own wifi")
    parser.add_argument("--map", metavar="HOST", help="Unity map's address; without it the flight is only recorded")
    args = parser.parse_args()

    with np.load(CALIBRATION_FILE) as calibration:
        camera_matrix = calibration["camera_matrix"]
    room = None
    if args.synthetic:
        drone = SyntheticDrone(camera_matrix)
        see, clock = drone.see, lambda: drone.clock_s
        room = np.array(CLASSROOM_FLOOR_CM, float) - CLASSROOM_TAKEOFF_CM
    else:
        drone = UnityDrone(args.sim) if args.sim else TelloDrone()
        start = time.monotonic()
        see, clock = (lambda: analyze_frame(drone.take_photo(), camera_matrix)), (lambda: time.monotonic() - start)

    session = datetime.now().strftime("%Y%m%d_%H%M%S")
    map_link = MapLink(session, args.map)
    explorer = Explorer(drone, see, clock, camera_fov(camera_matrix), map_link, room)
    try:
        result = explorer.run(session)
    finally:
        drone.close()
        map_link.close()
    print(f"{result}: {explorer.stops} stops, {map_link.seq} messages in {map_link.path}, grid images in {OUTPUT_DIR}/")


if __name__ == "__main__":
    main()
