"""Fly a scripted route through a synthetic 2D room and emit the map protocol messages (docs/protocol.md)."""

import argparse
import json
import math
import socket
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

from find_free_space import (
    DRONE_RADIUS_CM, FREE_RAY_BGR, MAX_RANGE_CM, SAFETY_MARGIN_CM, STOP_COLORS_BGR, STOP_OBSTACLE, STOP_RANGE,
    SWEEP_STEP_DEG,
)
from image_io import write_image

OUTPUT_DIR = Path("flights")

# Rectangles as (x_min, y_min, x_max, y_max) in cm, in the protocol frame: origin at take-off, x right,
# y forward. 10 x 8 m is the classroom size the voxel map is planned for.
ROOM = (-200, -100, 800, 700)
OBSTACLES = [(150, 250, 310, 330), (450, 250, 610, 330), (-200, 350, -140, 470), (300, 640, 420, 700)]
VICTIMS = [(650, 520), (-80, 600)]

# Every leg keeps at least 90 cm from walls and furniture, well above the 24 cm the planner requires,
# and each victim is seen at least once beyond the blind zone.
ROUTE = [
    "forward 150", "cw 90", "forward 200", "forward 200", "forward 200", "forward 100",
    "ccw 90", "forward 200", "forward 200", "ccw 90", "forward 200", "forward 200",
    "forward 200", "forward 100", "ccw 90", "forward 200", "forward 200", "forward 150", "cw 180",
]

# Tello camera field of view from the calibrated intrinsics, and the height it holds in the classroom
# (76-83 cm), so the synthetic blind zone matches the real one.
HFOV_DEG = 54.5
VFOV_UP_DEG = 20.4
VFOV_DOWN_DEG = 22.1
FLIGHT_HEIGHT_CM = 82.0
# A plausible YOLO score; Unity only displays it.
VICTIM_CONFIDENCE = 0.9

# Rough Tello figures. They only space the messages in time, so a replay looks like a real flight.
FORWARD_SPEED_CM_S = 50.0
YAW_SPEED_DEG_S = 90.0
PHOTO_TIME_S = 1.0

DEFAULT_PORT = 5005
TOP_VIEW_MARGIN_PX = 20
BACKGROUND_BGR = (255, 255, 255)
WALL_BGR = (0, 0, 0)
OBSTACLE_BGR = (120, 120, 120)
VICTIM_BGR = (0, 0, 200)
ROUTE_BGR = (200, 80, 0)


def rect_segments(rect: tuple) -> list[tuple]:
    x0, y0, x1, y1 = rect
    corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    return [(corners[i], corners[(i + 1) % 4]) for i in range(4)]


SEGMENTS = [s for rect in [ROOM, *OBSTACLES] for s in rect_segments(rect)]


def direction(heading_deg: float) -> tuple[float, float]:
    """Unit vector of a heading measured clockwise from +y, as yaw_deg is defined."""
    h = math.radians(heading_deg)
    return math.sin(h), math.cos(h)


def ray_hit_cm(x: float, y: float, heading_deg: float) -> float:
    """Distance to the first wall or obstacle along the ray, or infinity."""
    dx, dy = direction(heading_deg)
    nearest = math.inf
    for (ax, ay), (bx, by) in SEGMENTS:
        ex, ey = bx - ax, by - ay
        denom = dx * ey - dy * ex
        if abs(denom) < 1e-9:
            continue
        t = ((ax - x) * ey - (ay - y) * ex) / denom
        u = ((ax - x) * dy - (ay - y) * dx) / denom
        if t >= 0 and 0 <= u <= 1:
            nearest = min(nearest, t)
    return nearest


def entry_cm(angle_deg: float) -> float:
    """Start of the visible floor along a ray: the lowest image row meets the floor at a fixed forward distance."""
    forward_cm = FLIGHT_HEIGHT_CM / math.tan(math.radians(VFOV_DOWN_DEG))
    return forward_cm / math.cos(math.radians(angle_deg))


def observe(pose: dict) -> tuple[dict, list[int]]:
    # Same angular step and range as find_free_space.py, so Unity receives rays of the real density.
    half_rays = int(HFOV_DEG / 2 / SWEEP_STEP_DEG)
    rays = {"angle_deg": [], "entry_cm": [], "stop_cm": [], "kind": []}
    for i in range(-half_rays, half_rays + 1):
        angle = i * SWEEP_STEP_DEG
        hit = ray_hit_cm(pose["x"], pose["y"], pose["yaw_deg"] + angle)
        stop, kind = (hit, STOP_OBSTACLE) if hit < MAX_RANGE_CM else (MAX_RANGE_CM, STOP_RANGE)
        rays["angle_deg"].append(angle)
        rays["entry_cm"].append(round(min(entry_cm(angle), stop)))
        rays["stop_cm"].append(round(stop))
        rays["kind"].append(kind)

    seen = []
    for victim_id, (vx, vy) in enumerate(VICTIMS, start=1):
        distance = math.hypot(vx - pose["x"], vy - pose["y"])
        angle = (math.degrees(math.atan2(vx - pose["x"], vy - pose["y"])) - pose["yaw_deg"] + 180) % 360 - 180
        ray = round(angle / SWEEP_STEP_DEG) + half_rays
        if 0 <= ray < len(rays["angle_deg"]) and rays["entry_cm"][ray] < distance < rays["stop_cm"][ray]:
            seen.append(victim_id)
    return rays, seen


def apply_command(pose: dict, command: str) -> tuple[dict, float]:
    """Dead-reckon one SDK command, as the brain will do with the real drone. Returns the new pose and its duration."""
    name, value = command.split()
    value = float(value)
    new = dict(pose)
    if name == "forward":
        dx, dy = direction(pose["yaw_deg"])
        new["x"] = round(pose["x"] + value * dx, 1)
        new["y"] = round(pose["y"] + value * dy, 1)
        return new, value / FORWARD_SPEED_CM_S
    turn = value if name == "cw" else -value
    new["yaw_deg"] = (pose["yaw_deg"] + turn) % 360
    return new, value / YAW_SPEED_DEG_S


def generate(session: str) -> list[dict]:
    messages = []
    t = 0.0

    def emit(message_type: str, **fields) -> None:
        messages.append({"type": message_type, "seq": len(messages), "t": round(t, 2), **fields})

    def emit_observation(pose: dict) -> None:
        rays, seen = observe(pose)
        emit("observation", pose={**pose, "z": FLIGHT_HEIGHT_CM, "pitch_deg": 0.0}, rays=rays)
        for victim_id in seen:
            vx, vy = VICTIMS[victim_id - 1]
            emit("victim", id=victim_id, x=float(vx), y=float(vy), confidence=VICTIM_CONFIDENCE)

    emit("session_start", version=1, session=session,
         camera={"hfov_deg": HFOV_DEG, "vfov_up_deg": VFOV_UP_DEG, "vfov_down_deg": VFOV_DOWN_DEG},
         drone_radius_cm=DRONE_RADIUS_CM, safety_margin_cm=SAFETY_MARGIN_CM)
    emit("state", mode="exploring", person_distance_cm=-1.0)

    pose = {"x": 0.0, "y": 0.0, "yaw_deg": 0.0}
    t += PHOTO_TIME_S
    emit_observation(pose)
    for command in ROUTE:
        new_pose, duration = apply_command(pose, command)
        emit("move", command=command, **{"from": pose, "to": new_pose})
        pose = new_pose
        t += duration + PHOTO_TIME_S
        emit_observation(pose)

    emit("session_end", reason="back_at_start")
    return messages


def draw_top_view(messages: list[dict]) -> np.ndarray:
    x_min, y_min, x_max, y_max = ROOM
    image = np.full((y_max - y_min + 2 * TOP_VIEW_MARGIN_PX, x_max - x_min + 2 * TOP_VIEW_MARGIN_PX, 3), BACKGROUND_BGR, np.uint8)

    def px(x: float, y: float) -> tuple[int, int]:
        # Image rows grow downwards, the protocol's y grows forward.
        return round(x - x_min) + TOP_VIEW_MARGIN_PX, round(y_max - y) + TOP_VIEW_MARGIN_PX

    for message in messages:
        if message["type"] != "observation":
            continue
        pose, rays = message["pose"], message["rays"]
        for angle, entry, stop, kind in zip(rays["angle_deg"], rays["entry_cm"], rays["stop_cm"], rays["kind"]):
            dx, dy = direction(pose["yaw_deg"] + angle)
            start = px(pose["x"] + entry * dx, pose["y"] + entry * dy)
            end = px(pose["x"] + stop * dx, pose["y"] + stop * dy)
            cv2.line(image, start, end, FREE_RAY_BGR, 1)
            cv2.circle(image, end, 2, STOP_COLORS_BGR[kind], -1)

    for rect in OBSTACLES:
        cv2.rectangle(image, px(rect[0], rect[3]), px(rect[2], rect[1]), OBSTACLE_BGR, -1)
    cv2.rectangle(image, px(x_min, y_max), px(x_max, y_min), WALL_BGR, 2)
    for vx, vy in VICTIMS:
        cv2.circle(image, px(vx, vy), 12, VICTIM_BGR, 3)

    moves = [m for m in messages if m["type"] == "move"]
    for move in moves:
        cv2.line(image, px(move["from"]["x"], move["from"]["y"]), px(move["to"]["x"], move["to"]["y"]), ROUTE_BGR, 2)
    for move in moves:
        cv2.circle(image, px(move["to"]["x"], move["to"]["y"]), 5, ROUTE_BGR, -1)
    return image


def stream(messages: list[dict], host: str, port: int, speed: float) -> None:
    with socket.create_connection((host, port), timeout=5) as connection:
        previous_t = 0.0
        for message in messages:
            time.sleep((message["t"] - previous_t) / speed)
            previous_t = message["t"]
            connection.sendall((json.dumps(message) + "\n").encode("utf-8"))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", help="Unity's address; without it the flight is only written to a file")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--speed", type=float, default=1.0, help="replay speed factor when streaming")
    args = parser.parse_args()

    session = datetime.now().strftime("%Y%m%d_%H%M%S")
    messages = generate(session)

    OUTPUT_DIR.mkdir(exist_ok=True)
    log_path = OUTPUT_DIR / f"{session}.jsonl"
    log_path.write_text("".join(json.dumps(m) + "\n" for m in messages), encoding="utf-8")
    write_image(OUTPUT_DIR / f"{session}_top.png", draw_top_view(messages))
    print(f"Saved {len(messages)} messages to {log_path}")

    if args.host:
        stream(messages, args.host, args.port, args.speed)
        print(f"Streamed them to {args.host}:{args.port}")


if __name__ == "__main__":
    main()
