"""Fly a scripted route through the measured classroom and emit the map protocol messages (docs/protocol.md)."""

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
    DRONE_RADIUS_CM, MAX_RANGE_CM, SAFETY_MARGIN_CM, STEP_LIMIT_CM, STOP_OBSTACLE, STOP_RANGE, SWEEP_STEP_DEG,
)
from image_io import write_image

OUTPUT_DIR = Path("flights")

# The free floor of docs/classroom.md, an inverted U around the tables block, in the room frame (cm, origin at
# the bottom-left corner). Everything outside it is a wall or furniture.
FREE_FLOOR_ROOM_CM = [(0, 0), (120, 0), (120, 310), (370, 310), (370, 0), (460, 0), (460, 560), (0, 560)]
ROOM_SIZE_CM = (460, 560)
# The take-off pad, also from docs/classroom.md. The protocol frame starts there, so room = protocol + pad.
TAKEOFF_ROOM_CM = (60, 60)
# People lying on the free floor: one across the top band and one in the narrow right arm.
VICTIMS_ROOM_CM = [(330, 500), (430, 150)]

# Up the left arm, across the top band and down the narrow right arm, then back to the pad over the same stops,
# as the mission returns to wait for the rescuer there. Legs run along the middle of each arm, at most one 2 m
# step at a time.
ROUTE = [
    "forward 200", "forward 180", "cw 90", "forward 200", "forward 155", "cw 90", "forward 200", "forward 180",
    "cw 180", "forward 180", "forward 200", "ccw 90", "forward 155", "forward 200", "ccw 90", "forward 180",
    "forward 200", "cw 180",
]

# Tello camera field of view from the calibrated intrinsics.
HFOV_DEG = 54.5
VFOV_UP_DEG = 20.4
VFOV_DOWN_DEG = 22.1
# The height of the hand-held test video (median 79.7 cm), which the real flight will keep.
FLIGHT_HEIGHT_CM = 80.0
# A plausible YOLO score; Unity only displays it.
VICTIM_CONFIDENCE = 0.9

# Rough Tello figures. They only space the messages in time, so a replay looks like a real flight.
FORWARD_SPEED_CM_S = 50.0
YAW_SPEED_DEG_S = 90.0
PHOTO_TIME_S = 1.0
# Route legs are checked every 2 cm against the drone's size, the same resolution as the planner's grid.
ROUTE_CHECK_STEP_CM = 2.0

DEFAULT_PORT = 5005

# Top view layout: 1.5 px per cm makes the 4.6 x 5.6 m room about 700 x 850 px.
PX_PER_CM = 1.5
MARGIN_LEFT_PX, MARGIN_TOP_PX, MARGIN_BOTTOM_PX, LEGEND_WIDTH_PX = 70, 60, 60, 360
PAGE_BGR = (245, 245, 245)
UNSEEN_FLOOR_BGR = (255, 255, 255)
SEEN_FLOOR_BGR = (160, 220, 160)
BLOCKED_BGR = (95, 95, 95)
GRID_BGR = (205, 205, 205)
OBSTACLE_FACE_BGR = (40, 40, 220)
ROUTE_BGR = (200, 90, 20)
RETURN_BGR = (170, 60, 150)
# A leg flown back over an earlier one is drawn this far to its right, so both arrows stay visible.
RETURN_OFFSET_PX = 9
VICTIM_BGR = (30, 30, 200)
TEXT_BGR = (40, 40, 40)
FONT = cv2.FONT_HERSHEY_SIMPLEX


def to_protocol(room_point: tuple[float, float]) -> tuple[float, float]:
    return room_point[0] - TAKEOFF_ROOM_CM[0], room_point[1] - TAKEOFF_ROOM_CM[1]


FREE_FLOOR_CM = [to_protocol(point) for point in FREE_FLOOR_ROOM_CM]
WALLS = list(zip(FREE_FLOOR_CM, FREE_FLOOR_CM[1:] + FREE_FLOOR_CM[:1]))
VICTIMS = [to_protocol(point) for point in VICTIMS_ROOM_CM]


def direction(heading_deg: float) -> tuple[float, float]:
    """Unit vector of a heading measured clockwise from +y, as yaw_deg is defined."""
    h = math.radians(heading_deg)
    return math.sin(h), math.cos(h)


def ray_hit_cm(x: float, y: float, heading_deg: float) -> float:
    """Distance to the first wall or obstacle along the ray, or infinity."""
    dx, dy = direction(heading_deg)
    nearest = math.inf
    for (ax, ay), (bx, by) in WALLS:
        ex, ey = bx - ax, by - ay
        denom = dx * ey - dy * ex
        if abs(denom) < 1e-9:
            continue
        t = ((ax - x) * ey - (ay - y) * ex) / denom
        u = ((ax - x) * dy - (ay - y) * dx) / denom
        if t >= 0 and 0 <= u <= 1:
            nearest = min(nearest, t)
    return nearest


def wall_distance_cm(x: float, y: float) -> float:
    nearest = math.inf
    for (ax, ay), (bx, by) in WALLS:
        ex, ey = bx - ax, by - ay
        t = max(0.0, min(1.0, ((x - ax) * ex + (y - ay) * ey) / (ex * ex + ey * ey)))
        nearest = min(nearest, math.hypot(x - ax - t * ex, y - ay - t * ey))
    return nearest


def on_free_floor(x: float, y: float) -> bool:
    inside = False
    for (ax, ay), (bx, by) in WALLS:
        if (ay > y) != (by > y) and x < ax + (y - ay) * (bx - ax) / (by - ay):
            inside = not inside
    return inside


def entry_cm(angle_deg: float) -> float:
    """Start of the visible floor along a ray: the lowest image row meets the floor at a fixed forward distance."""
    forward_cm = FLIGHT_HEIGHT_CM / math.tan(math.radians(VFOV_DOWN_DEG))
    return forward_cm / math.cos(math.radians(angle_deg))


def observe(pose: dict) -> tuple[dict, list[int]]:
    # Same angular step and range as find_free_space.py, so Unity receives rays of the real density.
    half_rays = int(HFOV_DEG / 2 / SWEEP_STEP_DEG)
    rays = {"angle_deg": [], "entry_cm": [], "stop_cm": [], "kind": []}
    hits = []
    for i in range(-half_rays, half_rays + 1):
        angle = i * SWEEP_STEP_DEG
        hit = ray_hit_cm(pose["x"], pose["y"], pose["yaw_deg"] + angle)
        entry = entry_cm(angle)
        # The trusted range of docs/protocol.md: only the floor the next step will cross is excavated.
        trusted = entry + STEP_LIMIT_CM
        stop, kind = (hit, STOP_OBSTACLE) if hit <= trusted else (trusted, STOP_RANGE)
        rays["angle_deg"].append(angle)
        rays["entry_cm"].append(round(min(entry, stop)))
        rays["stop_cm"].append(round(stop))
        rays["kind"].append(kind)
        hits.append(hit)

    # Victims are found by the person detector, not by the map's rays, so the trusted range does not apply:
    # only real obstacles hide them, up to the range the pipeline measures.
    seen = []
    for victim_id, (vx, vy) in enumerate(VICTIMS, start=1):
        distance = math.hypot(vx - pose["x"], vy - pose["y"])
        angle = (math.degrees(math.atan2(vx - pose["x"], vy - pose["y"])) - pose["yaw_deg"] + 180) % 360 - 180
        ray = round(angle / SWEEP_STEP_DEG) + half_rays
        if 0 <= ray < len(hits) and rays["entry_cm"][ray] < distance < min(hits[ray], MAX_RANGE_CM):
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


def check_leg(start: dict, end: dict, command: str) -> None:
    """Refuse a leg the real planner would never allow, so an edited room or route cannot produce an impossible flight."""
    clearance = DRONE_RADIUS_CM + SAFETY_MARGIN_CM
    length = math.hypot(end["x"] - start["x"], end["y"] - start["y"])
    for i in range(int(length / ROUTE_CHECK_STEP_CM) + 1):
        f = i * ROUTE_CHECK_STEP_CM / length if length else 0.0
        x, y = start["x"] + f * (end["x"] - start["x"]), start["y"] + f * (end["y"] - start["y"])
        if not on_free_floor(x, y) or wall_distance_cm(x, y) < clearance:
            room_x, room_y = x + TAKEOFF_ROOM_CM[0], y + TAKEOFF_ROOM_CM[1]
            raise ValueError(f"'{command}' passes within {clearance:.0f} cm of a wall at room ({room_x:.0f}, {room_y:.0f}) cm")


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
        check_leg(pose, new_pose, command)
        emit("move", command=command, **{"from": pose, "to": new_pose})
        pose = new_pose
        t += duration + PHOTO_TIME_S
        emit_observation(pose)

    emit("session_end", reason="back_at_start")
    return messages


def to_px(x: float, y: float) -> tuple[int, int]:
    """Protocol cm to top view pixels. Image rows grow downwards, the room's y grows up the plan."""
    room_x, room_y = x + TAKEOFF_ROOM_CM[0], y + TAKEOFF_ROOM_CM[1]
    return round(MARGIN_LEFT_PX + room_x * PX_PER_CM), round(MARGIN_TOP_PX + (ROOM_SIZE_CM[1] - room_y) * PX_PER_CM)


def put_text(image: np.ndarray, text: str, origin: tuple[int, int], scale: float = 0.5, bold: bool = False) -> None:
    cv2.putText(image, text, origin, FONT, scale, TEXT_BGR, 2 if bold else 1, cv2.LINE_AA)


def draw_top_view(messages: list[dict], session: str) -> np.ndarray:
    room_w, room_h = (round(size * PX_PER_CM) for size in ROOM_SIZE_CM)
    height = MARGIN_TOP_PX + room_h + MARGIN_BOTTOM_PX
    width = MARGIN_LEFT_PX + room_w + 40 + LEGEND_WIDTH_PX
    image = np.full((height, width, 3), PAGE_BGR, np.uint8)
    floor = np.array([to_px(*point) for point in FREE_FLOOR_CM], np.int32)

    # Walls and furniture are whatever the room's rectangle holds outside the free floor.
    cv2.rectangle(image, (MARGIN_LEFT_PX, MARGIN_TOP_PX), (MARGIN_LEFT_PX + room_w, MARGIN_TOP_PX + room_h), BLOCKED_BGR, -1)
    cv2.fillPoly(image, [floor], UNSEEN_FLOOR_BGR)

    # The union of every photo's visible band, from its blind zone to its trusted range, as one area.
    seen = np.zeros(image.shape[:2], np.uint8)
    observations = [m for m in messages if m["type"] == "observation"]
    for message in observations:
        pose, rays = message["pose"], message["rays"]
        near, far = [], []
        for angle, entry, stop in zip(rays["angle_deg"], rays["entry_cm"], rays["stop_cm"]):
            dx, dy = direction(pose["yaw_deg"] + angle)
            near.append(to_px(pose["x"] + entry * dx, pose["y"] + entry * dy))
            far.append(to_px(pose["x"] + stop * dx, pose["y"] + stop * dy))
        cv2.fillPoly(seen, [np.array(near + far[::-1], np.int32)], 255)
    floor_mask = np.zeros_like(seen)
    cv2.fillPoly(floor_mask, [floor], 255)
    image[(seen > 0) & (floor_mask > 0)] = SEEN_FLOOR_BGR

    for metre in range(1, max(ROOM_SIZE_CM) // 100 + 1):
        if metre * 100 < ROOM_SIZE_CM[0]:
            x = MARGIN_LEFT_PX + round(metre * 100 * PX_PER_CM)
            cv2.line(image, (x, MARGIN_TOP_PX), (x, MARGIN_TOP_PX + room_h), GRID_BGR, 1)
        if metre * 100 < ROOM_SIZE_CM[1]:
            y = MARGIN_TOP_PX + room_h - round(metre * 100 * PX_PER_CM)
            cv2.line(image, (MARGIN_LEFT_PX, y), (MARGIN_LEFT_PX + room_w, y), GRID_BGR, 1)
    cv2.polylines(image, [floor], True, TEXT_BGR, 2)
    for metre in range(0, max(ROOM_SIZE_CM) // 100 + 1):
        if metre * 100 <= ROOM_SIZE_CM[0]:
            put_text(image, f"{metre} m", (MARGIN_LEFT_PX + round(metre * 100 * PX_PER_CM) - 12, MARGIN_TOP_PX + room_h + 25))
        if metre * 100 <= ROOM_SIZE_CM[1]:
            put_text(image, f"{metre} m", (15, MARGIN_TOP_PX + room_h - round(metre * 100 * PX_PER_CM) + 5))

    for message in observations:
        pose, rays = message["pose"], message["rays"]
        for angle, stop, kind in zip(rays["angle_deg"], rays["stop_cm"], rays["kind"]):
            if kind == STOP_OBSTACLE:
                dx, dy = direction(pose["yaw_deg"] + angle)
                cv2.circle(image, to_px(pose["x"] + stop * dx, pose["y"] + stop * dy), 2, OBSTACLE_FACE_BGR, -1, cv2.LINE_AA)

    flown = set()
    for move in (m for m in messages if m["type"] == "move"):
        start, end = to_px(move["from"]["x"], move["from"]["y"]), to_px(move["to"]["x"], move["to"]["y"])
        if start == end:
            continue
        colour = ROUTE_BGR
        if (end, start) in flown:
            # Image y grows downwards, so (-dy, dx) is the right-hand side of the direction of travel.
            length = math.dist(start, end)
            ox, oy = (round(RETURN_OFFSET_PX * c / length) for c in (-(end[1] - start[1]), end[0] - start[0]))
            start, end, colour = (start[0] + ox, start[1] + oy), (end[0] + ox, end[1] + oy), RETURN_BGR
        else:
            flown.add((start, end))
        cv2.arrowedLine(image, start, end, colour, 3, cv2.LINE_AA, tipLength=12 / math.dist(start, end))
    pad = to_px(0, 0)
    cv2.rectangle(image, (pad[0] - 11, pad[1] - 11), (pad[0] + 11, pad[1] + 11), ROUTE_BGR, -1)

    found = {m["id"] for m in messages if m["type"] == "victim"}
    for victim_id, victim in enumerate(VICTIMS, start=1):
        cv2.circle(image, to_px(*victim), 11, VICTIM_BGR, -1 if victim_id in found else 3, cv2.LINE_AA)

    put_text(image, f"Synthetic flight {session}", (MARGIN_LEFT_PX, 35), 0.7, bold=True)
    draw_legend(image, MARGIN_LEFT_PX + room_w + 40, MARGIN_TOP_PX)
    return image


def draw_legend(image: np.ndarray, left: int, top: int) -> None:
    def swatch(row: int, draw) -> int:
        y = top + 40 + row * 52
        draw((left + 20, y))
        return y

    put_text(image, "Legend", (left, top + 10), 0.6, bold=True)
    entries = [
        (lambda p: cv2.rectangle(image, (p[0] - 14, p[1] - 10), (p[0] + 14, p[1] + 10), UNSEEN_FLOOR_BGR, -1),
         "Free floor, not seen yet", None),
        (lambda p: cv2.rectangle(image, (p[0] - 14, p[1] - 10), (p[0] + 14, p[1] + 10), SEEN_FLOOR_BGR, -1),
         "Floor seen by the camera", "from the blind zone to one step beyond"),
        (lambda p: cv2.rectangle(image, (p[0] - 14, p[1] - 10), (p[0] + 14, p[1] + 10), BLOCKED_BGR, -1),
         "Wall or furniture", None),
        (lambda p: [cv2.circle(image, (p[0] + dx, p[1]), 2, OBSTACLE_FACE_BGR, -1, cv2.LINE_AA) for dx in (-10, 0, 10)],
         "Obstacle face seen", "drawn orange in the Unity map"),
        (lambda p: cv2.arrowedLine(image, (p[0] - 16, p[1]), (p[0] + 16, p[1]), ROUTE_BGR, 3, cv2.LINE_AA, tipLength=0.4),
         "Drone route", "one arrow per forward command"),
        (lambda p: cv2.arrowedLine(image, (p[0] - 16, p[1]), (p[0] + 16, p[1]), RETURN_BGR, 3, cv2.LINE_AA, tipLength=0.4),
         "Way back to the pad", "drawn beside the route it repeats"),
        (lambda p: cv2.rectangle(image, (p[0] - 11, p[1] - 11), (p[0] + 11, p[1] + 11), ROUTE_BGR, -1),
         "Take-off and landing pad", None),
        (lambda p: cv2.circle(image, p, 11, VICTIM_BGR, -1, cv2.LINE_AA), "Victim found", None),
        (lambda p: cv2.circle(image, p, 11, VICTIM_BGR, 3, cv2.LINE_AA), "Victim not found", None),
    ]
    for row, (draw, label, note) in enumerate(entries):
        y = swatch(row, draw)
        put_text(image, label, (left + 50, y + 5))
        if note:
            put_text(image, note, (left + 50, y + 24), 0.42)

    y = top + 40 + len(entries) * 52 + 10
    bar = round(100 * PX_PER_CM)
    cv2.line(image, (left + 6, y), (left + 6 + bar, y), TEXT_BGR, 3)
    for x in (left + 6, left + 6 + bar):
        cv2.line(image, (x, y - 6), (x, y + 6), TEXT_BGR, 2)
    put_text(image, "1 m", (left + 6 + bar + 12, y + 5))


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
    write_image(OUTPUT_DIR / f"{session}_top.png", draw_top_view(messages, session))
    print(f"Saved {len(messages)} messages to {log_path}")

    if args.host:
        stream(messages, args.host, args.port, args.speed)
        print(f"Streamed them to {args.host}:{args.port}")


if __name__ == "__main__":
    main()
