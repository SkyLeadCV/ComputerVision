"""Find where the drone can fly: angular sweep, pockets closed with common tangents, then the drone's size.

The order matters. Lids are fitted to the real feet of the obstacles, so they come before any
erosion, which would shift those feet and merge neighbouring pockets. The drone's size is applied
once to the final free polygon, so lids get the same safety margin as walls. Allowed moves are
then just a query on that result.
"""

from pathlib import Path

import cv2
import numpy as np

from image_io import list_images, read_image, write_image
from rectify_floor import NON_FLOOR_BRIGHTNESS, apply_homography

FRAMES_DIR = Path("frames")
MASKS_DIR = Path("morphology")
TOP_VIEW_DIR = Path("top_view")
OUTPUT_DIR = Path("free_space")

# The opening in apply_morphology.py removes mask features narrower than 15 px, about 0.9 degrees
# with this camera, so a 0.5 degree step guarantees at least one ray enters any surviving pocket.
SWEEP_STEP_DEG = 0.5
SWEEP_HALF_ANGLE_DEG = 45.0
RAY_STEP_CM = 2.0
MAX_RANGE_CM = 600.0

# Above the false jumps between neighbouring rays: up to ~20 cm along a wall seen at a grazing
# angle and ~15 cm from the ragged mask edge at 6 m. A shallower pocket cannot hold the drone anyway.
JUMP_CM = 30.0
# Only the part of each obstacle next to the pocket's mouth forms its point cloud. With the whole
# obstacle the tangent would bridge its far ends and cut off the clear floor in front of it.
CLOUD_RADIUS_CM = 50.0
# Legs and wheels are thinner than this; wider obstacles such as cabinets bound pockets themselves.
ISLAND_MAX_WIDTH_CM = 30.0
MAX_CLOSING_ROUNDS = 10

# A Tello with propeller guards is about 18 cm across.
DRONE_RADIUS_CM = 9.0
SAFETY_MARGIN_CM = 15.0
GRID_CM_PER_PX = 2.0
# Tello limits for a single "forward" command.
MIN_FORWARD_CM = 20
MAX_FORWARD_CM = 500
# Each move is capped and the next frame re-plans it. Far floor is the least reliable (a wall of
# the floor's colour can pass for floor there), and it is seen better once the drone is closer.
STEP_LIMIT_CM = 200
STEP_LIMIT_BGR = (0, 165, 255)
MOVE_ARROW_STEP_DEG = 5.0

STOP_OBSTACLE, STOP_FOV, STOP_RANGE, STOP_LID = 0, 1, 2, 3
STOP_COLORS_BGR = {STOP_OBSTACLE: (0, 0, 255), STOP_FOV: (160, 160, 160), STOP_RANGE: (160, 160, 160), STOP_LID: (0, 255, 255)}
FREE_RAY_BGR = (0, 200, 0)
LEFT_HULL_BGR = (255, 120, 0)
RIGHT_HULL_BGR = (255, 0, 255)
CANDIDATE_BGR = (0, 255, 255)
VIOLATOR_BGR = (0, 0, 255)
CLOUD_COLORS_BGR = [(255, 120, 0), (255, 0, 255), (0, 165, 255), (255, 255, 0), (120, 255, 120), (200, 120, 255)]


def put_label(image: np.ndarray, text: str, origin: tuple[int, int], scale: float = 0.6) -> None:
    # A dark box rather than an outline: OpenCV widens each glyph's advance with the thickness,
    # so a thick outline drifts away from the thin text drawn on top of it in long labels.
    (width, height), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)
    x, y = origin
    cv2.rectangle(image, (x - 3, y - height - 4), (x + width + 3, y + baseline + 2), (0, 0, 0), -1)
    cv2.putText(image, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (255, 255, 255), 1, cv2.LINE_AA)


def floor_canvas(
    frame: np.ndarray, floor_mask: np.ndarray, image_to_floor: np.ndarray,
    x_range: tuple[float, float], y_range: tuple[float, float], width_px: int,
) -> tuple[np.ndarray, np.ndarray]:
    """Render a top view of the given floor box and return it with its floor-to-canvas transform."""
    scale = width_px / (x_range[1] - x_range[0])
    floor_to_canvas = np.array([[scale, 0.0, -scale * x_range[0]], [0.0, -scale, scale * y_range[1]], [0.0, 0.0, 1.0]])
    size = (width_px, int(round(scale * (y_range[1] - y_range[0]))))
    dimmed = np.where(floor_mask[..., None] > 0, frame, (frame * NON_FLOOR_BRIGHTNESS).astype(np.uint8))
    return cv2.warpPerspective(dimmed, floor_to_canvas @ image_to_floor, size), floor_to_canvas


def to_px(transform: np.ndarray, points_cm: np.ndarray) -> np.ndarray:
    return apply_homography(transform, np.atleast_2d(points_cm)).round().astype(np.int32)


def sweep(floor_mask: np.ndarray, floor_to_image: np.ndarray, image_to_floor: np.ndarray) -> dict[str, np.ndarray]:
    """Cast one ray per angle from the point below the camera and stop at the first non-floor pixel."""
    height, width = floor_mask.shape
    # Homography scale is only defined up to sign; a floor point known to be in view fixes which
    # sign means "in front of the camera", so points behind it are not mistaken for visible ones.
    reference_floor = apply_homography(image_to_floor, np.array([[width / 2, height - 1.0]]))[0]
    in_front_sign = np.sign((floor_to_image @ np.append(reference_floor, 1.0))[2])

    distances = np.arange(RAY_STEP_CM, MAX_RANGE_CM + RAY_STEP_CM, RAY_STEP_CM)
    angles, entries, stops, kinds = [], [], [], []
    for angle in np.arange(-SWEEP_HALF_ANGLE_DEG, SWEEP_HALF_ANGLE_DEG + 1e-9, SWEEP_STEP_DEG):
        direction = np.array([np.sin(np.radians(angle)), np.cos(np.radians(angle))])
        points = distances[:, None] * direction
        projected = np.hstack([points, np.ones((len(points), 1))]) @ floor_to_image.T
        pixels = projected[:, :2] / projected[:, 2:]
        visible = (
            (np.sign(projected[:, 2]) == in_front_sign)
            & (pixels[:, 0] >= 0) & (pixels[:, 0] <= width - 1)
            & (pixels[:, 1] >= 0) & (pixels[:, 1] <= height - 1)
        )
        if not visible.any():
            continue
        entry = int(np.argmax(visible))
        columns = np.clip(pixels[:, 0].round().astype(int), 0, width - 1)
        rows = np.clip(pixels[:, 1].round().astype(int), 0, height - 1)
        blocked = ~visible[entry:] | (floor_mask[rows[entry:], columns[entry:]] == 0)

        if blocked.any():
            first_blocked = entry + int(np.argmax(blocked))
            kind = STOP_FOV if not visible[first_blocked] else STOP_OBSTACLE
            stop = max(first_blocked - 1, entry)
        else:
            kind, stop = STOP_RANGE, len(distances) - 1
        angles.append(angle)
        entries.append(distances[entry])
        stops.append(distances[stop])
        kinds.append(kind)

    return {"angles": np.array(angles), "entry": np.array(entries), "stop": np.array(stops), "kind": np.array(kinds)}


def ray_points(angles_deg: np.ndarray, distances: np.ndarray) -> np.ndarray:
    radians = np.radians(angles_deg)
    return np.column_stack([distances * np.sin(radians), distances * np.cos(radians)])


def find_pockets(distances: np.ndarray) -> tuple[list[tuple[int, int]], np.ndarray]:
    """Return (left mouth, right mouth) ray indices of every upward jump directly followed by a downward one.

    Only adjacent jumps are paired: a wall seen at an angle climbs in several jumps, and a stack
    that matched distant jumps paired unrelated ones. Pockets that contain others are found in the
    next closing round, once the inner ones are closed.
    """
    steps = np.diff(distances)
    jumps = np.flatnonzero(np.abs(steps) > JUMP_CM)
    pockets = [(up, down + 1) for up, down in zip(jumps[:-1], jumps[1:]) if steps[up] > 0 and steps[down] < 0]
    return pockets, jumps


def is_unbounded(kinds: np.ndarray, left_mouth: int, right_mouth: int) -> bool:
    """Tell whether most rays inside a pocket never found where the floor ends.

    That is the sign of a mask leaking onto a wall: the wall maps to floor ever farther away. A
    pocket whose rays end on obstacle feet is real floor, such as a gap leading to a far wall; it is
    left open, and the erosion later removes it if the drone does not fit.
    """
    inside = kinds[left_mouth + 1:right_mouth]
    return len(inside) > 0 and np.mean(inside != STOP_OBSTACLE) > 0.5


def pocket_side(
    distances: np.ndarray, stop_points: np.ndarray, jumps: np.ndarray, mouth: int, towards_left: bool, opposite_cm: float
) -> tuple[int, np.ndarray]:
    """Return the mouth and the rays of the obstacle that bounds a pocket on the side given.

    A thin obstacle much closer than the opposite mouth, with jumps away from the camera on both
    sides, stands in front of the pocket (a table wheel or leg) and hides the obstacle that really
    bounds it. It is skipped; it still stops its own rays, so it is not lost.
    """
    steps = np.diff(distances)
    while True:
        if towards_left:
            previous = jumps[jumps < mouth]
            run = np.arange(previous.max() + 1 if len(previous) else 0, mouth + 1)
            is_island = len(previous) > 0 and steps[previous.max()] < 0
            next_mouth = previous.max() if len(previous) else None
        else:
            following = jumps[jumps >= mouth]
            run = np.arange(mouth, following.min() + 1 if len(following) else len(distances))
            is_island = len(following) > 0 and steps[following.min()] > 0
            next_mouth = following.min() + 1 if len(following) else None
        width_cm = np.linalg.norm(stop_points[run[-1]] - stop_points[run[0]])
        in_front = is_island and width_cm < ISLAND_MAX_WIDTH_CM and distances[run].mean() < opposite_cm - JUMP_CM
        if not in_front:
            return mouth, run
        mouth = next_mouth


def obstacle_cloud(kinds: np.ndarray, stop_points: np.ndarray, mouth: int, run: np.ndarray) -> np.ndarray:
    """Return the obstacle stop points of a run that lie near the pocket mouth."""
    run = run[kinds[run] == STOP_OBSTACLE]
    near_mouth = np.linalg.norm(stop_points[run] - stop_points[mouth], axis=1) <= CLOUD_RADIUS_CM
    return stop_points[run[near_mouth]]


def convex_hull(points: np.ndarray) -> np.ndarray:
    return cv2.convexHull(points.astype(np.float32)).reshape(-1, 2).astype(float)


def side(line_start: np.ndarray, line_end: np.ndarray, point: np.ndarray) -> float:
    direction = line_end - line_start
    offset = point - line_start
    return direction[0] * offset[1] - direction[1] * offset[0]


def camera_side_neighbour(hull: np.ndarray, index: int, line_start: np.ndarray, line_end: np.ndarray) -> int | None:
    """Return the hull neighbour of `index` lying between the line and the camera, if any.

    On a convex polygon a line through a vertex is tangent exactly when both neighbours lie on
    the same side, so checking the two neighbours is enough.
    """
    camera = side(line_start, line_end, np.zeros(2))
    best, best_depth = None, 1e-6
    for neighbour in ((index - 1) % len(hull), (index + 1) % len(hull)):
        depth = side(line_start, line_end, hull[neighbour]) * np.sign(camera)
        if depth > best_depth:
            best, best_depth = neighbour, depth
    return best


def common_tangent(left_hull: np.ndarray, right_hull: np.ndarray, a: int, b: int) -> list[dict]:
    """Walk a and b along their hulls until the line a-b is tangent to both; return every state."""
    states = [{"a": a, "b": b, "moving": None, "violator": None}]
    while True:
        moved = False
        for moving in ("a", "b"):
            while True:
                hull = left_hull if moving == "a" else right_hull
                index = a if moving == "a" else b
                violator = camera_side_neighbour(hull, index, left_hull[a], right_hull[b])
                if violator is None:
                    break
                states[-1].update(moving=moving, violator=violator)
                if moving == "a":
                    a = violator
                else:
                    b = violator
                states.append({"a": a, "b": b, "moving": None, "violator": None})
                moved = True
        if not moved:
            return states


def clip_to_lid(angles_deg: np.ndarray, distances: np.ndarray, kinds: np.ndarray, start: np.ndarray, end: np.ndarray) -> None:
    """Shorten every ray between the lid's endpoints so it stops at the lid."""
    endpoint_angles = np.degrees(np.arctan2([start[0], end[0]], [start[1], end[1]]))
    inside = (angles_deg >= endpoint_angles.min() - 1e-9) & (angles_deg <= endpoint_angles.max() + 1e-9)
    directions = ray_points(angles_deg[inside], np.ones(inside.sum()))
    lid = end - start
    # Solve r·u on the lid line: cross(lid, r·u - start) = 0.
    lid_distances = (lid[0] * start[1] - lid[1] * start[0]) / (lid[0] * directions[:, 1] - lid[1] * directions[:, 0])
    shorter = lid_distances < distances[inside] - 1e-6
    indices = np.flatnonzero(inside)[shorter]
    distances[indices] = lid_distances[shorter]
    kinds[indices] = STOP_LID


def bounds_a_ray(angles_deg: np.ndarray, distances: np.ndarray, start: np.ndarray, end: np.ndarray) -> bool:
    probe = distances.copy()
    clip_to_lid(angles_deg, probe, np.zeros(len(probe), int), start - 0.5 * start / np.linalg.norm(start), end - 0.5 * end / np.linalg.norm(end))
    return bool(np.any(probe < distances))


def close_pockets(rays: dict[str, np.ndarray]) -> tuple[np.ndarray, np.ndarray, list[dict]]:
    """Close every unbounded pocket with a lid; return the clipped stop distances, their kinds and one record per lid.

    Each record keeps the clouds, hulls and tangent states of its pocket, so the step images can
    be drawn without the flight code paying for them.
    """
    distances, kinds = rays["stop"].copy(), rays["kind"].copy()
    records = []
    for round_index in range(MAX_CLOSING_ROUNDS):
        pockets, jumps = find_pockets(distances)
        if not pockets:
            break
        stop_points = ray_points(rays["angles"], distances)
        round_records = []
        for pocket_index, (left_mouth, right_mouth) in enumerate(pockets):
            left_mouth, left_run = pocket_side(distances, stop_points, jumps, left_mouth, True, distances[right_mouth])
            right_mouth, right_run = pocket_side(distances, stop_points, jumps, right_mouth, False, distances[left_mouth])
            left_cloud = obstacle_cloud(kinds, stop_points, left_mouth, left_run)
            right_cloud = obstacle_cloud(kinds, stop_points, right_mouth, right_run)
            if len(left_cloud) == 0 or len(right_cloud) == 0 or not is_unbounded(kinds, left_mouth, right_mouth):
                continue
            left_hull, right_hull = convex_hull(left_cloud), convex_hull(right_cloud)
            # Start from the hull vertices closest to the pocket's mouths.
            a = int(np.argmin(np.linalg.norm(left_hull - stop_points[left_mouth], axis=1)))
            b = int(np.argmin(np.linalg.norm(right_hull - stop_points[right_mouth], axis=1)))
            states = common_tangent(left_hull, right_hull, a, b)
            round_records.append({
                "round": round_index, "pocket": pocket_index, "left_cloud": left_cloud, "right_cloud": right_cloud,
                "left_hull": left_hull, "right_hull": right_hull, "states": states,
                "lid": (left_hull[states[-1]["a"]], right_hull[states[-1]["b"]]),
            })

        # Clipping only after every lid of the round is found keeps one pocket's lid from
        # relabelling the rays that form the point cloud of the next pocket.
        if not round_records:
            break
        for record in round_records:
            clip_to_lid(rays["angles"], distances, kinds, *record["lid"])
        records.extend(round_records)
    return distances, kinds, records


def draw_sweep_top(canvas: np.ndarray, to_canvas: np.ndarray, rays: dict, stops: np.ndarray, kinds: np.ndarray) -> np.ndarray:
    entries = ray_points(rays["angles"], rays["entry"])
    ends = ray_points(rays["angles"], stops)
    origin = tuple(to_px(to_canvas, np.zeros(2))[0])
    for entry, end, kind in zip(to_px(to_canvas, entries), to_px(to_canvas, ends), kinds):
        cv2.line(canvas, origin, tuple(entry), (90, 90, 90), 1)
        cv2.line(canvas, tuple(entry), tuple(end), FREE_RAY_BGR, 1)
        cv2.circle(canvas, tuple(end), 3, STOP_COLORS_BGR[kind], -1)
    cv2.circle(canvas, origin, 8, (0, 0, 255), -1)
    return canvas


def draw_sweep_photo(frame: np.ndarray, floor_to_image: np.ndarray, rays: dict) -> np.ndarray:
    overlay = frame.copy()
    entries = to_px(floor_to_image, ray_points(rays["angles"], rays["entry"]))
    ends = to_px(floor_to_image, ray_points(rays["angles"], rays["stop"]))
    for entry, end, kind in zip(entries, ends, rays["kind"]):
        cv2.line(overlay, tuple(entry), tuple(end), FREE_RAY_BGR, 1)
        cv2.circle(overlay, tuple(end), 3, STOP_COLORS_BGR[kind], -1)
    return overlay


def draw_profile(rays: dict, pockets: list[tuple[int, int]], jumps: np.ndarray) -> np.ndarray:
    """Plot the stop distance of each ray against its angle, with jumps and pockets marked."""
    width, height, margin = 1100, 520, 60
    plot = np.full((height, width, 3), 255, np.uint8)
    angles, distances = rays["angles"], rays["stop"]

    def to_plot(angle: float, distance: float) -> tuple[int, int]:
        x = margin + (angle + SWEEP_HALF_ANGLE_DEG) / (2 * SWEEP_HALF_ANGLE_DEG) * (width - 2 * margin)
        y = height - margin - distance / MAX_RANGE_CM * (height - 2 * margin)
        return int(x), int(y)

    for left, right in pockets:
        cv2.rectangle(plot, to_plot(angles[left], MAX_RANGE_CM), to_plot(angles[right], 0), (210, 235, 255), -1)
    for distance in range(0, int(MAX_RANGE_CM) + 1, 100):
        cv2.line(plot, to_plot(-SWEEP_HALF_ANGLE_DEG, distance), to_plot(SWEEP_HALF_ANGLE_DEG, distance), (225, 225, 225), 1)
        cv2.putText(plot, f"{distance // 100} m", (8, to_plot(0, distance)[1] + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
    for angle in range(-40, 41, 10):
        x, y = to_plot(angle, 0)
        cv2.putText(plot, f"{angle}", (x - 12, y + 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
    for index in jumps:
        cv2.line(plot, to_plot(angles[index], distances[index]), to_plot(angles[index + 1], distances[index + 1]), (0, 0, 255), 2)
    for angle, distance, kind in zip(angles, distances, rays["kind"]):
        cv2.circle(plot, to_plot(angle, distance), 2, (0, 140, 0) if kind == STOP_OBSTACLE else (150, 150, 150), -1)
    cv2.putText(plot, "stop distance vs ray angle (deg)   red: jump > 30 cm   orange band: pocket",
                (margin, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)
    return plot


def draw_tangent_state(
    frame: np.ndarray, floor_mask: np.ndarray, image_to_floor: np.ndarray,
    left_hull: np.ndarray, right_hull: np.ndarray, left_cloud: np.ndarray, right_cloud: np.ndarray,
    state: dict, title: str,
) -> np.ndarray:
    """Zoom on one pocket and show the candidate segment a-b and the vertex that breaks tangency."""
    points = np.vstack([left_hull, right_hull])
    margin = 0.25 * max(np.ptp(points[:, 0]), np.ptp(points[:, 1]), 60.0)
    x_range = (points[:, 0].min() - margin, points[:, 0].max() + margin)
    y_range = (points[:, 1].min() - margin, points[:, 1].max() + margin)
    canvas, to_canvas = floor_canvas(frame, floor_mask, image_to_floor, x_range, y_range, 900)

    for cloud, hull, color in ((left_cloud, left_hull, LEFT_HULL_BGR), (right_cloud, right_hull, RIGHT_HULL_BGR)):
        for point in to_px(to_canvas, cloud):
            cv2.circle(canvas, tuple(point), 2, color, -1)
        cv2.polylines(canvas, [to_px(to_canvas, hull)], True, color, 2)

    a_px, b_px = to_px(to_canvas, np.vstack([left_hull[state["a"]], right_hull[state["b"]]]))
    cv2.line(canvas, tuple(a_px), tuple(b_px), CANDIDATE_BGR, 2)
    cv2.circle(canvas, tuple(a_px), 8, CANDIDATE_BGR, -1)
    cv2.circle(canvas, tuple(b_px), 8, CANDIDATE_BGR, -1)
    put_label(canvas, "a", tuple(a_px + [10, -10]), 0.8)
    put_label(canvas, "b", tuple(b_px + [10, -10]), 0.8)
    if state["violator"] is not None:
        hull = left_hull if state["moving"] == "a" else right_hull
        violator = tuple(to_px(to_canvas, hull[state["violator"]])[0])
        cv2.circle(canvas, violator, 12, VIOLATOR_BGR, 3)
        cv2.arrowedLine(canvas, tuple(a_px if state["moving"] == "a" else b_px), violator, VIOLATOR_BGR, 2, tipLength=0.2)

    # The camera is below this crop, so an arrow shows which side of the line counts as "camera side".
    put_label(canvas, title, (10, 30), 0.7)
    put_label(canvas, "camera side: towards the bottom", (10, canvas.shape[0] - 15), 0.6)
    return canvas


def draw_free_space_top(canvas: np.ndarray, to_canvas: np.ndarray, polygon_cm: np.ndarray, lids: list) -> np.ndarray:
    fill = canvas.copy()
    cv2.fillPoly(fill, [to_px(to_canvas, polygon_cm)], (0, 200, 0))
    canvas = cv2.addWeighted(fill, 0.4, canvas, 0.6, 0)
    cv2.polylines(canvas, [to_px(to_canvas, polygon_cm)], True, (0, 255, 0), 2)
    for start, end in lids:
        start_px, end_px = to_px(to_canvas, np.vstack([start, end]))
        cv2.line(canvas, tuple(start_px), tuple(end_px), CANDIDATE_BGR, 3)
        put_label(canvas, f"{np.linalg.norm(end - start):.0f} cm", tuple((start_px + end_px) // 2 + [0, -10]))
    cv2.circle(canvas, tuple(to_px(to_canvas, np.zeros(2))[0]), 8, (0, 0, 255), -1)
    return canvas


def draw_free_space_photo(frame: np.ndarray, polygon_px: np.ndarray, lids_px: list) -> np.ndarray:
    fill = frame.copy()
    cv2.fillPoly(fill, [polygon_px], (0, 200, 0))
    overlay = cv2.addWeighted(fill, 0.4, frame, 0.6, 0)
    cv2.polylines(overlay, [polygon_px], True, (0, 255, 0), 2)
    for start, end in lids_px:
        cv2.line(overlay, tuple(start), tuple(end), CANDIDATE_BGR, 3)
    return overlay


def floor_to_grid() -> np.ndarray:
    """Return the transform from floor cm to a metric top-view grid covering every ray."""
    scale = 1.0 / GRID_CM_PER_PX
    return np.array([[scale, 0.0, MAX_RANGE_CM * scale], [0.0, -scale, MAX_RANGE_CM * scale], [0.0, 0.0, 1.0]])


def free_polygon_from_drone(angles_deg: np.ndarray, distances: np.ndarray) -> np.ndarray:
    """Return the free floor as a polygon that starts at the drone, so the blind zone is part of it."""
    return np.vstack([np.zeros((1, 2)), ray_points(angles_deg, distances)])


def drone_safe_area(free_polygon_cm: np.ndarray) -> np.ndarray:
    """Erode the free polygon by the drone's radius plus margin: where the drone's centre can be.

    This is done in the metric top view rather than on the image mask, because one image pixel
    covers a few cm near the camera but tens of cm far away, so no single image kernel fits.
    Unseen floor counts as blocked, which also keeps the drone's sides inside the field of view.
    """
    grid = free_grid(free_polygon_cm)
    radius_px = int(np.ceil((DRONE_RADIUS_CM + SAFETY_MARGIN_CM) / GRID_CM_PER_PX))
    disc = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * radius_px + 1, 2 * radius_px + 1))
    return cv2.erode(grid, disc)


def allowed_distances(safe_area: np.ndarray, angles_deg: np.ndarray, entry_cm: np.ndarray) -> np.ndarray:
    """Return how far the drone can fly straight along each heading.

    The floor between the drone and the first visible floor (the blind zone) is not seen in this
    frame, but nothing there can hit the drone: the camera sees the floor at the entry, and anything
    reaching the drone's height would stand in that line of sight, so the ray would stop at its entry
    and the heading would get no move. From the entry on, the whole path must stay inside the safe area.
    """
    distances = np.arange(0.0, MAX_RANGE_CM, GRID_CM_PER_PX)
    allowed = np.zeros(len(angles_deg))
    for index, (angle, entry) in enumerate(zip(angles_deg, entry_cm)):
        path = distances[distances >= entry]
        cells = to_px(floor_to_grid(), ray_points(np.full(len(path), angle), path))
        cells[:, 0] = np.clip(cells[:, 0], 0, safe_area.shape[1] - 1)
        cells[:, 1] = np.clip(cells[:, 1], 0, safe_area.shape[0] - 1)
        blocked = safe_area[cells[:, 1], cells[:, 0]] == 0
        if len(path) == 0 or blocked[0]:
            continue
        allowed[index] = path[np.argmax(blocked)] if blocked.any() else path[-1]
    return allowed


def tello_command(angles_deg: np.ndarray, allowed_cm: np.ndarray) -> str:
    """Pick the heading with the longest clear flight, preferring the smallest turn on ties."""
    # With the step limit many headings reach the cap, so the smallest turn wins among them.
    step_cm = np.minimum(allowed_cm, STEP_LIMIT_CM)
    best = max(range(len(angles_deg)), key=lambda i: (np.floor(step_cm[i] / 10), -abs(angles_deg[i])))
    forward = int(min(allowed_cm[best], STEP_LIMIT_CM, MAX_FORWARD_CM))
    if forward < MIN_FORWARD_CM:
        return "no clear heading"
    turn = int(round(angles_deg[best]))
    rotation = f"cw {turn}" if turn > 0 else f"ccw {-turn}" if turn < 0 else "no turn"
    return f"{rotation}, forward {forward}"


def free_grid(free_polygon_cm: np.ndarray) -> np.ndarray:
    size = int(2 * MAX_RANGE_CM / GRID_CM_PER_PX) + 1
    grid = np.zeros((size // 2 + 1, size), np.uint8)
    cv2.fillPoly(grid, [to_px(floor_to_grid(), free_polygon_cm)], 255)
    return grid


def draw_erosion_grid(grid: np.ndarray, safe_area: np.ndarray | None, boundary_cm: np.ndarray, title: str) -> np.ndarray:
    """Show the metric grid, optionally with the erosion result, and the drone disc on a few boundary points."""
    image = cv2.cvtColor(grid, cv2.COLOR_GRAY2BGR)
    if safe_area is not None:
        image[(grid > 0) & (safe_area == 0)] = (0, 0, 255)
        image[safe_area > 0] = (0, 200, 0)
    radius_px = int(round((DRONE_RADIUS_CM + SAFETY_MARGIN_CM) / GRID_CM_PER_PX))
    for point in to_px(floor_to_grid(), boundary_cm):
        cv2.circle(image, tuple(point), radius_px, (0, 220, 255), 1)
    cv2.circle(image, tuple(to_px(floor_to_grid(), np.zeros(2))[0]), 4, (255, 0, 0), -1)
    rows, columns = np.nonzero(grid)
    pad = 2 * radius_px
    image = image[max(rows.min() - pad, 0):rows.max() + pad, max(columns.min() - pad, 0):columns.max() + pad]
    # The grid is 2 cm per pixel; enlarging it makes the 24 cm disc easy to see.
    image = cv2.resize(image, None, fx=3, fy=3, interpolation=cv2.INTER_NEAREST)
    put_label(image, title, (10, 30))
    return image


def draw_erosion_photo(frame: np.ndarray, floor_to_image: np.ndarray, grid: np.ndarray, safe_area: np.ndarray) -> np.ndarray:
    """Project the erosion back onto the photo: green is kept, red is the band removed."""
    grid_to_image = floor_to_image @ np.linalg.inv(floor_to_grid())
    size = (frame.shape[1], frame.shape[0])
    free = cv2.warpPerspective(grid, grid_to_image, size, flags=cv2.INTER_NEAREST)
    safe = cv2.warpPerspective(safe_area, grid_to_image, size, flags=cv2.INTER_NEAREST)
    fill = frame.copy()
    fill[(free > 0) & (safe == 0)] = (0, 0, 255)
    fill[safe > 0] = (0, 200, 0)
    overlay = cv2.addWeighted(fill, 0.45, frame, 0.55, 0)
    put_label(overlay, "green: drone centre can be here   red: removed by the erosion", (10, 30))
    return overlay


def draw_safe_area_top(canvas: np.ndarray, to_canvas: np.ndarray, free_polygon_cm: np.ndarray, safe_area: np.ndarray) -> np.ndarray:
    # Map the grid onto the canvas so both layers share the same floor coordinates.
    grid_to_canvas = to_canvas @ np.linalg.inv(floor_to_grid())
    safe = cv2.warpPerspective(safe_area, grid_to_canvas, (canvas.shape[1], canvas.shape[0]), flags=cv2.INTER_NEAREST)
    fill = canvas.copy()
    cv2.fillPoly(fill, [to_px(to_canvas, free_polygon_cm)], (0, 120, 0))
    fill[safe > 0] = (0, 230, 0)
    canvas = cv2.addWeighted(fill, 0.5, canvas, 0.5, 0)
    cv2.polylines(canvas, [to_px(to_canvas, free_polygon_cm)], True, (0, 255, 0), 1)
    cv2.circle(canvas, tuple(to_px(to_canvas, np.zeros(2))[0]), 8, (0, 0, 255), -1)
    put_label(canvas, "dark green: clear floor", (10, 30))
    put_label(canvas, f"bright green: safe for the drone centre ({DRONE_RADIUS_CM + SAFETY_MARGIN_CM:.0f} cm from obstacles)", (10, 60))
    return canvas


def move_arrows(angles_deg: np.ndarray, allowed_cm: np.ndarray) -> list[tuple[float, float]]:
    picks = np.flatnonzero(np.isclose(np.mod(angles_deg, MOVE_ARROW_STEP_DEG), 0) | np.isclose(np.mod(angles_deg, MOVE_ARROW_STEP_DEG), MOVE_ARROW_STEP_DEG))
    return [(angles_deg[i], allowed_cm[i]) for i in picks]


def draw_moves_top(canvas: np.ndarray, to_canvas: np.ndarray, angles_deg: np.ndarray, allowed_cm: np.ndarray, command: str) -> np.ndarray:
    origin = tuple(to_px(to_canvas, np.zeros(2))[0])
    radius = int(round(STEP_LIMIT_CM * to_canvas[0, 0]))
    cv2.circle(canvas, origin, radius, STEP_LIMIT_BGR, 2)
    for angle, distance in move_arrows(angles_deg, allowed_cm):
        if distance >= MIN_FORWARD_CM:
            step = min(distance, STEP_LIMIT_CM)
            step_end, free_end = to_px(to_canvas, ray_points(np.array([angle, angle]), np.array([step, distance])))
            # Thin grey: floor that is clear but left for the next frame to confirm.
            cv2.line(canvas, tuple(step_end), tuple(free_end), (200, 200, 200), 1)
            cv2.arrowedLine(canvas, origin, tuple(step_end), (0, 230, 0), 2, tipLength=0.05)
            put_label(canvas, f"{distance / 100:.1f}", (free_end[0] - 15, free_end[1] - 8), 0.45)
        else:
            end = tuple(to_px(to_canvas, ray_points(np.array([angle]), np.array([60.0])))[0])
            cv2.line(canvas, origin, end, (0, 0, 255), 2)
    put_label(canvas, f"best: {command}", (10, 30))
    put_label(canvas, f"orange: {STEP_LIMIT_CM / 100:.0f} m step limit   green: next move   grey: clear (m), re-checked next frame", (10, 60), 0.5)
    return canvas


def draw_moves_photo(
    frame: np.ndarray, floor_to_image: np.ndarray, angles_deg: np.ndarray, entry_cm: np.ndarray, allowed_cm: np.ndarray, command: str
) -> np.ndarray:
    overlay = frame.copy()
    # The step limit is a circle on the floor; only its visible part is drawn.
    visible = entry_cm <= STEP_LIMIT_CM
    arc = to_px(floor_to_image, ray_points(angles_deg[visible], np.full(visible.sum(), float(STEP_LIMIT_CM))))
    cv2.polylines(overlay, [arc], False, STEP_LIMIT_BGR, 2)
    entries = dict(zip(angles_deg, entry_cm))
    for angle, distance in move_arrows(angles_deg, allowed_cm):
        if distance < MIN_FORWARD_CM:
            continue
        step = min(distance, STEP_LIMIT_CM)
        start, step_end, free_end = to_px(floor_to_image, ray_points(np.full(3, angle), np.array([entries[angle], step, distance])))
        cv2.line(overlay, tuple(step_end), tuple(free_end), (200, 200, 200), 1)
        if step > entries[angle]:
            cv2.arrowedLine(overlay, tuple(start), tuple(step_end), (0, 230, 0), 2, tipLength=0.1)
        put_label(overlay, f"{distance / 100:.1f} m", (free_end[0] - 20, free_end[1] - 8), 0.45)
    put_label(overlay, f"best: {command}", (10, 30))
    put_label(overlay, f"orange: {STEP_LIMIT_CM / 100:.0f} m step limit   green: next move   grey: clear, re-checked next frame", (10, 60), 0.5)
    return overlay


def process(frame_name: str) -> None:
    frame = read_image(FRAMES_DIR / f"{frame_name}.jpg")
    floor_mask = read_image(MASKS_DIR / f"{frame_name}_mask.png", cv2.IMREAD_GRAYSCALE)
    with np.load(TOP_VIEW_DIR / f"{frame_name}_floor.npz") as floor:
        image_to_floor = floor["image_to_floor"]
    floor_to_image = np.linalg.inv(image_to_floor)
    out = OUTPUT_DIR / frame_name
    out.mkdir(parents=True, exist_ok=True)

    full_view = (-MAX_RANGE_CM / 2, MAX_RANGE_CM / 2), (0.0, MAX_RANGE_CM)

    # Step 1: angular sweep.
    rays = sweep(floor_mask, floor_to_image, image_to_floor)
    canvas, to_canvas = floor_canvas(frame, floor_mask, image_to_floor, *full_view, 900)
    write_image(out / "1_sweep_top.jpg", draw_sweep_top(canvas, to_canvas, rays, rays["stop"], rays["kind"]))
    write_image(out / "1_sweep_photo.jpg", draw_sweep_photo(frame, floor_to_image, rays))

    # Step 2: pockets as jumps in the distance profile.
    pockets, jumps = find_pockets(rays["stop"])
    write_image(out / "2_profile.jpg", draw_profile(rays, pockets, jumps))

    # Steps 3-4: point clouds on each side of every pocket, then their common tangent.
    distances, kinds, records = close_pockets(rays)
    clouds_canvas, _ = floor_canvas(frame, floor_mask, image_to_floor, *full_view, 900)
    for record in records:
        left_hull, right_hull = record["left_hull"], record["right_hull"]
        if record["round"] == 0:
            pocket_index = record["pocket"]
            for cloud, hull, color_index in ((record["left_cloud"], left_hull, 2 * pocket_index), (record["right_cloud"], right_hull, 2 * pocket_index + 1)):
                color = CLOUD_COLORS_BGR[color_index % len(CLOUD_COLORS_BGR)]
                for point in to_px(to_canvas, cloud):
                    cv2.circle(clouds_canvas, tuple(point), 3, color, -1)
                cv2.polylines(clouds_canvas, [to_px(to_canvas, hull)], True, color, 2)

        label = f"{record['round'] + 1}{chr(ord('a') + record['pocket'])}"
        for step, state in enumerate(record["states"]):
            if state["violator"] is not None:
                title = f"pocket {label}, step {step}: {state['moving']} is not tangent (red vertex is on the camera side)"
            else:
                length = np.linalg.norm(right_hull[state["b"]] - left_hull[state["a"]])
                title = f"pocket {label}, step {step}: a and b both tangent -> common tangent, {length:.0f} cm"
            write_image(
                out / f"4_pocket{label}_step{step}.jpg",
                draw_tangent_state(frame, floor_mask, image_to_floor, left_hull, right_hull, record["left_cloud"], record["right_cloud"], state, title),
            )
        start, end = record["lid"]
        print(f"{frame_name}: pocket {label} lid {np.linalg.norm(end - start):.0f} cm after {len(record['states']) - 1} moves")
    write_image(out / "3_clouds.jpg", clouds_canvas)

    # Step 5: free polygon from the near entry points and the clipped stop points.
    polygon_cm = np.vstack([ray_points(rays["angles"], rays["entry"]), ray_points(rays["angles"], distances)[::-1]])
    polygon_px = to_px(floor_to_image, polygon_cm)
    # A lid from an early round can end up behind a later one; only lids still bounding a ray are shown.
    lids = [record["lid"] for record in records if bounds_a_ray(rays["angles"], distances, *record["lid"])]
    lids_px = [tuple(to_px(floor_to_image, np.vstack(lid))) for lid in lids]
    np.savez(out / "free_space.npz", polygon_cm=polygon_cm, polygon_px=polygon_px, angles_deg=rays["angles"], free_distance_cm=distances)

    canvas, to_canvas = floor_canvas(frame, floor_mask, image_to_floor, *full_view, 900)
    write_image(out / "5_free_top.jpg", draw_free_space_top(canvas, to_canvas, polygon_cm, lids))
    write_image(out / "5_free_photo.jpg", draw_free_space_photo(frame, polygon_px, lids_px))

    # Step 6: the drone's size.
    free_polygon_cm = free_polygon_from_drone(rays["angles"], distances)
    safe_area = drone_safe_area(free_polygon_cm)
    grid = free_grid(free_polygon_cm)
    examples = ray_points(rays["angles"], distances)[:: int(10 / SWEEP_STEP_DEG)]
    write_image(out / "6a_free_grid.jpg", draw_erosion_grid(grid, None, examples, "free polygon on a 2 cm/px grid; yellow: drone disc (24 cm)"))
    write_image(out / "6b_erosion_grid.jpg", draw_erosion_grid(grid, safe_area, examples, "after erosion: green kept, red removed"))
    write_image(out / "6c_erosion_photo.jpg", draw_erosion_photo(frame, floor_to_image, grid, safe_area))
    canvas, to_canvas = floor_canvas(frame, floor_mask, image_to_floor, *full_view, 900)
    write_image(out / "6d_safe_area_top.jpg", draw_safe_area_top(canvas, to_canvas, free_polygon_cm, safe_area))

    # Step 7: how far the drone can fly along each heading.
    allowed_cm = allowed_distances(safe_area, rays["angles"], rays["entry"])
    command = tello_command(rays["angles"], allowed_cm)
    canvas, to_canvas = floor_canvas(frame, floor_mask, image_to_floor, *full_view, 900)
    write_image(out / "7_moves_top.jpg", draw_moves_top(canvas, to_canvas, rays["angles"], allowed_cm, command))
    write_image(out / "7_moves_photo.jpg", draw_moves_photo(frame, floor_to_image, rays["angles"], rays["entry"], allowed_cm, command))
    np.savez(out / "moves.npz", angles_deg=rays["angles"], allowed_cm=allowed_cm)
    print(f"{frame_name}: {command}")


def main() -> None:
    # Only frames with a reference tile have a floor scale, so they are the only ones measurable.
    for floor_path in list_images(TOP_VIEW_DIR, "frame_*_floor.npz"):
        process(floor_path.stem.removesuffix("_floor"))
    print(f"Saved step images to {OUTPUT_DIR}/")


if __name__ == "__main__":
    main()
