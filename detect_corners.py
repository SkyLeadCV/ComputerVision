"""Intersect the two joint families into tile corners and select one complete tile per frame."""

from pathlib import Path

import cv2
import numpy as np

from image_io import list_images, read_image, write_image

FRAMES_DIR = Path("frames")
EDGES_DIR = Path("edges")
FAMILIES_DIR = Path("families")
CALIBRATION_FILE = Path("calibration/camera_calibration.npz")
OUTPUT_DIR = Path("corners")

# Hough often splits one joint into several collinear segments; those closer than this
# to an existing joint line are merged into it.
JOINT_MERGE_DISTANCE_PX = 10

# Tile joints meet at right angles on the floor. When the back-projected directions of
# the two families deviate further than this, the grouping picked up furniture edges
# and the frame is skipped rather than producing a wrong tile.
MIN_FAMILY_ANGLE_RAD = np.deg2rad(80)

# Joint lines are infinite, so an intersection can land where no joint was seen
# (behind furniture or off the floor). A side must lie on edge pixels along at least
# this fraction of its length for the tile outline to count as observed.
MIN_SIDE_EDGE_COVERAGE = 0.6
SIDE_SAMPLE_COUNT = 50
# Allows ~3 px between a fitted joint line and the edge pixels it came from.
EDGE_TOLERANCE_KERNEL = np.ones((7, 7), np.uint8)

# Tiles are square on the floor. A candidate whose rectified sides differ more than this
# spans two tiles because the joint between them was not detected.
MAX_SIDE_RATIO = 1.3

JOINT_COLORS_BGR = ((0, 0, 255), (255, 0, 0))
INTERSECTION_COLOR_BGR = (0, 255, 255)
TILE_COLOR_BGR = (0, 255, 0)
LINE_DRAW_EXTENT_PX = 5000


def to_homogeneous(points: np.ndarray) -> np.ndarray:
    return np.hstack([points, np.ones((len(points), 1))])


def normalize_line(line: np.ndarray) -> np.ndarray:
    """Scale (a, b, c) so that a^2 + b^2 = 1, making a·x + b·y + c a distance in pixels."""
    return line / np.linalg.norm(line[:2])


def intersect(line_a: np.ndarray, line_b: np.ndarray) -> np.ndarray:
    point = np.cross(line_a, line_b)
    if abs(point[2]) < 1e-12:
        return np.array([np.inf, np.inf])
    return point[:2] / point[2]


def merge_into_joints(segments: np.ndarray, vanishing_point: np.ndarray) -> np.ndarray:
    """Merge a family's segments into one line per tile joint, all through its vanishing point."""
    midpoints = (segments[:, :2] + segments[:, 2:]) / 2
    lengths = np.linalg.norm(segments[:, 2:] - segments[:, :2], axis=1)
    joint_lines, member_lists = [], []

    # Longest segments first, so each joint is anchored on its most reliable piece.
    for index in np.argsort(-lengths):
        midpoint = np.append(midpoints[index], 1.0)
        for joint_line, members in zip(joint_lines, member_lists):
            if abs(joint_line @ midpoint) < JOINT_MERGE_DISTANCE_PX:
                members.append(index)
                break
        else:
            joint_lines.append(normalize_line(np.cross(vanishing_point, midpoint)))
            member_lists.append([index])

    # Each joint is re-fitted through the vanishing point so every line of a family meets
    # at exactly the same point, as parallel joints on the floor must.
    joints = []
    for members in member_lists:
        centre = np.average(midpoints[members], axis=0, weights=lengths[members])
        joints.append(normalize_line(np.cross(vanishing_point, np.append(centre, 1.0))))
    return np.array(joints)


def sort_joints(joints: np.ndarray, reference_line: np.ndarray) -> np.ndarray:
    """Order joints by where they cross a line of the other family, so neighbours are adjacent."""
    crossings = np.array([intersect(joint, reference_line) for joint in joints])
    reference_direction = np.array([-reference_line[1], reference_line[0]])
    return joints[np.argsort(crossings @ reference_direction)]


def floor_basis(vanishing_points: np.ndarray, inverse_camera_matrix: np.ndarray) -> np.ndarray:
    """Return the 3D joint directions and the floor normal as rows, in camera coordinates."""
    first_direction, second_direction = (
        direction / np.linalg.norm(direction) for direction in vanishing_points @ inverse_camera_matrix.T
    )
    normal = np.cross(first_direction, second_direction)
    return np.array([first_direction, second_direction, normal / np.linalg.norm(normal)])


def angle_between_families(basis: np.ndarray) -> float:
    return float(np.arccos(min(1.0, abs(basis[0] @ basis[1]))))


def rectify(points: np.ndarray, inverse_camera_matrix: np.ndarray, basis: np.ndarray) -> np.ndarray:
    """Map image points onto the floor plane, in joint-aligned coordinates up to an unknown scale."""
    rays = to_homogeneous(points) @ inverse_camera_matrix.T
    projections = rays @ basis.T
    return projections[:, :2] / projections[:, 2:]


def edge_coverage(start: np.ndarray, end: np.ndarray, edges: np.ndarray) -> float:
    samples = np.linspace(start, end, SIDE_SAMPLE_COUNT).round().astype(int)
    # A side can reach the image border, where rounding lands a sample one pixel past it; what the image does not
    # show is no edge, so such samples count as uncovered instead of indexing outside the image.
    height, width = edges.shape[:2]
    inside = (samples[:, 0] >= 0) & (samples[:, 0] < width) & (samples[:, 1] >= 0) & (samples[:, 1] < height)
    covered = np.zeros(len(samples), bool)
    covered[inside] = edges[samples[inside, 1], samples[inside, 0]] > 0
    return float(np.mean(covered))


def tile_corners(joints_a: np.ndarray, joints_b: np.ndarray, i: int, j: int) -> np.ndarray:
    """Return the four corners of the cell between joints i, i+1 and j, j+1, in polygon order."""
    return np.array([
        intersect(joints_a[i], joints_b[j]),
        intersect(joints_a[i], joints_b[j + 1]),
        intersect(joints_a[i + 1], joints_b[j + 1]),
        intersect(joints_a[i + 1], joints_b[j]),
    ])


def is_valid_tile(
    corners: np.ndarray, edges: np.ndarray, inverse_camera_matrix: np.ndarray, basis: np.ndarray
) -> bool:
    height, width = edges.shape
    if not np.all((corners >= 0) & (corners < [width, height])):
        return False

    sides = list(zip(corners, np.roll(corners, -1, axis=0)))
    if min(edge_coverage(start, end, edges) for start, end in sides) < MIN_SIDE_EDGE_COVERAGE:
        return False

    floor_corners = rectify(corners, inverse_camera_matrix, basis)
    side_lengths = np.linalg.norm(np.roll(floor_corners, -1, axis=0) - floor_corners, axis=1)
    side_ratio = (side_lengths[0] + side_lengths[2]) / (side_lengths[1] + side_lengths[3])
    return max(side_ratio, 1 / side_ratio) <= MAX_SIDE_RATIO


def find_tile(
    joints_a: np.ndarray, joints_b: np.ndarray, edges: np.ndarray, inverse_camera_matrix: np.ndarray, basis: np.ndarray
) -> np.ndarray | None:
    """Return the corners of the largest valid tile, or None if no candidate passes."""
    best_corners, best_area = None, 0.0
    for i in range(len(joints_a) - 1):
        for j in range(len(joints_b) - 1):
            corners = tile_corners(joints_a, joints_b, i, j)
            if not is_valid_tile(corners, edges, inverse_camera_matrix, basis):
                continue
            # The largest tile in the image is the closest one, whose corners carry the
            # smallest relative error for the distance estimate.
            area = cv2.contourArea(corners.astype(np.float32))
            if area > best_area:
                best_corners, best_area = corners, area
    return best_corners


def detect_tile(
    families: np.lib.npyio.NpzFile, edges: np.ndarray, inverse_camera_matrix: np.ndarray
) -> tuple[np.ndarray | None, list[np.ndarray]]:
    """Return the selected tile corners (or None) and the joint lines of each family."""
    vanishing_points = families["vanishing_points"]
    if len(vanishing_points) < 2:
        return None, []

    basis = floor_basis(vanishing_points, inverse_camera_matrix)
    if angle_between_families(basis) < MIN_FAMILY_ANGLE_RAD:
        return None, []

    segments, labels = families["segments"], families["labels"]
    joints_a = merge_into_joints(segments[labels == 0], vanishing_points[0])
    joints_b = merge_into_joints(segments[labels == 1], vanishing_points[1])
    if len(joints_a) < 2 or len(joints_b) < 2:
        return None, [joints_a, joints_b]

    joints_a = sort_joints(joints_a, joints_b[0])
    joints_b = sort_joints(joints_b, joints_a[0])
    return find_tile(joints_a, joints_b, edges, inverse_camera_matrix, basis), [joints_a, joints_b]


def draw_tile(frame: np.ndarray, joint_families: list[np.ndarray], corners: np.ndarray | None) -> np.ndarray:
    overlay = frame.copy()
    for joints, color in zip(joint_families, JOINT_COLORS_BGR):
        for a, b, c in joints:
            closest_point = -c * np.array([a, b])
            direction = np.array([-b, a]) * LINE_DRAW_EXTENT_PX
            start, end = (closest_point - direction).astype(int), (closest_point + direction).astype(int)
            cv2.line(overlay, tuple(start), tuple(end), color, 1)

    if len(joint_families) == 2:
        for joint_a in joint_families[0]:
            for joint_b in joint_families[1]:
                point = intersect(joint_a, joint_b)
                if np.all(np.abs(point) < LINE_DRAW_EXTENT_PX):
                    cv2.circle(overlay, tuple(point.astype(int)), 4, INTERSECTION_COLOR_BGR, -1)

    if corners is not None:
        cv2.polylines(overlay, [corners.round().astype(np.int32)], True, TILE_COLOR_BGR, 3)
    return overlay


def main() -> None:
    OUTPUT_DIR.mkdir(exist_ok=True)
    inverse_camera_matrix = np.linalg.inv(np.load(CALIBRATION_FILE)["camera_matrix"])

    frames_with_tile = 0
    families_paths = list_images(FAMILIES_DIR, "frame_*_families.npz")
    for families_path in families_paths:
        frame_name = families_path.stem.removesuffix("_families")
        frame = read_image(FRAMES_DIR / f"{frame_name}.jpg")
        edges = read_image(EDGES_DIR / f"{frame_name}_edges.png", cv2.IMREAD_GRAYSCALE)
        edges = cv2.dilate(edges, EDGE_TOLERANCE_KERNEL)

        with np.load(families_path) as families:
            corners, joint_families = detect_tile(families, edges, inverse_camera_matrix)

        frames_with_tile += corners is not None
        np.save(OUTPUT_DIR / f"{frame_name}_tile.npy", corners if corners is not None else np.empty((0, 2)))
        write_image(OUTPUT_DIR / f"{frame_name}_tile.jpg", draw_tile(frame, joint_families, corners))

    print(f"Tile found in {frames_with_tile} of {len(families_paths)} frames, saved to {OUTPUT_DIR}/")


if __name__ == "__main__":
    main()
