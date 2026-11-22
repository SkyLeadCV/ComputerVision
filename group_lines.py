"""Group floor line segments into the two tile joint families by vanishing point."""

from pathlib import Path

import cv2
import numpy as np

from image_io import list_images, read_image, write_image

FRAMES_DIR = Path("frames")
LINES_DIR = Path("lines")
OUTPUT_DIR = Path("families")

# The floor grid has two directions of joints. Under perspective, all joints of one
# direction meet at a single vanishing point, which is a far more reliable grouping
# cue than the raw segment angle: parallel joints appear at very different angles
# in the image depending on where they lie on the floor.
NUM_FAMILIES = 2

# A segment supports a vanishing point if it points at it within this angle.
INLIER_ANGLE_DEG = 2.0
# A family needs at least this many segments; fewer are treated as clutter.
MIN_FAMILY_SIZE = 2
# Above this many segment pairs, hypotheses are sampled instead of enumerated.
MAX_HYPOTHESES = 5000
RANDOM_SEED = 0

FAMILY_COLORS_BGR = ((0, 0, 255), (255, 0, 0))
UNASSIGNED_COLOR_BGR = (128, 128, 128)
LINE_THICKNESS_PX = 3


def normalization_matrix(image_shape: tuple[int, ...]) -> np.ndarray:
    """Map pixel coordinates to roughly [-1, 1] for numerically stable cross products."""
    height, width = image_shape[:2]
    scale = 2.0 / max(height, width)
    return np.array(
        [[scale, 0.0, -scale * width / 2], [0.0, scale, -scale * height / 2], [0.0, 0.0, 1.0]]
    )


def to_homogeneous_lines(segments: np.ndarray, norm: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return unit-normal homogeneous lines, midpoints and unit directions in normalized coordinates."""
    ones = np.ones((len(segments), 1))
    p1 = np.hstack([segments[:, 0:2], ones]) @ norm.T
    p2 = np.hstack([segments[:, 2:4], ones]) @ norm.T
    lines = np.cross(p1, p2)
    lines /= np.linalg.norm(lines[:, :2], axis=1, keepdims=True)
    midpoints = (p1[:, :2] + p2[:, :2]) / 2
    directions = p2[:, :2] - p1[:, :2]
    directions /= np.linalg.norm(directions, axis=1, keepdims=True)
    return lines, midpoints, directions


def angular_errors(vp: np.ndarray, midpoints: np.ndarray, directions: np.ndarray) -> np.ndarray:
    """Angle (rad) between each segment and the ray from its midpoint to the vanishing point.

    Works for vanishing points at infinity (vp[2] == 0), where the ray is the vp direction.
    """
    to_vp = vp[:2] - midpoints * vp[2]
    norms = np.linalg.norm(to_vp, axis=1)
    cos = np.abs(np.sum(to_vp * directions, axis=1)) / np.maximum(norms, 1e-12)
    return np.arccos(np.clip(cos, 0.0, 1.0))


def candidate_pairs(count: int, rng: np.random.Generator) -> np.ndarray:
    i, j = np.triu_indices(count, k=1)
    pairs = np.stack([i, j], axis=1)
    if len(pairs) > MAX_HYPOTHESES:
        pairs = pairs[rng.choice(len(pairs), MAX_HYPOTHESES, replace=False)]
    return pairs


def refine_vanishing_point(lines: np.ndarray, weights: np.ndarray) -> np.ndarray:
    """Least-squares point closest to all inlier lines (null vector of the weighted line matrix)."""
    _, _, vt = np.linalg.svd(lines * weights[:, None])
    return vt[-1]


def find_vanishing_point(
    lines: np.ndarray, midpoints: np.ndarray, directions: np.ndarray, lengths: np.ndarray,
    rng: np.random.Generator,
) -> tuple[np.ndarray | None, np.ndarray]:
    """RANSAC: the intersection of two segments supported by the most segment length wins."""
    threshold = np.deg2rad(INLIER_ANGLE_DEG)
    best_vp, best_inliers, best_score = None, np.zeros(len(lines), bool), 0.0

    for i, j in candidate_pairs(len(lines), rng):
        vp = np.cross(lines[i], lines[j])
        if np.linalg.norm(vp) < 1e-12:  # identical lines give no intersection
            continue
        inliers = angular_errors(vp, midpoints, directions) < threshold
        # Weighting by length lets long, reliable joints outvote short clutter.
        score = lengths[inliers].sum()
        if score > best_score:
            best_vp, best_inliers, best_score = vp, inliers, score

    if best_vp is None or best_inliers.sum() < MIN_FAMILY_SIZE:
        return None, np.zeros(len(lines), bool)

    vp = refine_vanishing_point(lines[best_inliers], lengths[best_inliers])
    inliers = angular_errors(vp, midpoints, directions) < threshold
    return vp, inliers


def group_by_vanishing_point(
    segments: np.ndarray, image_shape: tuple[int, ...]
) -> tuple[list[np.ndarray], np.ndarray]:
    """Return the segment families and their vanishing points in homogeneous pixel coordinates.

    Families are ordered by total segment length. A family that is not found is an
    empty (0, 4) array and its vanishing point row is all zeros. Segments that fit no
    family (furniture, clutter) are left out.
    """
    norm = normalization_matrix(image_shape)
    families = [np.empty((0, 4), segments.dtype) for _ in range(NUM_FAMILIES)]
    vanishing_points = np.zeros((NUM_FAMILIES, 3))
    if len(segments) < MIN_FAMILY_SIZE:
        return families, vanishing_points

    lines, midpoints, directions = to_homogeneous_lines(segments.astype(np.float64), norm)
    lengths = np.hypot(segments[:, 2] - segments[:, 0], segments[:, 3] - segments[:, 1]).astype(np.float64)
    remaining = np.arange(len(segments))
    rng = np.random.default_rng(RANDOM_SEED)

    # Sequential RANSAC: once the dominant family is removed, the second one dominates.
    for family_index in range(NUM_FAMILIES):
        if len(remaining) < MIN_FAMILY_SIZE:
            break
        vp, inliers = find_vanishing_point(
            lines[remaining], midpoints[remaining], directions[remaining], lengths[remaining], rng
        )
        if vp is None:
            break
        families[family_index] = segments[remaining[inliers]]
        vp_pixels = np.linalg.solve(norm, vp)
        vanishing_points[family_index] = vp_pixels / np.linalg.norm(vp_pixels)
        remaining = remaining[~inliers]

    return families, vanishing_points


def draw_families(frame: np.ndarray, segments: np.ndarray, families: list[np.ndarray]) -> np.ndarray:
    overlay = frame.copy()
    for x1, y1, x2, y2 in segments:
        cv2.line(overlay, (int(x1), int(y1)), (int(x2), int(y2)), UNASSIGNED_COLOR_BGR, 1)
    for family, color in zip(families, FAMILY_COLORS_BGR):
        for x1, y1, x2, y2 in family:
            cv2.line(overlay, (int(x1), int(y1)), (int(x2), int(y2)), color, LINE_THICKNESS_PX)
    return overlay


def main() -> None:
    OUTPUT_DIR.mkdir(exist_ok=True)

    for lines_path in list_images(LINES_DIR, "frame_*_lines.npy"):
        frame_name = lines_path.stem.removesuffix("_lines")
        frame = read_image(FRAMES_DIR / f"{frame_name}.jpg")
        segments = np.load(lines_path)

        families, vanishing_points = group_by_vanishing_point(segments, frame.shape)

        for index, family in enumerate(families):
            np.save(OUTPUT_DIR / f"{frame_name}_family_{index}.npy", family)
        np.save(OUTPUT_DIR / f"{frame_name}_vanishing_points.npy", vanishing_points)
        write_image(OUTPUT_DIR / f"{frame_name}_families.jpg", draw_families(frame, segments, families))

    print(f"Saved line families to {OUTPUT_DIR}/")


if __name__ == "__main__":
    main()
