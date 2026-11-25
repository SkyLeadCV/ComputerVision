"""Group floor line segments into the two tile-joint families by vanishing point."""

from itertools import combinations
from pathlib import Path

import cv2
import numpy as np

from image_io import list_images, read_image, write_image

FRAMES_DIR = Path("frames")
LINES_DIR = Path("lines")
OUTPUT_DIR = Path("families")

FAMILY_COUNT = 2
OUTLIER_LABEL = -1

# Two segments always agree with the vanishing point they define, so a third
# agreeing segment is required before a family is trusted.
MIN_FAMILY_SEGMENTS = 3

# Maximum angle between a segment and the direction from its midpoint to the
# vanishing point for the segment to count as converging there.
ANGULAR_TOLERANCE_RAD = np.deg2rad(2.0)

# Floor vanishing points lie on the horizon, whereas vertical furniture edges
# converge far above or below the image and would otherwise form a family of their
# own. A candidate is kept if it is within one image height of the centre row, or
# far enough to the side that its elevation seen from the image centre is shallow.
HORIZON_BAND_HEIGHT_RATIO = 1.0
MAX_HORIZON_ELEVATION_RAD = np.deg2rad(30)

FAMILY_COLORS_BGR = ((0, 0, 255), (255, 0, 0))
OUTLIER_COLOR_BGR = (0, 255, 255)
LINE_THICKNESS_PX = 3


def to_homogeneous_lines(segments: np.ndarray) -> np.ndarray:
    """Return the line through each segment as (a, b, c), normalised so a^2 + b^2 = 1."""
    ones = np.ones((len(segments), 1))
    starts = np.hstack([segments[:, :2], ones])
    ends = np.hstack([segments[:, 2:], ones])
    lines = np.cross(starts, ends)
    return lines / np.linalg.norm(lines[:, :2], axis=1, keepdims=True)


def angular_errors(segments: np.ndarray, vanishing_point: np.ndarray) -> np.ndarray:
    midpoints = (segments[:, :2] + segments[:, 2:]) / 2
    directions = segments[:, 2:] - segments[:, :2]
    # Written in homogeneous form so it also holds for vanishing points at infinity (w = 0).
    towards_vanishing_point = vanishing_point[:2] - vanishing_point[2] * midpoints
    norms = np.linalg.norm(directions, axis=1) * np.linalg.norm(towards_vanishing_point, axis=1)
    cosines = np.abs(np.sum(directions * towards_vanishing_point, axis=1)) / np.maximum(norms, 1e-12)
    return np.arccos(np.clip(cosines, 0.0, 1.0))


def lies_near_horizon(vanishing_point: np.ndarray, image_size: tuple[int, int]) -> bool:
    width, height = image_size
    x, y, w = vanishing_point
    # Both sides are scaled by w instead of dividing by it, so points at infinity are handled too.
    vertical_offset = abs(y - w * height / 2)
    horizontal_offset = abs(x - w * width / 2)
    return vertical_offset <= (
        w * HORIZON_BAND_HEIGHT_RATIO * height + horizontal_offset * np.tan(MAX_HORIZON_ELEVATION_RAD)
    )


def is_crossing_on_floor(vanishing_point: np.ndarray, segments: np.ndarray, image_size: tuple[int, int]) -> bool:
    """A point inside the image and below some of its segments: where joints cross on the floor, not where they vanish.

    Every line of both joint families passes through such a crossing, so it can outvote the true vanishing points
    and mix the two families, as in the first real explorations with the joints seen diagonally. A true vanishing
    point inside the image lies on the horizon, above all the floor. Far ones are left alone: a camera held with some
    roll puts the far end of the horizon below the image's floor rows.
    """
    width, height = image_size
    x, y, w = vanishing_point
    inside = 0 <= x <= w * width and 0 <= y <= w * height
    return bool(inside and y > w * segments[:, [1, 3]].min())


def find_dominant_family(
    segments: np.ndarray, image_size: tuple[int, int]
) -> tuple[np.ndarray | None, np.ndarray | None]:
    """Return the vanishing point with the most support and a boolean mask of its segments."""
    lines = to_homogeneous_lines(segments)
    lengths = np.linalg.norm(segments[:, 2:] - segments[:, :2], axis=1)
    best_support, best_vanishing_point, best_inliers = 0.0, None, None

    # Every pair is tried instead of random RANSAC sampling: with a few dozen segments
    # per frame it stays cheap and the result is reproducible.
    for i, j in combinations(range(len(segments)), 2):
        vanishing_point = np.cross(lines[i], lines[j])
        norm = np.linalg.norm(vanishing_point)
        if norm < 1e-9:  # both segments lie on the same line
            continue
        vanishing_point /= norm
        if vanishing_point[2] < 0:
            vanishing_point = -vanishing_point
        if not lies_near_horizon(vanishing_point, image_size):
            continue

        inliers = angular_errors(segments, vanishing_point) < ANGULAR_TOLERANCE_RAD
        if inliers.any() and is_crossing_on_floor(vanishing_point, segments[inliers], image_size):
            continue
        # Weighted by length so a joint broken into several short segments does not
        # outvote one detected as a single long segment.
        support = lengths[inliers].sum()
        if inliers.sum() >= MIN_FAMILY_SEGMENTS and support > best_support:
            best_support, best_vanishing_point, best_inliers = support, vanishing_point, inliers

    return best_vanishing_point, best_inliers


def refine_vanishing_point(segments: np.ndarray) -> np.ndarray:
    """Return the length-weighted least-squares vanishing point of a family's segments."""
    # The seeding pair carries the noise of only two segments; solving over every member
    # averages it out, which matters for the orthogonality check against the calibration.
    lines = to_homogeneous_lines(segments)
    lengths = np.linalg.norm(segments[:, 2:] - segments[:, :2], axis=1)
    _, _, right_singular_vectors = np.linalg.svd(lines * lengths[:, None])
    vanishing_point = right_singular_vectors[-1]
    return vanishing_point if vanishing_point[2] >= 0 else -vanishing_point


def group_into_families(
    segments: np.ndarray, image_size: tuple[int, int]
) -> tuple[np.ndarray, np.ndarray]:
    """Label each segment with its family index (or OUTLIER_LABEL) and return the vanishing points."""
    labels = np.full(len(segments), OUTLIER_LABEL)
    vanishing_points = []
    remaining = np.arange(len(segments))

    for family_index in range(FAMILY_COUNT):
        if len(remaining) < MIN_FAMILY_SEGMENTS:
            break
        vanishing_point, inliers = find_dominant_family(segments[remaining], image_size)
        if vanishing_point is None:
            break
        labels[remaining[inliers]] = family_index
        vanishing_points.append(refine_vanishing_point(segments[remaining[inliers]]))
        remaining = remaining[~inliers]

    return labels, np.array(vanishing_points).reshape(-1, 3)


def draw_families(frame: np.ndarray, segments: np.ndarray, labels: np.ndarray) -> np.ndarray:
    overlay = frame.copy()
    for (x1, y1, x2, y2), label in zip(segments.astype(int), labels):
        color = OUTLIER_COLOR_BGR if label == OUTLIER_LABEL else FAMILY_COLORS_BGR[label]
        cv2.line(overlay, (x1, y1), (x2, y2), color, LINE_THICKNESS_PX)
    return overlay


def main() -> None:
    OUTPUT_DIR.mkdir(exist_ok=True)

    for lines_path in list_images(LINES_DIR, "frame_*_lines.npy"):
        frame_name = lines_path.stem.removesuffix("_lines")
        frame = read_image(FRAMES_DIR / f"{frame_name}.jpg")
        segments = np.load(lines_path).astype(float)
        image_size = (frame.shape[1], frame.shape[0])

        labels, vanishing_points = group_into_families(segments, image_size)

        np.savez(
            OUTPUT_DIR / f"{frame_name}_families.npz",
            segments=segments,
            labels=labels,
            vanishing_points=vanishing_points,
        )
        write_image(OUTPUT_DIR / f"{frame_name}_families.jpg", draw_families(frame, segments, labels))

    print(f"Saved line families to {OUTPUT_DIR}/")


if __name__ == "__main__":
    main()
