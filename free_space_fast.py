"""Turn one undistorted frame into free-space rays and the next Tello command, without drawing anything.

This is the flight version of steps 004-011: it chains the same functions in memory instead of
reading and writing a folder per step. Run on frames/ it checks itself against free_space/.
"""

import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from apply_morphology import refine_mask
from detect_corners import EDGE_TOLERANCE_KERNEL, detect_tile
from detect_edges import detect_floor_edges
from detect_lines import detect_line_segments
from find_free_space import (
    allowed_distances, close_pockets, drone_safe_area, free_polygon_from_drone, sweep, tello_command,
)
from group_lines import group_into_families
from image_io import list_images, read_image
from rectify_floor import estimate_camera_height, floor_axes, floor_homography
from segment_floor import segment_floor

FRAMES_DIR = Path("frames")
FREE_SPACE_DIR = Path("free_space")
CALIBRATION_FILE = Path("calibration/camera_calibration.npz")


@dataclass(frozen=True)
class FreeSpace:
    """Rays in the units and order of the protocol's observation message, plus the chosen move.

    The rays run to the full sweep range, as the planner needs them. Whoever builds the observation
    message cuts them at the trusted range of docs/protocol.md.
    """

    angle_deg: np.ndarray
    entry_cm: np.ndarray
    stop_cm: np.ndarray
    kind: np.ndarray
    allowed_cm: np.ndarray
    command: str
    camera_height_cm: float


def analyze_frame(frame: np.ndarray, camera_matrix: np.ndarray) -> FreeSpace | None:
    """Return the free space seen in an undistorted frame, or None when no reference tile is found.

    Without a tile there is no floor scale, so nothing can be measured; the caller takes another photo.
    """
    inverse_camera_matrix = np.linalg.inv(camera_matrix)
    floor_mask = refine_mask(segment_floor(frame))
    edges = detect_floor_edges(frame, floor_mask)
    segments = detect_line_segments(edges).astype(float)
    labels, vanishing_points = group_into_families(segments, (frame.shape[1], frame.shape[0]))
    families = {"segments": segments, "labels": labels, "vanishing_points": vanishing_points}
    tile_corners, _ = detect_tile(families, cv2.dilate(edges, EDGE_TOLERANCE_KERNEL), inverse_camera_matrix)
    if tile_corners is None:
        return None

    axes = floor_axes(vanishing_points, inverse_camera_matrix, tile_corners.mean(axis=0))
    camera_height = estimate_camera_height(tile_corners, floor_homography(inverse_camera_matrix, axes, 1.0))
    image_to_floor = floor_homography(inverse_camera_matrix, axes, camera_height)

    rays = sweep(floor_mask, np.linalg.inv(image_to_floor), image_to_floor)
    stops, kinds, _ = close_pockets(rays)
    safe_area = drone_safe_area(free_polygon_from_drone(rays["angles"], stops))
    allowed = allowed_distances(safe_area, rays["angles"], rays["entry"])
    return FreeSpace(
        rays["angles"], rays["entry"], stops, kinds, allowed, tello_command(rays["angles"], allowed), camera_height
    )


def main() -> None:
    camera_matrix = np.load(CALIBRATION_FILE)["camera_matrix"]
    mismatches, times_ms = [], []
    for frame_path in list_images(FRAMES_DIR, "frame_*.jpg"):
        frame = read_image(frame_path)
        start = time.perf_counter()
        result = analyze_frame(frame, camera_matrix)
        times_ms.append(1000 * (time.perf_counter() - start))

        # find_free_space.py only writes a folder for frames with a tile, so both must agree on which those are.
        reference_dir = FREE_SPACE_DIR / frame_path.stem
        if result is None or not reference_dir.exists():
            if (result is None) != (not reference_dir.exists()):
                mismatches.append(frame_path.stem)
            continue
        with np.load(reference_dir / "free_space.npz") as free, np.load(reference_dir / "moves.npz") as moves:
            same = (
                np.array_equal(free["angles_deg"], result.angle_deg)
                and np.allclose(free["entry_cm"], result.entry_cm)
                and np.allclose(free["free_distance_cm"], result.stop_cm)
                and np.array_equal(free["kind"], result.kind)
                and np.allclose(moves["allowed_cm"], result.allowed_cm)
            )
        if not same:
            mismatches.append(frame_path.stem)
        print(f"{frame_path.stem}: {result.command} ({times_ms[-1]:.0f} ms)")

    print(f"{len(times_ms)} frames, median {np.median(times_ms):.0f} ms, max {np.max(times_ms):.0f} ms")
    print(f"Differences from {FREE_SPACE_DIR}/: {', '.join(mismatches) if mismatches else 'none'}")


if __name__ == "__main__":
    main()
