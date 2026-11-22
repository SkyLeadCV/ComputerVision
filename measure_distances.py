"""Measure the floor distance from the camera to sample points and draw distance rings on the top view."""

from pathlib import Path

import cv2
import numpy as np

from image_io import list_images, read_image, write_image
from rectify_floor import apply_homography

FRAMES_DIR = Path("frames")
MASKS_DIR = Path("morphology")
TOP_VIEW_DIR = Path("top_view")
OUTPUT_DIR = Path("distances")

SAMPLE_SPACING_PX = 90
# Beyond this the floor is only a few pixels tall per metre, so one pixel of error in
# the image moves the point by tens of centimetres and the label would be misleading.
MAX_LABELLED_DISTANCE_CM = 600
RING_SPACING_CM = 100

POINT_COLOR_BGR = (0, 255, 255)
RING_COLOR_BGR = (0, 200, 255)
TEXT_COLOR_BGR = (255, 255, 255)
TEXT_OUTLINE_BGR = (0, 0, 0)


def floor_sample_points(floor_mask: np.ndarray) -> np.ndarray:
    """Return a regular grid of image points that fall on the floor."""
    height, width = floor_mask.shape
    xs, ys = np.meshgrid(
        np.arange(SAMPLE_SPACING_PX // 2, width, SAMPLE_SPACING_PX),
        np.arange(SAMPLE_SPACING_PX // 2, height, SAMPLE_SPACING_PX),
    )
    points = np.column_stack([xs.ravel(), ys.ravel()])
    return points[floor_mask[points[:, 1], points[:, 0]] > 0].astype(float)


def put_outlined_text(image: np.ndarray, text: str, origin: tuple[int, int], scale: float = 0.55) -> None:
    # The outline keeps labels readable on both the pale floor and dark furniture.
    cv2.putText(image, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale, TEXT_OUTLINE_BGR, 4)
    cv2.putText(image, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale, TEXT_COLOR_BGR, 1)


def draw_point_distances(frame: np.ndarray, points: np.ndarray, distances_cm: np.ndarray) -> np.ndarray:
    overlay = frame.copy()
    for point, distance_cm in zip(points.astype(int), distances_cm):
        if distance_cm > MAX_LABELLED_DISTANCE_CM:
            continue
        cv2.circle(overlay, tuple(point), 5, POINT_COLOR_BGR, -1)
        put_outlined_text(overlay, f"{distance_cm / 100:.1f} m", (point[0] + 7, point[1] - 7))
    return overlay


def draw_distance_rings(top_view: np.ndarray, floor_to_top_view: np.ndarray) -> np.ndarray:
    overlay = top_view.copy()
    camera_position = apply_homography(floor_to_top_view, np.zeros((1, 2)))[0]
    px_per_cm = floor_to_top_view[0, 0]
    for distance_cm in range(RING_SPACING_CM, MAX_LABELLED_DISTANCE_CM + 1, RING_SPACING_CM):
        radius = int(distance_cm * px_per_cm)
        cv2.circle(overlay, tuple(camera_position.astype(int)), radius, RING_COLOR_BGR, 1)
        label_position = camera_position + radius * np.array([np.cos(np.pi / 3), -np.sin(np.pi / 3)])
        put_outlined_text(overlay, f"{distance_cm // 100} m", tuple(label_position.astype(int)))
    return overlay


def main() -> None:
    OUTPUT_DIR.mkdir(exist_ok=True)

    for floor_path in list_images(TOP_VIEW_DIR, "frame_*_floor.npz"):
        frame_name = floor_path.stem.removesuffix("_floor")
        frame = read_image(FRAMES_DIR / f"{frame_name}.jpg")
        floor_mask = read_image(MASKS_DIR / f"{frame_name}_mask.png", cv2.IMREAD_GRAYSCALE)
        top_view = read_image(TOP_VIEW_DIR / f"{frame_name}_top_view.jpg")
        with np.load(floor_path) as floor:
            image_to_floor = floor["image_to_floor"]
            floor_to_top_view = floor["floor_to_top_view"]

        points = floor_sample_points(floor_mask)
        floor_positions_cm = apply_homography(image_to_floor, points)
        # Measured along the floor from the point right below the camera, which is what
        # matters for moving across the room.
        distances_cm = np.linalg.norm(floor_positions_cm, axis=1)

        write_image(OUTPUT_DIR / f"{frame_name}_points.jpg", draw_point_distances(frame, points, distances_cm))
        write_image(OUTPUT_DIR / f"{frame_name}_rings.jpg", draw_distance_rings(top_view, floor_to_top_view))

    print(f"Saved distance images to {OUTPUT_DIR}/")


if __name__ == "__main__":
    main()
