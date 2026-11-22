"""Undo the floor perspective: scale it with the reference tile and render a top view."""

from pathlib import Path

import cv2
import numpy as np

from image_io import list_images, read_image, write_image

FRAMES_DIR = Path("frames")
MASKS_DIR = Path("morphology")
FAMILIES_DIR = Path("families")
CORNERS_DIR = Path("corners")
CALIBRATION_FILE = Path("calibration/camera_calibration.npz")
OUTPUT_DIR = Path("top_view")

TILE_SIZE_CM = 60.0

# Floor area shown in the top view, centred on the point right below the camera.
TOP_VIEW_HALF_WIDTH_CM = 300
TOP_VIEW_DEPTH_CM = 600
TOP_VIEW_PX_PER_CM = 1.5

# Walls and furniture are not on the floor plane, so the warp stretches them across the
# top view. They are dimmed rather than removed so the reader still sees where they are.
NON_FLOOR_BRIGHTNESS = 0.3

OPTICAL_AXIS = np.array([0.0, 0.0, 1.0])

TILE_COLOR_BGR = (0, 255, 0)
CAMERA_COLOR_BGR = (0, 0, 255)
TEXT_COLOR_BGR = (255, 255, 255)


def floor_axes(
    vanishing_points: np.ndarray, inverse_camera_matrix: np.ndarray, floor_pixel: np.ndarray
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return the floor normal and the right and forward floor axes, in camera coordinates."""
    directions = vanishing_points @ inverse_camera_matrix.T
    directions /= np.linalg.norm(directions, axis=1, keepdims=True)
    normal = np.cross(directions[0], directions[1])
    normal /= np.linalg.norm(normal)
    # The cross product's sign is arbitrary. The normal must point from the camera
    # towards the floor, so that rays hitting the floor have a positive projection on it.
    if (inverse_camera_matrix @ np.append(floor_pixel, 1.0)) @ normal < 0:
        normal = -normal

    # Forward is where the camera looks, flattened onto the floor, so the top view is
    # oriented like the photo rather than along the tile joints.
    forward = OPTICAL_AXIS - (OPTICAL_AXIS @ normal) * normal
    forward /= np.linalg.norm(forward)
    right = np.cross(normal, forward)
    return normal, right, forward


def floor_homography(
    inverse_camera_matrix: np.ndarray, axes: tuple[np.ndarray, np.ndarray, np.ndarray], camera_height: float
) -> np.ndarray:
    """Return the homography from image pixels to floor (right, forward) coordinates.

    A pixel's ray r meets the floor at X = h·r / (r·n). Its coordinates along the right and
    forward axes are therefore h·(r·right) / (r·n) and h·(r·forward) / (r·n), with the origin
    right below the camera.
    """
    normal, right, forward = axes
    return np.vstack([camera_height * right, camera_height * forward, normal]) @ inverse_camera_matrix


def apply_homography(homography: np.ndarray, points: np.ndarray) -> np.ndarray:
    projected = np.hstack([points, np.ones((len(points), 1))]) @ homography.T
    return projected[:, :2] / projected[:, 2:]


def estimate_camera_height(tile_corners: np.ndarray, unit_height_homography: np.ndarray) -> float:
    """Return the camera height in cm that makes the reference tile TILE_SIZE_CM wide."""
    floor_corners = apply_homography(unit_height_homography, tile_corners)
    side_lengths = np.linalg.norm(np.roll(floor_corners, -1, axis=0) - floor_corners, axis=1)
    # With a unit camera height the floor is measured in multiples of that height,
    # so the real height is the ratio between the real and the measured tile side.
    return TILE_SIZE_CM / side_lengths.mean()


def top_view_matrix() -> np.ndarray:
    """Return the transform from floor cm to top view pixels, with forward pointing up."""
    return np.array([
        [TOP_VIEW_PX_PER_CM, 0.0, TOP_VIEW_HALF_WIDTH_CM * TOP_VIEW_PX_PER_CM],
        [0.0, -TOP_VIEW_PX_PER_CM, TOP_VIEW_DEPTH_CM * TOP_VIEW_PX_PER_CM],
        [0.0, 0.0, 1.0],
    ])


def draw_reference_tile(frame: np.ndarray, tile_corners: np.ndarray) -> np.ndarray:
    overlay = frame.copy()
    cv2.polylines(overlay, [tile_corners.round().astype(np.int32)], True, TILE_COLOR_BGR, 3)
    for index, corner in enumerate(tile_corners):
        cv2.circle(overlay, tuple(corner.round().astype(int)), 6, TILE_COLOR_BGR, -1)
        cv2.putText(overlay, str(index), tuple((corner + 8).astype(int)), cv2.FONT_HERSHEY_SIMPLEX, 0.8, TEXT_COLOR_BGR, 2)
    label_position = tuple((tile_corners.mean(axis=0) - [55, 0]).astype(int))
    cv2.putText(overlay, f"{TILE_SIZE_CM:.0f} x {TILE_SIZE_CM:.0f} cm", label_position, cv2.FONT_HERSHEY_SIMPLEX, 0.7, TEXT_COLOR_BGR, 2)
    return overlay


def render_top_view(
    frame: np.ndarray,
    floor_mask: np.ndarray,
    image_to_top_view: np.ndarray,
    tile_corners: np.ndarray,
    camera_height: float,
) -> np.ndarray:
    size = (int(2 * TOP_VIEW_HALF_WIDTH_CM * TOP_VIEW_PX_PER_CM), int(TOP_VIEW_DEPTH_CM * TOP_VIEW_PX_PER_CM))
    dimmed = np.where(floor_mask[..., None] > 0, frame, (frame * NON_FLOOR_BRIGHTNESS).astype(np.uint8))
    top_view = cv2.warpPerspective(dimmed, image_to_top_view, size)

    tile_in_top_view = apply_homography(image_to_top_view, tile_corners)
    cv2.polylines(top_view, [tile_in_top_view.round().astype(np.int32)], True, TILE_COLOR_BGR, 2)

    camera_position = apply_homography(top_view_matrix(), np.zeros((1, 2)))[0].astype(int)
    cv2.circle(top_view, tuple(camera_position), 10, CAMERA_COLOR_BGR, -1)
    cv2.putText(top_view, f"camera height {camera_height:.0f} cm", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, TEXT_COLOR_BGR, 2)
    return top_view


def main() -> None:
    OUTPUT_DIR.mkdir(exist_ok=True)
    inverse_camera_matrix = np.linalg.inv(np.load(CALIBRATION_FILE)["camera_matrix"])

    for tile_path in list_images(CORNERS_DIR, "frame_*_tile.npy"):
        tile_corners = np.load(tile_path)
        if len(tile_corners) == 0:
            continue
        frame_name = tile_path.stem.removesuffix("_tile")
        frame = read_image(FRAMES_DIR / f"{frame_name}.jpg")
        floor_mask = read_image(MASKS_DIR / f"{frame_name}_mask.png", cv2.IMREAD_GRAYSCALE)
        with np.load(FAMILIES_DIR / f"{frame_name}_families.npz") as families:
            vanishing_points = families["vanishing_points"]

        axes = floor_axes(vanishing_points, inverse_camera_matrix, tile_corners.mean(axis=0))
        camera_height = estimate_camera_height(tile_corners, floor_homography(inverse_camera_matrix, axes, 1.0))
        image_to_floor = floor_homography(inverse_camera_matrix, axes, camera_height)
        image_to_top_view = top_view_matrix() @ image_to_floor

        np.savez(
            OUTPUT_DIR / f"{frame_name}_floor.npz",
            camera_height_cm=camera_height,
            image_to_floor=image_to_floor,
            floor_to_top_view=top_view_matrix(),
        )
        write_image(OUTPUT_DIR / f"{frame_name}_reference.jpg", draw_reference_tile(frame, tile_corners))
        write_image(
            OUTPUT_DIR / f"{frame_name}_top_view.jpg",
            render_top_view(frame, floor_mask, image_to_top_view, tile_corners, camera_height),
        )
        print(f"{frame_name}: camera height {camera_height:.1f} cm")

    print(f"Saved reference tiles and top views to {OUTPUT_DIR}/")


if __name__ == "__main__":
    main()
