"""Calibrate the DJI Tello camera from chessboard photos."""

from pathlib import Path

import cv2
import numpy as np

from image_io import list_images, read_image, write_image

CHESSBOARD_IMAGES_DIR = Path("calibration_images")
OUTPUT_DIR = Path("calibration")
CORNERS_DIR = OUTPUT_DIR / "corners"
CALIBRATION_FILE = OUTPUT_DIR / "camera_calibration.npz"

# Inner corners per row and per column: the printed board has 7x5 squares.
CHESSBOARD_INNER_CORNERS = (6, 4)
# Only sets the scale of the board poses; the intrinsic matrix does not depend on it.
SQUARE_SIZE_MM = 30.0

# Half-size of the sub-pixel search window, i.e. an 11x11 px area around each corner.
SUBPIXEL_WINDOW_HALF_SIZE = (5, 5)
SUBPIXEL_CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)


def chessboard_object_points(inner_corners: tuple[int, int], square_size: float) -> np.ndarray:
    """Return the 3D corner positions on the board plane (Z = 0) in findChessboardCorners order."""
    columns, rows = inner_corners
    grid = np.mgrid[0:columns, 0:rows].T.reshape(-1, 2)
    object_points = np.zeros((columns * rows, 3), np.float32)
    object_points[:, :2] = grid * square_size
    return object_points


def detect_chessboard_corners(image: np.ndarray) -> np.ndarray | None:
    """Return the sub-pixel inner corners, or None if the whole board is not visible."""
    grayscale = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCorners(grayscale, CHESSBOARD_INNER_CORNERS)
    if not found:
        return None
    return cv2.cornerSubPix(grayscale, corners, SUBPIXEL_WINDOW_HALF_SIZE, (-1, -1), SUBPIXEL_CRITERIA)


def main() -> None:
    CORNERS_DIR.mkdir(parents=True, exist_ok=True)

    image_points = []
    image_sizes = set()
    rejected_names = []
    for image_path in list_images(CHESSBOARD_IMAGES_DIR, "*.png"):
        image = read_image(image_path)
        image_sizes.add((image.shape[1], image.shape[0]))

        corners = detect_chessboard_corners(image)
        if corners is None:
            rejected_names.append(image_path.name)
            continue
        image_points.append(corners)
        annotated = cv2.drawChessboardCorners(image.copy(), CHESSBOARD_INNER_CORNERS, corners, True)
        write_image(CORNERS_DIR / image_path.name, annotated)

    if len(image_sizes) != 1:
        raise ValueError(f"Calibration images have different sizes: {sorted(image_sizes)}")
    if not image_points:
        raise RuntimeError(f"Chessboard not found in any image in {CHESSBOARD_IMAGES_DIR}/")
    image_size = image_sizes.pop()

    object_points = chessboard_object_points(CHESSBOARD_INNER_CORNERS, SQUARE_SIZE_MM)
    rms_error_px, camera_matrix, distortion_coefficients, _, _ = cv2.calibrateCamera(
        [object_points] * len(image_points), image_points, image_size, None, None
    )

    np.savez(
        CALIBRATION_FILE,
        camera_matrix=camera_matrix,
        distortion_coefficients=distortion_coefficients,
        image_size=np.array(image_size),
        rms_error_px=rms_error_px,
    )

    print(f"Chessboard found in {len(image_points)} of {len(image_points) + len(rejected_names)} images")
    if rejected_names:
        print(f"Rejected: {', '.join(rejected_names)}")
    print(f"Camera matrix:\n{camera_matrix}")
    print(f"Distortion coefficients: {distortion_coefficients.ravel()}")
    print(f"RMS reprojection error: {rms_error_px:.3f} px")
    print(f"Saved calibration to {CALIBRATION_FILE}")


if __name__ == "__main__":
    main()
