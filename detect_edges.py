"""Detect tile joint edges on the floor with the Canny edge detector."""

from pathlib import Path

import cv2
import numpy as np

from image_io import list_images, read_image, write_image

FRAMES_DIR = Path("frames")
MASKS_DIR = Path("morphology")
OUTPUT_DIR = Path("edges")

# Light blur to suppress tile texture and JPEG noise without washing out the
# joints, which are only a few pixels wide in 960x720 frames.
BLUR_KERNEL_SIZE = (5, 5)

# Joints have low contrast against the tiles, so the thresholds are kept low;
# the hysteresis ratio (~1:3) keeps weak joint pixels connected to strong ones.
CANNY_LOW_THRESHOLD = 30
CANNY_HIGH_THRESHOLD = 90

# The mask is eroded before being applied so the floor/wall boundary itself is
# not reported as an edge; only joints inside the floor should reach Hough.
BORDER_ERODE_KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))


def detect_edges(image_bgr: np.ndarray, floor_mask: np.ndarray) -> np.ndarray:
    """Return a binary edge map (255 = edge) restricted to the floor."""
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, BLUR_KERNEL_SIZE, 0)
    # Canny runs on the full frame rather than the masked one: masking first would
    # create an artificial black border whose edge Canny would pick up.
    edges = cv2.Canny(blurred, CANNY_LOW_THRESHOLD, CANNY_HIGH_THRESHOLD)
    inner_mask = cv2.erode(floor_mask, BORDER_ERODE_KERNEL)
    return cv2.bitwise_and(edges, edges, mask=inner_mask)


def main() -> None:
    OUTPUT_DIR.mkdir(exist_ok=True)

    for mask_path in list_images(MASKS_DIR, "frame_*_mask.png"):
        frame_name = mask_path.stem.removesuffix("_mask")
        frame = read_image(FRAMES_DIR / f"{frame_name}.jpg")
        mask = read_image(mask_path, cv2.IMREAD_GRAYSCALE)

        edges = detect_edges(frame, mask)

        write_image(OUTPUT_DIR / f"{frame_name}_edges.png", edges)

    print(f"Saved edge maps to {OUTPUT_DIR}/")


if __name__ == "__main__":
    main()
