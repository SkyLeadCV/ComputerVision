"""Detect tile joint edges inside the floor mask using Canny."""

from pathlib import Path

import cv2
import numpy as np

from image_io import list_images, read_image, write_image

FRAMES_DIR = Path("frames")
MASKS_DIR = Path("morphology")
OUTPUT_DIR = Path("edges")

# Suppresses sensor noise and tile surface texture so Canny responds to joints.
BLUR_KERNEL_SIZE = (5, 5)

# Tile joints are low-contrast on the pale floor; median-based automatic thresholds
# came out roughly twice as high and dropped most of them. The 1:3 ratio follows
# Canny's recommendation.
CANNY_LOW_THRESHOLD = 15
CANNY_HIGH_THRESHOLD = 45

# Shrinking the mask discards edges of objects bordering the floor (furniture,
# wall base), so what remains is mostly tile joints.
MASK_EROSION_KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))


def detect_floor_edges(frame: np.ndarray, floor_mask: np.ndarray) -> np.ndarray:
    grayscale = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(grayscale, BLUR_KERNEL_SIZE, 0)
    # Canny runs on the full image and the mask is applied afterwards: masking first
    # would turn the mask contour into a strong artificial edge.
    edges = cv2.Canny(blurred, CANNY_LOW_THRESHOLD, CANNY_HIGH_THRESHOLD)
    inner_floor_mask = cv2.erode(floor_mask, MASK_EROSION_KERNEL)
    return cv2.bitwise_and(edges, inner_floor_mask)


def main() -> None:
    OUTPUT_DIR.mkdir(exist_ok=True)

    for mask_path in list_images(MASKS_DIR, "frame_*_mask.png"):
        frame_name = mask_path.stem.removesuffix("_mask")
        frame = read_image(FRAMES_DIR / f"{frame_name}.jpg")
        floor_mask = read_image(mask_path, cv2.IMREAD_GRAYSCALE)

        edges = detect_floor_edges(frame, floor_mask)
        write_image(OUTPUT_DIR / f"{frame_name}_edges.png", edges)

    print(f"Saved edge maps to {OUTPUT_DIR}/")


if __name__ == "__main__":
    main()
