"""Refine floor masks with morphological opening followed by closing."""

from pathlib import Path

import cv2
import numpy as np

from image_io import list_images, read_image, write_image

FRAMES_DIR = Path("frames")
MASKS_DIR = Path("segmentation")
OUTPUT_DIR = Path("morphology")

# Sizes assume 960x720 frames. Opening is kept small so it does not erode the
# thin strip of far floor along the wall, which is where Hough lines will be fit.
# Closing must exceed the width of stool legs (~15-25 px) to fill the gaps they leave.
OPEN_KERNEL_SIZE = 15
CLOSE_KERNEL_SIZE = 31

# Elliptical rather than rectangular so the kernel does not imprint artificial
# straight edges on the mask that Hough would later detect as lines.
OPEN_KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (OPEN_KERNEL_SIZE, OPEN_KERNEL_SIZE))
CLOSE_KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (CLOSE_KERNEL_SIZE, CLOSE_KERNEL_SIZE))


def refine_mask(mask: np.ndarray) -> np.ndarray:
    # Opening goes first: closing first would merge scattered false positives
    # into blobs too large for the opening to remove.
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, OPEN_KERNEL)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, CLOSE_KERNEL)


def main() -> None:
    OUTPUT_DIR.mkdir(exist_ok=True)

    for mask_path in list_images(MASKS_DIR, "frame_*_mask.png"):
        frame_name = mask_path.stem.removesuffix("_mask")
        frame = read_image(FRAMES_DIR / f"{frame_name}.jpg")
        mask = read_image(mask_path, cv2.IMREAD_GRAYSCALE)

        refined_mask = refine_mask(mask)
        floor_only = cv2.bitwise_and(frame, frame, mask=refined_mask)

        write_image(OUTPUT_DIR / f"{frame_name}_mask.png", refined_mask)
        write_image(OUTPUT_DIR / f"{frame_name}_floor.jpg", floor_only)

    print(f"Saved refined masks to {OUTPUT_DIR}/")


if __name__ == "__main__":
    main()
