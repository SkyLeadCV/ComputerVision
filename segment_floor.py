"""Segment the classroom floor using an HSV color threshold."""

from pathlib import Path

import cv2
import numpy as np

from image_io import list_images, read_image, write_image

FRAMES_DIR = Path("frames")
OUTPUT_DIR = Path("segmentation")

# A single generic range rather than one tuned per video. It is deliberately wide:
# the mask only bounds where tile lines are searched, so including some wall costs
# little, while missing floor can drop the tile needed for distance estimation.
# Low-saturation tiles under some lighting still fall inside the range.
# Hue is in OpenCV's 0-179 range.
FLOOR_HSV_LOWER = np.array([28, 6, 70])
FLOOR_HSV_UPPER = np.array([58, 70, 230])

DENOISE_KERNEL = np.ones((5, 5), np.uint8)


def segment_floor(image_bgr: np.ndarray) -> np.ndarray:
    """Return a binary mask where 255 marks floor pixels."""
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, FLOOR_HSV_LOWER, FLOOR_HSV_UPPER)
    # Only pixel-level noise is removed here; region-level cleanup is done in
    # apply_morphology.py so it can be tuned without re-running the threshold.
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, DENOISE_KERNEL)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, DENOISE_KERNEL)


def main() -> None:
    OUTPUT_DIR.mkdir(exist_ok=True)

    for frame_path in list_images(FRAMES_DIR, "frame_*.jpg"):
        frame = read_image(frame_path)
        floor_mask = segment_floor(frame)
        floor_only = cv2.bitwise_and(frame, frame, mask=floor_mask)

        write_image(OUTPUT_DIR / f"{frame_path.stem}_mask.png", floor_mask)
        write_image(OUTPUT_DIR / f"{frame_path.stem}_floor.jpg", floor_only)

    print(f"Saved masks to {OUTPUT_DIR}/")


if __name__ == "__main__":
    main()
