"""Image read/write helpers that raise on failure."""

from pathlib import Path

import cv2
import numpy as np


def read_image(path: Path, flags: int = cv2.IMREAD_COLOR) -> np.ndarray:
    # cv2.imread returns None instead of raising, which would otherwise surface
    # later as an unrelated error far from the actual cause.
    image = cv2.imread(str(path), flags)
    if image is None:
        raise FileNotFoundError(f"Cannot read image: {path}")
    return image


def write_image(path: Path, image: np.ndarray, params: tuple[int, ...] = ()) -> None:
    if not cv2.imwrite(str(path), image, list(params)):
        raise OSError(f"Cannot write image: {path}")


def list_images(directory: Path, pattern: str) -> list[Path]:
    paths = sorted(directory.glob(pattern))
    if not paths:
        raise FileNotFoundError(f"No files matching '{pattern}' in {directory}/")
    return paths
