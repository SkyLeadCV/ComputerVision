"""Detect straight line segments on the floor edge maps with the probabilistic Hough transform."""

from pathlib import Path

import cv2
import numpy as np

from image_io import list_images, read_image, write_image

FRAMES_DIR = Path("frames")
EDGES_DIR = Path("edges")
OUTPUT_DIR = Path("lines")

# The probabilistic variant is used because the standard transform, which votes for
# infinite lines, accumulated scattered edges from furniture into many spurious lines
# crossing the whole floor. Segments also keep where each joint actually lies.
RHO_RESOLUTION_PX = 1
THETA_RESOLUTION_RAD = np.pi / 180
VOTE_THRESHOLD = 50
# Long enough to reject the short curved fragments left by stool rings and legs.
MIN_LINE_LENGTH_PX = 80
# Bridges the gaps where a tile joint fades out in the edge map.
MAX_LINE_GAP_PX = 25

LINE_COLOR_BGR = (0, 255, 0)
LINE_THICKNESS_PX = 3


def detect_line_segments(edges: np.ndarray) -> np.ndarray:
    """Return an (N, 4) array of segments as x1, y1, x2, y2."""
    segments = cv2.HoughLinesP(
        edges,
        RHO_RESOLUTION_PX,
        THETA_RESOLUTION_RAD,
        VOTE_THRESHOLD,
        minLineLength=MIN_LINE_LENGTH_PX,
        maxLineGap=MAX_LINE_GAP_PX,
    )
    if segments is None:
        return np.empty((0, 4), dtype=np.int32)
    # OpenCV 4 returns (N, 1, 4) and OpenCV 5 returns (N, 4).
    return segments.reshape(-1, 4)


def draw_segments(frame: np.ndarray, segments: np.ndarray) -> np.ndarray:
    overlay = frame.copy()
    for x1, y1, x2, y2 in segments:
        cv2.line(overlay, (int(x1), int(y1)), (int(x2), int(y2)), LINE_COLOR_BGR, LINE_THICKNESS_PX)
    return overlay


def main() -> None:
    OUTPUT_DIR.mkdir(exist_ok=True)

    for edges_path in list_images(EDGES_DIR, "frame_*_edges.png"):
        frame_name = edges_path.stem.removesuffix("_edges")
        frame = read_image(FRAMES_DIR / f"{frame_name}.jpg")
        edges = read_image(edges_path, cv2.IMREAD_GRAYSCALE)

        segments = detect_line_segments(edges)

        np.save(OUTPUT_DIR / f"{frame_name}_lines.npy", segments)
        write_image(OUTPUT_DIR / f"{frame_name}_lines.jpg", draw_segments(frame, segments))

    print(f"Saved line segments to {OUTPUT_DIR}/")


if __name__ == "__main__":
    main()
