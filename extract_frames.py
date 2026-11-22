"""Extract evenly spaced frames from the classroom video."""

from pathlib import Path

import cv2
import numpy as np

from image_io import write_image

VIDEO_PATH = Path("20261118_201208.mp4")
CALIBRATION_FILE = Path("calibration/camera_calibration.npz")
OUTPUT_DIR = Path("frames")
FRAME_COUNT = 60
JPEG_QUALITY = 90

# The recording ends pointing at the ceiling and people, which carries no floor
# information. Frames after this index (~58.6 s at 30 fps) are ignored.
LAST_FRAME_INDEX = 1759


def main() -> None:
    calibration = np.load(CALIBRATION_FILE)
    camera_matrix = calibration["camera_matrix"]
    distortion_coefficients = calibration["distortion_coefficients"]

    capture = cv2.VideoCapture(str(VIDEO_PATH))
    if not capture.isOpened():
        raise FileNotFoundError(f"Cannot open video: {VIDEO_PATH}")

    total_frames = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    last_frame_index = min(LAST_FRAME_INDEX, total_frames - 1)
    selected_indices = set(np.linspace(0, last_frame_index, FRAME_COUNT).round().astype(int))
    OUTPUT_DIR.mkdir(exist_ok=True)

    # Decode sequentially rather than seeking with CAP_PROP_POS_FRAMES: seeking in
    # compressed video snaps to keyframes and can silently return the wrong frame.
    saved_count = 0
    for frame_index in range(last_frame_index + 1):
        success, frame = capture.read()
        if not success:
            break
        if frame_index in selected_indices:
            saved_count += 1
            # Undistorting once here lets every later step assume a pinhole camera: lens
            # distortion bends straight tile joints by up to ~18 px near the image borders,
            # which skews Hough lines, vanishing points and the geometry built on them.
            undistorted = cv2.undistort(frame, camera_matrix, distortion_coefficients)
            output_path = OUTPUT_DIR / f"frame_{saved_count:02d}.jpg"
            write_image(output_path, undistorted, (cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY))

    capture.release()
    print(f"Saved {saved_count} frames to {OUTPUT_DIR}/")


if __name__ == "__main__":
    main()
