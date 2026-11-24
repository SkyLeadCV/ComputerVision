"""Find the rescuer's face in a photo and how far away it is: Viola-Jones and the pinhole model.

Run on face photos named face_<distance>cm_<name>_<n>.jpg (capture_photos.py), it draws each detection into rescuer/
and checks the measured distance against the one in the file name.

Usage: python detect_rescuer.py [photos folder, default photos/]
"""

import re
import sys
from dataclasses import dataclass
from functools import cache
from pathlib import Path
from statistics import median

import cv2
import numpy as np

from image_io import list_images, read_image, write_image

PHOTOS_DIR = Path("photos")
RESCUER_DIR = Path("rescuer")
CALIBRATION_FILE = Path("calibration/camera_calibration.npz")
# OpenCV's pretrained frontal face cascade (data/haarcascades in the OpenCV repo), kept here because OpenCV 5 no
# longer ships it.
CASCADE_FILE = Path("models/haarcascade_frontalface_default.xml")

# Real width of the square Viola-Jones draws around a face. It spans from ear to ear, wider than the 13.5-14.8 cm
# from cheekbone to cheekbone, so it was measured as the median of fx · box / distance over the face photos taken
# at 1.0, 1.5 and 2.0 m. Running this script on them prints it again.
FACE_BOX_WIDTH_CM = 21.6

# Each pass of the detector grows the window by 10 %, so a face is matched within 5 % of its size.
SCALE_FACTOR = 1.1
# Overlapping hits needed to accept a face; OpenCV's usual value, which kept false faces out of the photos.
MIN_NEIGHBORS = 5
# The box of a face 4 m away, beyond the 3 m at which the guide waits for the rescuer.
MIN_FACE_PX = 50

FACE_PHOTO_NAME = re.compile(r"face_(\d+)cm_")
BOX_BGR = (0, 0, 255)


@dataclass(frozen=True)
class Rescuer:
    """The face found in a photo: its box (x, y, width, height) in px and its distance along the camera's axis."""

    box: tuple[int, int, int, int]
    distance_cm: float


@cache
def face_detector() -> cv2.CascadeClassifier:
    detector = cv2.CascadeClassifier(str(CASCADE_FILE))
    if detector.empty():
        raise FileNotFoundError(f"Cannot load the face cascade: {CASCADE_FILE}")
    return detector


def detect_faces(photo: np.ndarray) -> np.ndarray:
    """Every frontal face in a BGR photo, as rows of (x, y, width, height) in px."""
    gray = cv2.equalizeHist(cv2.cvtColor(photo, cv2.COLOR_BGR2GRAY))
    faces = face_detector().detectMultiScale(gray, scaleFactor=SCALE_FACTOR, minNeighbors=MIN_NEIGHBORS,
                                             minSize=(MIN_FACE_PX, MIN_FACE_PX))
    return np.reshape(faces, (-1, 4))


def face_distance_cm(box_width_px: float, camera_matrix: np.ndarray) -> float:
    """Pinhole model: an object of known width looks smaller in proportion to its distance."""
    return camera_matrix[0, 0] * FACE_BOX_WIDTH_CM / box_width_px


def find_rescuer(photo: np.ndarray, camera_matrix: np.ndarray) -> Rescuer | None:
    """Return the nearest frontal face in an undistorted photo, or None when there is none.

    The nearest face is the widest one. It is the rescuer walking behind the drone, and anything smaller that the
    detector mistakes for a face, such as a pattern on clothes, is dropped with it.
    """
    faces = detect_faces(photo)
    if len(faces) == 0:
        return None
    x, y, width, height = (int(value) for value in faces[np.argmax(faces[:, 2])])
    return Rescuer((x, y, width, height), face_distance_cm(width, camera_matrix))


def real_distance_cm(path: Path) -> float | None:
    match = FACE_PHOTO_NAME.match(path.stem)
    return float(match[1]) if match else None


def main() -> None:
    photos_dir = Path(sys.argv[1]) if len(sys.argv) > 1 else PHOTOS_DIR
    with np.load(CALIBRATION_FILE) as calibration:
        camera_matrix = calibration["camera_matrix"]
    RESCUER_DIR.mkdir(exist_ok=True)

    found, errors, box_widths_cm = 0, {}, []
    paths = list_images(photos_dir, "face_*.jpg")
    for path in paths:
        photo = read_image(path)
        rescuer = find_rescuer(photo, camera_matrix)
        real = real_distance_cm(path)
        if rescuer is None:
            print(f"{path.name}: no face")
            write_image(RESCUER_DIR / path.name, photo)
            continue
        found += 1
        x, y, width, height = rescuer.box
        cv2.rectangle(photo, (x, y), (x + width, y + height), BOX_BGR, 3)
        cv2.putText(photo, f"{rescuer.distance_cm:.0f} cm", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 1.0, BOX_BGR, 2)
        write_image(RESCUER_DIR / path.name, photo)
        line = f"{path.name}: box {width} px, {rescuer.distance_cm:.0f} cm"
        if real is not None:
            error = (rescuer.distance_cm - real) / real * 100
            errors.setdefault(real, []).append(error)
            box_widths_cm.append(width * real / camera_matrix[0, 0])
            line += f" (real {real:.0f} cm, {error:+.0f} %)"
        print(line)

    print(f"\nFaces found in {found} of {len(paths)} photos, drawn in {RESCUER_DIR}/")
    for real, values in sorted(errors.items()):
        print(f"  at {real:.0f} cm: median error {median(values):+.0f} %, worst {max(values, key=abs):+.0f} %")
    if box_widths_cm:
        print(f"Face box width measured on these photos: {median(box_widths_cm):.1f} cm "
              f"(FACE_BOX_WIDTH_CM = {FACE_BOX_WIDTH_CM})")


if __name__ == "__main__":
    main()
