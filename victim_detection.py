"""Find people lying on the floor in an undistorted frame and place them on the floor.

YOLO finds people; geometry decides who is lying: the top of a person's box, taken straight above the point where
the box meets the floor, must be lower than a seated person's head. The floor comes from the frame's own reference
tile or, without one, from the last photo that had a tile: turning on the spot changes neither the camera's height
nor its tilt, so the same floor holds.

Run on a folder of photos it writes, for each one, the people YOLO found and the victims that remain.
"""

import argparse
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from free_space_fast import CALIBRATION_FILE, analyze_frame
from image_io import list_images, read_image, write_image
from rectify_floor import apply_homography, axes_from_normal, floor_homography

OUTPUT_DIR = Path("victims")

# The small model finds every victim in the class photos with a confidence of 0.69 or more and no person on the
# empty floor above 0.37; the nano one drops to 0.37 on people half hidden by stools.
MODEL_FILE = "yolo11s.pt"
PERSON_CLASS = 0
MIN_CONFIDENCE = 0.5
# The tops of the class victims stand 30-78 cm above the floor (a raised shoulder is the highest); a seated
# person's head is about 130 cm and a standing one about 170.
LYING_MAX_TOP_CM = 100.0
# A box touching the bottom of the image has its nearest part in the blind zone, so where it meets the floor
# is unknown; a later photo from further away places it.
BOTTOM_MARGIN_PX = 3

PERSON_COLOR_BGR = (255, 160, 0)
VICTIM_COLOR_BGR = (0, 0, 255)
REJECTED_COLOR_BGR = (160, 160, 160)


@dataclass(frozen=True)
class Victim:
    """A person lying on the floor. floor_cm is the body's nearest point, to the right and forward of the drone."""

    floor_cm: np.ndarray
    confidence: float
    top_cm: float
    box: np.ndarray


@dataclass(frozen=True)
class Person:
    box: np.ndarray
    confidence: float


class VictimDetector:
    def __init__(self, camera_matrix: np.ndarray) -> None:
        # Imported here: PyTorch takes seconds to load, and the synthetic flight and the geometry need none of it.
        from ultralytics import YOLO

        self.model = YOLO(MODEL_FILE)
        self.inverse_camera_matrix = np.linalg.inv(camera_matrix)

    def find_people(self, frame: np.ndarray) -> list[Person]:
        boxes = self.model(frame, verbose=False)[0].boxes
        return [
            Person(box.cpu().numpy(), float(confidence))
            for box, confidence, label in zip(boxes.xyxy, boxes.conf, boxes.cls)
            if label == PERSON_CLASS and confidence >= MIN_CONFIDENCE
        ]

    def place(self, person: Person, frame_height: int, floor_normal: np.ndarray, camera_height_cm: float) -> Victim | None:
        return place(person, frame_height, self.inverse_camera_matrix, floor_normal, camera_height_cm)

    def find_victims(self, frame: np.ndarray, floor_normal: np.ndarray, camera_height_cm: float) -> list[Victim]:
        victims = (self.place(person, frame.shape[0], floor_normal, camera_height_cm) for person in self.find_people(frame))
        return [victim for victim in victims if victim is not None]


def place(person: Person, frame_height: int, inverse_camera_matrix: np.ndarray, floor_normal: np.ndarray,
          camera_height_cm: float) -> Victim | None:
    """Return the person as a victim, or None if not lying or with the nearest part out of sight."""
    x1, y1, x2, y2 = person.box
    if y2 >= frame_height - BOTTOM_MARGIN_PX:
        return None
    bottom = np.array([(x1 + x2) / 2, y2])
    top_cm = top_height_cm(bottom, y1, inverse_camera_matrix, floor_normal, camera_height_cm)
    if top_cm > LYING_MAX_TOP_CM:
        return None
    image_to_floor = floor_homography(inverse_camera_matrix, axes_from_normal(floor_normal), camera_height_cm)
    return Victim(apply_homography(image_to_floor, bottom[None])[0], person.confidence, top_cm, person.box)


def top_height_cm(bottom: np.ndarray, top_row: float, inverse_camera_matrix: np.ndarray,
                  floor_normal: np.ndarray, camera_height_cm: float) -> float:
    """Height of the box's top edge, taken straight above the point where the box's bottom meets the floor."""
    bottom_ray = inverse_camera_matrix @ np.append(bottom, 1.0)
    top_ray = inverse_camera_matrix @ np.array([bottom[0], top_row, 1.0])
    # The bottom ray meets the floor at h·r / (r·n); its horizontal distance is its length off the normal.
    floor_point = camera_height_cm * bottom_ray / (bottom_ray @ floor_normal)
    distance = np.linalg.norm(floor_point - (floor_point @ floor_normal) * floor_normal)
    # How much the top ray drops for each cm it travels horizontally; negative above the horizon.
    drop_per_cm = (top_ray @ floor_normal) / np.linalg.norm(top_ray - (top_ray @ floor_normal) * floor_normal)
    return camera_height_cm - distance * drop_per_cm


def draw(frame: np.ndarray, people: list[Person], victims: list[Victim | None] | None = None) -> np.ndarray:
    """Draw YOLO's people, or, given the victims, which ones are lying and where."""
    image = frame.copy()
    for index, person in enumerate(people):
        x1, y1, x2, y2 = person.box.astype(int)
        if victims is None:
            color, label = PERSON_COLOR_BGR, f"person {person.confidence:.2f}"
        elif victims[index] is None:
            color, label = REJECTED_COLOR_BGR, "not lying or out of sight"
        else:
            victim = victims[index]
            color, label = VICTIM_COLOR_BGR, f"top {victim.top_cm:.0f} cm, at {np.linalg.norm(victim.floor_cm):.0f} cm"
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
        cv2.putText(image, label, (x1, max(y1 - 8, 16)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    return image


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("photos", type=Path, help="folder of undistorted photos")
    parser.add_argument("--floor-from", type=Path, required=True,
                        help="a photo with a reference tile, used for photos that have none")
    args = parser.parse_args()

    camera_matrix = np.load(CALIBRATION_FILE)["camera_matrix"]
    reference = analyze_frame(read_image(args.floor_from), camera_matrix)
    if reference is None:
        raise SystemExit(f"No reference tile in {args.floor_from}")
    detector = VictimDetector(camera_matrix)
    OUTPUT_DIR.mkdir(exist_ok=True)

    for path in list_images(args.photos, "*.jpg"):
        frame = read_image(path)
        own = analyze_frame(frame, camera_matrix)
        floor = own or reference
        people = detector.find_people(frame)
        victims = [detector.place(person, frame.shape[0], floor.floor_normal, floor.camera_height_cm) for person in people]
        write_image(OUTPUT_DIR / f"{path.stem}_1_people.jpg", draw(frame, people))
        write_image(OUTPUT_DIR / f"{path.stem}_2_victims.jpg", draw(frame, people, victims))
        found = ", ".join(f"{np.linalg.norm(v.floor_cm):.0f} cm (top {v.top_cm:.0f})" for v in victims if v)
        print(f"{path.stem:30} {'own tile' if own else 'last tile'}  people {len(people)}  victims: {found or '-'}")


if __name__ == "__main__":
    main()
