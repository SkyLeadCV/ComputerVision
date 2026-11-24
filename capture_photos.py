"""Save undistorted Tello photos without flying: SPACE saves, Q quits.

Usage: python capture_photos.py victim_300cm_side
Photos go to photos/<prefix>_<n>.jpg (photos/ is git-ignored).
"""

import sys
from pathlib import Path

import cv2

from drone_interface import TelloDrone
from image_io import write_image

OUTPUT_DIR = Path("photos")


def main() -> None:
    if len(sys.argv) != 2:
        sys.exit("usage: python capture_photos.py <prefix>   e.g. face_150cm or victim_300cm_side")
    prefix = sys.argv[1]
    OUTPUT_DIR.mkdir(exist_ok=True)
    drone = TelloDrone()
    print(f"battery {drone.get_state()['battery_pct']:.0f} %  -  SPACE saves, Q quits")
    count = len(list(OUTPUT_DIR.glob(f"{prefix}_*.jpg")))
    try:
        while True:
            photo = drone.take_photo()
            cv2.imshow("tello", photo)
            key = cv2.waitKey(30) & 0xFF
            if key == ord(" "):
                count += 1
                path = OUTPUT_DIR / f"{prefix}_{count}.jpg"
                write_image(path, photo)
                print(f"saved {path}")
            elif key == ord("q"):
                break
    finally:
        cv2.destroyAllWindows()
        drone.close()


if __name__ == "__main__":
    main()
