"""Fly a recorded flight again, offline, to see every decision the drone took and why.

explorer.py --tello keeps each photo and each command with its reply and state in flights/<session>/. The explorer
decides only from those, so feeding them back in the same order repeats the flight decision by decision, now with
the decision log (flights/<session>_replay/decisions.jsonl) and the grid images (explorer/<session>_replay/). If
the code asks for a command the drone did not get, it says where the two part ways and stops there.

Usage: python replay_flight.py flights/<session>
"""

import ast
import re
import sys
from pathlib import Path

import numpy as np

from detect_rescuer import find_rescuer
from explorer import CALIBRATION_FILE, Explorer, Eyes, Guide, MapLink, camera_fov
from image_io import read_image
from victim_detection import VictimDetector

# "  12.3 s  cw 54        -> ok                       {'yaw_deg': ...}", "  13.1 s  photo_004.jpg" or
# "  14.0 s  forward 200  -> TimeoutError: ...", as FlightRecorder writes them.
LOG_LINE = re.compile(r"^\s*(?P<t>[\d.]+) s  (?P<rest>.*)$")
COMMAND_LINE = re.compile(r"^(?P<command>.+?)\s+-> (?P<reply>.*?)\s*(?P<state>\{.*\})?$")


class FlightOver(Exception):
    """The recording ended or the code asked for something the drone was never sent."""


class ReplayDrone:
    """Answers the explorer with what the real drone answered, in the same order."""

    def __init__(self, folder: Path) -> None:
        self.folder = folder
        self.entries = []
        for line in (folder / "commands.log").read_text(encoding="utf-8").splitlines():
            match = LOG_LINE.match(line)
            if match:
                self.entries.append((float(match["t"]), match["rest"].strip()))
        self.next = 0
        self.t = 0.0
        self.photos = 0
        first_state = next((COMMAND_LINE.match(rest)["state"] for _, rest in self.entries
                            if COMMAND_LINE.match(rest) and COMMAND_LINE.match(rest)["state"]), None)
        self.state = ast.literal_eval(first_state) if first_state else {}

    def pop(self, wanted: str) -> str:
        if self.next >= len(self.entries):
            raise FlightOver(f"the recording ends here; the code wanted {wanted}")
        self.t, rest = self.entries[self.next]
        self.next += 1
        return rest

    def send(self, command: str) -> str:
        rest = self.pop(command)
        match = COMMAND_LINE.match(rest)
        if match is None or match["command"].strip() != command:
            raise FlightOver(f"at {self.t:.1f} s the drone got '{rest}', the code wants '{command}'")
        reply = match["reply"]
        if match["state"] is None:
            # The command raised in flight; raise the same here.
            raise RuntimeError(f"{command}: {reply}")
        self.state = ast.literal_eval(match["state"])
        return reply

    def take_photo(self) -> np.ndarray:
        rest = self.pop("a photo")
        if not rest.startswith("photo_"):
            raise FlightOver(f"at {self.t:.1f} s the drone got '{rest}', the code wants a photo")
        self.photos += 1
        return read_image(self.folder / rest)

    def get_state(self) -> dict:
        return self.state

    def close(self) -> None:
        pass


def main() -> None:
    if len(sys.argv) != 2:
        raise SystemExit(__doc__)
    folder = Path(sys.argv[1])
    with np.load(CALIBRATION_FILE) as calibration:
        camera_matrix = calibration["camera_matrix"]
    drone = ReplayDrone(folder)
    eyes = Eyes(camera_matrix, VictimDetector(camera_matrix))
    guide = Guide(lambda: find_rescuer(drone.take_photo(), camera_matrix), camera_matrix)
    session = f"{folder.name}_replay"
    map_link = MapLink(session, None)
    explorer = Explorer(drone, lambda: eyes.look(drone.take_photo()), lambda: drone.t, camera_fov(camera_matrix),
                        map_link, None, guide)
    try:
        print(f"Replay: {explorer.run(session)}")
    except FlightOver as over:
        print(f"Replay stops: {over}")
    except RuntimeError as error:
        print(f"The flight stopped on an error, as in the recording: {error}")
    finally:
        map_link.close()
    left = len(drone.entries) - drone.next
    print(f"Used {drone.next} of {len(drone.entries)} recorded lines ({left} left); decisions in "
          f"flights/{session}/decisions.jsonl, grid images in {explorer.output_dir}/")


if __name__ == "__main__":
    main()
