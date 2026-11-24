"""Fly the real Tello or the Unity simulator through the same four methods (docs/simulator.md).

Run on its own it is the first-flight check: take off, save a photo and the state before and
after a 90 degree clockwise turn, and land. The yaw must grow by about 90, as docs/simulator.md
requires; if it does not, TelloDrone has to convert the sign.
"""

import argparse
import json
import socket
import time
from abc import ABC, abstractmethod
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
from djitellopy import Tello

from image_io import write_image

CALIBRATION_FILE = Path("calibration/camera_calibration.npz")
OUTPUT_DIR = Path("photos")

# The simulator contract's limit, also used for the real drone: djitellopy's 7 s default is too
# short for a slow take-off, which djitellopy itself gives 20 s. An int, because djitellopy's
# type check rejects a float timeout.
REPLY_TIMEOUT_S = 20
SIMULATOR_PORT = 5006
FRAME_SHAPE = (720, 960)
# The H.264 stream lags behind the camera, so a frame grabbed right after "ok" can still show the move.
STREAM_SETTLE_S = 1.0
VIDEO_POLL_S = 0.1


def state_from_fields(fields: dict) -> dict:
    """Build get_state() from the Tello state packet fields, which both drones send."""
    return {
        "yaw_deg": float(fields["yaw"]),
        "pitch_deg": float(fields["pitch"]),
        "height_cm": float(fields["tof"]),
        "battery_pct": float(fields["bat"]),
    }


class DroneInterface(ABC):
    def __init__(self) -> None:
        # Commands go out as plain text, so the drone classes cannot tell take-off from any other
        # reply; tracking it here is what lets close() land only when needed.
        self.flying = False

    def send(self, command: str) -> str:
        """Send one SDK command and return the reply once the move has finished."""
        # A take-off can time out and still happen, so the drone counts as flying from the moment
        # it is asked to take off. Landing a drone that never left the ground only returns an error.
        if command == "takeoff":
            self.flying = True
        reply = self._send(command)
        if command == "land" and reply == "ok":
            self.flying = False
        return reply

    def close(self) -> None:
        """Land if still flying and close the connection."""
        try:
            if self.flying:
                self.send("land")
        finally:
            self._disconnect()

    @abstractmethod
    def _send(self, command: str) -> str: ...

    @abstractmethod
    def take_photo(self) -> np.ndarray:
        """Return the current view as an undistorted 960x720 BGR image."""

    @abstractmethod
    def get_state(self) -> dict:
        """Return yaw_deg, pitch_deg, height_cm and battery_pct."""

    @abstractmethod
    def _disconnect(self) -> None: ...


class UnityDrone(DroneInterface):
    def __init__(self, host: str, port: int = SIMULATOR_PORT) -> None:
        super().__init__()
        # The timeout applies to every read, so a silent simulator raises TimeoutError
        # instead of blocking the brain forever.
        self.connection = socket.create_connection((host, port), timeout=REPLY_TIMEOUT_S)
        self.reader = self.connection.makefile("rb")

    def _request(self, command: str) -> str:
        self.connection.sendall(f"{command}\n".encode("ascii"))
        line = self.reader.readline()
        if not line:
            raise ConnectionError("The simulator closed the connection")
        # The contract ends lines with "\n", but a stray "\r" (C#'s WriteLine on Windows) must not turn
        # "ok" into something else and stop close() from landing.
        return line.decode("ascii").strip()

    def _send(self, command: str) -> str:
        return self._request(command)

    def take_photo(self) -> np.ndarray:
        header = self._request("photo")
        name, size = header.split()
        if name != "photo":
            raise RuntimeError(f"Unexpected reply to photo: {header}")
        data = self.reader.read(int(size))
        if len(data) != int(size):
            raise ConnectionError("The simulator closed the connection in the middle of a photo")
        image = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError("The simulator sent a photo that is not a valid JPEG")
        return image

    def get_state(self) -> dict:
        line = self._request("state?")
        return state_from_fields(dict(field.split(":") for field in line.strip(";").split(";")))

    def _disconnect(self) -> None:
        self.reader.close()
        self.connection.close()


class TelloDrone(DroneInterface):
    def __init__(self) -> None:
        super().__init__()
        with np.load(CALIBRATION_FILE) as calibration:
            self.camera_matrix = calibration["camera_matrix"]
            self.distortion_coefficients = calibration["distortion_coefficients"]
        self.tello = Tello()
        self.tello.connect()
        self.tello.streamon()
        self.video = self.tello.get_frame_read()
        self.last_reply_time = time.monotonic()

        # Until the first frame is decoded djitellopy serves a small black placeholder.
        deadline = time.monotonic() + REPLY_TIMEOUT_S
        while self.video.frame.shape[:2] != FRAME_SHAPE:
            if time.monotonic() > deadline:
                raise TimeoutError("No video from the Tello")
            time.sleep(VIDEO_POLL_S)

    def _send(self, command: str) -> str:
        reply = self.tello.send_command_with_return(command, timeout=REPLY_TIMEOUT_S)
        self.last_reply_time = time.monotonic()
        # djitellopy reports a timeout as a reply text instead of raising.
        if reply.startswith("Aborting command"):
            raise TimeoutError(reply)
        return reply

    def take_photo(self) -> np.ndarray:
        time.sleep(max(0.0, STREAM_SETTLE_S - (time.monotonic() - self.last_reply_time)))
        # djitellopy 2.5 decodes frames as RGB; the vision pipeline expects OpenCV's BGR.
        frame = cv2.cvtColor(self.video.frame, cv2.COLOR_RGB2BGR)
        return cv2.undistort(frame, self.camera_matrix, self.distortion_coefficients)

    def get_state(self) -> dict:
        return state_from_fields(self.tello.get_current_state())

    def _disconnect(self) -> None:
        self.tello.end()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    target = parser.add_mutually_exclusive_group(required=True)
    target.add_argument("--sim", metavar="HOST", help="the Unity simulator's address")
    target.add_argument("--tello", action="store_true", help="the real drone, over its own wifi")
    args = parser.parse_args()

    drone = UnityDrone(args.sim) if args.sim else TelloDrone()
    session = datetime.now().strftime("%Y%m%d_%H%M%S")
    OUTPUT_DIR.mkdir(exist_ok=True)
    try:
        for command in ("command", "streamon", "takeoff"):
            print(f"{command}: {drone.send(command)}")
        states = {}
        for label, command in (("before", None), ("after_cw_90", "cw 90")):
            if command:
                print(f"{command}: {drone.send(command)}")
            states[label] = drone.get_state()
            write_image(OUTPUT_DIR / f"{session}_{label}.jpg", drone.take_photo())
            print(f"{label}: {states[label]}")
        (OUTPUT_DIR / f"{session}_state.json").write_text(json.dumps(states, indent=2), encoding="utf-8")
    finally:
        drone.close()
    print(f"Saved photos and state to {OUTPUT_DIR}/")


if __name__ == "__main__":
    main()
