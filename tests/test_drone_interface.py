import socket
import threading
import time
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

import drone_interface
from drone_interface import TelloDrone, UnityDrone


class FakeSimulator:
    """A local TCP server that answers like the Unity simulator of docs/simulator.md."""

    def __init__(self):
        self.replies = {}
        self.received = []
        self.server = socket.create_server(("127.0.0.1", 0))
        self.port = self.server.getsockname()[1]
        threading.Thread(target=self._serve, daemon=True).start()

    def _serve(self):
        connection, _ = self.server.accept()
        with connection, connection.makefile("rb") as reader:
            for line in reader:
                command = line.decode("ascii").strip()
                self.received.append(command)
                reply = self.replies.get(command, b"ok\n")
                if reply is not None:
                    connection.sendall(reply)


@pytest.fixture
def simulator():
    fake = FakeSimulator()
    yield fake
    fake.server.close()


def test_unity_returns_the_reply(simulator):
    drone = UnityDrone("127.0.0.1", simulator.port)
    assert drone.send("takeoff") == "ok"
    assert simulator.received == ["takeoff"]


def test_unity_ignores_windows_line_endings(simulator):
    simulator.replies["takeoff"] = b"ok\r\n"
    drone = UnityDrone("127.0.0.1", simulator.port)
    assert drone.send("takeoff") == "ok"


def test_unity_decodes_the_photo(simulator):
    _, jpeg = cv2.imencode(".jpg", np.zeros((720, 960, 3), np.uint8))
    simulator.replies["photo"] = f"photo {jpeg.size}\n".encode("ascii") + jpeg.tobytes()
    drone = UnityDrone("127.0.0.1", simulator.port)
    assert drone.take_photo().shape == (720, 960, 3)


def test_unity_reads_the_state(simulator):
    simulator.replies["state?"] = b"pitch:0;roll:0;yaw:45;tof:80;bat:100;\n"
    drone = UnityDrone("127.0.0.1", simulator.port)
    assert drone.get_state() == {"yaw_deg": 45.0, "pitch_deg": 0.0, "height_cm": 80.0, "battery_pct": 100.0}


def test_unity_raises_when_the_simulator_is_silent(simulator, monkeypatch):
    monkeypatch.setattr(drone_interface, "REPLY_TIMEOUT_S", 0.2)
    simulator.replies["takeoff"] = None
    drone = UnityDrone("127.0.0.1", simulator.port)
    with pytest.raises(TimeoutError):
        drone.send("takeoff")


def test_close_lands_only_after_a_takeoff(simulator):
    drone = UnityDrone("127.0.0.1", simulator.port)
    drone.send("command")
    drone.close()
    assert simulator.received == ["command"]


def test_close_lands_a_flying_drone(simulator):
    drone = UnityDrone("127.0.0.1", simulator.port)
    drone.send("takeoff")
    drone.close()
    assert simulator.received == ["takeoff", "land"]


class FakeTello:
    """Stands in for djitellopy's Tello: no wifi, a pure red frame in RGB and scripted replies."""

    frame_shape = (720, 960, 3)

    def __init__(self):
        self.replies = {}
        self.sent = []
        self.timeouts = []

    def connect(self):
        pass

    def streamon(self):
        pass

    def get_frame_read(self):
        frame = np.zeros(self.frame_shape, np.uint8)
        frame[:, :, 0] = 255
        return SimpleNamespace(frame=frame)

    def send_command_with_return(self, command, timeout):
        # djitellopy type-checks its arguments and rejects a float timeout, which the first flight hit.
        if not isinstance(timeout, int):
            raise TypeError(f"timeout must be an int, got {type(timeout).__name__}")
        self.sent.append(command)
        self.timeouts.append(timeout)
        reply = self.replies.get(command, "ok")
        if isinstance(reply, BaseException):
            raise reply
        return reply

    def emergency(self):
        # djitellopy sends it and returns at once, without a reply.
        self.sent.append("emergency")

    def get_current_state(self):
        return {"yaw": 90, "pitch": 0, "roll": 0, "tof": 80, "bat": 50}

    def end(self):
        pass


@pytest.fixture
def fake_tello(tmp_path, monkeypatch, camera_matrix):
    calibration = tmp_path / "camera_calibration.npz"
    np.savez(calibration, camera_matrix=camera_matrix, distortion_coefficients=np.zeros(5))
    monkeypatch.setattr(drone_interface, "CALIBRATION_FILE", calibration)
    monkeypatch.setattr(drone_interface, "Tello", FakeTello)
    monkeypatch.setattr(drone_interface, "STREAM_SETTLE_S", 0.0)


@pytest.fixture
def tello(fake_tello):
    return TelloDrone()


def test_tello_photo_is_bgr(tello):
    assert tello.take_photo()[360, 480].tolist() == [0, 0, 255]


def test_tello_reads_the_state(tello):
    assert tello.get_state() == {"yaw_deg": 90.0, "pitch_deg": 0.0, "height_cm": 80.0, "battery_pct": 50.0}


def test_tello_timeout_raises_and_close_still_lands(tello):
    tello.tello.replies["takeoff"] = "Aborting command 'takeoff'. Did not receive a response after 20 seconds"
    with pytest.raises(TimeoutError):
        tello.send("takeoff")
    tello.close()
    assert tello.tello.sent == ["takeoff", "land"]


def test_tello_without_video_raises(fake_tello, monkeypatch):
    monkeypatch.setattr(FakeTello, "frame_shape", (300, 400, 3))
    monkeypatch.setattr(drone_interface, "REPLY_TIMEOUT_S", 0.2)
    with pytest.raises(TimeoutError):
        TelloDrone()


def test_close_lands_within_the_short_landing_timeout(tello):
    tello.send("takeoff")
    tello.close()
    assert tello.tello.sent == ["takeoff", "land"]
    assert tello.tello.timeouts[-1] == drone_interface.LAND_TIMEOUT_S


@pytest.mark.parametrize("landing", [
    "Aborting command 'land'. Did not receive a response after 5 seconds",
    "error Not joystick",
    KeyboardInterrupt(),
], ids=["no reply", "refused", "ctrl c again"])
def test_a_landing_that_fails_stops_the_motors(tello, landing):
    tello.send("takeoff")
    tello.tello.replies["land"] = landing
    tello.close()
    assert tello.tello.sent == ["takeoff", "land", "emergency"]
    assert not tello.flying


def test_unity_landing_that_never_answers_ends_the_flight_quickly(simulator, monkeypatch):
    monkeypatch.setattr(drone_interface, "LAND_TIMEOUT_S", 0.2)
    simulator.replies["land"] = None
    drone = UnityDrone(simulator.server.getsockname()[0], simulator.port)
    drone.send("takeoff")
    start = time.monotonic()
    drone.close()
    assert time.monotonic() - start < 2
    assert simulator.received == ["takeoff", "land"]
    assert not drone.flying
