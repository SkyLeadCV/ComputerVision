import numpy as np
import pytest

import guide
from detect_rescuer import Rescuer
from test_exploration import TAKEOFFS_CM, recorded_messages, synthetic_mission


@pytest.mark.parametrize("takeoff_cm", TAKEOFFS_CM, ids=[f"from_{x}_{y}" for x, y in TAKEOFFS_CM])
def test_the_rescuer_is_led_to_the_victim_and_never_left_behind(takeoff_cm, camera_matrix, output_dirs, monkeypatch):
    drone, map_link, exploration = synthetic_mission(camera_matrix, monkeypatch, takeoff_cm, guided=True)
    gaps = []

    def send_and_measure(command, timeout_s=None):
        reply = type(drone)._send(drone, command, timeout_s)
        if command.startswith("back"):
            gaps.append(np.linalg.norm(drone.rescuer - drone.position))
        return reply

    monkeypatch.setattr(drone, "_send", send_and_measure)
    assert exploration.run("test").startswith("at_victim")

    # Within a centimetre: Viola-Jones boxes are whole pixels, which rounds the measured distance.
    assert max(gaps, default=0.0) <= guide.MAX_RESCUER_CM + 1
    (_, points), = exploration.victims
    # The path ends on a cell centre, so the drone lands within a cell of the stopping distance.
    assert np.linalg.norm(drone.position - np.mean(points, axis=0)) <= guide.STOP_SHORT_CM + 10
    assert drone.height_cm == 0

    messages = recorded_messages(map_link)
    modes = [message["mode"] for message in messages if message["type"] == "state"]
    assert modes[0] == "exploring" and modes[-1] == "arrived"
    # Waiting only until the rescuer first comes within reach, then guiding until the drone lands.
    guided = [mode for mode in modes if mode != "exploring"]
    assert guided == sorted(guided, key=["waiting", "guiding", "arrived"].index)
    assert messages[-1]["reason"] == "at_victim"


def test_without_a_rescuer_the_drone_lands_on_the_way(camera_matrix, output_dirs, monkeypatch):
    drone, map_link, exploration = synthetic_mission(camera_matrix, monkeypatch, guided=True)
    monkeypatch.setattr(exploration.guide, "find_rescuer", lambda: None)

    assert exploration.run("test").startswith("rescuer_lost")
    assert drone.height_cm == 0
    assert recorded_messages(map_link)[-1]["reason"] == "rescuer_lost"


def face_box(centre_px, width_px=100):
    return (round(centre_px - width_px / 2), 0, width_px, width_px)


def test_a_face_in_the_middle_of_the_photo_is_straight_ahead(camera_matrix):
    cx = camera_matrix[0, 2]
    ahead, sideways = guide.rescuer_offset_cm(Rescuer(face_box(cx), 120.0), camera_matrix)
    assert (ahead, sideways) == pytest.approx((120.0, 0.0), abs=0.5)


def test_a_face_to_the_right_is_to_the_side_by_the_pinhole_model(camera_matrix):
    fx, cx = camera_matrix[0, 0], camera_matrix[0, 2]
    # A face fx / 2 px right of the centre is half as far to the side as it is ahead.
    ahead, sideways = guide.rescuer_offset_cm(Rescuer(face_box(cx + fx / 2), 120.0), camera_matrix)
    assert (ahead, sideways) == pytest.approx((120.0, 60.0), abs=0.5)
