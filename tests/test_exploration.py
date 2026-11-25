import json

import numpy as np
import pytest

import explorer

# The pad and four others spread over the arms and the top band of docs/classroom.md, clear of the victim.
TAKEOFFS_CM = [(60, 60), (415, 160), (230, 430), (60, 480), (420, 380)]


def synthetic_mission(camera_matrix, monkeypatch, takeoff_cm=(60, 60), guided=False):
    """The setup of explorer.py --synthetic, from the given take-off pad; without guided, it stops at the pad."""
    monkeypatch.setattr(explorer, "CLASSROOM_TAKEOFF_CM", takeoff_cm)
    drone = explorer.SyntheticDrone(camera_matrix)
    room = np.array(explorer.CLASSROOM_FLOOR_CM, float) - takeoff_cm
    map_link = explorer.MapLink("test", None)
    guide = explorer.Guide(drone.find_rescuer, camera_matrix) if guided else None
    exploration = explorer.Explorer(drone, drone.see, lambda: drone.clock_s,
                                    explorer.camera_fov(camera_matrix), map_link, room, guide)
    return drone, map_link, exploration


def recorded_messages(map_link):
    map_link.close()
    return [json.loads(line) for line in map_link.path.read_text(encoding="utf-8").splitlines()]


@pytest.mark.parametrize("takeoff_cm", TAKEOFFS_CM, ids=[f"from_{x}_{y}" for x, y in TAKEOFFS_CM])
def test_synthetic_exploration_comes_back_home(takeoff_cm, camera_matrix, output_dirs, monkeypatch):
    drone, map_link, exploration = synthetic_mission(camera_matrix, monkeypatch, takeoff_cm)
    result = exploration.run("test")

    assert result.startswith("back_at_start")
    # The Tello cannot fly a shorter leg, so the last one is skipped when the pad is closer than that.
    assert np.linalg.norm(drone.position) < explorer.MIN_FORWARD_CM
    assert exploration.stops <= 20

    messages = recorded_messages(map_link)
    assert [message["seq"] for message in messages] == list(range(len(messages)))
    times = [message["t"] for message in messages]
    assert times == sorted(times)
    assert messages[-1]["type"] == "session_end"
    assert messages[-1]["reason"] == "back_at_start"
    # The one victim of the classroom, found and placed on its body (its nearest point is at most half a body away).
    victims = [message for message in messages if message["type"] == "victim"]
    assert victims and {victim["id"] for victim in victims} == {1}
    (centre, _), = explorer.CLASSROOM_VICTIMS
    body_centre = np.array(centre) - takeoff_cm
    assert np.linalg.norm([victims[-1]["x"], victims[-1]["y"]] - body_centre) <= explorer.VICTIM_BODY_CM[0] / 2 + 10
    for message in messages:
        if message["type"] == "observation":
            # The synthetic drone hovers level, so the camera's pitch is its tilt.
            assert message["pose"]["pitch_deg"] == -explorer.CAMERA_TILT_DEG
            rays = message["rays"]
            assert all(stop - entry <= explorer.STEP_LIMIT_CM for entry, stop in zip(rays["entry_cm"], rays["stop_cm"]))


@pytest.mark.parametrize("failure, reason", [(RuntimeError, "error"), (KeyboardInterrupt, "user_stop")])
def test_an_interrupted_mission_still_ends_its_session(failure, reason, camera_matrix, output_dirs, monkeypatch):
    drone, map_link, exploration = synthetic_mission(camera_matrix, monkeypatch)

    def fail_on_first_forward(command, timeout_s=None):
        if command.startswith("forward"):
            raise failure(command)
        return explorer.SyntheticDrone._send(drone, command, timeout_s)

    monkeypatch.setattr(drone, "_send", fail_on_first_forward)
    with pytest.raises(failure):
        exploration.run("test")
    assert recorded_messages(map_link)[-1]["reason"] == reason


def test_a_missing_way_home_is_reported(camera_matrix, output_dirs, monkeypatch):
    drone, map_link, exploration = synthetic_mission(camera_matrix, monkeypatch)
    monkeypatch.setattr(exploration, "return_home", lambda: None)
    assert exploration.run("test").startswith("no_path_home")
    assert recorded_messages(map_link)[-1]["reason"] == "no_path_home"


def test_a_crash_is_kept_in_the_decision_log(camera_matrix, output_dirs, monkeypatch):
    drone, map_link, exploration = synthetic_mission(camera_matrix, monkeypatch)
    monkeypatch.setattr(exploration, "see", lambda: 1 / 0)
    with pytest.raises(ZeroDivisionError):
        exploration.run("crash")
    lines = (output_dirs / "flights" / "crash" / "decisions.jsonl").read_text(encoding="utf-8").splitlines()
    crash = json.loads(lines[-2])
    assert crash["event"] == "crash" and "ZeroDivisionError" in crash["traceback"]
