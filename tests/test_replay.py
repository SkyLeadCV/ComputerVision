import json

import numpy as np
import pytest

import explorer
from replay_flight import FlightOver, ReplayDrone


class BlankDrone:
    """A drone whose photos show no floor tile, so the explorer can only look around and give up."""

    def __init__(self):
        self.yaw_deg = 30.0

    def send(self, command):
        name, _, value = command.partition(" ")
        if name in ("cw", "ccw"):
            self.yaw_deg = explorer.wrap_deg(self.yaw_deg + (float(value) if name == "cw" else -float(value)))
        return "ok"

    def take_photo(self):
        return np.zeros((72, 96, 3), np.uint8)

    def get_state(self):
        return {"yaw_deg": self.yaw_deg, "pitch_deg": 0.0, "height_cm": 80.0, "battery_pct": 90.0}

    def close(self):
        pass


def fly(drone, camera_matrix, session):
    def see():
        drone.take_photo()
        return None, []

    exploration = explorer.Explorer(drone, see, lambda: 0.0, explorer.camera_fov(camera_matrix),
                                    explorer.MapLink(session, None), None)
    return exploration.run(session)


def decisions(output_dirs, session):
    path = output_dirs / "flights" / session / "decisions.jsonl"
    return [json.loads(line)["text"] for line in path.read_text(encoding="utf-8").splitlines()]


def test_a_recorded_flight_replays_to_the_same_decisions(camera_matrix, output_dirs):
    recorder = explorer.FlightRecorder(BlankDrone(), output_dirs / "flights" / "flown", lambda: 0.0)
    flown = fly(recorder, camera_matrix, "flown")
    recorder.close()

    replay = ReplayDrone(output_dirs / "flights" / "flown")
    assert fly(replay, camera_matrix, "again") == flown
    assert replay.next == len(replay.entries)
    assert decisions(output_dirs, "again") == decisions(output_dirs, "flown")


def test_a_replay_stops_where_the_code_asks_for_something_else(output_dirs):
    folder = output_dirs / "flights" / "short"
    folder.mkdir(parents=True)
    (folder / "commands.log").write_text("     0.5 s  takeoff      -> ok   {'yaw_deg': 0.0}\n", encoding="utf-8")
    replay = ReplayDrone(folder)
    with pytest.raises(FlightOver, match="the code wants 'land'"):
        replay.send("land")
