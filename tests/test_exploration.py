import json

import numpy as np
import pytest

import explorer

# The assumed pad and four others spread over the arms and the top band of docs/classroom.md.
TAKEOFFS_CM = [(60, 60), (415, 60), (230, 430), (60, 480), (400, 500)]


@pytest.mark.parametrize("takeoff_cm", TAKEOFFS_CM, ids=[f"from_{x}_{y}" for x, y in TAKEOFFS_CM])
def test_synthetic_exploration_comes_back_home(takeoff_cm, camera_matrix, output_dirs, monkeypatch):
    monkeypatch.setattr(explorer, "CLASSROOM_TAKEOFF_CM", takeoff_cm)
    drone = explorer.SyntheticDrone(camera_matrix)
    room = np.array(explorer.CLASSROOM_FLOOR_CM, float) - takeoff_cm
    map_link = explorer.MapLink("test", None)
    exploration = explorer.Explorer(drone, drone.see, lambda: drone.clock_s,
                                    explorer.camera_fov(camera_matrix), map_link, room)
    try:
        result = exploration.run("test")
    finally:
        drone.close()
        map_link.close()

    assert result.startswith("back_at_start")
    assert np.linalg.norm(drone.position) < 10
    assert exploration.stops <= 20

    messages = [json.loads(line) for line in map_link.path.read_text(encoding="utf-8").splitlines()]
    assert [message["seq"] for message in messages] == list(range(len(messages)))
    times = [message["t"] for message in messages]
    assert times == sorted(times)
    for message in messages:
        if message["type"] == "observation":
            rays = message["rays"]
            assert all(stop - entry <= explorer.STEP_LIMIT_CM for entry, stop in zip(rays["entry_cm"], rays["stop_cm"]))
