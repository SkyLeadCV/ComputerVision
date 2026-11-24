import numpy as np
import pytest

import explorer
from free_space_fast import FreeSpace

@pytest.mark.parametrize("angle, expected", [(30, 30), (190, -170), (360, 0), (-200, 160)])
def test_wrap_deg_keeps_angles_within_half_turn(angle, expected):
    assert explorer.wrap_deg(angle) == expected


def test_camera_fov_matches_calibration(camera_matrix):
    fov = explorer.camera_fov(camera_matrix)
    assert fov["left_deg"] == pytest.approx(25.4, abs=0.1)
    assert fov["right_deg"] == pytest.approx(29.1, abs=0.1)
    assert fov["up_deg"] == pytest.approx(20.4, abs=0.1)
    assert fov["down_deg"] == pytest.approx(22.1, abs=0.1)

def test_trusted_rays_cut_at_entry_plus_one_step():
    free_space = FreeSpace(
        angle_deg=np.array([-10.0, 0.0, 10.0]),
        entry_cm=np.array([100.0, 100.0, 100.0]),
        stop_cm=np.array([250.0, 300.0, 450.0]),
        kind=np.array([0, 0, 0]),
        allowed_cm=np.zeros(3),
        command="",
        camera_height_cm=80.0,
        floor_normal=np.array([0.0, 1.0, 0.0]),
    )
    rays = explorer.trusted_rays(free_space)
    assert rays["stop_cm"].tolist() == [250, 300, 300]
    assert rays["kind"].tolist() == [0, 0, 2]


def test_astar_goes_around_a_wall():
    # Arrange
    passable = np.ones((5, 5), dtype=bool)
    passable[:, 2] = False
    passable[-1, 2] = True
    # Act
    path = explorer.astar(passable, (0, 0), (0, 4))
    # Assert
    assert path is not None
    assert path[0] == (0, 0)
    assert path[-1] == (0, 4)
    assert (4, 2) in path
    assert all(passable[cell] for cell in path)


def test_astar_returns_none_without_path():
    passable = np.ones((5, 5), dtype=bool)
    passable[:, 2] = False
    assert explorer.astar(passable, (0, 0), (0, 4)) is None


def one_ray(angle_deg, entry_cm, stop_cm, kind):
    return {"angle_deg": [angle_deg], "entry_cm": [entry_cm], "stop_cm": [stop_cm], "kind": [kind]}


def cell_at(grid, x_cm, y_cm):
    rows, columns, _ = grid.index(np.array([x_cm, y_cm]))
    return grid.cells[rows[0], columns[0]]


def test_grid_marks_floor_up_to_the_stop_and_the_wall_behind_it():
    grid = explorer.Grid()
    grid.add_observation(np.array([0.0, 0.0]), 0.0, one_ray(0.0, 100.0, 150.0, explorer.STOP_OBSTACLE))
    assert cell_at(grid, 0, 50) == explorer.FREE
    assert cell_at(grid, 0, 145) == explorer.FREE
    assert cell_at(grid, 0, 155) == explorer.OBSTACLE
    assert cell_at(grid, 0, 175) == explorer.UNKNOWN


def test_grid_ignores_a_ray_that_saw_no_floor():
    grid = explorer.Grid()
    grid.add_observation(np.array([0.0, 0.0]), 0.0, one_ray(90.0, 100.0, 100.0, explorer.STOP_OBSTACLE))
    assert (grid.cells == explorer.UNKNOWN).all()
