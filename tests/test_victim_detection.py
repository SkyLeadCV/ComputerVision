import math

import numpy as np
import pytest

import victim_detection
from victim_detection import Person

CAMERA_HEIGHT_CM = 80.0
TILT_DEG = 11.0


def camera_from_room(tilt_deg):
    """Rotation from a level frame (x right, y down, z forward) to a camera looking tilt_deg down."""
    t = math.radians(tilt_deg)
    return np.array([[1.0, 0.0, 0.0], [0.0, math.cos(t), -math.sin(t)], [0.0, math.sin(t), math.cos(t)]])


def pixel(point_cm, rotation, camera_matrix):
    projected = camera_matrix @ (rotation @ point_cm)
    return projected[:2] / projected[2]


@pytest.mark.parametrize("tilt_deg", [0.0, TILT_DEG])
@pytest.mark.parametrize("top_cm", [30.0, 170.0])
def test_top_height_of_a_box_standing_on_the_floor(top_cm, tilt_deg, camera_matrix):
    rotation = camera_from_room(tilt_deg)
    # A point on the floor 250 cm ahead and 40 cm to the right, and one straight above it.
    feet = pixel(np.array([40.0, CAMERA_HEIGHT_CM, 250.0]), rotation, camera_matrix)
    head = pixel(np.array([40.0, CAMERA_HEIGHT_CM - top_cm, 250.0]), rotation, camera_matrix)
    floor_normal = rotation @ np.array([0.0, 1.0, 0.0])

    height = victim_detection.top_height_cm(feet, head[1], np.linalg.inv(camera_matrix), floor_normal, CAMERA_HEIGHT_CM)

    assert height == pytest.approx(top_cm, abs=1.0)


def place(box, camera_matrix):
    floor_normal = camera_from_room(TILT_DEG) @ np.array([0.0, 1.0, 0.0])
    return victim_detection.place(Person(np.array(box, float), 0.9), 720, np.linalg.inv(camera_matrix),
                                  floor_normal, CAMERA_HEIGHT_CM)


def test_a_lying_person_is_placed_at_the_nearest_point_of_the_body(camera_matrix):
    rotation = camera_from_room(TILT_DEG)
    feet = pixel(np.array([0.0, CAMERA_HEIGHT_CM, 250.0]), rotation, camera_matrix)
    head = pixel(np.array([0.0, CAMERA_HEIGHT_CM - 30.0, 250.0]), rotation, camera_matrix)

    victim = place([feet[0] - 60, head[1], feet[0] + 60, feet[1]], camera_matrix)

    assert victim is not None
    assert victim.floor_cm == pytest.approx([0.0, 250.0], abs=2.0)


def test_a_standing_person_is_not_a_victim(camera_matrix):
    rotation = camera_from_room(TILT_DEG)
    feet = pixel(np.array([0.0, CAMERA_HEIGHT_CM, 250.0]), rotation, camera_matrix)
    head = pixel(np.array([0.0, CAMERA_HEIGHT_CM - 170.0, 250.0]), rotation, camera_matrix)
    assert place([feet[0] - 30, head[1], feet[0] + 30, feet[1]], camera_matrix) is None


def test_a_box_cut_by_the_bottom_of_the_image_is_not_placed(camera_matrix):
    assert place([400, 500, 560, 719], camera_matrix) is None
