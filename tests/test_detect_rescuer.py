import numpy as np
import pytest

import detect_rescuer


def test_a_face_box_twice_as_wide_is_half_as_far(camera_matrix):
    width_at_2m = camera_matrix[0, 0] * detect_rescuer.FACE_BOX_WIDTH_CM / 200
    assert detect_rescuer.face_distance_cm(width_at_2m, camera_matrix) == pytest.approx(200)
    assert detect_rescuer.face_distance_cm(2 * width_at_2m, camera_matrix) == pytest.approx(100)


def test_the_nearest_face_is_the_rescuer(camera_matrix, monkeypatch):
    # A false face on a hoodie, then the real one: the widest box wins wherever it comes in the list.
    faces = np.array([[233, 410, 76, 76], [315, 74, 102, 102]])
    monkeypatch.setattr(detect_rescuer, "detect_faces", lambda photo: faces)

    rescuer = detect_rescuer.find_rescuer(np.zeros((720, 960, 3), np.uint8), camera_matrix)

    assert rescuer.box == (315, 74, 102, 102)
    assert rescuer.distance_cm == pytest.approx(detect_rescuer.face_distance_cm(102, camera_matrix))


def test_no_face_on_an_empty_floor(camera_matrix):
    # The floor tiles' colour, with their joints every 60 px: the real cascade must load and find nothing.
    photo = np.full((720, 960, 3), (118, 137, 130), np.uint8)
    photo[::60] = photo[:, ::60] = (64, 79, 76)
    assert detect_rescuer.find_rescuer(photo, camera_matrix) is None


def test_the_photo_name_gives_the_real_distance():
    assert detect_rescuer.real_distance_cm(detect_rescuer.Path("face_150cm_diego_6.jpg")) == 150
    assert detect_rescuer.real_distance_cm(detect_rescuer.Path("before.jpg")) is None
