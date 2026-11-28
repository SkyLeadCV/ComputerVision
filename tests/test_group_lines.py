import numpy as np

from group_lines import find_dominant_family


def towards(point, start_x, start_y, length):
    """A segment from (start_x, start_y) heading straight for point, length px long."""
    start = np.array([start_x, start_y], float)
    direction = (np.array(point, float) - start) / np.linalg.norm(np.array(point, float) - start)
    return np.concatenate([start, start + length * direction])


def test_a_crossing_on_the_floor_is_not_taken_for_a_vanishing_point():
    horizon_point = (480.0, 150.0)
    joints = [towards(horizon_point, x, 700.0, 250.0) for x in (100.0, 350.0, 650.0, 900.0)]
    # Longer lines that all cross at one point on the floor, as joints of both families do.
    crossing = (500.0, 500.0)
    through_crossing = [np.concatenate([np.array(crossing) - 100 * d, np.array(crossing) + 300 * d])
                        for d in (np.array([np.cos(a), np.sin(a)]) for a in np.radians([10.0, 35.0, 145.0, 170.0, 60.0]))]
    segments = np.array(joints + through_crossing)

    vanishing_point, inliers = find_dominant_family(segments, (960, 720))

    assert np.allclose(vanishing_point[:2] / vanishing_point[2], horizon_point, atol=1.0)
    assert inliers[:4].all()
