import numpy as np

from detect_corners import SIDE_SAMPLE_COUNT, edge_coverage


def test_a_side_ending_on_the_image_border_does_not_index_outside_it():
    # A real flight photo ended its mission here: a side up to x = 959.6 rounds its last sample to 960.
    edges = np.full((720, 960), 255, np.uint8)
    coverage = edge_coverage(np.array([900.0, 400.0]), np.array([959.6, 400.0]), edges)
    # Only that last sample falls outside, and counts as uncovered.
    assert coverage == (SIDE_SAMPLE_COUNT - 1) / SIDE_SAMPLE_COUNT


def test_samples_past_the_border_count_as_uncovered():
    edges = np.full((720, 960), 255, np.uint8)
    # Half of this side lies beyond the right border.
    coverage = edge_coverage(np.array([860.0, 400.0]), np.array([1060.0, 400.0]), edges)
    assert abs(coverage - 0.5) <= 1.5 / SIDE_SAMPLE_COUNT
