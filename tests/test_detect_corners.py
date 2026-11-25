import numpy as np
import pytest

from detect_corners import SIDE_SAMPLE_COUNT, edge_coverage


@pytest.mark.parametrize("end", [(959.6, 300.0), (500.0, 719.6), (1200.0, 300.0)],
                         ids=["right_border", "bottom_border", "beyond"])
def test_a_tile_side_reaching_the_image_border_counts_only_what_is_in_view(end):
    edges = np.full((720, 960), 255, np.uint8)
    coverage = edge_coverage(np.array([400.0, 300.0]), np.array(end), edges)
    assert 0.0 < coverage <= 1.0


def test_a_side_inside_the_image_is_covered_where_the_edges_are():
    edges = np.zeros((720, 960), np.uint8)
    edges[300, 400:500] = 255
    assert edge_coverage(np.array([400.0, 300.0]), np.array([599.0, 300.0]), edges) == pytest.approx(0.5, abs=0.05)


def test_only_the_sample_past_the_border_counts_as_uncovered():
    # A side up to x = 959.6 rounds its last sample to 960: that one sample, and no other, is uncovered.
    edges = np.full((720, 960), 255, np.uint8)
    coverage = edge_coverage(np.array([900.0, 400.0]), np.array([959.6, 400.0]), edges)
    assert coverage == (SIDE_SAMPLE_COUNT - 1) / SIDE_SAMPLE_COUNT


def test_a_side_half_past_the_border_is_half_covered():
    edges = np.full((720, 960), 255, np.uint8)
    coverage = edge_coverage(np.array([860.0, 400.0]), np.array([1060.0, 400.0]), edges)
    assert abs(coverage - 0.5) <= 1.5 / SIDE_SAMPLE_COUNT
