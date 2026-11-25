import numpy as np
import pytest

from detect_corners import edge_coverage


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
