import numpy as np
import pytest

from free_space_fast import CALIBRATION_FILE, FRAMES_DIR, FREE_SPACE_DIR, analyze_frame
from image_io import read_image

# The frames come from the test video and the references from find_free_space.py; neither is in the repo.
FRAMES = sorted(FRAMES_DIR.glob("frame_*.jpg"))
pytestmark = pytest.mark.skipif(
    not (FRAMES and FREE_SPACE_DIR.exists() and CALIBRATION_FILE.exists()),
    reason="needs frames/, free_space/ and calibration/, made locally from the test video",
)


@pytest.fixture(scope="module")
def calibrated_matrix():
    # The exact calibration the references were made with; the rounded K of conftest.py would differ slightly.
    with np.load(CALIBRATION_FILE) as calibration:
        return calibration["camera_matrix"]


@pytest.mark.parametrize("frame_path", FRAMES, ids=[path.stem for path in FRAMES])
def test_analyze_frame_matches_find_free_space(frame_path, calibrated_matrix):
    result = analyze_frame(read_image(frame_path), calibrated_matrix)
    reference_dir = FREE_SPACE_DIR / frame_path.stem
    if not reference_dir.exists():
        assert result is None
        return
    assert result is not None
    with np.load(reference_dir / "free_space.npz") as free, np.load(reference_dir / "moves.npz") as moves:
        np.testing.assert_array_equal(result.angle_deg, free["angles_deg"])
        np.testing.assert_allclose(result.entry_cm, free["entry_cm"])
        np.testing.assert_allclose(result.stop_cm, free["free_distance_cm"])
        np.testing.assert_array_equal(result.kind, free["kind"])
        np.testing.assert_allclose(result.allowed_cm, moves["allowed_cm"])
