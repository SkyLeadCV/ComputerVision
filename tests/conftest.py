import numpy as np
import pytest

import explorer


@pytest.fixture
def camera_matrix():
    """The calibrated Tello camera, written here because calibration/ is not in the repo."""
    return np.array([[931.5, 0.0, 441.9], [0.0, 924.5, 344.2], [0.0, 0.0, 1.0]])


@pytest.fixture
def output_dirs(tmp_path, monkeypatch):
    """Sends flights/ and explorer/ to a temporary folder so tests leave the repo clean."""
    monkeypatch.setattr(explorer, "FLIGHTS_DIR", tmp_path / "flights")
    monkeypatch.setattr(explorer, "OUTPUT_DIR", tmp_path / "explorer")
    return tmp_path