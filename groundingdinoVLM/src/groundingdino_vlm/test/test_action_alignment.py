from types import SimpleNamespace

import pytest

from groundingdino_vlm.core import Candidate, overlap_over_smaller


def roi(x, y, width, height):
    return SimpleNamespace(x_offset=x, y_offset=y, width=width, height=height)


def test_small_lidar_target_inside_large_detection_is_aligned():
    candidate = Candidate("fabric curtain", 0.91, (10, 10, 110, 210))
    assert overlap_over_smaller(candidate, roi(45, 80, 8, 16)) == pytest.approx(1.0)


def test_spatially_separate_detection_is_not_aligned():
    candidate = Candidate("chair", 0.92, (200, 100, 300, 240))
    assert overlap_over_smaller(candidate, roi(20, 20, 30, 40)) == 0.0
