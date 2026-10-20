import numpy as np

from vision.detect import find_target_center

BLUE_LO = (100, 150, 50)
BLUE_HI = (130, 255, 255)


def _blank():
    return np.zeros((480, 640, 3), dtype=np.uint8)


def test_detects_blue_square_center():
    img = _blank()
    img[200:260, 300:360] = (255, 0, 0)  # BGR 파란색, 중심 ≈ (329, 229)
    cx, cy = find_target_center(img, BLUE_LO, BLUE_HI)
    assert abs(cx - 329) <= 1 and abs(cy - 229) <= 1


def test_returns_none_when_no_target():
    assert find_target_center(_blank(), BLUE_LO, BLUE_HI) is None


def test_ignores_small_noise():
    img = _blank()
    img[10:13, 10:13] = (255, 0, 0)  # 9px짜리 노이즈
    assert find_target_center(img, BLUE_LO, BLUE_HI, min_area=100) is None
