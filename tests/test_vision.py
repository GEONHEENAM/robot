import numpy as np

from vision.detect import find_largest_red_or_blue, find_target_center

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


def test_selects_larger_red_target():
    img = _blank()
    img[50:150, 50:150] = (0, 0, 255)
    img[200:250, 300:350] = (255, 0, 0)

    center, color = find_largest_red_or_blue(img)

    assert color == "Red"
    assert abs(center[0] - 99) <= 1 and abs(center[1] - 99) <= 1


def test_selects_larger_blue_target():
    img = _blank()
    img[50:100, 50:100] = (0, 0, 255)
    img[200:300, 300:400] = (255, 0, 0)

    center, color = find_largest_red_or_blue(img)

    assert color == "Blue"
    assert abs(center[0] - 349) <= 1 and abs(center[1] - 249) <= 1
