"""OpenCV 색상 기반 물체 중심 검출.

카메라(cv2.VideoCapture)와 분리된 순수 함수로 만들어서 CI에서 합성 이미지로 테스트할 수 있게 했다.
"""

from __future__ import annotations

import cv2
import numpy as np


def find_target_center(
    bgr: np.ndarray,
    lower_hsv: tuple[int, int, int],
    upper_hsv: tuple[int, int, int],
    min_area: float = 100.0,
) -> tuple[int, int] | None:
    """HSV 범위 안의 가장 큰 덩어리 중심 픽셀 (cx, cy). 없으면 None."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(lower_hsv), np.array(upper_hsv))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) < min_area:
        return None
    m = cv2.moments(largest)
    if m["m00"] == 0:
        return None
    return int(m["m10"] / m["m00"]), int(m["m01"] / m["m00"])
