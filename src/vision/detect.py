"""카메라에서 가장 큰 빨강 또는 파랑 물체의 중심을 검출한다."""

from __future__ import annotations

import sys
import time

import cv2
import numpy as np


# ========================= 직접 조정할 설정 =========================
CAMERA_INDEX = 1
CAMERA_BACKEND = cv2.CAP_DSHOW if sys.platform == "win32" else cv2.CAP_ANY
MIN_AREA = 100.0

RED_LOW_1 = (0, 170, 150)
RED_HIGH_1 = (20, 255, 255)
RED_LOW_2 = (160, 170, 150)
RED_HIGH_2 = (179, 255, 255)
BLUE_LOW = (100, 170, 150)
BLUE_HIGH = (130, 255, 255)

WINDOW_NAME = "Vision - Color Detection"


def _target_from_mask(
    mask: np.ndarray,
    min_area: float,
) -> tuple[tuple[int, int], float] | None:
    """마스크의 가장 큰 물체에서 (중심 좌표, 면적)을 구한다."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    contour = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(contour)
    if area < min_area:
        return None

    moments = cv2.moments(contour)
    if moments["m00"] == 0:
        return None

    center = (
        int(moments["m10"] / moments["m00"]),
        int(moments["m01"] / moments["m00"]),
    )
    return center, area


def find_target_center(
    bgr: np.ndarray,
    lower_hsv: tuple[int, int, int],
    upper_hsv: tuple[int, int, int],
    min_area: float = 100.0,
) -> tuple[int, int] | None:
    """주어진 HSV 범위에서 가장 큰 물체의 중심을 반환한다."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(lower_hsv), np.array(upper_hsv))
    target = _target_from_mask(mask, min_area)
    return target[0] if target else None


def find_largest_red_or_blue(
    bgr: np.ndarray,
    min_area: float = MIN_AREA,
) -> tuple[tuple[int, int], str] | None:
    """빨강과 파랑 물체 중 더 큰 하나의 중심과 색상 이름을 반환한다."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

    red_1 = cv2.inRange(hsv, np.array(RED_LOW_1), np.array(RED_HIGH_1))
    red_2 = cv2.inRange(hsv, np.array(RED_LOW_2), np.array(RED_HIGH_2))
    red_mask = cv2.bitwise_or(red_1, red_2)
    blue_mask = cv2.inRange(hsv, np.array(BLUE_LOW), np.array(BLUE_HIGH))

    red = _target_from_mask(red_mask, min_area)
    blue = _target_from_mask(blue_mask, min_area)

    if red is None and blue is None:
        return None
    if blue is None or (red is not None and red[1] >= blue[1]):
        return red[0], "Red"
    return blue[0], "Blue"


def run_camera() -> None:
    """카메라 영상을 띄우고 가장 큰 빨강 또는 파랑 물체를 계속 표시한다."""
    open_started = time.perf_counter()
    camera = cv2.VideoCapture(CAMERA_INDEX, CAMERA_BACKEND)
    backend_name = "DirectShow" if CAMERA_BACKEND == cv2.CAP_DSHOW else "Default"
    if not camera.isOpened() and CAMERA_BACKEND != cv2.CAP_ANY:
        camera.release()
        camera = cv2.VideoCapture(CAMERA_INDEX)
        backend_name = "Default fallback"
    open_seconds = time.perf_counter() - open_started
    if not camera.isOpened():
        camera.release()
        raise RuntimeError(f"카메라 {CAMERA_INDEX}번을 열 수 없습니다.")

    first_frame = True
    try:
        while True:
            read_started = time.perf_counter()
            ok, frame = camera.read()
            if not ok:
                raise RuntimeError("카메라 프레임을 읽을 수 없습니다.")
            if first_frame:
                first_read_seconds = time.perf_counter() - read_started
                print(
                    f"VISION_READY,{backend_name},{open_seconds:.3f},{first_read_seconds:.3f}",
                    flush=True,
                )
                first_frame = False

            target = find_largest_red_or_blue(frame)
            if target is None:
                detection = "None"
                status = "Target: not found"
                text_color = (0, 0, 255)
            else:
                (cx, cy), color_name = target
                detection = color_name
                cv2.drawMarker(
                    frame,
                    (cx, cy),
                    (0, 255, 0),
                    cv2.MARKER_CROSS,
                    markerSize=24,
                    thickness=2,
                )
                cv2.circle(frame, (cx, cy), 12, (0, 255, 0), 2)
                status = f"{color_name}: ({cx}, {cy})"
                text_color = (0, 255, 0)

            print(f"DETECTION,{detection}", flush=True)

            cv2.putText(
                frame,
                status,
                (16, 32),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                text_color,
                2,
                cv2.LINE_AA,
            )
            cv2.imshow(WINDOW_NAME, frame)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                break
    finally:
        camera.release()
        cv2.destroyAllWindows()


def main() -> int:
    try:
        run_camera()
    except RuntimeError as error:
        print(f"VISION_ERROR,{error}", flush=True)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
