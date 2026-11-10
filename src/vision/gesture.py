"""MediaPipe Hand Landmarker로 손 스켈레톤을 검출하고 제스처를 판정한다.

- mediapipe 1.2.x Tasks API 기준 (mp.solutions 는 1.x 에서 제거되어 사용하지 않음)
- 모델 파일: models/hand_landmarker.task  (tools/download_hand_model.py 로 받음)
- 단독 실행: python src/vision/gesture.py   → 로봇 없이 판정만 확인 (q / ESC 종료)
"""

from __future__ import annotations

import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from vision.gesture_logic import (  # noqa: E402
    GESTURE_TO_TASK,
    MULTI,
    UNKNOWN,
    AutoTaskGate,
    classify_frame,
)
from vision.hand_model import MODEL_PATH, MODEL_URL  # noqa: E402,F401

CAMERA_INDEX = 0
CAMERA_BACKEND = cv2.CAP_DSHOW if sys.platform == "win32" else cv2.CAP_ANY
MAX_HANDS = 2             # 2개까지 찾아야 '손 여러 개 → 보류'를 판단할 수 있다
MIRROR = True             # 거울 모드 (사용자가 보기 자연스럽게)

LABEL_COLOR = {           # BGR
    "ONE": (60, 60, 230),
    "TWO": (230, 120, 40),
    "FIST": (160, 160, 160),
    "OPEN": (160, 160, 160),
    UNKNOWN: (0, 200, 255),
    MULTI: (0, 200, 255),
}


@dataclass
class GestureResult:
    label: str | None                                   # 프레임 판정 (None = 손 없음)
    hands: list[list[tuple[int, int]]] = field(default_factory=list)  # 픽셀 좌표
    scores: list[float] = field(default_factory=list)


class HandGestureDetector:
    """카메라 프레임(BGR) → GestureResult. 한 스레드에서만 사용한다."""

    def __init__(self, model_path: Path | str = MODEL_PATH, max_hands: int = MAX_HANDS):
        model_path = Path(model_path)
        if not model_path.is_file():
            raise FileNotFoundError(
                f"모델 파일이 없습니다: {model_path}\n"
                "python tools/download_hand_model.py 로 받으세요."
            )
        try:
            import mediapipe as mp
        except ImportError as e:
            raise RuntimeError(
                "mediapipe가 설치되지 않았습니다: pip install -r requirements.txt"
            ) from e

        self._mp = mp
        vision = mp.tasks.vision
        options = vision.HandLandmarkerOptions(
            base_options=mp.tasks.BaseOptions(model_asset_path=str(model_path)),
            running_mode=vision.RunningMode.VIDEO,
            num_hands=max_hands,
            min_hand_detection_confidence=0.5,
            min_hand_presence_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        self._landmarker = vision.HandLandmarker.create_from_options(options)
        hand_connections = vision.HandLandmarksConnections.HAND_CONNECTIONS
        self.connections = [(c.start, c.end) for c in hand_connections]
        self._last_ts = -1

    def detect(self, bgr: np.ndarray) -> GestureResult:
        h, w = bgr.shape[:2]
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        image = self._mp.Image(image_format=self._mp.ImageFormat.SRGB, data=rgb)

        # VIDEO 모드는 타임스탬프가 반드시 증가해야 한다 (같거나 작으면 예외)
        ts = int(time.monotonic() * 1000)
        if ts <= self._last_ts:
            ts = self._last_ts + 1
        self._last_ts = ts

        result = self._landmarker.detect_for_video(image, ts)

        hands_px: list[list[tuple[int, int]]] = []
        scores: list[float] = []
        for i, lms in enumerate(result.hand_landmarks):
            # 정규화 좌표 → 픽셀 좌표 (판정 비율 왜곡 방지)
            hands_px.append([(int(p.x * w), int(p.y * h)) for p in lms])
            cats = result.handedness[i] if i < len(result.handedness) else []
            scores.append(cats[0].score if cats else 0.0)

        label = classify_frame(list(zip(hands_px, scores, strict=True)))
        return GestureResult(label, hands_px, scores)

    def close(self) -> None:
        self._landmarker.close()


def draw_overlay(frame: np.ndarray, result: GestureResult,
                 connections: list[tuple[int, int]], status: str = "") -> None:
    """스켈레톤 + 손 테두리 + 제스처 이름을 프레임에 그린다."""
    color = LABEL_COLOR.get(result.label or "", (0, 200, 0))
    for pts in result.hands:
        for a, b in connections:
            cv2.line(frame, pts[a], pts[b], (0, 255, 0), 2, cv2.LINE_AA)
        for p in pts:
            cv2.circle(frame, p, 3, (255, 255, 255), -1, cv2.LINE_AA)
        xs, ys = [p[0] for p in pts], [p[1] for p in pts]
        x1, y1, x2, y2 = min(xs) - 15, min(ys) - 15, max(xs) + 15, max(ys) + 15
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        cv2.putText(frame, result.label or "", (x1, max(20, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2, cv2.LINE_AA)

    top = result.label or "No hand"
    task = GESTURE_TO_TASK.get(result.label or "")
    if task:
        top += " -> TASK R1" if task == 3 else f" -> TASK {task}"
    if status:
        top += f" | {status}"
    cv2.putText(frame, top, (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)


def open_camera(index: int = CAMERA_INDEX) -> cv2.VideoCapture:
    camera = cv2.VideoCapture(index, CAMERA_BACKEND)
    if not camera.isOpened() and CAMERA_BACKEND != cv2.CAP_ANY:
        camera.release()
        camera = cv2.VideoCapture(index)
    if not camera.isOpened():
        camera.release()
        raise RuntimeError(f"카메라 {index}번을 열 수 없습니다.")
    return camera


def main() -> int:
    """로봇 없이 제스처 판정·실행 판단만 확인하는 미리보기."""
    try:
        detector = HandGestureDetector()
        camera = open_camera()
    except (FileNotFoundError, RuntimeError) as e:
        print(f"GESTURE_ERROR,{e}", flush=True)
        return 1

    gate = AutoTaskGate()
    try:
        while True:
            ok, frame = camera.read()
            if not ok:
                print("GESTURE_ERROR,카메라 프레임을 읽을 수 없습니다.", flush=True)
                return 1
            if MIRROR:
                frame = cv2.flip(frame, 1)
            res = detector.detect(frame)
            now = time.monotonic()
            task = gate.update(res.label, now, busy=False)
            if task:
                print(f"TRIGGER,TASK,{task}", flush=True)
            status = "LOCKED" if gate.locked else f"{gate.progress(now) * 100:.0f}%"
            draw_overlay(frame, res, detector.connections, status)
            cv2.imshow("Gesture - MediaPipe Hand", frame)
            if (cv2.waitKey(1) & 0xFF) in (ord("q"), ord("Q"), 27):
                break
    finally:
        camera.release()
        detector.close()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
