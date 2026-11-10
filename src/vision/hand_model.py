"""MediaPipe 손 모델 파일 위치. cv2·mediapipe를 import하지 않는다.

tools/download_hand_model.py 가 OpenCV 설치 전에도 돌 수 있게 경로만 따로 둔다.
"""

from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
MODEL_PATH = PROJECT_ROOT / "models" / "hand_landmarker.task"
MODEL_URL = (
    "https://storage.googleapis.com/mediapipe-models/hand_landmarker/"
    "hand_landmarker/float16/latest/hand_landmarker.task"
)
