"""최종 프로젝트 공통 설정.

IP·포트·카메라·좌표는 이 파일 한 곳만 고친다.
GUI(final_gui.py), 검출(vision.py), 가짜 로봇(fake_robot.py)이 모두 이 파일을 import 한다.
"""

from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent

# ============================================================
# 로봇 통신 (퀴즈3과 동일: DRL이 TCP 서버, PC GUI가 클라이언트)
# ============================================================
ROBOT_IP = "192.168.137.100"
ROBOT_PORT = 9000
LOCAL_IP = "127.0.0.1"  # fake_robot.py로 테스트할 때 접속할 주소

# ============================================================
# Modbus (선택: 퀴즈3 server.py로 작업 횟수를 기록할 때만 사용)
# ============================================================
MODBUS_IP = "127.0.0.1"
MODBUS_PORT = 5020

# ============================================================
# 카메라
# ============================================================
CAMERA_INDEX = 0  # 노트북 내장 카메라 = 보통 0, USB 웹캠 = 보통 1
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720
CAMERA_USE_DSHOW = True  # 윈도우 DirectShow 백엔드 (카메라 속성 제어가 더 잘 됨)
CAMERA_DISABLE_AUTOFOCUS = True  # 자동초점 끄기 요청 (내장 카메라는 무시되는 경우가 많음)
CAMERA_FOCUS = 0  # 자동초점을 끈 뒤 고정할 초점 값 (웹캠마다 다름 → 현장 조정)
CAMERA_LOCK_EXPOSURE = False  # 노출 고정 (웹캠이 오면 True로 바꿔서 시험)
CAMERA_EXPOSURE = -6  # DirectShow 노출 값 (로그 스케일, 웹캠마다 다름)

# ============================================================
# 파일 경로
# ============================================================
VISION_PARAMS_FILE = BASE_DIR / "vision_params.json"  # GUI/단독 실행에서 저장한 HSV 등
HOMOGRAPHY_FILE = BASE_DIR / "calib" / "H.npy"  # 3단계 캘리브레이션 결과 (픽셀 → 로봇 XY)
CAMERA_CALIB_FILE = BASE_DIR / "calib" / "camera.npz"  # (선택) 렌즈 왜곡 보정값 K, dist
CAPTURE_DIR = BASE_DIR / "captures"  # 저장한 프레임
LOG_DIR = BASE_DIR / "logs"  # 명령 이력 CSV 등

# ============================================================
# 캘리브레이션 전 임시 변환 (GUI·가짜 로봇 테스트 전용)
#   H.npy가 없으면 화면 전체를 아래 범위에 그대로 늘려 붙인다.
#   실제 위치와 전혀 맞지 않으므로 GUI는 이 상태에서 "실제 로봇"으로 PICK을 보내지 않는다.
# ============================================================
ALLOW_DUMMY_HOMOGRAPHY = True
DUMMY_X_RANGE = (300.0, -300.0)  # 이미지 왼쪽 끝 → 오른쪽 끝에 대응하는 로봇 x (임시)
DUMMY_Y_RANGE = (700.0, 380.0)  # 이미지 위 끝 → 아래 끝에 대응하는 로봇 y (임시)

# ============================================================
# 작업 공간 (로봇 베이스 좌표, mm)
#   집기 영역: 막대가 놓이는 곳. 이 밖의 막대는 집지 않는다.
#   놓기 칸: 퀴즈2·3의 PLACE_UP XY 그대로. 번호는 1부터.
#   ※ 놓기 칸(y=600)과 집기 영역이 겹치지 않게 현장에서 다시 정할 것
# ============================================================
PICK_X_RANGE = (-250.0, 250.0)
PICK_Y_RANGE = (420.0, 560.0)
PLACE_SLOTS = {
    1: (200.0, 600.0),
    2: (0.0, 600.0),
    3: (-200.0, 600.0),
}

# 작업자 기준 "왼쪽"이 로봇 +x 방향이면 True (5단계 해석기에서 사용, 현장에서 확인)
#   작업자가 서는 자리에서 로봇 +x 방향이 왼손 쪽이면 True, 오른손 쪽이면 False
LEFT_IS_POSITIVE_X = True

# ============================================================
# 음성 인식 (6단계, Vosk)
#   모델: https://alphacephei.com/vosk/models 에서 vosk-model-small-ko-0.22.zip 을 받아
#         models 폴더에 압축 해제 → models/vosk-model-small-ko-0.22/ (안에 am, conf 등 폴더)
# ============================================================
VOSK_MODEL_DIR = BASE_DIR / "models" / "vosk-model-small-ko-0.22"
STT_DEVICE = None  # 마이크 장치 번호 (None = 윈도우 기본 마이크). 목록: python stt.py --devices
STT_MODE = "grammar_then_free"  # "grammar_then_free" | "free" | "grammar"
STT_MAX_SECONDS = 8.0  # 누르고 있어도 이 시간이 지나면 녹음 종료
STT_MIN_SECONDS = 0.4  # 이보다 짧은 녹음은 무시 (버튼 실수)
