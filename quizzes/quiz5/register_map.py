"""퀴즈5 Modbus 레지스터 맵 (서버 · UI · YOLO 프로그램 공용).

주소나 배율을 바꿀 때는 이 파일 한 곳만 고친다.
통신 주소는 0부터, 화면 번호는 1부터 센다.  화면 번호 = 통신 주소 + 1

    주소 0~9    작업 N 완료 횟수 (작업1=0, 랜덤R1=2, A이동=3, B이동=4)
    주소 10~19  예비 (퀴즈3의 작업2 측정 자리 — 퀴즈2 미구현으로 사용 안 함)
    주소 20~25  로봇 TCP X, Y, Z, A, B, C  x10 signed (로봇 Modbus 400~405 복사)
    주소 26~31  로봇 관절 J1~J6            x10 signed (로봇 Modbus 270~275 복사)
    주소 32     로봇 상태                  (로봇 Modbus 259 복사)
    주소 40     YOLO 검사한 사진 수
    주소 41     YOLO 개가 검출된 사진 수
    주소 42     YOLO 검출된 개의 수
    주소 43     YOLO 평균 신뢰도 % x100   (82.35% -> 8235, 개 0마리면 0)
    주소 44     YOLO 기준 값 % x100       (80% -> 8000)
    주소 45     YOLO 판별 결과            (0=없음, 1=A, 2=B)
    주소 46     YOLO 판별 횟수            (판별할 때마다 +1, 새 결과 확인용)
    주소 47     사용한 CONF x100          (0.25 -> 25)
    주소 48     사용한 IMGSZ              (640)
    주소 49     사용한 PREPROCESS         (0=original, 1=bright, 2=contrast)
"""

DEVICE_ID = 1
MODBUS_PORT = 5020
REGISTER_COUNT = 50

# ---- 작업 완료 횟수 ----
COUNT_START = 0
COUNT_SLOTS = 10

# ---- 로봇 좌표 미러 (UI가 로봇 Modbus에서 읽어 복사) ----
ROBOT_TCP_START = 20     # 20~25
ROBOT_JOINT_START = 26   # 26~31
ROBOT_STATE_ADDR = 32
LIVE_START, LIVE_END = 20, 32   # 0.2초마다 바뀌는 칸: 서버가 파일 저장·콘솔 출력을 생략

# ---- YOLO 판별 결과 ----
YOLO_PHOTOS = 40
YOLO_PHOTOS_WITH_DOG = 41
YOLO_DOGS = 42
YOLO_MEAN = 43
YOLO_THRESHOLD = 44
YOLO_DECISION = 45
YOLO_RUN_COUNT = 46
YOLO_CONF = 47
YOLO_IMGSZ = 48
YOLO_PREPROCESS = 49

DECISION_CODE = {"A": 1, "B": 2}
DECISION_TEXT = {0: "없음", 1: "A", 2: "B"}
PREPROCESS_CODE = {"original": 0, "bright": 1, "contrast": 2}
PREPROCESS_TEXT = {v: k for k, v in PREPROCESS_CODE.items()}

TCP_NAMES = ["X", "Y", "Z", "A", "B", "C"]

_NAMES = {
    YOLO_PHOTOS: "YOLO 검사 사진 수",
    YOLO_PHOTOS_WITH_DOG: "YOLO 개 검출 사진 수",
    YOLO_DOGS: "YOLO 검출 개 수",
    YOLO_MEAN: "YOLO 평균 신뢰도(%)",
    YOLO_THRESHOLD: "YOLO 기준 값(%)",
    YOLO_DECISION: "YOLO 판별 결과",
    YOLO_RUN_COUNT: "YOLO 판별 횟수",
    YOLO_CONF: "YOLO CONF",
    YOLO_IMGSZ: "YOLO IMGSZ",
    YOLO_PREPROCESS: "YOLO PREPROCESS",
    ROBOT_STATE_ADDR: "로봇 상태",
}
TASK_NAMES = {1: "작업1", 3: "랜덤R1", 4: "A이동", 5: "B이동"}


# ============================================================
# 값 변환
# ============================================================
def to_u16(value: int) -> int:
    """signed 정수 -> 레지스터 값(0~65535). 범위를 넘으면 잘라낸다."""
    value = max(-32768, min(32767, int(value)))
    return value & 0xFFFF


def to_signed(raw: int) -> int:
    return raw - 65536 if raw >= 32768 else raw


def scaled(value: float, scale: int) -> int:
    """실수 -> x scale 레지스터 값 (signed)."""
    return to_u16(round(value * scale))


def is_live(addr: int) -> bool:
    return LIVE_START <= addr <= LIVE_END


def yolo_registers(photos, photos_with_dog, dogs, mean_percent, threshold_percent,
                   decision, run_count, conf, imgsz, preprocess):
    """YOLO 결과 -> 주소 40~49에 쓸 값 10개."""
    return [
        min(photos, 0xFFFF),
        min(photos_with_dog, 0xFFFF),
        min(dogs, 0xFFFF),
        0 if mean_percent is None else round(mean_percent * 100),
        round(threshold_percent * 100),
        DECISION_CODE[decision],
        run_count & 0xFFFF,
        round(conf * 100),
        imgsz,
        PREPROCESS_CODE[preprocess],
    ]


# ============================================================
# 표시용
# ============================================================
def label(addr: int) -> str:
    if COUNT_START <= addr < COUNT_START + COUNT_SLOTS:
        n = addr - COUNT_START + 1
        return f"{TASK_NAMES.get(n, f'작업{n}')} 완료 횟수"
    if 10 <= addr <= 19:
        return "예비"
    if ROBOT_TCP_START <= addr < ROBOT_TCP_START + 6:
        return f"로봇 TCP {TCP_NAMES[addr - ROBOT_TCP_START]}"
    if ROBOT_JOINT_START <= addr < ROBOT_JOINT_START + 6:
        return f"로봇 J{addr - ROBOT_JOINT_START + 1}"
    return _NAMES.get(addr, "-")


def describe(addr: int, raw: int) -> str:
    """주소와 원본 값 -> 사람이 읽는 값."""
    if ROBOT_TCP_START <= addr < ROBOT_JOINT_START + 6:
        return f"{to_signed(raw) / 10:.1f}"
    if addr in (YOLO_MEAN, YOLO_THRESHOLD):
        return f"{raw / 100:.2f}%"
    if addr == YOLO_CONF:
        return f"{raw / 100:.2f}"
    if addr == YOLO_DECISION:
        return DECISION_TEXT.get(raw, f"? ({raw})")
    if addr == YOLO_PREPROCESS:
        return PREPROCESS_TEXT.get(raw, f"? ({raw})")
    return str(raw)
