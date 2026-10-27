"""퀴즈3 Modbus 레지스터 맵 (서버 · 확인 도구 공용).

팀 UI(ui/quiz3_gui.py)가 쓰는 배치와 같아야 한다. UI 쪽 상수:
    TASK_COUNT_START = 0, TASK_COUNT_SIZE = 10, TASK2_Z_START = 10, TASK2_Z_SIZE = 10, SCALE = 10
UI의 배치를 바꾸면 이 파일도 같이 바꾼다.

통신 주소는 0부터, 화면 번호(문제지 표기)는 1부터 센다.  화면 번호 = 통신 주소 + 1

    주소 0      작업1 완료 횟수 (정상 완료 후 +1)
    주소 1      작업2 완료 횟수 (정상 완료 후 +1)
    주소 2~9    작업3~10 완료 횟수 (예비)
    주소 10~19  작업2 측정 Z 높이 x10, 부호 있는 16비트 (측정 ID 1 -> 주소 10)
                UI가 작업2를 시작할 때 10~19를 0으로 비우고, 이번 실행 값만 남긴다
"""

DEVICE_ID = 1
MODBUS_PORT = 502        # UI 기본값(DEFAULT_MB_PORT)과 같게

REGISTER_COUNT = 20      # 통신 주소 0~19 (화면 1~20번)

COUNT_START = 0          # 작업 N 완료 횟수 = 주소 COUNT_START + N - 1
COUNT_SLOTS = 10         # 작업 1~10

MEAS_START = 10          # 작업2 측정값 시작 주소
MEAS_SLOTS = 10          # 주소 10~19

SCALE = 10               # 0.1 mm 단위


def count_address(task_no):
    """작업 번호(1~10) -> 완료 횟수 주소."""
    if not 1 <= task_no <= COUNT_SLOTS:
        raise ValueError(f"작업 번호 범위 오류: {task_no}")
    return COUNT_START + task_no - 1


def is_count(addr):
    return COUNT_START <= addr < COUNT_START + COUNT_SLOTS


def is_measurement(addr):
    return MEAS_START <= addr < MEAS_START + MEAS_SLOTS


def to_signed(raw):
    """레지스터 값(0~65535) -> 부호 있는 정수(-32768~32767)."""
    return raw - 65536 if raw >= 32768 else raw


def raw_to_mm(raw):
    """측정 레지스터 값 -> mm (UI의 from_reg와 같은 규칙: 부호 있는 값 / 10)."""
    return to_signed(raw) / SCALE


def label(addr):
    """주소 -> 사람이 읽는 이름."""
    if is_count(addr):
        return f"작업{addr - COUNT_START + 1} 완료횟수"
    if is_measurement(addr):
        return f"작업2 Z높이 #{addr - MEAS_START + 1}"
    return "-"


def describe(addr, raw):
    """주소와 원본 값 -> 표시 문자열 (측정값은 mm 병기)."""
    if is_measurement(addr) and raw != 0:
        return f"{raw} ({raw_to_mm(raw):.1f} mm)"
    return str(raw)
