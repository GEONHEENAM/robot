"""퀴즈3 Modbus 레지스터 맵 (서버 · 확인 도구 · UI 공용).

quiz3_server.py, check_server.py, quiz3_ui.py 가 모두 이 파일을 import 한다.
주소나 배율을 바꿀 때는 이 파일 한 곳만 고친다.

통신 주소는 0부터, 화면 번호(문제지 표기)는 1부터 센다.  화면 번호 = 통신 주소 + 1

    주소 0      작업1 수행 횟수 (정상 완료 후 +1)
    주소 1      작업2 수행 횟수 (정상 완료 후 +1)
    주소 2~9    작업3~10 수행 횟수 (예비)
    주소 10~19  작업2 측정 길이 x10 정수, 1회부터 차례로 (52.3 mm -> 523)
"""

from decimal import ROUND_HALF_UP, Decimal

DEVICE_ID = 1
MODBUS_PORT = 5020

REGISTER_COUNT = 20      # 통신 주소 0~19 (화면 1~20번)

COUNT_START = 0          # 작업 N 수행 횟수 = 주소 COUNT_START + N - 1
COUNT_SLOTS = 10         # 작업 1~10

MEAS_START = 10          # 작업2 측정값 시작 주소
MEAS_SLOTS = 10          # 주소 10~19
MEAS_PER_TASK2 = 3       # 작업2 1회에 측정하는 물체 수

SCALE = 10               # 0.1 mm 단위
RAW_MAX = 0xFFFF         # 16비트 레지스터 최대값


def count_address(task_no):
    """작업 번호(1~10) -> 수행 횟수 주소."""
    if not 1 <= task_no <= COUNT_SLOTS:
        raise ValueError(f"작업 번호 범위 오류: {task_no}")
    return COUNT_START + task_no - 1


def is_measurement(addr):
    return MEAS_START <= addr < MEAS_START + MEAS_SLOTS


def mm_to_raw(mm):
    """측정 길이(mm) -> 레지스터 정수. 10배 후 '사사오입' 반올림.

    Python 기본 round()는 0.5에서 짝수 쪽으로 반올림한다 (52.25 -> 522).
    문제지 규칙(10배 후 반올림)에 맞추려고 Decimal ROUND_HALF_UP을 쓴다 (52.25 -> 523).
    """
    raw = int((Decimal(str(mm)) * SCALE).quantize(Decimal("1"), rounding=ROUND_HALF_UP))
    if not 1 <= raw <= RAW_MAX:
        raise ValueError(f"측정값은 0.1~{RAW_MAX / SCALE} mm 범위여야 함: {mm}")
    return raw


def raw_to_mm(raw):
    return raw / SCALE


def label(addr):
    """주소 -> 사람이 읽는 이름."""
    if COUNT_START <= addr < COUNT_START + COUNT_SLOTS:
        return f"작업{addr - COUNT_START + 1} 수행횟수"
    if is_measurement(addr):
        return f"작업2 측정 {addr - MEAS_START + 1}회"
    return "-"


def describe(addr, raw):
    """주소와 원본 값 -> 표시 문자열 (측정값은 mm 병기)."""
    if is_measurement(addr) and raw != 0:
        return f"{raw} ({raw_to_mm(raw):.1f} mm)"
    return str(raw)
