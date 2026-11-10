"""제스처 판정·실행 판단 테스트 (카메라·모델 불필요)."""

import pytest

from vision.gesture_logic import (
    FIST,
    MULTI,
    ONE,
    OPEN,
    TWO,
    UNKNOWN,
    AutoTaskGate,
    classify_frame,
    classify_hand,
)

WRIST = (300.0, 400.0)
PIP_DIST = 100.0
# 손가락별 방향 (위쪽으로 부채꼴): 검지, 중지, 약지, 소지
DIRS = {6: (-0.3, -1.0), 10: (-0.1, -1.0), 14: (0.1, -1.0), 18: (0.3, -1.0)}


def _hand(index, middle, ring, pinky):
    """각 손가락의 (손끝 거리 / PIP 거리) 비율로 21점 손을 만든다."""
    pts = [WRIST] * 21
    for pip, ratio in zip((6, 10, 14, 18), (index, middle, ring, pinky), strict=True):
        dx, dy = DIRS[pip]
        n = (dx * dx + dy * dy) ** 0.5
        ux, uy = dx / n, dy / n
        pts[pip] = (WRIST[0] + ux * PIP_DIST, WRIST[1] + uy * PIP_DIST)
        pts[pip + 2] = (WRIST[0] + ux * PIP_DIST * ratio, WRIST[1] + uy * PIP_DIST * ratio)
    return pts


UP, DOWN, MID = 1.7, 0.8, 1.15


@pytest.mark.parametrize(
    "fingers, expected",
    [
        ((UP, DOWN, DOWN, DOWN), ONE),
        ((UP, UP, DOWN, DOWN), TWO),
        ((DOWN, DOWN, DOWN, DOWN), FIST),
        ((UP, UP, UP, UP), OPEN),
        ((UP, UP, UP, DOWN), UNKNOWN),      # 세 손가락: 정의되지 않은 제스처
        ((UP, MID, DOWN, DOWN), UNKNOWN),   # 중지가 애매 → 실행 안 함
    ],
)
def test_classify_hand(fingers, expected):
    assert classify_hand(_hand(*fingers)) == expected


def test_low_confidence_is_unknown():
    assert classify_hand(_hand(UP, DOWN, DOWN, DOWN), score=0.3) == UNKNOWN


def test_classify_frame_no_hand_and_multi():
    one = _hand(UP, DOWN, DOWN, DOWN)
    assert classify_frame([]) is None
    assert classify_frame([(one, 0.9), (one, 0.9)]) == MULTI


def _feed(gate, labels, dt=0.1, start=0.0, busy=False):
    """labels를 dt 간격으로 넣고 반환된 작업 번호 목록을 돌려준다."""
    out = []
    t = start
    for lab in labels:
        r = gate.update(lab, t, busy)
        if r is not None:
            out.append(r)
        t += dt
    return out, t


def test_gate_triggers_once_after_hold():
    gate = AutoTaskGate(hold_sec=1.0, release_sec=1.0)
    fired, _ = _feed(gate, [ONE] * 40)          # 4초 동안 계속 같은 제스처
    assert fired == [1]                          # 한 번만 실행


def test_gate_requires_continuous_hold():
    gate = AutoTaskGate(hold_sec=1.0, release_sec=1.0)
    labels = ([ONE] * 6 + [None]) * 5           # 0.6초마다 끊김
    fired, _ = _feed(gate, labels)
    assert fired == []


def test_gate_switching_gesture_restarts_hold():
    gate = AutoTaskGate(hold_sec=1.0, release_sec=1.0)
    labels = ([ONE] * 6 + [TWO] * 6) * 3
    fired, _ = _feed(gate, labels)
    assert fired == []


@pytest.mark.parametrize("label", [None, UNKNOWN, MULTI, FIST, OPEN])
def test_gate_ignores_non_action(label):
    gate = AutoTaskGate(hold_sec=1.0, release_sec=1.0)
    fired, _ = _feed(gate, [label] * 40)
    assert fired == []


def test_gate_rearms_after_release():
    gate = AutoTaskGate(hold_sec=1.0, release_sec=1.0)
    fired, t = _feed(gate, [TWO] * 15)
    assert fired == [2]
    fired, t = _feed(gate, [None] * 5, start=t)  # 0.5초만 내림 → 아직 잠금
    fired2, t = _feed(gate, [TWO] * 15, start=t)
    assert fired + fired2 == []
    _feed_none, t = _feed(gate, [None] * 12, start=t)  # 1.2초 내림 → 해제
    fired3, _ = _feed(gate, [ONE] * 15, start=t)
    assert fired3 == [1]


def test_gate_does_not_count_while_busy():
    gate = AutoTaskGate(hold_sec=1.0, release_sec=1.0)
    fired, t = _feed(gate, [ONE] * 15)
    assert fired == [1]
    # 작업 중에 손을 내려도 해제 시간이 쌓이지 않음
    _, t = _feed(gate, [None] * 30, start=t, busy=True)
    assert gate.locked
    # 작업 중에 다른 제스처를 계속 보여도 실행 안 됨
    gate2 = AutoTaskGate(hold_sec=1.0, release_sec=1.0)
    fired, _ = _feed(gate2, [TWO] * 30, busy=True)
    assert fired == []


def test_detector_reports_missing_model(tmp_path):
    from vision.gesture import HandGestureDetector

    with pytest.raises(FileNotFoundError):
        HandGestureDetector(tmp_path / "missing.task")
