"""손 스켈레톤(21개 랜드마크) → 제스처 판정, 제스처 → 로봇 작업 실행 판단.

MediaPipe를 import하지 않는다. 카메라·모델 없이 pytest로 검증할 수 있게
판정 로직만 이 파일에 둔다. (MediaPipe 호출은 gesture.py)

제스처 정의 (엄지는 사람마다 펴는 방식이 달라 판정에서 제외)
    ONE   검지만 폄             → 작업 1
    TWO   검지 + 중지만 폄      → 작업 2
    FIST  네 손가락 모두 접음   → 실행 없음
    OPEN  네 손가락 모두 폄     → 실행 없음
    UNKNOWN  판정 불확실(애매한 손가락, 낮은 신뢰도) → 실행 없음
    MULTI    손이 2개 이상      → 랜덤 동작 TaskRandom1
    None     손 미검출          → 실행 없음
"""

from __future__ import annotations

import math
from collections.abc import Sequence

# ========================= 랜드마크 번호 =========================
WRIST = 0
# (손가락 이름, PIP 관절, 손끝) — 엄지 제외
FINGERS = (
    ("index", 6, 8),
    ("middle", 10, 12),
    ("ring", 14, 16),
    ("pinky", 18, 20),
)

# ========================= 판정 기준 =========================
# 손목→손끝 거리 / 손목→PIP 거리
#   펴면 손끝이 PIP보다 훨씬 멀고, 접으면 손끝이 손목 쪽으로 돌아온다.
#   두 기준 사이는 '애매' → UNKNOWN 으로 처리해 실행하지 않는다.
EXTENDED_RATIO = 1.25
FOLDED_RATIO = 1.05
MIN_HAND_SCORE = 0.7      # handedness 신뢰도 하한

ONE, TWO, FIST, OPEN = "ONE", "TWO", "FIST", "OPEN"
UNKNOWN, MULTI = "UNKNOWN", "MULTI"

GESTURE_TO_TASK = {ONE: 1, TWO: 2, MULTI: 3}

Point = tuple[float, float]


def _dist(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def finger_states(points: Sequence[Point]) -> dict[str, bool | None]:
    """손가락별 펴짐 상태. True=폄, False=접음, None=애매.

    points: 픽셀 좌표 (x, y) 21개. 정규화 좌표(0~1)를 그대로 넣으면
    가로·세로 배율이 달라 비율이 왜곡되므로 반드시 픽셀로 바꿔서 넣는다.
    """
    if len(points) != 21:
        raise ValueError(f"랜드마크는 21개여야 함: {len(points)}")
    wrist = points[WRIST]
    states: dict[str, bool | None] = {}
    for name, pip, tip in FINGERS:
        d_pip = _dist(wrist, points[pip])
        if d_pip < 1e-6:
            states[name] = None
            continue
        ratio = _dist(wrist, points[tip]) / d_pip
        if ratio >= EXTENDED_RATIO:
            states[name] = True
        elif ratio <= FOLDED_RATIO:
            states[name] = False
        else:
            states[name] = None
    return states


def classify_hand(points: Sequence[Point], score: float = 1.0) -> str:
    """손 하나의 제스처 이름을 반환한다."""
    if score < MIN_HAND_SCORE:
        return UNKNOWN
    s = finger_states(points)
    if any(v is None for v in s.values()):
        return UNKNOWN
    pattern = (s["index"], s["middle"], s["ring"], s["pinky"])
    return {
        (True, False, False, False): ONE,
        (True, True, False, False): TWO,
        (False, False, False, False): FIST,
        (True, True, True, True): OPEN,
    }.get(pattern, UNKNOWN)


def classify_frame(hands: Sequence[tuple[Sequence[Point], float]]) -> str | None:
    """프레임 전체 판정. hands = [(픽셀 좌표 21개, 신뢰도), ...]"""
    if not hands:
        return None
    if len(hands) > 1:
        return MULTI
    points, score = hands[0]
    return classify_hand(points, score)


class AutoTaskGate:
    """제스처가 일정 시간 유지되면 작업을 한 번만 실행하게 하는 판단기.

    - 같은 실행 제스처가 hold_sec 동안 끊김 없이 유지 → 작업 번호 반환
    - 반환 직후 잠금: 실행 제스처가 아닌 상태(손 내림 등)가
      release_sec 동안 이어져야 다음 실행 허용 → 같은 제스처로 중복 실행 없음
    - 작업 중(busy)에는 유지 시간도, 해제 시간도 쌓지 않는다
    시간은 호출하는 쪽이 넘긴다(time.monotonic). 테스트에서 시간을 조작하기 위함.
    """

    def __init__(self, hold_sec: float = 1.0, release_sec: float = 1.0,
                 mapping: dict[str, int] | None = None):
        self.hold_sec = hold_sec
        self.release_sec = release_sec
        self.mapping = dict(GESTURE_TO_TASK if mapping is None else mapping)
        self.reset()

    def reset(self) -> None:
        self.candidate: str | None = None
        self.candidate_since = 0.0
        self.locked = False
        self.release_since: float | None = None

    def progress(self, now: float) -> float:
        """현재 후보 제스처의 유지 진행률 0~1 (UI 표시용)."""
        if self.locked or self.candidate is None or self.hold_sec <= 0:
            return 0.0
        return min(1.0, (now - self.candidate_since) / self.hold_sec)

    def update(self, label: str | None, now: float, busy: bool) -> int | None:
        is_action = label in self.mapping

        if busy:
            self.candidate = None
            self.release_since = None
            return None

        if self.locked:
            if is_action:
                self.release_since = None
                return None
            if self.release_since is None:
                self.release_since = now
            if now - self.release_since >= self.release_sec:
                self.locked = False
                self.release_since = None
            return None

        if not is_action:
            self.candidate = None
            return None

        if label != self.candidate:
            self.candidate = label
            self.candidate_since = now
            return None

        if now - self.candidate_since >= self.hold_sec:
            task = self.mapping[label]
            self.candidate = None
            self.locked = True
            self.release_since = None
            return task
        return None
