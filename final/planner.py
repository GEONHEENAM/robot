"""
실행 계획 (5단계): Intent + 스냅샷의 막대 목록 → 로봇이 할 일 목록(Plan)

  make_plan(intent, sticks, default_slot) → PlanResult
  rematch(stick, new_sticks)              → 정렬처럼 여러 번 집을 때, 다음 막대를 새 스냅샷에서 다시 찾기

원칙
  - 집기 영역 안 + 로봇 좌표가 있는 막대만 후보
  - 애매하면 실행하지 않고 오류로 돌려준다 (GUI가 사용자에게 다시 물음)
      · 막대가 2개인데 "가운데"
      · "왼쪽" 후보 두 개의 x 차이가 AMBIG_X_MM 미만
      · "긴/짧은" 후보 두 개의 길이 차이가 AMBIG_LEN_MM 미만
  - "왼쪽/오른쪽"은 작업자 기준 (config.LEFT_IS_POSITIVE_X)
  - 놓기 칸의 왼쪽/가운데/오른쪽도 같은 기준으로 정한다
"""

from dataclasses import dataclass, field

import numpy as np

import config

AMBIG_X_MM = 25.0       # 좌우 판단: 두 막대의 x 차이가 이보다 작으면 애매
AMBIG_LEN_MM = 5.0      # 길이 판단: 길이 차이가 이보다 작으면 애매
REMATCH_TOL_MM = 25.0   # 다음 스냅샷에서 같은 막대로 볼 최대 위치 차이
REMATCH_LEN_MM = 15.0   # 같은 막대로 볼 최대 길이 차이


# ============================================================
# 작업자 기준 정렬
# ============================================================
def left_to_right(sticks):
    """작업자 기준 왼쪽 → 오른쪽 순서."""
    return sorted(sticks, key=lambda s: s.x, reverse=config.LEFT_IS_POSITIVE_X)


def slots_left_to_right():
    items = sorted(config.PLACE_SLOTS.items(), key=lambda kv: kv[1][0], reverse=config.LEFT_IS_POSITIVE_X)
    return [n for n, _ in items]


def side_to_slot(side):
    order = slots_left_to_right()
    return {"left": order[0], "center": order[len(order) // 2], "right": order[-1]}[side]


# ============================================================
# 결과 형식
# ============================================================
@dataclass
class Step:
    stick: object          # vision.Stick
    slot: int

    def label(self):
        return f"#{self.stick.idx} ({self.stick.length:.0f}mm) → P{self.slot}"


@dataclass
class Plan:
    action: str                          # "PICK" | "SORT" | "HOME"
    steps: list = field(default_factory=list)
    summary: str = ""
    notes: list = field(default_factory=list)


@dataclass
class PlanResult:
    plan: Plan | None = None
    error: str | None = None


# ============================================================
# 대상 선택
# ============================================================
def _select(target, cands):
    """(막대, 오류문자열, 참고문구) 반환."""
    kind, val = target
    if kind == "only":
        if len(cands) == 1:
            return cands[0], None, None
        return None, f"막대가 {len(cands)}개라 어떤 막대인지 말해 주세요 (예: 왼쪽 막대, 긴 막대)", None

    if kind == "position":
        order = left_to_right(cands)
        if len(order) == 1:
            return order[0], None, "막대가 하나뿐이라 그 막대를 선택"
        if val == "left":
            if abs(order[0].x - order[1].x) < AMBIG_X_MM:
                return None, "왼쪽 막대가 두 개 비슷한 위치에 있음 → 길이나 순서로 말해 주세요", None
            return order[0], None, None
        if val == "right":
            if abs(order[-1].x - order[-2].x) < AMBIG_X_MM:
                return None, "오른쪽 막대가 두 개 비슷한 위치에 있음 → 길이나 순서로 말해 주세요", None
            return order[-1], None, None
        # center
        if len(order) % 2 == 0:
            return None, f"막대가 {len(order)}개라 가운데를 정할 수 없음", None
        mid = len(order) // 2
        if min(abs(order[mid].x - order[mid - 1].x), abs(order[mid + 1].x - order[mid].x)) < AMBIG_X_MM:
            return None, "가운데 막대와 옆 막대의 위치가 너무 가까움", None
        return order[mid], None, None

    if kind == "length":
        order = sorted(cands, key=lambda s: s.length, reverse=(val == "longest"))
        if len(order) >= 2 and abs(order[0].length - order[1].length) < AMBIG_LEN_MM:
            word = "긴" if val == "longest" else "짧은"
            return None, (f"가장 {word} 막대 후보 두 개의 길이 차이가 {abs(order[0].length - order[1].length):.1f}mm라 "
                          f"구분이 어려움 → 위치로 말해 주세요"), None
        return order[0], None, None

    if kind == "index":
        order = left_to_right(cands)
        if not 1 <= val <= len(order):
            return None, f"막대는 {len(order)}개뿐인데 {val}번째를 말함", None
        return order[val - 1], None, None

    return None, f"알 수 없는 대상: {target}", None


# ============================================================
# 계획 만들기
# ============================================================
def make_plan(intent, sticks, default_slot):
    if intent.action == "HOME":
        return PlanResult(Plan("HOME", summary="홈으로 이동"))

    cands = [s for s in sticks if s.has_robot_coords and s.in_pick_zone]
    if not cands:
        return PlanResult(error="집기 영역 안에서 막대를 찾지 못함 (스냅샷 확인)")

    if intent.action == "PICK":
        stick, err, note = _select(intent.target, cands)
        if err:
            return PlanResult(error=err)
        if intent.dest_slot is not None:
            slot = intent.dest_slot
        elif intent.dest_side is not None:
            slot = side_to_slot(intent.dest_side)
        else:
            slot = default_slot
        if slot not in config.PLACE_SLOTS:
            return PlanResult(error=f"없는 칸 번호: {slot}")
        step = Step(stick, slot)
        plan = Plan("PICK", [step], summary=f"막대 {step.label()}")
        if note:
            plan.notes.append(note)
        return PlanResult(plan)

    if intent.action == "SORT":
        slots = slots_left_to_right()
        if len(cands) > len(slots):
            return PlanResult(error=f"막대 {len(cands)}개 > 놓기 칸 {len(slots)}개 → 막대를 줄이거나 칸을 늘릴 것")
        order = sorted(cands, key=lambda s: s.length, reverse=(intent.order == "desc"))
        steps = [Step(s, slots[i]) for i, s in enumerate(order)]
        word = "긴 것부터" if intent.order == "desc" else "짧은 것부터"
        plan = Plan("SORT", steps, summary=f"길이순 정렬 ({word}): " + ", ".join(st.label() for st in steps))
        gaps = [abs(order[i].length - order[i + 1].length) for i in range(len(order) - 1)]
        if gaps and min(gaps) < AMBIG_LEN_MM:
            plan.notes.append(f"길이 차이가 {min(gaps):.1f}mm인 막대가 있어 순서가 틀릴 수 있음")
        return PlanResult(plan)

    return PlanResult(error=f"알 수 없는 동작: {intent.action}")


def rematch(stick, new_sticks, tol=REMATCH_TOL_MM, len_tol=REMATCH_LEN_MM):
    """앞 스냅샷의 stick과 같은 막대를 새 스냅샷에서 찾는다. 없으면 None.

    정렬 중 앞 막대를 집으면서 다른 막대를 건드렸을 수 있으므로, 매번 새 스냅샷 좌표로 바꿔서 집는다.
    """
    best, best_d = None, None
    for s in new_sticks:
        if not (s.has_robot_coords and s.in_pick_zone):
            continue
        d = float(np.hypot(s.x - stick.x, s.y - stick.y))
        if d <= tol and abs(s.length - stick.length) <= len_tol and (best_d is None or d < best_d):
            best, best_d = s, d
    return best
