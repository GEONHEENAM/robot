"""
의도 파서 (5단계): 문장(텍스트 입력 또는 STT 결과) → Intent

규칙 기반 키워드 매칭이다. STT 결과는 띄어쓰기·조사가 들쭉날쭉하므로
공백과 문장부호를 지운 뒤 "부분 문자열"로 단어를 찾는다.

지원하는 명령
  집기   "왼쪽 막대 집어", "오른쪽 거 가운데에 놔", "긴 막대 1번 칸에 옮겨",
         "두 번째 막대 집어", "제일 짧은 막대 오른쪽으로"
  정렬   "길이 순서대로 정렬해" (짧은 것부터 P1), "긴 것부터 정렬해"
  홈     "홈으로 가", "처음 위치로"
  승인   "실행", "응", "네" / 취소 "취소", "아니"   (승인 대기 중에만 GUI가 확인)

대상(어떤 막대) vs 목적지(어느 칸) 구분 규칙
  1) "N번 칸", "P1"처럼 칸을 직접 말하면 그것이 목적지
  2) 길이("긴/짧은")나 순서("두 번째")로 대상을 말했으면, 위치 단어는 목적지
     단, "오른쪽에 있는 긴 막대"처럼 위치 뒤에 "있는/의"가 붙으면 목적지가 아님
  3) 위치 단어만 있으면 첫 번째 = 대상, 두 번째 = 목적지
  4) 목적지를 말하지 않으면 GUI에서 고른 "놓을 칸"을 쓴다 (planner에서 처리)

단독 실행: python intent.py  → 문장을 입력하면 해석 결과 출력
"""

import re
from dataclasses import dataclass

# ============================================================
# 단어 사전 (긴 단어를 앞에 둬야 짧은 단어가 먼저 잡히지 않는다)
# ============================================================
POSITION_WORDS = {
    "left": ["왼쪽", "왼편", "좌측"],
    "center": ["가운데", "중간", "중앙"],
    "right": ["오른쪽", "오른편", "우측"],
}
LONGEST_WORDS = ["제일긴", "가장긴", "제일길", "가장길", "긴거", "긴것", "긴막대", "길쭉", "긴"]
SHORTEST_WORDS = [
    "제일짧은",
    "가장짧은",
    "짧은거",
    "짧은것",
    "짧은막대",
    "짧은",
    "제일작은",
    "작은",
]
ORDINAL_PATTERNS = [  # (정규식, 번호) — "이번"은 "이번에"와 헷갈려서 넣지 않음
    (r"첫번째|첫째|1번째|1번", 1),
    (r"두번째|둘째|2번째|2번", 2),
    (r"세번째|셋째|3번째|3번", 3),
    (r"네번째|넷째|4번째|4번", 4),
]
SLOT_PATTERNS = [  # 칸을 직접 말한 경우
    (r"(첫번째|첫|1번|일번|1)칸|(첫번째|첫|1번|일번|1)자리|p1|피일|피원", 1),
    (r"(두번째|두|2번|이번|2)칸|(두번째|두|2번|이번|2)자리|p2|피이|피투", 2),
    (r"(세번째|세|3번|삼번|3)칸|(세번째|세|3번|삼번|3)자리|p3|피삼|피쓰리", 3),
]
SORT_WORDS = ["정렬", "순서대로", "순서로", "줄세워", "배열", "나열"]
SORT_DESC_WORDS = ["긴것부터", "긴거부터", "긴순", "긴막대부터", "내림차순", "큰것부터", "긴것먼저"]
SORT_ASC_WORDS = [
    "짧은것부터",
    "짧은거부터",
    "짧은순",
    "짧은막대부터",
    "오름차순",
    "작은것부터",
    "짧은것먼저",
]
HOME_WORDS = ["홈", "원위치", "처음위치", "제자리", "초기위치", "기본위치"]
# 승인어는 짧은 단어("네", "응")가 다른 문장 속에 섞여 오인되지 않도록 "문장 전체"로 비교한다
CONFIRM_EXACT = {
    "실행",
    "실행해",
    "실행해줘",
    "진행",
    "진행해",
    "좋아",
    "오케이",
    "ok",
    "응",
    "네",
    "예",
    "그래",
    "해",
    "해줘",
}
CONFIRM_PREFIX = ("실행", "진행")
CANCEL_WORDS = ["취소", "아니", "그만", "멈춰", "정지", "스톱", "하지마"]

POSITION_KO = {"left": "왼쪽", "center": "가운데", "right": "오른쪽"}


def normalize(text):
    """소문자 + 공백·문장부호 제거. STT 결과의 띄어쓰기 차이를 없앤다."""
    return re.sub(r"[\s\.,!?~'\"]", "", (text or "").lower())


def _contains(t, words):
    return any(w in t for w in words)


@dataclass
class Intent:
    action: str  # "PICK" | "SORT" | "HOME"
    target: tuple | None = (
        None  # ("position", "left"|"center"|"right") | ("length", "longest"|"shortest")
    )
    # | ("index", n) | ("only", None) = 막대가 하나뿐일 때만 가능
    dest_slot: int | None = None  # 칸 번호를 직접 말한 경우
    dest_side: str | None = None  # "왼쪽에 놔"처럼 위치로 말한 경우 → planner가 칸 번호로 바꿈
    order: str = "asc"  # 정렬: "asc" = 짧은 것부터, "desc" = 긴 것부터
    text: str = ""

    def describe(self):
        if self.action == "HOME":
            return "홈으로 이동"
        if self.action == "SORT":
            return "길이순 정렬 (" + ("긴 것부터" if self.order == "desc" else "짧은 것부터") + ")"
        kind, val = self.target
        tgt = {
            "position": lambda v: f"{POSITION_KO[v]} 막대",
            "length": lambda v: "가장 긴 막대" if v == "longest" else "가장 짧은 막대",
            "index": lambda v: f"{v}번째 막대 (작업자 기준 왼쪽부터)",
            "only": lambda v: "하나뿐인 막대",
        }[kind](val)
        if self.dest_slot:
            dst = f"P{self.dest_slot}"
        elif self.dest_side:
            dst = f"{POSITION_KO[self.dest_side]} 칸"
        else:
            dst = "선택된 놓을 칸"
        return f"{tgt} → {dst}"


@dataclass
class ParseResult:
    intent: Intent | None = None
    error: str | None = None


def is_confirm(text):
    t = normalize(text)
    if not t or _contains(t, CANCEL_WORDS):
        return False
    return t in CONFIRM_EXACT or t.startswith(CONFIRM_PREFIX)


def is_cancel(text):
    return _contains(normalize(text), CANCEL_WORDS)


# ============================================================
# 내부 탐색
# ============================================================
def _find_positions(t):
    """[(시작, 끝, side)]를 문장 순서대로."""
    found = []
    for side, words in POSITION_WORDS.items():
        for w in words:
            for m in re.finditer(re.escape(w), t):
                found.append((m.start(), m.end(), side))
    found.sort()
    out, last_end = [], -1  # 겹치는 것 제거
    for s, e, side in found:
        if s >= last_end:
            out.append((s, e, side))
            last_end = e
    return out


def _find_first(t, words):
    """words 중 가장 먼저 나오는 위치 (없으면 None)."""
    hits = [t.find(w) for w in words if w in t]
    return min(hits) if hits else None


def _is_qualifier(t, end):
    """'오른쪽에 있는 ~', '오른쪽의 ~'처럼 위치가 대상을 꾸미는 경우 True."""
    rest = t[end:]
    return rest.startswith(("에있", "쪽에있", "의", "에놓인", "에있는"))


def _has_dest_particle(t, end):
    rest = t[end:]
    return rest.startswith(("에", "으로", "로", "칸", "자리")) and not _is_qualifier(t, end)


# ============================================================
# 파서
# ============================================================
def parse(text):
    t = normalize(text)
    if not t:
        return ParseResult(error="빈 문장")

    # ---- 정렬 ----
    if _contains(t, SORT_WORDS):
        order = "desc" if _contains(t, SORT_DESC_WORDS) else "asc"
        return ParseResult(Intent("SORT", order=order, text=text))

    # ---- 홈 ----
    if _contains(t, HOME_WORDS):
        return ParseResult(Intent("HOME", text=text))

    # ---- 집기 ----
    # 1) 칸 직접 지정 → 그 부분은 지우고 나머지를 해석 (번호가 대상 순서로 오해되지 않게)
    dest_slot = None
    rest = t
    for pat, n in SLOT_PATTERNS:
        m = re.search(pat, rest)
        if m:
            dest_slot = n
            rest = rest[: m.start()] + "#" * (m.end() - m.start()) + rest[m.end() :]
            break

    positions = _find_positions(rest)
    len_pos = _find_first(rest, LONGEST_WORDS)
    short_pos = _find_first(rest, SHORTEST_WORDS)
    ordinal, ord_pos = None, None
    for pat, n in ORDINAL_PATTERNS:
        m = re.search(pat, rest)
        if m and (ord_pos is None or m.start() < ord_pos):
            ordinal, ord_pos = n, m.start()

    target = None
    if len_pos is not None or short_pos is not None:
        # 둘 다 나오면 먼저 나온 쪽 ("짧은"이 "긴"보다 먼저 검사되지 않도록 위치로 판단)
        if short_pos is not None and (len_pos is None or short_pos <= len_pos):
            target = ("length", "shortest")
        else:
            target = ("length", "longest")
    elif ordinal is not None:
        target = ("index", ordinal)

    dest_side = None
    if target is not None:
        # 길이/순서로 대상을 말함 → 위치 단어는 목적지 후보 (꾸밈말이면 제외)
        cands = [(s, e, side) for s, e, side in positions if not _is_qualifier(rest, e)]
        if cands:
            dest_side = cands[-1][2]
    elif positions:
        s0, e0, side0 = positions[0]
        if len(positions) >= 2:
            target = ("position", side0)
            dest_side = positions[-1][2]
        elif _has_dest_particle(rest, e0):
            # "막대 왼쪽에 놔", "오른쪽으로 옮겨"처럼 목적지만 말한 경우
            # → 대상은 막대가 하나뿐일 때만 정할 수 있음 (planner가 확인)
            target = ("only", None)
            dest_side = side0
        else:
            target = ("position", side0)

    if target is None:
        if _contains(t, ["집어", "잡아", "옮겨", "놔", "놓아", "가져", "들어"]):
            if dest_slot is not None:
                return ParseResult(
                    Intent("PICK", target=("only", None), dest_slot=dest_slot, text=text)
                )
            return ParseResult(
                error="어떤 막대인지 모르겠음 (예: 왼쪽 막대, 긴 막대, 두 번째 막대)"
            )
        return ParseResult(error="명령을 이해하지 못함")

    return ParseResult(
        Intent("PICK", target=target, dest_slot=dest_slot, dest_side=dest_side, text=text)
    )


# ============================================================
# STT 문법 제한용 문장 목록 (stt.py가 사용)
#   Vosk는 이 목록에 있는 단어 조합 위주로 인식한다. 모델 사전에 없는 단어는 무시된다.
# ============================================================
def grammar_phrases():
    pos = ["왼쪽", "가운데", "오른쪽"]
    targets = (
        [f"{p} 막대" for p in pos]
        + [f"{p} 거" for p in pos]
        + [
            "긴 막대",
            "짧은 막대",
            "제일 긴 막대",
            "제일 짧은 막대",
            "가장 긴 막대",
            "가장 짧은 막대",
            "첫 번째 막대",
            "두 번째 막대",
            "세 번째 막대",
        ]
    )
    dests = (
        [f"{p}에" for p in pos]
        + [f"{p}으로" for p in pos]
        + ["첫 번째 칸에", "두 번째 칸에", "세 번째 칸에"]
    )
    verbs = ["집어", "옮겨", "놔", "놓아", "집어 줘", "옮겨 줘", "놔 줘"]
    phrases = []
    for tg in targets:
        phrases.append(f"{tg} 집어")
        for d in dests:
            phrases += [f"{tg} {d} 놔", f"{tg} {d} 옮겨"]
    phrases += [
        "길이 순서대로 정렬해",
        "짧은 것부터 정렬해",
        "긴 것부터 정렬해",
        "정렬해",
        "순서대로 놔",
        "홈으로 가",
        "처음 위치로",
        "원위치",
        "실행",
        "취소",
        "응",
        "네",
        "아니",
    ]
    phrases += pos + verbs + ["막대", "칸", "거", "제일", "가장", "긴", "짧은"]
    return sorted(set(phrases))


if __name__ == "__main__":
    print("문장을 입력하세요 (빈 줄 = 종료)")
    while True:
        try:
            s = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if not s:
            break
        r = parse(s)
        if r.error:
            print("  오류:", r.error)
        else:
            print("  의도:", r.intent.describe(), "|", r.intent)
        print("  승인어:", is_confirm(s), " 취소어:", is_cancel(s))
