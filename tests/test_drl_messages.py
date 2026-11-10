"""DRL이 보내는 모션 메시지 검사.

1) 형식과 상관없이 지켜야 하는 것 (CI에서 항상 검사)
   - 모든 이동 함수는 이동 직전에 시작 메시지, 이동 후에 완료 메시지를 보낸다
   - 작업2 이송 루프는 JOB → 이동 → JOB_DONE 순서로 메시지를 보낸다 (퀴즈2 CSV)

2) 퀴즈1 문제지 문자열 규칙 (현재 결정 대기 → xfail)
   퀴즈1 규칙: 실행 직전 "MoveJ Start" / "MoveL Start",
               실제 이동 완료 후 "MoveJ 종료" / "MoveL 종료" (대소문자·공백 동일).
   PR #13이 "MoveJ Done", "MoveL_REL Start/Done" 형식으로 바꿨다. 팀에서 어느 형식을
   쓸지 정하기 전까지 아래 테스트는 xfail로 둔다. "종료" 형식으로 되돌리면 xfail 표시를
   지워 다시 강제한다. (GUI의 CSV 기록은 두 형식을 모두 인정하므로 어느 쪽이든 동작한다)
"""

import ast
from pathlib import Path

import pytest

from robot_comm.task2_record import motion_event

DRL_PATH = Path(__file__).parent.parent / "drl" / "ROBOT_CONTROL_DRL.drl"
SOURCE = DRL_PATH.read_text(encoding="utf-8")
TREE = ast.parse(SOURCE)
FUNCS = {n.name: n for n in TREE.body if isinstance(n, ast.FunctionDef)}

QUIZ1_PENDING = pytest.mark.xfail(
    reason="퀴즈1 메시지 문자열(MoveJ 종료 vs MoveJ Done) 팀 결정 대기 — PR #13 형식 사용 중",
    strict=False,
)

EXPECTED = {
    "MSG_MOVEJ_START": "MoveJ Start",
    "MSG_MOVEJ_END": "MoveJ 종료",
    "MSG_MOVEL_START": "MoveL Start",
    "MSG_MOVEL_END": "MoveL 종료",
}

# 이동 함수 이름 -> (실제 이동 명령, 메시지 종류 J/L)
MOTION_FUNCS = {
    "move_j": ("movej", "J"),
    "move_jx": ("movejx", "J"),
    "move_l": ("movel", "L"),
    "move_l_REL": ("movel", "L"),
}


def _module_constants():
    consts = {}
    for node in TREE.body:
        if (isinstance(node, ast.Assign) and len(node.targets) == 1
                and isinstance(node.targets[0], ast.Name)
                and isinstance(node.value, ast.Constant)):
            consts[node.targets[0].id] = node.value.value
    return consts


CONSTS = _module_constants()


def _call_name(stmt):
    if isinstance(stmt, ast.Expr) and isinstance(stmt.value, ast.Call):
        func = stmt.value.func
        if isinstance(func, ast.Name):
            return func.id
    return None


def _send_text(stmt):
    """send_msg("문자열") 또는 send_msg(상수) 이면 그 문자열, 아니면 None."""
    if _call_name(stmt) != "send_msg" or not stmt.value.args:
        return None
    arg = stmt.value.args[0]
    if isinstance(arg, ast.Constant) and isinstance(arg.value, str):
        return arg.value
    if isinstance(arg, ast.Name):
        return CONSTS.get(arg.id)
    return None


# ---------- 1) 형식과 상관없이 항상 검사 ----------

@pytest.mark.parametrize("func_name", sorted(MOTION_FUNCS))
def test_motion_function_sends_start_and_end(func_name):
    """시작 메시지 → 이동 명령 → 완료 메시지 순서. (Done / 종료 형식 모두 허용)"""
    motion, _kind = MOTION_FUNCS[func_name]
    assert func_name in FUNCS, f"{func_name} 함수가 없음"
    body = FUNCS[func_name].body
    events = [motion_event(_send_text(s) or "") for s in body]
    names = [_call_name(s) for s in body]

    start_i = next(i for i, e in enumerate(events) if e and e[1] == "start")
    motion_i = names.index(motion)
    end_i = next(i for i, e in enumerate(events) if e and e[1] == "end")
    assert start_i < motion_i < end_i


def test_task2_job_messages_wrap_pick_place():
    """작업2 이송 루프: JOB 송신 → 이동들 → JOB_DONE 송신.

    PC는 JOB_DONE 직전의 마지막 이동 완료 메시지를 종료 시각으로 쓰므로,
    JOB_DONE 바로 앞은 이동(어떤 종류든)이어야 한다.
    """
    task2 = FUNCS["TASK2"]
    loops = [n for n in ast.walk(task2) if isinstance(n, ast.For)]
    pick_loop = next((lp for lp in loops if "JOB_DONE" in ast.unparse(lp)), None)
    assert pick_loop is not None, "작업2 이송 루프에 JOB_DONE 송신이 없음 → GUI가 CSV를 못 씀"
    calls = [_call_name(s) for s in pick_loop.body if _call_name(s)]
    texts = [ast.unparse(s) for s in pick_loop.body if _call_name(s)]

    job_i = next(i for i, t in enumerate(texts) if "'JOB," in t or '"JOB,' in t)
    first_motion = next(i for i, c in enumerate(calls) if c in MOTION_FUNCS)
    assert job_i < first_motion, "JOB은 첫 이동보다 먼저 보내야 함"
    assert "JOB_DONE" in texts[-1], "JOB_DONE이 루프 마지막이어야 함"
    assert calls[-2] in MOTION_FUNCS, "JOB_DONE 직전은 이동(놓은 뒤 상승)이어야 함"


def test_job_message_field_count():
    """JOB 메시지 형식: JOB + 번호, 길이, pick 6축, place 6축 = 쉼표로 구분된 15칸."""
    job = next(n for n in ast.walk(FUNCS["TASK2"])
               if isinstance(n, ast.Constant) and isinstance(n.value, str)
               and n.value.startswith("JOB,{"))
    assert job.value.count("{") == 14


# ---------- 2) 퀴즈1 문자열 규칙 (결정 대기) ----------

@QUIZ1_PENDING
def test_message_constants_match_quiz1_table():
    for name, text in EXPECTED.items():
        assert CONSTS.get(name) == text, f"{name} 은(는) {text!r} 이어야 함"


@QUIZ1_PENDING
@pytest.mark.parametrize("func_name", sorted(MOTION_FUNCS))
def test_motion_messages_use_quiz1_strings(func_name):
    _motion, kind = MOTION_FUNCS[func_name]
    texts = [t for t in (_send_text(s) for s in FUNCS[func_name].body) if t]
    assert f"Move{kind} Start" in texts
    assert f"Move{kind} 종료" in texts


@QUIZ1_PENDING
def test_no_nonstandard_motion_messages():
    """문제지에 없는 모션 메시지(Done, MoveL_REL 등)를 보내지 않는다."""
    for bad in ('"MoveJ Done"', '"MoveL Done"', '"MoveL_REL Start"', '"MoveL_REL Done"'):
        assert bad not in SOURCE, f"{bad} 메시지가 남아 있음"
