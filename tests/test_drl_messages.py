"""DRL이 보내는 모션 메시지가 퀴즈1 문제지 표와 같은지 검사한다.

퀴즈1 규칙: 실행 직전 "MoveJ Start" / "MoveL Start",
            실제 이동 완료 후 "MoveJ 종료" / "MoveL 종료" (대소문자·공백 동일).
퀴즈4는 이전 퀴즈 기능을 모두 유지해야 하므로 이 규칙도 계속 지켜야 한다.
"""

import ast
from pathlib import Path

import pytest

DRL_PATH = Path(__file__).parent.parent / "drl" / "ROBOT_CONTROL_DRL.drl"
SOURCE = DRL_PATH.read_text(encoding="utf-8")
TREE = ast.parse(SOURCE)

EXPECTED = {
    "MSG_MOVEJ_START": "MoveJ Start",
    "MSG_MOVEJ_END": "MoveJ 종료",
    "MSG_MOVEL_START": "MoveL Start",
    "MSG_MOVEL_END": "MoveL 종료",
}

# 모션 함수 이름 -> (실행 명령, 시작 메시지 상수, 종료 메시지 상수)
MOTION_FUNCS = {
    "move_j": ("movej", "MSG_MOVEJ_START", "MSG_MOVEJ_END"),
    "move_jx": ("movejx", "MSG_MOVEJ_START", "MSG_MOVEJ_END"),
    "move_l": ("movel", "MSG_MOVEL_START", "MSG_MOVEL_END"),
    "move_l_REL": ("movel", "MSG_MOVEL_START", "MSG_MOVEL_END"),
}


def _module_constants():
    consts = {}
    for node in TREE.body:
        if (isinstance(node, ast.Assign) and len(node.targets) == 1
                and isinstance(node.targets[0], ast.Name)
                and isinstance(node.value, ast.Constant)):
            consts[node.targets[0].id] = node.value.value
    return consts


def _call_name(stmt):
    if isinstance(stmt, ast.Expr) and isinstance(stmt.value, ast.Call):
        func = stmt.value.func
        if isinstance(func, ast.Name):
            return func.id
    return None


def _send_arg(stmt):
    """send_msg(NAME) 이면 NAME, 아니면 None."""
    if _call_name(stmt) != "send_msg" or not stmt.value.args:
        return None
    arg = stmt.value.args[0]
    return arg.id if isinstance(arg, ast.Name) else None


def test_message_constants_match_quiz1_table():
    consts = _module_constants()
    for name, text in EXPECTED.items():
        assert consts.get(name) == text, f"{name} 은(는) {text!r} 이어야 함"


@pytest.mark.parametrize("func_name", sorted(MOTION_FUNCS))
def test_motion_function_sends_start_and_end(func_name):
    """시작 메시지 → 모션 명령 → 종료 메시지 순서인지 확인한다."""
    motion, start_const, end_const = MOTION_FUNCS[func_name]
    funcs = {n.name: n for n in TREE.body if isinstance(n, ast.FunctionDef)}
    assert func_name in funcs, f"{func_name} 함수가 없음"
    body = funcs[func_name].body

    calls = [(_call_name(s), _send_arg(s)) for s in body]
    start_i = calls.index(("send_msg", start_const))
    motion_i = next(i for i, (name, _) in enumerate(calls) if name == motion)
    end_i = calls.index(("send_msg", end_const))
    assert start_i < motion_i < end_i


def test_no_nonstandard_motion_messages():
    """문제지에 없는 모션 메시지(Done, MoveL_REL 등)를 보내지 않는다."""
    for bad in ('"MoveJ Done"', '"MoveL Done"', '"MoveL_REL Start"', '"MoveL_REL Done"'):
        assert bad not in SOURCE, f"{bad} 메시지가 남아 있음"
