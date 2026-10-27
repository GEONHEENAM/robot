# quiz3_server.py
# 퀴즈3 Modbus TCP 서버 (Holding Register 1~20번 = 통신 주소 0~19)
#
# 역할
#   - Holding Register 20개를 제공한다. 값은 팀 UI(ui/quiz3_gui.py)가 쓰고 읽는다.
#   - 쓰기 요청이 들어올 때마다 파일(quiz3_registers.json)에 먼저 저장한다.
#     -> 서버를 껐다 켜도 작업 횟수/측정값이 그대로 복원된다.
#   - 어떤 칸이 어떻게 바뀌었는지 콘솔에 기록한다 (채점 화면/디버깅용).
#
# 레지스터 맵: register_map.py (UI의 레지스터 배치와 같아야 함)
#
# 실행 (kw2026 activate 후)
#   python quiz3_server.py               # 저장 파일이 있으면 복원해서 시작 (포트 502)
#   python quiz3_server.py --reset       # 전부 0으로 초기화하고 시작 (시연 전)
#   python quiz3_server.py --port 5020   # 포트 변경 (UI의 Modbus Port와 같아야 함)
# 종료: Ctrl + C

import argparse
import asyncio
import json
import logging
import os
import time
from pathlib import Path

from pymodbus.constants import ExcCodes
from pymodbus.server import StartAsyncTcpServer
from pymodbus.simulator import DataType, SimData, SimDevice
from register_map import DEVICE_ID, MODBUS_PORT, REGISTER_COUNT, describe, label

# ============================================================
# 설정
# ============================================================
HOST = "0.0.0.0"       # 이 PC의 모든 IP에서 접속 허용 (127.0.0.1 포함)
PORT = MODBUS_PORT      # 다른 PC에서 UI가 접속하면 방화벽에서 허용 필요
STATE_FILE = Path(__file__).with_name("quiz3_registers.json")

WRITE_FUNCTION_CODES = {6, 16, 22, 23}   # 단일/다중 쓰기, 마스크 쓰기, 읽기+쓰기


# ============================================================
# 저장 / 복원
# ============================================================
def load_state(path):
    """저장 파일에서 20개 값을 읽는다. 없거나 깨졌으면 None."""
    if not path.exists():
        return None
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        values = data["registers"]
        if len(values) != REGISTER_COUNT or not all(
            isinstance(v, int) and 0 <= v <= 0xFFFF for v in values
        ):
            raise ValueError("레지스터 개수/범위가 맞지 않음")
        return values
    except (OSError, ValueError, KeyError, TypeError) as exc:
        print(f"[경고] 저장 파일을 읽지 못함 ({exc}) -> 0으로 시작, 기존 파일은 .bad로 보관")
        try:
            path.replace(path.with_suffix(".bad"))
        except OSError:
            pass
        return None


def save_state(path, values):
    """임시 파일에 쓴 뒤 교체한다. 저장 도중 꺼져도 기존 파일이 깨지지 않는다."""
    tmp = path.with_suffix(".tmp")
    payload = {
        "saved_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "registers": list(values),
    }
    with tmp.open("w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=1)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)


# ============================================================
# 쓰기 감시: 메모리에 반영되기 "직전"에 호출됨 (pymodbus SimDevice action)
# ============================================================
def make_action(path):
    async def on_access(function_code, start_address, address, count,
                        registers, set_values):
        if function_code not in WRITE_FUNCTION_CODES or not set_values:
            return None   # 읽기는 그대로 통과

        offset = address - start_address
        if offset < 0 or offset + len(set_values) > len(registers):
            return None   # 범위 밖은 pymodbus가 ILLEGAL_ADDRESS로 응답함

        # pymodbus 내부 목록은 끝에 사용 불가 칸이 하나 더 있어서 20개만 잘라 저장한다
        after = list(registers[:REGISTER_COUNT])
        after[offset:offset + len(set_values)] = set_values

        # 파일 저장에 실패하면 쓰기를 거부한다 -> UI가 오류로 인식하고 재시도
        try:
            save_state(path, after)
        except OSError as exc:
            print(f"[오류] 저장 실패, 쓰기 거부: {exc}")
            return ExcCodes.DEVICE_FAILURE

        stamp = time.strftime("%H:%M:%S")
        for i, new in enumerate(set_values):
            addr = address + i
            old = registers[offset + i]
            if old != new:
                print(f"[{stamp}] 주소 {addr:2d} (화면 {addr + 1:2d}번) {label(addr)}: "
                      f"{describe(addr, old)} -> {describe(addr, new)}")
        return None

    return on_access


def to_init_value(v):
    # SimData 초기값은 부호 있는 16비트 범위로 넣어야 함 (65535 -> -1, 저장은 0~65535 그대로)
    return v - 65536 if v >= 32768 else v


def build_device(values, path):
    block = SimData(
        address=0,
        values=[to_init_value(v) for v in values],
        datatype=DataType.REGISTERS,
    )
    return SimDevice(id=DEVICE_ID, simdata=[block], action=make_action(path))


def print_table(values):
    print("  주소 화면  항목              값")
    for addr, v in enumerate(values):
        if v != 0:
            print(f"  {addr:>4} {addr + 1:>4}  {label(addr):<14} {describe(addr, v)}")
    if not any(values):
        print("  (모두 0)")


# ============================================================
# 메인
# ============================================================
def main():
    parser = argparse.ArgumentParser(description="퀴즈3 Modbus TCP 서버")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--state-file", type=Path, default=STATE_FILE)
    parser.add_argument("--reset", action="store_true", help="전부 0으로 초기화하고 시작")
    args = parser.parse_args()

    # pymodbus 내부 로그는 연결 끊김마다 16진수 덤프를 찍어서 끔
    logging.basicConfig(level=logging.WARNING)
    logging.getLogger("pymodbus").setLevel(logging.CRITICAL)

    if args.reset:
        values = [0] * REGISTER_COUNT
        save_state(args.state_file, values)
        print("[초기화] 모든 레지스터 0")
    else:
        values = load_state(args.state_file) or [0] * REGISTER_COUNT

    print(f"Modbus 서버 시작: {args.host}:{args.port}, device_id={DEVICE_ID}, "
          f"Holding Register 주소 0~{REGISTER_COUNT - 1} (화면 1~{REGISTER_COUNT}번)")
    print(f"저장 파일: {args.state_file.resolve()}")
    print("시작 값:")
    print_table(values)
    print("종료: Ctrl + C\n")

    try:
        asyncio.run(StartAsyncTcpServer(
            context=[build_device(values, args.state_file)],
            address=(args.host, args.port),
        ))
    except KeyboardInterrupt:
        pass
    except OSError as e:
        print(f"서버 시작 실패: {e}\n -> 포트 {args.port}를 다른 프로그램이 쓰고 있는지 확인")
    finally:
        print("서버 종료")


if __name__ == "__main__":
    main()
