# check_server.py
# quiz3_server.py 확인용 클라이언트
#   python check_server.py            레지스터 20개를 표로 1회 출력
#   python check_server.py --watch    1초마다 갱신 (Ctrl + C 종료)
#   python check_server.py --clear    서버를 끄지 않고 전부 0으로 초기화 (시연 전 정리)
# 서버 주소/포트가 다르면: --host 192.168.x.x --port 5020

import argparse
import logging
import os
import time

from pymodbus.client import ModbusTcpClient
from register_map import DEVICE_ID, MODBUS_PORT, REGISTER_COUNT, is_measurement, label, raw_to_mm

logging.getLogger("pymodbus").setLevel(logging.CRITICAL)  # 연결 실패 시 16진수 덤프 방지


def show(client):
    rr = client.read_holding_registers(0, count=REGISTER_COUNT, device_id=DEVICE_ID)
    if rr.isError():
        print("읽기 실패:", rr)
        return False
    print(f"{'주소':>4} {'화면':>4}  {'항목':<14} {'Raw':>6}  값")
    print("-" * 44)
    for addr, raw in enumerate(rr.registers):
        value = f"{raw_to_mm(raw):.1f} mm" if is_measurement(addr) and raw else str(raw)
        print(f"{addr:>4} {addr + 1:>4}  {label(addr):<14} {raw:>6}  {value}")
    return True


def clear(client):
    answer = input("레지스터 20개를 모두 0으로 바꿉니다. 계속할까요? (y/N): ")
    if answer.strip().lower() != "y":
        print("취소")
        return
    wr = client.write_registers(0, [0] * REGISTER_COUNT, device_id=DEVICE_ID)
    if wr.isError():
        print("초기화 실패:", wr)
        return
    rr = client.read_holding_registers(0, count=REGISTER_COUNT, device_id=DEVICE_ID)
    ok = (not rr.isError()) and not any(rr.registers)
    print("초기화 완료 (재조회 확인)" if ok else "초기화 후 재조회 불일치")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=MODBUS_PORT)
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--watch", action="store_true")
    group.add_argument("--clear", action="store_true")
    args = parser.parse_args()

    client = ModbusTcpClient(args.host, port=args.port, timeout=1)
    if not client.connect():
        print(f"연결 실패: {args.host}:{args.port} (quiz3_server.py 실행 중인지 확인)")
        return

    try:
        if args.clear:
            clear(client)
        elif args.watch:
            while True:
                os.system("cls" if os.name == "nt" else "clear")
                print(time.strftime("%H:%M:%S"), f"{args.host}:{args.port}\n")
                if not show(client):
                    break
                time.sleep(1)
        else:
            show(client)
    except KeyboardInterrupt:
        pass
    finally:
        client.close()


if __name__ == "__main__":
    main()
