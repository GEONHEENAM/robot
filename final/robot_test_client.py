"""
DRL 명령 직접 테스트용 터미널 클라이언트 (3단계)

GUI 없이 로봇에 한 줄씩 명령을 보내고 응답을 본다.
DRL을 고친 직후 회전 방향(YAW_SIGN)·오프셋(YAW_OFFSET) 확인에 쓴다.

실행
  python robot_test_client.py                 # config.ROBOT_IP:9000
  python robot_test_client.py --ip 127.0.0.1  # 가짜 로봇

입력 예
  HOME
  GETPOS
  MOVEXY,0,490,30          ← 집기 영역 가운데 30 mm 위로 (그리퍼 중심이 어디 오는지 확인)
  PICK,0,490,0,2           ← 각도 0°로 놓인 막대 집기 → 2번 칸
  PICK,0,490,30,2          ← 각도 +30° 막대: 그리퍼가 막대를 가로질러 잡는지 확인
  q                        ← 종료
"""

import argparse
import socket
import threading

import config


def reader(sock):
    buf = ""
    while True:
        try:
            data = sock.recv(4096)
        except OSError:
            break
        if not data:
            print("\n[연결 끊김]")
            break
        buf += data.decode(errors="ignore")
        while "\n" in buf:
            line, buf = buf.split("\n", 1)
            if line.strip():
                print(f"  ← {line.strip()}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip", default=config.ROBOT_IP)
    ap.add_argument("--port", type=int, default=config.ROBOT_PORT)
    a = ap.parse_args()

    sock = socket.create_connection((a.ip, a.port), timeout=5)
    sock.settimeout(None)
    print(f"[연결] {a.ip}:{a.port}  (명령 입력, q = 종료)")
    threading.Thread(target=reader, args=(sock,), daemon=True).start()
    try:
        while True:
            line = input().strip()
            if not line:
                continue
            if line.lower() in ("q", "quit", "exit"):
                break
            sock.sendall((line + "\n").encode())
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        sock.close()


if __name__ == "__main__":
    main()
