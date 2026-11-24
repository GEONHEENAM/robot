"""
가짜 로봇 서버 (로봇 없이 GUI 테스트용)

퀴즈3 main.drl처럼 TCP 서버(포트 9000)를 열고 같은 형식으로 응답한다.
실제 DRL처럼 명령을 실행하는 동안에는 다음 명령을 읽지 않는다.

실행 (kw2026 activate 후)
  python fake_robot.py                   # 0.0.0.0:9000
  python fake_robot.py --speed 3         # 동작 시간을 3배 빠르게
  python fake_robot.py --fail-rate 0.2   # 20% 확률로 PICK 실패(접촉 없음) 흉내
GUI에서 로봇 IP를 127.0.0.1로 두고 연결한다.
종료: Ctrl + C
"""

import argparse
import random
import socket
import time

import config

HOME_XYZ = (0.0, 450.0, 250.0)  # 보기용 임의값
HOME_JOINT = (-90.0, 0.0, -90.0, 0.0, -90.0, 0.0)  # 퀴즈3 HOME = posj(-90, 0, -90, 0, -90, 0)
MIN_DEPTH = 7.5
PLACE_MARGIN = 2.0


class FakeRobot:
    def __init__(self, conn, speed, fail_rate):
        self.conn = conn
        self.speed = speed
        self.fail_rate = fail_rate
        self.xyz = list(HOME_XYZ)
        self.joint = list(HOME_JOINT)

    # ---------------- 송신 ----------------
    def send(self, msg):
        self.conn.sendall((msg + "\n").encode())
        print(f"   ← {msg}")

    def send_pos(self):
        self.send("POS," + ",".join(f"{v:.3f}" for v in (*self.xyz, *self.joint)))

    def wait(self, sec):
        time.sleep(sec / self.speed)

    def move(self, kind, x, y, z, sec=0.6):
        self.send(f"{kind} Start")
        self.wait(sec)
        self.xyz = [x, y, z]
        self.joint[0] = -90.0 + x / 10.0  # 관절값은 화면 확인용으로 대충 바꿈
        self.joint[5] = self.joint[0]
        self.send(f"{kind} Done")
        self.send_pos()

    def grip(self, on):
        self.send("Gripper ON" if on else "Gripper OFF")
        self.wait(0.5)

    # ---------------- 명령 ----------------
    def home(self):
        self.move("MoveJ", *HOME_XYZ, sec=1.0)
        self.joint = list(HOME_JOINT)
        self.send_pos()
        self.send("AT_HOME")

    def pick(self, args):
        try:
            x, y, ang = (float(v) for v in args[:3])
            slot = int(args[3])
        except (ValueError, IndexError):
            self.send("ERROR,PICK,bad arguments")
            return
        (x0, x1), (y0, y1) = config.PICK_X_RANGE, config.PICK_Y_RANGE
        if not (min(x0, x1) <= x <= max(x0, x1) and min(y0, y1) <= y <= max(y0, y1)):
            self.send("ERROR,PICK,out of pick zone")
            return
        if slot not in config.PLACE_SLOTS:
            self.send("ERROR,PICK,bad slot")
            return

        self.send(f"PICK Accepted,{x:.1f},{y:.1f},{ang:.1f},{slot}")
        self.grip(False)
        self.move("MoveJ", x, y, 100.0)
        self.send("Compliance start,0")
        self.wait(1.0)
        if random.random() < self.fail_rate:
            self.send("Compliance finish,0")
            self.move("MoveL", x, y, 100.0, 0.3)
            self.send("ERROR,0,no contact")
            self.home()  # 실제 DRL(drl/main.drl)도 실패 후 홈 복귀
            return
        h = random.uniform(8.0, 20.0)
        self.xyz[2] = h
        self.send_pos()
        self.send("Compliance finish,0")
        self.move("MoveL", x, y, 100.0, 0.3)

        grip_z = max(h, MIN_DEPTH)
        self.move("MoveL", x, y, grip_z, 0.4)
        self.grip(True)
        self.move("MoveL", x, y, 100.0, 0.4)
        sx, sy = config.PLACE_SLOTS[slot]
        self.move("MoveJ", sx, sy, 100.0)
        self.move("MoveL", sx, sy, grip_z + PLACE_MARGIN, 0.4)
        self.grip(False)
        self.move("MoveL", sx, sy, 100.0, 0.4)
        self.home()  # 홈 복귀 → AT_HOME → DONE,3 (drl/main.drl과 같은 순서)
        self.send("DONE,3")

    def move_xy(self, args):
        try:
            x, y, z = (float(v) for v in args[:3])
        except (ValueError, IndexError):
            self.send("ERROR,MOVEXY,bad arguments")
            return
        (x0, x1), (y0, y1) = config.PICK_X_RANGE, config.PICK_Y_RANGE
        m = 50.0
        if z < 5.0 or not (
            min(x0, x1) - m <= x <= max(x0, x1) + m and min(y0, y1) - m <= y <= max(y0, y1) + m
        ):
            self.send("ERROR,MOVEXY,out of range")
            return
        self.move("MoveJ", x, y, 100.0)
        self.move("MoveL", x, y, z, 0.4)
        self.send("DONE,9")

    def task1(self):
        for x, y in ((0, 500), (100, 450), (-100, 450), (0, 700), (100, 650), (-100, 650)):
            self.move("MoveJ", x, y, 100.0, 0.4)
        self.send("DONE,1")

    def task2(self):
        results = []
        for i, (x, y) in enumerate(((200, 500), (0, 500), (-200, 500)), 1):
            self.move("MoveJ", x, y, 100.0, 0.4)
            self.send(f"Compliance start,{i}")
            self.wait(0.8)
            h = random.uniform(20.0, 60.0)
            self.send(f"Compliance finish,{i}")
            self.send(f"DATA,{i},{x:.2f},{y:.2f},100.00,{h:.2f}")
            results.append((i, h))
        order = sorted(results, key=lambda r: r[1], reverse=True)
        self.send("ORDER," + ",".join(str(r[0]) for r in order))
        for slot in range(3):
            sx, sy = config.PLACE_SLOTS[slot + 1]
            self.move("MoveJ", sx, sy, 100.0, 0.4)
        self.send("DONE,2")

    def handle(self, line):
        """명령 1줄 처리. False를 돌려주면 연결을 끊는다."""
        cmd, _, body = line.partition(",")
        cmd = cmd.strip().upper()
        args = [a.strip() for a in body.split(",")] if body else []
        if cmd == "TASK" and args == ["1"]:
            self.send("TASK 1 Accepted")
            self.task1()
        elif cmd == "TASK" and args == ["2"]:
            self.send("TASK 2 Accepted")
            self.task2()
        elif cmd == "HOME":
            self.home()
        elif cmd == "PICK":
            self.pick(args)
        elif cmd == "MOVEXY":
            self.move_xy(args)
        elif cmd == "GETPOS":
            self.send_pos()
        elif cmd == "END":
            self.send("END Received")
            return False
        else:
            self.send("UNKNOWN " + line)
        return True


def serve(host, port, speed, fail_rate):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(1)
    srv.settimeout(1.0)  # Ctrl+C가 먹히도록 accept를 1초마다 깨움
    print(f"가짜 로봇 서버 대기: {host}:{port}  (speed x{speed}, fail_rate {fail_rate})")
    try:
        while True:
            try:
                conn, addr = srv.accept()
            except socket.timeout:
                continue
            print(f"[연결] {addr}")
            conn.settimeout(None)
            robot = FakeRobot(conn, speed, fail_rate)
            buf = ""
            try:
                robot.send("READY")
                robot.send_pos()
                alive = True
                while alive:
                    data = conn.recv(4096)
                    if not data:
                        break
                    buf += data.decode(errors="ignore")
                    while "\n" in buf and alive:
                        line, buf = buf.split("\n", 1)
                        line = line.strip()
                        if line:
                            print(f"→ {line}")
                            alive = robot.handle(line)
            except OSError as e:
                print(f"[연결 오류] {e}")
            finally:
                conn.close()
                print("[연결 종료] 다음 접속 대기")
    except KeyboardInterrupt:
        print("\n종료")
    finally:
        srv.close()


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="가짜 로봇 서버 (퀴즈3 DRL 프로토콜)")
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=config.ROBOT_PORT)
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--fail-rate", type=float, default=0.0)
    a = ap.parse_args()
    serve(a.host, a.port, a.speed, a.fail_rate)
