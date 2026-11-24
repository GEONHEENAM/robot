"""
로봇 DRL 소켓 클라이언트 (GUI가 아닌 스크립트용: calibrate.py, robot_test_client.py)

퀴즈3 구조 그대로: DRL이 서버(포트 9000), PC가 클라이언트.
"""

import socket
import time

import config


class RobotError(RuntimeError):
    """로봇이 ERROR,... 를 보냈을 때"""


class RobotLink:
    def __init__(self, ip=config.ROBOT_IP, port=config.ROBOT_PORT, timeout=5.0, verbose=True):
        self.verbose = verbose
        self.buf = ""
        self.sock = socket.create_connection((ip, port), timeout=timeout)
        self._log(f"[연결] {ip}:{port}")
        self.wait_for(["READY"], timeout=10.0)

    def _log(self, msg):
        if self.verbose:
            print(msg)

    def send(self, line):
        self.sock.sendall((line + "\n").encode())
        self._log(f"→ {line}")

    def read_line(self, timeout):
        """한 줄을 읽는다. timeout 초 안에 못 받으면 TimeoutError."""
        deadline = time.time() + timeout
        while "\n" not in self.buf:
            remain = deadline - time.time()
            if remain <= 0:
                raise TimeoutError("로봇 응답 시간 초과")
            self.sock.settimeout(remain)
            try:
                data = self.sock.recv(4096)
            except socket.timeout as e:
                raise TimeoutError("로봇 응답 시간 초과") from e
            if not data:
                raise ConnectionError("로봇 연결 끊김")
            self.buf += data.decode(errors="ignore")
        line, self.buf = self.buf.split("\n", 1)
        return line.strip()

    def wait_for(self, expected, timeout=60.0, quiet_prefixes=("POS",)):
        """expected 중 하나로 시작하는 줄이 올 때까지 기다린다. ERROR가 오면 RobotError."""
        deadline = time.time() + timeout
        while True:
            line = self.read_line(max(0.1, deadline - time.time()))
            if not line:
                continue
            if line.upper().startswith("ERROR"):
                raise RobotError(line)
            if any(line.startswith(e) for e in expected):
                self._log(f"← {line}")
                return line
            if not line.startswith(quiet_prefixes):
                self._log(f"   {line}")

    def command(self, line, expected, timeout=60.0):
        self.send(line)
        return self.wait_for(expected, timeout)

    # ---- 자주 쓰는 명령 ----
    def home(self):
        return self.command("HOME", ["AT_HOME"], timeout=60.0)

    def move_xy(self, x, y, z):
        return self.command(f"MOVEXY,{x:.1f},{y:.1f},{z:.1f}", ["DONE,9"], timeout=60.0)

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass
