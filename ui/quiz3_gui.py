"""
협동로봇 작업 UI (PyQt6)
 - Modbus TCP Client  : pymodbus  → 서버 주소 0~19 쓰기/읽기
 - Robot TCP Client   : QTcpSocket → 작업 명령 / 완료·좌표·CSV 수신

설치: pip install PyQt6 pymodbus

[로봇 통신 프로토콜 - 예시, 실제 로봇 프로그램에 맞게 수정]
  UI → 로봇 : "TASK,1\n" / "TASK,2\n"
  로봇 → UI : "POS,x,y,z,j1,j2,j3,j4,j5,j6\n"   현재 좌표/관절값
              "DATA,id,x,y,approach_z,measured_z\n" 작업2 측정 결과
              "CSV,v1,v2,...\n"                    일반 CSV 한 줄
              "DONE,1\n"                           작업 완료
              "ERROR,메시지\n"                    작업 오류
"""

import csv
import datetime
import sys

from pymodbus.client import ModbusTcpClient
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QBrush, QColor, QFont
from PyQt6.QtNetwork import QAbstractSocket, QTcpSocket
from PyQt6.QtWidgets import (
    QApplication,
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QPlainTextEdit,
    QPushButton,
    QSpinBox,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

# ========================= 설정 =========================
DEFAULT_MB_IP, DEFAULT_MB_PORT = "127.0.0.1", 5020     # 이 PC에서 연 Modbus 서버
DEFAULT_ROBOT_IP, DEFAULT_ROBOT_PORT = "192.168.137.100", 9000  # 로봇 DRL 소켓

UNIT_ID = 1
HR_START = 0          # Modbus 시작 주소 (0-based)
HR_COUNT = 20
POLL_MS = 500         # 레지스터 조회 주기
TASK_TIMEOUT_MS = 60_000

# 레지스터 맵 (모두 0-based Modbus 주소)
#   0~9   : 작업 1~10의 성공적인 완료 횟수
#   10~19 : 작업 2 측정 Z축 높이 1~10번 (x10, signed), 실행마다 이어서 3칸씩 기록
TASK_COUNT_START = 0
TASK_COUNT_SIZE = 10
TASK2_Z_START = 10
TASK2_Z_SIZE = 10
TASK2_MEAS_COUNT = 3  # 작업 2 1회 실행의 측정 개수
SCALE = 10            # Z축 높이의 0.1 단위 표현용

REG_NAMES = {}
for i in range(TASK_COUNT_SIZE):
    REG_NAMES[TASK_COUNT_START + i] = f"작업 {i + 1} 완료 횟수"
for i in range(TASK2_Z_SIZE):
    REG_NAMES[TASK2_Z_START + i] = f"작업 2 Z 높이 #{i + 1} (x10)"

STATE_IDLE, STATE_BUSY, STATE_DONE, STATE_ERROR = 0, 1, 2, 3
STATE_TEXT = {STATE_IDLE: "대기", STATE_BUSY: "작업 중", STATE_DONE: "완료", STATE_ERROR: "오류"}
STATE_COLOR = {
    STATE_IDLE: "#6b7280",
    STATE_BUSY: "#d97706",
    STATE_DONE: "#16a34a",
    STATE_ERROR: "#dc2626",
}


# ========================= 유틸 =========================
def to_reg(value: float) -> int:
    """실수 → signed 16bit (x10) 레지스터 값"""
    v = int(round(value * SCALE))
    v = max(-32768, min(32767, v))
    return v & 0xFFFF


def from_reg(raw: int) -> float:
    """레지스터 값 → 실수 (/10)"""
    return (raw - 65536 if raw >= 32768 else raw) / SCALE


def mb_call(fn, *args, **kw):
    """pymodbus 버전별 인자명 차이(device_id / slave) 흡수"""
    try:
        return fn(*args, device_id=UNIT_ID, **kw)
    except TypeError:
        return fn(*args, slave=UNIT_ID, **kw)


# ========================= 메인 UI =========================
class RobotUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("협동로봇 작업 UI")
        self.resize(1000, 760)

        self.mb: ModbusTcpClient | None = None
        self.mb_ok = False
        self.robot = QTcpSocket(self)
        self.robot_ready = False
        self.rx_buf = ""
        self.busy = False
        self.current_task = 0
        self.task2_base = 0    # 이번 작업 2 실행이 기록할 첫 Z 칸 (0 = 주소 10)
        self.csv_header_set = False

        self._build_ui()
        self._connect_signals()

        self.poll_timer = QTimer(self)
        self.poll_timer.timeout.connect(self.poll_registers)

        self.task_timer = QTimer(self)
        self.task_timer.setSingleShot(True)
        self.task_timer.timeout.connect(lambda: self.finish_task(False, "응답 시간 초과"))

        self.set_task_state(STATE_IDLE)
        self.update_buttons()

    # ---------------- UI 구성 ----------------
    def _build_ui(self):
        # 1) 통신 연결
        conn = QGroupBox("통신 연결")
        g = QGridLayout(conn)

        self.mb_ip = QLineEdit(DEFAULT_MB_IP)
        self.mb_port = QSpinBox()
        self.mb_port.setRange(1, 65535)
        self.mb_port.setValue(DEFAULT_MB_PORT)
        self.mb_btn = QPushButton("연결")
        self.mb_status = QLabel()

        self.rb_ip = QLineEdit(DEFAULT_ROBOT_IP)
        self.rb_port = QSpinBox()
        self.rb_port.setRange(1, 65535)
        self.rb_port.setValue(DEFAULT_ROBOT_PORT)
        self.rb_btn = QPushButton("연결")
        self.rb_status = QLabel()

        for row, (name, ip, port, btn, st) in enumerate([
            ("Modbus 서버", self.mb_ip, self.mb_port, self.mb_btn, self.mb_status),
            ("로봇", self.rb_ip, self.rb_port, self.rb_btn, self.rb_status),
        ]):
            g.addWidget(QLabel(name), row, 0)
            g.addWidget(QLabel("IP"), row, 1)
            g.addWidget(ip, row, 2)
            g.addWidget(QLabel("Port"), row, 3)
            g.addWidget(port, row, 4)
            g.addWidget(btn, row, 5)
            g.addWidget(st, row, 6)
        self.set_conn_label(self.mb_status, False)
        self.set_conn_label(self.rb_status, False)

        # 2) 작업 실행
        task = QGroupBox("작업 실행")
        h = QHBoxLayout(task)
        self.btn_task1 = QPushButton("작업 1")
        self.btn_task2 = QPushButton("작업 2")
        for b in (self.btn_task1, self.btn_task2):
            b.setMinimumHeight(48)
        self.task_state = QLabel()
        self.task_state.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.task_state.setMinimumWidth(160)
        h.addWidget(self.btn_task1)
        h.addWidget(self.btn_task2)
        h.addWidget(QLabel("상태:"))
        h.addWidget(self.task_state)

        # 3) 로봇 TCP에서 POS로 받은 현재 위치
        pos = QGroupBox("로봇 위치 (TCP POS 수신 기준)")
        ph = QHBoxLayout(pos)
        tcp_box, self.tcp_table = self._make_pos_table("TCP 좌표", ["X", "Y", "Z"], "위치 [mm]")
        jnt_box, self.joint_table = self._make_pos_table(
            "관절 각도", [f"J{i + 1}" for i in range(6)], "각도 [°]"
        )
        ph.addWidget(tcp_box, 3)
        ph.addWidget(jnt_box, 6)
        self.base_xyz = self.base_joint = None     # 작업 시작 시점 값 (변화량 기준)
        self.prev_xyz = self.prev_joint = None     # 직전 POS 수신 값 (변경 강조용)

        # 4) 작업 결과 1: 레지스터 모니터
        self.reg_table = QTableWidget(HR_COUNT, 4)
        self.reg_table.setHorizontalHeaderLabels(["주소", "항목", "Raw", "값"])
        self.reg_table.verticalHeader().setVisible(False)
        self.reg_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.reg_table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        for i in range(HR_COUNT):
            self.reg_table.setItem(i, 0, QTableWidgetItem(str(HR_START + i)))
            self.reg_table.setItem(i, 1, QTableWidgetItem(REG_NAMES.get(i, "-")))
            self.reg_table.setItem(i, 2, QTableWidgetItem("-"))
            self.reg_table.setItem(i, 3, QTableWidgetItem("-"))

        # 작업2 CSV 모니터
        csv_w = QWidget()
        cv = QVBoxLayout(csv_w)
        self.csv_table = QTableWidget(0, 0)
        self.csv_table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        cb = QHBoxLayout()
        self.btn_csv_load = QPushButton("CSV 불러오기")
        self.btn_csv_save = QPushButton("CSV 저장")
        self.btn_csv_clear = QPushButton("지우기")
        cb.addWidget(self.btn_csv_load)
        cb.addWidget(self.btn_csv_save)
        cb.addWidget(self.btn_csv_clear)
        cb.addStretch()
        cv.addLayout(cb)
        cv.addWidget(self.csv_table)

        tabs = QTabWidget()
        tabs.addTab(self.reg_table, "레지스터 모니터")
        tabs.addTab(csv_w, "작업2 CSV")

        # 5) 작업 결과 2: 작업 로그
        log_box = QGroupBox("작업 로그")
        lv = QVBoxLayout(log_box)
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.btn_log_save = QPushButton("로그 저장")
        lv.addWidget(self.log_view)
        lv.addWidget(self.btn_log_save, alignment=Qt.AlignmentFlag.AlignRight)

        root = QVBoxLayout(self)
        root.addWidget(conn)
        root.addWidget(task)
        root.addWidget(pos)
        root.addWidget(tabs, 3)
        root.addWidget(log_box, 2)

    def _make_pos_table(self, title, cols, unit):
        """열 = 축, 행 = [현재값, 작업 시작 대비 변화량]"""
        box = QWidget()
        v = QVBoxLayout(box)
        v.setContentsMargins(0, 0, 0, 0)
        cap = QLabel(title)
        cap.setStyleSheet("font-weight: bold;")
        v.addWidget(cap)

        t = QTableWidget(2, len(cols))
        t.setHorizontalHeaderLabels(cols)
        t.setVerticalHeaderLabels([unit, "변화량"])
        t.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        t.horizontalHeader().setStyleSheet(
            "QHeaderView::section { font-weight: bold; padding: 4px; }"
        )
        t.verticalHeader().setDefaultSectionSize(38)
        t.verticalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Fixed)
        t.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        t.setSelectionMode(QTableWidget.SelectionMode.NoSelection)
        t.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        t.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        t.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)

        big = QFont("Consolas")
        big.setStyleHint(QFont.StyleHint.Monospace)
        big.setPointSize(14)
        big.setBold(True)
        small = QFont("Consolas")
        small.setStyleHint(QFont.StyleHint.Monospace)
        small.setPointSize(10)
        for r in range(2):
            for c in range(len(cols)):
                it = QTableWidgetItem("-")
                it.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                it.setFont(big if r == 0 else small)
                t.setItem(r, c, it)
        t.setFixedHeight(2 * 38 + 28)
        v.addWidget(t)
        return box, t

    def _update_pos_table(self, table, values, prev, base):
        for c, val in enumerate(values):
            cur = table.item(0, c)
            cur.setText(f"{val:.1f}")
            # 직전 조회 대비 바뀐 칸은 노란색 강조
            if prev is not None and abs(val - prev[c]) >= 0.05:
                cur.setBackground(QBrush(QColor("#fde68a")))
                cur.setForeground(QBrush(QColor("#111827")))
            else:
                cur.setData(Qt.ItemDataRole.BackgroundRole, None)
                cur.setData(Qt.ItemDataRole.ForegroundRole, None)
            # 작업 시작 대비 변화량
            d_item = table.item(1, c)
            if base is None:
                d_item.setText("-")
                d_item.setData(Qt.ItemDataRole.ForegroundRole, None)
            else:
                d = val - base[c]
                d_item.setText(f"{d:+.1f}")
                if abs(d) < 0.05:
                    d_item.setData(Qt.ItemDataRole.ForegroundRole, None)
                else:
                    d_item.setForeground(QBrush(QColor("#2563eb" if d > 0 else "#dc2626")))

    def _connect_signals(self):
        self.mb_btn.clicked.connect(self.toggle_modbus)
        self.rb_btn.clicked.connect(self.toggle_robot)
        self.btn_task1.clicked.connect(lambda: self.start_task(1))
        self.btn_task2.clicked.connect(lambda: self.start_task(2))
        self.btn_csv_load.clicked.connect(self.load_csv)
        self.btn_csv_save.clicked.connect(self.save_csv)
        self.btn_csv_clear.clicked.connect(self.clear_csv)
        self.btn_log_save.clicked.connect(self.save_log)

        self.robot.connected.connect(self.on_robot_connected)
        self.robot.disconnected.connect(self.on_robot_disconnected)
        self.robot.readyRead.connect(self.on_robot_ready_read)
        self.robot.errorOccurred.connect(self.on_robot_error)

    # ---------------- 공통 ----------------
    def log(self, msg):
        t = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.log_view.appendPlainText(f"[{t}] {msg}")

    @staticmethod
    def set_conn_label(label, ok, text=None):
        label.setText(text or ("● 연결됨" if ok else "● 끊김"))
        label.setStyleSheet(f"color: {'#16a34a' if ok else '#dc2626'}; font-weight: bold;")

    def set_task_state(self, state):
        self.task_state.setText(STATE_TEXT[state])
        self.task_state.setStyleSheet(
            f"background:{STATE_COLOR[state]}; color:white; font: bold 16px; "
            "padding:8px; border-radius:6px;"
        )

    def update_buttons(self):
        robot_ok = self.robot.state() == QAbstractSocket.SocketState.ConnectedState
        enable = self.mb_ok and robot_ok and self.robot_ready and not self.busy
        self.btn_task1.setEnabled(enable)
        self.btn_task2.setEnabled(enable)
        self.mb_btn.setEnabled(not self.busy)
        self.rb_btn.setEnabled(not self.busy)

    # ---------------- Modbus ----------------
    def toggle_modbus(self):
        if self.mb_ok:
            self.poll_timer.stop()
            self.mb.close()
            self.mb_ok = False
            self.set_conn_label(self.mb_status, False)
            self.log("Modbus 연결 해제")
        else:
            ip, port = self.mb_ip.text().strip(), self.mb_port.value()
            self.mb = ModbusTcpClient(ip, port=port, timeout=1)
            if self.mb.connect():
                self.mb_ok = True
                self.set_conn_label(self.mb_status, True)
                self.log(f"Modbus 연결: {ip}:{port}")
                self.poll_registers()
                self.poll_timer.start(POLL_MS)
            else:
                self.set_conn_label(self.mb_status, False, "● 연결 실패")
                self.log(f"Modbus 연결 실패: {ip}:{port}")
        self.mb_btn.setText("해제" if self.mb_ok else "연결")
        self.mb_ip.setEnabled(not self.mb_ok)
        self.mb_port.setEnabled(not self.mb_ok)
        self.update_buttons()

    def write_regs(self, index, values):
        if not self.mb_ok:
            return False
        try:
            rr = mb_call(self.mb.write_registers, HR_START + index, list(values))
            if rr.isError():
                self.log(f"레지스터 쓰기 오류 (주소 {HR_START + index}): {rr}")
                return False
            return True
        except Exception as e:
            self.log(f"레지스터 쓰기 예외: {e}")
            return False

    def increment_task_count(self, task_number):
        """작업 N의 완료 횟수를 주소 N-1에 1 증가시킨다."""
        if not 1 <= task_number <= TASK_COUNT_SIZE:
            self.log(f"완료 횟수 기록 범위 초과: 작업 {task_number}")
            return False
        address = TASK_COUNT_START + task_number - 1
        try:
            rr = mb_call(self.mb.read_holding_registers, HR_START + address, count=1)
            if rr.isError():
                self.log(f"완료 횟수 읽기 오류 (주소 {address}): {rr}")
                return False
            count = rr.registers[0]
        except Exception as e:
            self.log(f"완료 횟수 읽기 예외: {e}")
            return False
        if count >= 0xFFFF:
            self.log(f"작업 {task_number} 완료 횟수가 최대치입니다.")
            return False
        return self.write_regs(address, [count + 1])

    def poll_registers(self):
        if not self.mb_ok:
            return
        try:
            rr = mb_call(self.mb.read_holding_registers, HR_START, count=HR_COUNT)
        except Exception as e:
            self.set_conn_label(self.mb_status, False, "● 응답 없음")
            self.log(f"레지스터 읽기 예외: {e}")
            return
        if rr.isError():
            self.set_conn_label(self.mb_status, False, "● 읽기 오류")
            return
        self.set_conn_label(self.mb_status, True)
        regs = rr.registers

        for i, raw in enumerate(regs):
            self.reg_table.item(i, 2).setText(str(raw))
            if TASK2_Z_START <= i < TASK2_Z_START + TASK2_Z_SIZE:
                self.reg_table.item(i, 3).setText(f"{from_reg(raw):.1f}")
            else:
                self.reg_table.item(i, 3).setText(str(raw))

    # ---------------- Robot socket ----------------
    def toggle_robot(self):
        if self.robot.state() == QAbstractSocket.SocketState.UnconnectedState:
            ip, port = self.rb_ip.text().strip(), self.rb_port.value()
            self.log(f"로봇 연결 시도: {ip}:{port}")
            self.set_conn_label(self.rb_status, False, "● 연결 중...")
            self.robot.connectToHost(ip, port)
        else:
            self.robot.disconnectFromHost()

    def on_robot_connected(self):
        self.robot_ready = False
        self.set_conn_label(self.rb_status, True, "● READY 대기")
        self.rb_btn.setText("해제")
        self.rb_ip.setEnabled(False)
        self.rb_port.setEnabled(False)
        self.log("로봇 연결됨")
        self.update_buttons()

    def on_robot_disconnected(self):
        self.robot_ready = False
        self.set_conn_label(self.rb_status, False)
        self.rb_btn.setText("연결")
        self.rb_ip.setEnabled(True)
        self.rb_port.setEnabled(True)
        self.log("로봇 연결 해제")
        if self.busy:
            self.finish_task(False, "작업 중 로봇 연결 끊김")
        self.update_buttons()

    def on_robot_error(self, err):
        self.log(f"로봇 소켓 오류: {self.robot.errorString()}")
        if self.robot.state() == QAbstractSocket.SocketState.UnconnectedState:
            self.set_conn_label(self.rb_status, False, "● 연결 실패")
        self.update_buttons()

    def on_robot_ready_read(self):
        self.rx_buf += bytes(self.robot.readAll()).decode(errors="ignore")
        while "\n" in self.rx_buf:
            line, self.rx_buf = self.rx_buf.split("\n", 1)
            line = line.strip()
            if line:
                self.handle_robot_line(line)

    def handle_robot_line(self, line):
        cmd, _, body = line.partition(",")
        cmd = cmd.upper()

        if cmd == "POS":
            try:
                vals = [float(v) for v in body.split(",")]
                if len(vals) < 9:
                    raise ValueError
            except ValueError:
                self.log(f"POS 형식 오류: {line}")
                return
            xyz, joint = vals[:3], vals[3:9]
            self._update_pos_table(self.tcp_table, xyz, self.prev_xyz, self.base_xyz)
            self._update_pos_table(self.joint_table, joint, self.prev_joint, self.base_joint)
            self.prev_xyz, self.prev_joint = xyz, joint

        elif cmd == "DATA":
            cells = [cell.strip() for cell in body.split(",")]
            try:
                if len(cells) != 5:
                    raise ValueError
                job_id = int(cells[0])
                measured_z = float(cells[4])
                if not 1 <= job_id <= TASK2_MEAS_COUNT:
                    raise ValueError
            except ValueError:
                self.log(f"DATA 형식 오류: {line}")
                return

            if self.busy and self.current_task == 2:
                self.append_csv_row(cells)
                address = TASK2_Z_START + self.task2_base + job_id - 1
                self.write_regs(address, [to_reg(measured_z)])
            else:
                self.log(f"작업 2 외 DATA 수신: {line}")

        elif cmd == "CSV":
            self.append_csv_row(body.split(","))

        elif cmd == "DONE":
            try:
                done_task = int(body.strip())
            except ValueError:
                self.log(f"DONE 형식 오류: {line}")
                return
            if done_task != self.current_task:
                self.log(f"DONE 작업 번호 불일치: {line}")
                return
            self.finish_task(True)

        elif cmd == "ERROR":
            self.finish_task(False, body or "로봇 오류")

        elif cmd == "READY":
            if not self.robot_ready:
                self.robot_ready = True
                self.set_conn_label(self.rb_status, True, "● 연결됨 / READY")
                self.log("로봇 READY 수신")
                self.update_buttons()

        else:
            self.log(f"로봇 수신: {line}")

    # ---------------- 작업 실행 ----------------
    def next_task2_slot(self):
        """마지막으로 기록된 Z 칸의 다음 칸 번호. 3칸이 남지 않았거나 읽기 실패면 None."""
        if not self.mb_ok:
            self.log("Modbus 미연결: 작업 2 측정값을 기록할 수 없습니다.")
            return None
        try:
            rr = mb_call(self.mb.read_holding_registers, HR_START + TASK2_Z_START,
                         count=TASK2_Z_SIZE)
        except Exception as e:
            self.log(f"측정 칸 읽기 예외: {e}")
            return None
        if rr.isError():
            self.log(f"측정 칸 읽기 오류: {rr}")
            return None
        used = [i for i, raw in enumerate(rr.registers) if raw != 0]
        base = used[-1] + 1 if used else 0
        if base + TASK2_MEAS_COUNT > TASK2_Z_SIZE:
            self.log(f"작업 2 측정 칸 부족 (남은 칸 {TASK2_Z_SIZE - base}개 / "
                     f"필요 {TASK2_MEAS_COUNT}개): 서버 초기화 필요")
            return None
        return base

    def start_task(self, n):
        if self.busy:
            return
        if n == 2:
            # 로봇을 움직이기 전에 측정값 3개를 기록할 칸이 있는지 확인한다.
            base = self.next_task2_slot()
            if base is None:
                return
            self.task2_base = base
        self.busy = True
        self.current_task = n
        self.update_buttons()
        self.set_task_state(STATE_BUSY)

        if n == 2:
            self.clear_csv()
            self.append_csv_row(["job_id", "x", "y", "approach_z", "measured_z"])
            self.log(f"작업 2 측정값 기록 위치: 주소 {TASK2_Z_START + self.task2_base}~"
                     f"{TASK2_Z_START + self.task2_base + TASK2_MEAS_COUNT - 1}")

        # 변화량 기준점 = 작업 시작 시점 위치
        self.base_xyz, self.base_joint = self.prev_xyz, self.prev_joint

        self.robot.write(f"TASK,{n}\n".encode())
        self.robot.flush()
        self.task_timer.start(TASK_TIMEOUT_MS)
        self.log(f"작업 {n} 시작")

    def finish_task(self, ok, reason=""):
        if not self.busy:
            return
        self.task_timer.stop()
        state = STATE_DONE if ok else STATE_ERROR
        if ok:
            self.increment_task_count(self.current_task)
            self.poll_registers()
        self.set_task_state(state)
        self.log(f"작업 {self.current_task} {'완료' if ok else '오류: ' + reason}")
        self.busy = False
        self.update_buttons()

    # ---------------- CSV ----------------
    def clear_csv(self):
        self.csv_table.clear()
        self.csv_table.setRowCount(0)
        self.csv_table.setColumnCount(0)
        self.csv_header_set = False

    def append_csv_row(self, cells):
        cells = [c.strip() for c in cells]
        # 첫 줄이 숫자가 아니면 헤더로 사용
        if not self.csv_header_set and self.csv_table.rowCount() == 0:
            try:
                [float(c) for c in cells]
            except ValueError:
                self.csv_table.setColumnCount(len(cells))
                self.csv_table.setHorizontalHeaderLabels(cells)
                self.csv_header_set = True
                return
        if len(cells) > self.csv_table.columnCount():
            self.csv_table.setColumnCount(len(cells))
        r = self.csv_table.rowCount()
        self.csv_table.insertRow(r)
        for c, v in enumerate(cells):
            if "." in v:                       # 정수(idx 등)는 그대로, 실수만 0.1 자리
                try:
                    v = f"{float(v):.1f}"
                except ValueError:
                    pass
            item = QTableWidgetItem(v)
            item.setTextAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            self.csv_table.setItem(r, c, item)
        self.csv_table.scrollToBottom()

    def load_csv(self):
        path, _ = QFileDialog.getOpenFileName(self, "CSV 불러오기", "", "CSV (*.csv)")
        if not path:
            return
        self.clear_csv()
        with open(path, newline="", encoding="utf-8-sig") as f:
            for row in csv.reader(f):
                if row:
                    self.append_csv_row(row)
        self.log(f"CSV 불러옴: {path}")

    def save_csv(self):
        path, _ = QFileDialog.getSaveFileName(self, "CSV 저장", "task2.csv", "CSV (*.csv)")
        if not path:
            return
        with open(path, "w", newline="", encoding="utf-8-sig") as f:
            w = csv.writer(f)
            if self.csv_header_set:
                w.writerow([self.csv_table.horizontalHeaderItem(c).text()
                            for c in range(self.csv_table.columnCount())])
            for r in range(self.csv_table.rowCount()):
                w.writerow([(self.csv_table.item(r, c).text() if self.csv_table.item(r, c) else "")
                            for c in range(self.csv_table.columnCount())])
        self.log(f"CSV 저장: {path}")

    def save_log(self):
        path, _ = QFileDialog.getSaveFileName(self, "로그 저장", "task_log.txt", "Text (*.txt)")
        if path:
            with open(path, "w", encoding="utf-8") as f:
                f.write(self.log_view.toPlainText())

    # ---------------- 종료 ----------------
    def closeEvent(self, e):
        self.poll_timer.stop()
        if self.mb:
            self.mb.close()
        self.robot.abort()
        super().closeEvent(e)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = RobotUI()
    w.show()
    sys.exit(app.exec())
