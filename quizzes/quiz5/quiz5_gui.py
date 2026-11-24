"""
퀴즈5 YOLO 판별 UI (PyQt6)

화면 = 검출 설정 / 현재 좌표 / 검사 결과 / 로그

통신 3개
  1) Modbus 서버  (이 PC, server.py, 5020)  : 검사 결과 40~49 쓰기·읽기, 로봇 좌표 복사 20~32
  2) 로봇 소켓    (로봇 DRL, 9000)          : TASK,A / TASK,B 명령 보내기 + 로봇 로그 받기
  3) 로봇 Modbus  (로봇 컨트롤러, 502)      : 로봇 현재 좌표 읽기 (TCP 400~405, 관절 270~275)

판정 A -> TASK,A -> 작업물을 왼쪽으로  (DONE,4)
판정 B -> TASK,B -> 작업물을 오른쪽으로 (DONE,5)

실행 (kw2026 activate 후, 저장소 루트에서)
  python quizzes\\quiz5\\server.py
  python quizzes\\quiz5\\quiz5_gui.py
"""

import datetime
import sys
import threading
from pathlib import Path

import cv2
from pymodbus.client import ModbusTcpClient
from PyQt6.QtCore import Qt, QThread, QTimer, pyqtSignal
from PyQt6.QtGui import QBrush, QColor, QFont, QImage, QPixmap
from PyQt6.QtNetwork import QAbstractSocket, QTcpSocket
from PyQt6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
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
    QVBoxLayout,
    QWidget,
)

QUIZ5_DIR = Path(__file__).resolve().parent
if str(QUIZ5_DIR) not in sys.path:
    sys.path.insert(0, str(QUIZ5_DIR))

import register_map as rm  # noqa: E402
import yolo_judge as yj  # noqa: E402

# ========================= 설정 =========================
DEFAULT_MB_IP, DEFAULT_MB_PORT = "127.0.0.1", rm.MODBUS_PORT
DEFAULT_ROBOT_IP = "192.168.137.100"
ROBOT_SOCK_PORT = 9000          # DRL server_socket_open 포트
ROBOT_MB_PORT = 502             # 두산 컨트롤러 Modbus TCP Slave
ROBOT_MB_UNIT = 1
ROBOT_POLL_SEC = 0.2            # 로봇 좌표 갱신 주기 (컨트롤러 값 자체가 100ms 주기)
MIRROR_POS_TO_SERVER = True     # 로봇 좌표를 서버 20~32번에도 복사

# 로봇 컨트롤러 Modbus 주소 (Doosan Modbus-TCP Slave IO Table V1.1, Holding Register)
ROBOT_REG_STATE = 259
ROBOT_REG_JOINT = 270           # J1~J6 [0.1도], signed
ROBOT_REG_TCP = 400             # X,Y,Z [0.1mm], A,B,C [0.1도], base 기준, signed

POLL_MS = 500                   # 서버 레지스터(검사 결과) 조회 주기
TASK_IDLE_TIMEOUT_MS = 60_000   # 이동 중 로봇 로그가 이 시간 동안 안 오면 '응답 없음'

# 판정 -> (작업 번호, 로봇 명령, 표시 이름). 완료 횟수는 서버 주소 (번호 - 1)
MOVES = {"A": (4, "A", "A → 왼쪽"), "B": (5, "B", "B → 오른쪽")}
DONE_TO_DECISION = {4: "A", 5: "B"}

ROBOT_STATE_TEXT = {
    0: "BACKDRIVE HOLD", 1: "BACKDRIVE RELEASE", 3: "SAFE OFF", 4: "INITIALIZING",
    5: "INTERRUPTED", 6: "EMERGENCY STOP", 8: "RECOVERY STANDBY", 11: "MANUAL STANDBY",
    12: "MANUAL JOGGING", 15: "STANDALONE STANDBY", 16: "STANDALONE RUNNING",
    17: "COLLABORATIVE STANDBY", 18: "COLLABORATIVE RUNNING",
}
DECISION_COLOR = {"A": "#2563eb", "B": "#dc2626"}


def mb_call(fn, *args, unit=rm.DEVICE_ID, **kw):
    """pymodbus 버전별 인자명 차이(device_id / slave) 흡수"""
    try:
        return fn(*args, device_id=unit, **kw)
    except TypeError:
        return fn(*args, slave=unit, **kw)


def bgr_to_qimage(bgr, max_width=960):
    h, w = bgr.shape[:2]
    if w > max_width:
        bgr = cv2.resize(bgr, (max_width, round(h * max_width / w)))
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    h, w, ch = rgb.shape
    return QImage(rgb.data, w, h, ch * w, QImage.Format.Format_RGB888).copy()


# ========================= 로봇 현재 좌표 (로봇 Modbus 502) =========================
class RobotModbusThread(QThread):
    """로봇 컨트롤러 Modbus Slave에서 TCP/관절/상태를 주기적으로 읽는다 (UI 멈춤 방지용 스레드)."""

    pos_ready = pyqtSignal(list, list, int)     # tcp6, joint6, robot_state
    link_changed = pyqtSignal(bool, str)

    def __init__(self, ip, port, parent=None):
        super().__init__(parent)
        self.ip, self.port = ip, port
        self._stop = threading.Event()

    def stop(self):
        self._stop.set()

    def _read(self, client, address, count):
        rr = mb_call(client.read_holding_registers, address, count=count, unit=ROBOT_MB_UNIT)
        if rr.isError():
            raise ConnectionError(f"주소 {address} 읽기 오류: {rr}")
        return rr.registers

    def run(self):
        client = ModbusTcpClient(self.ip, port=self.port, timeout=1)
        linked = None
        try:
            while not self._stop.is_set():
                try:
                    if not client.connected and not client.connect():
                        raise ConnectionError("연결 실패")
                    state = self._read(client, ROBOT_REG_STATE, 1)[0]
                    joint = [rm.to_signed(v) / 10 for v in self._read(client, ROBOT_REG_JOINT, 6)]
                    tcp = [rm.to_signed(v) / 10 for v in self._read(client, ROBOT_REG_TCP, 6)]
                except Exception as e:   # 연결 끊김·응답 없음 모두 1초 뒤 재시도
                    if linked is not False:
                        self.link_changed.emit(False, str(e))
                        linked = False
                    client.close()
                    self._stop.wait(1.0)
                    continue
                if linked is not True:
                    self.link_changed.emit(True, "")
                    linked = True
                self.pos_ready.emit(tcp, joint, state)
                self._stop.wait(ROBOT_POLL_SEC)
        finally:
            client.close()


# ========================= YOLO 판별 스레드 =========================
class YoloThread(QThread):
    progress = pyqtSignal(int, int, str, int, QImage)   # i, total, 이름, 개 수, 결과 사진
    done = pyqtSignal(object)                            # yolo_judge.JudgeResult
    failed = pyqtSignal(str)

    def __init__(self, cache, root, photos, folder, threshold, conf, imgsz, mode, parent=None):
        super().__init__(parent)
        self.cache = cache          # {"root": Path, "model": YOLO} — 모델은 한 번만 불러온다
        self.args = (root, photos, folder, threshold, conf, imgsz, mode)

    def run(self):
        root, photos, folder, threshold, conf, imgsz, mode = self.args
        try:
            if self.cache.get("root") != root:
                self.cache["model"] = yj.load_model(root)
                self.cache["root"] = root

            def report(i, total, item, annotated):
                self.progress.emit(i, total, item.name, len(item.scores), bgr_to_qimage(annotated))

            result = yj.judge(self.cache["model"], photos, folder, threshold, conf, imgsz, mode,
                              out_root=root / "output_images", progress=report)
        except Exception as e:     # 모델 없음 / ultralytics 미설치 / 사진 읽기 실패
            self.failed.emit(f"{type(e).__name__}: {e}")
            return
        self.done.emit(result)


# ========================= 메인 UI =========================
class Quiz5UI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("퀴즈5 YOLO 판별 — 두산 M0609")
        self.resize(1280, 900)

        self.mb: ModbusTcpClient | None = None
        self.mb_ok = False
        self.robot = QTcpSocket(self)
        self.robot_ready = False
        self.robot_unsynced = False     # 응답 없음으로 끝난 뒤 로봇 상태를 모르는 상태
        self.rx_buf = b""
        self.moving = None              # 이동 중인 판정 "A"/"B", 대기 중이면 None

        self.robot_mb_thread: RobotModbusThread | None = None
        self.prev_tcp = self.prev_joint = None
        self.last_mirror = None

        self.yolo_thread: YoloThread | None = None
        self.yolo_cache = {}
        self.yolo_pixmap = None

        self._build_ui()
        self._connect_signals()

        self.poll_timer = QTimer(self)
        self.poll_timer.timeout.connect(self.poll_results)
        self.task_timer = QTimer(self)
        self.task_timer.setSingleShot(True)
        self.task_timer.timeout.connect(self.on_move_timeout)

        self.set_move_state("대기", "#6b7280")
        self.update_buttons()
        self.refresh_photo_count()

    # ---------------- UI 구성 ----------------
    def _build_ui(self):
        root = QVBoxLayout(self)
        root.addWidget(self._build_conn())

        top = QHBoxLayout()
        top.addWidget(self._build_settings(), 2)
        top.addWidget(self._build_position(), 3)
        root.addLayout(top)

        root.addWidget(self._build_result(), 4)
        root.addWidget(self._build_log(), 2)

    def _build_conn(self):
        box = QGroupBox("통신 연결")
        g = QGridLayout(box)

        def row(r, name, ip, port):
            ip_w, port_w = QLineEdit(ip), QSpinBox()
            port_w.setRange(1, 65535)
            port_w.setValue(port)
            btn, st = QPushButton("연결"), QLabel()
            for c, w in enumerate([QLabel(name), QLabel("IP"), ip_w, QLabel("Port"),
                                   port_w, btn, st]):
                g.addWidget(w, r, c)
            self.set_conn_label(st, False)
            return ip_w, port_w, btn, st

        self.mb_ip, self.mb_port, self.mb_btn, self.mb_status = row(
            0, "Modbus 서버 (검사 결과)", DEFAULT_MB_IP, DEFAULT_MB_PORT)
        self.rb_ip, self.rb_port, self.rb_btn, self.rb_status = row(
            1, "로봇 소켓 (명령·로그)", DEFAULT_ROBOT_IP, ROBOT_SOCK_PORT)
        self.rm_ip, self.rm_port, self.rm_btn, self.rm_status = row(
            2, "로봇 Modbus (현재 좌표)", DEFAULT_ROBOT_IP, ROBOT_MB_PORT)
        return box

    def _build_settings(self):
        box = QGroupBox("검출 설정")
        f = QFormLayout(box)

        folder_row = QHBoxLayout()
        self.yolo_root = QLineEdit(str(yj.DEFAULT_ROOT))
        self.btn_yolo_root = QPushButton("폴더…")
        folder_row.addWidget(self.yolo_root, 1)
        folder_row.addWidget(self.btn_yolo_root)

        self.yolo_dataset = QComboBox()
        self.yolo_dataset.addItems(yj.DATASETS)
        self.yolo_dataset.setCurrentText("valid")
        self.yolo_n = QSpinBox()
        self.yolo_n.setRange(1, 999)
        self.yolo_n.setValue(20)
        self.yolo_conf = QDoubleSpinBox()
        self.yolo_conf.setRange(0.01, 1.0)
        self.yolo_conf.setSingleStep(0.05)
        self.yolo_conf.setValue(yj.CONF)
        self.yolo_imgsz = QComboBox()
        self.yolo_imgsz.addItems(["320", "480", "640", "960"])
        self.yolo_imgsz.setCurrentText(str(yj.IMGSZ))
        self.yolo_prep = QComboBox()
        self.yolo_prep.addItems(yj.PREPROCESS_CHOICES)
        self.yolo_prep.setCurrentText(yj.PREPROCESS)
        self.yolo_threshold = QDoubleSpinBox()
        self.yolo_threshold.setRange(0, 100)
        self.yolo_threshold.setDecimals(1)
        self.yolo_threshold.setSuffix(" %")
        self.yolo_threshold.setValue(80)
        self.yolo_auto = QCheckBox("판별 후 로봇 자동 이동 (A 왼쪽 / B 오른쪽)")
        self.yolo_auto.setChecked(True)
        self.btn_yolo = QPushButton("YOLO 판별 실행")
        self.btn_yolo.setMinimumHeight(44)
        self.btn_yolo.setStyleSheet("font: bold 14px;")

        f.addRow("Yolo 폴더", folder_row)
        self.photo_count_label = QLabel("-")
        f.addRow("사진 묶음", self.yolo_dataset)
        f.addRow("찾은 사진", self.photo_count_label)
        f.addRow("검사 사진 수", self.yolo_n)
        f.addRow("CONF", self.yolo_conf)
        f.addRow("IMGSZ", self.yolo_imgsz)
        f.addRow("PREPROCESS", self.yolo_prep)
        f.addRow("기준 값", self.yolo_threshold)
        f.addRow(self.yolo_auto)
        f.addRow(self.btn_yolo)
        return box

    def _build_position(self):
        box = QGroupBox("현재 좌표 (로봇 Modbus 502)")
        v = QVBoxLayout(box)
        self.tcp_table = self._make_pos_table(rm.TCP_NAMES, ["mm", "mm", "mm", "°", "°", "°"])
        self.joint_table = self._make_pos_table([f"J{i + 1}" for i in range(6)], ["°"] * 6)
        self.robot_state_label = QLabel("로봇 상태: -")
        v.addWidget(QLabel("TCP (Base 기준)"))
        v.addWidget(self.tcp_table)
        v.addWidget(QLabel("관절 각도"))
        v.addWidget(self.joint_table)
        v.addWidget(self.robot_state_label)
        v.addStretch()
        return box

    def _make_pos_table(self, cols, units):
        t = QTableWidget(1, len(cols))
        t.setHorizontalHeaderLabels([f"{c} [{u}]" for c, u in zip(cols, units, strict=True)])
        t.verticalHeader().setVisible(False)
        t.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        t.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        t.setSelectionMode(QTableWidget.SelectionMode.NoSelection)
        t.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        font = QFont("Consolas", 14)
        font.setBold(True)
        for c in range(len(cols)):
            it = QTableWidgetItem("-")
            it.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            it.setFont(font)
            t.setItem(0, c, it)
        t.verticalHeader().setDefaultSectionSize(36)
        t.setFixedHeight(36 + 30)
        return t

    def _build_result(self):
        box = QGroupBox("검사 결과 (Modbus 서버 40~49번 값)")
        h = QHBoxLayout(box)

        # 왼쪽: 결과 5개 + 판별 결과 + 로봇 이동
        left = QVBoxLayout()
        grid = QGridLayout()
        self.result_labels = {}
        items = [
            (rm.YOLO_PHOTOS, "검출한 사진 수"),
            (rm.YOLO_PHOTOS_WITH_DOG, "개가 검출된 사진 수"),
            (rm.YOLO_DOGS, "검출된 개의 수"),
            (rm.YOLO_MEAN, "평균 신뢰도"),
            (rm.YOLO_THRESHOLD, "기준 값"),
            (rm.YOLO_RUN_COUNT, "판별 횟수"),
        ]
        value_font = QFont("Consolas", 15)
        value_font.setBold(True)
        for r, (addr, name) in enumerate(items):
            grid.addWidget(QLabel(f"{name}  (주소 {addr})"), r, 0)
            lab = QLabel("-")
            lab.setFont(value_font)
            lab.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            grid.addWidget(lab, r, 1)
            self.result_labels[addr] = lab
        left.addLayout(grid)

        left.addWidget(QLabel(f"판별 결과  (주소 {rm.YOLO_DECISION})"))
        self.decision_label = QLabel("-")
        self.decision_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.decision_label.setMinimumHeight(64)
        self._set_decision("-")
        left.addWidget(self.decision_label)

        move_row = QHBoxLayout()
        move_row.addWidget(QLabel("로봇 이동:"))
        self.move_state = QLabel()
        self.move_state.setAlignment(Qt.AlignmentFlag.AlignCenter)
        move_row.addWidget(self.move_state, 1)
        left.addLayout(move_row)

        manual = QHBoxLayout()
        self.btn_move = {d: QPushButton(f"수동 {MOVES[d][2]}") for d in MOVES}
        for b in self.btn_move.values():
            manual.addWidget(b)
        left.addLayout(manual)
        self.count_label = QLabel("이동 완료 횟수  A: -  /  B: -")
        left.addWidget(self.count_label)
        left.addStretch()

        # 가운데: 사진별 결과
        self.photo_table = QTableWidget(0, 3)
        self.photo_table.setHorizontalHeaderLabels(["사진", "개 수", "신뢰도"])
        self.photo_table.horizontalHeader().setSectionResizeMode(
            0, QHeaderView.ResizeMode.Stretch)
        self.photo_table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.photo_table.verticalHeader().setVisible(False)

        # 오른쪽: 검출 사진
        right = QVBoxLayout()
        self.photo_view = QLabel("판별하면 검출 사진이 표시됩니다.")
        self.photo_view.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.photo_view.setMinimumSize(360, 240)
        self.photo_view.setStyleSheet("background:#111827; color:#d1d5db;")
        self.progress_label = QLabel("")
        right.addWidget(self.photo_view, 1)
        right.addWidget(self.progress_label)

        h.addLayout(left, 3)
        h.addWidget(self.photo_table, 3)
        h.addLayout(right, 4)
        return box

    def _build_log(self):
        box = QGroupBox("로그 (로봇 로그 = 소켓 수신)")
        v = QVBoxLayout(box)
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.btn_log_save = QPushButton("로그 저장")
        v.addWidget(self.log_view)
        v.addWidget(self.btn_log_save, alignment=Qt.AlignmentFlag.AlignRight)
        return box

    def _connect_signals(self):
        self.mb_btn.clicked.connect(self.toggle_modbus)
        self.rb_btn.clicked.connect(self.toggle_robot)
        self.rm_btn.clicked.connect(self.toggle_robot_modbus)
        self.btn_yolo.clicked.connect(self.start_yolo)
        self.btn_yolo_root.clicked.connect(self.pick_yolo_root)
        self.yolo_root.editingFinished.connect(self.refresh_photo_count)
        self.yolo_dataset.currentTextChanged.connect(self.refresh_photo_count)
        for d, b in self.btn_move.items():
            b.clicked.connect(lambda _=False, d=d: self.start_move(d))
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

    def set_move_state(self, text, color):
        self.move_state.setText(text)
        self.move_state.setStyleSheet(f"background:{color}; color:white; font: bold 14px; "
                                      "padding:6px; border-radius:6px;")

    def _set_decision(self, text):
        color = DECISION_COLOR.get(text, "#6b7280")
        shown = {"A": "A  (왼쪽)", "B": "B  (오른쪽)"}.get(text, text)
        self.decision_label.setText(shown)
        self.decision_label.setStyleSheet(f"background:{color}; color:white; font: bold 26px; "
                                          "border-radius:8px;")

    def robot_link_ok(self):
        sock_ok = self.robot.state() == QAbstractSocket.SocketState.ConnectedState
        return self.mb_ok and sock_ok and self.robot_ready and not self.robot_unsynced

    def update_buttons(self):
        can_move = self.robot_link_ok() and self.moving is None
        for b in self.btn_move.values():
            b.setEnabled(can_move)
        self.btn_yolo.setEnabled(self.yolo_thread is None and self.moving is None)
        self.mb_btn.setEnabled(self.moving is None)
        self.rb_btn.setEnabled(self.moving is None)

    # ---------------- Modbus 서버 (검사 결과) ----------------
    def toggle_modbus(self):
        if self.mb_ok:
            self.poll_timer.stop()
            self.mb.close()
            self.mb_ok = False
            self.set_conn_label(self.mb_status, False)
            self.log("Modbus 서버 연결 해제")
        else:
            ip, port = self.mb_ip.text().strip(), self.mb_port.value()
            self.mb = ModbusTcpClient(ip, port=port, timeout=1)
            if self.mb.connect():
                self.mb_ok = True
                self.last_mirror = None
                self.set_conn_label(self.mb_status, True)
                self.log(f"Modbus 서버 연결: {ip}:{port}")
                self.poll_results()
                self.poll_timer.start(POLL_MS)
            else:
                self.set_conn_label(self.mb_status, False, "● 연결 실패")
                self.log(f"Modbus 서버 연결 실패: {ip}:{port} (server.py 실행 확인)")
        self.mb_btn.setText("해제" if self.mb_ok else "연결")
        self.update_buttons()

    def read_regs(self, address, count):
        if not self.mb_ok:
            return None
        try:
            rr = mb_call(self.mb.read_holding_registers, address, count=count)
        except Exception as e:
            self.log(f"레지스터 읽기 예외 (주소 {address}): {e}")
            return None
        if rr.isError():
            self.log(f"레지스터 읽기 오류 (주소 {address}): {rr}")
            return None
        return rr.registers

    def write_regs(self, address, values):
        if not self.mb_ok:
            return False
        try:
            rr = mb_call(self.mb.write_registers, address, list(values))
        except Exception as e:
            self.log(f"레지스터 쓰기 예외 (주소 {address}): {e}")
            return False
        if rr.isError():
            self.log(f"레지스터 쓰기 오류 (주소 {address}): {rr}")
            return False
        return True

    def poll_results(self):
        """서버에서 검사 결과(40~49)와 이동 완료 횟수(3, 4)를 읽어 화면에 표시한다."""
        if not self.mb_ok:
            return
        try:
            rr = mb_call(self.mb.read_holding_registers, 0, count=rm.REGISTER_COUNT)
        except Exception:
            self.set_conn_label(self.mb_status, False, "● 응답 없음")
            return
        if rr.isError():
            self.set_conn_label(self.mb_status, False, "● 읽기 오류")
            return
        self.set_conn_label(self.mb_status, True)
        regs = rr.registers
        for addr, lab in self.result_labels.items():
            lab.setText(rm.describe(addr, regs[addr]))
        self._set_decision(rm.describe(rm.YOLO_DECISION, regs[rm.YOLO_DECISION]))
        a_n, b_n = (regs[rm.COUNT_START + MOVES[d][0] - 1] for d in ("A", "B"))
        self.count_label.setText(f"이동 완료 횟수  A(왼쪽): {a_n}  /  B(오른쪽): {b_n}")

    # ---------------- 로봇 Modbus (현재 좌표) ----------------
    def toggle_robot_modbus(self):
        if self.robot_mb_thread is not None:
            self.robot_mb_thread.stop()
            self.rm_btn.setEnabled(False)          # 스레드가 끝나면 다시 활성화
            return
        ip, port = self.rm_ip.text().strip(), self.rm_port.value()
        self.log(f"로봇 Modbus 연결 시도: {ip}:{port}")
        self.set_conn_label(self.rm_status, False, "● 연결 중...")
        self.robot_mb_thread = RobotModbusThread(ip, port, self)
        self.robot_mb_thread.pos_ready.connect(self.on_robot_pos)
        self.robot_mb_thread.link_changed.connect(self.on_robot_mb_link)
        self.robot_mb_thread.finished.connect(self.on_robot_mb_finished)
        self.robot_mb_thread.start()
        self.rm_btn.setText("해제")
        self.rm_ip.setEnabled(False)
        self.rm_port.setEnabled(False)

    def on_robot_mb_link(self, ok, reason):
        if ok:
            self.set_conn_label(self.rm_status, True)
            self.log("로봇 Modbus 연결됨 (현재 좌표 수신 시작)")
        else:
            self.set_conn_label(self.rm_status, False, "● 재시도 중")
            self.log(f"로봇 Modbus 끊김/실패: {reason} → 1초마다 재시도")

    def on_robot_mb_finished(self):
        self.robot_mb_thread = None
        self.set_conn_label(self.rm_status, False)
        self.rm_btn.setText("연결")
        self.rm_btn.setEnabled(True)
        self.rm_ip.setEnabled(True)
        self.rm_port.setEnabled(True)
        self.log("로봇 Modbus 연결 해제")

    @staticmethod
    def _update_pos_table(table, values, prev):
        for c, val in enumerate(values):
            it = table.item(0, c)
            it.setText(f"{val:.1f}")
            if prev is not None and abs(val - prev[c]) >= 0.05:     # 움직이는 축 강조
                it.setBackground(QBrush(QColor("#fde68a")))
                it.setForeground(QBrush(QColor("#111827")))
            else:
                it.setData(Qt.ItemDataRole.BackgroundRole, None)
                it.setData(Qt.ItemDataRole.ForegroundRole, None)

    def on_robot_pos(self, tcp, joint, state):
        self._update_pos_table(self.tcp_table, tcp, self.prev_tcp)
        self._update_pos_table(self.joint_table, joint, self.prev_joint)
        self.prev_tcp, self.prev_joint = tcp, joint
        self.robot_state_label.setText(f"로봇 상태: {state} ({ROBOT_STATE_TEXT.get(state, '-')})")

        if MIRROR_POS_TO_SERVER and self.mb_ok:
            values = ([rm.scaled(v, 10) for v in tcp] + [rm.scaled(v, 10) for v in joint]
                      + [state & 0xFFFF])
            if values != self.last_mirror and self.write_regs(rm.ROBOT_TCP_START, values):
                self.last_mirror = values

    # ---------------- 로봇 소켓 (명령 · 로그) ----------------
    def toggle_robot(self):
        if self.robot.state() == QAbstractSocket.SocketState.UnconnectedState:
            ip, port = self.rb_ip.text().strip(), self.rb_port.value()
            self.log(f"로봇 소켓 연결 시도: {ip}:{port}")
            self.set_conn_label(self.rb_status, False, "● 연결 중...")
            self.robot.connectToHost(ip, port)
        else:
            self.robot.disconnectFromHost()

    def on_robot_connected(self):
        self.robot_ready = False
        self.robot_unsynced = False
        self.rx_buf = b""
        self.set_conn_label(self.rb_status, True, "● READY 대기")
        self.rb_btn.setText("해제")
        self.log("로봇 소켓 연결됨")
        self.update_buttons()

    def on_robot_disconnected(self):
        self.robot_ready = False
        self.set_conn_label(self.rb_status, False)
        self.rb_btn.setText("연결")
        self.log("로봇 소켓 연결 해제")
        if self.moving is not None:
            self.finish_move(False, "이동 중 로봇 연결 끊김")
        self.robot_unsynced = False
        self.update_buttons()

    def on_robot_error(self, _err):
        self.log(f"로봇 소켓 오류: {self.robot.errorString()}")
        if self.robot.state() == QAbstractSocket.SocketState.UnconnectedState:
            self.set_conn_label(self.rb_status, False, "● 연결 실패")
        self.update_buttons()

    def on_robot_ready_read(self):
        # 한글 로그("MoveJ 종료")가 TCP 조각에서 잘려도 깨지지 않게 줄 단위로 디코딩
        self.rx_buf += bytes(self.robot.readAll())
        while b"\n" in self.rx_buf:
            raw, self.rx_buf = self.rx_buf.split(b"\n", 1)
            line = raw.decode("utf-8", errors="replace").strip()
            if line:
                self.handle_robot_line(line)

    def handle_robot_line(self, line):
        self.log(f"[로봇] {line}")          # 로봇 로그는 소켓으로 받아서 그대로 남긴다
        cmd, _, body = line.partition(",")
        cmd = cmd.strip().upper()

        if self.moving is not None:
            self.task_timer.start(TASK_IDLE_TIMEOUT_MS)     # 로그가 오는 동안은 타임아웃 아님
        elif self.robot_unsynced and (cmd in ("DONE", "READY") or cmd.startswith("UNKNOWN")):
            self.robot_unsynced = False
            self.log("로봇 동작 종료 확인 → 다시 이동 가능")
            self.update_buttons()

        if cmd == "READY":
            if not self.robot_ready:
                self.robot_ready = True
                self.set_conn_label(self.rb_status, True, "● 연결됨 / READY")
                self.update_buttons()
        elif cmd == "DONE":
            try:
                n = int(body.strip())
            except ValueError:
                self.log(f"DONE 형식 오류: {line}")
                return
            if self.moving is None:
                return
            if DONE_TO_DECISION.get(n) != self.moving:
                self.log(f"DONE 번호 불일치 (보낸 {self.moving}, 받은 DONE,{n})")
                return
            self.finish_move(True)
        elif cmd == "ERROR":
            self.finish_move(False, body or "로봇 오류")
        elif cmd.startswith("UNKNOWN"):
            self.finish_move(False, f"로봇이 모르는 명령 ({line}) — 펜던트 DRL 확인")

    # ---------------- 로봇 이동 (A 왼쪽 / B 오른쪽) ----------------
    def start_move(self, decision):
        n, command, name = MOVES[decision]
        if self.moving is not None:
            self.log(f"{name} 이동 안 함: 이미 이동 중")
            return False
        if not self.robot_link_ok():
            self.log(f"{name} 이동 안 함: Modbus 서버/로봇 소켓 연결 또는 READY 미수신"
                     + (" (이전 이동 응답 없음)" if self.robot_unsynced else ""))
            return False
        self.moving = decision
        self.set_move_state(f"{name} 이동 중", "#d97706")
        self.update_buttons()
        self.robot.write(f"TASK,{command}\n".encode())
        self.robot.flush()
        self.task_timer.start(TASK_IDLE_TIMEOUT_MS)
        self.log(f"{name} 이동 시작 (명령: TASK,{command})")
        return True

    def finish_move(self, ok, reason=""):
        if self.moving is None:
            return
        self.task_timer.stop()
        n, _, name = MOVES[self.moving]
        if ok:
            address = rm.COUNT_START + n - 1
            regs = self.read_regs(address, 1)
            if regs is not None and regs[0] < 0xFFFF:
                self.write_regs(address, [regs[0] + 1])
            self.set_move_state(f"{name} 완료", "#16a34a")
            self.log(f"{name} 이동 완료")
        else:
            self.set_move_state(f"{name} 오류", "#dc2626")
            self.log(f"{name} 이동 오류: {reason}")
        self.moving = None
        self.poll_results()
        self.update_buttons()

    def on_move_timeout(self):
        if self.moving is None:
            return
        self.robot_unsynced = True      # 로봇은 아직 움직이는 중일 수 있음 → 새 명령 금지
        self.finish_move(False, f"로봇 응답 없음 ({TASK_IDLE_TIMEOUT_MS // 1000}초)")
        self.log("로봇이 DONE을 보내거나 재연결할 때까지 이동 명령을 보내지 않습니다. 펜던트 확인.")
        self.update_buttons()

    # ---------------- YOLO 판별 ----------------
    def pick_yolo_root(self):
        path = QFileDialog.getExistingDirectory(self, "Yolo 폴더 선택", self.yolo_root.text())
        if path:
            self.yolo_root.setText(path)
            self.refresh_photo_count()

    def _photo_folder(self):
        text = self.yolo_root.text().strip().strip('"')
        return yj.resolve_folder(Path(text), self.yolo_dataset.currentText())

    def refresh_photo_count(self):
        """경로·사진 묶음이 바뀔 때마다 실제로 찾은 사진 수를 표시한다."""
        _, folder = self._photo_folder()
        count = len(yj.list_photos(folder))
        ok = count >= self.yolo_n.value()
        self.photo_count_label.setText(f"{count}장  ({folder})")
        self.photo_count_label.setStyleSheet(f"color: {'#16a34a' if ok else '#dc2626'};")
        self.photo_count_label.setToolTip("" if count else yj.diagnose(folder))

    def start_yolo(self):
        if self.yolo_thread is not None or self.moving is not None:
            return
        root, folder = self._photo_folder()
        photos = yj.list_photos(folder)
        self.refresh_photo_count()
        n = self.yolo_n.value()
        if len(photos) < n:
            self.log(f"YOLO: 사진이 부족합니다 ({folder}에 {len(photos)}장, 필요 {n}장)")
            if not photos:
                self.log(f"YOLO: {yj.diagnose(folder)}")
            return
        conf, imgsz = self.yolo_conf.value(), int(self.yolo_imgsz.currentText())
        mode, threshold = self.yolo_prep.currentText(), self.yolo_threshold.value()
        self.log(f"YOLO 판별 시작: {folder} {n}장, CONF={conf:.2f}, IMGSZ={imgsz}, "
                 f"PREPROCESS={mode}, 기준={threshold:g}% (첫 실행은 모델 로딩으로 수 초)")
        self.photo_table.setRowCount(0)
        self.progress_label.setText("판별 중…")
        self.yolo_thread = YoloThread(self.yolo_cache, root, photos[:n], folder,
                                      threshold, conf, imgsz, mode, self)
        self.yolo_thread.progress.connect(self.on_yolo_progress)
        self.yolo_thread.done.connect(self.on_yolo_done)
        self.yolo_thread.failed.connect(self.on_yolo_failed)
        self.yolo_thread.finished.connect(self.on_yolo_finished)
        self.yolo_thread.start()
        self.update_buttons()

    def on_yolo_progress(self, i, total, name, dogs, image):
        self.progress_label.setText(f"판별 중… {i}/{total}  {name}")
        self.yolo_pixmap = QPixmap.fromImage(image)
        self._show_photo()
        row = self.photo_table.rowCount()
        self.photo_table.insertRow(row)
        self.photo_table.setItem(row, 0, QTableWidgetItem(name))
        self.photo_table.setItem(row, 1, QTableWidgetItem(str(dogs)))
        self.photo_table.scrollToBottom()

    def on_yolo_done(self, r):
        for row, item in enumerate(r.per_photo):
            scores = ", ".join(f"{s:.2f}" for s in item.scores) or "-"
            if row < self.photo_table.rowCount():
                self.photo_table.setItem(row, 2, QTableWidgetItem(scores))
        self.progress_label.setText(f"저장 폴더: {r.out_dir}")
        self.log(f"YOLO 판별 완료: {r.summary()}")

        # Modbus 서버 40~49번 기록 (판별 횟수 = 서버 값 + 1) → 화면 값은 서버에서 다시 읽어 표시
        prev = self.read_regs(rm.YOLO_RUN_COUNT, 1)
        if prev is None:
            self.log("검사 결과를 Modbus 서버에 쓰지 못함 (서버 미연결) → 화면 결과도 갱신 안 됨")
            self._set_decision(r.decision)
        else:
            values = rm.yolo_registers(r.photos, r.photos_with_dog, r.dogs, r.mean_percent,
                                       r.threshold_percent, r.decision, prev[0] + 1,
                                       r.conf, r.imgsz, r.preprocess)
            if self.write_regs(rm.YOLO_PHOTOS, values):
                self.log(f"검사 결과 → Modbus 주소 {rm.YOLO_PHOTOS}~{rm.YOLO_PREPROCESS} 기록")
            self.poll_results()

        if self.yolo_auto.isChecked():
            self.log(f"판정 {r.decision} → {MOVES[r.decision][2]}")
            self.start_move(r.decision)

    def on_yolo_failed(self, message):
        self.progress_label.setText("판별 실패 (로그 확인)")
        self.log(f"YOLO 오류: {message}")

    def on_yolo_finished(self):
        self.yolo_thread = None
        self.update_buttons()

    # ---------------- 화면 ----------------
    def _show_photo(self):
        if self.yolo_pixmap is not None:
            self.photo_view.setPixmap(self.yolo_pixmap.scaled(
                self.photo_view.size(), Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation))

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._show_photo()

    def save_log(self):
        path, _ = QFileDialog.getSaveFileName(self, "로그 저장", "quiz5_log.txt", "Text (*.txt)")
        if path:
            Path(path).write_text(self.log_view.toPlainText(), encoding="utf-8")

    def closeEvent(self, e):
        self.poll_timer.stop()
        if self.robot_mb_thread is not None:
            self.robot_mb_thread.stop()
        for t in (self.robot_mb_thread, self.yolo_thread):
            if t is not None and not t.wait(5000):
                self.log("스레드 종료 대기 중입니다. 잠시 후 다시 닫아 주세요.")
                e.ignore()
                return
        if self.mb:
            self.mb.close()
        self.robot.abort()
        super().closeEvent(e)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = Quiz5UI()
    w.show()
    sys.exit(app.exec())
