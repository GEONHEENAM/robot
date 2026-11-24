"""
퀴즈5 협동로봇 작업 UI (PyQt6) — 퀴즈3·4 UI에 YOLO 판별을 더한 버전

통신 3개
  1) Modbus 서버  (이 PC, server.py, 5020)   : 작업 횟수 · 로봇 좌표 복사 · YOLO 결과 40~49 기록
  2) 로봇 소켓    (로봇 DRL, 9000)           : 작업 명령 보내기 + 로봇 로그 받기
  3) 로봇 Modbus  (로봇 컨트롤러, 502)       : 로봇 현재 좌표 읽기 (TCP 400~405, 관절 270~275)

작업
  작업 1     TASK,1  -> DONE,1   퀴즈1 Pick & Place
  랜덤 R1    TASK,R1 -> DONE,3   퀴즈4 (Vision 파랑 / Gesture)
  A 이동     TASK,A  -> DONE,4   YOLO 판정 A: 작업물을 왼쪽으로
  B 이동     TASK,B  -> DONE,5   YOLO 판정 B: 작업물을 오른쪽으로
  (퀴즈2 작업2는 구현하지 않음)

실행 (kw2026 activate 후, 저장소 루트에서)
  python quizzes\\quiz5\\server.py
  python quizzes\\quiz5\\quiz5_gui.py
"""

import datetime
import sys
import threading
import time
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

QUIZ5_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = QUIZ5_DIR.parents[1]
for p in (QUIZ5_DIR, PROJECT_ROOT / "src", PROJECT_ROOT / "ui"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

import register_map as rm  # noqa: E402
import yolo_judge as yj  # noqa: E402
from quiz3_gui import GestureThread, VisionThread  # noqa: E402  (퀴즈4 카메라 스레드 재사용)

from vision.gesture_logic import GESTURE_TO_TASK, AutoTaskGate  # noqa: E402

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
ROBOT_REG_JOINT = 270           # J1~J6, 0.1도, signed
ROBOT_REG_TCP = 400             # X,Y,Z [0.1mm], A,B,C [0.1도], base 기준, signed

POLL_MS = 500                   # 서버 레지스터 조회 주기
TASK_IDLE_TIMEOUT_MS = 60_000   # 작업 중 로봇 로그가 이 시간 동안 안 오면 '응답 없음'
VISION_CONFIRM_FRAMES = 5
GESTURE_HOLD_SEC = 1.0
GESTURE_RELEASE_SEC = 1.0

# 작업 번호 -> (이름, 로봇 명령). 완료 횟수는 서버 주소 (번호 - 1)
TASKS = {
    1: ("작업 1", "1"),
    3: ("랜덤 R1", "R1"),
    4: ("A 이동(왼쪽)", "A"),
    5: ("B 이동(오른쪽)", "B"),
}
VISION_TO_TASK = {"Red": 1, "Blue": 3}          # 작업2가 없어져서 파랑 = 랜덤 R1
GESTURE_REMAP = {1: 1, 2: 3, 3: 3}               # ONE=작업1, TWO/MULTI=랜덤 R1

ROBOT_STATE_TEXT = {
    0: "BACKDRIVE HOLD", 1: "BACKDRIVE RELEASE", 3: "SAFE OFF", 4: "INITIALIZING",
    5: "INTERRUPTED", 6: "EMERGENCY STOP", 8: "RECOVERY STANDBY", 11: "MANUAL STANDBY",
    12: "MANUAL JOGGING", 15: "STANDALONE STANDBY", 16: "STANDALONE RUNNING",
    17: "COLLABORATIVE STANDBY", 18: "COLLABORATIVE RUNNING",
}

STATE_IDLE, STATE_BUSY, STATE_DONE, STATE_ERROR = 0, 1, 2, 3
STATE_TEXT = {STATE_IDLE: "대기", STATE_BUSY: "작업 중", STATE_DONE: "완료", STATE_ERROR: "오류"}
STATE_COLOR = {STATE_IDLE: "#6b7280", STATE_BUSY: "#d97706",
               STATE_DONE: "#16a34a", STATE_ERROR: "#dc2626"}


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


# ========================= 로봇 좌표 (로봇 Modbus 502) =========================
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
                except Exception as e:   # 연결 끊김·응답 없음 모두 재시도
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
class RobotUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("협동로봇 작업 UI — 퀴즈5")
        self.resize(1280, 960)

        self.mb: ModbusTcpClient | None = None
        self.mb_ok = False
        self.robot = QTcpSocket(self)
        self.robot_ready = False
        self.robot_unsynced = False     # 응답 없음으로 끝난 뒤 로봇 상태를 모르는 상태
        self.rx_buf = b""
        self.busy = False
        self.current_task = 0

        self.robot_mb_thread: RobotModbusThread | None = None
        self.prev_tcp = self.prev_joint = None
        self.base_tcp = self.base_joint = None
        self.last_mirror = None

        self.vision_thread: VisionThread | None = None
        self.vision_detection = "None"
        self.vision_frame_count = 0
        self.vision_blocked_color = None
        self.vision_release_count = 0
        self.gesture_thread: GestureThread | None = None
        self.gesture_gate = AutoTaskGate(GESTURE_HOLD_SEC, GESTURE_RELEASE_SEC)
        self.gesture_last_label = "init"

        self.yolo_thread: YoloThread | None = None
        self.yolo_cache = {}
        self.vision_pixmap = None
        self.yolo_pixmap = None

        self._build_ui()
        self._connect_signals()

        self.poll_timer = QTimer(self)
        self.poll_timer.timeout.connect(self.poll_registers)
        self.task_timer = QTimer(self)
        self.task_timer.setSingleShot(True)
        self.task_timer.timeout.connect(self.on_task_idle_timeout)

        self.set_task_state(STATE_IDLE)
        self.update_buttons()

    # ---------------- UI 구성 ----------------
    def _build_ui(self):
        # 1) 통신 연결
        conn = QGroupBox("통신 연결")
        g = QGridLayout(conn)

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
            0, "Modbus 서버 (PC)", DEFAULT_MB_IP, DEFAULT_MB_PORT)
        self.rb_ip, self.rb_port, self.rb_btn, self.rb_status = row(
            1, "로봇 소켓 (명령·로그)", DEFAULT_ROBOT_IP, ROBOT_SOCK_PORT)
        self.rm_ip, self.rm_port, self.rm_btn, self.rm_status = row(
            2, "로봇 Modbus (좌표)", DEFAULT_ROBOT_IP, ROBOT_MB_PORT)

        # 2) 작업 실행
        task = QGroupBox("작업 실행")
        h = QHBoxLayout(task)
        self.btn_task = {n: QPushButton(name) for n, (name, _) in TASKS.items()}
        self.btn_vision = QPushButton("Vision 시작")
        self.btn_gesture = QPushButton("Gesture 시작")
        self.btn_gesture.setToolTip("검지 1개 → 작업 1, 검지+중지 / 손 2개 → 랜덤 R1")
        for b in [*self.btn_task.values(), self.btn_vision, self.btn_gesture]:
            b.setMinimumHeight(44)
            h.addWidget(b)
        self.task_state = QLabel()
        self.task_state.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.task_state.setMinimumWidth(140)
        h.addWidget(QLabel("상태:"))
        h.addWidget(self.task_state)

        # 3) 로봇 위치 (로봇 Modbus) + 카메라
        pos = QGroupBox("로봇 위치 (로봇 Modbus 502 읽기)")
        pv = QVBoxLayout(pos)
        self.tcp_table = self._make_pos_table(rm.TCP_NAMES, "TCP [mm / °]")
        self.joint_table = self._make_pos_table([f"J{i + 1}" for i in range(6)], "관절 [°]")
        self.robot_state_label = QLabel("로봇 상태: -")
        pv.addWidget(self.tcp_table)
        pv.addWidget(self.joint_table)
        pv.addWidget(self.robot_state_label)

        cam = QGroupBox("실시간 Vision / Gesture")
        cv_ = QVBoxLayout(cam)
        self.vision_view = QLabel("Vision 또는 Gesture를 시작하면 카메라 영상이 표시됩니다.")
        self.vision_view.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.vision_view.setMinimumSize(320, 220)
        self.vision_view.setStyleSheet("background:#111827; color:#d1d5db;")
        self.vision_status = QLabel("정지")
        self.vision_status.setAlignment(Qt.AlignmentFlag.AlignCenter)
        cv_.addWidget(self.vision_view, 1)
        cv_.addWidget(self.vision_status)

        monitor = QHBoxLayout()
        monitor.addWidget(pos, 3)
        monitor.addWidget(cam, 2)

        # 4) YOLO 판별
        yolo = QGroupBox("YOLO 판별 (평균 신뢰도 ≥ 기준 → A 왼쪽, 아니면 B 오른쪽)")
        yg = QGridLayout(yolo)
        self.yolo_root = QLineEdit(str(yj.DEFAULT_ROOT))
        self.btn_yolo_root = QPushButton("폴더…")
        self.yolo_dataset = QComboBox()
        self.yolo_dataset.addItems(yj.DATASETS)
        self.yolo_dataset.setCurrentText("valid")
        self.yolo_n = QSpinBox()
        self.yolo_n.setRange(1, 999)
        self.yolo_n.setValue(20)
        self.yolo_threshold = QDoubleSpinBox()
        self.yolo_threshold.setRange(0, 100)
        self.yolo_threshold.setDecimals(1)
        self.yolo_threshold.setSuffix(" %")
        self.yolo_threshold.setValue(80)
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
        self.yolo_auto = QCheckBox("판별 후 로봇 자동 이동")
        self.yolo_auto.setChecked(True)
        self.btn_yolo = QPushButton("YOLO 판별 실행")
        self.btn_yolo.setMinimumHeight(40)
        self.yolo_result = QLabel("결과: -")
        self.yolo_result.setStyleSheet("font: bold 14px;")

        yg.addWidget(QLabel("Yolo 폴더"), 0, 0)
        yg.addWidget(self.yolo_root, 0, 1, 1, 7)
        yg.addWidget(self.btn_yolo_root, 0, 8)
        for c, (name, w) in enumerate([("사진 묶음", self.yolo_dataset), ("N장", self.yolo_n),
                                       ("기준 M", self.yolo_threshold), ("CONF", self.yolo_conf)]):
            yg.addWidget(QLabel(name), 1, c * 2)
            yg.addWidget(w, 1, c * 2 + 1)
        yg.addWidget(QLabel("IMGSZ"), 2, 0)
        yg.addWidget(self.yolo_imgsz, 2, 1)
        yg.addWidget(QLabel("PREPROCESS"), 2, 2)
        yg.addWidget(self.yolo_prep, 2, 3)
        yg.addWidget(self.yolo_auto, 2, 4, 1, 2)
        yg.addWidget(self.btn_yolo, 1, 8, 2, 1)
        yg.addWidget(self.yolo_result, 3, 0, 1, 9)

        # 5) 탭: 레지스터 / YOLO 사진별 결과
        self.reg_table = QTableWidget(rm.REGISTER_COUNT, 4)
        self.reg_table.setHorizontalHeaderLabels(["주소", "항목", "Raw", "값"])
        self.reg_table.verticalHeader().setVisible(False)
        self.reg_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.reg_table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        for i in range(rm.REGISTER_COUNT):
            for c, text in enumerate([str(i), rm.label(i), "-", "-"]):
                self.reg_table.setItem(i, c, QTableWidgetItem(text))

        yolo_tab = QWidget()
        yt = QHBoxLayout(yolo_tab)
        self.yolo_table = QTableWidget(0, 3)
        self.yolo_table.setHorizontalHeaderLabels(["사진", "개 수", "신뢰도"])
        self.yolo_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.yolo_table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.yolo_view = QLabel("판별하면 결과 사진이 표시됩니다.")
        self.yolo_view.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.yolo_view.setMinimumSize(320, 200)
        self.yolo_view.setStyleSheet("background:#111827; color:#d1d5db;")
        yt.addWidget(self.yolo_table, 1)
        yt.addWidget(self.yolo_view, 1)

        tabs = QTabWidget()
        tabs.addTab(self.reg_table, "레지스터 모니터")
        tabs.addTab(yolo_tab, "YOLO 사진별 결과")

        # 6) 로그 (로봇 소켓 수신 포함)
        log_box = QGroupBox("작업 로그 (로봇 로그 = 소켓 수신)")
        lv = QVBoxLayout(log_box)
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.btn_log_save = QPushButton("로그 저장")
        lv.addWidget(self.log_view)
        lv.addWidget(self.btn_log_save, alignment=Qt.AlignmentFlag.AlignRight)

        root = QVBoxLayout(self)
        root.addWidget(conn)
        root.addWidget(task)
        root.addLayout(monitor, 3)
        root.addWidget(yolo)
        root.addWidget(tabs, 3)
        root.addWidget(log_box, 2)

    def _make_pos_table(self, cols, unit):
        t = QTableWidget(2, len(cols))
        t.setHorizontalHeaderLabels(cols)
        t.setVerticalHeaderLabels([unit, "변화량"])
        t.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        t.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        t.setSelectionMode(QTableWidget.SelectionMode.NoSelection)
        t.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        big = QFont("Consolas", 13)
        big.setBold(True)
        for r in range(2):
            for c in range(len(cols)):
                it = QTableWidgetItem("-")
                it.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                if r == 0:
                    it.setFont(big)
                t.setItem(r, c, it)
        t.setFixedHeight(2 * 34 + 30)
        return t

    def _update_pos_table(self, table, values, prev, base):
        for c, val in enumerate(values):
            cur = table.item(0, c)
            cur.setText(f"{val:.1f}")
            if prev is not None and abs(val - prev[c]) >= 0.05:      # 직전 대비 바뀐 칸 강조
                cur.setBackground(QBrush(QColor("#fde68a")))
                cur.setForeground(QBrush(QColor("#111827")))
            else:
                cur.setData(Qt.ItemDataRole.BackgroundRole, None)
                cur.setData(Qt.ItemDataRole.ForegroundRole, None)
            d_item = table.item(1, c)
            d_item.setText("-" if base is None else f"{val - base[c]:+.1f}")

    def _connect_signals(self):
        self.mb_btn.clicked.connect(self.toggle_modbus)
        self.rb_btn.clicked.connect(self.toggle_robot)
        self.rm_btn.clicked.connect(self.toggle_robot_modbus)
        for n, b in self.btn_task.items():
            b.clicked.connect(lambda _=False, n=n: self.start_task(n))
        self.btn_vision.clicked.connect(self.toggle_vision)
        self.btn_gesture.clicked.connect(self.toggle_gesture)
        self.btn_yolo.clicked.connect(self.start_yolo)
        self.btn_yolo_root.clicked.connect(self.pick_yolo_root)
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
        self.task_state.setStyleSheet(f"background:{STATE_COLOR[state]}; color:white; "
                                      "font: bold 15px; padding:6px; border-radius:6px;")

    def robot_link_ok(self):
        sock_ok = self.robot.state() == QAbstractSocket.SocketState.ConnectedState
        return self.mb_ok and sock_ok and self.robot_ready and not self.robot_unsynced

    def update_buttons(self):
        camera_on = self.vision_thread is not None or self.gesture_thread is not None
        enable = self.robot_link_ok() and not self.busy and not camera_on
        for b in self.btn_task.values():
            b.setEnabled(enable)
        self.btn_vision.setEnabled(self.gesture_thread is None)
        self.btn_gesture.setEnabled(self.vision_thread is None)
        self.btn_yolo.setEnabled(self.yolo_thread is None)
        self.mb_btn.setEnabled(not self.busy)
        self.rb_btn.setEnabled(not self.busy)

    # ---------------- Modbus 서버 ----------------
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
                self.poll_registers()
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

    def increment_task_count(self, n):
        address = rm.COUNT_START + n - 1
        regs = self.read_regs(address, 1)
        if regs is None:
            return
        if regs[0] >= 0xFFFF:
            self.log(f"{TASKS[n][0]} 완료 횟수가 최대치입니다.")
            return
        self.write_regs(address, [regs[0] + 1])

    def poll_registers(self):
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
        for i, raw in enumerate(rr.registers):
            self.reg_table.item(i, 2).setText(str(raw))
            self.reg_table.item(i, 3).setText(rm.describe(i, raw))

    # ---------------- 로봇 Modbus (좌표) ----------------
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
            self.log("로봇 Modbus 연결됨 (좌표 수신 시작)")
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

    def on_robot_pos(self, tcp, joint, state):
        self._update_pos_table(self.tcp_table, tcp, self.prev_tcp, self.base_tcp)
        self._update_pos_table(self.joint_table, joint, self.prev_joint, self.base_joint)
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
        if self.busy:
            self.finish_task(False, "작업 중 로봇 연결 끊김")
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
        self.log(f"[로봇] {line}")          # 로봇 로그는 모두 소켓으로 받아서 그대로 남긴다
        cmd, _, body = line.partition(",")
        cmd = cmd.strip().upper()

        if self.busy:
            self.task_timer.start(TASK_IDLE_TIMEOUT_MS)     # 로그가 오는 동안은 타임아웃 아님
        elif self.robot_unsynced and (cmd in ("DONE", "READY") or cmd.startswith("UNKNOWN")):
            self.robot_unsynced = False
            self.log("로봇 작업 종료 확인 → 다시 작업 실행 가능")
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
            if not self.busy:
                return
            if n != self.current_task:
                self.log(f"DONE 작업 번호 불일치 (보낸 작업 {self.current_task}, 받은 {n})")
                return
            self.finish_task(True)
        elif cmd == "ERROR":
            self.finish_task(False, body or "로봇 오류")
        elif cmd.startswith("UNKNOWN"):
            self.finish_task(False, f"로봇이 모르는 명령 ({line}) — 펜던트 DRL 버전 확인")

    # ---------------- 작업 실행 ----------------
    def start_task(self, n):
        if self.busy:
            self.log(f"{TASKS[n][0]} 실행 안 함: 다른 작업 중")
            return False
        if not self.robot_link_ok():
            self.log(f"{TASKS[n][0]} 실행 안 함: Modbus 서버/로봇 소켓 연결 또는 READY 미수신"
                     + (" (이전 작업 응답 없음)" if self.robot_unsynced else ""))
            return False
        self.busy = True
        self.current_task = n
        self.base_tcp, self.base_joint = self.prev_tcp, self.prev_joint
        self.set_task_state(STATE_BUSY)
        self.update_buttons()
        command = TASKS[n][1]
        self.robot.write(f"TASK,{command}\n".encode())
        self.robot.flush()
        self.task_timer.start(TASK_IDLE_TIMEOUT_MS)
        self.log(f"{TASKS[n][0]} 시작 (명령: TASK,{command})")
        return True

    def finish_task(self, ok, reason=""):
        if not self.busy:
            return
        self.task_timer.stop()
        if ok:
            self.increment_task_count(self.current_task)
            self.poll_registers()
        self.set_task_state(STATE_DONE if ok else STATE_ERROR)
        self.log(f"{TASKS[self.current_task][0]} {'완료' if ok else '오류: ' + reason}")
        self.busy = False
        self.update_buttons()

    def on_task_idle_timeout(self):
        if not self.busy:
            return
        self.robot_unsynced = True      # 로봇은 아직 움직이는 중일 수 있음 → 새 명령 금지
        self.finish_task(False, f"로봇 응답 없음 ({TASK_IDLE_TIMEOUT_MS // 1000}초)")
        if self.vision_thread is not None:
            self.vision_thread.stop()
        if self.gesture_thread is not None:
            self.gesture_thread.stop()
        self.log("로봇이 DONE을 보내거나 재연결할 때까지 새 작업을 보내지 않습니다. 펜던트 확인.")
        self.update_buttons()

    # ---------------- YOLO ----------------
    def pick_yolo_root(self):
        path = QFileDialog.getExistingDirectory(self, "Yolo 폴더 선택", self.yolo_root.text())
        if path:
            self.yolo_root.setText(path)

    def start_yolo(self):
        if self.yolo_thread is not None:
            return
        root = Path(self.yolo_root.text().strip())
        folder = root / "input_images" / self.yolo_dataset.currentText()
        photos = yj.list_photos(folder)
        n = self.yolo_n.value()
        if len(photos) < n:
            self.log(f"YOLO: 사진이 부족합니다 ({folder}에 {len(photos)}장, 필요 {n}장)")
            return
        conf, imgsz = self.yolo_conf.value(), int(self.yolo_imgsz.currentText())
        mode, threshold = self.yolo_prep.currentText(), self.yolo_threshold.value()
        self.log(f"YOLO 판별 시작: {folder.name} {n}장, CONF={conf:.2f}, IMGSZ={imgsz}, "
                 f"PREPROCESS={mode}, 기준={threshold:g}% (첫 실행은 모델 로딩으로 수 초)")
        self.yolo_table.setRowCount(0)
        self.yolo_result.setText("결과: 판별 중…")
        self.yolo_thread = YoloThread(self.yolo_cache, root, photos[:n], folder,
                                      threshold, conf, imgsz, mode, self)
        self.yolo_thread.progress.connect(self.on_yolo_progress)
        self.yolo_thread.done.connect(self.on_yolo_done)
        self.yolo_thread.failed.connect(lambda m: self.log(f"YOLO 오류: {m}"))
        self.yolo_thread.finished.connect(self.on_yolo_finished)
        self.yolo_thread.start()
        self.update_buttons()

    def on_yolo_progress(self, i, total, name, dogs, image):
        self.yolo_result.setText(f"결과: 판별 중… {i}/{total}")
        self.yolo_pixmap = QPixmap.fromImage(image)
        self._show_pixmap(self.yolo_view, self.yolo_pixmap)
        self.log(f"YOLO [{i}/{total}] {name} | 검출 {dogs}마리")

    def on_yolo_done(self, r):
        for item in r.per_photo:
            row = self.yolo_table.rowCount()
            self.yolo_table.insertRow(row)
            scores = ", ".join(f"{s:.2f}" for s in item.scores) or "-"
            for c, text in enumerate([item.name, str(len(item.scores)), scores]):
                self.yolo_table.setItem(row, c, QTableWidgetItem(text))
        color = "#2563eb" if r.decision == "A" else "#dc2626"
        self.yolo_result.setText(r.summary())
        self.yolo_result.setStyleSheet(f"font: bold 14px; color:{color};")
        self.log(f"YOLO 판별 완료: {r.summary()}")
        self.log(f"YOLO 결과 사진 저장: {r.out_dir}")

        # Modbus 서버 40~49번 기록 (판별 횟수 = 서버 값 + 1)
        prev = self.read_regs(rm.YOLO_RUN_COUNT, 1)
        if prev is None:
            self.log("YOLO 결과를 Modbus 서버에 쓰지 못함 (서버 미연결)")
        else:
            values = rm.yolo_registers(r.photos, r.photos_with_dog, r.dogs, r.mean_percent,
                                       r.threshold_percent, r.decision, prev[0] + 1,
                                       r.conf, r.imgsz, r.preprocess)
            if self.write_regs(rm.YOLO_PHOTOS, values):
                self.log(f"YOLO 결과 → Modbus 주소 {rm.YOLO_PHOTOS}~{rm.YOLO_PREPROCESS} 기록")
                self.poll_registers()

        if self.yolo_auto.isChecked():
            n = 4 if r.decision == "A" else 5
            self.log(f"판정 {r.decision} → {TASKS[n][0]} 실행")
            self.start_task(n)

    def on_yolo_finished(self):
        self.yolo_thread = None
        if self.yolo_result.text().startswith("결과: 판별 중"):
            self.yolo_result.setText("결과: 실패 (로그 확인)")
        self.update_buttons()

    # ---------------- Vision (퀴즈4 HSV) ----------------
    def toggle_vision(self):
        if self.vision_thread is not None:
            self.vision_thread.stop()
            return
        self.vision_detection, self.vision_frame_count = "None", 0
        self.vision_blocked_color, self.vision_release_count = None, 0
        self.log("Vision 실행: 빨강 → 작업 1, 파랑 → 랜덤 R1")
        self.vision_status.setText("카메라 준비 중…")
        self.btn_vision.setText("Vision 정지")
        self.vision_thread = VisionThread(self)
        self.vision_thread.frame_ready.connect(self.on_camera_frame)
        self.vision_thread.detection_ready.connect(self.handle_vision_detection)
        self.vision_thread.vision_error.connect(lambda m: self.log(f"Vision 오류: {m}"))
        self.vision_thread.finished.connect(self.on_vision_finished)
        self.vision_thread.start()
        self.update_buttons()

    def handle_vision_detection(self, value):
        self.vision_status.setText(f"Vision · 검출: {value}")
        # 직전 작업을 일으킨 색은 5프레임 연속 사라져야 다시 인정 (같은 물체로 반복 실행 방지)
        if self.vision_blocked_color is not None:
            if value == self.vision_blocked_color:
                self.vision_release_count = 0
                return
            self.vision_release_count += 1
            if self.vision_release_count < VISION_CONFIRM_FRAMES:
                return
            self.log(f"{self.vision_blocked_color} 제거 확인: 재검출 허용")
            self.vision_blocked_color = None

        if self.busy or value not in VISION_TO_TASK:
            self.vision_detection, self.vision_frame_count = "None", 0
            return
        if value == self.vision_detection:
            self.vision_frame_count += 1
        else:
            self.vision_detection, self.vision_frame_count = value, 1
        if self.vision_frame_count >= VISION_CONFIRM_FRAMES:
            n = VISION_TO_TASK[value]
            self.vision_blocked_color, self.vision_release_count = value, 0
            self.vision_detection, self.vision_frame_count = "None", 0
            self.log(f"Vision {value} 확정 → {TASKS[n][0]}")
            self.start_task(n)

    def on_vision_finished(self):
        self.vision_thread = None
        self.btn_vision.setText("Vision 시작")
        self.vision_status.setText("정지")
        self.vision_view.clear()
        self.vision_view.setText("Vision 또는 Gesture를 시작하면 카메라 영상이 표시됩니다.")
        self.log("Vision 종료")
        self.update_buttons()

    # ---------------- Gesture (퀴즈4 MediaPipe) ----------------
    def toggle_gesture(self):
        if self.gesture_thread is not None:
            self.gesture_thread.stop()
            return
        self.gesture_gate.reset()
        self.gesture_last_label = "init"
        self.log("Gesture 실행: 검지 → 작업 1, 검지+중지 / 손 2개 → 랜덤 R1")
        self.vision_status.setText("Gesture 준비 중…")
        self.btn_gesture.setText("Gesture 정지")
        self.gesture_thread = GestureThread(self)
        self.gesture_thread.frame_ready.connect(self.on_camera_frame)
        self.gesture_thread.label_ready.connect(self.handle_gesture_label)
        self.gesture_thread.gesture_error.connect(lambda m: self.log(f"Gesture 오류: {m}"))
        self.gesture_thread.finished.connect(self.on_gesture_finished)
        self.gesture_thread.start()
        self.update_buttons()

    def handle_gesture_label(self, label):
        now = time.monotonic()
        gate_task = self.gesture_gate.update(label, now, self.busy)
        if self.gesture_thread is not None:
            if self.busy:
                self.gesture_thread.status_text = "작업 중"
            elif label in GESTURE_TO_TASK and not self.gesture_gate.locked:
                self.gesture_thread.status_text = f"유지 {self.gesture_gate.progress(now):.0%}"
            else:
                self.gesture_thread.status_text = ""
        self.vision_status.setText(f"Gesture · {label or '손 없음'}")
        if label != self.gesture_last_label:
            self.log(f"Gesture 판정: {label or '손 없음'}")
            self.gesture_last_label = label
        if gate_task is not None:
            n = GESTURE_REMAP[gate_task]
            self.log(f"{label} 제스처 확정 → {TASKS[n][0]}")
            self.start_task(n)

    def on_gesture_finished(self):
        self.gesture_thread = None
        self.gesture_gate.reset()
        self.btn_gesture.setText("Gesture 시작")
        self.vision_status.setText("정지")
        self.vision_view.clear()
        self.vision_view.setText("Vision 또는 Gesture를 시작하면 카메라 영상이 표시됩니다.")
        self.log("Gesture 종료")
        self.update_buttons()

    # ---------------- 화면 ----------------
    def on_camera_frame(self, image):
        self.vision_pixmap = QPixmap.fromImage(image)
        self._show_pixmap(self.vision_view, self.vision_pixmap)

    @staticmethod
    def _show_pixmap(view, pixmap):
        if pixmap is not None:
            view.setPixmap(pixmap.scaled(view.size(), Qt.AspectRatioMode.KeepAspectRatio,
                                         Qt.TransformationMode.SmoothTransformation))

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._show_pixmap(self.vision_view, self.vision_pixmap)
        self._show_pixmap(self.yolo_view, self.yolo_pixmap)

    def save_log(self):
        path, _ = QFileDialog.getSaveFileName(self, "로그 저장", "quiz5_log.txt", "Text (*.txt)")
        if path:
            Path(path).write_text(self.log_view.toPlainText(), encoding="utf-8")

    def closeEvent(self, e):
        self.poll_timer.stop()
        threads = [self.vision_thread, self.gesture_thread, self.robot_mb_thread,
                   self.yolo_thread]
        for t in threads:
            if t is not None and hasattr(t, "stop"):
                t.stop()
        for t in threads:
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
    w = RobotUI()
    w.show()
    sys.exit(app.exec())
