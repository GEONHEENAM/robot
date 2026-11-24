"""
최종 프로젝트 GUI (2단계: 골격)  ─ 퀴즈3 quiz3_gui.py 기반

  - 카메라 화면 + 막대 검출 표시 (vision.py, 별도 스레드)
  - [HOME 후 스냅샷] → 막대 선택 → 놓을 칸 선택 → PICK 전송 (수동 조작)
    PICK이 끝나면 DRL이 홈으로 돌아가 AT_HOME을 보내고, GUI는 그 뒤 프레임으로 자동 스냅샷 (4단계)
  - 로봇 TCP 클라이언트 (퀴즈3과 동일: DRL이 서버, 포트 9000)
  - Modbus TCP 클라이언트 (선택: 퀴즈3 server.py에 작업 횟수 기록)
  - 음성/텍스트 명령 (5~6단계): 문장 → 의도(intent.py) → 실행 계획(planner.py) → 승인 → 실행
    Space를 누르고 있는 동안 녹음 → 떼면 Vosk 인식(stt.py). 텍스트 입력란도 같은 경로.
    정렬은 PICK을 여러 번 보내며, 매번 새 스냅샷에서 다음 막대를 다시 찾는다.

로봇 없이 테스트
  1) python fake_robot.py
  2) python final_gui.py → 로봇 IP 옆 [로컬] 버튼 → [연결]

[로봇 통신 프로토콜]  (퀴즈3 + 추가분)
  GUI → 로봇 : "TASK,1" / "TASK,2"         퀴즈1·퀴즈2 작업 (기존)
               "HOME"                      홈 자세로 이동          (추가)
               "PICK,x,y,angle,slot"       막대 집기 → slot 칸에 놓기 (추가)
                                           x, y = 로봇 베이스 좌표(mm), angle = 막대 긴 축 각도(deg, -90~90)
                                           slot = 놓기 칸 번호 1~3 (config.PLACE_SLOTS)
               "END"                       DRL 종료 (기존)
  로봇 → GUI : "READY"                     접속 직후 (기존)
               "POS,x,y,z,j1..j6"          현재 위치 (기존)
               "DONE,n"                    작업 완료: 1=작업1, 2=작업2, 3=PICK (3은 추가)
               "AT_HOME"                   홈 도착 = 카메라 촬영해도 되는 시점 (추가)
                                           HOME 명령 끝, PICK 끝(DONE,3 직전), PICK 실패(ERROR 직후)에 온다
               "ERROR,..."                 작업 실패 (기존)
               "DATA,..." / "ORDER,..."    작업2 측정값 (기존)
"""

import csv
import datetime
import sys
import time

import cv2
from PyQt6.QtCore import QEvent, Qt, QThread, QTimer, pyqtSignal
from PyQt6.QtGui import QBrush, QColor, QFont, QImage, QPixmap
from PyQt6.QtNetwork import QAbstractSocket, QTcpSocket
from PyQt6.QtWidgets import (
    QAbstractItemView,
    QAbstractSpinBox,
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
    QSizePolicy,
    QSpinBox,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

import config
import stt
from intent import is_cancel, is_confirm, parse
from planner import make_plan, rematch
from vision import (
    StickDetector,
    Undistorter,
    VisionParams,
    draw_detections,
    load_mapper,
    open_camera,
)

try:
    from pymodbus.client import ModbusTcpClient
except ImportError:          # Modbus는 선택 기능이라 없어도 GUI는 동작
    ModbusTcpClient = None

# ========================= 설정 =========================
UNIT_ID = 1
HR_START = 0
HR_COUNT = 20
POLL_MS = 500

# 명령 종류 → (Modbus 작업 번호 = DONE 번호, 타임아웃 ms)
COMMANDS = {
    "TASK1": (1, 60_000),
    "TASK2": (2, 90_000),
    "PICK": (3, 60_000),
    "HOME": (None, 30_000),   # HOME은 DONE 대신 AT_HOME으로 완료
}

REG_NAMES = {0: "작업1 완료 횟수 (퀴즈1)", 1: "작업2 완료 횟수 (퀴즈2)",
             2: "작업3 완료 횟수 (PICK)", 3: "작업4 완료 횟수 (정렬, 예정)"}
for _i in range(4, 10):
    REG_NAMES[_i] = f"작업{_i + 1} 완료 횟수 (예비)"
for _i in range(10, 20):
    REG_NAMES[_i] = f"측정값 #{_i - 9} (x10)"

SNAP_SETTLE_S = 0.5        # AT_HOME 후 이만큼 지난 프레임으로 스냅샷 (진동·영상 지연 여유)
SNAP_PENDING_TIMEOUT_S = 120

SORT_TASK_NO = 4           # 정렬 전체가 끝나면 Modbus 작업4 횟수 +1
SNAP_MAX_AGE_S = 30        # 명령을 받았을 때 이보다 오래된 스냅샷이면 HOME 후 새로 찍는다

STATE_IDLE, STATE_BUSY, STATE_DONE, STATE_ERROR = 0, 1, 2, 3
STATE_LISTEN, STATE_THINK, STATE_CONFIRM = 4, 5, 6
STATE_TEXT = {STATE_IDLE: "대기", STATE_BUSY: "작업 중", STATE_DONE: "완료", STATE_ERROR: "오류",
              STATE_LISTEN: "듣는 중", STATE_THINK: "인식 중", STATE_CONFIRM: "승인 대기"}
STATE_COLOR = {STATE_IDLE: "#6b7280", STATE_BUSY: "#d97706",
               STATE_DONE: "#16a34a", STATE_ERROR: "#dc2626",
               STATE_LISTEN: "#2563eb", STATE_THINK: "#7c3aed", STATE_CONFIRM: "#0891b2"}

LOCAL_HOSTS = {"127.0.0.1", "localhost", "::1"}


def mb_call(fn, *args, **kw):
    """pymodbus 버전별 인자명 차이(device_id / slave) 흡수 (퀴즈3 그대로)"""
    try:
        return fn(*args, device_id=UNIT_ID, **kw)
    except TypeError:
        return fn(*args, slave=UNIT_ID, **kw)


# ========================= 카메라 스레드 =========================
class CameraWorker(QThread):
    """프레임 읽기 + 막대 검출을 GUI 스레드 밖에서 수행한다."""
    frame_ready = pyqtSignal(object, object, object, float, float)  # frame, sticks, mask, 시각, fps
    status = pyqtSignal(str)
    mapper_ready = pyqtSignal(object)
    failed = pyqtSignal(str)

    def __init__(self, index, params, parent=None):
        super().__init__(parent)
        self.index = index
        self.detector = StickDetector(params)
        self.undistort = Undistorter()
        self.mapper = None
        self._running = False

    def set_params(self, params):
        self.detector.params = params      # 참조 교체만 하므로 스레드 간 공유해도 안전

    def stop(self):
        self._running = False
        self.wait(3000)

    def run(self):
        cap, msgs = open_camera(self.index)
        for m in msgs:
            self.status.emit(m)
        if cap is None:
            self.failed.emit(f"카메라 {self.index}를 열 수 없음")
            return
        if self.undistort.enabled:
            self.status.emit("렌즈 왜곡 보정 적용")

        self._running = True
        fails, fps, t_prev = 0, 0.0, time.time()
        try:
            while self._running:
                ok, frame = cap.read()
                if not ok:
                    fails += 1
                    if fails >= 30:
                        self.failed.emit("프레임을 연속으로 읽지 못함 (카메라 연결 확인)")
                        break
                    self.msleep(30)
                    continue
                fails = 0
                ts = time.time()
                frame = self.undistort(frame)
                if self.mapper is None:
                    self.mapper = load_mapper(frame.shape[1], frame.shape[0])
                    self.mapper_ready.emit(self.mapper)
                sticks, mask = self.detector.detect(frame, self.mapper)
                dt = ts - t_prev
                t_prev = ts
                if dt > 0:
                    fps = 0.9 * fps + 0.1 * (1.0 / dt)
                self.frame_ready.emit(frame, sticks, mask, ts, fps)
        finally:
            cap.release()


# ========================= STT 스레드 =========================
class SttLoadWorker(QThread):
    """Vosk 모델 로드는 몇 초 걸리므로 시작할 때 백그라운드에서 한 번만."""
    loaded = pyqtSignal(object, str)    # recognizer 또는 None, 메시지

    def run(self):
        try:
            r = stt.load_recognizer()
            self.loaded.emit(r, f"STT 준비됨 (모델 로드 {r.load_seconds:.1f}s, {r.mode})")
        except Exception as e:
            self.loaded.emit(None, f"STT 로드 실패: {e}")


class SttRecognizeWorker(QThread):
    done = pyqtSignal(dict)

    def __init__(self, recognizer, audio, parent=None):
        super().__init__(parent)
        self.recognizer = recognizer
        self.audio = audio

    def run(self):
        try:
            self.done.emit(self.recognizer.recognize(self.audio))
        except Exception as e:
            self.done.emit({"text": "", "error": str(e)})


# ========================= 메인 UI =========================
class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("협동로봇 비전·음성 집기 UI")
        self.resize(1500, 950)

        # --- 통신 상태 (퀴즈3과 동일) ---
        self.mb = None
        self.mb_ok = False
        self.robot = QTcpSocket(self)
        self.robot_ready = False
        self.rx_buf = ""
        self.busy = False
        self.current = None            # 진행 중 명령 {kind, task_no, line, target, t0}

        # --- 카메라 / 검출 ---
        self.params = VisionParams.load()
        self.cam_worker = None
        self.mapper = None
        self.live_sticks = []
        self.live_ts = 0.0
        self.snap_sticks = []           # 스냅샷 시점의 막대 목록 (선택·PICK은 이것 기준)
        self.snap_ts = 0.0
        self.selected = None            # 선택된 막대 (Stick)
        self.scene_changed_ts = 0.0     # 로봇이 막대를 움직였을 수 있는 마지막 시각
        self.at_home_ts = 0.0           # 마지막 AT_HOME 수신 시각
        self.robot_at_home = False      # AT_HOME 이후 다른 동작 명령을 보내지 않았으면 True
        self.pending_snap = False       # AT_HOME 뒤 자동 스냅샷 예약
        self.pending_since = 0.0

        # --- 음성/텍스트 명령 (5~6단계) ---
        self.pending_intent = None      # 새 스냅샷을 기다리는 의도
        self.plan = None                # 승인 대기 중인 실행 계획
        self.seq = None                 # 실행 중인 계획 {plan, i, sent_t0, last_ok, abort}
        self.recognizer = None
        self.recorder = None
        self.stt_worker = None
        self.stt_loader = None

        # --- 위치 표시 (퀴즈3과 동일) ---
        self.base_xyz = self.base_joint = None
        self.prev_xyz = self.prev_joint = None

        self._build_ui()
        self._connect_signals()

        self.poll_timer = QTimer(self)
        self.poll_timer.timeout.connect(self.poll_registers)
        self.task_timer = QTimer(self)
        self.task_timer.setSingleShot(True)
        self.task_timer.timeout.connect(lambda: self.finish_command(False, "응답 시간 초과"))

        self.set_task_state(STATE_IDLE)
        self.update_buttons()
        self.log("프로그램 시작. 카메라 시작 → 로봇 연결 → [HOME 후 스냅샷] → 막대 선택 → 놓을 칸 선택 → 집기")
        self.log("음성/텍스트 명령: Space를 누른 채 말하기 또는 입력란에 문장 → [승인]")

        QApplication.instance().installEventFilter(self)    # Space 누르고 말하기
        self.start_stt_loading()

    # =====================================================================
    # UI 구성
    # =====================================================================
    def _build_ui(self):
        # ---------------- 왼쪽: 카메라 ----------------
        cam_box = QGroupBox("카메라")
        cv = QVBoxLayout(cam_box)
        self.view = QLabel("카메라 정지")
        self.view.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.view.setMinimumSize(800, 450)
        self.view.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Ignored)
        self.view.setStyleSheet("background:#111; color:#aaa; font-size:16px;")

        cbar = QHBoxLayout()
        self.cam_index = QSpinBox()
        self.cam_index.setRange(0, 9)
        self.cam_index.setValue(config.CAMERA_INDEX)
        self.cam_btn = QPushButton("카메라 시작")
        self.view_mode = QComboBox()
        self.view_mode.addItems(["원본", "마스크"])
        self.btn_capture = QPushButton("프레임 저장")
        self.fps_label = QLabel("FPS -")
        self.calib_label = QLabel("좌표 변환: -")
        self.calib_label.setStyleSheet("font-weight:bold;")
        for w in (QLabel("카메라 번호"), self.cam_index, self.cam_btn,
                  QLabel("보기"), self.view_mode, self.btn_capture):
            cbar.addWidget(w)
        cbar.addStretch()
        cbar.addWidget(self.fps_label)
        cbar.addSpacing(12)
        cbar.addWidget(self.calib_label)
        cv.addWidget(self.view, 1)
        cv.addLayout(cbar)

        # ---------------- 오른쪽 1: 통신 연결 ----------------
        conn = QGroupBox("통신 연결")
        g = QGridLayout(conn)
        self.rb_ip = QLineEdit(config.ROBOT_IP)
        self.rb_port = QSpinBox()
        self.rb_port.setRange(1, 65535)
        self.rb_port.setValue(config.ROBOT_PORT)
        self.rb_local_btn = QPushButton("로컬")
        self.rb_local_btn.setToolTip("가짜 로봇(fake_robot.py) 테스트용으로 127.0.0.1 입력")
        self.rb_btn = QPushButton("연결")
        self.rb_status = QLabel()

        self.mb_ip = QLineEdit(config.MODBUS_IP)
        self.mb_port = QSpinBox()
        self.mb_port.setRange(1, 65535)
        self.mb_port.setValue(config.MODBUS_PORT)
        self.mb_btn = QPushButton("연결")
        self.mb_status = QLabel()
        if ModbusTcpClient is None:
            self.mb_btn.setEnabled(False)
            self.mb_btn.setToolTip("pymodbus 미설치")

        g.addWidget(QLabel("로봇"), 0, 0)
        g.addWidget(self.rb_ip, 0, 1)
        g.addWidget(self.rb_port, 0, 2)
        g.addWidget(self.rb_local_btn, 0, 3)
        g.addWidget(self.rb_btn, 0, 4)
        g.addWidget(self.rb_status, 0, 5)
        g.addWidget(QLabel("Modbus (선택)"), 1, 0)
        g.addWidget(self.mb_ip, 1, 1)
        g.addWidget(self.mb_port, 1, 2)
        g.addWidget(self.mb_btn, 1, 4)
        g.addWidget(self.mb_status, 1, 5)
        self.set_conn_label(self.rb_status, False)
        self.set_conn_label(self.mb_status, False)

        # ---------------- 오른쪽 2: 상태 ----------------
        state_box = QGroupBox("상태")
        sh = QHBoxLayout(state_box)
        self.task_state = QLabel()
        self.task_state.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.task_state.setMinimumWidth(120)
        self.cmd_label = QLabel("현재 명령: -")
        self.cmd_label.setWordWrap(True)
        sh.addWidget(self.task_state)
        sh.addWidget(self.cmd_label, 1)

        # ---------------- 오른쪽 3: 음성/텍스트 명령 (5~6단계) ----------------
        voice_box = QGroupBox("음성 / 텍스트 명령")
        vg = QGridLayout(voice_box)
        self.btn_listen = QPushButton("누르고 말하기 (Space)")
        self.btn_listen.setMinimumHeight(36)
        self.stt_status = QLabel("STT: -")
        self.text_cmd = QLineEdit()
        self.text_cmd.setPlaceholderText("예: 왼쪽 막대 오른쪽에 놔 / 길이 순서대로 정렬해 / 홈으로 가")
        self.btn_text_cmd = QPushButton("해석")
        self.lbl_heard = QLabel("인식: -")
        self.lbl_intent = QLabel("해석: -")
        self.lbl_plan = QLabel("계획: -")
        for lb in (self.lbl_heard, self.lbl_intent, self.lbl_plan):
            lb.setWordWrap(True)
        self.btn_confirm = QPushButton("승인 (실행)")
        self.btn_cancel = QPushButton("취소")
        self.btn_confirm.setStyleSheet("font-weight:bold;")
        vg.addWidget(self.btn_listen, 0, 0, 1, 2)
        vg.addWidget(self.stt_status, 0, 2, 1, 2)
        vg.addWidget(self.text_cmd, 1, 0, 1, 3)
        vg.addWidget(self.btn_text_cmd, 1, 3)
        vg.addWidget(self.lbl_heard, 2, 0, 1, 4)
        vg.addWidget(self.lbl_intent, 3, 0, 1, 4)
        vg.addWidget(self.lbl_plan, 4, 0, 1, 4)
        vg.addWidget(self.btn_confirm, 5, 0, 1, 2)
        vg.addWidget(self.btn_cancel, 5, 2, 1, 2)

        # ---------------- 오른쪽 4: 검출 목록 ----------------
        det_box = QGroupBox("검출 목록 (스냅샷 기준)")
        dv = QVBoxLayout(det_box)
        dh = QHBoxLayout()
        self.btn_home_snap = QPushButton("HOME 후 스냅샷")
        self.btn_home_snap.setToolTip("로봇을 홈으로 보낸 뒤 AT_HOME을 받으면 자동으로 스냅샷 (권장)")
        self.snap_btn = QPushButton("스냅샷")
        self.snap_btn.setToolTip("현재 프레임의 검출 결과를 바로 고정한다. 로봇이 시야를 가리지 않을 때만")
        self.auto_snap_chk = QCheckBox("PICK 후 자동 스냅샷")
        self.auto_snap_chk.setChecked(True)
        self.snap_info = QLabel("스냅샷 없음")
        dh.addWidget(self.btn_home_snap)
        dh.addWidget(self.snap_btn)
        dh.addWidget(self.auto_snap_chk)
        dv.addLayout(dh)
        dv.addWidget(self.snap_info)
        self.det_table = QTableWidget(0, 6)
        self.det_table.setHorizontalHeaderLabels(["#", "X (mm)", "Y (mm)", "각도 (°)", "길이 (mm)", "집기 영역"])
        self.det_table.verticalHeader().setVisible(False)
        self.det_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.det_table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.det_table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.det_table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        dv.addWidget(self.det_table)

        # ---------------- 오른쪽 5: 수동 조작 ----------------
        man_box = QGroupBox("수동 조작")
        mg = QGridLayout(man_box)
        self.dest_combo = QComboBox()
        for n, (sx, sy) in config.PLACE_SLOTS.items():
            self.dest_combo.addItem(f"P{n}  ({sx:.0f}, {sy:.0f})", n)
        self.btn_pick = QPushButton("선택한 막대 집기")
        self.btn_home = QPushButton("HOME")
        self.btn_task1 = QPushButton("작업 1 (퀴즈1)")
        self.btn_task2 = QPushButton("작업 2 (퀴즈2)")
        for b in (self.btn_pick, self.btn_home):
            b.setMinimumHeight(40)
        mg.addWidget(QLabel("놓을 칸"), 0, 0)
        mg.addWidget(self.dest_combo, 0, 1)
        mg.addWidget(self.btn_pick, 0, 2)
        mg.addWidget(self.btn_home, 1, 0)
        mg.addWidget(self.btn_task1, 1, 1)
        mg.addWidget(self.btn_task2, 1, 2)

        right = QWidget()
        rv = QVBoxLayout(right)
        rv.setContentsMargins(0, 0, 0, 0)
        rv.addWidget(conn)
        rv.addWidget(state_box)
        rv.addWidget(voice_box)
        rv.addWidget(det_box, 1)
        rv.addWidget(man_box)

        # ---------------- 아래: 탭 ----------------
        tabs = QTabWidget()

        # 로봇 위치 (퀴즈3)
        pos_w = QWidget()
        ph = QHBoxLayout(pos_w)
        tcp_box, self.tcp_table = self._make_pos_table("TCP 좌표", ["X", "Y", "Z"], "위치 [mm]")
        jnt_box, self.joint_table = self._make_pos_table("관절 각도", [f"J{i + 1}" for i in range(6)], "각도 [°]")
        ph.addWidget(tcp_box, 3)
        ph.addWidget(jnt_box, 6)
        tabs.addTab(pos_w, "로봇 위치")

        # 명령 이력
        hist_w = QWidget()
        hv = QVBoxLayout(hist_w)
        hb = QHBoxLayout()
        self.btn_hist_save = QPushButton("CSV 저장")
        self.btn_hist_clear = QPushButton("지우기")
        hb.addWidget(self.btn_hist_save)
        hb.addWidget(self.btn_hist_clear)
        hb.addStretch()
        hv.addLayout(hb)
        self.hist_table = QTableWidget(0, 6)
        self.hist_table.setHorizontalHeaderLabels(["시각", "명령", "대상", "결과", "소요 (s)", "비고"])
        self.hist_table.verticalHeader().setVisible(False)
        self.hist_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.hist_table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        hv.addWidget(self.hist_table)
        tabs.addTab(hist_w, "명령 이력")

        # 레지스터 (퀴즈3, 선택)
        self.reg_table = QTableWidget(HR_COUNT, 3)
        self.reg_table.setHorizontalHeaderLabels(["주소", "항목", "값"])
        self.reg_table.verticalHeader().setVisible(False)
        self.reg_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.reg_table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        for i in range(HR_COUNT):
            self.reg_table.setItem(i, 0, QTableWidgetItem(str(HR_START + i)))
            self.reg_table.setItem(i, 1, QTableWidgetItem(REG_NAMES.get(i, "-")))
            self.reg_table.setItem(i, 2, QTableWidgetItem("-"))
        tabs.addTab(self.reg_table, "레지스터 (선택)")

        # 카메라 설정 (검출 파라미터)
        tabs.addTab(self._build_param_tab(), "카메라 설정")

        # 작업 로그 (퀴즈3)
        log_w = QWidget()
        lv = QVBoxLayout(log_w)
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.btn_log_save = QPushButton("로그 저장")
        lv.addWidget(self.log_view)
        lv.addWidget(self.btn_log_save, alignment=Qt.AlignmentFlag.AlignRight)
        tabs.addTab(log_w, "작업 로그")
        self.tabs = tabs

        # ---------------- 전체 배치 ----------------
        top = QHBoxLayout()
        top.addWidget(cam_box, 3)
        top.addWidget(right, 2)
        root = QVBoxLayout(self)
        root.addLayout(top, 7)
        root.addWidget(tabs, 3)

    def _build_param_tab(self):
        w = QWidget()
        g = QGridLayout(w)
        self.param_spins = {}
        spec = [("H 하한", "h_low", 0, 179), ("H 상한", "h_high", 0, 179),
                ("S 하한", "s_low", 0, 255), ("S 상한", "s_high", 0, 255),
                ("V 하한", "v_low", 0, 255), ("V 상한", "v_high", 0, 255),
                ("최소 면적 (px)", "min_area", 0, 200_000), ("블러", "blur", 1, 31)]
        for i, (label, name, lo, hi) in enumerate(spec):
            sp = QSpinBox()
            sp.setRange(lo, hi)
            sp.setValue(int(getattr(self.params, name)))
            g.addWidget(QLabel(label), i // 4, (i % 4) * 2)
            g.addWidget(sp, i // 4, (i % 4) * 2 + 1)
            self.param_spins[name] = sp
        self.aspect_spin = QDoubleSpinBox()
        self.aspect_spin.setRange(1.0, 20.0)
        self.aspect_spin.setSingleStep(0.1)
        self.aspect_spin.setValue(float(self.params.min_aspect))
        self.invert_chk = QCheckBox("마스크 반전")
        self.invert_chk.setChecked(bool(self.params.invert))
        self.btn_param_save = QPushButton("파라미터 저장")
        self.btn_param_default = QPushButton("기본값")
        g.addWidget(QLabel("최소 가로세로비"), 2, 0)
        g.addWidget(self.aspect_spin, 2, 1)
        g.addWidget(self.invert_chk, 2, 2, 1, 2)
        g.addWidget(self.btn_param_save, 2, 4)
        g.addWidget(self.btn_param_default, 2, 5)
        note = QLabel("· H 하한 > H 상한이면 빨강처럼 0/179를 넘나드는 범위로 처리\n"
                      "· [보기: 마스크]로 바꾸면 흰색 = 막대로 인식된 영역\n"
                      "· 저장하면 vision_params.json에 기록되고 vision.py 단독 실행과 공유")
        note.setStyleSheet("color:#555;")
        g.addWidget(note, 3, 0, 1, 8)
        g.setRowStretch(4, 1)
        return w

    # ---- 퀴즈3의 위치 테이블 그대로 ----
    def _make_pos_table(self, title, cols, unit):
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
        t.verticalHeader().setDefaultSectionSize(38)
        t.verticalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Fixed)
        t.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        t.setSelectionMode(QAbstractItemView.SelectionMode.NoSelection)
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
        v.addStretch()
        return box, t

    def _update_pos_table(self, table, values, prev, base):
        for c, val in enumerate(values):
            cur = table.item(0, c)
            cur.setText(f"{val:.1f}")
            if prev is not None and abs(val - prev[c]) >= 0.05:
                cur.setBackground(QBrush(QColor("#fde68a")))
                cur.setForeground(QBrush(QColor("#111827")))
            else:
                cur.setData(Qt.ItemDataRole.BackgroundRole, None)
                cur.setData(Qt.ItemDataRole.ForegroundRole, None)
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
        self.cam_btn.clicked.connect(self.toggle_camera)
        self.btn_capture.clicked.connect(self.save_frame)
        self.snap_btn.clicked.connect(lambda: self.take_snapshot())
        self.btn_home_snap.clicked.connect(self.on_home_snapshot)
        self.det_table.itemSelectionChanged.connect(self.on_selection_changed)

        self.rb_local_btn.clicked.connect(lambda: self.rb_ip.setText(config.LOCAL_IP))
        self.rb_btn.clicked.connect(self.toggle_robot)
        self.mb_btn.clicked.connect(self.toggle_modbus)

        self.btn_pick.clicked.connect(self.on_pick)
        self.btn_home.clicked.connect(lambda: self.send_command("HOME", "HOME"))
        self.btn_task1.clicked.connect(lambda: self.send_command("TASK1", "TASK,1"))
        self.btn_task2.clicked.connect(lambda: self.send_command("TASK2", "TASK,2"))

        for sp in self.param_spins.values():
            sp.valueChanged.connect(self.on_params_changed)
        self.aspect_spin.valueChanged.connect(self.on_params_changed)
        self.invert_chk.toggled.connect(self.on_params_changed)
        self.btn_param_save.clicked.connect(self.save_params)
        self.btn_param_default.clicked.connect(self.reset_params)

        self.btn_listen.pressed.connect(self.start_listening)
        self.btn_listen.released.connect(self.stop_listening)
        self.text_cmd.returnPressed.connect(self.on_text_command)
        self.btn_text_cmd.clicked.connect(self.on_text_command)
        self.btn_confirm.clicked.connect(self.execute_plan)
        self.btn_cancel.clicked.connect(self.cancel_plan)

        self.btn_hist_save.clicked.connect(self.save_history)
        self.btn_hist_clear.clicked.connect(lambda: self.hist_table.setRowCount(0))
        self.btn_log_save.clicked.connect(self.save_log)

        self.robot.connected.connect(self.on_robot_connected)
        self.robot.disconnected.connect(self.on_robot_disconnected)
        self.robot.readyRead.connect(self.on_robot_ready_read)
        self.robot.errorOccurred.connect(self.on_robot_error)

    # =====================================================================
    # 공통
    # =====================================================================
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
            f"background:{STATE_COLOR[state]}; color:white; font: bold 16px; padding:8px; border-radius:6px;")

    def robot_connected(self):
        return self.robot.state() == QAbstractSocket.SocketState.ConnectedState

    def robot_is_local(self):
        return self.rb_ip.text().strip().lower() in LOCAL_HOSTS

    def update_buttons(self):
        # 정렬 같은 연속 실행 중에는 PICK 사이사이에도 다른 명령을 막는다
        can_cmd = self.robot_connected() and self.robot_ready and not self.busy and self.seq is None
        for b in (self.btn_home, self.btn_task1, self.btn_task2):
            b.setEnabled(can_cmd)
        self.btn_pick.setEnabled(can_cmd and self.selected is not None)
        self.snap_btn.setEnabled(self.cam_worker is not None)
        self.btn_home_snap.setEnabled(can_cmd and self.cam_worker is not None)

        listening = self.recorder is not None and self.recorder.recording
        stt_free = self.stt_worker is None
        self.btn_listen.setEnabled(self.recognizer is not None and stt_free and
                                   (listening or (not self.busy and self.seq is None) or self.plan is not None))
        self.btn_text_cmd.setEnabled(stt_free)
        self.text_cmd.setEnabled(stt_free)
        self.btn_confirm.setEnabled(self.plan is not None and can_cmd)
        self.btn_cancel.setEnabled(self.plan is not None or self.pending_intent is not None or
                                   (self.seq is not None and not self.seq.get("abort")))
        self.btn_capture.setEnabled(self.cam_worker is not None)
        self.rb_btn.setEnabled(not self.busy)
        self.mb_btn.setEnabled(ModbusTcpClient is not None and not self.busy)

    # =====================================================================
    # 카메라
    # =====================================================================
    def toggle_camera(self):
        if self.cam_worker is None:
            w = CameraWorker(self.cam_index.value(), self.current_params(), self)
            w.frame_ready.connect(self.on_frame)
            w.status.connect(lambda m: self.log(f"[카메라] {m}"))
            w.mapper_ready.connect(self.on_mapper)
            w.failed.connect(lambda m: self.log(f"[카메라 오류] {m}"))
            w.finished.connect(self.on_camera_finished)
            self.cam_worker = w
            w.start()
            self.cam_btn.setText("카메라 정지")
            self.cam_index.setEnabled(False)
            self.update_buttons()
        else:
            self.cam_btn.setEnabled(False)
            self.cam_worker.stop()

    def on_camera_finished(self):
        if self.cam_worker is not None:
            self.cam_worker.deleteLater()
        self.cam_worker = None
        self.live_sticks = []
        self.view.clear()
        self.view.setText("카메라 정지")
        self.cam_btn.setText("카메라 시작")
        self.cam_btn.setEnabled(True)
        self.cam_index.setEnabled(True)
        self.fps_label.setText("FPS -")
        self.update_buttons()

    def on_mapper(self, mapper):
        self.mapper = mapper
        if mapper is None:
            self.calib_label.setText("좌표 변환 없음 (캘리브레이션 필요)")
            self.calib_label.setStyleSheet("color:#dc2626; font-weight:bold;")
        elif mapper.is_dummy:
            self.calib_label.setText("임시 변환 (실제 로봇 PICK 금지)")
            self.calib_label.setStyleSheet("color:#d97706; font-weight:bold;")
        else:
            self.calib_label.setText("캘리브레이션 적용")
            self.calib_label.setStyleSheet("color:#16a34a; font-weight:bold;")
        self.log(f"[좌표 변환] {'없음' if mapper is None else mapper.source}")

    def on_frame(self, frame, sticks, mask, ts, fps):
        self.live_sticks, self.live_ts = sticks, ts
        self._last_frame = frame

        # 예약된 자동 스냅샷: AT_HOME을 받고 SNAP_SETTLE_S가 지난 뒤 들어온 프레임으로 찍는다
        if self.pending_snap:
            if self.at_home_ts >= self.pending_since and ts >= self.at_home_ts + SNAP_SETTLE_S:
                self.pending_snap = False
                self.take_snapshot(auto=True)
            elif time.time() - self.pending_since > SNAP_PENDING_TIMEOUT_S:
                self.pending_snap = False
                self.log("자동 스냅샷 취소 (AT_HOME을 받지 못함)")
                if self.seq is not None:
                    self._end_seq(False, "PICK 후 새 스냅샷을 찍지 못함")
                if self.pending_intent is not None:
                    self.pending_intent = None
                    self.lbl_plan.setText("계획: 스냅샷을 찍지 못해 취소")

        base = mask if self.view_mode.currentIndex() == 1 else frame
        info = [f"live sticks: {len(sticks)}",
                f"snapshot: {len(self.snap_sticks)}" + ("" if self.snapshot_valid() or not self.snap_sticks
                                                         else "  (STALE - take new snapshot)")]
        targets = []
        if self.plan is not None:                       # 승인 대기 중인 계획
            targets = [(st.stick, f"{k + 1}: -> P{st.slot}") for k, st in enumerate(self.plan.steps)]
        elif self.seq is not None:                      # 실행 중인 계획의 남은 단계
            steps = self.seq["plan"].steps
            targets = [(steps[k].stick, f"{k + 1}: -> P{steps[k].slot}") for k in range(self.seq["i"], len(steps))]
        img = draw_detections(base, sticks, self.mapper, target=self.selected, info_lines=info, targets=targets)
        self._show_image(img)
        self.fps_label.setText(f"FPS {fps:.1f}")

    def _show_image(self, bgr):
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        h, w = rgb.shape[:2]
        qimg = QImage(rgb.data, w, h, 3 * w, QImage.Format.Format_RGB888).copy()
        pix = QPixmap.fromImage(qimg).scaled(self.view.size(), Qt.AspectRatioMode.KeepAspectRatio,
                                             Qt.TransformationMode.SmoothTransformation)
        self.view.setPixmap(pix)

    def save_frame(self):
        frame = getattr(self, "_last_frame", None)
        if frame is None:
            return
        config.CAPTURE_DIR.mkdir(parents=True, exist_ok=True)
        path = config.CAPTURE_DIR / time.strftime("frame_%Y%m%d_%H%M%S.png")
        cv2.imwrite(str(path), frame)
        self.log(f"프레임 저장: {path}")

    # ---------------- 검출 파라미터 ----------------
    def current_params(self):
        vals = {name: sp.value() for name, sp in self.param_spins.items()}
        return VisionParams(min_aspect=self.aspect_spin.value(), invert=self.invert_chk.isChecked(), **vals)

    def on_params_changed(self, *_):
        self.params = self.current_params()
        if self.cam_worker is not None:
            self.cam_worker.set_params(self.params)

    def save_params(self):
        self.params = self.current_params()
        self.params.save()
        self.log(f"검출 파라미터 저장: {config.VISION_PARAMS_FILE}")

    def reset_params(self):
        d = VisionParams()
        for name, sp in self.param_spins.items():
            sp.blockSignals(True)
            sp.setValue(int(getattr(d, name)))
            sp.blockSignals(False)
        self.aspect_spin.setValue(d.min_aspect)
        self.invert_chk.setChecked(d.invert)
        self.on_params_changed()

    # ---------------- 스냅샷 / 선택 ----------------
    def snapshot_valid(self):
        """스냅샷 이후 로봇이 막대를 움직이지 않았으면 True."""
        return bool(self.snap_sticks) and self.snap_ts > self.scene_changed_ts

    def on_home_snapshot(self):
        """로봇을 홈으로 보내고, AT_HOME 뒤 프레임으로 자동 스냅샷."""
        if self.send_command("HOME", "HOME", target="HOME 후 스냅샷"):
            self.pending_snap = True
            self.pending_since = self.current["t0"]

    def take_snapshot(self, auto=False):
        if self.cam_worker is None or self.live_ts == 0:
            self.log("카메라 영상이 없어 스냅샷을 찍을 수 없음")
            return
        if not auto:
            if self.busy:
                self.log("[주의] 로봇 동작 중 스냅샷: 팔이 막대를 가렸을 수 있음")
            elif self.robot_connected() and not self.robot_at_home:
                self.log("[주의] 로봇이 홈 자세인지 확인되지 않음 → [HOME 후 스냅샷] 권장")
        self.snap_sticks = list(self.live_sticks)
        self.snap_ts = self.live_ts
        self.selected = None
        self.det_table.clearSelection()
        self.det_table.setRowCount(len(self.snap_sticks))
        for r, s in enumerate(self.snap_sticks):
            if s.has_robot_coords:
                cells = [f"{s.idx}", f"{s.x:.1f}", f"{s.y:.1f}", f"{s.angle:+.1f}", f"{s.length:.1f}",
                         "O" if s.in_pick_zone else "X"]
            else:
                cells = [f"{s.idx}", "-", "-", f"{s.angle_px:+.1f} (px)", f"{s.length_px:.0f} (px)", "-"]
            for c, v in enumerate(cells):
                it = QTableWidgetItem(v)
                it.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                if s.has_robot_coords and not s.in_pick_zone:
                    it.setForeground(QBrush(QColor("#9ca3af")))
                self.det_table.setItem(r, c, it)
        stamp = datetime.datetime.fromtimestamp(self.snap_ts).strftime("%H:%M:%S")
        self.snap_info.setText(f"{len(self.snap_sticks)}개 검출  ({stamp}{', 자동' if auto else ''})")
        self.log(f"{'자동 ' if auto else ''}스냅샷: 막대 {len(self.snap_sticks)}개")
        self.update_buttons()

        # 새 스냅샷을 기다리던 명령 처리
        if self.pending_intent is not None:
            intent, self.pending_intent = self.pending_intent, None
            self.plan_and_confirm(intent)
        elif self.seq is not None:
            self._try_continue_seq()

    def on_selection_changed(self):
        rows = {i.row() for i in self.det_table.selectedIndexes()}
        self.selected = self.snap_sticks[rows.pop()] if rows else None
        self.update_buttons()

    # =====================================================================
    # 로봇 명령
    # =====================================================================
    def on_pick(self):
        s = self.selected
        if s is None:
            self.log("집을 막대를 목록에서 선택하세요")
            return
        if not self.snapshot_valid():
            self.log("로봇이 움직인 뒤라 스냅샷이 오래됨 → 스냅샷을 다시 찍으세요")
            return
        if not s.has_robot_coords or self.mapper is None:
            self.log("좌표 변환이 없어 로봇 좌표를 계산할 수 없음")
            return
        if not s.in_pick_zone:
            self.log(f"막대 #{s.idx}가 집기 영역 밖에 있음 ({s.x:.0f}, {s.y:.0f})")
            return
        if self.mapper.is_dummy and not self.robot_is_local():
            self.log("임시 변환 상태에서는 실제 로봇으로 PICK을 보낼 수 없음 (3단계 캘리브레이션 후 사용)")
            return
        slot = self.dest_combo.currentData()
        line = f"PICK,{s.x:.1f},{s.y:.1f},{s.angle:.1f},{slot}"
        if self.send_command("PICK", line, target=f"막대 #{s.idx} → P{slot}") and self.auto_snap_chk.isChecked():
            # DRL은 PICK이 끝나면(성공·실패 모두) 홈으로 돌아가 AT_HOME을 보낸다 → 그 뒤 자동 스냅샷
            self.pending_snap = True
            self.pending_since = self.current["t0"]

    def send_command(self, kind, line, target=""):
        if self.busy:
            self.log("다른 작업 실행 중")
            return False
        if not (self.robot_connected() and self.robot_ready):
            self.log("로봇이 연결되지 않았거나 READY 전")
            return False
        task_no, timeout_ms = COMMANDS[kind]
        self.busy = True
        self.robot_at_home = False
        self.current = {"kind": kind, "task_no": task_no, "line": line, "target": target, "t0": time.time()}
        self.base_xyz, self.base_joint = self.prev_xyz, self.prev_joint
        self.robot.write((line + "\n").encode())
        self.robot.flush()
        self.task_timer.start(timeout_ms)
        self.set_task_state(STATE_BUSY)
        self.cmd_label.setText(f"현재 명령: {line}" + (f"  ({target})" if target else ""))
        self.log(f"→ 로봇: {line}")
        self.update_buttons()
        return True

    def finish_command(self, ok, reason=""):
        if not self.busy or self.current is None:
            return
        self.task_timer.stop()
        cur = self.current
        elapsed = time.time() - cur["t0"]
        if cur["kind"] != "HOME":
            # 막대가 움직였을 수 있음 → 그 전에 찍은 스냅샷은 무효.
            # 이번 명령 중에 AT_HOME을 받았다면 그때 이미 장면 변화가 끝났으므로 그 시각을 기준으로 한다
            # (PICK 후 자동 스냅샷은 AT_HOME 뒤, DONE 전에 찍히기 때문)
            self.scene_changed_ts = self.at_home_ts if self.at_home_ts >= cur["t0"] else time.time()
        if ok and cur["task_no"] and self.mb_ok:
            self.increment_task_count(cur["task_no"])
            self.poll_registers()
        self.set_task_state(STATE_DONE if ok else STATE_ERROR)
        self.add_history(cur, ok, reason, elapsed)
        self.log(f"{cur['kind']} {'완료' if ok else '실패: ' + reason} ({elapsed:.1f}s)")
        self.busy = False
        self.current = None
        self.cmd_label.setText("현재 명령: -")
        if cur["kind"] != "HOME" and not self.snapshot_valid():
            self.selected = None
            self.det_table.clearSelection()
            if self.snap_sticks:
                self.snap_info.setText(self.snap_info.text() + "  → 오래됨, 다시 찍을 것")

        # 연속 실행(정렬 등) 중이면 다음 단계로
        if self.seq is not None and cur["kind"] == "PICK":
            if not ok:
                self._end_seq(False, f"{self.seq['i'] + 1}번째 PICK 실패: {reason}")
            else:
                self.seq["last_ok"] = True
                self._try_continue_seq()
        elif cur["kind"] == "HOME" and self.pending_intent is not None and not ok:
            self.pending_intent = None
            self.lbl_plan.setText("계획: HOME 실패로 취소")
        self.update_buttons()

    def add_history(self, cur, ok, reason, elapsed):
        r = self.hist_table.rowCount()
        self.hist_table.insertRow(r)
        cells = [datetime.datetime.fromtimestamp(cur["t0"]).strftime("%H:%M:%S"),
                 cur["line"], cur["target"], "성공" if ok else "실패", f"{elapsed:.1f}", reason]
        for c, v in enumerate(cells):
            it = QTableWidgetItem(v)
            if c == 3:
                it.setForeground(QBrush(QColor("#16a34a" if ok else "#dc2626")))
            self.hist_table.setItem(r, c, it)
        self.hist_table.scrollToBottom()

    # =====================================================================
    # 음성/텍스트 명령 → 해석 → 계획 → 승인 → 실행 (5단계)
    # =====================================================================
    def _rest_state(self):
        if self.plan is not None:
            return STATE_CONFIRM
        return STATE_BUSY if (self.busy or self.seq is not None) else STATE_IDLE

    def on_text_command(self):
        text = self.text_cmd.text().strip()
        if text:
            self.text_cmd.clear()
            self.handle_text_command(text, "텍스트")

    def handle_text_command(self, text, source="텍스트"):
        """텍스트 입력과 STT 결과가 모두 여기로 들어온다."""
        self.lbl_heard.setText(f"인식 ({source}): {text}")
        self.log(f"[{source}] {text}")

        # 1) 승인 대기 중: 승인어/취소어 먼저 확인 (그 외 문장은 새 명령으로 처리)
        if self.plan is not None:
            if is_confirm(text):
                self.execute_plan()
                return
            if is_cancel(text):
                self.cancel_plan()
                return
        # 2) 연속 실행 중: 취소만 받는다
        if self.seq is not None:
            if is_cancel(text):
                self.cancel_plan()
            else:
                self.log("연속 실행 중이라 새 명령은 받지 않음 (취소만 가능)")
            return
        if self.busy:
            self.log("로봇 작업 중이라 명령을 받지 않음")
            return
        if is_cancel(text):
            self.cancel_plan()
            return
        if is_confirm(text):
            self.lbl_intent.setText("해석: 승인할 계획이 없음")
            return

        # 3) 새 명령 해석
        res = parse(text)
        self.plan = None
        if res.error:
            self.lbl_intent.setText(f"해석 실패: {res.error}")
            self.lbl_plan.setText("계획: -")
            self.log(f"해석 실패: {res.error}")
            self.set_task_state(STATE_IDLE)
            self.update_buttons()
            return
        intent = res.intent
        self.lbl_intent.setText(f"해석: {intent.describe()}")
        self.log(f"해석: {intent.describe()}")

        if intent.action == "HOME":           # 홈 이동은 위험이 적어 승인 없이 바로 실행
            self.lbl_plan.setText("계획: 홈으로 이동 (바로 실행)")
            self.send_command("HOME", "HOME", target=f"{source} 명령")
            return

        # 4) 판단에 쓸 스냅샷: 충분히 최근이고 로봇이 홈에 있을 때 찍은 것이면 그대로, 아니면 새로
        fresh = (self.snapshot_valid() and time.time() - self.snap_ts < SNAP_MAX_AGE_S and
                 (self.robot_at_home or not self.robot_connected()))
        if fresh:
            self.plan_and_confirm(intent)
        elif self.robot_connected() and self.robot_ready and self.cam_worker is not None:
            self.pending_intent = intent
            self.lbl_plan.setText("계획: 로봇을 홈으로 보내고 새 스냅샷을 찍는 중...")
            self.on_home_snapshot()
            if not self.busy:                 # HOME 전송 실패
                self.pending_intent = None
                self.lbl_plan.setText("계획: HOME 전송 실패")
        elif self.cam_worker is not None and self.live_ts:
            self.take_snapshot()              # 로봇 미연결(테스트): 지금 화면으로 판단
            self.plan_and_confirm(intent)
        else:
            self.lbl_plan.setText("계획 불가: 카메라가 꺼져 있음")
        self.update_buttons()

    def plan_and_confirm(self, intent):
        res = make_plan(intent, self.snap_sticks, self.dest_combo.currentData())
        if res.error:
            self.plan = None
            self.lbl_plan.setText(f"계획 불가: {res.error}")
            self.log(f"계획 불가: {res.error}")
            self.set_task_state(STATE_IDLE)
            self.update_buttons()
            return
        self.plan = res.plan
        self.plan_snap_ts = self.snap_ts
        notes = ("  / 주의: " + "; ".join(res.plan.notes)) if res.plan.notes else ""
        self.lbl_plan.setText(f"계획: {res.plan.summary}{notes}\n→ [승인] 또는 '실행'이라고 말하기 / 취소")
        self.log(f"계획: {res.plan.summary}{notes}")
        self.set_task_state(STATE_CONFIRM)
        self.update_buttons()

    def execute_plan(self):
        plan = self.plan
        if plan is None:
            return
        if self.busy or self.seq is not None or not (self.robot_connected() and self.robot_ready):
            self.log("로봇이 연결되지 않았거나 작업 중이라 실행할 수 없음")
            return
        if not self.snapshot_valid() or self.snap_ts != getattr(self, "plan_snap_ts", None):
            self.plan = None
            self.lbl_plan.setText("계획: 그 사이 장면이 바뀌어 취소됨 → 명령을 다시 말해 주세요")
            self.set_task_state(STATE_IDLE)
            self.update_buttons()
            return
        if self.mapper is None or (self.mapper.is_dummy and not self.robot_is_local()):
            self.log("임시 변환 상태에서는 실제 로봇으로 PICK을 보낼 수 없음 (3단계 캘리브레이션 후 사용)")
            return
        self.plan = None
        self.seq = {"plan": plan, "i": 0, "sent_t0": 0.0, "last_ok": False, "abort": False,
                    "t_start": time.time()}
        self.log(f"계획 실행: {plan.summary}")
        self._seq_send_current()

    def _seq_send_current(self):
        seq = self.seq
        plan = seq["plan"]
        step = plan.steps[seq["i"]]
        s = step.stick
        line = f"PICK,{s.x:.1f},{s.y:.1f},{s.angle:.1f},{step.slot}"
        target = f"[{plan.action} {seq['i'] + 1}/{len(plan.steps)}] 막대 #{s.idx} → P{step.slot}"
        seq["last_ok"] = False
        self.lbl_plan.setText(f"실행 중: {target}")
        if self.send_command("PICK", line, target=target):
            seq["sent_t0"] = self.current["t0"]
            self.pending_snap = True            # 다음 막대를 찾기 위해 항상 자동 스냅샷
            self.pending_since = seq["sent_t0"]
        else:
            self._end_seq(False, "PICK 전송 실패")

    def _try_continue_seq(self):
        """앞 PICK이 성공했고, 그 뒤 새 스냅샷이 찍혔으면 다음 PICK을 보낸다."""
        seq = self.seq
        if seq is None or self.busy or not seq["last_ok"]:
            return
        last = seq["i"] >= len(seq["plan"].steps) - 1
        if self.cam_worker is None:            # 카메라가 꺼지면 다음 막대를 찾을 수 없음
            self._end_seq(last, "" if last else "카메라가 꺼져 다음 막대를 찾을 수 없음")
            return
        if self.snap_ts <= seq["sent_t0"] or not self.snapshot_valid():
            return                             # 아직 새 스냅샷 전 → 스냅샷이 찍히면 다시 호출됨
        if seq["abort"] and not last:
            self._end_seq(False, "사용자가 취소 (진행 중이던 PICK까지만 실행)")
            return
        seq["i"] += 1
        plan = seq["plan"]
        if seq["i"] >= len(plan.steps):
            self._end_seq(True)
            return
        step = plan.steps[seq["i"]]
        new = rematch(step.stick, self.snap_sticks)
        if new is None:
            self._end_seq(False, f"다음 막대 #{step.stick.idx}를 새 스냅샷에서 찾지 못함 (움직였거나 가려짐)")
            return
        step.stick = new
        self._seq_send_current()

    def _end_seq(self, ok, reason=""):
        seq, self.seq = self.seq, None
        if seq is None:
            return
        plan = seq["plan"]
        elapsed = time.time() - seq["t_start"]
        if plan.action == "SORT":
            if ok and self.mb_ok:
                self.increment_task_count(SORT_TASK_NO)
                self.poll_registers()
            self.add_history({"t0": seq["t_start"], "line": "SORT (정렬 전체)", "target": plan.summary},
                             ok, reason, elapsed)
        if ok:
            self.lbl_plan.setText(f"계획: 완료 ({len(plan.steps)}개, {elapsed:.0f}s)")
            self.log(f"계획 완료: {plan.summary} ({elapsed:.1f}s)")
            self.set_task_state(STATE_DONE)
        else:
            self.lbl_plan.setText(f"계획: 중단 - {reason}")
            self.log(f"계획 중단: {reason}")
            self.set_task_state(STATE_ERROR)
        self.update_buttons()

    def cancel_plan(self):
        if self.plan is not None:
            self.plan = None
            self.lbl_plan.setText("계획: 취소됨")
            self.log("계획 취소")
            self.set_task_state(STATE_IDLE)
        elif self.pending_intent is not None:
            self.pending_intent = None
            self.lbl_plan.setText("계획: 취소됨 (홈 이동은 계속)")
            self.log("명령 취소 (스냅샷 대기 중)")
        elif self.seq is not None:
            self.seq["abort"] = True
            self.lbl_plan.setText("계획: 진행 중인 PICK이 끝나면 멈춤 (즉시 정지는 비상정지 버튼)")
            self.log("연속 실행 취소 요청")
        self.update_buttons()

    # =====================================================================
    # 음성 인식 (6단계)
    # =====================================================================
    def start_stt_loading(self):
        ok, msg = stt.check_available()
        if not ok:
            self.stt_status.setText("STT 꺼짐 (텍스트 입력만)")
            self.stt_status.setToolTip(msg + "\nREADME의 6단계 설치 안내 참고")
            self.log(f"[STT] 사용 불가: {msg}")
            return
        self.stt_status.setText("STT 모델 로드 중...")
        self.stt_loader = SttLoadWorker(self)
        self.stt_loader.loaded.connect(self.on_stt_loaded)
        self.stt_loader.start()

    def on_stt_loaded(self, recognizer, msg):
        self.recognizer = recognizer
        self.stt_status.setText("STT 준비됨" if recognizer else "STT 로드 실패")
        self.stt_status.setToolTip(msg)
        self.log(f"[STT] {msg}")
        self.update_buttons()

    def start_listening(self):
        if self.recognizer is None or self.stt_worker is not None:
            return
        if self.recorder is not None and self.recorder.recording:
            return
        if (self.busy or self.seq is not None) and self.plan is None:
            self.log("로봇 작업 중에는 음성 명령을 받지 않음")
            return
        try:
            self.recorder = stt.Recorder()
            self.recorder.start()
        except Exception as e:
            self.recorder = None
            self.log(f"[STT] 마이크 열기 실패: {e}")
            return
        self.btn_listen.setText("듣는 중... (떼면 인식)")
        self.set_task_state(STATE_LISTEN)
        rec = self.recorder
        QTimer.singleShot(int(config.STT_MAX_SECONDS * 1000) + 200,
                          lambda: self.stop_listening() if self.recorder is rec and rec.recording else None)
        self.update_buttons()

    def stop_listening(self):
        if self.recorder is None or not self.recorder.recording:
            return
        audio = self.recorder.stop()
        self.btn_listen.setText("누르고 말하기 (Space)")
        if len(audio) < config.STT_MIN_SECONDS * stt.SAMPLE_RATE:
            self.log("[STT] 녹음이 너무 짧음")
            self.set_task_state(self._rest_state())
            self.update_buttons()
            return
        self.set_task_state(STATE_THINK)
        w = SttRecognizeWorker(self.recognizer, audio, self)
        w.done.connect(self.on_stt_result)
        w.finished.connect(w.deleteLater)
        self.stt_worker = w
        w.start()
        self.update_buttons()

    def on_stt_result(self, res):
        self.stt_worker = None
        self.set_task_state(self._rest_state())
        if res.get("error"):
            self.log(f"[STT] 인식 오류: {res['error']}")
        text = res.get("text", "")
        if not text:
            self.lbl_heard.setText("인식: (알아듣지 못함) → 다시 말해 주세요")
            self.log("[STT] 인식 결과 없음")
            self.update_buttons()
            return
        self.log(f"[STT] '{text}' ({res.get('mode')}, 인식 {res.get('seconds', 0):.2f}s)")
        self.handle_text_command(text, f"음성/{res.get('mode')}")
        self.update_buttons()

    def eventFilter(self, obj, ev):
        """Space를 누르고 있는 동안 녹음 (입력란에 글을 쓰는 중이면 제외)."""
        if ev.type() in (QEvent.Type.KeyPress, QEvent.Type.KeyRelease) and \
                ev.key() == Qt.Key.Key_Space and self.isActiveWindow():
            fw = QApplication.focusWidget()
            if isinstance(fw, (QLineEdit, QAbstractSpinBox, QPlainTextEdit)):
                return False
            if not ev.isAutoRepeat():
                if ev.type() == QEvent.Type.KeyPress:
                    self.start_listening()
                else:
                    self.stop_listening()
            return True        # 버튼에 포커스가 있어도 Space로 눌리지 않게 막음
        return super().eventFilter(obj, ev)

    # =====================================================================
    # 로봇 소켓 (퀴즈3 기반)
    # =====================================================================
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
        self.rx_buf = ""
        self.set_conn_label(self.rb_status, True, "● READY 대기")
        self.rb_btn.setText("해제")
        self.rb_ip.setEnabled(False)
        self.rb_port.setEnabled(False)
        self.rb_local_btn.setEnabled(False)
        self.log("로봇 연결됨" + (" (로컬 = 가짜 로봇)" if self.robot_is_local() else ""))
        self.update_buttons()

    def on_robot_disconnected(self):
        self.robot_ready = False
        self.set_conn_label(self.rb_status, False)
        self.rb_btn.setText("연결")
        self.rb_ip.setEnabled(True)
        self.rb_port.setEnabled(True)
        self.rb_local_btn.setEnabled(True)
        self.log("로봇 연결 해제")
        if self.busy:
            self.finish_command(False, "작업 중 로봇 연결 끊김")
        self.update_buttons()

    def on_robot_error(self, _err):
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
        cmd = cmd.strip().upper()

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

        elif cmd == "DONE":
            try:
                n = int(body.strip())
            except ValueError:
                self.log(f"DONE 형식 오류: {line}")
                return
            if self.busy and self.current["task_no"] == n:
                self.finish_command(True)
            else:
                self.log(f"예상하지 않은 DONE: {line}")

        elif cmd == "AT_HOME":
            self.at_home_ts = time.time()
            self.robot_at_home = True
            if self.busy and self.current["kind"] == "HOME":
                self.finish_command(True)
            else:
                self.log("로봇: AT_HOME")

        elif cmd == "ERROR":
            if self.busy:
                self.finish_command(False, body or "로봇 오류")
            else:
                self.log(f"로봇 오류 (작업 외): {line}")

        elif cmd == "READY":
            if not self.robot_ready:
                self.robot_ready = True
                self.set_conn_label(self.rb_status, True, "● 연결됨 / READY")
                self.log("로봇 READY 수신")
                self.update_buttons()

        else:   # DATA, ORDER, "MoveJ Start" 같은 진행 메시지
            self.log(f"로봇: {line}")

    # =====================================================================
    # Modbus (퀴즈3 기반, 선택 기능)
    # =====================================================================
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

    def increment_task_count(self, task_number):
        address = task_number - 1
        try:
            rr = mb_call(self.mb.read_holding_registers, HR_START + address, count=1)
            if rr.isError():
                self.log(f"완료 횟수 읽기 오류 (주소 {address}): {rr}")
                return False
            count = rr.registers[0]
            if count >= 0xFFFF:
                return False
            wr = mb_call(self.mb.write_registers, HR_START + address, [count + 1])
            if wr.isError():
                self.log(f"완료 횟수 쓰기 오류 (주소 {address}): {wr}")
                return False
            return True
        except Exception as e:
            self.log(f"Modbus 예외: {e}")
            return False

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
        for i, raw in enumerate(rr.registers):
            self.reg_table.item(i, 2).setText(f"{raw / 10:.1f}" if i >= 10 else str(raw))

    # =====================================================================
    # 저장 / 종료
    # =====================================================================
    def save_history(self):
        config.LOG_DIR.mkdir(parents=True, exist_ok=True)
        default = str(config.LOG_DIR / time.strftime("history_%Y%m%d_%H%M%S.csv"))
        path, _ = QFileDialog.getSaveFileName(self, "명령 이력 저장", default, "CSV (*.csv)")
        if not path:
            return
        with open(path, "w", newline="", encoding="utf-8-sig") as f:
            w = csv.writer(f)
            w.writerow([self.hist_table.horizontalHeaderItem(c).text() for c in range(self.hist_table.columnCount())])
            for r in range(self.hist_table.rowCount()):
                w.writerow([(self.hist_table.item(r, c).text() if self.hist_table.item(r, c) else "")
                            for c in range(self.hist_table.columnCount())])
        self.log(f"명령 이력 저장: {path}")

    def save_log(self):
        path, _ = QFileDialog.getSaveFileName(self, "로그 저장", "task_log.txt", "Text (*.txt)")
        if path:
            with open(path, "w", encoding="utf-8") as f:
                f.write(self.log_view.toPlainText())

    def closeEvent(self, e):
        QApplication.instance().removeEventFilter(self)
        if self.recorder is not None and self.recorder.recording:
            self.recorder.stop()
        for w in (self.stt_worker, self.stt_loader):
            if w is not None and w.isRunning():
                w.wait(5000)
        if self.cam_worker is not None:
            self.cam_worker.stop()
        self.poll_timer.stop()
        if self.mb:
            self.mb.close()
        self.robot.abort()
        super().closeEvent(e)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec())
