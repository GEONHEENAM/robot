"""Quiz 3 PC dashboard: Doosan DRL socket commands + Modbus TCP client.

Required: pip install PyQt6 pymodbus==3.13.1
Run quiz3_server.py (port 5020) first, then start this GUI, then run
Quiz 2 main.drl on the robot (after verifying taught positions and safety).

The robot DRL connects via raw TCP to PC:20000 and understands start1/start2/end.
The separate Modbus server contains 20 holding registers at protocol address 0..19.
Register addresses / scaling come from register_map.py (shared with the server).
This program NEVER sends motion commands directly to a Modbus register.
"""
import csv
import queue
import socket
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import register_map as rm
from pymodbus.client import ModbusTcpClient
from PyQt6.QtCore import QThread, QTimer, pyqtSignal
from PyQt6.QtWidgets import (
    QApplication,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSpinBox,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

ROBOT_SOCKET_PORT = 20000          # Must match PC_PORT in Quiz 2 main.drl.
DEVICE_ID = rm.DEVICE_ID           # For the separate Python Modbus server.
CSV_PATH = Path(__file__).with_name("task2_height_result.csv")
CSV_COLUMNS = [
    "작업ID", "티칭X", "티칭Y", "티칭Z",
    "측정높이(mm)", "측정시각", "소요시간(s)",
]


def now_str():
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def signed16(value):
    """Doosan signed 16-bit holding-register value -> Python signed int."""
    return value - 65536 if value >= 32768 else value


class RobotSocketWorker(QThread):
    """Run TCP server for the robot's existing socket client.

    Only this worker thread owns accept()/recv()/sendall()/close().
    GUI sends commands into a thread-safe queue.
    """
    peer_changed = pyqtSignal(bool, str)
    received = pyqtSignal(str)
    failed = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.stop_event = threading.Event()
        self.commands = queue.Queue()

    def send_command(self, command):
        self.commands.put(command)

    def stop(self):
        self.stop_event.set()

    @staticmethod
    def _enable_keepalive(conn):
        """Detect a silently dead robot link in ~20 s instead of the OS default (2 h)."""
        conn.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        if sys.platform == "win32":
            # (on, idle ms, interval ms)
            conn.ioctl(socket.SIO_KEEPALIVE_VALS, (1, 10_000, 3_000))
        elif hasattr(socket, "TCP_KEEPIDLE"):
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, 10)
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 3)
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 3)

    def _drop(self, conn):
        """Close the robot link and discard commands meant for that session."""
        try:
            conn.close()
        except OSError:
            pass
        # Commands queued for a prior robot session must not
        # accidentally execute after a new robot connects.
        while True:
            try:
                self.commands.get_nowait()
            except queue.Empty:
                break
        self.peer_changed.emit(False, "")

    def run(self):
        conn = None
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
                # Windows: SO_REUSEADDR may allow a second server to bind the same port.
                if sys.platform != "win32":
                    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                server.bind(("0.0.0.0", ROBOT_SOCKET_PORT))
                server.listen(2)
                server.settimeout(0.05)
                buffer = b""
                while not self.stop_event.is_set():
                    # 1) A new robot connection always wins. If the robot program was
                    #    restarted without a clean close, the old link is already dead.
                    try:
                        new_conn, addr = server.accept()
                    except socket.timeout:
                        new_conn = None
                    if new_conn is not None:
                        if conn is not None:
                            self.failed.emit("로봇 재접속 감지: 이전 연결을 닫고 새 연결 사용")
                            self._drop(conn)
                        conn, buffer = new_conn, b""
                        conn.settimeout(0.2)
                        self._enable_keepalive(conn)
                        self.peer_changed.emit(True, str(addr))
                    if conn is None:
                        continue

                    # 2) Send one queued command, then wait briefly for robot lines.
                    try:
                        try:
                            command = self.commands.get_nowait()
                        except queue.Empty:
                            command = None
                        if command is not None:
                            conn.sendall((command + "\n").encode("utf-8"))
                        try:
                            chunk = conn.recv(4096)
                        except socket.timeout:
                            continue
                        if not chunk:
                            # Robot closed the socket normally (e.g. after "end").
                            self._drop(conn)
                            conn, buffer = None, b""
                            continue
                        buffer += chunk
                        if len(buffer) > 65536:
                            raise ValueError("로봇 소켓 수신 버퍼 초과")
                        while b"\n" in buffer:
                            raw_line, buffer = buffer.split(b"\n", 1)
                            line = raw_line.decode("utf-8", errors="replace").strip()
                            if line:
                                self.received.emit(line)
                    except (OSError, ValueError) as exc:
                        if not self.stop_event.is_set():
                            self.failed.emit(f"로봇 소켓: {exc}")
                        self._drop(conn)
                        conn, buffer = None, b""
        except OSError as exc:
            if not self.stop_event.is_set():
                self.failed.emit(f"로봇 소켓 서버 시작 실패: {exc}")
        finally:
            if conn is not None:
                self._drop(conn)


class ModbusWriteConflict(Exception):
    """A logical/protocol response error, not a dropped TCP connection."""


class ModbusWorker(QThread):
    """Single-writer Modbus client with readback and reconnect reconciliation.

    Requests stay in ``active_request`` across reconnects. On a lost response,
    read the same address before retrying. This requires no other writer to
    modify the task counters/measurement slots. For durability across a server
    *restart*, the server itself must persist its registers.
    """

    connected_changed = pyqtSignal(bool)
    values_ready = pyqtSignal(list)
    write_done = pyqtSignal(int, bool, str)  # operation_id, success, message
    capacity_ready = pyqtSignal(bool, list, str)  # available, 3 addresses, detail
    failed = pyqtSignal(str)  # transient communication / polling errors

    def __init__(self, host, port, parent=None):
        super().__init__(parent)
        self.host = host
        self.port = port
        self.stop_event = threading.Event()
        self.writes = queue.Queue()
        self.active_request = None

    def stop(self):
        self.stop_event.set()

    def increment(self, protocol_address, operation_id):
        self.writes.put({"kind": "increment", "address": protocol_address,
                         "value": None, "before": None, "attempted": False,
                         "id": operation_id})

    def reserve_measurements(self):
        # This check must complete before start2 is sent to the robot.
        self.writes.put({"kind": "capacity"})

    def append_measurement(self, protocol_address, height_mm, operation_id):
        try:
            raw = rm.mm_to_raw(height_mm)
        except ValueError as exc:
            self.write_done.emit(operation_id, False, f"측정값 변환 범위 오류: {exc}")
            return
        if not rm.is_measurement(protocol_address):
            self.write_done.emit(operation_id, False, "측정 레지스터 주소 범위 오류")
            return
        self.writes.put({"kind": "append", "address": protocol_address,
                         "value": raw, "before": 0, "attempted": False,
                         "id": operation_id})

    @staticmethod
    def read_registers(client, address, count):
        result = client.read_holding_registers(
            address=address, count=count, device_id=DEVICE_ID
        )
        if result.isError() or len(result.registers) != count:
            raise ModbusWriteConflict(
                f"HR {address}~{address + count - 1} 읽기 응답 오류: {result}"
            )
        return result.registers

    @staticmethod
    def write_register(client, address, value):
        result = client.write_register(address=address, value=value, device_id=DEVICE_ID)
        if result.isError():
            raise ModbusWriteConflict(f"HR {address} 쓰기 응답 오류: {result}")

    def handle_capacity(self, client):
        need = rm.MEAS_PER_TASK2
        slots = self.read_registers(client, rm.MEAS_START, rm.MEAS_SLOTS)
        free = [rm.MEAS_START + index for index, value in enumerate(slots) if value == 0]
        if len(free) < need:
            self.capacity_ready.emit(False, [],
                                     f"측정 공간 부족: 빈 칸 {len(free)}개 / 필요 {need}개")
        else:
            self.capacity_ready.emit(True, free[:need],
                                     "작업2 측정 레지스터 3칸 확보")

    def handle_write(self, client, request):
        """Resolve an operation based on its original and desired values.

        Never blindly replay a previously attempted write. Instead re-read its
        address and check: original -> write, desired -> already committed,
        anything else -> conflict (manual intervention required).
        """
        address = request["address"]
        if request["kind"] == "increment" and request["before"] is None:
            original = self.read_registers(client, address, 1)[0]
            if original == 65535:
                raise ModbusWriteConflict(f"HR {address} 수행 횟수 상한 초과")
            request["before"] = original
            request["value"] = original + 1

        original, desired = request["before"], request["value"]
        observed = self.read_registers(client, address, 1)[0]
        if observed == desired and request["attempted"]:
            return f"통신 주소 {address}: 값 {desired} 재조회 확인 (이전 요청 반영)"
        if observed != original:
            raise ModbusWriteConflict(
                f"HR {address} 충돌: 원래 {original}, 목표 {desired}, 현재 {observed}"
            )

        # Mark *before* sending: a connection error may arrive after execution.
        request["attempted"] = True
        self.write_register(client, address, desired)
        confirmed = self.read_registers(client, address, 1)[0]
        if confirmed != desired:
            if confirmed != original:
                raise ModbusWriteConflict(
                    f"HR {address} 재조회 충돌: 기대 {desired}, 수신 {confirmed}"
                )
            # Not applied (or the response was lost). Keep operation for retry.
            raise ConnectionError(f"HR {address} 쓰기 후 재조회 불일치; 재접속 후 확인")
        return f"통신 주소 {address}: 값 {desired} 쓰기 + 재조회 검증 완료"

    def process_active(self, client):
        req = self.active_request
        if req["kind"] == "capacity":
            self.handle_capacity(client)
        else:
            text = self.handle_write(client, req)
            self.write_done.emit(req["id"], True, text)
        self.active_request = None

    def run(self):
        while not self.stop_event.is_set():
            client = ModbusTcpClient(self.host, port=self.port, timeout=1.5, retries=0)
            connected = False
            try:
                if not client.connect():
                    raise ConnectionError(f"{self.host}:{self.port} 연결 실패")
                connected = True
                self.connected_changed.emit(True)
                next_poll = 0.0
                while not self.stop_event.is_set():
                    if self.active_request is None:
                        try:
                            self.active_request = self.writes.get_nowait()
                        except queue.Empty:
                            pass
                    if self.active_request is not None:
                        try:
                            self.process_active(client)
                        except ModbusWriteConflict as exc:
                            # A *logical* error does not mean the TCP socket is dead.
                            req = self.active_request
                            self.active_request = None
                            if req["kind"] == "capacity":
                                self.capacity_ready.emit(False, [], str(exc))
                            else:
                                self.write_done.emit(req["id"], False, str(exc))
                        next_poll = 0.0
                    if time.monotonic() >= next_poll:
                        self.values_ready.emit(
                            self.read_registers(client, 0, rm.REGISTER_COUNT))
                        next_poll = time.monotonic() + 1.0
                    self.stop_event.wait(0.1)
            except Exception as exc:
                # Keep active_request untouched; reconnect and reconcile it.
                if not self.stop_event.is_set():
                    self.failed.emit(f"Modbus 재접속/재검증 대기: {exc}")
            finally:
                if connected:
                    self.connected_changed.emit(False)
                client.close()
            self.stop_event.wait(1.0)


class PoseWorker(QThread):
    """Optional extra credit: read-only Doosan controller Modbus TCP client."""
    poses_ready = pyqtSignal(list, list)
    connection_changed = pyqtSignal(bool)
    failed = pyqtSignal(str)

    def __init__(self, host, port, parent=None):
        super().__init__(parent)
        self.host, self.port = host, port
        self.stop_event = threading.Event()

    def stop(self):
        self.stop_event.set()

    def run(self):
        while not self.stop_event.is_set():
            client = ModbusTcpClient(self.host, port=self.port, timeout=1.5, retries=0)
            connected = False
            try:
                if not client.connect():
                    raise ConnectionError(f"로봇 컨트롤러 {self.host}:{self.port} 접속 실패")
                connected = True
                self.connection_changed.emit(True)
                while not self.stop_event.is_set():
                    joints = ModbusWorker.read_registers(client, 270, 6)
                    xyz = ModbusWorker.read_registers(client, 400, 3)
                    self.poses_ready.emit(
                        [signed16(r) / 10.0 for r in joints],
                        [signed16(r) / 10.0 for r in xyz]
                    )
                    self.stop_event.wait(0.5)
            except Exception as exc:
                if not self.stop_event.is_set():
                    self.failed.emit(f"로봇 자세 모니터: {exc}")
            finally:
                if connected:
                    self.connection_changed.emit(False)
                client.close()
            self.stop_event.wait(1.0)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("광운대학교 Quiz 3 · Robot + Modbus Monitor")
        self.resize(1100, 860)

        self.modbus = None
        self.modbus_connected = False
        self.robot_connected = False
        self.robot_ready = False
        self.busy = False
        self.active_task = None
        self.pending_close = False
        self.pose_worker = None
        self.compliance_start = {}
        self.compliance_elapsed = {}
        self.measured_ids = set()
        self.motion_tag_count = 0
        self.last_modbus_error = ""
        self.last_pose_error = ""
        self.next_operation_id = 0
        self.pending_measurement_writes = set()
        self.pending_increment_id = None
        self.reserved_measurement_addresses = []
        self.awaiting_capacity = False
        self.robot_finished = False
        self.record_error = ""
        self.task_started_at = None
        self.timeout_warned = False

        root = QWidget()
        self.setCentralWidget(root)
        outer = QVBoxLayout(root)
        outer.setContentsMargins(16, 12, 16, 12)
        outer.setSpacing(10)

        # 1) Separate Python Modbus server connection (not robot controller).
        modbus_box = QGroupBox("1. 별도 Python Modbus 서버 연결 (Holding Register 1~20)")
        modbus_row = QHBoxLayout(modbus_box)
        self.host = QLineEdit("127.0.0.1")
        self.host.setFixedWidth(150)
        self.port = QSpinBox()
        self.port.setRange(1, 65535)
        self.port.setValue(rm.MODBUS_PORT)
        self.modbus_button = QPushButton("Modbus 연결")
        self.modbus_button.clicked.connect(self.toggle_modbus)
        self.modbus_state = QLabel("미연결")
        modbus_row.addWidget(QLabel("IP"))
        modbus_row.addWidget(self.host)
        modbus_row.addWidget(QLabel("Port"))
        modbus_row.addWidget(self.port)
        modbus_row.addWidget(self.modbus_button)
        modbus_row.addWidget(self.modbus_state, 1)
        outer.addWidget(modbus_box)

        # 2) Quiz2 DRL connects to PC port 20000 (raw socket, NOT Modbus).
        robot_box = QGroupBox("2. 로봇 DRL Socket 연결 (로봇 → PC:20000)")
        robot_row = QHBoxLayout(robot_box)
        self.robot_state = QLabel("접속 대기 중 · DRL PC_IP를 이 컴퓨터의 IP로 설정")
        robot_row.addWidget(self.robot_state)
        outer.addWidget(robot_box)

        # 3) Task commands are only sent when both connections are ready.
        task_box = QGroupBox("3. 로봇 작업 실행")
        task_row = QHBoxLayout(task_box)
        self.task1_button = QPushButton("작업1 · Pick & Place")
        self.task2_button = QPushButton("작업2 · 높이 측정/정렬")
        self.task1_button.clicked.connect(lambda: self.start_task("start1"))
        self.task2_button.clicked.connect(lambda: self.start_task("start2"))
        # "end": Quiz 2 DRL leaves its command loop and closes the socket.
        self.end_button = QPushButton("로봇 프로그램 종료 (end)")
        self.end_button.clicked.connect(self.send_end)
        self.task_state = QLabel("대기")
        task_row.addWidget(self.task1_button)
        task_row.addWidget(self.task2_button)
        task_row.addWidget(self.end_button)
        task_row.addWidget(QLabel("작업 상태:"))
        task_row.addWidget(self.task_state, 1)
        outer.addWidget(task_box)

        # 4) All 20 Modbus registers shown with logical/protocol addresses.
        register_box = QGroupBox("4. Modbus Holding Register 모니터")
        register_layout = QVBoxLayout(register_box)
        self.register_table = QTableWidget(rm.REGISTER_COUNT, 3)
        self.register_table.setHorizontalHeaderLabels(
            ["논리 주소", "통신 주소 (0 기반)", "현재 값"]
        )
        self.register_table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.register_table.verticalHeader().setVisible(False)
        self.register_table.horizontalHeader().setStretchLastSection(True)
        for row in range(rm.REGISTER_COUNT):
            self.register_table.setItem(row, 0, QTableWidgetItem(f"{row+1} · {rm.label(row)}"))
            self.register_table.setItem(row, 1, QTableWidgetItem(str(row)))
            self.register_table.setItem(row, 2, QTableWidgetItem("—"))
        self.register_table.setMinimumHeight(155)
        register_layout.addWidget(self.register_table)
        outer.addWidget(register_box, 2)

        # 5) CSV results (bonus point) mirrored in GUI.
        csv_box = QGroupBox("5. 작업2 CSV 측정 결과")
        csv_layout = QVBoxLayout(csv_box)
        self.csv_table = QTableWidget(0, len(CSV_COLUMNS))
        self.csv_table.setHorizontalHeaderLabels(CSV_COLUMNS)
        self.csv_table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.csv_table.horizontalHeader().setStretchLastSection(True)
        self.csv_table.setMinimumHeight(120)
        csv_layout.addWidget(self.csv_table)
        outer.addWidget(csv_box, 2)

        # 6) Optional extra bonus: robot controller's own Modbus registers.
        pose_box = QGroupBox("6. 추가 점수 · 로봇 실제 J1~J6 / X·Y·Z 모니터 (별도 연결)")
        pose_layout = QVBoxLayout(pose_box)
        pose_controls = QHBoxLayout()
        self.pose_host = QLineEdit("192.168.137.106")
        self.pose_host.setFixedWidth(150)
        self.pose_port = QSpinBox()
        self.pose_port.setRange(1, 65535)
        self.pose_port.setValue(502)
        self.pose_button = QPushButton("로봇 자세 읽기")
        self.pose_button.clicked.connect(self.toggle_pose)
        self.pose_state = QLabel("미연결")
        for control in (QLabel("로봇 IP"), self.pose_host, QLabel("Port"),
                        self.pose_port, self.pose_button, self.pose_state):
            pose_controls.addWidget(control)
        pose_controls.addStretch()
        pose_layout.addLayout(pose_controls)
        self.pose_label = QLabel("J1 ~ J6: —    |    X / Y / Z: —")
        self.pose_label.setWordWrap(True)
        pose_layout.addWidget(self.pose_label)
        outer.addWidget(pose_box)

        # 7) Work log.
        log_box = QGroupBox("7. 작업 로그")
        log_layout = QVBoxLayout(log_box)
        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMinimumHeight(110)
        log_layout.addWidget(self.log_view)
        outer.addWidget(log_box, 2)
        self.load_existing_csv()

        self.setStyleSheet("""
            QGroupBox {font-weight: bold; border: 1px solid #b9c7d2;
                border-radius: 6px; margin-top: 7px; padding-top: 8px;}
            QPushButton {padding: 6px 12px; background: #143e57; color: white;
                border-radius: 5px;}
            QPushButton:disabled {background: #9daeb7;}
            QLineEdit, QSpinBox {padding: 4px;}
        """)

        self.robot = RobotSocketWorker(self)
        self.robot.peer_changed.connect(self.on_robot_peer)
        self.robot.received.connect(self.on_robot_message)
        self.robot.failed.connect(self.on_robot_error)
        self.robot.finished.connect(self.maybe_close)
        self.robot.start()
        self.watchdog = QTimer(self)
        self.watchdog.timeout.connect(self.check_task_timeout)
        self.watchdog.start(1000)
        self.refresh_buttons()
        self.log("PC 로봇 Socket Server 시작: 0.0.0.0:20000")

    def log(self, message):
        self.log_view.append(f"[{now_str()}] {message}")

    def refresh_buttons(self):
        can_start = (self.modbus_connected and self.robot_connected
                     and self.robot_ready and not self.busy and not self.pending_close)
        self.task1_button.setEnabled(can_start)
        self.task2_button.setEnabled(can_start)
        # "end" needs only the robot link (no Modbus record), never during a job.
        self.end_button.setEnabled(self.robot_connected and self.robot_ready
                                   and not self.busy and not self.pending_close)
        # The Modbus worker must not be manually stopped during an active job.
        self.modbus_button.setEnabled(not self.busy and not self.pending_close)

    def send_end(self):
        if not (self.robot_connected and self.robot_ready and not self.busy):
            return
        reply = QMessageBox.question(
            self, "로봇 프로그램 종료",
            "로봇 DRL 프로그램에 end를 보내 명령 대기를 종료합니다.\n"
            "다시 작업하려면 로봇에서 DRL을 재실행해야 합니다. 계속할까요?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if reply != QMessageBox.StandardButton.Yes:
            return
        self.robot_ready = False   # no more task commands to this DRL session
        self.robot.send_command("end")
        self.robot_state.setText("end 전송 · 로봇 프로그램 종료 대기")
        self.log("[TX] end")
        self.refresh_buttons()

    def new_operation_id(self):
        self.next_operation_id += 1
        return self.next_operation_id

    def check_task_timeout(self):
        # Warning only. A timeout is NOT evidence the physical robot stopped.
        if (self.busy and self.task_started_at is not None and
                not self.timeout_warned and
                time.monotonic() - self.task_started_at > 180):
            self.timeout_warned = True
            self.log("[경고] 작업 응답 180초 지연: 로봇 실제 상태 확인 필요. "
                     "작업 버튼은 안전을 위해 계속 잠급니다.")
            self.task_state.setText("응답 지연 · 실제 로봇 상태 확인 필요")

    def toggle_modbus(self):
        if self.modbus is not None:
            self.modbus_button.setEnabled(False)
            self.modbus.stop()
            return
        host = self.host.text().strip()
        if not host:
            QMessageBox.warning(self, "IP 누락", "Modbus 서버 IP를 입력해 주세요.")
            return
        self.modbus = ModbusWorker(host, self.port.value(), self)
        self.modbus.connected_changed.connect(self.on_modbus_connection)
        self.modbus.values_ready.connect(self.update_registers)
        self.modbus.write_done.connect(self.on_modbus_write_done)
        self.modbus.capacity_ready.connect(self.on_capacity_ready)
        self.modbus.failed.connect(self.on_modbus_error)
        self.modbus.finished.connect(self.on_modbus_finished)
        self.modbus_state.setText("접속 시도 중…")
        self.modbus_button.setText("Modbus 해제")
        self.host.setEnabled(False)
        self.port.setEnabled(False)
        self.modbus.start()

    def on_modbus_connection(self, connected):
        self.modbus_connected = connected
        self.modbus_state.setText("연결됨" if connected else "미연결 / 자동 재시도")
        if connected:
            self.last_modbus_error = ""
            self.log("[Modbus] 서버 연결 완료")
        elif self.busy:
            self.task_state.setText("Modbus 통신 복구/미확정 기록 확인 중")
        self.refresh_buttons()

    def on_modbus_error(self, message):
        if message != self.last_modbus_error:
            self.log("[오류] " + message)
            self.last_modbus_error = message

    def update_registers(self, values):
        for i, value in enumerate(values[:rm.REGISTER_COUNT]):
            self.register_table.item(i, 2).setText(rm.describe(i, value))

    def on_modbus_finished(self):
        self.modbus.deleteLater()
        self.modbus = None
        self.modbus_connected = False
        self.modbus_state.setText("미연결")
        self.modbus_button.setText("Modbus 연결")
        self.modbus_button.setEnabled(True)
        self.host.setEnabled(True)
        self.port.setEnabled(True)
        for i in range(rm.REGISTER_COUNT):
            self.register_table.item(i, 2).setText("—")
        if self.busy:
            self.record_error = "Modbus 작업자 종료로 기록 결과 미확정"
            self.finalize_task(False, self.record_error)
        self.refresh_buttons()
        self.maybe_close()

    def on_robot_peer(self, connected, detail):
        self.robot_connected = connected
        self.robot_ready = False
        self.robot_state.setText(f"로봇 연결됨: {detail} / READY 대기" if connected
                                 else "로봇 접속 대기 중 (또는 연결 끊김)")
        if not connected and self.busy and not self.robot_finished:
            self.record_error = "로봇 Socket 통신 단절 · 실제 로봇 상태 확인 필요"
            # Keep task locked until outstanding measurements are resolved.
            self.robot_finished = True
            if not self.pending_measurement_writes and self.pending_increment_id is None:
                self.finalize_task(False, self.record_error)
        self.log("로봇 Socket " + (f"접속: {detail}" if connected else "연결 종료"))
        self.refresh_buttons()

    def on_robot_error(self, message):
        self.log("[로봇 소켓 오류] " + message)
        self.robot_state.setText(message)
        self.refresh_buttons()

    def start_task(self, command):
        if not (self.robot_ready and self.robot_connected
                and self.modbus_connected and not self.busy
                and not self.pending_close):
            return
        self.busy = True
        self.active_task = command
        self.task_started_at = time.monotonic()
        self.timeout_warned = False
        self.robot_finished = False
        self.record_error = ""
        self.awaiting_capacity = command == "start2"
        self.reserved_measurement_addresses = []
        self.pending_measurement_writes.clear()
        self.pending_increment_id = None
        self.motion_tag_count = 0
        self.compliance_start.clear()
        self.compliance_elapsed.clear()
        self.measured_ids.clear()
        self.refresh_buttons()
        if command == "start2":
            # Check all 3 slots BEFORE any robot movement for this task.
            self.task_state.setText("작업2 준비 · 측정 공간 3칸 확인 중")
            self.modbus.reserve_measurements()
        else:
            self.dispatch_robot_task()

    def dispatch_robot_task(self):
        command = self.active_task
        self.task_state.setText("작업 중 · " + command)
        self.robot.send_command(command)
        self.log("[TX] " + command)

    def on_capacity_ready(self, available, addresses, message):
        if self.active_task != "start2" or not self.awaiting_capacity:
            return
        self.awaiting_capacity = False
        self.log("[Modbus 사전 확인] " + message)
        if not available or len(addresses) != rm.MEAS_PER_TASK2:
            self.finalize_task(False, message)
            return
        if not self.robot_connected or not self.robot_ready or not self.modbus_connected:
            self.finalize_task(False, "통신 연결이 끊겨 작업2를 시작하지 않음")
            return
        self.reserved_measurement_addresses = addresses
        self.dispatch_robot_task()

    def on_modbus_write_done(self, operation_id, success, message):
        self.log(("[Modbus WRITE] " if success else "[Modbus 기록 오류] ") + message)
        if operation_id in self.pending_measurement_writes:
            self.pending_measurement_writes.remove(operation_id)
            if not success:
                self.record_error = message
            if self.robot_finished and not self.pending_measurement_writes:
                self.try_finish_robot_task()
        elif operation_id == self.pending_increment_id:
            self.pending_increment_id = None
            self.finalize_task(success, message if not success else "기록 검증 완료")

    def try_finish_robot_task(self):
        if not self.robot_finished or self.pending_measurement_writes:
            self.task_state.setText("로봇 완료 · Modbus 기록 검증 중")
            return
        if self.record_error:
            self.finalize_task(False, self.record_error)
            return
        if self.pending_increment_id is not None:
            return
        if self.modbus is None:
            self.finalize_task(False, "Modbus 작업자 없음: 수행 횟수 기록 불가")
            return
        address = rm.count_address(1 if self.active_task == "start1" else 2)
        op_id = self.new_operation_id()
        self.pending_increment_id = op_id
        self.task_state.setText("로봇 완료 · 수행 횟수 쓰기/재조회 중")
        self.modbus.increment(address, op_id)

    def finalize_task(self, success, message=""):
        command = self.active_task or "알 수 없음"
        self.busy = False
        self.active_task = None
        self.awaiting_capacity = False
        self.robot_finished = False
        self.task_started_at = None
        self.task_state.setText("완료" if success else "오류: " + message)
        self.refresh_buttons()
        self.log(f"[작업 {'완료' if success else '실패'}] {command}" +
                 (f" · {message}" if message else ""))

    def on_robot_message(self, line):
        self.log("[ROBOT RX] " + line)
        if line == "READY":
            self.robot_ready = True
            self.robot_state.setText("로봇 READY · 작업 명령 대기")
            self.refresh_buttons()
            return
        if line in {"MoveJ Start", "MoveJ 종료", "MoveL Start", "MoveL 종료",
                    "Gripper ON", "Gripper OFF", "Gripper On", "Gripper Off"}:
            self.motion_tag_count += 1
            return
        if line.startswith("COMPLIANCE Start,"):
            try:
                job_id = int(line.split(",", 1)[1])
                self.compliance_start[job_id] = time.perf_counter()
            except ValueError:
                self.log("[오류] 순응제어 시작 메시지 파싱 실패")
            return
        if line.startswith("COMPLIANCE End,"):
            try:
                job_id = int(line.split(",", 1)[1])
                if job_id in self.compliance_start:
                    self.compliance_elapsed[job_id] = (time.perf_counter()
                                                       - self.compliance_start[job_id])
            except ValueError:
                self.log("[오류] 순응제어 종료 메시지 파싱 실패")
            return
        if line.startswith("DATA,"):
            self.record_measurement(line)
            return
        if line.startswith("ORDER,"):
            self.log("[배치 순서] " + line.replace("ORDER,", ""))
            return
        if line.startswith("ERROR,"):
            self.log("[로봇 측정 실패] " + line)
            return
        if line.startswith("FAIL "):
            self.complete_task(False, line)
            return
        if line.startswith("DONE "):
            self.complete_task(True, line)
            return

    def record_measurement(self, message):
        if self.active_task != "start2" or self.robot_finished:
            self.log("[경고] 작업2 실행 구간 외 DATA 메시지 무시")
            return
        try:
            parts = message.split(",")
            if len(parts) != 6:
                raise ValueError("필드 수가 6개가 아님")
            job_id = int(parts[1])
            x, y, z, height = map(float, parts[2:6])
            if not 1 <= job_id <= rm.MEAS_PER_TASK2 or height <= 0:
                raise ValueError("작업 ID 또는 높이가 범위를 벗어남")
            if job_id in self.measured_ids:
                self.log(f"[경고] 작업ID {job_id}: 중복 DATA")
                return
            if job_id not in self.compliance_elapsed:
                raise ValueError("순응제어 시작·종료 시간 누락")
            elapsed = self.compliance_elapsed[job_id]
            row = [job_id, round(x), round(y), round(z),
                   f"{height:.2f}", now_str(), f"{elapsed:.3f}"]
            rm.mm_to_raw(height)   # ValueError if out of the 16-bit register range
            if len(self.reserved_measurement_addresses) != rm.MEAS_PER_TASK2:
                raise ValueError("작업2 측정 공간이 예약되지 않음")
            self.append_csv(row)
            self.add_csv_row(row)
            self.measured_ids.add(job_id)
            if self.modbus is not None:
                op_id = self.new_operation_id()
                self.pending_measurement_writes.add(op_id)
                address = self.reserved_measurement_addresses[job_id - 1]
                self.modbus.append_measurement(address, height, op_id)
            else:
                self.record_error = "Modbus 작업자가 종료되어 측정값 기록 불가"
            self.log(f"[CSV] 작업 {job_id}, 높이={height:.2f} mm, {elapsed:.3f} s")
        except (ValueError, OSError) as exc:
            self.record_error = f"DATA/CSV 처리 실패: {exc}"
            self.log(f"[CSV 오류] {self.record_error}")

    def append_csv(self, row):
        needs_header = not CSV_PATH.exists() or CSV_PATH.stat().st_size == 0
        with CSV_PATH.open("a", encoding="utf-8-sig", newline="") as f:
            writer = csv.writer(f)
            if needs_header:
                writer.writerow(CSV_COLUMNS)
            writer.writerow(row)

    def add_csv_row(self, row):
        idx = self.csv_table.rowCount()
        self.csv_table.insertRow(idx)
        for col, value in enumerate(row):
            self.csv_table.setItem(idx, col, QTableWidgetItem(str(value)))
        self.csv_table.scrollToBottom()

    def load_existing_csv(self):
        if not CSV_PATH.exists():
            return
        try:
            with CSV_PATH.open("r", encoding="utf-8-sig", newline="") as f:
                for row in list(csv.reader(f))[1:]:
                    if len(row) == len(CSV_COLUMNS):
                        self.add_csv_row(row)
        except OSError as exc:
            self.log(f"CSV 읽기 오류: {exc}")

    def complete_task(self, success, message):
        command = message.split(" ", 1)[-1]
        if self.active_task != command:
            self.log(f"[경고] 진행 중인 작업과 완료 응답 불일치: {message}")
            return
        if success and command == "start2" and len(self.measured_ids) != rm.MEAS_PER_TASK2:
            success = False
            self.log("[오류] 작업2는 DONE이지만 유효한 측정 결과가 3개가 아님")
        if success and command == "start1" and self.motion_tag_count not in (24, 28):
            self.log(f"[참고] 작업1 모션/그리퍼 로그 {self.motion_tag_count}개 "
                     "(DRL 버전에 따라 24개 또는 28개)")
        if not success:
            self.record_error = message
        self.robot_finished = True
        self.try_finish_robot_task()

    def toggle_pose(self):
        if self.pose_worker is not None:
            self.pose_button.setEnabled(False)
            self.pose_worker.stop()
            return
        host = self.pose_host.text().strip()
        if not host:
            return
        self.pose_worker = PoseWorker(host, self.pose_port.value(), self)
        self.pose_worker.poses_ready.connect(self.show_pose)
        self.pose_worker.connection_changed.connect(
            lambda yes: self.pose_state.setText("연결됨" if yes else "연결 중 / 재시도"))
        self.pose_worker.failed.connect(self.on_pose_error)
        self.pose_worker.finished.connect(self.on_pose_finished)
        self.pose_button.setText("자세 읽기 중지")
        self.pose_host.setEnabled(False)
        self.pose_port.setEnabled(False)
        self.pose_worker.start()

    def show_pose(self, joints, xyz):
        j_text = "  ".join(f"J{i + 1}={angle:.1f}°" for i, angle in enumerate(joints))
        xyz_text = "  ".join(
            f"{axis}={value:.1f} mm" for axis, value in zip("XYZ", xyz, strict=True)
        )
        self.pose_label.setText(j_text + "\n" + xyz_text)
        self.last_pose_error = ""

    def on_pose_error(self, message):
        if self.last_pose_error != message:
            self.log("[오류] " + message)
            self.last_pose_error = message

    def on_pose_finished(self):
        self.pose_worker.deleteLater()
        self.pose_worker = None
        self.pose_button.setEnabled(True)
        self.pose_button.setText("로봇 자세 읽기")
        self.pose_host.setEnabled(True)
        self.pose_port.setEnabled(True)
        self.pose_state.setText("미연결")
        self.maybe_close()

    def maybe_close(self):
        if (self.pending_close and not self.robot.isRunning() and
                self.modbus is None and self.pose_worker is None):
            self.close()

    def closeEvent(self, event):
        if (self.robot.isRunning() or self.modbus is not None or
                self.pose_worker is not None):
            if self.busy and not self.pending_close:
                reply = QMessageBox.question(
                    self, "로봇 작업 중",
                    "로봇 작업 중 프로그램을 닫으면 통신이 끊기지만, "
                    "로봇이 정지한다는 보장은 없습니다. 정말 닫으시겠습니까?",
                    QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                    QMessageBox.StandardButton.No
                )
                if reply != QMessageBox.StandardButton.Yes:
                    event.ignore()
                    return
            self.pending_close = True
            self.robot.stop()
            if self.modbus is not None:
                self.modbus.stop()
            if self.pose_worker is not None:
                self.pose_worker.stop()
            event.ignore()
        else:
            event.accept()


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
