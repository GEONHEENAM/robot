"""두산 M0609 컨트롤러와 Modbus TCP로 통신하는 PC 측 클라이언트.

레지스터 주소 맵은 티치펜던트(또는 DART-Platform)의 Modbus 설정과 반드시 일치시켜야 한다.
여기서는 주소를 하드코딩하지 않고 호출하는 쪽에서 넘기도록 했다.
"""

from __future__ import annotations

from pymodbus.client import ModbusTcpClient
from pymodbus.exceptions import ModbusException


class RobotModbusError(RuntimeError):
    """Modbus 요청이 실패했을 때 올리는 예외."""


class RobotModbusClient:
    def __init__(
        self,
        host: str,
        port: int = 502,
        device_id: int = 1,
        timeout: float = 1.0,
    ) -> None:
        self._client = ModbusTcpClient(host, port=port, timeout=timeout, retries=1)
        self.device_id = device_id

    # --- 연결 관리 ---
    def connect(self) -> bool:
        return self._client.connect()

    def close(self) -> None:
        self._client.close()

    def __enter__(self) -> RobotModbusClient:
        if not self.connect():
            raise RobotModbusError("Modbus 서버에 연결하지 못했습니다.")
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # --- 레지스터 읽기/쓰기 ---
    def read_registers(self, address: int, count: int = 1) -> list[int]:
        try:
            rr = self._client.read_holding_registers(
                address, count=count, device_id=self.device_id
            )
        except ModbusException as e:
            raise RobotModbusError(f"read 실패 (addr={address}): {e}") from e
        if rr.isError():
            raise RobotModbusError(f"read 응답 오류 (addr={address}): {rr}")
        return list(rr.registers)

    def write_register(self, address: int, value: int) -> None:
        # Modbus 홀딩 레지스터는 16비트 부호 없는 정수(0~65535)만 담는다.
        if not 0 <= value <= 0xFFFF:
            raise ValueError(f"레지스터 값 범위(0~65535)를 벗어남: {value}")
        try:
            wr = self._client.write_register(address, value, device_id=self.device_id)
        except ModbusException as e:
            raise RobotModbusError(f"write 실패 (addr={address}): {e}") from e
        if wr.isError():
            raise RobotModbusError(f"write 응답 오류 (addr={address}): {wr}")
