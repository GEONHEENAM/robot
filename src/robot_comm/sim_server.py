"""로봇 없이 개발/CI에서 쓰는 가짜 Modbus TCP 서버.

실제 컨트롤러를 흉내 내는 것이 아니라, '클라이언트 코드가 Modbus 프로토콜을
올바르게 주고받는가'만 검증하기 위한 용도다.

단독 실행:  python -m robot_comm.sim_server --port 5020
"""

from __future__ import annotations

import argparse
import asyncio
import threading

from pymodbus.server import ModbusTcpServer
from pymodbus.simulator import DataType, SimData, SimDevice


def build_context(size: int = 100, device_id: int = 1) -> SimDevice:
    # pymodbus 3.13부터 ModbusDeviceContext/ModbusSequentialDataBlock 방식은 deprecated이고
    # (v4에서 제거 예정) 실제로 주소 0 하나만 응답하는 문제가 있어 SimDevice/SimData를 쓴다.
    # 주소 0 ~ size-1 에 0으로 초기화된 16비트 레지스터 size개.
    return SimDevice(
        id=device_id,
        simdata=[SimData(0, count=size, values=0, datatype=DataType.REGISTERS)],
    )


class SimServerThread:
    """별도 스레드에서 asyncio 이벤트 루프로 서버를 돌린다(테스트용)."""

    def __init__(self, host: str = "192.168.0.0", port: int = 502, size: int = 100) -> None:
        self.host, self.port, self.size = host, port, size
        self._loop: asyncio.AbstractEventLoop | None = None
        self._server: ModbusTcpServer | None = None
        self._ready = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    async def _main(self) -> None:
        # ModbusTcpServer는 실행 중인 이벤트 루프 안에서 생성해야 한다(밖에서 만들면 RuntimeError).
        self._stop = asyncio.Event()
        self._server = ModbusTcpServer(build_context(self.size), address=(self.host, self.port))
        await self._server.listen()
        self._ready.set()
        await self._stop.wait()
        await self._server.shutdown()

    def _run(self) -> None:
        self._loop = asyncio.new_event_loop()
        try:
            self._loop.run_until_complete(self._main())
        finally:
            self._loop.close()

    def start(self, timeout: float = 5.0) -> None:
        self._thread.start()
        if not self._ready.wait(timeout):
            raise RuntimeError("시뮬레이션 서버가 시간 내에 시작되지 않았습니다.")

    def stop(self) -> None:
        if self._loop is not None and self._loop.is_running():
            self._loop.call_soon_threadsafe(self._stop.set)
        self._thread.join(timeout=5.0)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5020)
    args = parser.parse_args()

    async def serve() -> None:
        server = ModbusTcpServer(build_context(), address=(args.host, args.port))
        print(f"[SIM] Modbus TCP server on {args.host}:{args.port}  (Ctrl+C로 종료)")
        await server.serve_forever()

    try:
        asyncio.run(serve())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
