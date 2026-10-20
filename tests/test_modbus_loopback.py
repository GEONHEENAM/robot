"""PC 클라이언트 <-> 가짜 서버 루프백 테스트 (실제 로봇 불필요)."""

import pytest

from robot_comm.modbus_client import RobotModbusClient, RobotModbusError
from robot_comm.sim_server import SimServerThread

PORT = 5020


@pytest.fixture(scope="module")
def server():
    srv = SimServerThread(port=PORT, size=100)
    srv.start()
    yield srv
    srv.stop()


@pytest.fixture
def client(server):
    with RobotModbusClient("127.0.0.1", port=PORT) as c:
        yield c


def test_write_then_read(client):
    client.write_register(10, 1234)
    assert client.read_registers(10, 1) == [1234]


def test_read_multiple(client):
    for i, v in enumerate([1, 2, 3]):
        client.write_register(20 + i, v)
    assert client.read_registers(20, 3) == [1, 2, 3]


def test_address_boundary(client):
    # size=100 이면 주소 0~99만 유효해야 한다 (off-by-one 회귀 방지)
    client.write_register(0, 7)
    client.write_register(99, 9)
    assert client.read_registers(0, 1) == [7]
    assert client.read_registers(99, 1) == [9]
    with pytest.raises(RobotModbusError):
        client.read_registers(100, 1)


def test_value_out_of_range(client):
    with pytest.raises(ValueError):
        client.write_register(0, 70000)


def test_connect_fails_without_server():
    c = RobotModbusClient("127.0.0.1", port=PORT + 1, timeout=0.3)
    assert c.connect() is False
    c.close()
