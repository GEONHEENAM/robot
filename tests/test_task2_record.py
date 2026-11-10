"""퀴즈2 CSV 기록 규칙 테스트 (로봇·PyQt 불필요)."""

import csv
import datetime as dt

import pytest

from robot_comm.task2_record import (
    CSV_HEADER,
    JobRecordError,
    Task2Recorder,
    append_row,
    motion_event,
)

T0 = dt.datetime(2026, 11, 10, 15, 0, 0)
JOB1 = "1,52.25,200,500,52.25,0,-180,0,200,600,54.25,0,-180,0"


def at(sec):
    return T0 + dt.timedelta(seconds=sec)


def run_job(rec, body=JOB1, job_id=1, start=1.0, end=9.5):
    """DRL 순서대로 한 작업을 흘려보낸다."""
    rec.start_job(body)
    rec.on_line("Gripper OFF", at(start - 0.5))
    rec.on_line("MoveJ Start", at(start))
    rec.on_line("MoveJ 종료", at(start + 1))
    rec.on_line("MoveL Start", at(start + 1.1))
    rec.on_line("MoveL 종료", at(start + 2))
    rec.on_line("MoveJ Start", at(start + 3))     # 두 번째 MoveJ Start는 시작 시각을 바꾸지 않음
    rec.on_line("MoveL Start", at(end - 1))
    rec.on_line("MoveL 종료", at(end))            # 마지막 MoveL 종료 = 종료 시각
    return rec.finish_job(str(job_id))


def test_complete_job_row():
    rec = Task2Recorder()
    row = run_job(rec)
    assert len(row) == len(CSV_HEADER)
    assert row[0] == "1"
    assert row[1] == "52.25"
    assert row[2:8] == ["200.00", "500.00", "52.25", "0.00", "-180.00", "0.00"]
    assert row[8:14] == ["200.00", "600.00", "54.25", "0.00", "-180.00", "0.00"]
    assert row[14] == "2026-11-10 15:00:01.000"
    assert row[15] == "2026-11-10 15:00:09.500"
    assert row[16] == "8.500"


def test_missing_movel_end_not_saved():
    rec = Task2Recorder()
    rec.start_job(JOB1)
    rec.on_line("MoveJ Start", at(1))
    with pytest.raises(JobRecordError):
        rec.finish_job("1")
    assert rec.saved_ids == set()


def test_movel_end_before_start_is_ignored():
    """MoveJ Start 전에 온 이동 완료(이전 작업 잔여)는 종료 시각으로 쓰지 않는다."""
    rec = Task2Recorder()
    rec.start_job(JOB1)
    rec.on_line("MoveL 종료", at(0.5))
    rec.on_line("MoveJ Start", at(1))
    with pytest.raises(JobRecordError, match="이동 완료"):
        rec.finish_job("1")


def test_pr13_messages_and_movej_final_lift():
    """PR #13 DRL 형식: Done 메시지, MoveL_REL, 놓은 뒤 상승이 move_jx(MoveJ).

    종료 시각은 JOB_DONE 직전의 마지막 '이동 완료'(여기서는 MoveJ Done)여야 한다.
    """
    rec = Task2Recorder()
    rec.start_job(JOB1)
    rec.on_line("Gripper OFF", at(0.5))
    rec.on_line("MoveJ Start", at(1))          # 시작
    rec.on_line("MoveJ Done", at(2))
    rec.on_line("MoveL Start", at(2.1))
    rec.on_line("MoveL Done", at(3))
    rec.on_line("Gripper ON", at(4))
    rec.on_line("MoveL_REL Start", at(4.1))
    rec.on_line("MoveL_REL Done", at(5))
    rec.on_line("MoveJ Start", at(6))
    rec.on_line("MoveJ Done", at(7))
    rec.on_line("MoveL Start", at(7.1))
    rec.on_line("MoveL Done", at(8))           # 놓기 위치 하강 완료 (상승 전)
    rec.on_line("Gripper OFF", at(9))
    rec.on_line("MoveJ Start", at(9.1))
    rec.on_line("MoveJ Done", at(10))          # 상승 완료 = 종료
    row = rec.finish_job("1")
    assert row[14] == "2026-11-10 15:00:01.000"
    assert row[15] == "2026-11-10 15:00:10.000"
    assert row[16] == "9.000"


def test_first_start_must_be_movej():
    """퀴즈2 규칙: 시작 시각은 첫 'MoveJ Start'. MoveL Start로는 시작하지 않는다."""
    rec = Task2Recorder()
    rec.start_job(JOB1)
    rec.on_line("MoveL Start", at(0.5))
    rec.on_line("MoveL 종료", at(0.8))
    rec.on_line("MoveJ Start", at(1))
    rec.on_line("MoveJ 종료", at(2))
    row = rec.finish_job("1")
    assert row[14].endswith("15:00:01.000")
    assert row[15].endswith("15:00:02.000")


@pytest.mark.parametrize("line, expected", [
    ("MoveJ Start", ("J", "start")),
    ("MoveJ 종료", ("J", "end")),
    ("MoveJ Done", ("J", "end")),
    ("MoveL Start", ("L", "start")),
    ("MoveL 종료", ("L", "end")),
    ("MoveL Done", ("L", "end")),
    ("MoveL_REL Start", ("L_REL", "start")),
    ("MoveL_REL Done", ("L_REL", "end")),
    ("  MoveJ Done ", ("J", "end")),
    ("Gripper ON", None),
    ("MoveJ", None),
    ("MoveJ Started", None),
    ("TASK 1 Accepted", None),
])
def test_motion_event(line, expected):
    assert motion_event(line) == expected


def test_job_id_mismatch_not_saved():
    rec = Task2Recorder()
    rec.start_job(JOB1)
    rec.on_line("MoveJ Start", at(1))
    rec.on_line("MoveL 종료", at(2))
    with pytest.raises(JobRecordError, match="불일치"):
        rec.finish_job("2")


def test_duplicate_job_rejected():
    rec = Task2Recorder()
    run_job(rec)
    with pytest.raises(JobRecordError, match="이미 저장"):
        rec.start_job(JOB1)


def test_new_job_before_done_warns_and_drops_previous():
    rec = Task2Recorder()
    rec.start_job(JOB1)
    warning = rec.start_job(JOB1.replace("1,", "2,", 1))
    assert warning and "작업 1" in warning
    assert rec.job.job_id == 2


def test_abort_drops_current_job():
    rec = Task2Recorder()
    rec.start_job(JOB1)
    rec.on_line("MoveJ Start", at(1))
    assert rec.abort() == 1
    assert rec.job is None
    with pytest.raises(JobRecordError):
        rec.finish_job("1")


@pytest.mark.parametrize("body", ["1,2,3", "x," + JOB1[2:], JOB1 + ",9"])
def test_bad_job_message(body):
    with pytest.raises(JobRecordError):
        Task2Recorder().start_job(body)


def _read(path):
    with path.open(newline="", encoding="utf-8-sig") as f:
        return list(csv.reader(f))


def test_append_writes_header_once(tmp_path):
    path = tmp_path / "task2.csv"
    rec = Task2Recorder()
    rows = [
        run_job(rec, JOB1, 1),
        run_job(rec, JOB1.replace("1,", "2,", 1), 2),
        run_job(rec, JOB1.replace("1,", "3,", 1), 3),
    ]
    for r in rows:
        assert append_row(path, r) is None
    data = _read(path)
    assert data[0] == CSV_HEADER
    assert data[1:] == rows                  # 열 제목 제외 3행, 누락·중복 없음
    assert path.read_bytes().count(b"\xef\xbb\xbf") == 1   # BOM은 맨 앞에 한 번만


def test_append_continues_existing_file(tmp_path):
    path = tmp_path / "task2.csv"
    row = run_job(Task2Recorder())
    append_row(path, row)
    append_row(path, row)                    # 프로그램을 다시 켠 뒤 이어 쓰는 경우
    data = _read(path)
    assert data.count(CSV_HEADER) == 1
    assert len(data) == 3


def test_old_format_file_is_backed_up(tmp_path):
    path = tmp_path / "task2.csv"
    path.write_text("job_id,x,y,approach_z,measured_z\n1,0,500,100,52.3\n", encoding="utf-8")
    row = run_job(Task2Recorder())
    backup = append_row(path, row)
    assert backup is not None and backup.exists()
    assert "measured_z" in backup.read_text(encoding="utf-8")
    assert _read(path) == [CSV_HEADER, row]
