"""퀴즈2 작업 결과 CSV 기록 (PyQt 없이 테스트 가능하게 분리).

로봇(DRL) → PC 메시지 흐름 (작업 2, 물체 1개 = 작업 1개)
    JOB,id,length,px,py,pz,prx,pry,prz,qx,qy,qz,qrx,qry,qrz   작업 번호·길이·Pick/Place 목표점
    MoveJ Start          ← 이 작업의 첫 MoveJ Start 수신 시각 = 시작 시각
    ... (MoveL / Gripper 메시지)
    MoveL 종료           ← 마지막 MoveL 종료 수신 시각 = 종료 시각 (놓고 상승 완료)
    JOB_DONE,id          ← 필수 데이터 확인 후 CSV 한 행 추가

규칙 (퀴즈2 문제지)
    - 작업 1회당 한 행, 정상 종료 시 한 번만 저장
    - 중단되거나 필수 데이터가 빠진 작업은 저장하지 않음
    - CSV는 최초 생성 시에만 열 제목을 쓰고 이후 한 행씩 추가
    - 시각은 PC 메시지 수신 기준, 소요 시간은 초(통신 지연 포함)
"""

from __future__ import annotations

import csv
import datetime as dt
from dataclasses import dataclass
from pathlib import Path

MSG_MOVEJ_START = "MoveJ Start"
MSG_MOVEL_END = "MoveL 종료"

AXES = ("x", "y", "z", "rx", "ry", "rz")
CSV_HEADER = (
    ["job_id", "length_mm"]
    + [f"pick_{a}" for a in AXES]
    + [f"place_{a}" for a in AXES]
    + ["start_time", "end_time", "duration_s"]
)
JOB_FIELD_COUNT = 2 + 6 + 6   # id, length, pick 6, place 6
TIME_FMT = "%Y-%m-%d %H:%M:%S.%f"


class JobRecordError(Exception):
    """작업 데이터가 잘못되었거나 빠져서 저장하지 않는 경우."""


@dataclass
class Task2Job:
    job_id: int
    length: float
    pick: list[float]
    place: list[float]
    start: dt.datetime | None = None
    end: dt.datetime | None = None


def _fmt_time(t: dt.datetime) -> str:
    return t.strftime(TIME_FMT)[:-3]   # 밀리초까지


class Task2Recorder:
    """작업 2 메시지를 받아 작업별 CSV 행을 만든다. 시각은 호출하는 쪽이 넘긴다."""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        """작업 2 한 번 실행을 시작할 때 호출한다."""
        self.job: Task2Job | None = None
        self.saved_ids: set[int] = set()

    def start_job(self, body: str) -> str | None:
        """JOB 메시지 본문(쉼표 뒤)을 받는다. 이전 작업이 끝나지 않았으면 경고 문자열 반환."""
        cells = [c.strip() for c in body.split(",")]
        if len(cells) != JOB_FIELD_COUNT:
            raise JobRecordError(f"JOB 필드 수 {len(cells)}개 (필요 {JOB_FIELD_COUNT}개)")
        try:
            job_id = int(cells[0])
            nums = [float(c) for c in cells[1:]]
        except ValueError as e:
            raise JobRecordError(f"JOB 숫자 변환 실패: {body}") from e
        if job_id in self.saved_ids:
            raise JobRecordError(f"작업 {job_id}는 이미 저장됨 (중복 JOB)")

        warning = None
        if self.job is not None:
            warning = f"작업 {self.job.job_id}가 JOB_DONE 없이 끝나 저장하지 않음"
        self.job = Task2Job(job_id, nums[0], nums[1:7], nums[7:13])
        return warning

    def on_line(self, line: str, now: dt.datetime) -> None:
        """모든 수신 줄을 넘긴다. 진행 중인 작업의 시작·종료 시각만 갱신한다."""
        if self.job is None:
            return
        if line == MSG_MOVEJ_START and self.job.start is None:
            self.job.start = now
        elif line == MSG_MOVEL_END and self.job.start is not None:
            self.job.end = now   # 계속 덮어써서 '마지막' MoveL 종료가 남는다

    def finish_job(self, body: str) -> list[str]:
        """JOB_DONE 본문을 받아 CSV 한 행을 반환한다. 저장할 수 없으면 JobRecordError."""
        job, self.job = self.job, None
        try:
            done_id = int(body.strip())
        except ValueError as e:
            raise JobRecordError(f"JOB_DONE 형식 오류: {body}") from e
        if job is None:
            raise JobRecordError(f"작업 {done_id}: JOB 없이 JOB_DONE 수신")
        if job.job_id != done_id:
            raise JobRecordError(f"작업 번호 불일치: JOB {job.job_id} / JOB_DONE {done_id}")
        if job.start is None:
            raise JobRecordError(f"작업 {done_id}: MoveJ Start 미수신")
        if job.end is None:
            raise JobRecordError(f"작업 {done_id}: MoveL 종료 미수신")
        if done_id in self.saved_ids:
            raise JobRecordError(f"작업 {done_id}는 이미 저장됨")

        self.saved_ids.add(done_id)
        duration = (job.end - job.start).total_seconds()
        return (
            [str(job.job_id), f"{job.length:.2f}"]
            + [f"{v:.2f}" for v in job.pick]
            + [f"{v:.2f}" for v in job.place]
            + [_fmt_time(job.start), _fmt_time(job.end), f"{duration:.3f}"]
        )

    def abort(self) -> int | None:
        """작업 2가 중단됐을 때 호출. 버린 작업 번호(없으면 None)를 반환한다."""
        job, self.job = self.job, None
        return job.job_id if job else None


def append_row(path: Path | str, row: list[str]) -> Path | None:
    """CSV에 한 행을 추가한다. 파일이 없거나 비었으면 열 제목을 먼저 쓴다.

    기존 파일의 열 제목이 CSV_HEADER와 다르면(예전 형식) 섞이지 않게 기존 파일을
    이름을 바꿔 보관하고 새로 만든다. 보관한 경우 그 경로를 반환한다.
    """
    path = Path(path)
    backup = None
    if path.exists() and path.stat().st_size > 0:
        with path.open(newline="", encoding="utf-8-sig") as f:
            first = next(csv.reader(f), [])
        if first != CSV_HEADER:
            stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
            backup = path.with_name(f"{path.stem}_old_{stamp}{path.suffix}")
            path.replace(backup)

    write_header = not path.exists() or path.stat().st_size == 0
    # 열 제목을 처음 쓸 때만 BOM(utf-8-sig)을 붙인다 → 엑셀에서 한글/열 정상 표시
    encoding = "utf-8-sig" if write_header else "utf-8"
    with path.open("a", newline="", encoding=encoding) as f:
        w = csv.writer(f)
        if write_header:
            w.writerow(CSV_HEADER)
        w.writerow(row)
    return backup
