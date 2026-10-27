# 퀴즈3 — Modbus TCP 서버

퀴즈3 구성 중 **별도 Python Modbus 서버** 부분이다. 서버는 Holding Register 20개를 들고 있고,
값은 팀 UI(`ui/quiz3_gui.py`)가 쓰고 다시 읽는다.

## 전체 구성

```
 PC                                                              로봇 컨트롤러
 ┌──────────────────────┐   Modbus TCP (502)  ┌──────────────────┐
 │ ui/quiz3_gui.py      │ ── 읽기/쓰기 ──────▶ │ quiz3_server.py  │──▶ quiz3_registers.json
 │ (PyQt UI, 클라이언트) │                      └──────────────────┘
 │                      │   TCP 소켓 (9000)                         ┌──────────────────────┐
 │                      │ ── TASK,1 / TASK,2 ─────────────────────▶ │ DRL (로봇이 서버)     │
 │                      │ ◀── READY / POS / DATA / DONE,n ───────── │                      │
 └──────────────────────┘                                           └──────────────────────┘
```

| 파일 | 역할 |
|---|---|
| `quiz3_server.py` | Modbus TCP 서버. Holding Register 20개, 쓰기마다 파일 저장 |
| `check_server.py` | 서버 값 확인 / 1초 감시 / 초기화 |
| `register_map.py` | 주소 배치 (UI의 레지스터 배치와 같아야 함) |

## 실행 순서 (Windows)

```bat
C:\work_kw\kw2026\Scripts\activate
cd C:\work_kw\robot\quizzes\quiz3

:: 1) 서버 (시연 전에는 --reset 으로 0부터)
python quiz3_server.py --reset

:: 2) 다른 cmd 창: UI 실행 → Modbus 127.0.0.1:502 [연결]
cd C:\work_kw\robot
python ui\quiz3_gui.py

:: 3) 로봇 DRL 실행 → UI에서 로봇 IP:9000 [연결] → READY 수신 후 작업 버튼 활성
```

서버 확인 (다른 cmd 창):

```bat
python check_server.py            :: 레지스터 20개 표
python check_server.py --watch    :: 1초마다 갱신
python check_server.py --clear    :: 서버를 켠 채 전부 0으로 (y 확인)
```

## 레지스터 맵

화면 번호(문제지 표기)는 1부터, 통신 주소는 0부터. **화면 번호 = 통신 주소 + 1**

| 화면 번호 | 통신 주소 | 내용 | 쓰는 쪽 |
|---|---|---|---|
| 1 | 0 | 작업1 완료 횟수 (정상 완료 후 +1) | UI |
| 2 | 1 | 작업2 완료 횟수 (정상 완료 후 +1) | UI |
| 3–10 | 2–9 | 작업3–10 완료 횟수 (예비) | - |
| 11–20 | 10–19 | 작업2 측정 Z 높이 ×10, 부호 있는 16비트 (측정 ID 1 → 주소 10) | UI |

- 서버는 값을 저장만 한다. 어느 칸에 무엇을 쓸지는 UI가 정한다.
- UI는 작업2를 시작할 때 10–19를 0으로 비우고, 이번 실행의 측정값만 남긴다.
- UI의 배치(`TASK_COUNT_START` 등)를 바꾸면 `register_map.py`도 같이 바꿔야 서버 로그·확인 도구 표시가 맞는다.

## 서버 동작

- 쓰기 요청이 오면 메모리에 반영하기 **직전에** `quiz3_registers.json`에 저장한다
  (pymodbus `SimDevice(action=...)` 훅). 서버를 다시 켜면 이 파일에서 값을 복원한다.
- 파일 저장에 실패하면 쓰기를 거부(`DEVICE_FAILURE`)한다 → UI에는 쓰기 오류로 보인다.
- 주소 20 이상은 `ILLEGAL_ADDRESS`로 거부한다.
- 같은 PC에서 Modbus 서버는 포트 하나에 하나만 뜬다. 연습용 `server.py`(502)와 동시에 켜지 말 것.

## 확인이 필요한 것

- 작업1 횟수 위치: 문제지 표기("작업1 → 주소 1", "11번 / 주소 10")가 섞여 있음.
  현재는 **화면 1번 = 통신 주소 0**으로 해석 (UI와 동일)
- 측정값: DRL이 보내는 `measured_z`(접촉 시점 Z)를 그대로 기록. 테이블 Z가 0이 아니면 길이와 다름
