# 퀴즈3 — Modbus TCP 통신 + UI

UI에서 로봇 작업1(Pick & Place)·작업2(높이 측정)를 실행하고, 결과를 별도 Python Modbus 서버의
Holding Register에 기록·재조회한다. 로봇 동작 코드는 퀴즈2 `main.drl`을 그대로 쓴다.

## 구성

| 파일 | 역할 | 담당 |
|---|---|---|
| `quiz3_server.py` | Modbus TCP 서버. Holding Register 20개 제공, 쓰기마다 파일 저장 | 서버 |
| `check_server.py` | 서버 값 확인 / 감시 / 초기화 도구 | 서버 |
| `quiz3_ui.py` | PyQt UI. 로봇 소켓 서버(PC:20000) + Modbus 클라이언트 | UI |
| `register_map.py` | **주소표·배율 공용 파일** (서버·확인 도구·UI가 모두 import) | 공용 |
| `legacy/quiz3_gui_v1.py` | 이전 UI (로봇 9000번 접속, `TASK,n` 프로토콜). 현재 미사용, 보관용 | - |

```
 로봇 (퀴즈2 main.drl) ──소켓──▶ PC:20000  quiz3_ui.py ──Modbus TCP──▶ 127.0.0.1:5020  quiz3_server.py
   start1/start2/end ◀──────────                     ◀── 읽기/쓰기 ──           └─ quiz3_registers.json
 로봇 컨트롤러 :502 ◀──Modbus 읽기(자세 모니터, 270~275 / 400~402)── quiz3_ui.py
```

## 실행 순서 (Windows, cmd 3개)

```bat
C:\work_kw\kw2026\Scripts\activate
cd C:\work_kw\robot\quizzes\quiz3

:: 1) 서버 (시연 전에는 --reset 으로 0부터)
python quiz3_server.py --reset

:: 2) UI → [Modbus 연결]
python quiz3_ui.py

:: 3) 로봇에서 퀴즈2 main.drl 실행 (PC_IP = 이 PC, PC_PORT = 20000)
::    UI에 READY가 뜨면 작업 버튼 활성화
```

서버 확인: `python check_server.py` (표 출력), `--watch` (1초마다), `--clear` (서버를 켠 채 전부 0)

## 레지스터 맵 (`register_map.py`)

화면 번호(문제지 표기)는 1부터, 통신 주소는 0부터. **화면 번호 = 통신 주소 + 1**

| 화면 번호 | 통신 주소 | 내용 | 쓰는 쪽 |
|---|---|---|---|
| 1 | 0 | 작업1 수행 횟수 (정상 완료 후 +1) | UI |
| 2 | 1 | 작업2 수행 횟수 (정상 완료 후 +1) | UI |
| 3–10 | 2–9 | 작업3–10 수행 횟수 (예비) | - |
| 11–20 | 10–19 | 작업2 측정 길이 ×10 정수, 1회부터 차례로 | UI |

- 측정값: `mm × 10`을 **사사오입** 반올림 (52.25 mm → 523). Python `round()`는 52.25 → 522 이므로 쓰지 않는다.
- 서버는 값을 저장만 한다. 해석·순번 관리는 UI가 한다.
- 주소를 바꿀 때는 `register_map.py` 한 곳만 고친다.

## 기록 규칙 (UI 구현)

- 작업2 시작 **전**에 빈 측정 칸 3개를 확인, 부족하면 로봇에 명령을 보내지 않음
- 모든 쓰기는 **쓰기 → 재조회 → 일치 확인** 후 완료 처리
- 통신이 끊기면 요청을 보관했다가 재접속 후 **현재 값을 먼저 읽고** 반영 여부를 판단 (중복 증가 방지)
- 서버는 쓰기를 받을 때마다 `quiz3_registers.json`에 먼저 저장 → 서버가 재시작돼도 값 유지.
  저장에 실패하면 쓰기를 거부(DEVICE_FAILURE)해서 UI가 오류로 인식

## 확인이 필요한 것

- 작업1 횟수 위치: 문제지 표기("작업1 → 주소 1", "11번 / 주소 10")가 섞여 있음. 현재는 **화면 1번 = 통신 주소 0**으로 해석
- "파지 순간의 실제 Z값으로 길이 측정": 퀴즈2 DRL은 잡기 전 순응제어 접촉으로 높이를 잼
- UI 자세 모니터 기본 IP(`192.168.137.106`)가 실제 로봇 컨트롤러 IP인지
