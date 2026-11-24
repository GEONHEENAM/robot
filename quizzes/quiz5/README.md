# 퀴즈5 — YOLO 판별 → 로봇 A(왼쪽) / B(오른쪽) 이동

## 파일

| 파일 | 역할 |
|---|---|
| `../../drl/quiz5/main.drl` | 로봇 프로그램. 소켓 9000으로 명령 받고 로그 보냄. `TASK,A` 왼쪽, `TASK,B` 오른쪽 |
| `server.py` | PC Modbus 서버 (5020, 레지스터 0~49) |
| `register_map.py` | 레지스터 맵 (서버·GUI·YOLO 공용) |
| `yolo_judge.py` | YOLO 판별 (N장 → 평균 신뢰도 ≥ 기준이면 A). 단독 실행 가능 |
| `yolo_tune.py` | CONF / IMGSZ / PREPROCESS 조합 비교 (tune 사진) |
| `quiz5_gui.py` | PyQt6 UI (퀴즈1 작업1, 퀴즈3 Modbus, 퀴즈4 Vision/Gesture + YOLO) |

퀴즈2(작업2 높이 측정)는 구현하지 않음.

## 통신 흐름

```
[YOLO 사진 N장] → quiz5_gui(YoloThread) → 판정 A/B
      │ 결과 40~49 쓰기                         │ TASK,A / TASK,B
      ▼                                         ▼
 PC Modbus 서버(5020)  ◀─ 좌표 복사 20~32 ─  quiz5_gui  ──소켓 9000──▶ 로봇 DRL
                                               ▲   ◀── 로그·DONE,n ───┘
                                               └── 로봇 Modbus(502) 400~405 / 270~275 / 259 읽기
```

- 로봇 현재 좌표: 로봇 컨트롤러 Modbus Slave(포트 502)에서 PC가 직접 읽는다 (DRL은 좌표를 안 보냄)
- 로그: DRL `send_msg()` → 소켓 → GUI 로그 창

## 레지스터 (PC 서버)

| 주소 | 내용 |
|---|---|
| 0, 2, 3, 4 | 작업1 / 랜덤R1 / A이동 / B이동 완료 횟수 |
| 20~25 | 로봇 TCP X,Y,Z,A,B,C (x10) |
| 26~31 | 로봇 J1~J6 (x10) |
| 32 | 로봇 상태 |
| 40 | 검사한 사진 수 |
| 41 | 개가 검출된 사진 수 |
| 42 | 검출된 개 수 |
| 43 | 평균 신뢰도 % x100 |
| 44 | 기준 값 % x100 |
| 45 | 판별 결과 (1=A, 2=B) |
| 46 | 판별 횟수 |
| 47~49 | 사용한 CONF x100 / IMGSZ / PREPROCESS(0,1,2) |

## 설치 (kw2026)

```bat
pip install ultralytics
:: ultralytics가 opencv-python을 같이 깔아서 mediapipe용 opencv-contrib-python과 충돌한다
pip uninstall -y opencv-python
pip install --force-reinstall --no-deps opencv-contrib-python==5.0.0.93
```

## 실행 순서

```bat
cd C:\work_kw\robot
:: 1) 최적값 찾기 → yolo_judge.py 위쪽 CONF / IMGSZ / PREPROCESS에 적기
python quizzes\quiz5\yolo_tune.py
:: 2) 서버
python quizzes\quiz5\server.py --reset
:: 3) 펜던트에서 drl\quiz5\main.drl 실행 (READY 대기)
:: 4) UI → 세 연결 버튼 → YOLO 판별 실행
python quizzes\quiz5\quiz5_gui.py
```

## 시연 전 확인

- `main.drl`의 `SORT_PICK_UP`, `AREA_A_UP`, `AREA_B_UP`은 임시 값이다. 실제 왼쪽/오른쪽과 X 부호가 맞는지 티칭으로 확인
- 로봇 Modbus 좌표가 펜던트 값과 같은지 확인 (다르면 주소가 1칸 밀린 것: 400 → 399)
