# robot

광운대학교 로봇 수업 팀 저장소 — 두산 협동로봇 M0609
PC 측 Modbus TCP 통신, PyQt UI, OpenCV 비전, 로봇 DRL 코드

---

## ⚠️ 작업 규칙 (먼저 읽기)

**main 브랜치에는 직접 push할 수 없습니다.** (Ruleset으로 보호 중)
모든 변경은 아래 순서로만 main에 들어갑니다.

```
브랜치 만들기 → 작업·커밋 → push → PR 생성 → CI 통과 → merge
```

### 1. 최신 main에서 브랜치 만들기

```bat
cd C:\work_kw\robot
git switch main
git pull
git switch -c quiz3/server
```

브랜치 이름 규칙: `종류/내용`

| 종류 | 용도 | 예 |
|---|---|---|
| `quizN/` | 퀴즈 작업 | `quiz3/server`, `quiz3/ui`, `quiz4/drl` |
| `fix/` | 버그 수정 | `fix/register-address` |
| `docs/` | 문서 | `docs/readme` |
| `ci/` | CI/CD 설정 | `ci/pin-versions` |

### 2. push 전에 내 PC에서 검사 (CI와 같은 검사)

```bat
C:\work_kw\kw2026\Scripts\activate
ruff check .
python -m pytest
```

- 둘 다 통과해야 PR에서도 통과합니다. 여기서 실패하면 CI에서도 실패합니다.
- `ruff check --fix .` 로 자동 수정 가능한 것(import 순서 등)은 고칠 수 있습니다.
- 자주 걸리는 규칙: 한 줄에 `;`로 두 문장 쓰기(E702), 한 줄 100자 초과(E501)
- VS Code의 **Ruff 확장**을 설치하면 저장할 때 바로 표시됩니다.

### 3. 커밋하고 push

```bat
git add 파일이름
git commit -m "quiz3: Modbus 서버 레지스터 맵 추가"
git push -u origin quiz3/server
```

- `git add .` 는 가상환경·캐시까지 올라갈 수 있으니 파일을 지정하는 것을 권장합니다.
- 커밋 메시지: `범위: 무엇을 했는지` (예: `quiz3: UI 측정값 기록 주소 수정`)

### 4. GitHub에서 PR 만들기

1. push 후 저장소 페이지에 뜨는 **Compare & pull request** 클릭
2. base: `main` ← compare: 내 브랜치 확인
3. 무엇을 바꿨는지, 어떻게 확인했는지 적고 **Create pull request**

### 5. CI 통과 확인 후 merge

- PR 아래 **test** 체크가 ✅ 가 되어야 **Merge** 버튼이 활성화됩니다.
- ❌ 실패하면: **Details** 에서 원인 확인 → 같은 브랜치에서 고쳐서 다시 push → PR에 자동 반영
- merge 후 **Delete branch** 클릭

### 6. merge 후 내 PC 정리

```bat
git switch main
git pull
git branch -d quiz3/server
```

### 하지 말 것

- main에서 바로 작업하기 (push가 거부되고, 작업한 커밋을 브랜치로 옮겨야 함)
- 한 PR에 여러 작업 섞기 (리뷰·되돌리기가 어려워짐)
- `git push --force` (보호 규칙으로 main에는 막혀 있지만, 공유 브랜치에서도 쓰지 말 것)

### main에서 실수로 커밋했을 때

```bat
git switch -c fix/내작업        :: 지금 커밋을 새 브랜치로 가져감
git switch main
git reset --hard origin/main   :: main을 GitHub 상태로 되돌림 (push 안 된 변경은 삭제됨)
git switch fix/내작업
git push -u origin fix/내작업
```

---

## 구조

```
src/robot_comm/modbus_client.py   PC → Modbus 서버 클라이언트
src/robot_comm/sim_server.py      로봇 없이 테스트하는 가짜 Modbus 서버
src/vision/detect.py              OpenCV 색상 기반 물체 중심 검출
src/vision/gesture_logic.py       손 스켈레톤 → 제스처 판정, 제스처 → 작업 실행 판단 (MediaPipe 없이 테스트)
src/vision/gesture.py             MediaPipe Hand Landmarker 검출·오버레이, 단독 미리보기
tools/download_hand_model.py      MediaPipe 손 모델(models/hand_landmarker.task) 다운로드
ui/quiz3_gui.py                   퀴즈3 PyQt UI
drl/                              로봇에 올리는 DRL 코드(.drl) — CI는 문법만 검사
tests/                            pytest 테스트
.github/workflows/ci.yml          PR / main push마다 lint + 테스트
.github/workflows/release.yml     v* 태그 push 시 테스트 후 zip을 GitHub Release로 배포
```

퀴즈는 1~5단계까지 있으며, 퀴즈별 폴더(`quizzes/quiz1` ~ `quiz5`)로 정리할 예정입니다.

## 개발 환경 (Windows)

```bat
cd C:\work_kw
py -3.11 -m venv kw2026
kw2026\Scripts\activate
cd robot
pip install -r requirements.txt -r requirements-dev.txt
```

- 새 cmd 창을 열 때마다 `activate` 필요
- VS Code: `Ctrl+Shift+P` → Python: Select Interpreter → `C:\work_kw\kw2026\Scripts\python.exe`

## 실행

로봇 없이 Modbus 서버 띄우기:

```bat
set PYTHONPATH=src
python -m robot_comm.sim_server --port 5020
```

퀴즈3 UI:

```bat
python ui\quiz3_gui.py
```

### 작업 2 결과 CSV (퀴즈2 누적 기능)

작업 2에서 물체 하나의 이송이 정상 종료될 때마다 `task2_height_result.csv`(저장소 루트)에 한 행씩 자동 추가된다.

| 열 | 값 |
|---|---|
| `job_id`, `length_mm` | 물체 번호, 접촉 측정 Z (mm) |
| `pick_*`, `place_*` | 실제 명령한 집기·놓기 목표점 X, Y, Z, Rx, Ry, Rz |
| `start_time`, `end_time`, `duration_s` | 그 물체의 첫 `MoveJ Start` ~ `JOB_DONE` 직전 마지막 이동 완료 = 놓고 상승 완료 (PC 수신 시각) |

- 열 제목은 파일을 처음 만들 때만 쓴다. 이후 실행은 같은 파일에 이어 쓴다.
- 이송 중 오류·연결 끊김·타임아웃으로 끝난 물체는 저장하지 않는다.
- 시연·제출용으로 "3행"만 남기려면 실행 전에 기존 파일을 지우거나 옮겨 둔다.
- 엑셀로 파일을 열어 둔 상태면 Windows에서 쓰기가 막혀 저장되지 않는다 (로그에 표시됨).
- 예전 형식(열 제목이 다른) 파일이 있으면 `task2_height_result_old_날짜.csv`로 보관하고 새로 만든다.

로봇 → PC 메시지 (작업 2, 물체마다): `JOB,id,length,pick6,place6` → 모션 메시지 → `JOB_DONE,id`

- 이동 메시지는 `MoveJ 종료` / `MoveJ Done` / `MoveL_REL Done` 형식을 모두 인정한다 (DRL 형식이 바뀌어도 CSV 유지).
- **펜던트 DRL이 `JOB`/`JOB_DONE`을 안 보내면 CSV는 0행**이다. 작업 중 로그에 경고가 뜨고, 끝날 때 원인이 표시된다. DART-Studio 프로젝트의 `main.drl`을 저장소 최신본으로 바꿔 올릴 것.

### 작업 응답 없음 (타임아웃)

- 작업 중 로봇 메시지가 `TASK_IDLE_TIMEOUT_MS`(기본 60초) 동안 **한 줄도 안 오면** 오류로 끝낸다. 작업 전체 시간 제한이 아니어서, 로봇이 메시지를 계속 보내는 한 오래 걸리는 작업도 끝까지 기록된다.
- 응답 없음으로 끝나면 Vision/Gesture 자동 실행을 멈추고, 로봇이 `DONE`/`ERROR`를 보내거나 재연결할 때까지 새 작업을 보내지 않는다 (움직이는 중인 로봇에 다음 명령이 쌓이는 것 방지).

### Gesture (MediaPipe 손 스켈레톤 → 로봇 작업)

처음 한 번: opencv-python을 지우고 모델을 받는다 (mediapipe는 opencv-contrib-python에 의존, 둘이 같이 있으면 cv2가 꼬임).

```bat
pip uninstall -y opencv-python
pip install -r requirements.txt
python tools\download_hand_model.py
```

| 제스처 (엄지는 판정 제외) | 결과 |
|---|---|
| 검지만 폄 (ONE) | 1초 유지 → 작업 1 |
| 검지 + 중지 (TWO) | 1초 유지 → 작업 2 |
| 주먹 / 손바닥 / 애매한 모양 / 손 없음 | 실행 없음 |
| 손 2개 이상 (MULTI) | 1초 유지 → 랜덤 동작 TaskRandom1 (`TASK,R1`) |

- 실행 후 같은 제스처를 계속 보여도 재실행 안 됨. 손을 내린 상태가 1초 이어져야 다음 실행.
- Gesture 실행 중에는 작업1·작업2 버튼과 Vision 버튼이 비활성화된다 (카메라 하나).
- 로봇 없이 판정만 확인: `python src\vision\gesture.py` (q 종료)

## CI/CD

| 워크플로 | 실행 시점 | 하는 일 |
|---|---|---|
| CI (`ci.yml`) | PR, main push, 수동 | Windows + Python 3.11에서 `ruff check .` → `pytest` |
| Release (`release.yml`) | `v*` 태그 push | CI 통과 후 `src`, `drl`, `requirements.txt`, `README.md`를 zip으로 Release 등록 |

릴리스 만들기 (main에 merge된 상태에서):

```bat
git switch main
git pull
git tag v0.1.0
git push origin v0.1.0
```

로봇 컨트롤러에 DRL을 올리는 작업은 자동화하지 않습니다. 릴리스 zip을 받아 티치펜던트 / DART-Studio에서 수동으로 올립니다.
