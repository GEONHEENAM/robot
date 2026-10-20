# robot

두산 협동로봇 M0609 — PC 측 Modbus TCP 통신 + OpenCV 비전 코드

## 구조

```
src/robot_comm/modbus_client.py   PC → 로봇 컨트롤러 Modbus TCP 클라이언트
src/robot_comm/sim_server.py      로봇 없이 테스트하는 가짜 Modbus 서버
src/vision/detect.py              OpenCV 색상 기반 물체 중심 검출
drl/                              로봇에 올리는 DRL 코드(.drl) — CI는 문법만 검사
tests/                            pytest 테스트
.github/workflows/ci.yml          push/PR마다 lint + 테스트
.github/workflows/release.yml     v* 태그 push 시 테스트 후 zip을 GitHub Release로 배포
```

## 로컬 실행 (Windows)

```bat
cd C:\work_kw
kw2026\Scripts\activate
pip install -r requirements.txt -r requirements-dev.txt
ruff check .
python -m pytest -v
```

로봇 없이 서버 띄우기: `set PYTHONPATH=src && python -m robot_comm.sim_server --port 5020`

## 배포(릴리스)

```bat
git tag v0.1.0
git push origin v0.1.0
```

로봇 컨트롤러에 DRL을 올리는 작업은 자동화하지 않는다. 릴리스 zip을 받아 티치펜던트/DART-Studio에서 수동으로 올린다.
