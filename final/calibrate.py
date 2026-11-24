"""
카메라 ↔ 로봇 캘리브레이션 (3단계)

테이블 평면에서 "픽셀 좌표 → 로봇 XY(mm)" 호모그래피를 구해 calib/H.npy로 저장한다.
GUI와 vision.py는 이 파일이 있으면 자동으로 사용한다. (GUI는 카메라를 껐다 켜야 다시 읽음)

[기본 모드: 로봇이 점을 찍어 준다]
  1) 로봇이 집기 영역 안의 격자점(기본 3x3)으로 차례로 내려간다 (MOVEXY)
  2) 그리퍼 중심 바로 아래 테이블에 마커(검은 원 스티커 등)를 하나씩 붙인다
  3) 로봇이 홈으로 비키면 카메라로 촬영 → 화면에서 마커를 번호 순서대로 클릭
  4) 호모그래피 계산 + 오차(mm) 출력 → calib/H.npy 저장
     python calibrate.py
     python calibrate.py --ip 127.0.0.1          # 가짜 로봇으로 흐름 연습

[로봇 없이: 좌표를 직접 입력]
  마커의 로봇 좌표를 티칭 펜던트에서 읽어 CSV(x,y 한 줄씩)로 적어 둔 경우
     python calibrate.py --points-file calib/points.csv

[검증 모드]
  화면에서 아무 점이나 클릭 → 로봇 좌표 출력 → (선택) 로봇이 그 위치로 내려가 실제 어긋남 확인
     python calibrate.py --verify
     python calibrate.py --verify --no-robot     # 좌표만 확인

클릭 창 키: 왼쪽 클릭 = 점 찍기 | u = 하나 취소 | r = 처음부터 | Enter/Space = 완료 | q/ESC = 중단
  ※ 클릭한 근처에서 마커 중심을 자동으로 다시 찾는다 (실패하면 클릭 위치 그대로)
  ※ 렌즈 왜곡 보정(calib/camera.npz)을 쓴다면 반드시 그 파일을 먼저 만들고 캘리브레이션할 것
"""

import argparse
import csv
import json
import time
from pathlib import Path

import cv2
import numpy as np

import config
from robot_link import RobotError, RobotLink
from vision import PixelToRobot, Undistorter, in_pick_zone, open_camera

MARK_Z = 15.0        # 마커 붙일 때 그리퍼 중심 높이 (mm). 낮을수록 눈대중 오차가 줄어듦
CHECK_Z = 15.0       # 검증 모드에서 내려갈 높이
GRID_MARGIN = 30.0   # 격자점을 집기 영역 경계에서 안쪽으로 떨어뜨릴 거리 (mm)


# ============================================================
# 계산
# ============================================================
def grid_points(cols=3, rows=3, margin=GRID_MARGIN):
    (x0, x1), (y0, y1) = config.PICK_X_RANGE, config.PICK_Y_RANGE
    xs = np.linspace(min(x0, x1) + margin, max(x0, x1) - margin, cols)
    ys = np.linspace(min(y0, y1) + margin, max(y0, y1) - margin, rows)
    return [(float(x), float(y)) for y in ys for x in xs]


def compute_homography(pixels, points):
    """픽셀 N개 ↔ 로봇 XY N개 → (H, 적합 오차[mm], 교차검증 오차[mm]).

    교차검증(leave-one-out): 점 하나를 빼고 구한 H로 그 점을 예측했을 때의 오차.
    처음 보는 위치에서의 실제 정확도에 더 가깝다.
    """
    src = np.asarray(pixels, np.float64)
    dst = np.asarray(points, np.float64)
    if len(src) < 4:
        raise ValueError("점이 4개 이상 필요")
    H, _ = cv2.findHomography(src, dst, 0)
    if H is None:
        raise ValueError("호모그래피 계산 실패 (점이 한 줄로 늘어서 있지 않은지 확인)")
    fit = np.linalg.norm(cv2.perspectiveTransform(src.reshape(-1, 1, 2), H).reshape(-1, 2) - dst, axis=1)

    loo = []
    if len(src) >= 5:
        for i in range(len(src)):
            keep = [j for j in range(len(src)) if j != i]
            Hi, _ = cv2.findHomography(src[keep], dst[keep], 0)
            if Hi is None:
                loo.append(float("nan"))
                continue
            p = cv2.perspectiveTransform(src[i].reshape(1, 1, 2), Hi).reshape(2)
            loo.append(float(np.linalg.norm(p - dst[i])))
    return H, fit, np.asarray(loo)


def refine_click(gray, u, v, radius=30):
    """클릭 근처에서 마커(어둡거나 밝은 덩어리)의 중심을 찾는다. 못 찾으면 클릭 위치."""
    h, w = gray.shape
    x0, y0 = max(0, int(u) - radius), max(0, int(v) - radius)
    x1, y1 = min(w, int(u) + radius + 1), min(h, int(v) + radius + 1)
    roi = gray[y0:y1, x0:x1]
    if roi.size == 0:
        return float(u), float(v)
    best, best_d = None, 1e9
    _, th = cv2.threshold(roi, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    for binimg in (th, cv2.bitwise_not(th)):
        n, _lab, stats, cents = cv2.connectedComponentsWithStats(binimg)
        for k in range(1, n):
            area = stats[k, cv2.CC_STAT_AREA]
            bw, bh = stats[k, cv2.CC_STAT_WIDTH], stats[k, cv2.CC_STAT_HEIGHT]
            touches_edge = (stats[k, cv2.CC_STAT_LEFT] == 0 or stats[k, cv2.CC_STAT_TOP] == 0 or
                            stats[k, cv2.CC_STAT_LEFT] + bw >= roi.shape[1] or
                            stats[k, cv2.CC_STAT_TOP] + bh >= roi.shape[0])
            if area < 15 or touches_edge:
                continue
            cx, cy = cents[k][0] + x0, cents[k][1] + y0
            d = np.hypot(cx - u, cy - v)
            if d < best_d and d < radius * 0.8:
                best, best_d = (float(cx), float(cy)), d
    return best if best is not None else (float(u), float(v))


# ============================================================
# 카메라 / 클릭 UI
# ============================================================
def grab_frame(cap, und, settle_frames=10, average=5):
    """버퍼에 남은 옛 프레임을 버리고 몇 장을 평균해서 잡음을 줄인다."""
    for _ in range(settle_frames):
        cap.read()
    acc, n = None, 0
    for _ in range(average):
        ok, f = cap.read()
        if not ok:
            continue
        f = und(f).astype(np.float32)
        acc = f if acc is None else acc + f
        n += 1
    if n == 0:
        raise RuntimeError("카메라 프레임을 읽지 못함")
    return (acc / n).astype(np.uint8)


def click_points(frame, labels, title="calibration"):
    """labels 순서대로 클릭을 받는다. 완료하면 [(u, v), ...], 중단하면 None."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    pts = []

    def on_mouse(event, x, y, _flags, _param):
        if event == cv2.EVENT_LBUTTONDOWN and len(pts) < len(labels):
            pts.append(refine_click(gray, x, y))

    cv2.namedWindow(title, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(title, on_mouse)
    try:
        while True:
            img = frame.copy()
            for i, (u, v) in enumerate(pts):
                cv2.drawMarker(img, (int(u), int(v)), (0, 0, 255), cv2.MARKER_CROSS, 22, 2)
                cv2.putText(img, str(i + 1), (int(u) + 8, int(v) - 8), cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (0, 0, 255), 2, cv2.LINE_AA)
            if len(pts) < len(labels):
                msg = f"click {len(pts) + 1}/{len(labels)}: {labels[len(pts)]}   (u:undo r:reset q:quit)"
            else:
                msg = "all clicked - Enter/Space: OK   u:undo r:reset q:quit"
            cv2.rectangle(img, (0, 0), (img.shape[1], 34), (0, 0, 0), -1)
            cv2.putText(img, msg, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.imshow(title, img)
            key = cv2.waitKey(30) & 0xFF
            if key in (ord("q"), 27):
                return None
            if key == ord("u") and pts:
                pts.pop()
            elif key == ord("r"):
                pts.clear()
            elif key in (13, 10, 32) and len(pts) == len(labels):
                return list(pts)
    finally:
        cv2.destroyWindow(title)


# ============================================================
# 저장
# ============================================================
def save_result(H, pixels, points, fit, loo, frame, cam_index):
    calib_dir = config.HOMOGRAPHY_FILE.parent
    calib_dir.mkdir(parents=True, exist_ok=True)
    if config.HOMOGRAPHY_FILE.exists():   # 이전 결과는 날짜를 붙여 보관
        backup = calib_dir / time.strftime("H_backup_%Y%m%d_%H%M%S.npy")
        config.HOMOGRAPHY_FILE.replace(backup)
        print(f"이전 H.npy 보관: {backup.name}")
    np.save(config.HOMOGRAPHY_FILE, H)

    report = {
        "saved_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "camera_index": cam_index,
        "frame_size": [int(frame.shape[1]), int(frame.shape[0])],
        "undistort": Path(config.CAMERA_CALIB_FILE).exists(),
        "points_robot_mm": [list(map(float, p)) for p in points],
        "pixels": [list(map(float, p)) for p in pixels],
        "fit_error_mm": [round(float(e), 2) for e in fit],
        "loo_error_mm": [round(float(e), 2) for e in loo],
    }
    (calib_dir / "calib_report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")

    img = frame.copy()
    mapper = PixelToRobot(H)
    for i, ((u, v), (x, y)) in enumerate(zip(pixels, points)):
        pu, pv = mapper.to_pixel([[x, y]])[0]     # 로봇 좌표를 다시 화면으로 → 클릭점과 비교
        cv2.drawMarker(img, (int(u), int(v)), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
        cv2.circle(img, (int(pu), int(pv)), 6, (0, 255, 0), 2)
        cv2.putText(img, f"{i + 1}: {fit[i]:.1f}mm", (int(u) + 8, int(v) - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1, cv2.LINE_AA)
    cv2.imwrite(str(calib_dir / "calib_result.png"), img)
    print(f"저장: {config.HOMOGRAPHY_FILE}, calib_report.json, calib_result.png")


def print_errors(fit, loo, tol):
    print("\n점별 오차 (mm)   적합 / 교차검증")
    for i, f in enumerate(fit):
        l = f"{loo[i]:.2f}" if len(loo) else "-"
        print(f"  {i + 1:2d}: {f:6.2f} / {l}")
    worst = float(np.nanmax(loo)) if len(loo) else float(np.max(fit))
    print(f"적합 평균 {np.mean(fit):.2f}, 최대 {np.max(fit):.2f}")
    if len(loo):
        print(f"교차검증 평균 {np.nanmean(loo):.2f}, 최대 {np.nanmax(loo):.2f}")
    verdict = "통과" if worst <= tol else "불합격"
    print(f"판정 (허용 {tol:.1f} mm): {verdict}")
    if worst > tol:
        print("  → 마커 위치(눈대중), 클릭 정확도, 카메라 고정, 렌즈 왜곡을 확인하고 다시 할 것")
        print("  → 오차가 큰 점만 튀면 그 마커를 다시 붙이고 재시도")
    return worst <= tol


# ============================================================
# 모드
# ============================================================
def open_cam_or_image(args):
    und = Undistorter()
    if args.image:
        img = cv2.imread(args.image)
        if img is None:
            raise SystemExit(f"사진을 읽을 수 없음: {args.image}")
        return None, und, img
    cap, msgs = open_camera(args.camera)
    for m in msgs:
        print("[camera]", m)
    if cap is None:
        raise SystemExit("카메라를 열 수 없음")
    return cap, und, None


def run_calibration(args):
    if args.points_file:
        with open(args.points_file, newline="", encoding="utf-8-sig") as f:
            points = [(float(r[0]), float(r[1])) for r in csv.reader(f) if r and not r[0].startswith("#")]
        link = None
    else:
        points = grid_points(args.cols, args.rows)
        link = RobotLink(args.ip, args.port)

    print(f"캘리브레이션 점 {len(points)}개 (로봇 XY, mm):")
    for i, (x, y) in enumerate(points, 1):
        print(f"  {i}: ({x:.1f}, {y:.1f})")

    cap, und, still = open_cam_or_image(args)
    try:
        if link is not None:
            link.home()
            input("\n작업 영역(테이블)을 비우고 Enter → 로봇이 첫 점으로 내려갑니다 ")
            for i, (x, y) in enumerate(points, 1):
                link.move_xy(x, y, MARK_Z)
                input(f"[{i}/{len(points)}] 그리퍼 중심 바로 아래 테이블에 마커 {i}를 붙이고 Enter ")
            link.home()
            time.sleep(1.0)

        frame = still if still is not None else grab_frame(cap, und)
        labels = [f"marker {i} ({x:.0f},{y:.0f})" for i, (x, y) in enumerate(points, 1)]
        pixels = click_points(frame, labels)
        if pixels is None:
            print("중단됨 (저장 안 함)")
            return
        H, fit, loo = compute_homography(pixels, points)
        ok = print_errors(fit, loo, args.tol)
        if ok or input("허용 오차를 넘었지만 저장할까요? [y/N] ").strip().lower() == "y":
            save_result(H, pixels, points, fit, loo, frame, args.camera)
            print("GUI를 쓰고 있다면 [카메라 정지] → [카메라 시작]으로 새 변환을 읽을 것")
    finally:
        if cap is not None:
            cap.release()
        if link is not None:
            link.close()


def run_verify(args):
    mapper = PixelToRobot.from_file()
    if mapper is None:
        raise SystemExit(f"{config.HOMOGRAPHY_FILE}가 없음 → 먼저 캘리브레이션")
    link = None if args.no_robot else RobotLink(args.ip, args.port)
    cap, und, still = open_cam_or_image(args)
    print("화면에서 확인할 점을 클릭 → Enter. q로 종료")
    try:
        while True:
            if link is not None:
                link.home()
                time.sleep(0.5)
            frame = still if still is not None else grab_frame(cap, und)
            pts = click_points(frame, ["check point"], title="verify")
            if pts is None:
                break
            x, y = mapper.to_robot(pts)[0]
            print(f"픽셀 ({pts[0][0]:.1f}, {pts[0][1]:.1f}) → 로봇 ({x:.1f}, {y:.1f}) mm"
                  f"{'' if in_pick_zone(x, y) else '  (집기 영역 밖)'}")
            if link is None:
                continue
            if input(f"로봇을 ({x:.1f}, {y:.1f}, {CHECK_Z:.0f})로 내릴까요? [y/N] ").strip().lower() != "y":
                continue
            try:
                link.move_xy(x, y, CHECK_Z)
            except RobotError as e:
                print(f"로봇 거부: {e}")
                continue
            input("그리퍼 중심과 클릭한 점의 어긋남(mm)을 자로 재서 기록한 뒤 Enter → 홈 복귀 ")
    finally:
        if cap is not None:
            cap.release()
        if link is not None:
            link.close()


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="카메라-로봇 캘리브레이션 (픽셀 → 로봇 XY)")
    ap.add_argument("--ip", default=config.ROBOT_IP)
    ap.add_argument("--port", type=int, default=config.ROBOT_PORT)
    ap.add_argument("--camera", type=int, default=config.CAMERA_INDEX)
    ap.add_argument("--image", default=None, help="카메라 대신 저장된 사진 사용 (연습용)")
    ap.add_argument("--cols", type=int, default=3)
    ap.add_argument("--rows", type=int, default=3)
    ap.add_argument("--tol", type=float, default=3.0, help="허용 오차 mm = (그리퍼 벌림 폭 - 막대 폭) / 2 보다 작게")
    ap.add_argument("--points-file", default=None, help="로봇 없이: 마커 로봇 좌표 CSV (x,y)")
    ap.add_argument("--verify", action="store_true", help="검증 모드")
    ap.add_argument("--no-robot", action="store_true", help="검증 모드에서 로봇 이동 없이 좌표만 출력")
    a = ap.parse_args()
    try:
        run_verify(a) if a.verify else run_calibration(a)
    except (RobotError, TimeoutError, ConnectionError, OSError) as e:
        print(f"[오류] {e}")
