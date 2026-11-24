"""
막대 검출 모듈 (1단계)

  StickDetector   : 프레임 → 막대 목록 (픽셀 기준 + 좌표 변환이 있으면 로봇 좌표)
  PixelToRobot    : 픽셀 ↔ 로봇 XY 변환 (테이블 평면 호모그래피)
                    H.npy가 없으면 GUI 테스트용 임시 변환을 만든다 (config.ALLOW_DUMMY_HOMOGRAPHY)
  Undistorter     : (선택) 렌즈 왜곡 보정. calib/camera.npz가 있을 때만 동작
  draw_detections : 검출 결과를 프레임 위에 그린다 (GUI와 단독 실행 공용)
  open_camera     : 카메라 열기 + 해상도/초점/노출 설정

단독 실행 (kw2026 activate 후)
  python vision.py                  # config.CAMERA_INDEX 카메라
  python vision.py --camera 1       # 다른 카메라 번호
  python vision.py --image a.png    # 저장된 사진으로 테스트
  키: q/ESC 종료 | m 원본/마스크 전환 | s 현재 프레임 저장 | p 파라미터 저장 | d 검출 목록 출력

※ cv2.putText는 한글을 못 그리므로 화면 글자는 영어로 표시한다.
"""

import argparse
import json
import math
import sys
import time
from dataclasses import asdict, dataclass, fields
from pathlib import Path

import cv2
import numpy as np

import config


# ============================================================
# 공통 유틸
# ============================================================
def normalize_angle(deg):
    """막대는 180° 대칭이므로 각도를 -90 이상 90 미만으로 맞춘다."""
    return (deg + 90.0) % 180.0 - 90.0


def in_pick_zone(x, y):
    (x0, x1), (y0, y1) = config.PICK_X_RANGE, config.PICK_Y_RANGE
    return min(x0, x1) <= x <= max(x0, x1) and min(y0, y1) <= y <= max(y0, y1)


# ============================================================
# 검출 파라미터 (GUI 슬라이더 / 단독 실행 트랙바로 조정 → JSON 저장)
# ============================================================
@dataclass
class VisionParams:
    h_low: int = 0
    h_high: int = 179
    s_low: int = 80
    s_high: int = 255
    v_low: int = 50
    v_high: int = 255
    min_area: int = 400       # 픽셀 면적 하한 (작은 잡음 제거)
    min_aspect: float = 2.5   # 긴 변 / 짧은 변 하한 (길쭉한 것만 막대로 인정)
    blur: int = 5             # 가우시안 블러 커널 (1 이하면 끔, 짝수면 +1)
    invert: bool = False      # 마스크 반전 (예: 밝은 배경 위 어두운 막대를 V 범위로 잡을 때)

    @classmethod
    def load(cls, path=config.VISION_PARAMS_FILE):
        p = Path(path)
        if not p.exists():
            return cls()
        try:
            data = json.loads(p.read_text(encoding="utf-8"))
            names = {f.name for f in fields(cls)}
            return cls(**{k: v for k, v in data.items() if k in names})
        except (OSError, ValueError, TypeError) as e:
            print(f"[vision] 파라미터 파일을 읽지 못함 ({e}) → 기본값 사용")
            return cls()

    def save(self, path=config.VISION_PARAMS_FILE):
        p = Path(path)
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(asdict(self), ensure_ascii=False, indent=2), encoding="utf-8")


# ============================================================
# 검출 결과 1개 = 막대 1개
# ============================================================
@dataclass
class Stick:
    idx: int                     # 화면 왼쪽부터 1, 2, 3 ... (프레임마다 다시 매김)
    box_px: np.ndarray           # 회전 사각형 꼭짓점 4개 (픽셀)
    center_px: tuple             # (u, v)
    end_a_px: tuple              # 긴 축 양 끝 (픽셀)
    end_b_px: tuple
    length_px: float
    width_px: float
    angle_px: float              # 이미지 좌표계 긴 축 각도 (참고용)
    x: float | None = None       # 로봇 베이스 좌표 (mm). 좌표 변환이 없으면 None
    y: float | None = None
    angle: float | None = None   # 로봇 좌표계 긴 축 각도 (deg, -90~90)
    length: float | None = None  # 막대 길이 (mm)
    in_pick_zone: bool = False

    @property
    def has_robot_coords(self):
        return self.x is not None


# ============================================================
# 픽셀 ↔ 로봇 좌표 변환
# ============================================================
class PixelToRobot:
    def __init__(self, H, is_dummy=False, source=""):
        self.H = np.asarray(H, dtype=np.float64)
        self.H_inv = np.linalg.inv(self.H)
        self.is_dummy = is_dummy
        self.source = source

    @staticmethod
    def _apply(M, pts):
        arr = np.asarray(pts, dtype=np.float64).reshape(-1, 1, 2)
        return cv2.perspectiveTransform(arr, M).reshape(-1, 2)

    def to_robot(self, pts):
        """픽셀 [(u, v), ...] → 로봇 [(x, y), ...]"""
        return self._apply(self.H, pts)

    def to_pixel(self, pts):
        """로봇 [(x, y), ...] → 픽셀 [(u, v), ...] (화면에 작업 영역·놓기 칸 표시용)"""
        return self._apply(self.H_inv, pts)

    @classmethod
    def from_file(cls, path=config.HOMOGRAPHY_FILE):
        p = Path(path)
        if not p.exists():
            return None
        H = np.load(p)
        if H.shape != (3, 3):
            raise ValueError(f"호모그래피 크기가 3x3이 아님: {H.shape}")
        return cls(H, is_dummy=False, source=str(p))

    @classmethod
    def dummy(cls, frame_w, frame_h):
        """캘리브레이션 전 테스트용: 화면 네 모서리를 config의 임시 범위에 대응시킨다."""
        x0, x1 = config.DUMMY_X_RANGE
        y0, y1 = config.DUMMY_Y_RANGE
        src = np.float32([[0, 0], [frame_w, 0], [frame_w, frame_h], [0, frame_h]])
        dst = np.float32([[x0, y0], [x1, y0], [x1, y1], [x0, y1]])
        return cls(cv2.getPerspectiveTransform(src, dst), is_dummy=True, source="임시 변환")


def load_mapper(frame_w, frame_h):
    """H.npy가 있으면 그것을, 없으면 (허용된 경우) 임시 변환을 돌려준다. 둘 다 아니면 None."""
    try:
        mapper = PixelToRobot.from_file()
        if mapper is not None:
            return mapper
    except (OSError, ValueError) as e:
        print(f"[vision] 호모그래피 파일 오류 ({e})")
    if config.ALLOW_DUMMY_HOMOGRAPHY:
        return PixelToRobot.dummy(frame_w, frame_h)
    return None


# ============================================================
# (선택) 렌즈 왜곡 보정
#   ※ 3단계 캘리브레이션도 같은 보정을 거친 프레임으로 해야 좌표가 맞는다.
# ============================================================
class Undistorter:
    def __init__(self, path=config.CAMERA_CALIB_FILE):
        self.K = self.dist = None
        self._maps = None
        p = Path(path)
        if p.exists():
            try:
                data = np.load(p)
                self.K, self.dist = data["K"], data["dist"]
            except (OSError, KeyError, ValueError) as e:
                print(f"[vision] 렌즈 보정 파일 오류 ({e}) → 보정 없이 진행")

    @property
    def enabled(self):
        return self.K is not None

    def __call__(self, frame):
        if not self.enabled:
            return frame
        h, w = frame.shape[:2]
        if self._maps is None or self._maps[0] != (w, h):
            new_k, _ = cv2.getOptimalNewCameraMatrix(self.K, self.dist, (w, h), 0)
            m1, m2 = cv2.initUndistortRectifyMap(self.K, self.dist, None, new_k, (w, h), cv2.CV_16SC2)
            self._maps = ((w, h), m1, m2)
        return cv2.remap(frame, self._maps[1], self._maps[2], cv2.INTER_LINEAR)


# ============================================================
# 막대 검출
# ============================================================
def _long_axis_ends(box):
    """회전 사각형 꼭짓점 4개 → 긴 축 양 끝점 (짧은 변의 중점 2개)."""
    d01 = np.linalg.norm(box[0] - box[1])
    d12 = np.linalg.norm(box[1] - box[2])
    if d01 >= d12:   # 0-1, 2-3이 긴 변 → 짧은 변은 1-2, 3-0
        return (box[1] + box[2]) / 2, (box[3] + box[0]) / 2
    return (box[0] + box[1]) / 2, (box[2] + box[3]) / 2


class StickDetector:
    def __init__(self, params=None):
        self.params = params or VisionParams.load()
        self._kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

    def make_mask(self, frame):
        p = self.params
        img = frame
        k = int(p.blur)
        if k >= 2:
            k = k if k % 2 == 1 else k + 1
            img = cv2.GaussianBlur(img, (k, k), 0)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        lo_s, hi_s = np.uint8(p.s_low), np.uint8(p.s_high)
        lo_v, hi_v = np.uint8(p.v_low), np.uint8(p.v_high)
        if p.h_low <= p.h_high:
            mask = cv2.inRange(hsv, np.array([p.h_low, lo_s, lo_v], np.uint8),
                               np.array([p.h_high, hi_s, hi_v], np.uint8))
        else:   # 빨강처럼 0/179를 넘나드는 색: H 하한 > H 상한이면 두 구간을 합친다
            m1 = cv2.inRange(hsv, np.array([p.h_low, lo_s, lo_v], np.uint8),
                             np.array([179, hi_s, hi_v], np.uint8))
            m2 = cv2.inRange(hsv, np.array([0, lo_s, lo_v], np.uint8),
                             np.array([p.h_high, hi_s, hi_v], np.uint8))
            mask = cv2.bitwise_or(m1, m2)

        if p.invert:
            mask = cv2.bitwise_not(mask)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._kernel)    # 점 잡음 제거
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._kernel)   # 막대 안 구멍 메우기
        return mask

    def detect(self, frame, mapper=None):
        """프레임 → (막대 목록, 마스크). mapper가 있으면 로봇 좌표·길이·각도까지 계산."""
        mask = self.make_mask(frame)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        found = []
        for c in contours:
            if cv2.contourArea(c) < self.params.min_area:
                continue
            rect = cv2.minAreaRect(c)
            (cx, cy), (w, h), _ = rect
            long_side, short_side = max(w, h), min(w, h)
            if short_side < 1 or long_side / short_side < self.params.min_aspect:
                continue

            box = cv2.boxPoints(rect)
            a, b = _long_axis_ends(box)
            # minAreaRect의 각도는 OpenCV 버전마다 규칙이 달라서 쓰지 않고 끝점으로 직접 계산
            ang_px = normalize_angle(math.degrees(math.atan2(b[1] - a[1], b[0] - a[0])))
            s = Stick(
                idx=0,
                box_px=box,
                center_px=(float(cx), float(cy)),
                end_a_px=(float(a[0]), float(a[1])),
                end_b_px=(float(b[0]), float(b[1])),
                length_px=float(long_side),
                width_px=float(short_side),
                angle_px=ang_px,
            )
            if mapper is not None:
                # 각도는 픽셀이 아니라 "로봇 좌표로 바꾼 끝점"으로 계산해야 정확하다
                A, B = mapper.to_robot([a, b])
                s.x = float((A[0] + B[0]) / 2)
                s.y = float((A[1] + B[1]) / 2)
                s.angle = normalize_angle(math.degrees(math.atan2(B[1] - A[1], B[0] - A[0])))
                s.length = float(np.hypot(B[0] - A[0], B[1] - A[1]))
                s.in_pick_zone = in_pick_zone(s.x, s.y)
            found.append(s)

        found.sort(key=lambda s: s.center_px[0])   # 화면 왼쪽부터 번호
        for i, s in enumerate(found, 1):
            s.idx = i
        return found, mask


# ============================================================
# 화면 표시
# ============================================================
COLOR_OK = (0, 200, 0)          # 집기 영역 안 막대
COLOR_OUT = (150, 150, 150)     # 집기 영역 밖 막대
COLOR_TARGET = (0, 140, 255)    # 선택된 막대
COLOR_ZONE = (255, 200, 0)      # 집기 영역 경계
COLOR_SLOT = (255, 0, 255)      # 놓기 칸


def _text(img, s, org, color, scale=0.55, bg=True):
    font = cv2.FONT_HERSHEY_SIMPLEX
    x, y = int(org[0]), int(org[1])
    if bg:
        (w, h), base = cv2.getTextSize(s, font, scale, 1)
        cv2.rectangle(img, (x - 2, y - h - 4), (x + w + 2, y + base), (0, 0, 0), -1)
    cv2.putText(img, s, (x, y), font, scale, color, 1, cv2.LINE_AA)


def draw_detections(img, sticks, mapper=None, target=None, info_lines=(), targets=()):
    """img(BGR 또는 마스크) 위에 검출 결과를 그린 복사본을 돌려준다.

    target  : 수동 선택한 막대 1개 (TARGET 표시)
    targets : 음성/텍스트 명령의 실행 계획 [(막대, "1: -> P1"), ...] (여러 개 가능)
    """
    out = img.copy() if img.ndim == 3 else cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    if mapper is not None:
        (x0, x1), (y0, y1) = config.PICK_X_RANGE, config.PICK_Y_RANGE
        zone = mapper.to_pixel([[x0, y0], [x1, y0], [x1, y1], [x0, y1]]).astype(np.int32)
        cv2.polylines(out, [zone], True, COLOR_ZONE, 2)
        top_left = zone[np.argmin(zone.sum(axis=1))]     # 화면에서 왼쪽 위 꼭짓점에 라벨
        _text(out, "PICK ZONE", top_left + np.array([4, 18]), COLOR_ZONE, 0.5)
        for n, (sx, sy) in config.PLACE_SLOTS.items():
            u, v = mapper.to_pixel([[sx, sy]])[0]
            cv2.circle(out, (int(u), int(v)), 18, COLOR_SLOT, 2)
            _text(out, f"P{n}", (u - 12, v - 24), COLOR_SLOT)

    for s in sticks:
        color = COLOR_OK if (s.in_pick_zone or not s.has_robot_coords) else COLOR_OUT
        cv2.polylines(out, [s.box_px.astype(np.int32)], True, color, 2)
        a = (int(s.end_a_px[0]), int(s.end_a_px[1]))
        b = (int(s.end_b_px[0]), int(s.end_b_px[1]))
        cv2.line(out, a, b, color, 1)
        c = (int(s.center_px[0]), int(s.center_px[1]))
        cv2.circle(out, c, 4, color, -1)
        if s.has_robot_coords:
            label = f"#{s.idx} {s.length:.0f}mm"
            sub = f"({s.x:.0f},{s.y:.0f}) {s.angle:+.0f}deg"
        else:
            label = f"#{s.idx} {s.length_px:.0f}px"
            sub = f"{s.angle_px:+.0f}deg"
        _text(out, label, (c[0] + 8, c[1] - 8), color)
        _text(out, sub, (c[0] + 8, c[1] + 14), color, 0.45)

    if target is not None:
        cv2.polylines(out, [target.box_px.astype(np.int32)], True, COLOR_TARGET, 4)
        tc = (int(target.center_px[0]), int(target.center_px[1]))
        _text(out, f"TARGET #{target.idx}", (tc[0] + 8, tc[1] - 30), COLOR_TARGET, 0.7)

    for st, label in targets:
        cv2.polylines(out, [st.box_px.astype(np.int32)], True, COLOR_TARGET, 4)
        tc = (int(st.center_px[0]), int(st.center_px[1]))
        _text(out, label, (tc[0] + 8, tc[1] - 30), COLOR_TARGET, 0.7)

    y = 24
    for line in info_lines:
        _text(out, line, (10, y), (255, 255, 255), 0.55)
        y += 24
    return out


# ============================================================
# 카메라 열기
# ============================================================
def open_camera(index=config.CAMERA_INDEX):
    """카메라를 열고 설정을 적용한다. (cap, 메시지 목록)을 돌려준다. 실패하면 cap=None.

    cap.set()이 True를 돌려줘도 실제로는 적용 안 되는 카메라가 많다.
    초점·노출이 고정됐는지는 화면을 보고 확인해야 한다.
    """
    msgs = []
    if sys.platform.startswith("win") and config.CAMERA_USE_DSHOW:
        cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
    else:
        cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        return None, [f"카메라 {index} 열기 실패 (번호 확인, 다른 프로그램이 쓰고 있는지 확인)"]

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)
    if config.CAMERA_DISABLE_AUTOFOCUS:
        ok1 = cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        ok2 = cap.set(cv2.CAP_PROP_FOCUS, config.CAMERA_FOCUS)
        msgs.append(f"자동초점 끄기 요청 {'수락' if ok1 else '거부'}, 초점 고정 요청 {'수락' if ok2 else '거부'}")
    if config.CAMERA_LOCK_EXPOSURE:
        ok = cap.set(cv2.CAP_PROP_EXPOSURE, config.CAMERA_EXPOSURE)
        msgs.append(f"노출 고정 요청 {'수락' if ok else '거부'}")

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    msgs.append(f"카메라 {index} 열림: {w}x{h}")
    return cap, msgs


# ============================================================
# 단독 실행: 트랙바로 파라미터를 맞추며 검출 결과 확인
# ============================================================
_TRACKBARS = [  # (트랙바 이름, 파라미터 이름, 최대값)
    ("H low", "h_low", 179), ("H high", "h_high", 179),
    ("S low", "s_low", 255), ("S high", "s_high", 255),
    ("V low", "v_low", 255), ("V high", "v_high", 255),
    ("min area", "min_area", 20000), ("aspect x10", "min_aspect", 200),
    ("blur", "blur", 15), ("invert", "invert", 1),
]


def _params_from_trackbars(win):
    vals = {name: cv2.getTrackbarPos(label, win) for label, name, _ in _TRACKBARS}
    vals["min_aspect"] = max(1.0, vals["min_aspect"] / 10.0)
    vals["invert"] = bool(vals["invert"])
    return VisionParams(**vals)


def _run_standalone(args):
    det = StickDetector()
    und = Undistorter()
    win, pwin = "stick detection", "params"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.namedWindow(pwin, cv2.WINDOW_NORMAL)
    for label, name, mx in _TRACKBARS:
        val = getattr(det.params, name)
        if name == "min_aspect":
            val = int(round(val * 10))
        cv2.createTrackbar(label, pwin, int(val), mx, lambda _v: None)

    cap, still = None, None
    if args.image:
        still = cv2.imread(args.image)
        if still is None:
            print(f"사진을 읽을 수 없음: {args.image}")
            return
    else:
        cap, msgs = open_camera(args.camera)
        for m in msgs:
            print("[camera]", m)
        if cap is None:
            return
    if und.enabled:
        print("[camera] 렌즈 왜곡 보정 적용")

    mapper, show_mask, fps, t_prev = None, False, 0.0, time.time()
    sticks = []
    try:
        while True:
            if cap is not None:
                ok, frame = cap.read()
                if not ok:
                    print("프레임 읽기 실패")
                    break
            else:
                frame = still.copy()
            frame = und(frame)
            if mapper is None:
                mapper = load_mapper(frame.shape[1], frame.shape[0])
                print("[map]", "없음" if mapper is None else mapper.source)

            det.params = _params_from_trackbars(pwin)
            sticks, mask = det.detect(frame, mapper)

            now = time.time()
            dt = now - t_prev
            t_prev = now
            if dt > 0:
                fps = 0.9 * fps + 0.1 * (1.0 / dt)
            map_txt = "none" if mapper is None else ("DUMMY (not real mm)" if mapper.is_dummy else "CALIBRATED")
            info = [f"sticks: {len(sticks)}   fps: {fps:.1f}",
                    f"map: {map_txt}",
                    "q:quit  m:mask  s:save  p:params  d:print"]
            cv2.imshow(win, draw_detections(mask if show_mask else frame, sticks, mapper, info_lines=info))

            key = cv2.waitKey(1 if cap is not None else 30) & 0xFF
            if key in (ord("q"), 27):
                break
            elif key == ord("m"):
                show_mask = not show_mask
            elif key == ord("s"):
                config.CAPTURE_DIR.mkdir(parents=True, exist_ok=True)
                path = config.CAPTURE_DIR / time.strftime("frame_%Y%m%d_%H%M%S.png")
                cv2.imwrite(str(path), frame)
                print("[save]", path)
            elif key == ord("p"):
                det.params.save()
                print("[params] 저장:", config.VISION_PARAMS_FILE)
            elif key == ord("d"):
                print(f"--- 막대 {len(sticks)}개 ---")
                for s in sticks:
                    if s.has_robot_coords:
                        print(f"#{s.idx}: x={s.x:.1f} y={s.y:.1f} angle={s.angle:+.1f} "
                              f"len={s.length:.1f}mm zone={s.in_pick_zone}")
                    else:
                        print(f"#{s.idx}: center={s.center_px} len={s.length_px:.1f}px angle={s.angle_px:+.1f}")
    finally:
        if cap is not None:
            cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="막대 검출 단독 테스트")
    ap.add_argument("--camera", type=int, default=config.CAMERA_INDEX)
    ap.add_argument("--image", type=str, default=None)
    _run_standalone(ap.parse_args())
