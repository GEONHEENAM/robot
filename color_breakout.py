"""
Color Breakout - 색 물체 추적으로 조종하는 벽돌깨기 (OpenCV만 사용)

방법
  1. 색이 선명한 물체(빨간 펜뚜껑, 파란 병뚜껑, 형광 포스트잇 등)를 카메라에 보여줌
  2. 화면에서 그 물체를 마우스로 클릭 -> 색 자동 등록
  3. 물체를 좌우로 움직이면 패들이 따라감
  4. SPACE로 공 발사

조작
  마우스 클릭 : 추적할 색 등록 (CALIB 상태에서)
  SPACE       : 공 발사 / 재시작
  C           : 색 다시 등록 (게임 상태 유지)
  M           : 색 마스크 미리보기 on/off (튜닝용)
  Q/ESC       : 종료

실행
  (kw2026) C:\\work_kw> python color_breakout.py
"""
import math
import random
import time

import cv2
import numpy as np

# ---------------- 카메라 ----------------
CAM_INDEX = 0               # 웹캠 번호 (안 열리면 1로)
FRAME_W, FRAME_H = 640, 480

# ---------------- 색 추적 ----------------
SAMPLE_HALF = 5             # 클릭 지점 주변 (2*5+1)^2 픽셀로 색 샘플링
H_TOL = 10                  # Hue 허용 범위 (+-), OpenCV Hue는 0~179
S_TOL = 70                  # Saturation 허용 범위 (-)
V_TOL = 70                  # Value 허용 범위 (-)
S_MIN = 60                  # 이보다 채도가 낮으면 무시 (흰색/회색/검정 배경 제거)
V_MIN = 50                  # 이보다 어두우면 무시
MIN_AREA = 400              # 이보다 작은 덩어리는 노이즈로 간주
SMOOTH = 0.5                # 패들 이동 평활화 (0~1, 클수록 빠르게 반응, 작을수록 부드러움)
TRACK_MARGIN = 80           # 화면 좌우 끝까지 손을 안 뻗어도 패들이 끝까지 가도록 매핑 여유

# ---------------- 게임 ----------------
HUD_H = 45
PADDLE_W, PADDLE_H = 110, 14
PADDLE_BOTTOM = 40          # 화면 아래에서 패들까지 거리
BALL_R = 8
BALL_SPEED = 330.0          # px/s (프레임레이트와 무관하게 시간 기반 이동)
SPEED_UP = 6.0              # 벽돌 하나 깰 때마다 속도 증가량
MAX_SPEED = 650.0
MAX_BOUNCE_DEG = 60         # 패들 끝에 맞을수록 이 각도까지 꺾여 튐
BRICK_ROWS, BRICK_COLS = 5, 10
BRICK_H, BRICK_GAP = 20, 4
BRICK_TOP = HUD_H + 15
LIVES = 3
BG_DIM = 0.45               # 배경 웹캠 영상 밝기 (게임 요소가 잘 보이게 어둡게)
MAX_DT = 0.05               # 프레임이 끊겨도 공이 벽을 뚫지 않게 dt 상한

WIN_NAME = "Color Breakout"

# BGR
ROW_COLORS = [(60, 60, 230), (40, 150, 255), (40, 220, 240), (80, 200, 80), (230, 160, 60)]
COLOR_PADDLE = (240, 240, 240)
COLOR_BALL = (255, 255, 255)
COLOR_TRACK = (0, 255, 0)
COLOR_WARN = (0, 0, 255)
COLOR_TEXT = (255, 255, 255)
COLOR_OUTLINE = (0, 0, 0)


# ============================================================
# 색 추적
# ============================================================
def sample_color(hsv, x, y):
    """클릭 지점 주변 픽셀의 대표 HSV.
    Hue는 0과 179가 이웃한 원형 값이라 (빨간색) 단순 평균/중앙값을 쓰면 틀림 -> 원형 평균 사용."""
    h_img, w_img = hsv.shape[:2]
    x0, x1 = max(0, x - SAMPLE_HALF), min(w_img, x + SAMPLE_HALF + 1)
    y0, y1 = max(0, y - SAMPLE_HALF), min(h_img, y + SAMPLE_HALF + 1)
    patch = hsv[y0:y1, x0:x1].reshape(-1, 3).astype(np.float64)

    ang = patch[:, 0] * (math.pi / 90.0)          # 0~180 -> 0~2pi
    mean_ang = math.atan2(np.sin(ang).mean(), np.cos(ang).mean())
    h = int(round((mean_ang * 90.0 / math.pi) % 180)) % 180
    s = int(np.median(patch[:, 1]))
    v = int(np.median(patch[:, 2]))
    return h, s, v


def color_mask(hsv, target):
    """등록된 색 기준으로 이진 마스크 생성 (Hue가 0/179 경계를 넘는 경우 두 구간으로 나눔)"""
    h, s, v = target
    lo_s, lo_v = max(S_MIN, s - S_TOL), max(V_MIN, v - V_TOL)
    lo_h, hi_h = h - H_TOL, h + H_TOL

    if lo_h < 0:
        m1 = cv2.inRange(hsv, (0, lo_s, lo_v), (hi_h, 255, 255))
        m2 = cv2.inRange(hsv, (180 + lo_h, lo_s, lo_v), (179, 255, 255))
        mask = cv2.bitwise_or(m1, m2)
    elif hi_h > 179:
        m1 = cv2.inRange(hsv, (lo_h, lo_s, lo_v), (179, 255, 255))
        m2 = cv2.inRange(hsv, (0, lo_s, lo_v), (hi_h - 180, 255, 255))
        mask = cv2.bitwise_or(m1, m2)
    else:
        mask = cv2.inRange(hsv, (lo_h, lo_s, lo_v), (hi_h, 255, 255))

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)   # 작은 점 노이즈 제거
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # 물체 내부 구멍 메우기
    return mask


def find_object(mask):
    """가장 큰 색 덩어리의 중심 (없으면 None)"""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    c = max(contours, key=cv2.contourArea)
    if cv2.contourArea(c) < MIN_AREA:
        return None
    m = cv2.moments(c)
    return int(m["m10"] / m["m00"]), int(m["m01"] / m["m00"]), c


def map_to_paddle(cx, w):
    """물체 x좌표 -> 패들 중심 x좌표 (가운데 영역을 화면 전체 폭으로 늘림)"""
    t = (cx - TRACK_MARGIN) / max(1, w - 2 * TRACK_MARGIN)
    t = min(max(t, 0.0), 1.0)
    return PADDLE_W / 2 + t * (w - PADDLE_W)


# ============================================================
# 게임 로직
# ============================================================
def make_bricks(w):
    bw = (w - BRICK_GAP * (BRICK_COLS + 1)) / BRICK_COLS
    bricks = []
    for r in range(BRICK_ROWS):
        for c in range(BRICK_COLS):
            x = int(BRICK_GAP + c * (bw + BRICK_GAP))
            y = BRICK_TOP + r * (BRICK_H + BRICK_GAP)
            bricks.append([x, y, int(bw), BRICK_H, r])  # x, y, w, h, row
    return bricks


def circle_hits_rect(cx, cy, r, rect):
    x, y, w, h = rect[:4]
    nx = min(max(cx, x), x + w)
    ny = min(max(cy, y), y + h)
    return (cx - nx) ** 2 + (cy - ny) ** 2 <= r * r


def hit_brick_index(ball, bricks):
    for i, b in enumerate(bricks):
        if circle_hits_rect(ball["x"], ball["y"], BALL_R, b):
            return i
    return None


def speed_up(ball):
    spd = math.hypot(ball["vx"], ball["vy"])
    new = min(MAX_SPEED, spd + SPEED_UP)
    ball["vx"] *= new / spd
    ball["vy"] *= new / spd


def break_brick(ball, bricks, i):
    row = bricks.pop(i)[4]
    speed_up(ball)
    return (BRICK_ROWS - row) * 10   # 위쪽 줄일수록 점수 높음


def update_ball(ball, dt, bricks, paddle, w, h):
    """공 이동 + 충돌 처리. (얻은 점수, 공 놓침 여부) 반환
    x축과 y축을 따로 움직여서 어느 면에 부딪혔는지 판단 (단순하지만 벽돌깨기엔 충분)"""
    gained = 0

    # X축
    old_x = ball["x"]
    ball["x"] += ball["vx"] * dt
    if ball["x"] < BALL_R:
        ball["x"], ball["vx"] = BALL_R, abs(ball["vx"])
    elif ball["x"] > w - BALL_R:
        ball["x"], ball["vx"] = w - BALL_R, -abs(ball["vx"])
    i = hit_brick_index(ball, bricks)
    if i is not None:
        ball["x"] = old_x
        ball["vx"] = -ball["vx"]
        gained += break_brick(ball, bricks, i)

    # Y축
    old_y = ball["y"]
    ball["y"] += ball["vy"] * dt
    if ball["y"] < HUD_H + BALL_R:
        ball["y"], ball["vy"] = HUD_H + BALL_R, abs(ball["vy"])
    i = hit_brick_index(ball, bricks)
    if i is not None:
        ball["y"] = old_y
        ball["vy"] = -ball["vy"]
        gained += break_brick(ball, bricks, i)

    # 패들: 맞은 위치에 따라 반사각 결정 (가운데 = 수직, 끝 = 최대 각도)
    if ball["vy"] > 0 and circle_hits_rect(ball["x"], ball["y"], BALL_R, paddle):
        center = paddle[0] + paddle[2] / 2
        off = min(max((ball["x"] - center) / (paddle[2] / 2), -1.0), 1.0)
        ang = math.radians(off * MAX_BOUNCE_DEG)
        spd = math.hypot(ball["vx"], ball["vy"])
        ball["vx"] = spd * math.sin(ang)
        ball["vy"] = -abs(spd * math.cos(ang))
        ball["y"] = paddle[1] - BALL_R - 1

    lost = ball["y"] - BALL_R > h
    return gained, lost


def launch(ball):
    ang = math.radians(random.uniform(-30, 30))
    ball["vx"] = BALL_SPEED * math.sin(ang)
    ball["vy"] = -BALL_SPEED * math.cos(ang)


# ============================================================
# 그리기
# ============================================================
def put_text(img, text, org, scale=0.8, thickness=2, color=COLOR_TEXT):
    """외곽선 텍스트 (cv2.putText는 한글 미지원)"""
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale,
                COLOR_OUTLINE, thickness + 3, cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale,
                color, thickness, cv2.LINE_AA)


def put_center(img, text, y, scale=1.0, thickness=2, color=COLOR_TEXT):
    (tw, _), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, thickness)
    put_text(img, text, ((img.shape[1] - tw) // 2, y), scale, thickness, color)


def draw_game(img, bricks, paddle, ball):
    for x, y, bw, bh, row in bricks:
        cv2.rectangle(img, (x, y), (x + bw, y + bh), ROW_COLORS[row % len(ROW_COLORS)], -1)
        cv2.rectangle(img, (x, y), (x + bw, y + bh), (30, 30, 30), 1)
    px, py, pw, ph = paddle
    cv2.rectangle(img, (px, py), (px + pw, py + ph), COLOR_PADDLE, -1)
    cv2.circle(img, (int(ball["x"]), int(ball["y"])), BALL_R, COLOR_BALL, -1, cv2.LINE_AA)


# ============================================================
# 메인
# ============================================================
def main():
    cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_DSHOW)  # Windows에서 열기 속도 개선
    if not cap.isOpened():
        print(f"[ERROR] 웹캠 {CAM_INDEX}번을 열 수 없음. CAM_INDEX를 바꿔보세요.")
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)

    ok, frame = cap.read()
    if not ok:
        print("[ERROR] 프레임을 읽을 수 없음")
        return
    h, w = frame.shape[:2]

    # 마우스 콜백과 공유할 상태
    shared = {"hsv": None, "target": None, "warn": ""}

    def on_mouse(event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN or shared["hsv"] is None:
            return
        hh, ss, vv = sample_color(shared["hsv"], x, y)
        if ss < S_MIN or vv < V_MIN:
            shared["warn"] = "Too dull/dark. Click a vivid colored object."
            return
        shared["target"] = (hh, ss, vv)
        shared["warn"] = ""
        print(f"[INFO] 색 등록: H={hh} S={ss} V={vv}")

    cv2.namedWindow(WIN_NAME)
    cv2.setMouseCallback(WIN_NAME, on_mouse)

    state = "CALIB"      # CALIB -> READY -> PLAYING -> (READY ...) -> WIN / OVER
    bricks = make_bricks(w)
    score, lives = 0, LIVES
    paddle_x = w / 2
    paddle_y = h - PADDLE_BOTTOM
    ball = {"x": paddle_x, "y": paddle_y - BALL_R - 2, "vx": 0.0, "vy": 0.0}
    show_mask = False
    last_t = time.time()

    while True:
        ok, frame = cap.read()
        if not ok:
            print("[ERROR] 프레임을 읽을 수 없음")
            break
        frame = cv2.flip(frame, 1)  # 거울 모드
        now = time.time()
        dt = min(now - last_t, MAX_DT)
        last_t = now

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        shared["hsv"] = hsv

        if state == "CALIB" and shared["target"] is not None:
            state = "READY"

        # ---------------- 색 추적 ----------------
        mask, obj = None, None
        if shared["target"] is not None:
            mask = color_mask(hsv, shared["target"])
            obj = find_object(mask)
            if obj is not None:
                target_x = map_to_paddle(obj[0], w)
                paddle_x += SMOOTH * (target_x - paddle_x)

        paddle = (int(paddle_x - PADDLE_W / 2), paddle_y, PADDLE_W, PADDLE_H)

        # ---------------- 게임 업데이트 ----------------
        if state == "READY":
            ball["x"], ball["y"] = paddle_x, paddle_y - BALL_R - 2
        elif state == "PLAYING" and obj is not None:   # 물체를 놓치면 일시정지
            gained, lost = update_ball(ball, dt, bricks, paddle, w, h)
            score += gained
            if not bricks:
                state = "WIN"
            elif lost:
                lives -= 1
                state = "OVER" if lives <= 0 else "READY"

        # ---------------- 그리기 ----------------
        view = cv2.convertScaleAbs(frame, alpha=BG_DIM)
        if obj is not None:
            cv2.drawContours(view, [obj[2]], -1, COLOR_TRACK, 2)
            cv2.circle(view, (obj[0], obj[1]), 5, COLOR_TRACK, -1)

        if state != "CALIB":
            draw_game(view, bricks, paddle, ball)
            put_text(view, f"SCORE {score}", (15, 32))
            put_text(view, f"LIVES {lives}", (220, 32))
            put_text(view, f"BRICKS {len(bricks)}", (w - 190, 32))

        if state == "CALIB":
            view = frame.copy()   # 색 고르기 쉽게 원본 밝기
            put_center(view, "Hold a COLORED object", h // 2 - 30, 0.9)
            put_center(view, "and CLICK on it", h // 2 + 10, 0.9)
            if shared["warn"]:
                put_center(view, shared["warn"], h // 2 + 60, 0.6, color=COLOR_WARN)
        elif state == "READY":
            put_center(view, "Press SPACE to launch", h // 2 + 40, 0.9)
        elif state == "WIN":
            put_center(view, "YOU WIN!", h // 2, 1.6, 3, color=(80, 220, 80))
            put_center(view, f"SCORE {score}   SPACE: restart", h // 2 + 50, 0.8)
        elif state == "OVER":
            put_center(view, "GAME OVER", h // 2, 1.6, 3, color=COLOR_WARN)
            put_center(view, f"SCORE {score}   SPACE: restart", h // 2 + 50, 0.8)

        if state in ("READY", "PLAYING") and obj is None:
            put_center(view, "OBJECT LOST - paused", h // 2 + 80, 0.8, color=COLOR_WARN)

        if show_mask and mask is not None:
            small = cv2.resize(mask, (w // 4, h // 4))
            view[h - h // 4:h, w - w // 4:w] = cv2.cvtColor(small, cv2.COLOR_GRAY2BGR)

        cv2.imshow(WIN_NAME, view)

        # ---------------- 입력 ----------------
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord("m"):
            show_mask = not show_mask
        if key == ord("c"):
            if state in ("WIN", "OVER"):   # 끝난 게임이면 새 게임으로
                bricks, score, lives = make_bricks(w), 0, LIVES
            shared["target"] = None        # 진행 중이면 벽돌/점수 유지, 공만 패들로 복귀
            state = "CALIB"
        if key == ord(" "):
            if state == "READY" and obj is not None:
                launch(ball)
                state = "PLAYING"
            elif state in ("WIN", "OVER"):
                bricks, score, lives = make_bricks(w), 0, LIVES
                state = "READY"

        if cv2.getWindowProperty(WIN_NAME, cv2.WND_PROP_VISIBLE) < 1:
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()