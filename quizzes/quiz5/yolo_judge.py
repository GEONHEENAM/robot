"""퀴즈5 YOLO 판별 프로그램 (Yolo 폴더의 detect_dogs_ab.py 기준).

N장 사진에서 개(COCO class 16)를 검출하고, 검출된 모든 개의 신뢰도 평균이
기준 값(M%) 이상이면 A, 아니면 B로 판정한다. 개가 0마리면 B.
결과 5개(사진 수 / 개 수 / 평균 신뢰도 / 기준 값 / 판별 결과)는 Modbus 서버 40~49번에 쓴다.

GUI(quiz5_gui.py)가 judge()를 불러 쓰고, 단독 실행도 된다.

    python yolo_judge.py --dataset valid --n 20 --threshold 80
    python yolo_judge.py --dataset valid --n 20 --threshold 80 --conf 0.25 --imgsz 640 \\
                         --prep original --modbus 127.0.0.1:5020
"""

from __future__ import annotations

import argparse
import os
import sys
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

# ========================= 설정 =========================
# Yolo 실습 폴더 (input_images/, yolo11n.pt 가 있는 곳). GUI에서 바꿀 수 있다.
# PC마다 바탕 화면 위치가 달라서 (OneDrive 사용 여부, 탐색기에 "바탕 화면"으로 보여도
# 실제 폴더 이름은 "Desktop"인 경우 등) 아래 후보 중 실제로 있는 곳을 쓴다.
# 환경 변수 YOLO_ROOT를 지정하면 그것을 가장 먼저 쓴다.
MODEL_NAME = "yolo11n.pt"

# yolo_tune.py로 찾은 최적값을 여기에 적어 둔다 (GUI 기본값으로 쓰임)
CONF = 0.25
IMGSZ = 640
PREPROCESS = "original"

DOG_CLASS_ID = 16
PREPROCESS_CHOICES = ("original", "bright", "contrast")
DATASETS = ("tune", "test", "valid")
# OpenCV(imdecode)가 읽을 수 있는 형식. 웹에서 받은 사진은 webp/jfif인 경우가 많다.
IMAGE_EXT = {".jpg", ".jpeg", ".jfif", ".png", ".webp", ".bmp", ".tif", ".tiff"}


def _windows_desktop() -> Path | None:
    """Windows가 실제로 쓰는 바탕 화면 경로 (OneDrive 백업·폴더 이동까지 반영)."""
    if sys.platform != "win32":
        return None
    try:
        import ctypes
        from ctypes import wintypes

        buf = ctypes.create_unicode_buffer(wintypes.MAX_PATH)
        # CSIDL_DESKTOPDIRECTORY = 0x10
        if ctypes.windll.shell32.SHGetFolderPathW(None, 0x10, None, 0, buf) == 0:
            return Path(buf.value)
    except Exception:
        pass
    return None


def root_candidates() -> list[Path]:
    home = Path.home()
    found = [Path(os.environ["YOLO_ROOT"])] if os.environ.get("YOLO_ROOT") else []
    desktop = _windows_desktop()
    if desktop is not None:
        found.append(desktop / "Yolo")
    for base in (home / "OneDrive" / "바탕 화면", home / "OneDrive" / "Desktop",
                 home / "바탕 화면", home / "Desktop"):
        found.append(base / "Yolo")
    found.append(Path(__file__).resolve().parent / "Yolo")
    return found


def find_default_root() -> Path:
    """input_images 폴더가 있는 첫 후보. 하나도 없으면 첫 후보를 그대로 돌려준다."""
    candidates = root_candidates()
    for c in candidates:
        if (c / "input_images").is_dir():
            return c
    return candidates[0]


DEFAULT_ROOT = find_default_root()


@dataclass
class PhotoResult:
    name: str            # 입력 폴더 기준 상대 경로
    scores: list[float]  # 이 사진에서 검출된 개들의 신뢰도 (0~1)


@dataclass
class JudgeResult:
    photos: int
    photos_with_dog: int
    dogs: int
    mean_percent: float | None   # 개 0마리면 None
    threshold_percent: float
    decision: str                # "A" / "B"
    conf: float
    imgsz: int
    preprocess: str
    out_dir: Path | None
    per_photo: list[PhotoResult] = field(default_factory=list)

    def summary(self) -> str:
        mean = "해당 없음" if self.mean_percent is None else f"{self.mean_percent:.2f}%"
        return (f"검사 {self.photos}장 | 개 검출 사진 {self.photos_with_dog}장 | "
                f"검출 개 {self.dogs}마리 | 평균 신뢰도 {mean} | "
                f"기준 {self.threshold_percent:g}% | 판정 {self.decision}")


# ========================= 공통 함수 =========================
def resolve_folder(root: Path, dataset: str) -> tuple[Path, Path]:
    """GUI/명령에서 받은 경로 -> (Yolo 폴더, 사진 폴더).

    Yolo 폴더 대신 input_images 나 input_images\\valid 를 직접 골라도 동작하게 한다.
    """
    root = Path(root)
    if root.name.lower() in DATASETS and root.parent.name.lower() == "input_images":
        return root.parent.parent, root
    if root.name.lower() == "input_images":
        return root.parent, root / dataset
    return root, root / "input_images" / dataset


def diagnose(folder: Path) -> str:
    """사진이 0장일 때 이유를 사람이 읽는 문장으로."""
    if not folder.exists():
        return f"폴더가 없습니다: {folder}"
    if not folder.is_dir():
        return f"폴더가 아닙니다: {folder}"
    files = [p for p in folder.rglob("*") if p.is_file()]
    if not files:
        return (f"폴더는 있지만 파일이 0개입니다: {folder} "
                "(OneDrive '온라인 전용'이면 '이 디바이스에 항상 유지'로 내려받기)")
    exts: dict[str, int] = {}
    for p in files:
        exts[p.suffix.lower() or "(없음)"] = exts.get(p.suffix.lower() or "(없음)", 0) + 1
    shown = ", ".join(f"{k} {v}개" for k, v in sorted(exts.items()))
    return f"{folder} 안의 파일 확장자: {shown} (사진으로 인정: {sorted(IMAGE_EXT)})"


def list_photos(folder: Path) -> list[Path]:
    """하위 폴더까지 사진 파일을 파일 이름순으로."""
    if not folder.is_dir():
        return []
    return sorted(p for p in folder.rglob("*")
                  if p.is_file() and p.suffix.lower() in IMAGE_EXT)


def read_image(path: Path) -> np.ndarray:
    """한글 경로도 읽히도록 바이트로 읽어서 디코딩."""
    data = np.frombuffer(path.read_bytes(), dtype=np.uint8)
    image = cv2.imdecode(data, cv2.IMREAD_COLOR) if data.size else None
    if image is None:
        raise OSError("이미지를 읽을 수 없습니다.")
    return image


def preprocess(image: np.ndarray, mode: str) -> np.ndarray:
    if mode == "original":
        return image
    if mode == "bright":
        return cv2.convertScaleAbs(image, alpha=1.0, beta=30)
    if mode == "contrast":
        ycrcb = cv2.cvtColor(image, cv2.COLOR_BGR2YCrCb)
        ycrcb[:, :, 0] = cv2.equalizeHist(ycrcb[:, :, 0])
        return cv2.cvtColor(ycrcb, cv2.COLOR_YCrCb2BGR)
    raise ValueError(f"PREPROCESS는 {PREPROCESS_CHOICES} 중 하나: {mode}")


def load_model(root: Path):
    """ultralytics는 무거워서 실제로 쓸 때 import 한다."""
    from ultralytics import YOLO

    root = Path(root)
    for path in (root / MODEL_NAME, root.parent / MODEL_NAME, Path(MODEL_NAME)):
        if path.exists():
            return YOLO(str(path))
    raise FileNotFoundError(f"모델 파일이 없습니다: {root / MODEL_NAME} "
                            "(Yolo 폴더에 yolo11n.pt를 넣으세요)")


def detect(model, image, conf, imgsz, mode):
    """한 장 검출 -> (박스 [[x1,y1,x2,y2], ...], 신뢰도 [..])"""
    result = model.predict(source=preprocess(image, mode), conf=conf, imgsz=imgsz,
                           classes=[DOG_CLASS_ID], verbose=False)[0]
    boxes = result.boxes.cpu()
    return boxes.xyxy.tolist(), boxes.conf.tolist()


def decide(scores: list[float], threshold_percent: float) -> tuple[float | None, str]:
    """검출된 모든 개의 신뢰도를 한 번씩 평균한다. 0마리면 (None, B)."""
    if not scores:
        return None, "B"
    mean = 100 * sum(scores) / len(scores)
    return mean, ("A" if mean >= threshold_percent else "B")


def draw_result(original, boxes, scores, conf, imgsz, mode):
    """원본 위에 박스와 신뢰도, 위쪽 띠에 설정값을 그린다 (detect_dogs.py와 같은 모양)."""
    canvas = original.copy()
    height, width = canvas.shape[:2]
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = max(0.5, min(height, width) / 800)
    thick = max(1, round(scale * 2))
    pad = max(4, round(scale * 6))
    white, dark = (255, 255, 255), (35, 35, 35)
    colors = [(40, 210, 90), (0, 165, 255), (255, 160, 60)]

    for i, (box, score) in enumerate(zip(boxes, scores, strict=True), start=1):
        x1, y1, x2, y2 = map(round, box)
        color = colors[(i - 1) % len(colors)]
        cv2.rectangle(canvas, (x1, y1), (x2, y2), color, thick + 1)
        text = f"Dog {i} | {score:.2f}"
        (tw, th), base = cv2.getTextSize(text, font, scale, thick)
        x = max(0, min(x1, width - tw - 2 * pad))
        y = max(th + pad, min(y1 - pad, height - base - pad))
        cv2.rectangle(canvas, (x, y - th - pad), (x + tw + 2 * pad, y + base), dark, cv2.FILLED)
        cv2.putText(canvas, text, (x + pad, y), font, scale, white, thick, cv2.LINE_AA)

    title = f"Dogs: {len(boxes)} | Conf: {conf} | ImgSz: {imgsz} | Prep: {mode}"
    (tw, th), base = cv2.getTextSize(title, font, scale, thick)
    top = th + base + 2 * pad
    right = max(0, tw + 2 * pad - width)
    canvas = cv2.copyMakeBorder(canvas, top, 0, 0, right, cv2.BORDER_CONSTANT, value=dark)
    cv2.putText(canvas, title, (pad, pad + th), font, scale, white, thick, cv2.LINE_AA)
    return canvas


def write_image(path: Path, image) -> None:
    ok, encoded = cv2.imencode(path.suffix.lower() or ".jpg", image)
    if not ok:
        raise OSError("결과 사진을 변환할 수 없습니다.")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(encoded.tobytes())


# ========================= 판별 =========================
def judge(model, photos: list[Path], folder: Path, threshold_percent: float,
          conf: float = CONF, imgsz: int = IMGSZ, mode: str = PREPROCESS,
          out_root: Path | None = None, progress=None) -> JudgeResult:
    """photos 전부를 검사해 A/B 판정. out_root를 주면 박스 그린 사진을 out_A|out_B/실행명/에 저장.

    progress(index, total, PhotoResult, annotated_bgr)가 있으면 사진마다 호출한다.
    한 장이라도 읽기 실패하면 OSError (판정 기준이 되는 사진 수가 달라지면 안 되므로 중단).
    """
    if mode not in PREPROCESS_CHOICES:
        raise ValueError(f"PREPROCESS는 {PREPROCESS_CHOICES} 중 하나: {mode}")
    if not photos:
        raise ValueError("검사할 사진이 없습니다.")

    run_name = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    staging = None if out_root is None else Path(out_root) / f"_running_{run_name}"
    results: list[PhotoResult] = []
    scores: list[float] = []

    for index, path in enumerate(photos, start=1):
        rel = path.relative_to(folder) if path.is_relative_to(folder) else Path(path.name)
        try:
            image = read_image(path)
            boxes, photo_scores = detect(model, image, conf, imgsz, mode)
            annotated = draw_result(image, boxes, photo_scores, conf, imgsz, mode)
            if staging is not None:
                write_image(staging / rel, annotated)
        except (OSError, cv2.error) as error:
            raise OSError(f"검사 중단: {rel} | {error}") from error
        item = PhotoResult(str(rel), photo_scores)
        results.append(item)
        scores.extend(photo_scores)
        if progress is not None:
            progress(index, len(photos), item, annotated)

    mean, decision = decide(scores, threshold_percent)

    out_dir = None
    if staging is not None:
        out_dir = Path(out_root) / f"out_{decision}" / run_name
        out_dir.parent.mkdir(parents=True, exist_ok=True)
        staging.rename(out_dir)

    return JudgeResult(
        photos=len(results),
        photos_with_dog=sum(1 for r in results if r.scores),
        dogs=len(scores),
        mean_percent=mean,
        threshold_percent=threshold_percent,
        decision=decision,
        conf=conf,
        imgsz=imgsz,
        preprocess=mode,
        out_dir=out_dir,
        per_photo=results,
    )


# ========================= Modbus 기록 (단독 실행용) =========================
def write_to_server(result: JudgeResult, host: str, port: int) -> None:
    """서버 40~49번에 결과를 쓴다. 판별 횟수(46)는 서버 값 +1."""
    import register_map as rm
    from pymodbus.client import ModbusTcpClient

    client = ModbusTcpClient(host, port=port, timeout=2)
    if not client.connect():
        raise ConnectionError(f"Modbus 서버 연결 실패: {host}:{port}")
    try:
        rr = client.read_holding_registers(rm.YOLO_RUN_COUNT, count=1, device_id=rm.DEVICE_ID)
        if rr.isError():
            raise ConnectionError(f"판별 횟수 읽기 실패: {rr}")
        values = rm.yolo_registers(
            result.photos, result.photos_with_dog, result.dogs, result.mean_percent,
            result.threshold_percent, result.decision, rr.registers[0] + 1,
            result.conf, result.imgsz, result.preprocess)
        wr = client.write_registers(rm.YOLO_PHOTOS, values, device_id=rm.DEVICE_ID)
        if wr.isError():
            raise ConnectionError(f"결과 쓰기 실패: {wr}")
    finally:
        client.close()


def main() -> int:
    ap = argparse.ArgumentParser(description="퀴즈5 YOLO A/B 판별")
    ap.add_argument("--root", type=Path, default=DEFAULT_ROOT, help="Yolo 실습 폴더")
    ap.add_argument("--dataset", choices=DATASETS, default="valid")
    ap.add_argument("--n", type=int, default=20, help="검사할 사진 수 (앞에서부터)")
    ap.add_argument("--threshold", type=float, default=80.0, help="평균 신뢰도 기준 %")
    ap.add_argument("--conf", type=float, default=CONF)
    ap.add_argument("--imgsz", type=int, default=IMGSZ)
    ap.add_argument("--prep", choices=PREPROCESS_CHOICES, default=PREPROCESS)
    ap.add_argument("--modbus", default="", help="결과를 쓸 서버 host:port (예: 127.0.0.1:5020)")
    args = ap.parse_args()

    args.root, folder = resolve_folder(args.root, args.dataset)
    photos = list_photos(folder)
    print(f"입력 폴더: {folder} | 전체 사진 {len(photos)}장")
    if len(photos) < args.n:
        print(f"사진이 {args.n}장보다 적습니다 ({len(photos)}장).")
        if not photos:
            print(diagnose(folder))
        return 1
    if not 0 <= args.threshold <= 100 or not 0 < args.conf <= 1:
        print("기준은 0~100, CONF는 0~1 사이여야 합니다.")
        return 1

    model = load_model(args.root)

    def show(i, total, item, _img):
        print(f"[{i}/{total}] {item.name} | 검출 {len(item.scores)}마리")

    result = judge(model, photos[:args.n], folder, args.threshold, args.conf, args.imgsz,
                   args.prep, out_root=args.root / "output_images", progress=show)
    print(result.summary())
    print(f"저장 폴더: {result.out_dir}")
    if args.modbus:
        host, _, port = args.modbus.partition(":")
        write_to_server(result, host, int(port or 5020))
        print(f"Modbus {args.modbus} 40~49번 기록 완료")
    print(result.decision)
    return 0


if __name__ == "__main__":
    sys.exit(main())
