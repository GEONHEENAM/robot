"""퀴즈5 최적 CONF / IMGSZ / PREPROCESS 찾기 (tune 사진 묶음으로).

모든 조합(CONF x IMGSZ x PREPROCESS)의 결과를 표로 출력하고 CSV로 저장한다.

    python yolo_tune.py                       # 정답 없이: 검출 사진 수 → 평균 신뢰도 순 정렬
    python yolo_tune.py --labels labels.csv   # 정답 있음: 사진별 개 수 일치 장수 순 정렬

labels.csv 형식 (사진마다 실제 개 수, 이름은 input_images/tune 기준 상대 경로):
    file,dogs
    dog01.jpg,1
    dog02.jpg,2

속도: CONF마다 추론을 다시 돌리지 않는다. (IMGSZ, PREPROCESS)별로 가장 낮은 CONF로
한 번만 추론하고, 더 높은 CONF는 결과 박스를 걸러서 계산한다.
(NMS는 높은 점수 박스가 낮은 점수 박스를 지우는 방식이라 결과가 같다)
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import yolo_judge as yj

CONF_LIST = [0.15, 0.25, 0.35, 0.45, 0.55]
IMGSZ_LIST = [320, 480, 640]


def load_labels(path: Path) -> dict[str, int]:
    with path.open(encoding="utf-8-sig", newline="") as f:
        return {row["file"].replace("\\", "/"): int(row["dogs"]) for row in csv.DictReader(f)}


def main() -> int:
    ap = argparse.ArgumentParser(description="CONF/IMGSZ/PREPROCESS 조합 비교")
    ap.add_argument("--root", type=Path, default=yj.DEFAULT_ROOT)
    ap.add_argument("--dataset", choices=yj.DATASETS, default="tune")
    ap.add_argument("--labels", type=Path, default=None, help="사진별 실제 개 수 CSV")
    ap.add_argument("--out", type=Path, default=Path("tune_result.csv"))
    args = ap.parse_args()

    folder = args.root / "input_images" / args.dataset
    photos = yj.list_photos(folder)
    if not photos:
        print(f"사진이 없습니다: {folder}")
        return 1
    labels = load_labels(args.labels) if args.labels else None
    names = [p.relative_to(folder).as_posix() for p in photos]
    if labels is not None:
        missing = [n for n in names if n not in labels]
        if missing:
            print(f"labels.csv에 없는 사진 {len(missing)}장: {missing[:5]} ...")
            return 1

    images = [yj.read_image(p) for p in photos]
    model = yj.load_model(args.root)
    low = min(CONF_LIST)
    rows = []
    for mode in yj.PREPROCESS_CHOICES:
        for imgsz in IMGSZ_LIST:
            # 사진별 신뢰도 목록 (가장 낮은 CONF로 한 번만 추론)
            raw = [yj.detect(model, img, low, imgsz, mode)[1] for img in images]
            print(f"추론 완료: PREPROCESS={mode}, IMGSZ={imgsz}")
            for conf in CONF_LIST:
                per = [[s for s in scores if s >= conf] for scores in raw]
                flat = [s for scores in per for s in scores]
                row = {
                    "conf": conf, "imgsz": imgsz, "preprocess": mode,
                    "photos_with_dog": sum(1 for s in per if s),
                    "dogs": len(flat),
                    "mean_percent": round(100 * sum(flat) / len(flat), 2) if flat else 0.0,
                }
                if labels is not None:
                    row["count_match"] = sum(
                        1 for n, s in zip(names, per, strict=True) if len(s) == labels[n])
                    row["count_error"] = sum(
                        abs(len(s) - labels[n]) for n, s in zip(names, per, strict=True))
                rows.append(row)

    if labels is not None:
        rows.sort(key=lambda r: (-r["count_match"], r["count_error"], -r["mean_percent"]))
    else:
        rows.sort(key=lambda r: (-r["photos_with_dog"], -r["mean_percent"]))

    with args.out.open("w", encoding="utf-8-sig", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    print(f"\n사진 {len(photos)}장, 조합 {len(rows)}개 → {args.out.resolve()}")
    print("상위 5개:")
    for r in rows[:5]:
        print("  " + ", ".join(f"{k}={v}" for k, v in r.items()))
    if labels is None:
        print("\n주의: 정답(labels.csv)이 없으면 '검출 사진 수'가 많은 순서일 뿐입니다.\n"
              "  CONF를 낮추면 오검출(개가 아닌 것)도 늘어나 개 수가 부풀 수 있고,\n"
              "  CONF를 높이면 낮은 점수 박스가 빠져 평균 신뢰도는 저절로 올라갑니다.\n"
              "  결과 사진(yolo_judge.py 저장본)을 눈으로 확인하고 고르세요.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
