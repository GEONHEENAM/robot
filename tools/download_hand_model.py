"""MediaPipe Hand Landmarker 모델(hand_landmarker.task)을 models/ 에 받는다.

실행: python tools/download_hand_model.py
모델은 용량 때문에 git에 올리지 않는다(.gitignore). 각자 PC에서 한 번 실행.
"""

from __future__ import annotations

import sys
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from vision.hand_model import MODEL_PATH, MODEL_URL  # noqa: E402


def main() -> int:
    if MODEL_PATH.is_file():
        print(f"이미 있음: {MODEL_PATH} ({MODEL_PATH.stat().st_size / 1e6:.1f} MB)")
        return 0
    MODEL_PATH.parent.mkdir(parents=True, exist_ok=True)
    tmp = MODEL_PATH.with_suffix(".part")
    print(f"다운로드: {MODEL_URL}")
    try:
        urllib.request.urlretrieve(MODEL_URL, tmp)
    except OSError as e:
        tmp.unlink(missing_ok=True)
        print(f"실패: {e}")
        return 1
    tmp.replace(MODEL_PATH)
    print(f"저장: {MODEL_PATH} ({MODEL_PATH.stat().st_size / 1e6:.1f} MB)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
