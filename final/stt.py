"""
음성 인식 (6단계): Vosk 한국어 소형 모델 + 누르고 말하기

설치
  pip install vosk sounddevice
  모델: https://alphacephei.com/vosk/models → vosk-model-small-ko-0.22.zip
        → models 폴더에 압축 해제 (config.VOSK_MODEL_DIR)

인식 방식 (config.STT_MODE)
  "grammar_then_free"  먼저 명령어 문법(intent.grammar_phrases)으로 제한해서 인식,
                       결과가 비면 제한 없이 다시 인식 (기본값)
  "free"               제한 없이 인식
  "grammar"            문법 제한만
  ※ 문법 제한은 정해진 명령에 강하지만, 엉뚱한 말도 비슷한 명령으로 맞춰 버릴 수 있다
    → GUI는 항상 "승인" 단계를 거친다.
  ※ 누르고 있는 동안 녹음 → 손을 떼면 한 번에 인식 (녹음 중 실시간 인식 아님)

단독 실행 (kw2026 activate 후)
  python stt.py --devices      # 마이크 목록
  python stt.py                # Enter로 녹음 시작/끝 → 인식 결과·해석·시간 출력 (q = 종료)
  python stt.py --test 5       # 인식률 테스트: 명령 10개를 5번씩 말하기 → logs/stt_test_*.csv
  python stt.py --wav a.wav    # 녹음 파일(16kHz 모노 권장)로 인식
"""

import argparse
import csv
import json
import time
import wave

import numpy as np

import config

try:
    import vosk
    vosk.SetLogLevel(-1)
except ImportError:
    vosk = None

try:
    import sounddevice as sd
except (ImportError, OSError):     # PortAudio가 없으면 OSError
    sd = None

SAMPLE_RATE = 16000


def check_available():
    """(사용 가능 여부, 안내 문구)"""
    if vosk is None:
        return False, "vosk 미설치 → pip install vosk"
    if sd is None:
        return False, "sounddevice 사용 불가 → pip install sounddevice (마이크 라이브러리)"
    if not config.VOSK_MODEL_DIR.exists():
        return False, f"모델 폴더 없음: {config.VOSK_MODEL_DIR}"
    return True, "사용 가능"


def resample(audio, src_rate, dst_rate=SAMPLE_RATE):
    """int16 모노 → dst_rate (선형 보간). 마이크가 16kHz를 지원하지 않을 때만 쓴다."""
    if src_rate == dst_rate or len(audio) == 0:
        return audio
    n_out = int(round(len(audio) * dst_rate / src_rate))
    x_old = np.linspace(0.0, 1.0, num=len(audio), endpoint=False)
    x_new = np.linspace(0.0, 1.0, num=n_out, endpoint=False)
    return np.interp(x_new, x_old, audio.astype(np.float32)).astype(np.int16)


# ============================================================
# 녹음 (누르고 말하기)
# ============================================================
class Recorder:
    def __init__(self, device=config.STT_DEVICE, max_seconds=config.STT_MAX_SECONDS):
        self.device = device
        self.max_seconds = max_seconds
        self.stream = None
        self.chunks = []
        self.rate = SAMPLE_RATE
        self.t0 = 0.0

    @property
    def recording(self):
        return self.stream is not None

    def _callback(self, indata, _frames, _time_info, _status):
        if time.time() - self.t0 <= self.max_seconds:
            self.chunks.append(indata[:, 0].copy())

    def start(self):
        if sd is None:
            raise RuntimeError("sounddevice 미설치")
        self.chunks = []
        self.t0 = time.time()
        try:
            self.stream = sd.InputStream(samplerate=SAMPLE_RATE, channels=1, dtype="int16",
                                         device=self.device, callback=self._callback)
            self.rate = SAMPLE_RATE
        except Exception:
            # 16kHz를 못 받는 장치 → 장치 기본 샘플레이트로 녹음 후 변환
            info = sd.query_devices(self.device, "input")
            self.rate = int(info["default_samplerate"])
            self.stream = sd.InputStream(samplerate=self.rate, channels=1, dtype="int16",
                                         device=self.device, callback=self._callback)
        self.stream.start()

    def stop(self):
        """녹음을 멈추고 16kHz int16 배열을 돌려준다."""
        if self.stream is None:
            return np.zeros(0, np.int16)
        try:
            self.stream.stop()
            self.stream.close()
        finally:
            self.stream = None
        audio = np.concatenate(self.chunks) if self.chunks else np.zeros(0, np.int16)
        return resample(audio, self.rate)

    def elapsed(self):
        return time.time() - self.t0 if self.recording else 0.0


# ============================================================
# 인식
# ============================================================
class SpeechRecognizer:
    def __init__(self, model_dir=config.VOSK_MODEL_DIR, phrases=None, mode=config.STT_MODE):
        if vosk is None:
            raise RuntimeError("vosk 미설치")
        t0 = time.time()
        self.model = vosk.Model(str(model_dir))
        self.load_seconds = time.time() - t0
        self.mode = mode
        self.grammar = json.dumps(phrases, ensure_ascii=False) if phrases else None
        self.grammar_error = None

    def _run(self, audio, grammar):
        if grammar:
            rec = vosk.KaldiRecognizer(self.model, SAMPLE_RATE, grammar)
        else:
            rec = vosk.KaldiRecognizer(self.model, SAMPLE_RATE)
        rec.AcceptWaveform(audio.astype(np.int16).tobytes())
        text = json.loads(rec.FinalResult()).get("text", "")
        return text.replace("[unk]", "").strip()

    def recognize(self, audio):
        """→ {"text", "mode", "grammar_text", "free_text", "seconds", "audio_seconds"}"""
        t0 = time.time()
        out = {"text": "", "mode": "", "grammar_text": "", "free_text": "",
               "audio_seconds": len(audio) / SAMPLE_RATE}
        use_grammar = self.mode in ("grammar", "grammar_then_free") and self.grammar and not self.grammar_error
        if use_grammar:
            try:
                out["grammar_text"] = self._run(audio, self.grammar)
            except Exception as e:      # 모델이 문법 제한을 지원하지 않는 경우 → 이후 자유 인식만
                self.grammar_error = str(e)
        if out["grammar_text"]:
            out["text"], out["mode"] = out["grammar_text"], "grammar"
        elif self.mode != "grammar":
            out["free_text"] = self._run(audio, None)
            out["text"], out["mode"] = out["free_text"], "free"
        out["seconds"] = time.time() - t0
        return out


def load_recognizer():
    """GUI·단독 실행 공용: 명령어 문법을 넣어서 인식기를 만든다."""
    from intent import grammar_phrases
    return SpeechRecognizer(phrases=grammar_phrases())


# ============================================================
# 단독 실행
# ============================================================
TEST_SENTENCES = [   # 인식률 테스트용 (1일차 테스트, 결과표에 사용)
    "왼쪽 막대 집어",
    "오른쪽 막대 집어",
    "가운데 막대 집어",
    "왼쪽 막대 오른쪽에 놔",
    "긴 막대 집어",
    "짧은 막대 왼쪽에 놔",
    "두 번째 막대 집어",
    "길이 순서대로 정렬해",
    "홈으로 가",
    "실행",
]


def _same_intent(a, b):
    if a is None or b is None:
        return False
    return (a.action, a.target, a.dest_slot, a.dest_side, a.order) == (b.action, b.target, b.dest_slot,
                                                                     b.dest_side, b.order)


def _record_console():
    rec = Recorder()
    input("  Enter → 녹음 시작 ")
    rec.start()
    input("  말한 뒤 Enter → 녹음 끝 ")
    return rec.stop()


def _judge(text, expected):
    """인식 문장이 기대 문장과 '같은 명령'인지 (승인어는 승인어끼리)."""
    from intent import is_confirm, parse
    if is_confirm(expected):
        return is_confirm(text)
    return _same_intent(parse(text).intent, parse(expected).intent)


def run_interactive(r):
    from intent import is_cancel, is_confirm, parse
    print("말하고 싶은 명령을 녹음하세요. (q = 종료)")
    while True:
        if input("\nEnter = 녹음 준비 / q = 종료: ").strip().lower() == "q":
            break
        audio = _record_console()
        if len(audio) < config.STT_MIN_SECONDS * SAMPLE_RATE:
            print("  너무 짧음")
            continue
        res = r.recognize(audio)
        print(f"  인식({res['mode']}): '{res['text']}'   [문법 '{res['grammar_text']}' / 자유 '{res['free_text']}']")
        print(f"  녹음 {res['audio_seconds']:.1f}s, 인식 {res['seconds']:.2f}s")
        p = parse(res["text"])
        print("  해석:", p.intent.describe() if p.intent else p.error,
              "| 승인어" if is_confirm(res["text"]) else "", "| 취소어" if is_cancel(res["text"]) else "")


def run_test(r, repeat):
    rows = []
    print(f"인식률 테스트: 문장 {len(TEST_SENTENCES)}개 × {repeat}회. 화면의 문장을 그대로 말하세요.")
    for sent in TEST_SENTENCES:
        for k in range(1, repeat + 1):
            print(f"\n[{sent}]  ({k}/{repeat})")
            audio = _record_console()
            res = r.recognize(audio)
            ok = _judge(res["text"], sent)
            print(f"  → '{res['text']}' ({res['mode']}, {res['seconds']:.2f}s)  {'정답' if ok else '오답'}")
            rows.append([sent, k, res["text"], res["mode"], f"{res['seconds']:.2f}", int(ok)])

    print("\n===== 결과 =====")
    for sent in TEST_SENTENCES:
        rs = [x for x in rows if x[0] == sent]
        print(f"  {sent:<16} {sum(x[5] for x in rs)}/{len(rs)}")
    total = sum(x[5] for x in rows)
    avg_t = np.mean([float(x[4]) for x in rows]) if rows else 0
    print(f"  전체 {total}/{len(rows)} = {100 * total / max(1, len(rows)):.0f}%,  평균 인식 시간 {avg_t:.2f}s")
    config.LOG_DIR.mkdir(parents=True, exist_ok=True)
    path = config.LOG_DIR / time.strftime("stt_test_%Y%m%d_%H%M%S.csv")
    with open(path, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.writer(f)
        w.writerow(["expected", "trial", "recognized", "mode", "seconds", "correct"])
        w.writerows(rows)
    print("저장:", path)


def read_wav(path):
    with wave.open(path, "rb") as wf:
        if wf.getsampwidth() != 2:
            raise SystemExit("16비트 PCM wav만 지원")
        data = np.frombuffer(wf.readframes(wf.getnframes()), np.int16)
        if wf.getnchannels() > 1:
            data = data.reshape(-1, wf.getnchannels())[:, 0]
        return resample(data, wf.getframerate())


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Vosk 음성 인식 테스트")
    ap.add_argument("--devices", action="store_true", help="마이크 목록 출력")
    ap.add_argument("--test", type=int, default=0, help="인식률 테스트 반복 횟수")
    ap.add_argument("--wav", default=None)
    ap.add_argument("--mode", default=config.STT_MODE, choices=["grammar_then_free", "free", "grammar"])
    a = ap.parse_args()

    if a.devices:
        if sd is None:
            raise SystemExit("sounddevice 미설치")
        print(sd.query_devices())
        raise SystemExit

    ok, msg = check_available()
    if not ok and not (a.wav and vosk is not None and config.VOSK_MODEL_DIR.exists()):
        raise SystemExit(f"STT 사용 불가: {msg}")
    print("모델 로드 중...")
    recognizer = load_recognizer()
    recognizer.mode = a.mode
    print(f"로드 완료 ({recognizer.load_seconds:.1f}s), 모드 = {recognizer.mode}")

    if a.wav:
        res = recognizer.recognize(read_wav(a.wav))
        print(json.dumps(res, ensure_ascii=False, indent=2))
    elif a.test:
        run_test(recognizer, a.test)
    else:
        run_interactive(recognizer)
