"""WHISPER DUSK — 말을 받아쓰고 다듬는 부분.

    소리  →  ears(받아쓰기)  →  fixer(다듬기)  →  깨끗한 글자
                                                      ↓
                                            hints.py → RACHEL → 블록

★ 이름에 대하여
  'Whisper' 는 OpenAI 모델 이름이기도 하지만, 여기 Whisper Dusk 는
  그것과 아무 관계가 없다. 이 프로젝트에서 붙인 이름일 뿐이다.
    - ears  : Moonshine 한국어 27MB (Useful Sensors 의 오픈 모델)
    - fixer : 이 프로젝트에서 직접 만든 7.5MB 교정 모델
  OpenAI Whisper 는 쓰지 않는다.

왜 둘로 나뉘어 있나
  받아쓰기(ears)는 27MB 짜리라 정확도가 낮다. 대신 노트북에서 혼자
  돈다 — 대회장에서 네트워크가 끊겨도 말로 조작할 수 있다.
  그 대가로 말이 지저분해지는데, 그걸 fixer 가 다듬는다.

  AI 경계선 원칙은 그대로다. 둘 다 '말을 말로' 만 다룬다 —
  좌표·경로·제어에는 손대지 않는다.

쓰는 법
    import whisper_dusk as wd
    wd.listen(wav_bytes)      # 소리 -> 깨끗한 글자 (받아쓰기 + 다듬기)
    wd.hear(wav_bytes)        # 받아쓰기만 (다듬기 전 원문이 필요할 때)
    wd.polish("지저분한 말")   # 다듬기만
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def _ears():
    """받아쓰기 쪽 (Moonshine). 준비 안 됐으면 None."""
    d = os.path.join(HERE, "ears")
    if d not in sys.path:
        sys.path.insert(0, d)
    try:
        import stt
        return stt if stt.ready() else None
    except Exception as e:
        print(f"[WHISPER DUSK] 받아쓰기를 못 올렸습니다: {e}")
        return None


def _fixer():
    """다듬기 쪽 (자작). 준비 안 됐으면 None."""
    d = os.path.join(HERE, "fixer")
    if d not in sys.path:
        sys.path.insert(0, d)
    try:
        import fixer
        return fixer if fixer.ready() else None
    except Exception as e:
        print(f"[WHISPER DUSK] 다듬기를 못 올렸습니다: {e}")
        return None


def ready():
    """받아쓰기가 가능한가 (다듬기는 없어도 된다)."""
    return _ears() is not None


def hear(wav_bytes, max_sec=30.0):
    """소리 -> 글자 (다듬기 전 원문)."""
    e = _ears()
    if e is None:
        raise RuntimeError("받아쓰기 모델(ears)이 없습니다")
    return e.from_wav_bytes(wav_bytes, max_sec=max_sec)


def polish(text):
    """지저분한 글자 -> 깨끗한 글자.

    다듬기 모델이 없으면 원문을 그대로 돌려준다.
    """
    f = _fixer()
    return f.fix(text) if f else text


def listen(wav_bytes, max_sec=30.0):
    """소리 -> 깨끗한 글자 (받아쓰기 + 다듬기).

    돌려주는 것: (깨끗한 글자, 받아쓴 원문)
    원문도 같이 주는 이유는, 무엇이 어떻게 고쳐졌는지 로그로
    남겨야 다음에 무엇을 고칠지 알 수 있기 때문이다.
    """
    raw = hear(wav_bytes, max_sec=max_sec)
    return polish(raw), raw
