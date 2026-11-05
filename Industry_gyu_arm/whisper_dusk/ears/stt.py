#!/usr/bin/env python3
"""Moonshine 한국어 STT — Spark 없이 이 노트북에서 도는 음성 인식.

왜 이걸 쓰나
  지금은 음성 인식을 Spark(DGX)의 whisper-server 에 맡긴다.
  대회장에서 네트워크가 끊기면 말로 하는 조작이 통째로 죽는다.
  Moonshine 한국어(27M)는 노트북 CPU 에서 돌 만큼 작다.

왜 완벽하지 않아도 되나
  뒤에 두 단계가 더 있다:
    STT      "빠간 토킹으로 옮겨줘"   ← 대충 받아써도 된다
    hints.py  빨간 -> d10             ← 규칙이 색을 정확히 잡는다
    RACHEL    블록 배열               ← 오염된 말을 견디게 학습돼 있다
  RACHEL 은 "도킹->토킹" 같은 오염을 25% 섞어 학습했다 (gen_data.py).

파일
  onnx/encoder_model.onnx              29.5MB (fp32)
  onnx/decoder_model_merged_int8.onnx  19.4MB (int8)

  인코더는 int8 판이 있지만 이 onnxruntime 에서 안 돈다
  (ConvInteger 미구현). 그래서 fp32 를 쓴다.

쓰는 법
  python stt.py 소리.wav
  python stt.py --bench          (합성음으로 속도만 측정)
"""
import os, sys, time
import numpy as np
import onnxruntime as ort
from tokenizers import Tokenizer

HERE = os.path.dirname(os.path.abspath(__file__))
SR   = 16000        # Moonshine 은 16kHz 파형을 그대로 받는다 (멜 변환 불필요)

# 이 모델이 한 번에 낼 수 있는 최대 토큰 수.
#   config.json 의 max_position_embeddings 와 같다 — 그 너머는
#   위치 정보가 없어서 더 달라고 해도 소용이 없다.
MODEL_MAX = 194

_enc = _dec = _tok = None


def load():
    """모델을 한 번만 올린다. 상주 모드에서 재사용."""
    global _enc, _dec, _tok
    if _enc is not None:
        return
    so = ort.SessionOptions()
    so.intra_op_num_threads = 4
    so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    _enc = ort.InferenceSession(f"{HERE}/onnx/encoder_model.onnx",
                                so, providers=["CPUExecutionProvider"])
    _dec = ort.InferenceSession(f"{HERE}/onnx/decoder_model_merged_int8.onnx",
                                so, providers=["CPUExecutionProvider"])
    _tok = Tokenizer.from_file(f"{HERE}/tokenizer.json")


def _empty_past(n_layers, n_heads, head_dim):
    """첫 단계용 빈 KV 캐시.

    merged 디코더는 past 를 항상 요구한다. ★ 첫 단계에는 인코더 쪽도
    길이 0 으로 줘야 한다 — 실제 길이로 주면 2단계에서 모양이 안 맞아
    "cannot broadcast on dim 0" 으로 죽는다 (실측).
    인코더 캐시는 1단계 출력(present)이 채워서 돌려준다."""
    past = {}
    for l in range(n_layers):
        for side in ("decoder", "encoder"):
            past[f"past_key_values.{l}.{side}.key"]   = np.zeros((1, n_heads, 0, head_dim), np.float32)
            past[f"past_key_values.{l}.{side}.value"] = np.zeros((1, n_heads, 0, head_dim), np.float32)
    return past


def transcribe(wav, max_new=None):
    """16kHz float32 파형 -> 한국어 글자.

    길이 제한은 두지 않는다. 말이 끝나는 시점은 모델이 정한다(eos).

    ★ 예전에는 64 로 못박아 놨다. 한국어는 글자당 약 1.8토큰이라
      (실측: 50글자 = 90토큰) 30글자만 넘으면 말이 중간에서 끊겼고,
      글자 중간에서 끊겨 "검은 도<깨짐>" 같은 깨진 글자가 남았다.
    """
    load()
    if wav.ndim > 1:
        wav = wav.mean(axis=1)            # 스테레오 -> 모노
    wav = wav.astype(np.float32)

    if max_new is None:
        # ★ 글자 수로 자르지 않는다.
        #   멈추는 건 모델이 정한다 — 말이 끝나면 eos 를 내고 거기서 선다.
        #   여기 숫자는 '말이 끝났는데도 eos 가 안 나올 때' 를 대비한
        #   안전장치일 뿐이다. 그래서 모델이 갈 수 있는 끝까지 준다.
        #
        #   MODEL_MAX 는 모델 스스로의 한계다 (config.json 의
        #   max_position_embeddings = 194). 그 너머는 위치 정보가 없어서
        #   더 줘도 의미가 없다.
        max_new = MODEL_MAX

    enc_out = _enc.run(None, {"input_values": wav[None, :]})[0]
    enc_len = enc_out.shape[1]

    L, H, HD = 6, 8, 36                   # config.json 기준
    past = _empty_past(L, H, HD)
    has_past = False                      # 첫 단계는 캐시가 비어 있다
    names = {i.name for i in _dec.get_inputs()}
    outs = [o.name for o in _dec.get_outputs()]

    ids = [1]                             # decoder_start_token_id
    out_ids = []
    for step in range(max_new):
        feed = {
            "input_ids": np.array([[ids[-1]]], np.int64),
            "encoder_hidden_states": enc_out,
            **past,
        }
        # merged 디코더는 'use_cache_branch' 로 분기한다.
        #   1단계 False (캐시 없음) -> 2단계부터 True (present 를 물려받음)
        if "use_cache_branch" in names:
            feed["use_cache_branch"] = np.array([has_past], bool)

        res = _dec.run(None, feed)
        logits = res[0]
        nxt = int(logits[0, -1].argmax())
        if nxt == 2:                      # eos
            break
        out_ids.append(nxt)
        ids.append(nxt)

        # ── 같은 말 반복에서 빠져나오기 ──────────────────────
        #   argmax 는 늘 '제일 그럴듯한 것 하나' 만 고른다. 그래서
        #   모델이 한번 "색" 에 빠지면 같은 상태가 되풀이되어 끝까지
        #   "색 색 색 ..." 만 뽑는다 (실측: 46번 반복).
        #   스스로 빠져나올 길이 없으므로 여기서 끊는다.
        if _looping(out_ids):
            out_ids = _trim_loop(out_ids)
            break

        # 다음 단계용 캐시 갱신 (present.* -> past_key_values.*)
        #
        # ★ 인코더 캐시는 1단계 것만 쓰고 그 뒤로는 건드리지 않는다.
        #   소리는 한 번 인코딩하면 안 바뀌므로 cross-attention 캐시도
        #   고정이다. 매 단계 덮어쓰면 모양이 어긋나
        #   "cannot broadcast on dim 0" 으로 죽는다 (실측).
        for name, val in zip(outs[1:], res[1:]):
            key = "past_key_values." + name[len("present."):]
            if not has_past or ".decoder." in key:
                past[key] = val
        has_past = True

    txt = _tok.decode(out_ids).strip()

    # 모델 한계(194)까지 꽉 차서 멈췄으면 글자 중간에서 끊겼을 수 있다.
    # 깨진 조각이 뒤에 남으면 그 뒤 단계(hints/RACHEL)가 엉뚱하게 읽으므로
    # 마지막 어절 하나를 떼어낸다. (아주 긴 말에서만 일어난다)
    if len(out_ids) >= max_new and " " in txt:
        txt = txt.rsplit(" ", 1)[0]
    return txt


def _looping(ids, need=4):
    """같은 말이 되풀이되고 있나.

    두 가지를 본다:
      ① 같은 토큰이 연달아 need 번
      ② 같은 묶음이 연달아 need 번

    ★ 묶음을 10 까지 본다. 한글 한 글자가 UTF-8 바이트로 쪼개져
      3 토큰이 되기 때문이다 — "주임 " 하나가 5 토큰이었다 (실측).
      4 까지만 보던 때는 이걸 통째로 놓쳤다.
    """
    n = len(ids)
    if n < need:
        return False
    # ① 한 토큰이 계속
    last = ids[-1]
    if all(t == last for t in ids[-need:]):
        return True
    # ② 짧은 묶음이 계속
    for size in range(2, 11):
        span = size * need
        if n < span:
            continue
        tail = ids[n - span:]                 # 끝에서 span 개
        unit = tail[:size]
        if all(tail[i * size:(i + 1) * size] == unit for i in range(need)):
            return True
    return False


def _trim_loop(ids):
    """반복이 시작된 자리까지 되돌린다 — 되풀이된 부분은 버린다."""
    for size in range(1, 11):
        unit = ids[-size:]
        k = len(ids)
        while k - size >= 0 and ids[k - size:k] == unit:
            k -= size
        if k < len(ids) - size:           # 실제로 되풀이가 있었다
            return ids[:k + size]         # 한 번만 남긴다
    return ids


def _tone(sec=2.0):
    """벤치용 합성음 (말은 아니지만 계산량은 같다)."""
    t = np.arange(int(SR * sec), dtype=np.float32) / SR
    return (0.1 * np.sin(2 * np.pi * 220 * t)).astype(np.float32)


def main():
    args = sys.argv[1:]
    if not args or args[0] == "--bench":
        sec = float(args[1]) if len(args) > 1 else 2.0
        print(f"  합성음 {sec}초로 속도만 측정 (내용은 무의미)")
        t0 = time.time(); load(); print(f"  모델 로딩 {(time.time()-t0)*1000:.0f}ms")
        w = _tone(sec)
        for i in range(3):
            t0 = time.time()
            txt = transcribe(w)
            el = (time.time() - t0) * 1000
            print(f"  {i+1}회: {el:>6.0f}ms  (실시간의 {el/1000/sec:.2f}배)  \"{txt[:30]}\"")
        return

    import soundfile as sf
    wav, sr = sf.read(args[0], dtype="float32")
    if sr != SR:
        print(f"  ! {sr}Hz 입니다. 16kHz 로 맞춰야 정확합니다.")
    t0 = time.time()
    txt = transcribe(wav)
    print(f"  {(time.time()-t0)*1000:.0f}ms")
    print(f"  \"{txt}\"")


if __name__ == "__main__":
    main()


# ── 앱이 쓰는 입구 ──────────────────────────────────────────
#   arm_studio_pro.py 의 /api/stt 가 Spark 실패 시 이걸 부른다.

def ready():
    """모델 파일이 다 있나. 없으면 앱은 Spark 만 쓴다."""
    return (os.path.exists(f"{HERE}/onnx/encoder_model.onnx") and
            os.path.exists(f"{HERE}/onnx/decoder_model_merged_int8.onnx") and
            os.path.exists(f"{HERE}/tokenizer.json"))


def _windows(wav, win_sec=4.0, hop_sec=3.0):
    """긴 소리를 '겹치게' 나눈다 (sliding window).

    왜 겹치게 하나
      이 모델은 한 번에 194 토큰까지만 본다. 인코더는 1초에 약 41 토큰을
      만드니(실측) 4.7 초가 한계다. 그래서 긴 말은 나눠 넣어야 한다.

      그런데 딱 잘라 이어붙이면 경계에서 말이 깨진다. 숨 쉬는 자리를
      찾아 자르는 방법도 써 봤지만, 쉬지 않고 말하면 자를 데가 없다.

      -> 다음 조각을 앞 조각이 끝나기 **전에** 시작한다.
         그러면 두 조각이 같은 구간을 겹쳐서 받아쓰고, 그 겹친 글자가
         이어붙일 기준점이 된다. 경계에 빈 자리가 생기지 않는다.

           |------ 1번 (0~4초) ------|
                      |------ 2번 (3~7초) ------|
                      ^^^^^^ 겹침 1초 — 여기 글자로 맞춘다

    win_sec 4.0  한 번에 넣는 길이 (194 토큰 안쪽)
    hop_sec 3.0  다음 조각이 시작하는 간격 -> 겹침 1.0 초
    """
    n = len(wav)
    win = int(SR * win_sec)
    hop = int(SR * hop_sec)
    if n <= win:
        return [(0, n)]
    out = []
    a = 0
    while a < n:
        b = min(a + win, n)
        out.append((a, b))
        if b >= n:
            break
        a += hop
    return out


def _sim(a, b):
    """두 글자가 얼마나 닮았나 (0~1). 한 글자씩 맞춰 센다."""
    if not a or not b:
        return 0.0
    from difflib import SequenceMatcher
    return SequenceMatcher(None, a, b).ratio()


def _merge(prev, cur, min_sim=0.80):
    """겹쳐 받아쓴 두 글자를 이어붙인다.

    앞 글자의 꼬리와 뒤 글자의 머리가 '같은 구간' 을 받아쓴 것이므로,
    그만큼을 찾아 한 번만 남긴다.

    ★ 딱 맞는 글자만 찾으면 안 된다.
      겹친 구간이라도 두 창이 조금씩 다르게 받아쓴다 (실측):
        앞창: "...보라색 도킹 스테이션에 있는"
        뒷창: "카페 스테이션에 있는 초록색 펜을..."
                ^^ 도킹 -> 카페 로 다르게 들었다
      그래서 '닮은 정도' 로 맞춘다.

    ★ 다만 확실할 때만 지운다 (min_sim 0.80).
      글자만 세면 "첫 번째 문장" 과 "두 번째 문장" 이 0.857 로 나와,
      완전히 다른 말을 겹침으로 오해할 수 있다 (실측).
      애매하면 그냥 둔다 — 남은 중복은 뒤의 FIXER 가 지운다.
      잘못 지운 말은 아무도 되살릴 수 없지만, 중복은 지울 수 있다.
    """
    if not prev:
        return cur
    if not cur:
        return prev

    pw = prev.split()
    cw = cur.split()
    if not pw or not cw:
        return (prev + " " + cur).strip()

    # 앞의 끝 k 어절과 뒤의 앞 k 어절이 가장 닮은 k 를 찾는다.
    # k 가 클수록(많이 겹칠수록) 좋으니 큰 것부터 본다.
    best_k, best_s = 0, 0.0
    for k in range(min(len(pw), len(cw), 14), 1, -1):   # 2어절 이상만
        s1 = " ".join(pw[-k:])
        s2 = " ".join(cw[:k])
        if len(s1) < 4 or len(s2) < 4:      # 너무 짧으면 우연일 수 있다
            continue
        sim = _sim(s1, s2)
        score = sim + min(k, 6) * 0.01      # 길게 겹치는 쪽에 가산점
        if sim >= min_sim and score > best_s:
            best_k, best_s = k, score

    # ★ 앞창을 통째로 먹어버리는 합침은 거부한다.
    #   진짜 겹침이면 앞창의 '꼬리' 만 겹친다. 앞창 전체가 겹친다는 건
    #   두 창이 같은 자리를 봤다는 뜻이 아니라, 그냥 비슷한 말이라는 뜻이다
    #   (실측: "첫 번째 문장" vs "두 번째 문장" 이 0.857 로 나왔다).
    #   이걸 합치면 앞말이 통째로 사라진다 — 제일 나쁜 실패다.
    if best_k and best_k >= len(pw):
        best_k = 0

    if best_k:
        # ★ 겹친 부분은 '앞창' 것을 남긴다.
        #   뒤창 것을 쓰면 뒤창이 잘못 들은 말이 채택된다
        #   (실측: "도킹" 을 "카페" 로 들은 쪽이 남았다).
        #   앞창은 그 구간을 뒤쪽 여유까지 두고 들었으므로 대체로 낫다.
        return " ".join(pw + cw[best_k:])

    return prev + " " + cur


def transcribe_long(wav):
    """길이에 상관없이 받아쓴다 — 겹치게 나눠 돌리고 이어붙인다."""
    parts = _windows(wav)
    if len(parts) == 1:
        return transcribe(wav)
    txt = ""
    for a, b in parts:
        t = transcribe(wav[a:b]).strip()
        if t:
            txt = _merge(txt, t)
    return txt


def from_wav_bytes(data, max_sec=30.0):
    """브라우저가 보낸 WAV 바이트 -> 한국어 글자.

    16kHz 가 아니면 간단히 재표본화한다. 정확도가 조금 떨어지지만
    뒤에서 hints.py 와 RACHEL 이 교정하므로 실용에는 지장이 없다.
    (라이브러리를 더 깔지 않으려고 선형 보간을 쓴다)
    """
    import io
    import soundfile as sf

    wav, sr = sf.read(io.BytesIO(data), dtype="float32")
    if wav.ndim > 1:
        wav = wav.mean(axis=1)

    if sr != SR:
        n = int(len(wav) * SR / sr)
        wav = np.interp(np.linspace(0, len(wav) - 1, n),
                        np.arange(len(wav)), wav).astype(np.float32)

    # 너무 길면 앞부분만 — 명령은 짧다
    if len(wav) > int(SR * max_sec):
        wav = wav[:int(SR * max_sec)]

    # 길면 토막내서 이어붙인다 (짧으면 그냥 한 번에)
    return transcribe_long(wav)
