"""말 정리기 글자 사전 — 어떤 한글이 들어와도 받는다.

RACHEL 은 학습 문장에 나온 179글자만 알아서, 실제 말 106문장 중 87문장에
모르는 글자('싶', '건', '한', '를' …)가 있었다. 정리기는 입력이 지저분한 게 일이므로
글자 사전을 처음부터 넓게 잡는다:
  ① 자주 쓰는 한글 2,350자 (KS X 1001)  ② 영문·숫자·기호  ③ 낱자모(ㄱ~ㅣ)
사전에 없는 한글 음절은 낱자모로 풀어서 넣는다 (모르는 글자로 뭉개지 않게).
"""
import json

PAD, UNK, EOS = 0, 1, 2
SPECIALS = ["<pad>", "<unk>", "<eos>"]

def _ksx1001():
    out = []
    for hi in range(0xB0, 0xC9):
        for lo in range(0xA1, 0xFF):
            try: out.append(bytes([hi, lo]).decode("euc-kr"))
            except UnicodeDecodeError: pass
    return out

CHO = "ㄱㄲㄴㄷㄸㄹㅁㅂㅃㅅㅆㅇㅈㅉㅊㅋㅌㅍㅎ"
JUNG = "ㅏㅐㅑㅒㅓㅔㅕㅖㅗㅘㅙㅚㅛㅜㅝㅞㅟㅠㅡㅢㅣ"
JONG = ["", "ㄱ", "ㄲ", "ㄳ", "ㄴ", "ㄵ", "ㄶ", "ㄷ", "ㄹ", "ㄺ", "ㄻ", "ㄼ", "ㄽ", "ㄾ", "ㄿ", "ㅀ",
        "ㅁ", "ㅂ", "ㅄ", "ㅅ", "ㅆ", "ㅇ", "ㅈ", "ㅊ", "ㅋ", "ㅌ", "ㅍ", "ㅎ"]
JAMO = sorted(set(CHO + JUNG + "".join(JONG)))

def base_vocab(extra_texts=()):
    chars = set(_ksx1001()) | set(chr(c) for c in range(32, 127)) | set(JAMO) | {"\n", "→", "~", "…"}
    for t in extra_texts: chars |= set(t)
    return SPECIALS + sorted(chars)

def build(texts, path="vocab.json"):
    vocab = base_vocab(texts)
    json.dump(vocab, open(path, "w", encoding="utf-8"), ensure_ascii=False)
    return {c: i for i, c in enumerate(vocab)}

def load(path="vocab.json"):
    vocab = json.load(open(path, encoding="utf-8"))
    return {c: i for i, c in enumerate(vocab)}, vocab

def _decompose(ch):
    o = ord(ch) - 0xAC00
    if not (0 <= o < 11172): return None
    return CHO[o // 588] + JUNG[(o % 588) // 28] + JONG[o % 28]

def encode(stoi, t):
    ids = []
    for ch in t:
        i = stoi.get(ch)
        if i is not None: ids.append(i); continue
        dj = _decompose(ch)                      # 사전에 없는 음절 → 낱자모
        if dj: ids.extend(stoi.get(j, UNK) for j in dj)
        else: ids.append(UNK)
    return ids

def decode(itos, ids):
    return "".join(itos[i] for i in ids if i >= len(SPECIALS))
