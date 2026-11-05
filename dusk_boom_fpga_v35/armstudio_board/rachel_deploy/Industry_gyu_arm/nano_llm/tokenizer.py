"""글자 단위 토크나이저 — 직접 제작.

한국어 명령 어휘가 좁아(색 6개·동작 몇 개) 글자 단위로 충분하다.
서브워드 학습이 필요 없어 재현이 단순하고, 미등록 글자는 <unk> 처리.
"""
import json

PAD, UNK, EOS = 0, 1, 2
SPECIALS = ["<pad>", "<unk>", "<eos>"]

def build(texts, path="vocab.json"):
    chars = sorted({ch for t in texts for ch in t})
    vocab = SPECIALS + chars
    json.dump(vocab, open(path, "w", encoding="utf-8"), ensure_ascii=False)
    return {c: i for i, c in enumerate(vocab)}

def load(path="vocab.json"):
    vocab = json.load(open(path, encoding="utf-8"))
    return {c: i for i, c in enumerate(vocab)}, vocab

def encode(stoi, t):  return [stoi.get(ch, UNK) for ch in t]
def decode(itos, ids):
    return "".join(itos[i] for i in ids if i >= len(SPECIALS))
