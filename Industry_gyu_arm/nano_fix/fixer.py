#!/usr/bin/env python3
"""FIXER — 27MB STT 가 뱉은 지저분한 말을 깨끗하게 고친다.

파이프라인에서의 자리
    Moonshine 27M  ->  FIXER(여기)  ->  hints.py  ->  RACHEL  ->  블록
      지저분한 글자      깨끗한 글자     색->도킹     블록 배열

  AI 경계선 원칙은 그대로다 — FIXER 도 '말을 말로' 만 고친다.
  좌표·경로·제어에는 손대지 않는다.

쓰는 법
    import fixer
    fixer.fix("보라색 도킹 스테이션에 있는 카페 스테이션에 있는 초록색 펜을...")
    -> "보라색 도킹 스테이션에 있는 초록색 펜을..."

  모델이 없으면 fix() 는 입력을 그대로 돌려준다 (안전한 기본값).
"""
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
CKPT = os.path.join(HERE, "fixer.pt")

_m = None          # 모델 (없으면 False)
_stoi = _itos = None
_torch = None


def ready():
    """모델 파일이 있나."""
    return os.path.exists(CKPT)


def load():
    """한 번만 올린다. 실패하면 False 로 표시하고 다시 시도하지 않는다."""
    global _m, _stoi, _itos, _torch
    if _m is not None:
        return _m is not False
    if not ready():
        _m = False
        return False
    try:
        import torch
        import sys
        sys.path.insert(0, HERE)
        from model import Rachel
        import tokenizer as tk

        ck = torch.load(CKPT, map_location="cpu", weights_only=False)
        itos = ck["vocab"]
        m = Rachel(len(itos), d=ck["dim"], layers=ck["layers"],
                   heads=ck["heads"], ctx=ck["ctx"])
        m.load_state_dict(ck["model"])
        m.eval()
        _torch = torch
        _m = m
        _itos = itos
        _stoi = {c: i for i, c in enumerate(itos)}
        return True
    except Exception as e:
        print(f"[FIXER] 모델을 못 올렸습니다 (그냥 지나갑니다): {e}")
        _m = False
        return False


_COLOR_FORMS = {
    "빨강": ("빨간", "빨강", "붉은"), "주황": ("주황", "오렌지"),
    "노랑": ("노란", "노랑"),        "초록": ("초록", "녹색"),
    "파랑": ("파란", "파랑"),        "보라": ("보라", "자주"),
}


def _colors(s):
    """문장에 나온 색의 '종류' (같은 색의 여러 표기를 하나로).

    ★ 몇 번 나왔는지는 세지 않는다.
      겹침 중복이 있으면 같은 색이 두 번 나오는데, 그 중복을 지우는 게
      바로 FIXER 가 할 일이다. 개수로 검사하면 옳은 교정을 막는다
      (실측: "주황 운동한 / 주황색으로" 에서 가짜를 지웠더니 걸렸다).
    """
    return {std for std, forms in _COLOR_FORMS.items()
            if any(f in s for f in forms)}


_ACT_FORMS = {
    "잡기": ("잡아", "집어", "잡을", "쥐"),
    "놓기": ("놓", "놔", "넣", "꽂"),
    "이동": ("옮기", "옮긴", "옮겨", "이동", "가져", "보내"),
    "열기": ("열어", "열기", "벌려"),
    "닫기": ("닫아", "닫기", "잠가"),
    "하강": ("하강", "내려"),
    "상승": ("상승", "올려", "들어올"),
}


def _acts(s):
    """문장에 담긴 동작의 종류."""
    return {std for std, forms in _ACT_FORMS.items()
            if any(f in s for f in forms)}


def _color_order(s):
    """색이 나온 '순서'. 순서가 곧 작업 순서라 절대 바뀌면 안 된다."""
    hits = []
    for std, forms in _COLOR_FORMS.items():
        for f in forms:
            i = s.find(f)
            while i >= 0:
                hits.append((i, std))
                i = s.find(f, i + 1)
    hits.sort()
    # 연달아 같은 색이면 하나로 (빨간색 -> '빨간' 과 '빨강' 이 겹칠 수 있다)
    out = []
    for _, c in hits:
        if not out or out[-1] != c:
            out.append(c)
    return out


def _color_hint(s):
    """색 비슷한 소리가 있나 — 오타라도 첫 글자는 대개 남는다.

    "빨깐" 은 '빨' 이 있어서 통과, "주임" 은 '주' 가 있지만
    뒤에 색 꼴이 없으므로 걸러진다.
    """
    seeds = ("빨", "붉", "주황", "오렌", "노", "초", "녹", "파", "푸",
             "보라", "자주", "블루", "레드", "그린", "옐로", "퍼플")
    return any(x in s for x in seeds)


def _act_hint(s):
    """동작 비슷한 소리가 있나 (오타라도 앞글자는 남는다)."""
    seeds = ("잡", "집", "쥐", "놓", "놔", "넣", "꽂", "옮", "이동", "가져",
             "보내", "열", "닫", "잠", "하강", "내려", "상승", "올려", "들")
    return any(x in s for x in seeds)


def _keeps_order(want, got):
    """want 의 순서가 got 안에 그대로 들어 있나 (부분 수열)."""
    it = iter(got)
    return all(any(g == w for g in it) for w in want)


def _has_loop(s, need=3):
    """같은 글자/어절이 되풀이되나 (모델이 갇힌 자국)."""
    for size in (1, 2, 3):
        for i in range(len(s) - size * need + 1):
            unit = s[i:i + size]
            if unit.strip() and all(
                    s[i + k * size:i + (k + 1) * size] == unit
                    for k in range(need)):
                return True
    w = s.split()
    for i in range(len(w) - need + 1):
        if len(set(w[i:i + need])) == 1:
            return True
    return False


def _lean_len(s):
    """되풀이를 뺀 길이 — 원문이 얼마나 부풀어 있는지 잰다.

    같은 어절이 연달아 나오면("색 색 색") 한 번만 센다.
    이 길이가 원문보다 많이 짧으면, 교정 결과도 그만큼 짧아지는 게
    정상이라는 뜻이다.
    """
    w = s.split()
    if not w:
        return len(s)
    out = [w[0]]
    for x in w[1:]:
        if x != out[-1]:
            out.append(x)
    return len(" ".join(out))


# 명령과 무관한 '끝에 붙는 말' — 소리가 끝났는데 모델이 안 멈춰서 생긴다.
#   실측: "...주황색으로 옮겨줘. 저는 처음 봤습니다. 처음 봤습니다. ..."
_TAIL_JUNK = (
    "처음 봤", "감사합니다", "구독", "좋아요", "시청", "안녕하세",
    "뉴스", "다음 영상", "좋은 하루", "그럼 이만", "수고하셨",
    "알겠습니다", "아멘", "함께해", "채널",
)


def _strip_tail_junk(s):
    """끝에 붙은 엉뚱한 말을 잘라낸다.

    ★ 모델에 맡기지 않고 규칙으로 먼저 지운다.
      이건 '소리에 없던 말' 이라 지워도 잃을 게 없고, 규칙으로
      확실히 잡히는 유형이다. 모델이 배울 때까지 기다릴 이유가 없다.

    마침표로 끊고, 뒤에서부터 군더더기 조각을 떼어낸다.
    명령이 들어 있는 조각을 만나면 멈춘다.
    """
    parts = [p.strip() for p in s.replace("!", ".").replace("?", ".").split(".")]
    parts = [p for p in parts if p]
    if len(parts) < 2:
        return s
    keep = list(parts)
    while len(keep) > 1:
        last = keep[-1]
        # 명령 낌새(색·동작)가 있으면 건드리지 않는다
        if _colors(last) or _acts(last):
            break
        if any(j in last for j in _TAIL_JUNK):
            keep.pop()
            continue
        break
    if len(keep) == len(parts):
        return s
    out = ". ".join(keep)
    return out + ("." if s.rstrip().endswith(".") else "")


def fix(text, max_new=None):
    """지저분한 말 -> 깨끗한 말.

    모델이 없거나 결과가 미심쩍으면 원문을 그대로 돌려준다.
    ★ 고치려다 말을 잃는 것보다 안 고치는 게 낫다.
    """
    text = (text or "").strip()
    if not text:
        return text

    # ① 끝에 붙은 엉뚱한 말부터 규칙으로 잘라낸다 (모델 없이도 된다)
    text = _strip_tail_junk(text)

    if not load():
        return text

    import tokenizer as tk
    torch = _torch
    prompt = f"{text}\n>"
    ids = tk.encode(_stoi, prompt)
    ctx = _m.ctx
    if len(ids) >= ctx - 8:               # 너무 길면 손대지 않는다
        return text
    if max_new is None:
        max_new = min(ctx - len(ids) - 1, len(text) + 32)

    x = torch.tensor([ids], dtype=torch.long)
    with torch.no_grad():
        out = _m.generate(x, tk.EOS, max_new=max_new)
    got = tk.decode(_itos, out[0].tolist()[len(ids):]).strip()

    # ── 안전장치 ──────────────────────────────────────────
    #   고친 결과가 이상하면 원문을 쓴다.
    if not got:
        return text
    #   ★ 길이로 막지 않는다.
    #     지저분한 말을 고치면 많이 짧아지는 게 정상이다 (실측:
    #     "토킹 스테이선에 있는 팬이 잡아써 빨깐 자리로 옴기는 거"
    #     -> "펜 잡아서 빨간 자리로 옮겨줘" 는 0.52 배지만 정답이다).
    #     길이 대신 '뜻이 남았는지' 로 본다 — 아래 색·동작 검사.

    #   ★ 색은 하나도 잃으면 안 된다 — 뒤의 hints.py 가 색으로 도킹을
    #     찾기 때문에, 색이 빠지면 목적지가 통째로 사라진다.
    #     '빨간/빨강/빨간색' 은 같은 색이므로 묶어서 센다.
    if _colors(text) - _colors(got):
        return text

    #   ★ 동작도 잃으면 안 된다. 색은 '어디로' 고, 동작은 '무엇을' 이다.
    #     둘 중 하나만 빠져도 블록이 통째로 달라진다.
    if _acts(text) - _acts(got):
        return text

    #   ★ 색의 '순서' 가 바뀌면 안 된다.
    #     색이 다 있어도 순서가 바뀌면 전혀 다른 작업이 된다
    #     (실측: 빨강->노랑->파랑 이 빨강->파랑->노랑 으로 뒤집혔다).
    #     종류 검사만으로는 이걸 못 잡는다.
    #     단, 색이 '늘어나는' 것은 괜찮다 — 오타('빨깐')를 제대로
    #     고쳐서 색으로 인식된 경우다. 원문의 순서가 결과 안에
    #     그 순서대로 들어 있으면 통과시킨다.
    if not _keeps_order(_color_order(text), _color_order(got)):
        return text

    #   ★ 없던 색을 지어내면 안 된다.
    #     오타를 고치는 건 괜찮지만(빨깐->빨간), 색이 아예 없던 말에서
    #     색이 튀어나오면 엉뚱한 도킹으로 간다
    #     (실측: "주임" -> "주황 자" — 말하지도 않은 주황이 생겼다).
    #     원문에 색 비슷한 낌새조차 없으면 거부한다.
    new_colors = _colors(got) - _colors(text)
    if new_colors and not _color_hint(text):
        return text

    #   ★ 없던 동작도 지어내면 안 된다 (같은 이유).
    #     실측: "아 그 뭐지" -> "하강". 명령이 아닌 말에서 동작이 생겼다.
    if (_acts(got) - _acts(text)) and not _act_hint(text):
        return text

    #   ★ 같은 말이 되풀이되면 거부 — 모델이 갇힌 것이다
    #     (실측: "상승" -> "상상상상상승").
    if _has_loop(got):
        return text
    #   너무 길어졌으면(헛말을 지어냈으면) 안 쓴다
    if len(got) > len(text) * 1.35 + 10:
        return text
    return got


def main():
    import sys
    if len(sys.argv) > 1:
        src = " ".join(sys.argv[1:])
        print(f"입력: {src}")
        print(f"출력: {fix(src)}")
        return
    # 시험용 — 실제로 관찰된 오염
    cases = [
        "보라색 도킹 스테이션에 있는 카페 스테이션에 있는 초록색 펜을 잡아서 "
        "빨간색으로 이동한 다음에 주황 운동한 다음에 주황색으로 다시 옮기는 거 하고 싶어.",
        "빨간 도킹으로 옮겨줘",
        "어 그 초록색 색 색 색 펜을 음 잡아서 빨간 도킹이로 옮겨줘.",
    ]
    if not load():
        print("모델(fixer.pt)이 아직 없습니다.")
        return
    for c in cases:
        print(f"  입력: {c}")
        print(f"  출력: {fix(c)}")
        print()


if __name__ == "__main__":
    main()
