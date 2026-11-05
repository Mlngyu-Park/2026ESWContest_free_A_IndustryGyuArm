#!/usr/bin/env python3
"""FIXER 학습 데이터 — 27MB STT 가 뱉은 지저분한 말을 깨끗하게 고친다.

왜 필요한가
  음성 인식(Moonshine 27M)은 노트북에서 도는 대신 정확도가 낮다.
  실제로 이런 말이 나온다 (사용자 로그, 2026-09-15):

    Spark(정답) : 보라색 도킹 스테이션에 있는 초록색 펜을 잡아서
                  빨간색으로 이동한 다음에 주황색으로 다시 옮기는 거 하고 싶어
    27MB(오염)  : 보라색 도킹 스테이션에 있는 [카페 스테이션에 있는] 초록색 펜을
                  잡아서 빨간색으로 이동한 다음에 [주황 운동한 다음에] 주황색으로 ...

  이 오염된 말을 RACHEL 에 그대로 넣으면 블록이 망가진다.
  그래서 사이에 '고치는 모델' 을 둔다:

      27MB  ->  FIXER(이 모델)  ->  hints.py  ->  RACHEL  ->  블록
      지저분        깨끗하게         색->도킹      블록 배열

  AI 경계선 원칙은 그대로다. FIXER 도 '말을 말로' 고칠 뿐,
  좌표나 제어에는 손대지 않는다.

관찰된 오염 다섯 가지 (실측)
  1) 겹침 중복 — 창을 겹쳐 돌린 자리가 두 번 들어간다  (제일 흔하다)
  2) 낱말 오인 — 도킹->카페, 이동->운동 처럼 비슷한 소리로
  3) 반복 루프 — "색 색 색 색 ..." (디코더가 갇힌다)
  4) 잔말      — 어, 음, 그 …
  5) 끝 부호   — 마침표가 붙거나 빠진다

만드는 법
  python gen_data.py 60000 > fix_data.jsonl
"""
import json
import random
import sys

# ── 어휘 (RACHEL 과 같은 세계를 쓴다) ──────────────────────
COLORS = ["빨강", "주황", "노랑", "초록", "파랑", "보라"]
C_ADJ  = {"빨강": "빨간", "주황": "주황", "노랑": "노란",
          "초록": "초록", "파랑": "파란", "보라": "보라"}

DOCK_WORDS = ["도킹", "도킹 스테이션", "도킹스테이션", "스테이션", "자리"]
PEN_WORDS  = ["펜", "그거", "저거", "부품", "물체", "펜대"]

# ── 소리가 닮아 잘못 들리는 말 (실측 + 같은 계열 확장) ──────
#   27MB 가 실제로 "도킹->카페", "이동->운동" 으로 들었다.
#   같은 자리에서 흔들릴 만한 말을 같이 넣는다.
MISHEARD = {
    "도킹":   ["토킹", "카페", "도킹이", "도깅", "노킹", "도크"],
    "스테이션": ["스테이", "스테이션이", "스테이선", "스테이숀", "테이션"],
    "이동":   ["운동", "이동이", "이송", "이돈"],
    "펜":     ["팬", "편", "페", "펜이"],
    "잡아서": ["잡아써", "자바서", "잡아", "잡아서요"],
    "옮기는": ["옮기는", "옴기는", "올기는", "옮기능"],
    "빨간":   ["빨강", "빨간색", "발간", "빨깐"],
    "주황":   ["주항", "주왕", "주황색", "쭈황"],
    "노란":   ["노랑", "노란색", "노람", "노랸"],
    "초록":   ["초록색", "초륵", "쵸록", "초록이"],
    "파란":   ["파랑", "파란색", "파람", "파랸"],
    "보라":   ["보라색", "보람", "볼라", "보라가"],
}

FILLERS = ["어", "음", "그", "저", "뭐", "아"]


def RO(word=""):
    """'~로/~으로' — 앞말 받침에 맞춘다 ('도킹으로', '자리로')."""
    if word:
        ch = word.strip()[-1]
        if "가" <= ch <= "힣":
            has_batchim = (ord(ch) - 0xAC00) % 28 != 0
            # ㄹ 받침은 '로' 를 쓴다 ('자물쇠로')
            return "로" if not has_batchim or (ord(ch) - 0xAC00) % 28 == 8 else "으로"
    return random.choice(["로", "으로"])


def EUL(word):
    """'~을/~를' — 받침에 맞춘다 ('펜을', '그거를')."""
    ch = word.strip()[-1]
    if "가" <= ch <= "힣":
        return "을" if (ord(ch) - 0xAC00) % 28 else "를"
    return "을"


def dock_call(col):
    """색으로 도킹을 부른다 — 실제 사용자가 그렇게 부른다."""
    a = C_ADJ[col]
    ac = a if a.endswith("색") else a + "색"
    return random.choice([
        f"{a} {random.choice(DOCK_WORDS)}",
        f"{ac} {random.choice(DOCK_WORDS)}",
        f"{ac}",
    ])


def simple_sentence():
    """짧은 명령 — 실제로 이런 말이 제일 많다."""
    d = random.choice(COLORS)
    dc = dock_call(d)
    pen = random.choice(PEN_WORDS)
    return random.choice([
        f"{dc}{RO(dc)} 옮겨줘",
        f"{pen} 잡아서 {dc}{RO(dc)} 옮겨줘",
        f"{pen} 잡아",
        f"{dc}{RO(dc)} 가",
        f"{dc}에 놔줘",
        f"{pen}{EUL(pen)} {dc}{RO(dc)} 옮겨",
        f"그리퍼 열어",
        f"그리퍼 닫아",
        f"하강",
        f"상승",
    ])


def num_dock(n):
    """번호로 부르기 — 색 말고 번호를 쓰는 사람도 있다."""
    return random.choice([f"{n}번 도킹", f"{n}번 자리", f"{n}번",
                          f"도킹 {n}번"])


def numbered_sentence():
    """번호로 말하는 문장."""
    ns = random.sample(range(2, 15), random.randint(2, 3))
    pen = random.choice(PEN_WORDS)
    head = f"{num_dock(ns[0])}에 있는 {pen}{EUL(pen)} 잡아서"
    mids = [f"{num_dock(n)}{RO(num_dock(n))} 옮긴 다음에" for n in ns[1:-1]]
    last = f"{num_dock(ns[-1])}{RO(num_dock(ns[-1]))} 옮겨줘"
    return " ".join([head] + mids + [last])


def clean_sentence():
    """정답 문장 하나 (사람이 실제로 하는 말투)."""
    r = random.random()
    if r < 0.22:
        return simple_sentence()
    if r < 0.32:
        return numbered_sentence()
    cols = random.sample(COLORS, random.randint(2, 4))
    src, *dsts = cols
    pen = random.choice(PEN_WORDS)
    pcol = random.choice(COLORS)

    head = random.choice([
        f"{dock_call(src)}에 있는 {C_ADJ[pcol]}색 {pen}{EUL(pen)} 잡아서",
        f"{dock_call(src)}에 있는 {pen} 잡아서",
        f"{C_ADJ[pcol]}색 {pen}{EUL(pen)} 잡아서",
        f"{pen} 잡아서",
    ])
    mids = []
    for i, d in enumerate(dsts):
        dc = dock_call(d)
        if i == 0:
            mids.append(f"{dc}{RO(dc)} 옮긴 다음에")
        elif i == len(dsts) - 1:
            mids.append(random.choice([
                f"다시 {dc}{RO(dc)} 옮기",
                f"마지막으로 {dc}{RO(dc)} 옮기",
            ]))
        else:
            mids.append(random.choice([
                f"다시 {dc}{RO(dc)} 옮긴 다음에",
                f"그 다음에 {dc}{RO(dc)} 옮긴 다음에",
            ]))
    tail = random.choice(["는 거 하고 싶어", "고 싶어", "자", "면 돼",
                          "는 거야", "줘", "면 좋겠어"])
    body = " ".join(mids)
    pre = random.choice(["", "", "내가 하고 싶은 건 ", "일단 ", "이제 "])
    return f"{pre}{head} {body}{tail}".strip()


# ── 오염 만들기 ────────────────────────────────────────────

def add_overlap(s):
    """겹침 중복 — 창을 겹쳐 돌린 자리가 두 번 들어간다.

    ★ 이게 제일 흔하고 제일 해롭다.
      두 번째로 들어가는 조각은 '살짝 다르게' 들린 판이다
      (실측: "도킹 스테이션에 있는" -> "카페 스테이션에 있는").
    """
    w = s.split()
    if len(w) < 6:
        return s
    i = random.randint(2, len(w) - 4)
    k = random.randint(2, min(5, len(w) - i))
    dup = w[i:i + k]
    # 겹친 조각을 살짝 다르게 (같은 말을 다르게 들은 것처럼)
    dup = [mishear_word(x) if random.random() < 0.6 else x for x in dup]
    return " ".join(w[:i + k] + dup + w[i + k:])


def mishear_word(word):
    """낱말 하나를 비슷한 소리로 바꾼다."""
    for key, alts in MISHEARD.items():
        if key in word:
            return word.replace(key, random.choice(alts))
    return word


def add_mishear(s):
    w = s.split()
    n = max(1, len(w) // 8)
    for _ in range(n):
        i = random.randrange(len(w))
        w[i] = mishear_word(w[i])
    return " ".join(w)


def add_loop(s):
    """반복 루프 — 같은 말이 여러 번 (디코더가 갇힌 자국)."""
    w = s.split()
    if len(w) < 3:
        return s
    i = random.randrange(len(w))
    rep = random.randint(3, 8)
    return " ".join(w[:i] + [w[i]] * rep + w[i + 1:])


def add_filler(s):
    w = s.split()
    for _ in range(random.randint(1, 2)):
        i = random.randrange(len(w) + 1)
        w.insert(i, random.choice(FILLERS))
    return " ".join(w)


def add_punct(s):
    return s + random.choice([".", ".", "..", "?", ""])


def add_tail_slur(s):
    """어미가 늘어지거나 잘린다 — Gemma 가 보여준 실제 양상.

      열어 -> 열어어    가 -> 가아    잡아 -> 잡어
      옮겨줘 -> 옮겨주  (끝소리가 사라진다)
    """
    w = s.split()
    if not w:
        return s
    last = w[-1]
    r = random.random()
    if r < 0.35 and len(last) >= 2:
        w[-1] = last + last[-1]            # 늘어짐: 열어 -> 열어어
    elif r < 0.60 and len(last) >= 3:
        w[-1] = last[:-1]                  # 잘림:  옮겨줘 -> 옮겨
    elif r < 0.80:
        w[-1] = last.replace("아", "어").replace("어", "아", 1)
    return " ".join(w)


_JOSA_BAD = {"으로": ["이로", "로", "우로"], "로": ["이로", "르"],
             "를": ["을", "르"], "을": ["를", "르"],
             "에": ["이", "에이"], "에서": ["에소", "에서어"]}


def add_josa_slip(s):
    """조사가 틀린다 (도킹으로 -> 도킹이로). 아주 흔한 오류."""
    w = s.split()
    idx = [i for i, x in enumerate(w)
           if any(x.endswith(j) for j in _JOSA_BAD)]
    if not idx:
        return s
    i = random.choice(idx)
    for j, alts in _JOSA_BAD.items():
        if w[i].endswith(j):
            w[i] = w[i][:-len(j)] + random.choice(alts)
            break
    return " ".join(w)


# 문장 끝에 엉뚱한 말이 통째로 붙는다 — 모델이 소리를 다 쓰고도
# 멈추지 못해 아무 말이나 이어 붙이는 것이다 (환각).
#   실측: "...주황색으로 옮겨줘. 저는 처음 봤습니다. 처음 봤습니다. 처음 봤습니다."
_HALLUC = [
    "저는 처음 봤습니다", "감사합니다", "구독과 좋아요 부탁드립니다",
    "시청해 주셔서 감사합니다", "네 알겠습니다", "안녕하세요",
    "MBC 뉴스 이덕영입니다", "다음 영상에서 만나요", "좋은 하루 되세요",
    "그럼 이만", "수고하셨습니다", "아멘",
]


def add_halluc(s):
    """문장 뒤에 관계없는 말이 붙는다 (같은 말이 여러 번 붙기도 한다)."""
    tail = random.choice(_HALLUC)
    n = random.randint(1, 3)
    return s + ". " + ". ".join([tail] * n) + "."


def add_geu(s):
    """'그' 가 여기저기 낀다 — 실제 말버릇이 그대로 받아써진다."""
    w = s.split()
    for _ in range(random.randint(1, 3)):
        i = random.randrange(len(w) + 1)
        w.insert(i, random.choice(["그", "그", "어", "음", "저"]))
    return " ".join(w)


def dirty(s):
    """정답 문장을 27MB 가 뱉을 법한 꼴로 더럽힌다.

    ★ 오염의 '세기' 를 섞는다.
      전에는 거의 모든 문장에 중복을 넣어서(97%), 모델이
      "중복 지우기" 만 배우고 다른 오염에는 약했다.
      실제로는 살짝 틀린 말이 훨씬 흔하다.
    """
    out = s
    lv = random.random()

    # ★ 끝에 엉뚱한 말이 붙는 환각은 세기와 무관하게 일어난다.
    #   소리가 끝났는데도 모델이 안 멈춰서 생기는 일이다.
    halluc = random.random() < 0.18

    if lv < 0.35:
        # 약함 — 조사나 어미가 틀리는 정도 (실제로 제일 흔하다)
        if random.random() < 0.6:
            out = add_josa_slip(out)
        if random.random() < 0.5:
            out = add_tail_slur(out)
        if random.random() < 0.4:
            out = add_mishear(out)
        if random.random() < 0.4:
            out = add_punct(out)
    elif lv < 0.60:
        # 보통 — 잔말('그')이 끼고 낱말이 틀린다
        if random.random() < 0.7:
            out = add_geu(out)
        if random.random() < 0.6:
            out = add_mishear(out)
        if random.random() < 0.5:
            out = add_josa_slip(out)
        if random.random() < 0.3:
            out = add_tail_slur(out)
    elif lv < 0.90:
        # 강함 — 겹침 중복 (창을 나눠 돌린 자국)
        out = add_overlap(out)
        if random.random() < 0.5:
            out = add_mishear(out)
        if random.random() < 0.3:
            out = add_punct(out)
    else:
        # 아주 강함 — 반복 루프까지
        out = add_loop(out)
        if random.random() < 0.5:
            out = add_overlap(out)
        if random.random() < 0.4:
            out = add_filler(out)

    if halluc:
        out = add_halluc(out)
    return out


def main():
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 60000
    random.seed(20260915)
    seen = set()
    made = 0
    while made < n:
        ans = clean_sentence()

        # ★ 3분의 1은 일부러 '깨끗한 그대로' 로 둔다.
        #   전에는 이게 3.5% 뿐이었다. 그러니 모델이 "입력이 오면
        #   무조건 뭔가 고쳐야 한다" 고 배워서, 멀쩡한 말도 망쳤다
        #   (실측: "하강" -> "하하강", 짧은 명령을 통째로 지어냄).
        #   고칠 게 없으면 그대로 두는 것도 똑같이 중요한 일이다.
        if random.random() < 0.34:
            src = ans
        else:
            src = dirty(ans)
            if src == ans:            # 더럽히기가 아무 일도 안 했으면
                continue              # 위의 '그대로' 몫과 겹치니 버린다
        key = (src, ans)
        if key in seen:
            continue
        seen.add(key)
        print(json.dumps({"src": src, "dst": ans}, ensure_ascii=False))
        made += 1


if __name__ == "__main__":
    main()
