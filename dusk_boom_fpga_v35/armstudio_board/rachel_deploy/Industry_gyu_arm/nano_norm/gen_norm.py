#!/usr/bin/env python3
"""말 정리기 학습 데이터 — 「말투 엔진」.

뜻 하나에서 '정해진 꼴(정답)' 과 '실제처럼 지저분한 말(입력)' 을 같이 만든다.
정답이 생성 순간 정해지므로 라벨이 틀릴 일이 없다.

  정답 꼴 (RACHEL v9 이 실제 말 87개를 이 꼴로 바꿨을 때 86/87 맞힘):
    보라색 도킹의 초록 펜을 빨간색으로 옮긴 다음 주황색으로 옮겨

  python gen_norm.py 1200000 > norm_data.jsonl     # {"src": 지저분한 말, "dst": 정해진 꼴}
  python gen_norm.py --show 40                      # 예시만 보기

설계: nano_llm/말정리기_설계.md
"""
import json, os, random, re, sys

HERE = os.path.dirname(os.path.abspath(__file__))
random.seed(int(os.environ.get("NORM_SEED", "29")))

# ── 기본 낱말 ────────────────────────────────────────────────
COLORS = ["빨강", "주황", "노랑", "초록", "파랑", "보라"]
CANON_C = {"빨강": "빨간색", "주황": "주황색", "노랑": "노란색", "초록": "초록색",
           "파랑": "파란색", "보라": "보라색"}
# 말로 부르는 색 (표준 → 여러 꼴). 실제 기록·흔한 말
SAY_C = {"빨강": ["빨간색", "빨간", "빨강", "빨간 색", "붉은색", "레드"],
         "주황": ["주황색", "주황", "오렌지색", "오렌지 색", "주황 색"],
         "노랑": ["노란색", "노란", "노랑", "노란 색"],
         "초록": ["초록색", "초록", "녹색", "초록 색", "초록 색깔"],
         "파랑": ["파란색", "파란", "파랑", "파란 색"],
         "보라": ["보라색", "보라", "보라 색", "자주색"]}
PEN_COLOR_SAY = {"초록": ["초록색", "초록", "초록 색깔", "녹색"], "빨강": ["빨간색", "빨간"],
                 "주황": ["주황색"], "노랑": ["노란색", "노란"], "파랑": ["파란색", "파란"],
                 "보라": ["보라색"]}
PEN_NOUN = ["펜", "펜", "펜", "피피에이피", "PPAP", "ppap", "펜대", "부품", "물체"]
PEN_PRON = ["그거", "그 펜", "저거", "그걸"]

# ── 받침과 조사 ──────────────────────────────────────────────
def _jong(w):
    w = w.rstrip()
    if not w: return 0
    c = w[-1]
    if "가" <= c <= "힣": return (ord(c) - 0xAC00) % 28
    if c.isdigit(): return {"0": 1, "1": 8, "2": 0, "3": 1, "4": 0, "5": 0, "6": 1, "7": 8, "8": 8, "9": 0}[c]
    return 1 if c.isalpha() else 0          # 영어는 받침 있다고 봄 (PPAP을)

def eul(w):  return w + ("을" if _jong(w) else "를")
def ro(w):
    j = _jong(w)
    return w + ("로" if j in (0, 8) else "으로")      # 받침 없음·ㄹ → 로
def eun(w):  return w + ("은" if _jong(w) else "는")

# ── 자리 부르기 ─────────────────────────────────────────────
def place_say(p):
    """p = ('c', 색) 또는 ('n', 번호). 말로 부르는 꼴 하나."""
    if p[0] == "n":
        return random.choice([f"{p[1]}번 도킹", f"{p[1]}번", f"{p[1]}번 자리", f"도킹 {p[1]}번",
                              f"{p[1]}번 도킹 스테이션"])
    a = random.choice(SAY_C[p[1]])
    tail = random.choices(["", " 도킹", " 도킹 스테이션", " 도킹스테이션", " 자리", " 데", " 쪽", " 스테이션"],
                          weights=[18, 18, 36, 10, 6, 4, 4, 4])[0]   # 실제 말은 '도킹 스테이션' 을 다 부른다
    if a.endswith(" 색") and tail: a = a.replace(" 색", "색")
    return a + tail

def place_canon(p):
    return f"{p[1]}번 도킹" if p[0] == "n" else CANON_C[p[1]]

def place_canon_at(p):
    """출발·잡기 자리: '보라색 도킹' / '3번 도킹' (시험에서 99% 나온 꼴)."""
    return f"{p[1]}번 도킹" if p[0] == "n" else CANON_C[p[1]] + " 도킹"

# ── 동사 활용표 ─────────────────────────────────────────────
#   adn: 관형(다음/후/뒤 앞) · go: ~고 · daga: ~다가 · seo: ~서 · myeon: ~면 · neun: ~는(거)
#   end: 끝맺음들 · jo: 목적지 조사 ('ro' = 으로/로, 'e' = 에)
VERBS = [
    dict(adn="옮긴", go="옮기고", daga="옮겼다가", seo="옮겨서", myeon="옮기면", neun="옮기는",
         end=["옮겨", "옮겨줘", "옮겨 줘", "옮겨봐", "옮겨 봐", "옮기자", "옮겨주세요", "옮겨줄래", "옮겨 놔"], jo="ro", w=30),
    dict(adn="이동한", go="이동하고", daga="이동했다가", seo="이동해서", myeon="이동하면", neun="이동하는",
         end=["이동해", "이동해줘", "이동해 줘", "이동해봐"], jo="ro", w=12),
    dict(adn="이동시킨", go="이동시키고", daga="이동시켰다가", seo="이동시켜서", myeon="이동시키면", neun="이동시키는",
         end=["이동시켜", "이동시켜줘", "이동시켜 줘", "이동시켜봐"], jo="ro", w=10),
    dict(adn="갖다 놓은", go="갖다 놓고", daga="갖다 놨다가", seo="갖다 놔서", myeon="갖다 놓으면", neun="갖다 놓는",
         end=["갖다 놔", "갖다 놔줘", "갖다 둬", "갖다 놓아줘"], jo="e", w=8),
    dict(adn="놓은", go="놓고", daga="놨다가", seo="놔서", myeon="놓으면", neun="놓는",
         end=["놔", "놔줘", "놓아줘", "내려놔"], jo="e", w=8),
    dict(adn="둔", go="두고", daga="뒀다가", seo="둬서", myeon="두면", neun="두는",
         end=["둬", "둬줘", "놔둬"], jo="e", w=6),
    dict(adn="꽂은", go="꽂고", daga="꽂았다가", seo="꽂아서", myeon="꽂으면", neun="꽂는",
         end=["꽂아", "꽂아줘", "꽂아 줘"], jo="e", w=8),
    dict(adn="넣은", go="넣고", daga="넣었다가", seo="넣어서", myeon="넣으면", neun="넣는",
         end=["넣어", "넣어줘"], jo="e", w=5),
    dict(adn="보낸", go="보내고", daga="보냈다가", seo="보내서", myeon="보내면", neun="보내는",
         end=["보내", "보내줘"], jo="ro", w=5),
    dict(adn="가져간", go="가져가고", daga="가져갔다가", seo="가져가서", myeon="가져가면", neun="가져가는",
         end=["가져가", "가져가줘"], jo="ro", w=3),
    dict(adn="이동된", go="이동되고", daga="이동됐다가", seo="이동돼서", myeon="이동되면", neun="이동되는",
         end=["이동되게 해줘"], jo="ro", w=1),
]
def pick_verb(): return random.choices(VERBS, weights=[v["w"] for v in VERBS])[0]

def dest(v, p):
    """목적지 + 조사 (동사에 맞게)."""
    s = place_say(p)
    if v["jo"] == "e":
        return s + random.choice(["에", "에", "에다", "에다가"])
    return random.choices([ro(s), s + "에", ro(s) + " 쪽" if False else ro(s)], weights=[80, 15, 5])[0]

# 연결어: 동사 v 의 앞 이송을 끝내는 꼴 → 다음 이송으로 넘어간다
CONNECT = [
    ("다음에", lambda v: v["adn"] + " 다음에", 9), ("다음", lambda v: v["adn"] + " 다음", 6),
    ("후에", lambda v: v["adn"] + " 후에", 7), ("후", lambda v: v["adn"] + " 후", 5),
    ("뒤에", lambda v: v["adn"] + " 뒤에", 5), ("뒤", lambda v: v["adn"] + " 뒤", 3),
    ("고 나서", lambda v: v["go"] + " 나서", 7), ("고", lambda v: v["go"], 7),
    ("고 다시", lambda v: v["go"] + " 다시", 5), ("다가", lambda v: v["daga"], 8),
    ("서", lambda v: v["seo"], 3), ("고 그 다음에", lambda v: v["go"] + " 그 다음에", 4),
    ("그리고 나서", lambda v: random.choice(v["end"]) + " 그리고 나서", 3),
    ("고 이어서", lambda v: v["go"] + " 이어서", 2), ("면", lambda v: v["myeon"], 2),
    ("다음 다시", lambda v: v["adn"] + " 다음에 다시", 4), ("후 다시", lambda v: v["adn"] + " 후에 다시", 3),
    ("고 그리고", lambda v: v["go"] + " 그리고", 2), ("놓고", lambda v: v["go"] + " 또", 2),
]
def pick_connect(): return random.choices(CONNECT, weights=[c[2] for c in CONNECT])[0]

LAST_MARK = ["", "", "", "", "마지막으로 ", "최종적으로 ", "끝으로 ", "다시 ", "한 번 더 ", "그 다음에 "]

# ── 펜 부르기 ───────────────────────────────────────────────
def pen_say(pcol, said_color):
    if said_color:
        return random.choice(PEN_COLOR_SAY[pcol]) + " " + random.choice(PEN_NOUN)
    return random.choice(PEN_NOUN + PEN_PRON + ["펜"])

def regrasp(prev_place, pen):
    """방금 놓은 자리를 다시 부르며 다시 잡기 — 단계를 늘리지 않는다."""
    return random.choice([
        "다시 그 펜을 잡아서 ", "그 펜 다시 잡아서 ", "다시 잡아서 ", "그걸 다시 ", "그거를 다시 잡아서 ",
        f"{place_say(prev_place)}에 있는 {eul(pen)} 잡아서 ", f"다시 {place_say(prev_place)}에 있는 {eul(pen)} 잡아서 ",
        f"{place_say(prev_place)}에 있는 {pen} 다시 잡아서 ", "다시 그 펜 잡아다가 "])

GRAB = ["잡아서", "잡아다가", "잡고", "집어서", "빼서", "잡은 다음에", "잡은 후에", "잡고 나서", "들어서"]

# ── 감싸기 ──────────────────────────────────────────────────
PREFIX = ["내가 하고 싶은 건", "내가 원하는 건", "내가 원하는 거는", "하고 싶은 공정은", "내가 하고 싶은 공정은",
          "해보고 싶은 건", "내가 이동하길 바라는 거는", "제가 원하는 건", "제가 하고 싶은 건",
          "오늘 하고 싶은 건", "하고 싶은 건 뭐냐면", "내가 원하는 움직임은", "원하는 작업은"]
NOM_END = ["{n} 거 하고 싶어", "{n} 걸 하고 싶어", "{n} 거를 하고 싶어", "{n} 거야", "{n} 그 공정을 하고 싶어",
           "{n} 그런 공정을 하고 싶어", "{n} 거 해보고 싶어", "{n} 걸 해보고 싶다", "{n} 그거를 하고 싶어",
           "{n} 거 하고 싶어요", "{n} 작업을 하고 싶어"]
MYEON_END = ["{m} 좋겠어", "{m} 좋겠어요"]
CALL = ["야 ", "자 ", "레이첼 ", "레이첼아 ", "음 ", "어 ", "그러니까 ", "저기 ", "응 야 ", "아 "]
TAIL = [" 알겠지", " 잘 해봐", " 어렵게 말했지만 알아서 해봐", " 부탁해", " 빨리", " 좀", " 해줘",
        " 복잡하게 말했지만 알아서 잘 해봐", " 니가 잘 이해해서 만들어 봐", " 이게 어려워"]
CHAT = ["안녕하세요", "감사합니다", "네 알겠습니다", "잠깐만요", "이거 맞나", "아 이거 왜 안 돼", "오늘 날씨 좋네",
        "밥 먹었어요", "저 그냥 들어가면 돼요", "다행히 열려 있어요", "저는 처음 봤습니다", "그거 어디서 봐요",
        "응 잠깐만", "아 진짜", "이거 녹음되는 거지", "언니 이거 봐봐", "몇 시야 지금", "수업 끝나고 가요",
        "아 미안해 절었어요", "다시 할게", "어디까지 했지", "이거 진짜 어렵네", "좋아요 좋아요",
        "오빠 영상 어디서 보는 거야", "그럼 맨날 왔다 갔다 해요"]
NONCMD = CHAT + ["내가 원하는 건", "내가 하고 싶은 건", "오늘 내가 하고 싶은 업적은 그렇게 대단한 건 아니고 다만",
                 "너 누구야", "너가 누군지 좀 알려줘볼래", "이름이 뭐야", "뭐 할 수 있어", "박정현입니다",
                 "안녕", "고마워", "음", "어", "잠깐", "아니야 됐어", "이거 진짜 일주일 했네", "테스트 테스트",
                 "마이크 테스트", "잘 들려", "하나 둘 셋", "그거 말고", "음 뭐였지", "야 이 빡대가리야",
                 "왜 안 되는 거야", "다시 해볼게", "커피 타줘", "불 켜", "노래 틀어", "오늘 날씨 어때", "청소해줘"]

# ── 음성 오염 (실제 기록에서 본 것) ─────────────────────────
def stt_noise(t):
    r = random.random
    if r() < 0.20: t = t.replace("도킹", random.choice(["토킹", "토킹", "쿠킹", "도킹"]), 1)
    if r() < 0.06: t = t.replace("도킹 스테이션", "카페 스테이션", 1)
    if r() < 0.10: t = t.replace("스테이션", "스테 이션", 1)
    if r() < 0.12: t = t.replace("펜", random.choice(["팬", "핀", "팬", "캔"]), 1)
    if r() < 0.04: t = t.replace("초록", "키록", 1)
    if r() < 0.04: t = t.replace("주황", "주화앙", 1)
    if r() < 0.12:                                   # 띄어쓰기 붕괴·추가
        sp = [m.start() for m in re.finditer(" ", t)]
        if sp: i = random.choice(sp); t = t[:i] + t[i + 1:]
    if r() < 0.06:
        i = random.randrange(1, max(2, len(t) - 1)); t = t[:i] + " " + t[i:]
    # 같은 말 되풀이 (명령처럼 들리는 말은 넣지 않는다 — '명령 아님' 문장과 어긋남)
    if r() < 0.04: t = t + " " + " ".join([random.choice(["색", "처음 봤습니다", "좋아요", "저는 좋아요"])] * random.randint(3, 6))
    if r() < 0.03:                                   # 반복 루프 (색 색 색 …)
        w = random.choice(["색", "그", "어"])
        sp = [m.start() for m in re.finditer(" ", t)] or [len(t)]   # 낱말 사이에만 (낱말 가운데를 쪼개면 사람도 못 읽는다)
        i = random.choice(sp)
        t = t[:i] + " " + " ".join([w] * random.randint(4, 12)) + t[i:]
    if r() < 0.10: t = t.rstrip() + random.choice([".", ".", "?", "~", ".."])
    return t

def hesitate(t):
    """머뭇거림·반복 (말 사이에 끼운다)."""
    ws = t.split(" ")
    if len(ws) < 3: return t
    k = random.random()
    i = random.randrange(1, len(ws))
    if k < 0.35: ws.insert(i, random.choice(["음", "어", "그", "음...", "어~", "그~", "막", "이제"]))
    elif k < 0.6: ws.insert(i, ws[i - 1])                 # 같은 말 두 번
    elif k < 0.75:
        w = ws[i]; j = max(1, len(w) // 2); ws[i] = w[:j] + "...." + w[j:]
    else: ws.insert(i, random.choice(["아 잠깐", "그러니까", "뭐지"]))   # '아니' 는 넣지 않는다 (정정으로 읽혀 정답과 어긋남)
    return " ".join(ws)

# ── 정해진 꼴 (정답) ─────────────────────────────────────────
def canon_transfer(start, pcol_said, pcol, stops):
    pen = (CANON_C[pcol].replace("색", "") + " 펜") if pcol_said else "펜"
    head = f"{place_canon_at(start)}의 {pen}" if start else pen
    out = eul(head) + " "
    for s in stops[:-1]: out += ro(place_canon(s)) + " 옮긴 다음 "
    return out + ro(place_canon(stops[-1])) + " 옮겨"

# ── 뜻 → (입력, 정답) ───────────────────────────────────────
def rand_place(exclude=None):
    if random.random() < 0.10:
        while True:
            p = ("n", random.randint(1, 20))
            if p != exclude: return p
    while True:
        p = ("c", random.choice(COLORS))
        if p != exclude: return p

def make_transfer(nsteps):
    start = rand_place() if random.random() < 0.65 else None
    pcol = random.choices(COLORS, weights=[1, 1, 1, 6, 1, 1])[0]
    pcol_said = random.random() < 0.6
    stops = []; prev = start
    for _ in range(nsteps):
        p = rand_place(exclude=prev); stops.append(p); prev = p
    pen = pen_say(pcol, pcol_said)
    # 첫 마디
    if start:
        first = random.choice([
            f"{place_say(start)}에 있는 {eul(pen)} {random.choice(GRAB)} ",
            f"{place_say(start)}에 있는 {pen} {random.choice(GRAB)} ",
            f"{place_say(start)}의 {eul(pen)} ",
            f"{place_say(start)}에서 {eul(pen)} {random.choice(GRAB)} ",
            f"{place_say(start)}에 있는 {eul(pen)} ",
            f"{place_say(start)}에는 {eul(pen)} {random.choice(GRAB)} "])
    else:
        first = random.choice([f"{eul(pen)} {random.choice(GRAB)} ", f"{pen} {random.choice(GRAB)} ",
                               f"{eul(pen)} ", f"{pen} "])
    body = first
    prev_place = start
    for k, s in enumerate(stops):
        v = pick_verb()
        last = (k == len(stops) - 1)
        if k > 0:
            body += random.choice(LAST_MARK) if last else random.choice(["", "", "", "다시 ", "그 다음에 "])
            if random.random() < 0.35: body += regrasp(prev_place, random.choice(["펜", "초록색 펜", "그 펜", "피피에이피"]))
        d = dest(v, s)
        if last:
            body += d + " " + "{END}" + "|" + str(VERBS.index(v))
        else:
            c = pick_connect()[1](v)
            body += d + " " + c + " "
        prev_place = s
    return body, canon_transfer(start, pcol_said, pcol, stops), stops

def finish(body):
    """'{END}|동사번호' 를 맺음말로 — 감쌀지 그냥 명령할지 정한다."""
    body, vi = body.rsplit("|", 1); v = VERBS[int(vi)]
    style = random.random()
    if style < 0.25:                                    # 그냥 명령 (실제 말은 대부분 감싼다)
        t = body.replace("{END}", random.choice(v["end"]))
        if random.random() < 0.3: t = random.choice(CALL) + t
    elif style < 0.80:                                  # "내가 하고 싶은 건 … 는 거 하고 싶어"
        t = random.choice(PREFIX) + " " + body.replace("{END}", random.choice(NOM_END).format(n=v["neun"]))
        if random.random() < 0.25: t = random.choice(CALL) + t
    else:                                               # "… 하면 좋겠어"
        t = random.choice(PREFIX) + " " + body.replace("{END}", random.choice(MYEON_END).format(m=v["myeon"]))
    t = t.replace("{END}", "")
    return t

def wrap_noise(t):
    # 실제 말 길이(틀린 문장 중앙 94자, 최대 221자)에 맞춰 감싸기·잡담을 넉넉히 (10-29 조정)
    if random.random() < 0.45: t = hesitate(t)
    if random.random() < 0.20: t = hesitate(t)
    if random.random() < 0.25: t = random.choice(CHAT) + ". " + t        # 앞 잡담
    if random.random() < 0.10: t = random.choice(CHAT) + " " + random.choice(CHAT) + ". " + t
    if random.random() < 0.08: t = ". ".join(random.sample(CHAT, random.randint(3, 6))) + ". " + t   # 옆 사람 대화가 길게 붙은 경우
    if random.random() < 0.35: t = t + random.choice(TAIL)               # 뒤 맺음
    if random.random() < 0.15: t = t + ". " + random.choice(CHAT)        # 뒤 잡담
    if random.random() < 0.05: t = t + ". " + random.choice(CHAT) + " " + random.choice(CHAT)
    if random.random() < 0.05:                                           # 앞말 반복
        for p in PREFIX:
            if t.startswith(p): t = p + " " + t; break
    return stt_noise(t)

def make_correction():
    """정정 — 마지막 말이 정답."""
    start = rand_place() if random.random() < 0.6 else None
    pcol = random.choices(COLORS, weights=[1, 1, 1, 6, 1, 1])[0]; said = random.random() < 0.6
    wrong = rand_place(exclude=start); right = rand_place(exclude=start)
    while right == wrong: right = rand_place(exclude=start)
    pen = pen_say(pcol, said); v = pick_verb()
    lead = (f"{place_say(start)}에 있는 {eul(pen)} {random.choice(GRAB)} " if start else f"{eul(pen)} ")
    end = random.choice(v["end"])
    t = random.choice([
        f"{lead}{dest(v, wrong)} {random.choice(['옮기지 말고', '말고', '아니 아니'])} {dest(v, right)} {end}",
        f"{lead}{place_say(wrong)} 아니 {dest(v, right)} {end}",
        f"{lead}{place_say(wrong)} 말고 {dest(v, right)} {end}",
        f"{lead}{dest(v, wrong)} 아 정정하자 {dest(v, right)} {end}",
        f"{lead}{dest(v, wrong)} 옮기는 거 아니고 {dest(v, right)} {end}"])
    return t, canon_transfer(start, said, pcol, [right])

SIMPLE = [  # (말 여러 꼴, 정해진 꼴)
    (["그리퍼 열어", "집게 열어줘", "그리퍼 좀 열어봐", "손 벌려"], "그리퍼 열어"),
    (["그리퍼 닫아", "집게 닫아줘", "그리퍼 오므려", "손 오므려"], "그리퍼 닫아"),
    (["내려", "아래로 내려줘", "하강", "밑으로 내려"], "내려"),
    (["올려", "위로 올려줘", "상승", "위로 들어"], "올려"),
    (["멈춰", "정지", "멈춰 새끼야", "그만", "스톱", "멈춰봐"], "멈춰"),
    (["다 지워", "초기화", "비워", "다 비워줘", "처음부터 다시"], "다 지워"),
    (["미리보기", "돌려봐", "미리 보여줘", "시뮬레이션 돌려봐"], "미리보기"),
    (["빠르게", "빨리 해줘", "속도 올려"], "빠르게"),
    (["천천히", "천천히 해줘", "느리게 해줘", "속도 줄여"], "천천히"),
]

def make_grab_only():
    start = rand_place(); pcol = random.choice(COLORS); said = random.random() < 0.5
    pen = pen_say(pcol, said)
    t = random.choice([f"{place_say(start)}에 있는 {eul(pen)} 잡아", f"{place_say(start)}에서 {pen} 집어줘",
                       f"{place_say(start)}의 {pen} 잡아줘", f"{place_say(start)}에 있는 거 잡아"])
    return t, f"{place_canon_at(start)}의 펜을 잡아"

def make_place_only():
    d = rand_place()
    t = random.choice([f"{place_say(d)}에 꽂아", f"{place_say(d)}에 놓아줘", f"{place_say(d)}에 넣어",
                       f"{place_say(d)}에 내려놔"])
    return t, f"{place_canon_at(d)}에 놓아줘"

def sample():
    k = random.random()
    if k < 0.28:   b, c, _ = make_transfer(1); return wrap_noise(finish(b)), c, "1단계"
    if k < 0.50:   b, c, _ = make_transfer(2); return wrap_noise(finish(b)), c, "2단계"
    if k < 0.68:   b, c, _ = make_transfer(random.randint(3, 5)); return wrap_noise(finish(b)), c, "3~5단계"
    if k < 0.73:                                   # 1단계 함정: 잡은 다음/후/나서
        b, c, _ = make_transfer(1)
        return wrap_noise(finish(b)), c, "1단계"
    if k < 0.80:   t, c = make_correction(); return wrap_noise(t), c, "정정"
    if k < 0.82:   t, c = make_grab_only(); return wrap_noise(t), c, "잡기만"
    if k < 0.83:   t, c = make_place_only(); return wrap_noise(t), c, "놓기만"
    if k < 0.85:
        says, c = random.choice(SIMPLE); return wrap_noise(random.choice(says)), c, "조작"
    # 명령 아님 (15%)
    t = random.choice(NONCMD)
    if random.random() < 0.4: t = t + " " + random.choice(NONCMD)
    if random.random() < 0.3: t = hesitate(t)
    return stt_noise(t), "(명령 아님)", "명령아님"

# ── 정답이 RACHEL 이 아는 글자뿐인가 (만들 때 검사) ──────────
RV = set(json.load(open(os.path.join(HERE, "rachel_vocab.json"), encoding="utf-8")))
def canon_ok(c):
    return c == "(명령 아님)" or all(ch in RV for ch in c)

def main():
    a = sys.argv[1:]
    if a and a[0] == "--show":
        n = int(a[1]) if len(a) > 1 else 30
        for _ in range(n):
            s, c, k = sample()
            print(f"[{k}] {s}\n      → {c}")
        return
    want = int(a[0]) if a else 100000
    bad = 0; seen = 0
    from collections import Counter
    kinds = Counter(); L = []
    while seen < want:
        s, c, k = sample()
        s = " ".join(s.split())
        if not canon_ok(c): bad += 1; continue
        print(json.dumps({"src": s, "dst": c}, ensure_ascii=False))
        seen += 1; kinds[k] += 1; L.append(len(s))
    L.sort()
    print(f"[생성] {seen:,}쌍 · 정답 글자 검사 탈락 {bad} · 종류 {dict(kinds)} · "
          f"입력 길이 중앙 {L[len(L)//2]} / 90% {L[int(len(L)*.9)]} / 최대 {L[-1]}", file=sys.stderr)

if __name__ == "__main__":
    main()
