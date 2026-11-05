#!/usr/bin/env python3
"""더블 레이첼 학습 데이터 — 장면 + 지저분한 말 → 정리문 + 블록.

말투 엔진은 nano_norm/gen_norm.py 를 그대로 쓴다 (실제 말 87개에서 정리기 92% 로 검증됨).
여기서 더하는 것:
  · 장면 줄 (팔 A/B 와 색, 도킹 색→번호, 펜 위치)
  · 팔 부르기 ("빨간 팔이", "B팔로", "두 번째 팔은" …) — 도킹도 색으로 부르므로 '팔' 이 붙어야 팔
  · 두 팔 넘겨받기 ("… 노랑에 두면 파란 팔이 받아서 초록으로")
  · 기다리기 ("10초 뒤에", "10초 있다가")
정답(정리문)은 생성 순간 정해지고, 블록은 canon.to_blocks 가 계산한다 → 라벨이 틀릴 일이 없다.

  python gen_double.py 1500000 > double_data.jsonl
  python gen_double.py --show 30
"""
import json, os, random, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "nano_norm"))
import gen_norm as G
from canon import COLORS, CANON_C, to_blocks, to_named, scene_str, NONE

random.seed(int(os.environ.get("DOUBLE_SEED", "30")))
G.random.seed(int(os.environ.get("DOUBLE_SEED", "30")) + 1)

NUM_MAX = 30
def rand_place(exclude=None):                        # 번호 도킹을 30번까지
    if random.random() < 0.10:
        while True:
            p = ("n", random.randint(1, NUM_MAX))
            if p != exclude: return p
    while True:
        p = ("c", random.choice(COLORS))
        if p != exclude: return p
G.rand_place = rand_place

# ── 팔 부르기 ───────────────────────────────────────────────
ARM_C_SAY = {"빨강": ["빨간", "빨간색", "빨강"], "주황": ["주황", "주황색", "오렌지"], "노랑": ["노란", "노란색", "노랑"],
             "초록": ["초록", "초록색", "녹색"], "파랑": ["파란", "파란색", "파랑"], "보라": ["보라", "보라색"]}
# 8차 (11-03): 음성 인식이 적는 팔 이름 — 실제 말 "비팔로 피피에이피를 잡아서 …" 를 A팔로 바꿨다.
#   B팔 → '비팔'·'비 팔'·'B8'(팔→8)·'비8'·'B8팔'·'비 로봇'·'비암'·'B 암' … (A팔 = 에이)
#   시험지(make_grid_arm.py)의 '처음 보는 꼴'(소문자 b팔 · 로봇 비 · 둘째 팔 · '을 써서' · '한테 시켜서')은 넣지 않는다.
V8 = os.environ.get("DOUBLE_V8", "1") == "1"
KOR8 = {"A": "에이", "B": "비", "C": "씨"}
def arm_say8(letter):
    K = KOR8[letter]
    return random.choices([
        f"{K}팔", f"{K} 팔", f"{letter}8", f"{K}8", f"{letter}8팔", f"{letter} 8", f"{K} 8",
        f"{K} 로봇", f"{K} 로봇팔", f"{K}로봇", f"{K}암", f"{K} 암", f"{letter} 암", f"{letter}암",
        f"{letter} 로봇팔", f"{letter}로봇"],
        weights=[20, 14, 8, 5, 5, 4, 3, 5, 5, 2, 3, 2, 2, 2, 3, 2])[0]

# 9차 (11-03): 8차 모델이 '비우고'·'비닐' 의 '비' 를 B팔로 읽었다 (실제 말 5문장 거부) → '비…' 낱말은 팔이 아니다.
#   시험지(make_grid_bi.py)의 처음 보는 낱말(비스듬히·비밀번호·비교·비행기·비빔밥·비번·씨앗)은 넣지 않는다.
#   실제 말 문장을 그대로 베끼지 않는다 ('비닐건 없어요?' ✗).
V9 = os.environ.get("DOUBLE_V9", "1") == "1"
BI9 = ["비우고", "판 비우고", "다 비우고", "먼저 비우고", "거기 비워 두고", "자리 비워 놓고", "싹 비워 주고",
       "비닐 치우고", "비닐 벗기고", "비닐 있어요?", "비닐 어디 있어요?", "비닐 좀 줘.",
       "비켜 봐", "좀 비켜 줄래?", "비켜 주세요.", "비슷하게", "비슷한 걸로", "비싼 거니까 조심해서",
       "비 오네.", "비 온대요.", "비상이야", "비디오 찍을게", "비용은 나중에 하고",
       "민수씨 이것 좀 봐요.", "지영 씨 여기요.", "김 씨가 그러는데", "박씨는 저기 들어가면 돼요?",
       "에이포 용지 치우고", "씨디 치우고", "에이 에스 끝났어?"]
def bi9(text):
    """'비…' 낱말을 맨 앞에 (정리문은 그대로 — 팔이 생기지 않는다)."""
    return random.choice(BI9) + random.choice([" ", " ", ", ", ". "]) + text

def arm_jong(s):
    """팔 이름의 받침 — 영어 글자는 읽으면 '에이'·'비' 라 받침 없음 (gen_norm._jong 은 영어를 받침 있다고 본다)."""
    s = s.rstrip()
    return 0 if s and s[-1].isascii() and s[-1].isalpha() else G._jong(s)
def arm_ro(s): return s + ("로" if arm_jong(s) in (0, 8) else "으로")

def arm_say(letter, color, idx):
    if V8 and random.random() < 0.35: return arm_say8(letter)
    return random.choices([
        f"{random.choice(ARM_C_SAY[color])} 팔", f"{random.choice(ARM_C_SAY[color])} 로봇팔",
        f"{random.choice(ARM_C_SAY[color])} 로봇 팔", f"{random.choice(ARM_C_SAY[color])}팔",
        f"{letter}팔", f"{letter} 팔", f"로봇 {letter}", f"{letter}번 팔" if False else f"{letter} 로봇",
        f"{idx + 1}번 팔", ["첫 번째 팔", "두 번째 팔", "세 번째 팔"][idx], f"{idx + 1}번 로봇팔"],
        weights=[22, 10, 4, 4, 10, 5, 3, 2, 8, 3, 3])[0]

def arm_subj(s):
    """팔 + 조사 (이/가/은/는/로/(없음)) · 8차: 가지고 · 로 해서."""
    j = arm_jong(s) if V8 else G._jong(s)
    ro = arm_ro(s) if V8 else G.ro(s)
    return random.choices([s + ("이" if j else "가"), s + ("은" if j else "는"), ro, s,
                           s + ("이" if j else "가") + " 먼저", s + " 가지고", s + "가지고", ro + " 해서"],
                          weights=[40, 15, 25, 10, 3] + ([6, 3, 2] if V8 else [0, 0, 0]))[0]

# ── 기다리기 ────────────────────────────────────────────────
KOR_NUM = {3: "삼", 5: "오", 10: "십", 20: "이십", 30: "삼십"}
def wait_say(sec):
    n = random.choices([f"{sec}초", f"{sec} 초", f"{KOR_NUM.get(sec, sec)} 초", f"한 {sec}초"], weights=[60, 15, 10, 15])[0]
    return random.choice([f"{n} 뒤에", f"{n} 후에", f"{n} 있다가", f"{n} 기다렸다가", f"{n} 쉬고", f"{n} 지나면",
                          f"{n} 정도 기다린 다음에", f"{n} 뒤"])

# ── 한 구간 (팔 하나가 하는 옮기기) ─────────────────────────
# ── 2차 (10-30): 실제 말 87개 중 정리기가 틀린 7개에서 본 말투 ──────
#   같은 자리에 다시 놓기 · 조사 실수(…스테이션을 놓는) · 무시할 동작(베이스 자세로 돌아와서)
#   같은 목적지 다시 말하기 · 끝에서 같은 말 반복 · 말 다시 시작하기 · 출발지 정정
#   "X에서 Y로" · 펜 없이 짧게("노란 도킹으로 옮겨") · 아주 긴 잡담
V2 = os.environ.get("DOUBLE_V2", "1") == "1"
# 4차 (10-30 새벽): 3차 2000걸음 채점에서 틀린 말투 — 팔을 중간·맨 뒤에 말하기 · 넘겨받기를 말하지 않기 ·
#   '다시 X에 꽂아' · 팔+조작('파란 팔 멈춰') · 도킹 이야기지만 명령 아님 · 'X, Y 순서로' · 잡은 뒤 '그 초록색 펜을' ·
#   같은 자리 다시 말하기('노란색에 이동을 하고 노란색에 이동한 다음')
V4 = os.environ.get("DOUBLE_V4", "1") == "1"
# 5차 (10-30 아침): 4차 12000걸음에서 거부된 실제 말 — 중간 단계를 '…하면 좋겠어. 그 다음에' 로 끊고 그 자리 펜을 다시 잡기 ·
#   '있을/위치할 펜' · 펜을 빠뜨림('빨간 도킹에 있는 노란색으로') · '음,.....' 머뭇거림 · '다시 그렇게 공정을 끝내고 싶어' · 더 긴 잡담
V5 = os.environ.get("DOUBLE_V5", "1") == "1"
# 6차 (10-30 오전): 말씨 격자(make_grid.py)에서 틀린 동사 — 꺼내다·챙기다·끼우다 — 와, 학습에 없던 잡기 말씨·동사·어미를 넓힌다.
#   격자 세트2(GRID_SET=2: 꽂아 넣다·집어넣다·움켜쥐고·-ㄹ까? …)는 일부러 넣지 않는다 — 처음 보는 말씨로 다시 재려고.
V6 = os.environ.get("DOUBLE_V6", "1") == "1"
I_OM = next(i for i, v in enumerate(G.VERBS) if v["adn"] == "옮긴")
IGNORE_ACT = ["베이스 자세로 돌아와서", "그 후 다시 베이스 자세로 돌아와서", "기본 자세로 돌아갔다가",
              "원래 자세로 돌아와서 그 이후에", "처음 자세로 돌아왔다가", "베이스로 갔다가"]
I_DU = next(i for i, v in enumerate(G.VERBS) if v["adn"] == "둔")

# ── 6차: 동사 활용 넓히기 ───────────────────────────────────
#   a: -어 꼴 · ul: -ㄹ · ja: -자 · sip: -ㅂ시다 · stem: 줄기 · gi: -기
FORMS6 = {"옮긴": ("옮겨", "옮길", "옮기자", "옮깁시다", "옮기", "옮기기"),
          "이동한": ("이동해", "이동할", "이동하자", "이동합시다", "이동하", "이동하기"),
          "이동시킨": ("이동시켜", "이동시킬", "이동시키자", "이동시킵시다", "이동시키", "이동시키기"),
          "갖다 놓은": ("갖다 놔", "갖다 놓을", "갖다 놓자", "갖다 놓읍시다", "갖다 놓", "갖다 놓기"),
          "놓은": ("놔", "놓을", "놓자", "놓읍시다", "놓", "놓기"), "둔": ("둬", "둘", "두자", "둡시다", "두", "두기"),
          "꽂은": ("꽂아", "꽂을", "꽂자", "꽂읍시다", "꽂", "꽂기"), "넣은": ("넣어", "넣을", "넣자", "넣읍시다", "넣", "넣기"),
          "보낸": ("보내", "보낼", "보내자", "보냅시다", "보내", "보내기"),
          "가져간": ("가져가", "가져갈", "가져가자", "가져갑시다", "가져가", "가져가기")}
def _verb6(a, adn, go, daga, seo, myeon, neun, ul, ja, sip, stem, gi, jo, w):
    return dict(adn=adn, go=go, daga=daga, seo=seo, myeon=myeon, neun=neun, end=[a, a + "줘", a + " 줘"], jo=jo, w=w,
                a=a, ul=ul, ja=ja, sip=sip, stem=stem, gi=gi)
NEW_VERBS6 = [
    _verb6("끼워", "끼운", "끼우고", "끼웠다가", "끼워서", "끼우면", "끼우는", "끼울", "끼우자", "끼웁시다", "끼우", "끼우기", "e", 6),
    _verb6("옮겨 놔", "옮겨 놓은", "옮겨 놓고", "옮겨 놨다가", "옮겨 놔서", "옮겨 놓으면", "옮겨 놓는", "옮겨 놓을", "옮겨 놓자", "옮겨 놓읍시다", "옮겨 놓", "옮겨 놓기", "e", 3),
    _verb6("가져다 놔", "가져다 놓은", "가져다 놓고", "가져다 놨다가", "가져다 놔서", "가져다 놓으면", "가져다 놓는", "가져다 놓을", "가져다 놓자", "가져다 놓읍시다", "가져다 놓", "가져다 놓기", "e", 4),
    _verb6("가져다 둬", "가져다 둔", "가져다 두고", "가져다 뒀다가", "가져다 둬서", "가져다 두면", "가져다 두는", "가져다 둘", "가져다 두자", "가져다 둡시다", "가져다 두", "가져다 두기", "e", 3),
    _verb6("갖다 꽂아", "갖다 꽂은", "갖다 꽂고", "갖다 꽂았다가", "갖다 꽂아서", "갖다 꽂으면", "갖다 꽂는", "갖다 꽂을", "갖다 꽂자", "갖다 꽂읍시다", "갖다 꽂", "갖다 꽂기", "e", 3),
    _verb6("올려놔", "올려놓은", "올려놓고", "올려놨다가", "올려놔서", "올려놓으면", "올려놓는", "올려놓을", "올려놓자", "올려놓읍시다", "올려놓", "올려놓기", "e", 2),
    _verb6("내려놔", "내려놓은", "내려놓고", "내려놨다가", "내려놔서", "내려놓으면", "내려놓는", "내려놓을", "내려놓자", "내려놓읍시다", "내려놓", "내려놓기", "e", 3),
    _verb6("가서 내려놔", "가서 내려놓은", "가서 내려놓고", "가서 내려놨다가", "가서 내려놔서", "가서 내려놓으면", "가서 내려놓는", "가서 내려놓을", "가서 내려놓자", "가서 내려놓읍시다", "가서 내려놓", "가서 내려놓기", "ro", 2),
    _verb6("가져가서 꽂아", "가져가서 꽂은", "가져가서 꽂고", "가져가서 꽂았다가", "가져가서 꽂아서", "가져가서 꽂으면", "가져가서 꽂는", "가져가서 꽂을", "가져가서 꽂자", "가져가서 꽂읍시다", "가져가서 꽂", "가져가서 꽂기", "ro", 2),
]
def _end6(v):
    """맺음 어미 20가지 (원래 끝맺음은 두 번 넣어 비중 유지)."""
    a, ul, ja, sip, stem, gi = v["a"], v["ul"], v["ja"], v["sip"], v["stem"], v["gi"]
    return [a + " 주세요", a + " 줄래?", a + " 줄 수 있어?", a + " 주시겠어요?", a + " 주라", a + "라", a + "요", a + " 줘요",
            ja, sip, ul + "래?", ul + " 수 있어?", v["myeon"] + " 돼", v["go"] + " 싶어", a + "야 돼", ul + " 거야",
            gi + " 바래", stem + "도록 해", a + " 주면 돼", a + " 봐"]
GRAB6 = ["꺼내서", "꺼내 가지고", "꺼내다가", "꺼내고", "챙겨서", "챙겨 가지고", "챙기고", "뽑아서", "뽑아 가지고", "갖고",
         "가지고", "들고", "쥐고", "집어 들고", "잡아 가지고", "잡아 갖고", "빼 가지고", "빼다가", "집고", "집어다가", "집은 뒤에",
         "들어 올려서", "꺼낸 다음에", "뽑은 다음에", "챙긴 다음에"]
if V6:
    for v in G.VERBS:
        f = FORMS6.get(v["adn"])
        if f: v.update(a=f[0], ul=f[1], ja=f[2], sip=f[3], stem=f[4], gi=f[5])
    G.VERBS.extend(NEW_VERBS6)
    for v in G.VERBS:
        if "a" in v: v["end"] = v["end"] * 2 + _end6(v)
    G.GRAB = G.GRAB * 2 + GRAB6
    _a = lambda v: v.get("a")
    G.CONNECT += [("어 가지고", lambda v: (_a(v) + " 가지고") if _a(v) else v["seo"], 3),
                  ("자마자", lambda v: (v["stem"] + "자마자") if v.get("stem") else v["go"], 2),
                  ("고서", lambda v: v["go"] + "서", 2),
                  ("어 놓고", lambda v: (_a(v) + " 놓고") if _a(v) and not _a(v).endswith(("놔", "둬")) else v["go"], 2),
                  (". 그리고", lambda v: random.choice(v["end"][:3]) + ". 그리고", 3),
                  (". 그 다음에", lambda v: random.choice(v["end"][:3]) + ". 그 다음에", 2),
                  ("어 줘. 그러고 나서", lambda v: random.choice(v["end"][:3]) + ". 그러고 나서", 1)]

# 7차 (10-31): 모르는 잇는 말이 오면 뒤 단계를 버린다 ('옮긴 담에 노랑으로' → 파랑만).
#   잇는 말 자리에 '아무 말' 을 끼워 '목적지가 더 나오면 단계가 더 있다' 를 배우게 한다 (목록을 외우는 대신).
#   격자 세트2·세트3(make_grid.py GRID_SET=2·3) 의 말씨는 넣지 않는다 — BLOCK7 으로 막는다.
V7 = os.environ.get("DOUBLE_V7", "1") == "1"
FILLER7 = ["음", "어", "그", "일단", "이제", "자", "좀", "그러면", "그럼", "막", "또", "다시", "얼른", "천천히", "아",
           "그래서", "그리고", "그 다음", "거기", "그쪽", "그러니까", "뭐", "응", "저기", "있지", "음…", "어…", "그…"]
SYL7 = "가나다라마바사아자차카타파하너더러머버서어저처커터퍼허노도로모보소오조초코토포호누두루무부수우주추쿠투푸후기니디리미비시이지치키티피히"
BLOCK7 = ["담", "거기서", "바로", "그러고", "나면", "다음엔", "난 뒤", "고는", "구", "나선", "그 자리", "놓고서", "찍고",
          "있잖아", "말이야", "도킨", "토킹", "팬", "스테이숀"]
def glue7():
    """잇는 말 자리에 끼우는 아무 말 1~2개 (뜻 없는 말 포함)."""
    while True:
        ws = []
        for _ in range(random.choices([1, 2], weights=[70, 30])[0]):
            r = random.random()
            if r < 0.55: ws.append(random.choice(FILLER7))
            elif r < 0.85: ws.append("".join(random.choice(SYL7) for _ in range(random.randint(1, 3))))
            else: ws.append(random.choice(CHAT2).split()[0])
        g = " ".join(ws)
        if not any(b in g for b in BLOCK7): return g
def connect7(v):
    """아는 잇는 말 + 아무 말 · 관형형 + 아무 말 · 맺음 + 아무 말."""
    r = random.random()
    if r < 0.4: c = G.pick_connect()[1](v) + " " + glue7()
    elif r < 0.75: c = v["adn"] + " " + glue7()
    else: c = random.choice(v["end"]) + random.choice([". ", ", ", " "]) + glue7()
    return c if not any(b in c for b in BLOCK7) else G.pick_connect()[1](v)
TOPIC7 = ["{p}에 있는 펜 알지? 그거 ", "{p}에 있는 펜 보이지? 그걸 ", "{p}에 꽂혀 있는 펜 있지 그거 ", "{p} 펜 그거 ",
          "{p}에 있는 거 그 펜 ", "저기 {p}에 있는 펜 그걸 "]
TOPIC7 = [t for t in TOPIC7 if t != "{p} 펜 그거 "]                     # '보라색 펜' 은 펜 색으로 읽힌다

def sample_single7(arms, pcol):
    """7차: 자리만 이어 말하기 · 팔 이름 없는 한 팔 + 기다리기."""
    said = random.random() < 0.6; pen = G.pen_say(pcol, said)
    start = rand_place(); ps = G.place_say
    if random.random() < 0.5:                           # "보라색에 있는 펜 노랑으로, 노랑에서 파랑으로 옮겨"
        stops = []; prev = start
        for _ in range(random.choice([2, 2, 3])): p = rand_place(exclude=prev); stops.append(p); prev = p
        links = [f"{ps(a)}에서 {G.ro(ps(b))}" for a, b in zip(stops, stops[1:])]
        sep = random.choice([", ", " 그리고 ", ", 그리고 ", " 그 다음 ", " 다음에 "])
        if random.random() < 0.5: body = f"{ps(start)}에 있는 {pen} {G.ro(ps(stops[0]))}" + sep + sep.join(links)
        else: body = f"{ps(start)}에서 {G.ro(ps(stops[0]))}" + sep + sep.join(links)
        body += " {END}|" + str(I_OM)
        canon = G.canon_transfer(start, said and "에 있는" in body, pcol, stops); kind = "나열"
    else:                                               # "… 노랑에 꽂고 10초 뒤에 다시 보라로 옮겨" (팔 이름 없음)
        s1 = rand_place(exclude=start); s2 = start if random.random() < 0.4 else rand_place(exclude=s1)
        sec = random.choice([3, 5, 5, 10, 10, 10, 15, 20, 30])
        v = G.pick_verb()
        body1, st = seg_body(start, 1, False, pen, False); s1 = st[-1]
        if s2 == s1: s2 = rand_place(exclude=s1)
        body = body1 + wait_say(sec) + " " + random.choice(["다시 ", "그거 다시 ", "", "그 펜 "]) + G.dest(v, s2) + \
               " {END}|" + str(G.VERBS.index(v))
        canon = G.canon_transfer(start, said, pcol, [s1]) + f" / {sec}초 기다려 / " + G.canon_transfer(s1, False, pcol, [s2])
        kind = "한팔기다리기"
    letter = None
    if random.random() < 0.3:
        letter = random.choice(list(arms)); body = arm_intro(letter, arms) + body
        canon = " / ".join(c if "기다려" in c else f"{letter}팔: " + c for c in canon.split(" / "))
    text = more_noise(G.wrap_noise(G.finish(body)))
    return text, canon, finish_scene(arms, canon, pcol, start), kind + ("·팔" if letter else "")

def regrasp5(p, pen):
    """방금 놓은 자리를 다시 부르며 다시 잡기 (5차) — 단계를 늘리지 않는다."""
    adn = random.choice(["있는", "있을", "위치할", "있던", "놓인", "위치한", "있는"])
    g = random.choice(["다시 잡은 다음에", "잡아서", "다시 잡아서", "잡고", "잡은 다음에", "다시 잡고", "잡아다가"])
    return f"{G.place_say(p)}에 {adn} {random.choice([G.eul(pen), G.eul('펜'), pen])} {g} "

def dest2(v, p):
    if V2 and random.random() < 0.04: return G.eul(G.place_say(p))      # 조사 실수
    return G.dest(v, p)

def seg_body(start, nsteps, handoff, pen, last_seg, drop_pen=False):
    """말 몸통과 (stops) — 끝은 이어지는 꼴(마지막 구간이면 {END}|v)."""
    stops = []; prev = start
    for _ in range(nsteps):
        if V2 and prev and random.random() < 0.06: p = prev                # 같은 자리에 다시
        else: p = rand_place(exclude=prev)
        stops.append(p); prev = p
    fromto = V2 and start and not handoff and random.random() < 0.15        # "보라색에서 노란색으로 펜을 옮기고"
    if handoff and V4 and random.random() < 0.25:
        first = random.choice(["", "", "다시 ", "그 다음에 ", "이어서 "])           # 넘겨받기를 말하지 않는다
    elif handoff:
        first = random.choice(["그거 받아서 ", "그걸 받아서 ", "받아서 ", "이어받아서 ", "그 펜을 받아서 ",
                               "거기 있는 펜 잡아서 ", f"{G.place_say(start)}에 있는 {G.eul(pen)} 잡아서 ",
                               f"{G.place_say(start)}에서 {pen} 받아서 ", "그거 다시 잡아서 ", "그걸 "]
                              + ([f"{G.place_say(start)}에서 {random.choice(G.GRAB)} ", f"{G.place_say(start)}에 있는 거 {random.choice(G.GRAB)} "]
                                 if V6 else []))
    elif fromto:
        first = f"{G.place_say(start)}에서 "
    elif start and drop_pen:
        first = f"{G.place_say(start)}에 있는 "                              # 펜을 빠뜨림 (5차)
    elif V7 and start and random.random() < 0.05:
        first = random.choice(TOPIC7).format(p=G.place_say(start))          # "보라색에 있는 펜 알지? 그거" (7차)
    elif start:
        first = random.choice([f"{G.place_say(start)}에 있는 {G.eul(pen)} {random.choice(G.GRAB)} ",
                               f"{G.place_say(start)}에 있는 {pen} {random.choice(G.GRAB)} ",
                               f"{G.place_say(start)}에서 {G.eul(pen)} {random.choice(G.GRAB)} ",
                               f"{G.place_say(start)}의 {G.eul(pen)} "])
    elif V2 and random.random() < 0.10:
        first = ""                                                          # 펜 없이 짧게
    else:
        first = random.choice([f"{G.eul(pen)} {random.choice(G.GRAB)} ", f"{pen} {random.choice(G.GRAB)} ", f"{G.eul(pen)} "])
    if V4 and start and not handoff and not fromto and random.random() < 0.08:
        first += f"그 {G.eul(pen)} "                                          # 잡은 다음에 그 초록색 펜을 …
    body = first; prev_place = start; regrasped = False
    if V4 and last_seg and len(stops) >= 3 and not fromto and random.random() < 0.10:   # "주황색, 보라색 순서로"
        v = G.VERBS[I_OM]
        body += G.dest(v, stops[0]) + " " + random.choice([v["go"], v["adn"] + " 다음에"]) + " " + \
                random.choice(["", "다음에 ", "그 다음 "]) + ", ".join(G.place_say(x) for x in stops[1:]) + \
                random.choice([" 순서로 ", " 순으로 ", " 차례로 "]) + "{END}|" + str(I_OM)
        return body, stops
    for k, s in enumerate(stops):
        v = G.pick_verb(); last = (k == len(stops) - 1)
        if k > 0 and not regrasped and random.random() < 0.3:
            body += regrasp5(prev_place, pen) if V5 and random.random() < 0.3 else G.regrasp(prev_place, "펜")
        regrasped = False
        if k > 0 and V2 and random.random() < 0.04: body += random.choice(IGNORE_ACT) + " "
        if fromto and k == 0:
            d = G.ro(G.place_say(s)) + random.choice(["", " " + G.eul(pen), " 펜을", " 그거를"])
        else: d = dest2(v, s)
        if last and last_seg and V2 and random.random() < 0.05:            # 같은 목적지 다시 말하기
            body += f"{G.ro(G.place_say(s))} {v['seo']} {G.eul(pen)} {G.place_say(s)}에 " + "{END}|" + str(I_DU)
        elif last and last_seg: body += d + " {END}|" + str(G.VERBS.index(v))
        elif last:                                    # 다음 팔로 넘어가는 꼴
            body += d + " " + random.choice([v["go"], v["adn"] + " 다음에", v["myeon"], v["adn"] + " 후에",
                                              v["go"] + " 나서", random.choice(v["end"]) + ". 그리고",
                                              random.choice(v["end"]) + ". 그 다음에", v["daga"]]) + " "
        elif V5 and random.random() < 0.07:                 # "노란색으로 이동하면 좋겠어. 그 다음에 노란색에 있는 펜을 다시 잡고"
            body += d + " " + random.choice([v["myeon"] + " 좋겠어", v["end"][0] + "주면 좋겠어", v["myeon"] + " 좋겠어요",
                                              random.choice(v["end"])]) + random.choice([". ", " . ", " ", ", "]) + \
                    random.choice(["그리고 나서 ", "그 다음에 ", "그 다음 ", "그리고 ", "그러고 나서 "]) + regrasp5(s, pen)
            regrasped = True
        elif V4 and random.random() < 0.04:                 # "노란색에 이동을 하고 노란색에 이동한 다음"
            body += d + " " + v["go"] + " " + G.place_say(s) + "에 " + v["adn"] + " 다음에 "
        elif V7 and random.random() < 0.20:                 # 잇는 말 자리에 아무 말 (7차)
            body += d + " " + connect7(v) + " "
        else: body += d + " " + G.pick_connect()[1](v) + " "
        prev_place = s
    return body, stops

CHAT2 = ["어디서 와요", "버스 몇 번 타요", "점심 뭐 먹지", "이따 몇 시에 끝나", "주말에 뭐 해요", "알바 끝나고 와요",
         "그 가게 알아요", "거기 맨날 가요", "기숙사 살아요", "시험 언제야", "과제 다 했어", "배고프다",
         "수업 몇 시에 끝나요", "그거 다른 거예요", "아 그렇구나", "진짜요", "몰랐어요", "집이 어디예요",
         "다음 주에 와요", "그럼 내일 봐요", "택배 왔대", "충전기 있어요", "와이파이 비번 뭐야", "아 졸려",
         "역 근처에서 해요", "일요일 밤에 와요", "수업 끝나면 돌아가요", "주말 알바예요", "매주 왔다 갔다 해요",
         "편의점 다른 거예요", "그 마트 알아요", "학교 앞에 있어요", "기숙사 몇 관이에요", "집은 멀어요"]
NONCMD_DOCK = ["{c1} 도킹 스테이션이랑 {c2} 도킹 스테이션 위치 바꿀까", "{c1} 도킹 스테이션 몇 개야", "{c1} 도킹 어디 뒀더라",
               "{c1} 도킹 스테이션 색 다시 칠할까", "{c1} 도킹이 {c2} 도킹보다 높네", "{c1} 도킹 스테이션 하나 더 만들까",
               "{c1} 도킹 스테이션 좀 흔들리는데", "{c1}이랑 {c2} 중에 뭐가 예뻐", "{c1} 도킹 스테이션 나사 풀렸어"]
NONCMD_ARM = ["{arm} 잘 되네", "{arm} 예쁘다", "{arm} 소리가 좀 나는데", "{arm} 지금 뭐 하는 거야",
              "{arm} 언제 만들었어", "{arm} 색 바꿀까", "{col} 도킹 스테이션 어디 있어", "{col} 좋아해",
              "{col} 펜 예쁘다", "{arm} 오늘 상태 괜찮네", "{arm} 아까 떨리던데", "{col} 색깔 별로다"]

HES5 = ["음,.....", "음,...", "어,..", "음,", "그,....", ".....", "음..,", "어...음"]
def hesitate5(t):
    """5차 머뭇거림: '음,.....' 끼우기 · 낱말 가운데 '....' (빨간....색)."""
    ws = t.split(" ")
    if len(ws) < 2: return t
    i = random.randrange(1, len(ws))
    if random.random() < 0.6: ws.insert(i, random.choice(HES5))
    else:
        w = ws[i]
        if len(w) >= 2: j = random.randint(1, len(w) - 1); ws[i] = w[:j] + random.choice(["....", ".....", "...", "..."]) + w[j:]
    return " ".join(ws)

END5 = ["다시 그렇게 공정을 끝내고 싶어", "그렇게 공정을 끝내고 싶어", "그걸로 공정 끝", "그렇게 끝내줘", "그러고 공정을 마치고 싶어",
        "그리고 끝내", "그렇게 하고 끝", "그러면 공정 끝이야", "그렇게 마무리해줘", "다시 그렇게 해서 끝내고 싶어"]

def more_noise(t):
    """2차 잡음: 말 다시 시작하기 · 아주 긴 되풀이 루프."""
    if not V2: return t
    if V5 and random.random() < 0.12: t = hesitate5(t)
    if V5 and random.random() < 0.04: t = hesitate5(t)
    if random.random() < 0.06:                        # 말 다시 시작하기 ("보라색 ppap 초록색 보라색 도킹 …")
        ws = t.split()
        if len(ws) > 4:
            frag = ws[:min(8, len(ws))]
            pick = [random.choice(frag) for _ in range(random.randint(1, 3))]
            pick = [w[:random.randint(1, len(w))] if random.random() < 0.3 else w for w in pick]
            i = random.randrange(0, min(6, len(ws)))
            ws = ws[:i] + pick + ws[i:]; t = " ".join(ws)
    if random.random() < 0.02:
        sp = [m.start() for m in __import__("re").finditer(" ", t)] or [len(t)]
        i = random.choice(sp); t = t[:i] + " " + " ".join(["색"] * random.randint(15, 50)) + t[i:]
    return t

def arm_intro(L, arms):
    """구간 앞 팔 부르기 (+ 가끔 팔 정정·다른 팔은 쉬고)."""
    s = arm_subj(arm_say(L, arms[L], ord(L) - 65)) + " "
    others = [o for o in arms if o != L]
    if V2 and others and random.random() < 0.06:
        o = random.choice(others)
        s = f"{arm_say(o, arms[o], ord(o) - 65)} {random.choice(['아니', '말고', '아니 아니', '아 아니다'])} " + s
    elif V2 and others and random.random() < 0.04:
        o = random.choice(others)
        s = f"{arm_say(o, arms[o], ord(o) - 65)}{random.choice(['은', '는'])} {random.choice(['쉬고', '가만히 있고', '놔두고'])} " + s
    return s

def canon_seg(letter, start, said, pcol, stops):
    c = G.canon_transfer(start, said, pcol, stops)
    return (f"{letter}팔: " + c) if letter else c

# ── 장면 ────────────────────────────────────────────────────
def make_arms(n):
    cols = random.sample(COLORS, n)
    return {chr(65 + i): c for i, c in enumerate(cols)}

def finish_scene(arms, canon, pen_col, start_place):
    """정리문에 나온 색은 반드시 장면에, 그 밖에 0~3개 더. 번호와 겹치지 않는 번호를 준다."""
    used_nums = {int(x) for x in __import__("re").findall(r"(\d+)번 도킹", canon)}
    said_cols = [c for c in COLORS if CANON_C[c] in canon]
    extra = [c for c in COLORS if c not in said_cols]
    random.shuffle(extra)
    cols = said_cols + extra[:random.randint(0, min(3, len(extra)))]
    random.shuffle(cols)
    free = [i for i in range(1, NUM_MAX + 1) if i not in used_nums]
    ids = random.sample(free, len(cols))
    docks = {c: f"d{i}" for c, i in zip(cols, ids)}
    if start_place:
        home = f"d{start_place[1]}" if start_place[0] == "n" else docks[start_place[1]]
    else:
        home = random.choice(list(docks.values()) + [f"d{random.choice(free)}"]) if docks else f"d{random.choice(free)}"
    return dict(arms=arms, docks=docks, pen=pen_col, pen_home=home)

# ── 표본 ────────────────────────────────────────────────────
def sample_single():
    """팔 하나 — 실제 말과 같은 종류 (gen_norm 그대로) + 가끔 팔 이름을 붙인다."""
    arms = make_arms(random.choice([1, 2, 2]))
    k = random.random()
    start = None; pcol = random.choices(COLORS, weights=[1, 1, 1, 6, 1, 1])[0]
    if V7 and k < 0.06: return sample_single7(arms, pcol)
    if k < 0.70:
        n = random.choices([1, 2, 3, 4, 5], weights=[30, 30, 18, 12, 10])[0]
        if V2 and random.random() < 0.5:              # 2차 말투 (seg_body)
            start = rand_place() if random.random() < 0.65 else None
            said = random.random() < 0.6; pen = G.pen_say(pcol, said)
            drop = V5 and start is not None and random.random() < 0.05
            if drop: said = False
            body, stops = seg_body(start, n, False, pen, True, drop_pen=drop)
            canon = G.canon_transfer(start, said, pcol, stops)
            if start and random.random() < 0.05:      # 출발지 정정
                wrong = rand_place(exclude=start)
                body = f"{G.place_say(wrong)}에 있는 {random.choice(['어~', '음', ''])} " \
                       f"{random.choice(['정정하자.', '아니 아니', '아 잠깐 정정', '아니다', '아 틀렸다'])} " + body
        else:
            body, canon, stops = G.make_transfer(n)
            m = __import__("re").match(r"(\d+)번 도킹의|(빨간색|주황색|노란색|초록색|파란색|보라색) 도킹의", canon)
            if m: start = ("n", int(m.group(1))) if m.group(1) else ("c", {v: k2 for k2, v in CANON_C.items()}[m.group(2)])
        letter = None
        arm_tail = None
        if random.random() < 0.35:                    # 팔 이름을 붙인다
            letter = random.choice(list(arms)); idx = ord(letter) - 65
            r4 = random.random()
            if V4 and r4 < 0.15:                      # 문장 중간에 ("2번 도킹에 있는 펜을 빨간 팔로 9번에")
                ws = body.split(" "); i = random.randint(1, max(1, min(6, len(ws) - 2)))
                ws.insert(i, (arm_ro if V8 else G.ro)(arm_say(letter, arms[letter], idx))); body = " ".join(ws)
            elif V4 and r4 < 0.25:                    # 맨 뒤에 덧붙임 ("… 옮겨 주세요 빨간 팔로요")
                arm_tail = (arm_ro if V8 else G.ro)(arm_say(letter, arms[letter], idx)) + random.choice(["요", "", " 해 줘", " 부탁해"])
            else:
                body = arm_intro(letter, arms) + body
            canon = f"{letter}팔: " + canon
        text = G.finish(body)
        if V8 and letter is None and random.random() < 0.03:     # 감탄사 '에이' 는 팔이 아니다 (8차)
            text = random.choice(["에이 ", "에이, ", "에이 뭐 ", "아 에이 ", "에이 좋아 ", "에이 알았어 "]) + text
        if V9 and random.random() < 0.08: text = bi9(text)    # '비우고 …' 는 B팔이 아니다 (9차)
        if arm_tail: text = text.rstrip(".?~ ") + " " + arm_tail
        if V5 and random.random() < 0.04:             # "… 갖다 두고 다시 그렇게 공정을 끝내고 싶어"
            text = text.rstrip(".?~ ") + random.choice([" ", ". ", " 그리고 "]) + random.choice(END5)
        if V2 and random.random() < 0.04:             # 끝에서 같은 말 반복 (단계가 늘지 않는다)
            text += " " + " ".join([f"{G.ro(G.place_say(stops[-1]))} {random.choice(['옮겨야죠.', '옮겨.', '옮겨줘.', '옮겨져', '옮기는 거야.'])}"]
                                   * random.randint(1, 4))
        text = more_noise(G.wrap_noise(text)); kind = f"{n}단계" + ("·팔" if letter else "")
    elif k < 0.80: text, canon = G.make_correction(); text = G.wrap_noise(text); kind = "정정"
    elif k < 0.83: text, canon = G.make_grab_only(); text = G.wrap_noise(text); kind = "잡기만"
    elif k < 0.85: text, canon = G.make_place_only(); text = G.wrap_noise(text); kind = "놓기만"
    elif k < 0.88:
        says, canon = random.choice(G.SIMPLE); t0 = random.choice(says); kind = "조작"
        if V4 and random.random() < 0.35:              # "파란 팔 멈춰" · "빨간 팔 그리퍼 열어"
            L = random.choice(list(arms)); a = arm_say(L, arms[L], ord(L) - 65)
            t0 = random.choice([a + " ", a + " ", (arm_ro if V8 else G.ro)(a) + " ", a + ("이 " if (arm_jong if V8 else G._jong)(a) else "가 ")]) + t0
            if canon in ("그리퍼 열어", "그리퍼 닫아", "내려", "올려"): canon = f"{L}팔: {canon}"   # 그 팔만
            kind = "조작·팔"                                                                    # 멈춰·다 지워 … 는 전체
        text = G.wrap_noise(t0)
    else:
        r = random.random()
        if V2 and r < 0.35:                           # 아주 긴 잡담 (옆 사람 대화가 통째로 들어온 경우)
            t = " ".join(random.choice(CHAT2 + G.CHAT + (BI9 if V9 else [])) + random.choice(["", ".", "?", "..", " 어?"])
                         for _ in range(random.randint(4, 28 if V5 else 16)))
        elif V4 and r < 0.43:                         # 도킹 이야기지만 시키는 말이 아님
            c1, c2 = random.sample(COLORS, 2)
            t = random.choice(NONCMD_DOCK).format(c1=random.choice(G.SAY_C[c1]), c2=random.choice(G.SAY_C[c2]))
        elif V2 and r < 0.50:                         # 팔·색 이야기지만 시키는 말이 아님
            L = random.choice(list(arms))
            t = random.choice(NONCMD_ARM).format(arm=arm_say(L, arms[L], ord(L) - 65),
                                                 col=random.choice(G.SAY_C[random.choice(COLORS)]))
        else:
            t = random.choice(G.NONCMD)
            if random.random() < 0.4: t = t + " " + random.choice(G.NONCMD)
        if random.random() < 0.3: t = G.hesitate(t)
        text, canon, kind = G.stt_noise(t), NONE, "명령아님"
    if canon not in (NONE,) and start is None:
        m = __import__("re").match(r"(?:[A-Z]팔: )?(?:(\d+)번 도킹|(빨간색|주황색|노란색|초록색|파란색|보라색) 도킹)", canon)
        if m: start = ("n", int(m.group(1))) if m.group(1) else ("c", {v: k2 for k2, v in CANON_C.items()}[m.group(2)])
    sc = finish_scene(arms, canon, pcol, start)
    return text, canon, sc, kind

def sample_double():
    """두 팔 (또는 같은 팔 두 번) — 구간 2~3개, 사이에 기다리기."""
    arms = make_arms(2)
    nseg = random.choices([2, 3], weights=[75, 25])[0]
    first = random.choice("AB")
    order = [first]
    for _ in range(nseg - 1):
        order.append(("B" if order[-1] == "A" else "A") if random.random() < 0.85 else order[-1])
    pcol = random.choices(COLORS, weights=[1, 1, 1, 6, 1, 1])[0]; said = random.random() < 0.6
    pen = G.pen_say(pcol, said)
    start = rand_place() if random.random() < 0.7 else None
    first_start = start
    text = ""; canons = []; arm_named = set()
    for i, L in enumerate(order):
        last = (i == nseg - 1)
        handoff = i > 0 and random.random() < 0.85
        if i > 0 and not handoff: start = rand_place(exclude=start)
        nsteps = random.choices([1, 2], weights=[80, 20])[0]
        body, stops = seg_body(start, nsteps, handoff, pen, last)
        name_arm = (i == 0 and random.random() < 0.9) or (i > 0 and (L != order[i - 1] or random.random() < 0.3))
        if not name_arm and L not in arm_named and i == 0: name_arm = True
        seg_text = (arm_intro(L, arms) if name_arm else "") + body
        if name_arm: arm_named.add(L)
        if i > 0 and random.random() < 0.5:           # 기다리기
            sec = random.choice([3, 5, 5, 10, 10, 10, 15, 20, 30])
            w = wait_say(sec)
            seg_text = (w + " " + seg_text) if random.random() < 0.7 else (seg_text.replace(" ", " " + w + " ", 1) if False else w + " " + seg_text)
            canons.append(f"{sec}초 기다려")
        text += seg_text
        cstart = start if (i == 0 or handoff or True) else None
        canons.append(canon_seg(L, cstart, said and i == 0, pcol, stops))
        start = stops[-1]
    text = G.finish(text)
    if V9 and random.random() < 0.05: text = bi9(text)
    text = more_noise(G.wrap_noise(text))
    canon = " / ".join(canons)
    sc = finish_scene(arms, canon, pcol, first_start)
    return text, canon, sc, f"두팔{nseg}"

def sample():
    if random.random() < 0.30: return sample_double()
    return sample_single()

def make_row():
    while True:
        text, canon, sc, kind = sample()
        try: to_blocks(canon, sc); blocks = to_named(canon)     # 3차: 이름 블록 (번호 찾기는 계산)
        except Exception as e: continue
        return dict(scene=scene_str(sc), text=" ".join(text.split()), canon=canon, blocks=" ".join(blocks), kind=kind)

def main():
    a = sys.argv[1:]
    if a and a[0] == "--show":
        for _ in range(int(a[1]) if len(a) > 1 else 20):
            r = make_row()
            print(f"[{r['kind']}] {r['scene']}\n  말: {r['text']}\n  정리: {r['canon']}\n  블록: {r['blocks']}\n")
        return
    want = int(a[0]) if a else 100000
    from collections import Counter
    kinds = Counter(); L = []
    for _ in range(want):
        r = make_row(); print(json.dumps(r, ensure_ascii=False)); kinds[r["kind"]] += 1; L.append(len(r["text"]))
    L.sort()
    print(f"[생성] {want:,} · 종류 {dict(kinds.most_common())} · 말 길이 중앙 {L[len(L)//2]} / 90% {L[int(len(L)*.9)]} / 최대 {L[-1]}",
          file=sys.stderr)

if __name__ == "__main__":
    main()
