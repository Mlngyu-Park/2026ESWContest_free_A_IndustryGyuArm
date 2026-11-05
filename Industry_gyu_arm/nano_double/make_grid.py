#!/usr/bin/env python3
"""말씨 격자 시험 세트 — 동사·어미를 하나씩 바꿔 끼워 어디서 틀리는지 본다 (10-30).

학습 생성기(gen_norm·gen_double)와 따로 손으로 쓴 활용표로 만든다. 정답은 틀에서 정해진다.
  · 잡기 말씨 (G): 잡아서 · 갖고 · 가지고 · 들고 · 꺼내서 · 뽑아 가지고 …
  · 옮기기 동사 (M): 옮기다 · 갖다 놓다 · 가져다 두다 · 꽂다 · 끼우다 · 가서 내려놓다 …
  · 연결 어미 (C): -고 · -고 나서 · -ㄴ 다음에 · -어서 · -었다가 · -어 가지고 · -자마자 · -면 …
  · 맺음 어미 (E): -어 · -어 줘 · -어 주세요 · -ㄹ래? · -ㄹ 수 있어? · -어라 · -ㅂ시다 · -도록 해 …
각 말씨마다 팔 꼴 셋: 한 팔(팔 이름 없음) · 한 팔(이름 있음) · 두 팔(넘겨받기).

  python make_grid.py > grid_eval.json
"""
import json, random, sys, os
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
from canon import CANON_C

random.seed(1030)
COLORS = ["빨강", "주황", "노랑", "초록", "파랑", "보라"]
SAY = {"빨강": "빨간색", "주황": "주황색", "노랑": "노란색", "초록": "초록색", "파랑": "파란색", "보라": "보라색"}
PLACE = lambda c: random.choice([SAY[c], SAY[c] + " 도킹", SAY[c] + " 도킹 스테이션"])

def ro(w):                                   # 으로/로
    ch = w[-1]; o = ord(ch) - 0xAC00
    if not (0 <= o <= 11171): return w + "로"
    j = o % 28
    return w + ("로" if j in (0, 8) else "으로")

# 옮기기 동사 활용표 — a: -어 꼴, adn: -ㄴ(과거 관형), go: -고, myeon: -면, neun: -는, ul: -ㄹ, gi: -기(이름꼴),
#   stem: 줄기(-도록·-자마자), ja: -자, sip: -ㅂ시다, daga: -었다가, jo: 목적지 조사 (ro=으로, e=에)
M = {
 "옮기다":     dict(a="옮겨", adn="옮긴", go="옮기고", myeon="옮기면", neun="옮기는", ul="옮길", gi="옮기기", stem="옮기", ja="옮기자", sip="옮깁시다", daga="옮겼다가", jo="ro"),
 "옮겨 놓다":  dict(a="옮겨 놔", adn="옮겨 놓은", go="옮겨 놓고", myeon="옮겨 놓으면", neun="옮겨 놓는", ul="옮겨 놓을", gi="옮겨 놓기", stem="옮겨 놓", ja="옮겨 놓자", sip="옮겨 놓읍시다", daga="옮겨 놨다가", jo="e"),
 "갖다 놓다":  dict(a="갖다 놔", adn="갖다 놓은", go="갖다 놓고", myeon="갖다 놓으면", neun="갖다 놓는", ul="갖다 놓을", gi="갖다 놓기", stem="갖다 놓", ja="갖다 놓자", sip="갖다 놓읍시다", daga="갖다 놨다가", jo="e"),
 "가져다 놓다": dict(a="가져다 놔", adn="가져다 놓은", go="가져다 놓고", myeon="가져다 놓으면", neun="가져다 놓는", ul="가져다 놓을", gi="가져다 놓기", stem="가져다 놓", ja="가져다 놓자", sip="가져다 놓읍시다", daga="가져다 놨다가", jo="e"),
 "가져다 두다": dict(a="가져다 둬", adn="가져다 둔", go="가져다 두고", myeon="가져다 두면", neun="가져다 두는", ul="가져다 둘", gi="가져다 두기", stem="가져다 두", ja="가져다 두자", sip="가져다 둡시다", daga="가져다 뒀다가", jo="e"),
 "갖다 꽂다":  dict(a="갖다 꽂아", adn="갖다 꽂은", go="갖다 꽂고", myeon="갖다 꽂으면", neun="갖다 꽂는", ul="갖다 꽂을", gi="갖다 꽂기", stem="갖다 꽂", ja="갖다 꽂자", sip="갖다 꽂읍시다", daga="갖다 꽂았다가", jo="e"),
 "놓다":       dict(a="놔", adn="놓은", go="놓고", myeon="놓으면", neun="놓는", ul="놓을", gi="놓기", stem="놓", ja="놓자", sip="놓읍시다", daga="놨다가", jo="e"),
 "두다":       dict(a="둬", adn="둔", go="두고", myeon="두면", neun="두는", ul="둘", gi="두기", stem="두", ja="두자", sip="둡시다", daga="뒀다가", jo="e"),
 "꽂다":       dict(a="꽂아", adn="꽂은", go="꽂고", myeon="꽂으면", neun="꽂는", ul="꽂을", gi="꽂기", stem="꽂", ja="꽂자", sip="꽂읍시다", daga="꽂았다가", jo="e"),
 "넣다":       dict(a="넣어", adn="넣은", go="넣고", myeon="넣으면", neun="넣는", ul="넣을", gi="넣기", stem="넣", ja="넣자", sip="넣읍시다", daga="넣었다가", jo="e"),
 "끼우다":     dict(a="끼워", adn="끼운", go="끼우고", myeon="끼우면", neun="끼우는", ul="끼울", gi="끼우기", stem="끼우", ja="끼우자", sip="끼웁시다", daga="끼웠다가", jo="e"),
 "올려놓다":   dict(a="올려놔", adn="올려놓은", go="올려놓고", myeon="올려놓으면", neun="올려놓는", ul="올려놓을", gi="올려놓기", stem="올려놓", ja="올려놓자", sip="올려놓읍시다", daga="올려놨다가", jo="e"),
 "내려놓다":   dict(a="내려놔", adn="내려놓은", go="내려놓고", myeon="내려놓으면", neun="내려놓는", ul="내려놓을", gi="내려놓기", stem="내려놓", ja="내려놓자", sip="내려놓읍시다", daga="내려놨다가", jo="e"),
 "보내다":     dict(a="보내", adn="보낸", go="보내고", myeon="보내면", neun="보내는", ul="보낼", gi="보내기", stem="보내", ja="보내자", sip="보냅시다", daga="보냈다가", jo="ro"),
 "이동하다":   dict(a="이동해", adn="이동한", go="이동하고", myeon="이동하면", neun="이동하는", ul="이동할", gi="이동하기", stem="이동하", ja="이동하자", sip="이동합시다", daga="이동했다가", jo="ro"),
 "이동시키다": dict(a="이동시켜", adn="이동시킨", go="이동시키고", myeon="이동시키면", neun="이동시키는", ul="이동시킬", gi="이동시키기", stem="이동시키", ja="이동시키자", sip="이동시킵시다", daga="이동시켰다가", jo="ro"),
 "가져가다":   dict(a="가져가", adn="가져간", go="가져가고", myeon="가져가면", neun="가져가는", ul="가져갈", gi="가져가기", stem="가져가", ja="가져가자", sip="가져갑시다", daga="가져갔다가", jo="ro"),
 "가서 내려놓다": dict(a="가서 내려놔", adn="가서 내려놓은", go="가서 내려놓고", myeon="가서 내려놓으면", neun="가서 내려놓는", ul="가서 내려놓을", gi="가서 내려놓기", stem="가서 내려놓", ja="가서 내려놓자", sip="가서 내려놓읍시다", daga="가서 내려놨다가", jo="ro"),
 "가져가서 꽂다": dict(a="가져가서 꽂아", adn="가져가서 꽂은", go="가져가서 꽂고", myeon="가져가서 꽂으면", neun="가져가서 꽂는", ul="가져가서 꽂을", gi="가져가서 꽂기", stem="가져가서 꽂", ja="가져가서 꽂자", sip="가져가서 꽂읍시다", daga="가져가서 꽂았다가", jo="ro"),
}
# 잡기 말씨 (펜을 ___ 다음 자리로)
G = ["잡아서", "잡고", "잡아다가", "잡아 가지고", "잡아 갖고", "집어서", "집고", "집어 들고", "집어다가", "들고", "들어서",
     "빼서", "빼 가지고", "빼다가", "꺼내서", "꺼내 가지고", "꺼내다가", "뽑아서", "뽑아 가지고", "쥐고", "가지고", "갖고",
     "챙겨서", "잡은 다음에", "잡은 후에", "잡고 나서", "집은 뒤에", "들어 올려서", "(없음)"]
# 연결 어미 (앞 단계 끝 → 다음 단계)
C = {"-고": "{go}", "-고 나서": "{go} 나서", "-고서": "{go}서", "-ㄴ 다음에": "{adn} 다음에", "-ㄴ 다음": "{adn} 다음",
     "-ㄴ 후에": "{adn} 후에", "-ㄴ 뒤에": "{adn} 뒤에", "-어서": "{a}서", "-었다가": "{daga}", "-어 놓고": "{a} 놓고",
     "-어 가지고": "{a} 가지고", "-자마자": "{stem}자마자", "-면": "{myeon}", "-고 그 다음에": "{go} 그 다음에",
     "-어. 그리고": "{a}. 그리고", "-어. 그 다음에": "{a}. 그 다음에", "-어 줘. 그러고 나서": "{a} 줘. 그러고 나서",
     "-ㄴ 다음 다시": "{adn} 다음 다시", "-고 이어서": "{go} 이어서", "-고 또": "{go} 또"}
# 맺음 어미
E = {"-어": "{a}", "-어 줘": "{a} 줘", "-어줘": "{a}줘", "-어 주세요": "{a} 주세요", "-어 줘요": "{a} 줘요", "-어요": "{a}요",
     "-어 줄래?": "{a} 줄래?", "-어 줄 수 있어?": "{a} 줄 수 있어?", "-어 주시겠어요?": "{a} 주시겠어요?", "-어 주라": "{a} 주라",
     "-어라": "{a}라", "-어 봐": "{a} 봐", "-자": "{ja}", "-ㅂ시다": "{sip}", "-ㄹ래?": "{ul}래?", "-ㄹ 수 있어?": "{ul} 수 있어?",
     "-면 좋겠어": "{myeon} 좋겠어", "-면 돼": "{myeon} 돼", "-고 싶어": "{go} 싶어", "-는 거 해줘": "{neun} 거 해줘",
     "-는 거야": "{neun} 거야", "-어야 돼": "{a}야 돼", "-ㄹ 거야": "{ul} 거야", "-기 바래": "{gi} 바래",
     "-도록 해": "{stem}도록 해", "-기": "{gi}", "-어 주면 돼": "{a} 주면 돼", "-는 거 하고 싶어": "{neun} 거 하고 싶어"}

# ── 세트2 (GRID_SET=2): 학습 생성기에 넣지 않는 말씨만 — 학습 뒤 '처음 보는 말씨' 로 다시 잴 때 쓴다 ──
def _v(a, adn, go, myeon, neun, ul, gi, stem, ja, sip, daga, jo):
    return dict(a=a, adn=adn, go=go, myeon=myeon, neun=neun, ul=ul, gi=gi, stem=stem, ja=ja, sip=sip, daga=daga, jo=jo)
M2 = {
 "꽂아 넣다": _v("꽂아 넣어", "꽂아 넣은", "꽂아 넣고", "꽂아 넣으면", "꽂아 넣는", "꽂아 넣을", "꽂아 넣기", "꽂아 넣", "꽂아 넣자", "꽂아 넣읍시다", "꽂아 넣었다가", "e"),
 "옮겨다 놓다": _v("옮겨다 놔", "옮겨다 놓은", "옮겨다 놓고", "옮겨다 놓으면", "옮겨다 놓는", "옮겨다 놓을", "옮겨다 놓기", "옮겨다 놓", "옮겨다 놓자", "옮겨다 놓읍시다", "옮겨다 놨다가", "e"),
 "집어넣다": _v("집어넣어", "집어넣은", "집어넣고", "집어넣으면", "집어넣는", "집어넣을", "집어넣기", "집어넣", "집어넣자", "집어넣읍시다", "집어넣었다가", "e"),
 "갖다 두다": _v("갖다 둬", "갖다 둔", "갖다 두고", "갖다 두면", "갖다 두는", "갖다 둘", "갖다 두기", "갖다 두", "갖다 두자", "갖다 둡시다", "갖다 뒀다가", "e"),
 "옮겨 두다": _v("옮겨 둬", "옮겨 둔", "옮겨 두고", "옮겨 두면", "옮겨 두는", "옮겨 둘", "옮겨 두기", "옮겨 두", "옮겨 두자", "옮겨 둡시다", "옮겨 뒀다가", "e"),
 "가져다주다": _v("가져다줘", "가져다준", "가져다주고", "가져다주면", "가져다주는", "가져다줄", "가져다주기", "가져다주", "가져다주자", "가져다줍시다", "가져다줬다가", "e"),
 "가서 꽂다": _v("가서 꽂아", "가서 꽂은", "가서 꽂고", "가서 꽂으면", "가서 꽂는", "가서 꽂을", "가서 꽂기", "가서 꽂", "가서 꽂자", "가서 꽂읍시다", "가서 꽂았다가", "ro"),
}
G2 = ["움켜쥐고", "붙잡고", "빼내서", "뽑아 들고", "꺼내 들고", "집어 올려서", "잡아 올려서", "쥐어서", "잡아다", "집어 가지고"]
C2 = {"-고는": "{go}는", "-ㄴ 다음엔": "{adn} 다음엔", "-고 난 뒤에": "{go} 난 뒤에", "-고 나면": "{go} 나면", "-ㄴ 담에": "{adn} 담에",
      "-ㄴ 다음 거기서": "{adn} 다음 거기서", "-고 바로": "{go} 바로", "-어. 그러고": "{a}. 그러고", "-ㄴ 후 곧바로": "{adn} 후 곧바로"}
E2 = {"-어 주실래요?": "{a} 주실래요?", "-어 주겠니?": "{a} 주겠니?", "-어 줘라": "{a} 줘라", "-어 줘 봐": "{a} 줘 봐",
      "-ㄹ 수 있을까?": "{ul} 수 있을까?", "-어 주면 좋겠어": "{a} 주면 좋겠어", "-어 주길 바래": "{a} 주길 바래",
      "-어 줄 수 있나요?": "{a} 줄 수 있나요?", "-어 주십시오": "{a} 주십시오", "-는 걸로 해": "{neun} 걸로 해",
      "-ㄹ까?": "{ul}까?", "-어 줄래요?": "{a} 줄래요?", "-어 주렴": "{a} 주렴"}
# ── 세트3 (GRID_SET=3, 10-31): 7차 생성기를 고치기 전에 먼저 쓴 시험지 — 7차 생성기에 넣지 않는다 ──
#   C: 말할 때 '고→구' · '담엔' · '거기서' 꼴 · '걸 다시' …  (모르는 잇는 말이면 뒤 단계를 버리는지)
#   F: 말 중간에 끼는 말 ('펜 있잖아 그거') · S: 음성 인식 모양 (띄어쓰기 없음·도킨·토킹·팬)
#   L: 자리만 나열 ('노랑으로 노랑에서 파랑으로') · W: 팔 이름 없는 한 팔 + 기다리기
def _gu(go): return go[:-1] + "구"
C3 = {"-구": "{gu}", "-구 나서": "{gu} 나서", "-구서": "{gu}서", "-ㄴ 담엔": "{adn} 담엔", "-어. 그 담엔": "{a}. 그 담엔",
      "-고 나선": "{go} 나선", "-고 난 다음": "{go} 난 다음", "-ㄴ 걸 다시": "{adn} 걸 다시", "-고 거기서": "{go} 거기서",
      "-ㄴ 다음 그 자리에서": "{adn} 다음 그 자리에서", "-어 놓고서": "{a} 놓고서", "-ㄴ 다음엔 또": "{adn} 다음엔 또"}
F3 = {"있잖아 그거": "{p}에 있는 펜 있잖아 그거 ", "말이야 그거": "{p}에 꽂힌 펜 말이야 그거 ",
      "그거 말이야": "{p}에 있는 펜 그거 말이야 ", "그러니까": "음 그러니까 {p}에 있는 펜을 "}
S3 = {"띄어쓰기 없음": None, "도킹→도킨": ("도킹", "도킨"), "도킹→토킹": ("도킹", "토킹"), "펜→팬": ("펜", "팬"),
      "스테이션→스테이숀": ("스테이션", "스테이숀"), "-ㄴ다음에 붙여 씀": None}
L3 = ["동사 없음", "쉼표 나열", "찍고"]
W3 = ["기다렸다가 다시", "있다가", "쉬었다가"]
if os.environ.get("GRID_SET") == "2":
    M.update(M2); G, C, E = G2, {**C, **C2}, {**E, **E2}
    TEST_M, TEST_C, TEST_E = list(M2), list(C2), list(E2)
    random.seed(2030)
elif os.environ.get("GRID_SET") == "3":
    for v in M.values(): v["gu"] = _gu(v["go"])
    C = {**C, **C3}
    TEST_M, TEST_C, TEST_E = [], list(C3), []
    G = []
    random.seed(3031)
else:
    TEST_M, TEST_C, TEST_E = list(M), list(C), list(E)

def fill(t, v): return t.format(**v)

def dest(c, v):
    p = PLACE(c)
    return ro(p) if v["jo"] == "ro" else p + random.choice(["에", "에다", "에다가"])

def canon_seg(start, stops, pen_col):
    pen = f"{pen_col} 펜" if pen_col else "펜"
    parts = [f"{CANON_C[start]} 도킹의 {pen}을 " + ro(CANON_C[stops[0]])]
    for s in stops[1:]: parts.append(ro(CANON_C[s]))
    return " 옮긴 다음 ".join(parts) + " 옮겨"

ARM_SAY = {"A": ["빨간 팔", "빨간 로봇팔", "A팔"], "B": ["파란 팔", "파란 로봇팔", "B팔"]}

def subj(L):
    a = random.choice(ARM_SAY[L])
    j = (ord(a[-1]) - 0xAC00) % 28 if 0 <= ord(a[-1]) - 0xAC00 <= 11171 else 0
    return random.choice([a + ("이" if j else "가"), ro(a), a + ("은" if j else "는")])

def build(factor, val, mode):
    """factor 하나만 val 로 바꾸고 나머지는 기본값. mode: 한팔 · 한팔이름 · 두팔."""
    g = val if factor == "G" else "잡아서"
    mv = M[val] if factor == "M" else M["옮기다"]
    cf = C[val] if factor == "C" else C["-ㄴ 다음에"]
    ef = E[val] if factor == "E" else E["-어 줘"]
    x, y, z = random.sample(COLORS, 3)
    pen_col = random.choice([None, None, "초록"])
    pen_say = f"{SAY[pen_col]} 펜" if pen_col else "펜"
    grab = "" if g == "(없음)" else g + " "
    head = f"{PLACE(x)}에 있는 {pen_say}{'을' if True else ''} {grab}"
    if mode in ("한팔", "한팔이름"):
        two_step = factor == "C" or random.random() < 0.5
        if two_step:
            t = head + dest(y, mv) + " " + fill(cf, mv) + " " + dest(z, mv) + " " + fill(ef, mv)
            stops = [y, z]
        else:
            t = head + dest(y, mv) + " " + fill(ef, mv); stops = [y]
        canon = canon_seg(x, stops, pen_col)
        if mode == "한팔이름":
            t = subj("A") + " " + t; canon = "A팔: " + canon
        scene = "팔 A=빨강" if mode == "한팔" and random.random() < 0.5 else "팔 A=빨강 B=파랑"
    else:
        hand = random.choice(["그걸 받아서", "그거 받아서", f"{PLACE(y)}에서 {g if g != '(없음)' else '잡아서'}"])
        t = (subj("A") + " " + head + dest(y, mv) + " " + fill(cf, mv) + " " + subj("B") + " " + hand + " " +
             dest(z, mv) + " " + fill(ef, mv))
        canon = "A팔: " + canon_seg(x, [y], pen_col) + " / B팔: " + canon_seg(y, [z], None)
        scene = "팔 A=빨강 B=파랑"
    docks = " ".join(f"{c}=d{i}" for c, i in zip(["보라", "빨강", "주황", "노랑", "파랑", "초록"], [2, 10, 11, 12, 13, 14]))
    home = {"보라": 2, "빨강": 10, "주황": 11, "노랑": 12, "파랑": 13, "초록": 14}[x]
    scene = f"{scene} | {docks} | 펜=초록@d{home}"
    return dict(text=" ".join(t.split()), canon=canon, scene=scene, factor=factor, form=val, mode=mode)

WAIT3 = {"기다렸다가 다시": "{n} 기다렸다가 다시", "있다가": "{n} 있다가", "쉬었다가": "{n} 쉬었다가"}
N3 = {10: ["10초", "십 초"], 30: ["30초", "삼십 초"], 5: ["5초", "오 초"]}

def build3(factor, val, mode):
    """세트3 의 F·S·L·W (C 는 build 그대로)."""
    if factor == "S":
        for _ in range(200):
            it = build("C", "-ㄴ 다음에", mode); t = it["text"]
            if val == "띄어쓰기 없음":
                t = "".join(w + ("" if random.random() < 0.5 else " ") for w in t.split()).strip()
            elif val == "-ㄴ다음에 붙여 씀":
                if " 다음에" not in t: continue
                t = t.replace(" 다음에", "다음에")
            else:
                a, b = S3[val]
                if a not in t: continue
                t = t.replace(a, b)
            it.update(text=t, factor="S", form=val); return it
        raise RuntimeError(val)
    x, y, z = random.sample(COLORS, 3)
    mv = M["옮기다"]
    ef = fill(random.choice([E["-어 줘"], E["-어"], E["-어 주세요"]]), mv)
    if factor == "F":
        head = F3[val].format(p=PLACE(x))
        if random.random() < 0.5:
            t = head + dest(y, mv) + " " + fill(C["-ㄴ 다음에"], mv) + " " + dest(z, mv) + " " + ef; stops = [y, z]
        else:
            t = head + dest(y, mv) + " " + ef; stops = [y]
        canon = canon_seg(x, stops, None)
    elif factor == "L":
        w = random.choice([c for c in COLORS if c not in (x, y, z)])
        if val == "동사 없음":
            stops = [y, z] + ([w] if random.random() < 0.5 else [])
            t = f"{PLACE(x)}에 있는 펜 {ro(PLACE(stops[0]))}"
            for a, b in zip(stops, stops[1:]): t += f" {PLACE(a)}에서 {ro(PLACE(b))}"
        elif val == "쉼표 나열":
            stops = [y, z]; t = f"{PLACE(x)}에서 {PLACE(y)}, {PLACE(y)}에서 {ro(PLACE(z))} {ef}"
        else:
            stops = [y, z]; t = f"{PLACE(x)}에 있는 펜을 {PLACE(y)} 찍고 {ro(PLACE(z))} {ef}"
        canon = canon_seg(x, stops, None)
    else:   # W
        n = random.choice(list(N3)); z = random.choice([x, z])
        t = (f"{PLACE(x)}에 있는 펜 {PLACE(y)}에 꽂고 " + WAIT3[val].format(n=random.choice(N3[n])) +
             f" {ro(PLACE(z))} {ef}")
        canon = canon_seg(x, [y], None) + f" / {n}초 기다려 / " + canon_seg(y, [z], None)
    if mode == "한팔이름":
        t = subj("A") + " " + t
        canon = " / ".join(p if "기다려" in p else "A팔: " + p for p in canon.split(" / "))
    scene = "팔 A=빨강" if mode == "한팔" and random.random() < 0.5 else "팔 A=빨강 B=파랑"
    docks = " ".join(f"{c}=d{i}" for c, i in zip(["보라", "빨강", "주황", "노랑", "파랑", "초록"], [2, 10, 11, 12, 13, 14]))
    home = {"보라": 2, "빨강": 10, "주황": 11, "노랑": 12, "파랑": 13, "초록": 14}[x]
    return dict(text=" ".join(t.split()), canon=canon, scene=f"{scene} | {docks} | 펜=초록@d{home}",
                factor=factor, form=val, mode=mode)

def main():
    items = []
    if os.environ.get("GRID_SET") == "3":
        for v in C3:
            for mode in ("한팔", "한팔", "한팔이름", "두팔"): items.append(build("C", v, mode))
        for v in S3:
            for mode in ("한팔", "한팔", "한팔이름", "두팔"): items.append(build3("S", v, mode))
        for factor, vals in (("F", F3), ("L", L3), ("W", W3)):
            for v in vals:
                for mode in ("한팔", "한팔", "한팔", "한팔이름") * 2: items.append(build3(factor, v, mode))
        json.dump(dict(note="말씨 격자 세트3 — 7차 생성기 전에 쓴 시험지 (10-31)", items=items), sys.stdout, ensure_ascii=False, indent=0)
        print(f"{len(items)}문장", file=sys.stderr); return
    for factor, vals in (("G", G), ("M", TEST_M), ("C", TEST_C), ("E", TEST_E)):
        for v in vals:
            for mode in ("한팔", "한팔", "한팔이름", "두팔"):
                items.append(build(factor, v, mode))
    json.dump(dict(note=__doc__.strip().split("\n")[0], items=items), sys.stdout, ensure_ascii=False, indent=0)
    print(f"{len(items)}문장", file=sys.stderr)

if __name__ == "__main__":
    main()
