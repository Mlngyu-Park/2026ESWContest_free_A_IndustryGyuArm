#!/usr/bin/env python3
"""팔 이름 시험지 (11-03) — 8차 생성기를 고치기 전에 먼저 쓴다.

계기: 실제 말 "비팔로 피피에이피를 잡아서 …" 를 A팔로 바꿈 (음성 인식이 B팔을 '비팔'·'B8팔' 로 적는다).
팔 부르는 꼴 × 조사 × 자리(앞·중간·끝) × 팔 꼴(한 팔 A/B · 두 팔 A→B / B→A) + 반례 + 실제 말.
  · 학습한 꼴: B팔 · 파란 팔
  · 8차에 넣을 꼴: 비팔 · 비 팔 · 비 로봇 · 비 로봇팔 · B8 · 비8 · B8팔 · B 8 · 비암 · B 암
  · 처음 보는 꼴 (8차 생성기에 넣지 않는다): 소문자 b팔 · 로봇 비 · 둘째 팔 · 조사 '을 써서' · '한테 시켜서'
  · 반례: 감탄사 '에이' (팔 아님) · 'B팔로 8번 도킹' (8 = 도킹 번호)

  python make_grid_arm.py > grid_eval_arm.json
"""
import json, os, random, sys
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import make_grid as MG

random.seed(1103)
COLORS = MG.COLORS
KOR = {"A": "에이", "B": "비"}
ORD = {"A": "첫째", "B": "둘째"}
CSAY = {"빨강": "빨간", "주황": "주황색", "노랑": "노란", "초록": "초록색", "파랑": "파란", "보라": "보라색"}

# 꼴 이름 → (L, 팔 색) -> 부르는 말,  학습 여부
FORMS = {
    "B팔":       (lambda L, c: f"{L}팔", "학습함"),
    "파란 팔":   (lambda L, c: f"{CSAY[c]} 팔", "학습함"),
    "비팔":      (lambda L, c: f"{KOR[L]}팔", "8차"),
    "비 팔":     (lambda L, c: f"{KOR[L]} 팔", "8차"),
    "비 로봇":   (lambda L, c: f"{KOR[L]} 로봇", "8차"),
    "비 로봇팔": (lambda L, c: f"{KOR[L]} 로봇팔", "8차"),
    "B8":        (lambda L, c: f"{L}8", "8차"),
    "비8":       (lambda L, c: f"{KOR[L]}8", "8차"),
    "B8팔":      (lambda L, c: f"{L}8팔", "8차"),
    "B 8":       (lambda L, c: f"{L} 8", "8차"),
    "비암":      (lambda L, c: f"{KOR[L]}암", "8차"),
    "B 암":      (lambda L, c: f"{L} 암", "8차"),
    "소문자 b팔": (lambda L, c: f"{L.lower()}팔", "처음"),
    "로봇 비":   (lambda L, c: f"로봇 {KOR[L]}", "처음"),
    "둘째 팔":   (lambda L, c: f"{ORD[L]} 팔", "처음"),
}

def jong(s):
    """받침 (영어 글자 A·B 는 읽으면 '에이'·'비' 라 받침 없음, 8 = 팔 = ㄹ)."""
    c = s.rstrip()[-1]
    o = ord(c) - 0xAC00
    if 0 <= o < 11172: return o % 28
    if c == "8": return 8
    return 0

def pj(s, kind):
    j = jong(s)
    return {"이": s + ("이" if j else "가"), "은": s + ("은" if j else "는"), "로": s + ("로" if j in (0, 8) else "으로"),
            "가지고": s + random.choice([" 가지고", "가지고"]), "없음": s,
            "을 써서": s + ("을" if j else "를") + " 써서", "한테 시켜서": s + "한테 시켜서"}[kind]

PART_SEEN = ["이", "은", "로", "가지고", "없음"]

def scene(arms, x, nums=()):
    docks = " ".join(f"{c}=d{i}" for c, i in zip(["보라", "빨강", "주황", "노랑", "파랑", "초록"], [2, 10, 11, 12, 13, 14]))
    home = {"보라": 2, "빨강": 10, "주황": 11, "노랑": 12, "파랑": 13, "초록": 14}.get(x, x)
    a = " ".join(f"{k}={v}" for k, v in arms.items())
    return f"팔 {a} | {docks} | 펜=초록@d{home}"

def pick_arms():
    return random.choice([{"A": "빨강", "B": "파랑"}, {"A": "빨강", "B": "파랑"}, {"A": "초록", "B": "노랑"}, {"A": "보라", "B": "주황"}])

def one_seg(x=None):
    """한 팔 몸통: (출발 구, 나머지, 정리문)."""
    x2, y, z = random.sample(COLORS, 3)
    x = x or x2
    if y == x or z == x: y, z = [c for c in COLORS if c != x][:2]
    pen_col = random.choice([None, None, "초록"])
    pen_say = f"{MG.SAY[pen_col]} 펜" if pen_col else "펜"
    mv = MG.M[random.choice(["옮기다", "갖다 놓다", "꽂다", "옮겨 놓다", "가져다 두다"])]
    grab = random.choice(["잡아서", "집어서", "들고", "잡은 다음에", ""])
    two = random.random() < 0.5
    rest = (grab + " " if grab else "") + MG.dest(y, mv)
    if two: rest += " " + MG.fill(MG.C[random.choice(["-ㄴ 다음에", "-고", "-었다가"])], mv) + " " + MG.dest(z, mv)
    rest += " " + MG.fill(MG.E[random.choice(["-어 줘", "-어", "-어 주세요", "-고 싶어", "-어 줄래?"])], mv)
    src = f"{MG.PLACE(x)}에 있는 {pen_say}을"
    return src, rest, MG.canon_seg(x, [y, z] if two else [y], pen_col), x

def single(form, L, pos=None, part=None):
    arms = pick_arms(); say = FORMS[form][0](L, arms[L])
    src, rest, canon, x = one_seg()
    pos = pos or random.choice(["앞", "앞", "중간", "끝"])
    if pos == "앞": t = f"{pj(say, part or random.choice(PART_SEEN))} {src} {rest}"
    elif pos == "중간": t = f"{src} {pj(say, random.choice(['로', '가지고']))} {rest}"
    else: t = f"{src} {rest} {pj(say, '로')}{random.choice(['', '요', ' 해 줘'])}"
    return dict(text=" ".join(t.split()), canon=f"{L}팔: " + canon, scene=scene(arms, x), pos=pos)

def double(form, first):
    arms = pick_arms(); L1, L2 = first, ("B" if first == "A" else "A")
    s1, s2 = FORMS[form][0](L1, arms[L1]), FORMS[form][0](L2, arms[L2])
    x, y, z = random.sample(COLORS, 3)
    pen_col = random.choice([None, "초록"])
    pen_say = f"{MG.SAY[pen_col]} 펜" if pen_col else "펜"
    mv = MG.M[random.choice(["옮기다", "갖다 놓다", "꽂다"])]
    hand = random.choice(["그걸 받아서", "그거 받아서", f"{MG.PLACE(y)}에서 잡아서"])
    conn = MG.fill(MG.C[random.choice(["-ㄴ 다음에", "-면", "-고 나서"])], mv)
    t = (f"{pj(s1, random.choice(['이', '로', '가지고']))} {MG.PLACE(x)}에 있는 {pen_say}을 잡아서 {MG.dest(y, mv)} {conn} "
         f"{pj(s2, random.choice(['이', '은', '로']))} {hand} {MG.dest(z, mv)} {MG.fill(MG.E['-어 줘'], mv)}")
    canon = f"{L1}팔: " + MG.canon_seg(x, [y], pen_col) + f" / {L2}팔: " + MG.canon_seg(y, [z], None)
    return dict(text=" ".join(t.split()), canon=canon, scene=scene(arms, x), pos="앞")

def main():
    items = []
    for form, (_, seen) in FORMS.items():
        for mode, mk in (("한팔B", lambda: single(form, "B")), ("한팔B", lambda: single(form, "B")), ("한팔B", lambda: single(form, "B")),
                         ("한팔A", lambda: single(form, "A")), ("한팔A", lambda: single(form, "A")),
                         ("두팔AB", lambda: double(form, "A")), ("두팔BA", lambda: double(form, "B"))):
            it = mk(); it.update(factor="팔꼴", form=f"{form} ({seen})", mode=mode); items.append(it)
    for part in ["이", "은", "로", "가지고", "없음", "을 써서", "한테 시켜서"]:
        seen = "학습함" if part in PART_SEEN else "처음"
        for L in "BBBA":
            form = random.choice(["비팔", "비 팔", "B8", "B팔", "비 로봇"])
            it = single(form, L, pos="앞", part=part)
            it.update(factor="조사", form=f"{part} ({seen})", mode="한팔" + L); items.append(it)
    for _ in range(8):                                      # 감탄사 '에이' — 팔이 아니다 (정리문에 팔 없음)
        arms = pick_arms(); src, rest, canon, x = one_seg()
        t = random.choice(["에이 그냥 ", "에이, ", "아 에이 몰라 그냥 ", "에이 그럼 "]) + f"{src} {rest}"
        items.append(dict(text=" ".join(t.split()), canon=canon, scene=scene(arms, x), pos="-",
                          factor="반례", form="감탄사 에이 (팔 아님)", mode="반례"))
    for _ in range(8):                                      # 'B팔로 8번 도킹' — 8 은 도킹 번호
        arms = pick_arms(); L = random.choice("AB")
        say = FORMS[random.choice(["B팔", "비팔", "비 팔"])][0](L, arms[L])
        y = random.choice(COLORS); mv = MG.M["옮기다"]
        t = f"{pj(say, '로')} 8번 도킹에 있는 펜을 잡아서 {MG.dest(y, mv)} {MG.fill(MG.E['-어 줘'], mv)}"
        items.append(dict(text=" ".join(t.split()), canon=f"{L}팔: 8번 도킹의 펜을 " + MG.ro(MG.CANON_C[y]) + " 옮겨",
                          scene=scene(arms, 8), pos="앞", factor="반례", form="8번 도킹 + 팔", mode="한팔" + L))
    real_sc = "팔 A=빨강 B=파랑 | 보라=d2 빨강=d10 주황=d11 | 펜=초록@d2"   # 11-03 그때 배치 (ppap = 보라 위)
    for t, c in [("내가 하고 싶은 건 비팔로 피피에이피를 잡아서 주황색에 옮겼다가 다시 보라색으로 옮기는 걸 하고 싶어",
                  "B팔: 펜을 주황색으로 옮긴 다음 보라색으로 옮겨"),
                 ("내가 하고 싶은 건 B8팔가지고 PPAP를 잡은 다음에 주황색 도킹 스테이션을 옮기는 걸 하고 싶어",
                  "B팔: 펜을 주황색으로 옮겨")]:
        items.append(dict(text=t, canon=c, scene=real_sc, pos="-", factor="실제", form="11-03 실제 말", mode="한팔B"))
    json.dump(dict(note="팔 이름 시험지 (11-03) — 8차 생성기 전에 씀", items=items), sys.stdout, ensure_ascii=False, indent=0)
    print(f"{len(items)}문장", file=sys.stderr)

if __name__ == "__main__":
    main()
