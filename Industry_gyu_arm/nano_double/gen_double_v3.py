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
def arm_say(letter, color, idx):
    return random.choices([
        f"{random.choice(ARM_C_SAY[color])} 팔", f"{random.choice(ARM_C_SAY[color])} 로봇팔",
        f"{random.choice(ARM_C_SAY[color])} 로봇 팔", f"{random.choice(ARM_C_SAY[color])}팔",
        f"{letter}팔", f"{letter} 팔", f"로봇 {letter}", f"{letter}번 팔" if False else f"{letter} 로봇",
        f"{idx + 1}번 팔", ["첫 번째 팔", "두 번째 팔", "세 번째 팔"][idx], f"{idx + 1}번 로봇팔"],
        weights=[22, 10, 4, 4, 10, 5, 3, 2, 8, 3, 3])[0]

def arm_subj(s):
    """팔 + 조사 (이/가/은/는/로/(없음))."""
    j = G._jong(s)
    return random.choices([s + ("이" if j else "가"), s + ("은" if j else "는"), G.ro(s), s,
                           s + ("이" if j else "가") + " 먼저"],
                          weights=[40, 15, 25, 10, 3])[0]

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
IGNORE_ACT = ["베이스 자세로 돌아와서", "그 후 다시 베이스 자세로 돌아와서", "기본 자세로 돌아갔다가",
              "원래 자세로 돌아와서 그 이후에", "처음 자세로 돌아왔다가", "베이스로 갔다가"]
I_DU = next(i for i, v in enumerate(G.VERBS) if v["adn"] == "둔")

def dest2(v, p):
    if V2 and random.random() < 0.04: return G.eul(G.place_say(p))      # 조사 실수
    return G.dest(v, p)

def seg_body(start, nsteps, handoff, pen, last_seg):
    """말 몸통과 (stops) — 끝은 이어지는 꼴(마지막 구간이면 {END}|v)."""
    stops = []; prev = start
    for _ in range(nsteps):
        if V2 and prev and random.random() < 0.06: p = prev                # 같은 자리에 다시
        else: p = rand_place(exclude=prev)
        stops.append(p); prev = p
    fromto = V2 and start and not handoff and random.random() < 0.15        # "보라색에서 노란색으로 펜을 옮기고"
    if handoff:
        first = random.choice(["그거 받아서 ", "그걸 받아서 ", "받아서 ", "이어받아서 ", "그 펜을 받아서 ",
                               "거기 있는 펜 잡아서 ", f"{G.place_say(start)}에 있는 {G.eul(pen)} 잡아서 ",
                               f"{G.place_say(start)}에서 {pen} 받아서 ", "그거 다시 잡아서 ", "그걸 "])
    elif fromto:
        first = f"{G.place_say(start)}에서 "
    elif start:
        first = random.choice([f"{G.place_say(start)}에 있는 {G.eul(pen)} {random.choice(G.GRAB)} ",
                               f"{G.place_say(start)}에 있는 {pen} {random.choice(G.GRAB)} ",
                               f"{G.place_say(start)}에서 {G.eul(pen)} {random.choice(G.GRAB)} ",
                               f"{G.place_say(start)}의 {G.eul(pen)} "])
    elif V2 and random.random() < 0.10:
        first = ""                                                          # 펜 없이 짧게
    else:
        first = random.choice([f"{G.eul(pen)} {random.choice(G.GRAB)} ", f"{pen} {random.choice(G.GRAB)} ", f"{G.eul(pen)} "])
    body = first; prev_place = start
    for k, s in enumerate(stops):
        v = G.pick_verb(); last = (k == len(stops) - 1)
        if k > 0 and random.random() < 0.3: body += G.regrasp(prev_place, "펜")
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
        else: body += d + " " + G.pick_connect()[1](v) + " "
        prev_place = s
    return body, stops

CHAT2 = ["어디서 와요", "버스 몇 번 타요", "점심 뭐 먹지", "이따 몇 시에 끝나", "주말에 뭐 해요", "알바 끝나고 와요",
         "그 가게 알아요", "거기 맨날 가요", "기숙사 살아요", "시험 언제야", "과제 다 했어", "배고프다",
         "수업 몇 시에 끝나요", "그거 다른 거예요", "아 그렇구나", "진짜요", "몰랐어요", "집이 어디예요",
         "다음 주에 와요", "그럼 내일 봐요", "택배 왔대", "충전기 있어요", "와이파이 비번 뭐야", "아 졸려"]
NONCMD_ARM = ["{arm} 잘 되네", "{arm} 예쁘다", "{arm} 소리가 좀 나는데", "{arm} 지금 뭐 하는 거야",
              "{arm} 언제 만들었어", "{arm} 색 바꿀까", "{col} 도킹 스테이션 어디 있어", "{col} 좋아해",
              "{col} 펜 예쁘다", "{arm} 오늘 상태 괜찮네", "{arm} 아까 떨리던데", "{col} 색깔 별로다"]

def more_noise(t):
    """2차 잡음: 말 다시 시작하기 · 아주 긴 되풀이 루프."""
    if not V2: return t
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
    if k < 0.70:
        n = random.choices([1, 2, 3, 4, 5], weights=[30, 30, 18, 12, 10])[0]
        if V2 and random.random() < 0.5:              # 2차 말투 (seg_body)
            start = rand_place() if random.random() < 0.65 else None
            said = random.random() < 0.6; pen = G.pen_say(pcol, said)
            body, stops = seg_body(start, n, False, pen, True)
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
        if random.random() < 0.35:                    # 팔 이름을 붙인다
            letter = random.choice(list(arms)); idx = ord(letter) - 65
            body = arm_intro(letter, arms) + body
            canon = f"{letter}팔: " + canon
        text = G.finish(body)
        if V2 and random.random() < 0.04:             # 끝에서 같은 말 반복 (단계가 늘지 않는다)
            text += " " + " ".join([f"{G.ro(G.place_say(stops[-1]))} {random.choice(['옮겨야죠.', '옮겨.', '옮겨줘.', '옮겨져', '옮기는 거야.'])}"]
                                   * random.randint(1, 4))
        text = more_noise(G.wrap_noise(text)); kind = f"{n}단계" + ("·팔" if letter else "")
    elif k < 0.80: text, canon = G.make_correction(); text = G.wrap_noise(text); kind = "정정"
    elif k < 0.83: text, canon = G.make_grab_only(); text = G.wrap_noise(text); kind = "잡기만"
    elif k < 0.85: text, canon = G.make_place_only(); text = G.wrap_noise(text); kind = "놓기만"
    elif k < 0.88:
        says, canon = random.choice(G.SIMPLE); text = G.wrap_noise(random.choice(says)); kind = "조작"
    else:
        r = random.random()
        if V2 and r < 0.35:                           # 아주 긴 잡담 (옆 사람 대화가 통째로 들어온 경우)
            t = " ".join(random.choice(CHAT2 + G.CHAT) + random.choice(["", ".", "?", "..", " 어?"])
                         for _ in range(random.randint(4, 16)))
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
    text = more_noise(G.wrap_noise(G.finish(text)))
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
