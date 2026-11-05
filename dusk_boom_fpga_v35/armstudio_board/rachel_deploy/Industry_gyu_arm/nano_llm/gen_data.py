#!/usr/bin/env python3
"""nano LLM 학습 데이터 생성 — 장면 인식형.

기존 make_llm_data.py 와 다른 점:
  ① 장면 헤더를 준다. 색↔도킹 매칭은 배치마다 다르므로 외우게 하지 않고
     '장면을 읽고' 답하게 학습시킨다.
       장면: d8=빨강 d9=주황 d12=노랑 | ppap@d8
       명령: 빨간 도킹의 펜을 노란 데로 옮겨
       답: {"blocks": ["@goto:d8", "@grasp:ppap", "@goto:d12", "@release:d12"]}
  ② 2단계 이송(A->B, B->C) 포함 — 실제 시연 명령.
  ③ STT 오염을 섞는다 — 실사용에서 "도킹"이 "토킹"으로 들어온 사례 그대로.

  python gen_data.py            -> train.jsonl / val.jsonl
  python gen_data.py 120000     -> 원하는 개수
"""
import json, random, sys
import hints

random.seed(int(os.environ.get("DATA_SEED", "11")))   # 데이터 시드 (기본 11 = 지금과 동일)
COLORS  = ["빨강", "주황", "노랑", "초록", "파랑", "보라"]
C_ADJ   = {"빨강":"빨간", "주황":"주황", "노랑":"노란",
           "초록":"초록", "파랑":"파란", "보라":"보라색"}
BRICKS  = ["ppap"]
B_CALLS = ["펜", "그거", "저거", "부품", "물체", "펜대"]

def scene(nd_min=2):
    """무작위 장면: 도킹 2~6개(번호·색 무작위), 펜 1개(색 있음, 한 도킹에 결합).

    v9 에서 5 -> 6 으로 늘렸다. 색이 6가지라 6개가 최대다.
    도킹이 5개뿐이면 출발지를 빼고 경유지가 4곳까지밖에 안 되어,
    "6색 6단계" 를 아예 못 배운다 (실측: 6단계 요청에 2곳만 나옴)."""
    nd = random.randint(max(2, nd_min), 6)
    nums = random.sample(range(2, 15), nd)
    docks = [f"d{n}" for n in nums]
    cols  = random.sample(COLORS, nd)
    home  = random.choice(docks)
    pcol  = random.choice(COLORS)
    # 질문이 대부분 색으로 들어오므로 '색=도킹' 방향 — 색을 찾으면 바로 뒤가 답
    hdr = " ".join(f"{c}={d}" for d, c in zip(docks, cols))
    return docks, dict(zip(docks, cols)), home, pcol, f"{hdr} | 펜={pcol}@{home}"

def RO():
    """'~로/~으로' — 실제 말에는 둘 다 나온다 ('빨간색으로')."""
    return random.choice(["로", "으로"])

def call_dock(d, col):
    """색 지칭이 주력(70%) — 사용자가 색으로 부르는 걸 절대적으로 잘해야 한다."""
    n = d[1:]; a = C_ADJ[col]
    ac = a if a.endswith("색") else a + "색"          # '보라색색' 방지
    color_calls = [f"{a} 도킹", f"{a} 자리", f"{a} 도킹스테이션",
                   f"{ac} 도킹", f"{a} 데", f"{ac} 자리", f"{a} 스테이션"]
    num_calls   = [f"{n}번 도킹", f"{n}번 자리", f"{n}번", f"도킹 {n}번"]
    call = random.choice(color_calls if random.random() < 0.7 else num_calls)
    # 자기 정정 (실사용에서 관찰): "빨간 아 정정 보라 도킹" — 마지막 말이 정답
    if random.random() < 0.13:
        w = C_ADJ[random.choice([c for c in COLORS if c != col])]
        fix = random.choice(["아 아니 ", "아니 ", "정정 ", "아 ", "말고 "])
        call = f"{w} {fix}{call}" if fix != "말고 " else f"{w} 말고 {call}"
    return call

NAME_CALLS = ["레이첼 ", "레이첼아 ", "레이첼, ", "rachel ", "레이첼! "]
FILLERS_A = ["야 ", "어 ", "음 ", "아 ", "그 "]
FILLERS_Z = [" 좀 해줘 봐", " 해줘 봐", " 부탁해", " 좀", "요"]
def vocative(t):
    """실전처럼: 이름(25%)·군말(15%)을 앞에, 맺음말(15%)을 뒤에."""
    if random.random() < 0.25: t = random.choice(NAME_CALLS) + t
    if random.random() < 0.15: t = random.choice(FILLERS_A) + t
    if random.random() < 0.15: t = t + random.choice(FILLERS_Z)
    return t

def stt_noise(t):
    """실사용 STT 오염 재현 (llm_usage.jsonl 에서 실제로 본 것들)."""
    if random.random() < 0.25: t = t.replace("도킹스테이션", "토킹스테이션")
    if random.random() < 0.15: t = t.replace("도킹", "토킹")
    if random.random() < 0.15: t = t.replace(" ", "", 1)          # 띄어쓰기 붕괴
    if random.random() < 0.10: t = t + random.choice(["요", " 좀", " 빨리"])
    return t

def ans(blocks=None, **kw):
    d = {"blocks": blocks or []}
    d.update({k: v for k, v in kw.items() if v})
    return json.dumps(d, ensure_ascii=False)

S = []
def add(sc, text, answer):
    t = stt_noise(vocative(text))
    h = hints.build(add.cmap, add.home, add.pcol, t)   # 오염된 최종 문장 기준
    # 재학습 대조용: NOHINT=1 이면 힌트 줄을 안 붙인다 (= 조회를 신경망이 하는 v1 형식)
    if os.environ.get("NOHINT") == "1": h = ""
    S.append((f"{sc}\n힌트 {h}" if h else sc, t, answer))

def gen(want):
    while len(S) < want:
        docks, cmap, home, pcol, sc = scene()
        add.cmap, add.home, add.pcol = cmap, home, pcol
        others = [d for d in docks if d != home]
        if not others: continue
        kind = random.random()
        pen = random.choice(B_CALLS)
        if random.random() < 0.5:                          # "초록색 펜" 처럼 색으로
            a = C_ADJ[pcol]; a = a if a.endswith("색") else a + "색"
            pen = f"{a} {random.choice(['펜','부품','물체'])}"
        if kind < 0.30:                                   # 잡아서 옮기기
            dst = random.choice(others)
            s_call = call_dock(home, cmap[home]); d_call = call_dock(dst, cmap[dst])
            t = random.choice([
                f"{s_call}에 있는 {pen} {d_call}에 꽂아",
                f"{s_call}의 {pen}을 {d_call}{RO()} 옮겨줘",
                f"{s_call}에서 빼서 {d_call}에 넣어",
                f"{pen} {d_call}에 옮겨주세요",           # 출발지 생략(펜 위치=장면)
                f"{d_call}{RO()} {pen} 이동",
                # '잡은 다음에 …' 는 2단계가 아니라 1단계 이송이다 (실사용 관찰).
                # '~한 다음' 만 보고 2단계로 오인해 없는 목적지를 지어내던 것 교정.
                f"{s_call}에 있는 {pen} 잡은 다음에 그거 {d_call}{RO()} 이동시켜줘",
                f"{s_call}의 {pen} 잡은 다음 {d_call}에 꽂아줘",
                f"{s_call}에서 {pen} 잡고 {d_call}에 놓아줘",
                f"{pen} 잡아서 {d_call}{RO()} 이동시켜"])
            add(sc, t, ans([f"@goto:{home}", "@grasp:ppap",
                            f"@goto:{dst}", f"@release:{dst}"]))
        elif kind < 0.38 and len(others) >= 2:            # 2단계 이송 (옛 분기, 8%)
            mid, dst = random.sample(others, 2)
            s_c, m_c, d_c = (call_dock(home, cmap[home]),
                             call_dock(mid, cmap[mid]), call_dock(dst, cmap[dst]))
            pen2 = random.choice(["그 펜", "그거", "펜", pen])
            t = random.choice([
                f"{s_c}에 있는 {pen}을 {m_c}{RO()} 옮긴 다음 {d_c}{RO()} 옮겨",
                f"{s_c}의 {pen} {m_c}에 놓고 다시 {d_c}에 꽂아줘",
                f"{pen}을 {m_c} 거쳐서 {d_c}{RO()} 보내",
                # '잡아서 …' 가 두 번 나오는 2단계 (실사용 관찰) —
                # '잡+다음' 만으로 1단계로 오인하지 않게, 두 번째 목적지가
                # 있으면 2단계라는 걸 이런 말투로도 배우게 한다.
                f"{s_c}에 있는 {pen} 잡아서 {m_c}{RO()} 옮긴 다음에 "
                f"다시 {pen2} 잡아서 {d_c}{RO()} 옮기는 거 해보자",
                f"{s_c}의 {pen} 잡아서 {m_c}에 놓고, 다시 {pen2} 잡아서 {d_c}에 꽂아",
                f"{pen} 잡아서 {m_c}에 놨다가 또 잡아서 {d_c}{RO()} 옮겨줘",
                f"{s_c}에서 {pen} 잡아 {m_c}에 두고 그 다음 {d_c}{RO()} 이동시켜"])
            add(sc, t, ans([f"@goto:{home}", "@grasp:ppap",
                            f"@goto:{mid}", f"@release:{mid}",
                            "@grasp:ppap",
                            f"@goto:{dst}", f"@release:{dst}"]))
        elif kind < 0.72 and len(others) >= 2:            # 다단계 2~5 (v9.1, 34%)
            # v7 은 3단계 고정이었다. 6색 6도킹으로 시험해 보니
            # 4단계 이상은 아예 못 했고(6단계 요청에 2곳만), 2단계도
            # "옮겼다가" 말투면 틀렸다. 그래서 두 가지를 같이 고친다:
            #   ① 단계 수를 2~6 으로 무작위 (있는 도킹 수 안에서)
            #   ② "옮겼다가/했다가/들렀다가" 말투 추가
            # 단계 수를 먼저 정하고, 그만큼 담을 장면을 새로 뽑는다.
            # 장면을 먼저 뽑으면 도킹이 적을 때가 많아 2단계만 잔뜩
            # 나온다 (실측 2단계 631 vs 5단계 15).
            # 색이 6가지라 도킹은 최대 6개다. 출발지를 빼면 경유지는
            # 최대 5곳 — 그래서 단계는 2~5 로 한다.
            n = random.randint(2, 5)
            docks, cmap, home, pcol, sc = scene(nd_min=n + 1)
            add.cmap, add.home, add.pcol = cmap, home, pcol
            others = [d for d in docks if d != home]
            if len(others) < n: continue
            stops = random.sample(others, n)
            s_c = call_dock(home, cmap[home])
            cs = [call_dock(d, cmap[d]) for d in stops]

            # 말투 — 이어 붙이는 방식이 여러 가지다
            joiners = [
                lambda a, b: f"{a}{RO()} 옮겼다가 {b}",      # ★ v9 추가
                lambda a, b: f"{a}{RO()} 갔다가 {b}",
                lambda a, b: f"{a}에 놨다가 {b}",             # ★
                lambda a, b: f"{a} 들렀다가 {b}",             # ★
                lambda a, b: f"{a} 거쳐서 {b}",
                lambda a, b: f"{a} 다음에 {b}",
            ]
            style = random.random()
            if style < 0.45:
                # "A로 옮겼다가 B로 옮겼다가 C로 옮겨줘"
                j = random.choice(joiners)
                body = cs[0]
                for c in cs[1:]:
                    body = j(body, c)
                t = random.choice([
                    f"{pen}을 {body}{RO()} 옮겨줘",
                    f"{s_c}의 {pen} {body}까지 보내",
                    f"{pen} {body}{RO()} 이동시켜",
                ])
            elif style < 0.75:
                # "A, B, C 순서로"
                lst = ", ".join(cs)
                t = random.choice([
                    f"{pen}을 {lst} 순서로 옮겨줘",
                    f"{lst} 순서대로 {pen} 옮겨",
                    f"{pen} {' → '.join(cs)} 순서로 이동",
                ])
            else:
                # "A에 놓고 또 B에 놓고 마지막에 C에"
                mid = "에 놓고 또 ".join(cs[:-1])
                t = random.choice([
                    f"{pen} 잡아서 {mid}에 놓고 마지막에 {cs[-1]}에 꽂아",
                    f"{s_c}에서 {pen} 빼서 {mid}에 뒀다가 {cs[-1]}에 넣어",
                ])

            blocks = [f"@goto:{home}", "@grasp:ppap"]
            for d in stops:
                blocks += [f"@goto:{d}", f"@release:{d}", "@grasp:ppap"]
            blocks.pop()                 # 마지막에 다시 잡지는 않는다
            add(sc, t, ans(blocks))
        elif kind < 0.80 and len(others) >= 2:            # 부정·정정 (8%)
            # 실사용: "빨간색으로 옮기지 말고 주황색으로 옮겨봐" 를 시켰더니
            # 빨간색으로 갔다. '~하지 말고 ~' 는 뒤쪽이 정답이다.
            wrong, dst = random.sample(others, 2)
            w_c, d_c = call_dock(wrong, cmap[wrong]), call_dock(dst, cmap[dst])
            t = random.choice([
                f"{pen} 잡아서 {w_c}{RO()} 옮기지 말고 {d_c}{RO()} 옮겨봐",
                f"{pen}을 {w_c} 말고 {d_c}에 놔줘",
                f"{w_c}{RO()} 옮기는 거 아니고 {d_c}{RO()} 옮겨",
                f"{pen} {w_c}에 넣지 마 {d_c}에 넣어",
                f"{w_c} 빼고 {d_c}{RO()} {pen} 이동",
                f"{pen}을 {d_c}{RO()} 옮겨 {w_c} 말고"])
            add(sc, t, ans([f"@goto:{home}", "@grasp:ppap",
                            f"@goto:{dst}", f"@release:{dst}"]))
        elif kind < 0.83:                                 # 잡기만
            c = call_dock(home, cmap[home])
            t = random.choice([f"{c}에 있는 거 잡아", f"{c} {pen} 잡아줘",
                               f"{c}에서 집어", f"{pen} 잡아"])
            add(sc, t, ans([f"@goto:{home}", "@grasp:ppap"]))
        elif kind < 0.86:                                 # 놓기만
            dst = random.choice(others)
            c = call_dock(dst, cmap[dst])
            t = random.choice([f"{c}에 꽂아", f"{c}에 놓아줘", f"{c}에 넣어"])
            add(sc, t, ans([f"@goto:{dst}", f"@release:{dst}"]))
        elif kind < 0.93:                                 # 제자리·조작·속도
            t, a = random.choice([
                ("그리퍼 열어", ans(["@open"])), ("집게 열어줘", ans(["@open"])),
                ("그리퍼 닫아", ans(["@close"])), ("잡아 오므려", ans(["@close"])),
                ("내려", ans(["@down"])), ("하강", ans(["@down"])),
                ("올려", ans(["@up"])), ("상승해", ans(["@up"])),
                ("다 지워", ans([], clear=True)), ("초기화", ans([], clear=True)),
                ("멈춰", ans([], stop=True)), ("정지", ans([], stop=True)),
                ("미리보기", ans([], preview=True)), ("돌려봐", ans([], preview=True)),
                ("빠르게", ans([], speed="2.0x")), ("천천히", ans([], speed="0.5x")),
                ("최고 속도로", ans([], speed="4.0x"))])
            add(sc, t, a)
        else:                                             # 모르는 것 -> 빈 블록
            wrong_n = random.choice([n for n in range(2, 30)
                                     if f"d{n}" not in docks])
            absent = [c for c in COLORS if c not in cmap.values()]
            wrong_c = C_ADJ[random.choice(absent)] if absent else "회색"
            t = random.choice(["커피 타줘", "불 켜", "노래 틀어", "청소해줘",
                               f"{wrong_n}번 도킹에 꽂아",       # 장면에 없는 번호
                               f"{wrong_c} 도킹에 꽂아",         # 장면에 없는 색
                               "문 열어", "오늘 날씨 어때"])
            add(sc, t, ans([]))

def main():
    want = int(sys.argv[1]) if len(sys.argv) > 1 else 80000
    gen(want)
    random.shuffle(S)
    n_val = max(500, want // 50)
    for name, part in (("val.jsonl", S[:n_val]), ("train.jsonl", S[n_val:])):
        with open(name, "w", encoding="utf-8") as f:
            for sc, t, a in part:
                f.write(json.dumps({"scene": sc, "text": t, "answer": a},
                                   ensure_ascii=False) + "\n")
    print(f"train {len(S)-n_val:,} / val {n_val:,}")

if __name__ == "__main__":
    main()
