#!/usr/bin/env python3
"""경유 말투 시험지 (10차, 11-05) → grid_eval_via.json

  사용자 실제 말: "내가 하고 싶은 건 초록색 펜을 주황색에 걸쳐서 빨간색으로 옮기는 걸 하고 싶어."
  9차 모델은 주황을 빠뜨리고 "초록 펜을 빨간색으로 옮겨" 를 냈다. 정답은 "… 주황색으로 옮긴 다음 빨간색으로 옮겨".
  꼴 = 학습함(10차 생성기에 있는 말투) / 처음(생성기에 없는 말투) / 그대로(경유 아닌 말 — 망가지지 않았는지)
"""
import json, os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "nano_norm"))
import gen_norm as G

KO = {"빨강": "빨간색", "주황": "주황색", "노랑": "노란색", "초록": "초록색", "파랑": "파란색", "보라": "보라색"}
ADJ = {"빨강": "빨간", "주황": "주황", "노랑": "노란", "초록": "초록", "파랑": "파란", "보라": "보라"}

# (펜 색, 펜 자리 색, 경유 색들, 도착 색, 장면)
CASES = [
    ("초록", "초록", ["주황"], "빨강", "팔 A=빨강 B=파랑 | 초록=d2 빨강=d8 주황=d9 노랑=d12 파랑=d13 | 펜=초록@d2"),
    ("노랑", "노랑", ["파랑"], "보라", "팔 A=초록 B=노랑 | 보라=d2 빨강=d10 주황=d11 노랑=d12 파랑=d13 초록=d14 | 펜=노랑@d12"),
    ("초록", "보라", ["빨강"], "초록", "팔 A=빨강 B=파랑 | 보라=d2 빨강=d10 주황=d11 노랑=d12 파랑=d13 초록=d14 | 펜=초록@d2"),
]
# (말 꼴, 묶음, 펜 색을 말함, 출발지를 말함) — {p}=펜 색 형용 {s}=출발 {a}=경유 {b}=도착
ONE = [
    ("내가 하고 싶은 건 {p} 펜을 {a}에 걸쳐서 {b}으로 옮기는 걸 하고 싶어.", "실제 말 (사용자)", True, False),
    ("{p} 펜을 {a}에 걸쳐서 {b}으로 옮겨", "에 걸쳐서 (학습함)", True, False),
    ("{p} 펜을 {a} 도킹 스테이션을 거쳐서 {b} 도킹 스테이션에 꽂아 줘", "을 거쳐서 (학습함)", True, False),
    ("{p} 펜 잡아서 {a}에 들렀다가 {b}에 놔 줘", "에 들렀다가 (학습함)", True, False),
    ("펜을 {a} 찍고 {b}으로 옮겨", "찍고 (학습함)", False, False),
    ("{s} 도킹에 있는 펜을 {a}에 잠깐 놨다가 {b}으로 옮기고 싶어", "잠깐 놨다가 (학습함)", False, True),
    ("{p} 펜을 {a}을 경유해서 {b}으로 보내 줘", "경유해서 (학습함)", True, False),
    ("{p} 펜을 {a}에 한 번 걸쳤다가 {b}으로 옮겨 줘", "걸쳤다가 (처음)", True, False),
    ("{p} 펜을 {a} 거쳐 가서 {b}에다 둬", "거쳐 가서 (처음)", True, False),
    ("{p} 펜을 {a} 들러서 가지고 {b}으로 옮겨줄래?", "들러서 가지고 (처음)", True, False),
    ("음 그러니까 {p} 펜을 {a}에다가 걸쳐 놓고 나서 {b}으로 옮겨 주세요", "걸쳐 놓고 나서 (처음)", True, False),
]
PLAIN = [
    ("{p} 펜을 {b}으로 옮겨", "그대로: 한 단계", True, False),
    ("{p} 펜을 {a}으로 옮긴 다음에 {b}으로 옮겨", "그대로: 옮긴 다음에", True, False),
]


def fix(t):
    """'빨간색으로' 처럼 받침에 맞게 (으로/로 · 을/를)."""
    for w in set(list(KO.values()) + list(KO.keys())):
        t = t.replace(w + "으로", G.ro(w)).replace(w + "을 ", G.eul(w) + " ")
    return t


def main():
    items = []
    for pen, start, vias, dest, scene in CASES:
        for forms, fac in ((ONE, "경유"), (PLAIN, "그대로")):
            for tpl, form, pen_said, start_said in forms:
                stops = [("c", c) for c in (vias if fac == "경유" or "다음" in tpl else [])] + [("c", dest)]
                if fac == "그대로" and "다음" not in tpl:
                    stops = [("c", dest)]
                t = tpl.format(p=KO[pen], s=KO[start], a=KO[vias[0]], b=KO[dest])
                canon = G.canon_transfer(("c", start) if start_said else None, pen_said, pen, stops)
                items.append(dict(text=fix(t), canon=canon, scene=scene, pos="앞", factor=fac, form=form, mode="팔 없음"))
        # 두 군데 경유 · 팔 이름
        t2 = f"{KO[pen]} 펜을 {KO[vias[0]]}이랑 노란색을 거쳐서 {KO[dest]}으로 옮겨"
        st2 = [("c", vias[0]), ("c", "노랑"), ("c", dest)]
        if "노랑" not in scene or pen == "노랑":
            continue
        items.append(dict(text=fix(t2), canon=G.canon_transfer(None, True, pen, st2), scene=scene, pos="앞",
                          factor="경유", form="두 군데 이랑…거쳐서 (학습함)", mode="팔 없음"))
        t3 = f"{KO[pen]} 펜을 {KO[vias[0]]} 찍고 노란색 찍고 {KO[dest]}으로 옮겨줘"
        items.append(dict(text=fix(t3), canon=G.canon_transfer(None, True, pen, st2), scene=scene, pos="앞",
                          factor="경유", form="두 군데 찍고 찍고 (학습함)", mode="팔 없음"))
        t4 = f"파란 팔로 {KO[pen]} 펜을 {KO[vias[0]]}에 걸쳐서 {KO[dest]}으로 옮겨"
        items.append(dict(text=fix(t4), canon="B팔: " + G.canon_transfer(None, True, pen, [("c", vias[0]), ("c", dest)]),
                          scene=scene, pos="앞", factor="경유", form="팔 이름 + 걸쳐서 (학습함)", mode="한팔B"))
    json.dump({"note": "경유 말투 시험지 (11-05, 10차)", "items": items}, open(os.path.join(HERE, "grid_eval_via.json"), "w"),
              ensure_ascii=False, indent=1)
    for it in items[:14]:
        print(it["form"], "|", it["text"], "→", it["canon"])
    print(len(items), "문항")


if __name__ == "__main__":
    main()
