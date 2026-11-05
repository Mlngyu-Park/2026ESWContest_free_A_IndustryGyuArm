#!/usr/bin/env python3
"""'비…' 낱말 시험지 (11-03, 9차) — 8차 모델이 '비우고'·'비닐' 의 '비' 를 B팔로 읽었다.

낱말 × (팔 없음 2 · A팔 부름 1 · B팔 부름 1). 팔을 안 불렀으면 정리문에 팔이 없어야 하고,
불렀으면 '비…' 낱말과 상관없이 부른 팔이어야 한다.
  · 학습함 (9차 생성기 BI9 에 있음): 비우고 · 다 비우고 · 비닐 치우고 · 비켜 봐 · 비슷하게 · 지영 씨 여기요.
  · 처음 (생성기에 없음): 비스듬히 · 비밀번호 치고 · 비교해 보고 · 비행기 소리 나네 · 비빔밥 먹고 · 비번 바꾸고 · 씨앗 치우고

  python make_grid_bi.py > grid_eval_bi.json
"""
import json, os, random, sys
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import make_grid_arm as GA

random.seed(1104)
WORDS = [("비우고", "학습함"), ("다 비우고", "학습함"), ("비닐 치우고", "학습함"), ("비켜 봐", "학습함"),
         ("비슷하게", "학습함"), ("지영 씨 여기요.", "학습함"),
         ("비스듬히", "처음"), ("비밀번호 치고", "처음"), ("비교해 보고", "처음"), ("비행기 소리 나네.", "처음"),
         ("비빔밥 먹고", "처음"), ("비번 바꾸고", "처음"), ("씨앗 치우고", "처음")]

def item(word, L):
    """L = None (팔 없음) · 'A' · 'B'."""
    arms = GA.pick_arms(); src, rest, canon, x = GA.one_seg()
    if L is None:
        t, c = f"{word} {src} {rest}", canon
    else:
        form = random.choice(["B팔", "파란 팔", "비팔", "비 팔"])
        say = GA.FORMS[form][0](L, arms[L])
        t, c = f"{word} {GA.pj(say, random.choice(['이', '로', '가지고']))} {src} {rest}", f"{L}팔: " + canon
    return dict(text=" ".join(t.split()), canon=c, scene=GA.scene(arms, x), pos="앞")

def main():
    items = []
    for w, seen in WORDS:
        for L, mode in ((None, "팔 없음"), (None, "팔 없음"), ("A", "한팔A"), ("B", "한팔B")):
            it = item(w, L); it.update(factor="비낱말", form=f"{w} ({seen})", mode=mode); items.append(it)
    json.dump(dict(note="'비…' 낱말 시험지 (11-03, 9차)", items=items), sys.stdout, ensure_ascii=False, indent=0)
    print(f"{len(items)}문장", file=sys.stderr)

if __name__ == "__main__":
    main()
