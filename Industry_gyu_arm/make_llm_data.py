#!/usr/bin/env python3
"""ARM STUDIO 전용 LLM 학습 데이터 만들기

지금은 매번 긴 설명서를 프롬프트로 보낸다(물체 목록·블록 형식·호칭 규칙 …).
파인튜닝하면 모델이 그걸 외우므로 프롬프트가 짧아지고 응답이 빨라진다.

여기서는 '이렇게 말하면 이렇게 답해라' 예시를 규칙으로 대량 생성한다.
같은 뜻을 여러 말투로 만들어(존댓말·반말·줄임말·색깔·번호) 실제 사용에 가깝게.

  python make_llm_data.py            -> llm_train.jsonl
  python make_llm_data.py 3000       -> 3000 개
"""
import json, random, sys, itertools

random.seed(7)

# ---- 공장에 있을 법한 것들 (실제 id 형식 그대로) ----
DOCK_BASE = "Industry_gyu_arm_20260727_-_docking_station-5"
# ★ 짧은 별명을 쓴다 ★
#   실제 id 는 48자("Industry_gyu_arm_20260727_-_docking_station-5#11").
#   그대로 뱉게 하면 답이 200자가 넘어 작은 기기에서 그만큼 느려진다.
#   "d11" 처럼 짧게 배우게 하고, 서버가 실제 id 로 되돌린다(_llm_expand).
DOCKS = [f"d{n}" for n in (2, 3, 8, 9, 10, 11)]
COLORS = ["빨간", "주황", "노란", "초록", "파란", "보라"]
BRICK = "ppap"
BRICK_CALLS = ["펜", "그거", "저거", "부품", "ppap", "펜대", "물체"]

def dock_no(d):      return d[1:]        # d11 -> 11
def dock_calls(d, color=None):
    n = dock_no(d)
    out = [f"{n}번 도킹", f"{n}번 자리", f"{n}번", f"{n}번 스테이션", f"도킹 {n}번"]
    if color: out += [f"{color} 도킹", f"{color} 자리", f"{color}색 도킹"]
    return out

def blocks_move_place(dst):
    """이동 -> 놓기 (잡기·놓기는 좌표를 갖지 않는다: 앞에 반드시 @goto)"""
    return [f"@goto:{dst}", f"@release:{dst}"]

def blocks_pick(src):
    return [f"@goto:{src}", f"@grasp:{BRICK}"]

def blocks_pick_place(src, dst):
    return [f"@goto:{src}", f"@grasp:{BRICK}",
            f"@goto:{dst}", f"@release:{dst}"]

# ★ say(안내 문구)는 넣지 않는다 ★
#   작은 기기(라떼판다 N100)에서는 토큰 하나가 곧 시간이다.
#   say 를 빼면 답이 80토큰 -> 30토큰으로 줄어 2배 이상 빨라진다.
#   안내 문구는 서버가 블록을 보고 규칙으로 만들면 된다 — LLM 이 만들 이유가 없다.
#   같은 이유로 기본값(false/null)인 키도 뺀다.
def ans(blocks=None, clear=False, preview=True, stop=False, speed=None, say=""):
    d = {"blocks": blocks or []}
    if clear:   d["clear"] = True
    if not preview: d["preview"] = False
    if stop:    d["stop"] = True
    if speed:   d["speed"] = speed
    return d

SAMPLES = []
def add(text, answer):
    SAMPLES.append({"text": text, "answer": answer})

# ① 잡아서 옮기기 (가장 흔한 명령)
for src, dst in itertools.permutations(DOCKS, 2):
    ci, cj = COLORS[DOCKS.index(src) % 6], COLORS[DOCKS.index(dst) % 6]
    b = blocks_pick_place(src, dst)
    for sc in dock_calls(src, ci)[:3]:
        for dc in dock_calls(dst, cj)[:3]:
            for pat in (f"{sc}에 있는 {random.choice(BRICK_CALLS)} {dc}에 꽂아",
                        f"{sc} 펜 {dc}" + ("로" if dc[-1] in "0123456789" or
                            (ord(dc[-1])-0xAC00) % 28 == 0 else "으로") + " 옮겨줘",
                        f"{sc}에서 빼서 {dc}에 넣어",
                        f"{sc}의 부품을 {dc}에 옮겨주세요"):
                add(pat, ans(b, say=f"{dock_no(src)}번에서 빼서 {dock_no(dst)}번에 꽂겠습니다"))

# ② 놓기만
for d in DOCKS:
    c = COLORS[DOCKS.index(d) % 6]
    b = blocks_move_place(d)
    for dc in dock_calls(d, c):
        for pat in (f"{dc}에 꽂아", f"{dc}에 놓아줘", f"{dc}에 넣어",
                    f"{dc}로 가서 놓아", f"{dc}에 꽂아주세요"):
            add(pat, ans(b, say=f"{dock_no(d)}번에 놓겠습니다"))

# ③ 잡기만
for d in DOCKS:
    c = COLORS[DOCKS.index(d) % 6]
    b = blocks_pick(d)
    for dc in dock_calls(d, c):
        for pat in (f"{dc}에 있는 거 잡아", f"{dc} 펜 잡아줘",
                    f"{dc}에서 집어", f"{dc}에 있는 부품 들어"):
            add(pat, ans(b, say=f"{dock_no(d)}번에서 잡겠습니다"))

# ④ 그리퍼·하강·상승 (제자리 동작 — 좌표 없음)
for t in ("그리퍼 열어", "집게 열어", "열어줘", "펴"):
    add(t, ans(["@open"], say="그리퍼를 엽니다"))
for t in ("그리퍼 닫아", "집게 닫아", "닫아줘", "잡아 오므려"):
    add(t, ans(["@close"], say="그리퍼를 닫습니다"))
for t in ("하강", "내려", "아래로", "내려가", "조금 내려"):
    add(t, ans(["@down"], say="내려갑니다"))
for t in ("상승", "올려", "위로", "올라가", "조금 올려"):
    add(t, ans(["@up"], say="올라갑니다"))

# ⑤ 파이프라인 조작
for t in ("파이프라인 지워", "다 지워", "블록 전부 삭제", "처음부터", "초기화해", "비워"):
    add(t, ans([], clear=True, preview=False, say="파이프라인을 비웠습니다"))
for t in ("정지", "멈춰", "스톱", "그만", "정지해"):
    add(t, ans([], preview=False, stop=True, say="정지합니다"))
for t in ("미리보기", "돌려봐", "실행", "재생", "보여줘", "시뮬 돌려"):
    add(t, ans([], preview=True, say="3D 미리보기를 재생합니다"))

# ⑥ 속도
for sp, words in (("0.5x", ["아주 느리게", "half 로", "천천히 아주"]),
                  ("1.0x", ["보통 속도로", "기본 속도"]),
                  ("1.5x", ["조금 빠르게", "1.5배속"]),
                  ("2.0x", ["빠르게", "2배속", "빨리"]),
                  ("4.0x", ["최고 속도", "아주 빠르게", "4배속"])):
    for w in words:
        add(w, ans([], preview=False, speed=sp, say=f"속도를 {sp} 로 맞췄습니다"))

# ⑦ 모르는 것 -> 되물어야 한다 (환각 방지 — 중요)
for t in ("커피 타줘", "불 켜", "노래 틀어줘", "청소해", "99번 도킹에 꽂아",
          "빨래 개줘", "문 열어", "테레비 켜"):
    add(t, ans([], preview=False,
               say="그건 제가 할 수 없습니다. 이동·잡기·놓기만 됩니다"))

def main():
    want = int(sys.argv[1]) if len(sys.argv) > 1 else 2000
    random.shuffle(SAMPLES)
    out = SAMPLES[:want]
    # 프롬프트도 짧게. 파인튜닝하면 모델이 규칙을 외우므로 길게 설명할 필요가 없다.
    # (짧아진 만큼 입력 처리 시간도 준다 — 작은 기기에서 크게 체감된다)
    SYS = ("로봇팔 블록 통역. JSON 만 답한다. "
           "blocks: @goto:<id> @grasp:<id> @release:<id> @open @close @down @up. "
           "잡기/놓기 앞엔 그 대상 @goto 필수. "
           "선택키: clear, preview, stop, speed.")
    with open('llm_train.jsonl', 'w', encoding='utf-8') as f:
        for s in out:
            f.write(json.dumps({"messages": [
                {"role": "system", "content": SYS},
                {"role": "user", "content": s["text"]},
                {"role": "assistant",
                 "content": json.dumps(s["answer"], ensure_ascii=False)}
            ]}, ensure_ascii=False) + "\n")
    print(f"만든 예시: {len(out):,}개  (전체 후보 {len(SAMPLES):,})")
    print("저장: llm_train.jsonl")

if __name__ == '__main__':
    main()
