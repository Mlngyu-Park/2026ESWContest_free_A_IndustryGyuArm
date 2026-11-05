"""더블 레이첼 출력 검사 (계산) — AI 출력을 그대로 믿지 않는다.

  ① 정리문이 문법에 맞나 (canon.to_blocks 가 풀리나)
  ② 블록 줄(이름 블록)이 정리문에서 계산한 이름 블록과 같나 (모델이 스스로 어긋나지 않았나)
     → 통과하면 이름 → 도킹 번호는 계산(canon.resolve)이 바꾼다
  ③ 도킹 색·번호를 지어내지 않았나: 정리문의 언급 순서가 원문 언급 순서의 '부분 수열'
     (팔 색은 정리문에서 A/B 로 바뀌므로 원문 쪽에 더 있는 건 괜찮다)
  ④ 팔을 지어내지 않았나: 정리문에 팔이 있으면 원문에 '팔'·'로봇' 이 있어야 한다
  ⑤ 명령을 지우지 않았나
  ⑥ 뒤쪽 단계를 빠뜨리지 않았나
  ⑦ 가운데 단계를 빠뜨리지 않았나 · 기다리기를 빠뜨리지 않았나 (10-31)
  ⑧ 팔을 바꾸지 않았나: 원문이 '확실히' 부른 팔과 정리문의 팔이 맞나 (11-03)
  ⑨ 출발지를 목적지로 바꾸지 않았나: 원문 첫 자리가 'X에 꽂힌/있는' 이면 정리문은 X(또는 펜 자리)에서 출발,
     첫 목적지가 X 면 안 된다 (11-03 9차)
"""
import os, re, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "nano_norm"))
from safety import mentions, _subseq, MOVE_WORDS
from canon import to_blocks, to_named, NONE

def split_out(out):
    """모델 출력 '정리문\\n=블록' -> (정리문, 블록 목록)."""
    if "\n=" not in out: return out.strip(), None
    c, b = out.split("\n=", 1)
    return c.strip(), b.split()

# ⑥ 뒤쪽 단계를 빠뜨리지 않았나 (10-30): 원문에서 '마지막 목적지' 뒤에 또 자리가 나오면 의심한다.
#    팔 색('빨간 팔로요')·펜 색('초록색 펜')은 자리로 치지 않는다. 같은 자리를 되풀이하는 건 괜찮다.
from safety import FORMS, _PAT
_ARM_AFTER = re.compile(r"^(?:색깔|색)?(?:팔|로봇)")
_PEN_AFTER = re.compile(r"^(?:색깔|색)?(?:그러니까|그|막|음|어|저|이)*(?:펜|팬|핀|캔|피피|ppap|PPAP|펜대|부품|물체)")
def place_mentions(t):
    t = re.sub(r"[\s.~…,]+", "", t)
    t = t.replace("자주황", "_주황").replace("자주화", "_주화")
    out = []
    for m in re.finditer(_PAT, t):
        rest = t[m.end():m.end() + 40]
        if _ARM_AFTER.match(rest) or _PEN_AFTER.match(rest): continue
        out.append((m.start(), FORMS[m.group()]))
    for m in re.finditer(r"(\d+)번", t):
        if not _ARM_AFTER.match(t[m.end():m.end() + 4]): out.append((m.start(), f"#{m.group(1)}"))
    return [x for _, x in sorted(out)]

def dropped_tail(src, canon):
    """빠뜨린 뒤쪽 자리 (없으면 ''). 마지막 목적지가 원문에 안 보이면 판단하지 않는다 (③이 본다)."""
    named = to_named(canon)
    rel = [b.split(":", 1)[1] for b in named if b.startswith(("release:", "move_to:"))]
    if not rel or rel[-1] == "펜": return ""
    last = rel[-1]
    pm = place_mentions(src)
    if last not in pm: return ""
    k = len(pm) - 1 - pm[::-1].index(last)
    after = [x for x in pm[k + 1:] if x != last]
    return ",".join(dict.fromkeys(after))

# ⑦ 가운데 단계·기다리기를 빠뜨리지 않았나 (10-31): 격자 세트3 에서 '보라 찍고 파랑으로' → 파랑만,
#    '5초 쉬었다가' → 기다리기 없이 바로 옮김 — ⑥ 은 마지막 목적지 뒤만 봐서 못 잡았다 (6차 2000걸음 틀린 채 실행 23).
#    첫 자리와 마지막 목적지 사이에서 원문이 말한 자리가 출력에 없으면 의심한다.
#    정정하는 말('아니'·'정정' …)이 있으면 보지 않는다. 펜 색 · ('팔/로봇' 이 있으면) 팔 색은 자리로 치지 않는다.
#    실측: 실제 말·손 세트·격자 1·2 의 맞는 답을 거부한 것 0, 생성기 정답 5000 중 8, 세트3 틀린 채 실행 19 중 16 을 막음.
_WAIT = re.compile(r"(\d+|[일이삼사오육칠팔구십]+) ?초(?!록)")
_FIX = re.compile(r"정정|아니|말고|잠깐|틀렸|취소")
def dropped_mid(src, canon, sc=None):
    """첫 자리와 마지막 목적지 사이에서 빠진 자리 (없으면 '')."""
    if _FIX.search(src): return ""
    skip = set()
    if sc:
        if sc.get("pen"): skip.add(sc["pen"])
        if re.search(r"팔|로봇", src): skip |= set(sc["arms"].values())
    rel = [b.split(":", 1)[1] for b in to_named(canon) if b.startswith(("release:", "move_to:"))]
    got = set(rel)
    pm = place_mentions(src)
    idx = [i for i, x in enumerate(pm) if x in got]
    if not idx or rel[-1] not in pm: return ""
    k = len(pm) - 1 - pm[::-1].index(rel[-1])
    return ",".join(dict.fromkeys(x for x in pm[idx[0]:k] if x not in got and x not in skip))

def dropped_wait(src, canon):
    """원문에 'N초' 가 정리문의 기다리기보다 많으면 True."""
    return len(set(_WAIT.findall(src))) > canon.count("초 기다려")

# ⑧ 팔을 바꾸지 않았나 (11-03): 실제 말 "비팔로 피피에이피를 잡아서 …" → 'A팔:' 로 실행될 뻔했다.
#    음성 인식은 B팔을 '비팔'·'B8'(팔→8)·'비 로봇' 으로도 적는다. 원문에서 확실한 팔 부름만 찾고
#    (못 찾으면 검사하지 않는다), 정리문의 팔이 그 안에 없으면 거부 → 다수결 보충이 다시 뽑는다.
#    정정('아니'·'말고' …)·'다른 팔'·'쉬고' 처럼 부른 팔과 하는 팔이 다를 수 있는 말은 보지 않는다.
_ARM_K = {"에이": "A", "비": "B", "씨": "C"}
_ARM_TAIL = r"\s?(?:팔|로봇|8(?!\s?번)|암)"
_ARM_PCL = r"(?=(?:으로|로|가|는|은|이|를|을|한테|랑|하고|도|만|가지고)?(?:[^가-힣]|$))"   # 뒤에 조사만 (비우고 ✗)
_ARM_C = {"빨강": "빨간|빨강", "주황": "주황|오렌지", "노랑": "노란|노랑", "초록": "초록|녹색", "파랑": "파란|파랑", "보라": "보라"}
_ARM_ORD = [(r"첫\s?번째|첫째|(?<!\d)1\s?번째?", "A"), (r"두\s?번째|둘째|(?<!\d)2\s?번째?", "B"), (r"세\s?번째|셋째|(?<!\d)3\s?번째?", "C")]
_ARM_SKIP = re.compile(r"정정|아니|말고|잠깐|틀렸|취소|다른\s?(?:팔|로봇)|나머지|반대|둘\s?다|두\s?팔|양\s?팔|쉬고|가만|놔두")
def said_arms(src, sc):
    """원문에서 확실히 부른 팔 글자 집합 (모르면 빈 집합)."""
    out = set()
    for m in re.finditer(r"(?<![A-Za-z])([A-Za-z])" + _ARM_TAIL, src): out.add(m.group(1).upper())
    for m in re.finditer(r"로봇\s?([A-Za-z])(?![A-Za-z])", src): out.add(m.group(1).upper())
    for m in re.finditer(r"(?<![가-힣])(에이|비|씨)" + _ARM_TAIL, src): out.add(_ARM_K[m.group(1)])
    for m in re.finditer(r"로봇\s?(에이|비|씨)" + _ARM_PCL, src): out.add(_ARM_K[m.group(1)])   # '로봇 비로 해 줘' (9차)
    for pat, L in _ARM_ORD:
        if re.search(r"(?:" + pat + r")\s?(?:로봇\s?)?팔", src): out.add(L)
    for L, c in (sc or {}).get("arms", {}).items():
        if c in _ARM_C and re.search(r"(?:" + _ARM_C[c] + r")\s?(?:색\s?)?(?:깔\s?)?(?:로봇\s?)?팔", src): out.add(L)
    return out

def wrong_arm(src, canon, sc):
    """정리문의 팔이 원문과 다르면 그 설명, 아니면 ''."""
    if _ARM_SKIP.search(src): return ""
    said = said_arms(src, sc)
    if not said: return ""
    if not any(b.startswith(("move_to:", "grab:", "release:")) for b in to_named(canon)): return ""   # '파란 팔 멈춰' 같은 전체 조작
    got = set(re.findall(r"([A-Z])팔:", canon))
    if not re.match(r"\s*[A-Z]팔:", canon): got.add("A")                  # 맨 앞에 팔이 없으면 기본 A팔
    # 원문이 부른 팔이 정리문에 없으면 거부. (반대로 정리문에만 있는 팔은 보지 않는다 —
    #  '보라 뭐지 로봇 팔' 처럼 머뭇거림이 끼면 부른 팔을 다 못 찾으므로: 생성기 정답 6000 중 48 을 잘못 거부했다)
    miss = said - got
    if miss: return f"원문 팔 {''.join(sorted(said))} · 정리문 팔 {''.join(sorted(got))}"
    return ""

# ⑨ 출발지를 목적지로 바꾸지 않았나 (11-03 9차): '보라색 도킹에 꽂힌 펜을 노랑으로 보내 놓고 빨강으로' →
#    '펜을 보라색으로 옮긴 다음 노란색 …'. 원문의 맨 처음 자리가 'X에 꽂힌/있는/놓인' 인데 정리문이
#    다른 곳에서 출발하거나, '펜을 X로' (펜 자리에서 X 로 — 첫 목적지가 출발지) 면 거부. '펜을 Y로' 는 괜찮다
#    (펜 자리는 장면에서 계산). 정정하는 말이 있으면 보지 않는다.
#    (같은 자리로 다시 옮기는 건 막지 않는다 — 실제 말 '보라색에 있는 걸 잡아서 다시 보라색에다 내려놓은 다음에' 가 있다)
_ORIGIN = re.compile(r"^(?:색깔|색)?(?:도킹스테이션|토킹스테이션|도킹|토킹|스테이션|쪽|자리)?에"
                     r"(?:꽂힌|꽂혀있는|꽂혀있던|있는|있던|놓인|놓여있는|놓여있던|꽂아둔|꽂아놓은)")
def first_origin(src):
    """원문 맨 처음 자리가 출발지로 말해졌으면 그 이름 ('보라' · '#7'), 아니면 ''."""
    t = re.sub(r"[\s.~…,]+", "", src)
    t = t.replace("자주황", "_주황").replace("자주화", "_주화")
    found = []
    for m in re.finditer(_PAT, t):
        rest = t[m.end():m.end() + 40]
        if _ARM_AFTER.match(rest) or _PEN_AFTER.match(rest): continue
        found.append((m.start(), FORMS[m.group()], rest))
    for m in re.finditer(r"(\d+)번", t):
        rest = t[m.end():m.end() + 30]
        if not _ARM_AFTER.match(rest): found.append((m.start(), f"#{m.group(1)}", rest))
    if not found: return ""
    _, name, rest = min(found)
    return name if _ORIGIN.match(rest) else ""

def lost_origin(src, canon):
    if _FIX.search(src): return ""
    o = first_origin(src)
    if not o: return ""
    mv = [b.split(":", 1)[1] for b in to_named(canon) if b.startswith("move_to:")]
    if not mv: return ""
    if mv[0] == "펜":
        if len(mv) > 1 and mv[1] == o: return f"원문 출발 {o} · 정리문 첫 목적지 {o}"
        return ""
    if mv[0] != o: return f"원문 출발 {o} · 정리문 출발 {mv[0]}"
    return ""

def check(src, out, sc):
    """(통과?, 이유, 블록)."""
    canon, blocks = split_out(out)
    if not canon: return False, "빈 출력", None
    try: calc = to_blocks(canon, sc)
    except Exception as e: return False, f"정리문 꼴 아님: {e}", None
    if blocks is not None and blocks != to_named(canon): return False, "블록이 정리문과 다름", calc
    if canon == NONE:
        if mentions(src) and any(w in src for w in MOVE_WORDS): return False, "명령을 지운 것 같음", calc
        return True, "", calc
    body = re.sub(r"[A-Z]팔: ", "", canon)
    body = re.sub(r"\d+초 기다려", "", body)
    body = re.sub(r"(빨간|주황|노란|초록|파란|보라) 펜", "펜", body)   # 펜 색은 블록과 무관 — 장면에서 끌어와도 괜찮다
    if not _subseq(mentions(body), mentions(src)):
        return False, f"색·번호가 원문과 다름 {mentions(body)} ⊄ {mentions(src)}", calc
    tail = dropped_tail(src, canon)
    if tail: return False, f"뒤쪽 단계를 빠뜨린 것 같음 (마지막 목적지 뒤에 {tail})", calc
    mid = dropped_mid(src, canon, sc)
    if mid: return False, f"가운데 단계를 빠뜨린 것 같음 ({mid})", calc
    if dropped_wait(src, canon): return False, "기다리기를 빠뜨린 것 같음", calc
    lo = lost_origin(src, canon)
    if lo: return False, f"출발지가 원문과 다름 ({lo})", calc
    if re.search(r"[A-Z]팔:", canon) and not re.search(r"팔|로봇|(?:[A-Za-z]|에이|비|씨)\s?8|암", src):   # '비8' 도 팔 (9차)
        return False, "원문에 없는 팔", calc
    wa = wrong_arm(src, canon, sc)
    if wa: return False, f"팔이 원문과 다름 ({wa})", calc
    return True, "", calc
