#!/usr/bin/env python3
"""정리문 → 블록 (규칙 해석기). 더블 레이첼 학습 정답을 만들 때와, 실행 때 검사할 때 같이 쓴다.

정리문 문법 (" / " 로 구간을 나눈다):
  [A팔: ]<옮기기>            보라색 도킹의 초록 펜을 빨간색으로 옮긴 다음 3번 도킹으로 옮겨
  [A팔: ]<잡기만>            보라색 도킹의 펜을 잡아
  [A팔: ]<놓기만>            보라색 도킹에 놓아줘
  N초 기다려
  그리퍼 열어 · 그리퍼 닫아 · 내려 · 올려 · 멈춰 · 다 지워 · 미리보기 · 빠르게 · 천천히
  (명령 아님)

장면:  팔 A=빨강 B=파랑 | 보라=d2 빨강=d10 | 펜=초록@d2
블록 이름 (정형 영어 — 앱 블록과 1:1):
  arm:A  move_to:d10  grab:pen  release:d10  wait:10  open close down up stop clear preview faster slower
"""
import re

COLORS = ["빨강", "주황", "노랑", "초록", "파랑", "보라"]
CANON_C = {"빨강": "빨간색", "주황": "주황색", "노랑": "노란색", "초록": "초록색", "파랑": "파란색", "보라": "보라색"}
C_OF = {v: k for k, v in CANON_C.items()}
SIMPLE = {"그리퍼 열어": "open", "그리퍼 닫아": "close", "내려": "down", "올려": "up", "멈춰": "stop",
          "다 지워": "clear", "미리보기": "preview", "빠르게": "faster", "천천히": "slower"}
NONE = "(명령 아님)"
# 앱 블록과 1:1 (앱 안 이름은 바꾸지 않는다)
TO_APP = {"move_to": "@goto", "grab": "@grasp", "release": "@release", "open": "@open", "close": "@close",
          "down": "@down", "up": "@up"}

def parse_scene(h):
    """'팔 A=빨강 B=파랑 | 보라=d2 빨강=d10 | 펜=초록@d2' -> dict"""
    sc = dict(arms={}, docks={}, pen=None, pen_home=None)
    for part in h.split("|"):
        part = part.strip()
        if part.startswith("팔 "):
            for kv in part[2:].split():
                k, v = kv.split("="); sc["arms"][k] = v
        elif part.startswith("펜="):
            c, d = part[2:].split("@"); sc["pen"], sc["pen_home"] = c, d
        else:
            for kv in part.split():
                k, v = kv.split("="); sc["docks"][k] = v
    return sc

def scene_str(sc):
    a = " ".join(f"{k}={v}" for k, v in sc["arms"].items())
    d = " ".join(f"{k}={v}" for k, v in sc["docks"].items())
    return f"팔 {a} | {d} | 펜={sc['pen']}@{sc['pen_home']}"

_PLACE = r"(?:(\d+)번 도킹|(빨간색|주황색|노란색|초록색|파란색|보라색))"
_PLACE_AT = r"(?:(\d+)번 도킹|(빨간색|주황색|노란색|초록색|파란색|보라색) 도킹)"

class ParseError(Exception): pass

def _dock(num, col, sc):
    if num: return f"d{int(num)}"
    c = C_OF[col]
    if c not in sc["docks"]: raise ParseError(f"장면에 없는 색: {c}")
    return sc["docks"][c]

def _segment(seg, sc, state):
    out = []
    m = re.fullmatch(r"([A-Z])팔: (.*)", seg)
    if m:
        if m.group(1) not in sc["arms"]: raise ParseError(f"장면에 없는 팔: {m.group(1)}")
        out.append(f"arm:{m.group(1)}"); seg = m.group(2)
    m = re.fullmatch(r"(\d+)초 기다려", seg)
    if m: return out + [f"wait:{int(m.group(1))}"]
    if seg in SIMPLE: return out + [SIMPLE[seg]]
    m = re.fullmatch(_PLACE_AT + r"의 펜을 잡아", seg)
    if m:
        d = _dock(m.group(1), m.group(2), sc); state["at"] = d
        return out + [f"move_to:{d}", "grab:pen"]
    m = re.fullmatch(_PLACE_AT + r"에 놓아줘", seg)
    if m:
        d = _dock(m.group(1), m.group(2), sc); state["at"] = d
        return out + [f"move_to:{d}", f"release:{d}"]
    # 옮기기: [자리의 ][색 ]펜을 자리(으)로 [옮긴 다음 자리(으)로]* 옮겨
    m = re.fullmatch(r"(?:" + _PLACE_AT + r"의 )?(?:(빨간|주황|노란|초록|파란|보라) )?펜을 (.*) 옮겨", seg)
    if not m: raise ParseError(f"모르는 꼴: {seg}")
    start = _dock(m.group(1), m.group(2), sc) if (m.group(1) or m.group(2)) else None
    stops = []
    for p in m.group(4).split(" 옮긴 다음 "):
        pm = re.fullmatch(_PLACE + r"(?:으로|로)", p)
        if not pm: raise ParseError(f"모르는 자리: {p}")
        stops.append(_dock(pm.group(1), pm.group(2), sc))
    home = start or state.get("at") or sc["pen_home"]
    out += [f"move_to:{home}"]
    for d in stops: out += ["grab:pen", f"move_to:{d}", f"release:{d}"]
    state["at"] = stops[-1]
    return out

# ── 이름 블록 (3차, 10-30): 모델은 도킹 번호 대신 '자리 이름' 으로 블록을 쓴다 ──
#   move_to:보라 · move_to:#3 · move_to:펜(펜이 지금 있는 곳)  → 번호 찾기는 계산(resolve)이 한다.
#   (실측: 색→번호 찾기를 모델이 하면 블록 줄이 자주 틀렸다 — 2차 2000걸음 실제 말 55% vs 정리문 87%)
_NAMED_SC = dict(arms={c: c for c in "ABCDEFGHIJKLMNOPQRSTUVWXYZ"},
                 docks={c: c for c in COLORS}, pen=None, pen_home="펜")

def to_named(canon):
    """정리문 → 이름 블록 (장면 없이)."""
    out = to_blocks(canon, _NAMED_SC)
    return [re.sub(r":d(\d+)$", r":#\1", b) if b.startswith(("move_to:", "release:")) else b for b in out]

def resolve(named, sc):
    """이름 블록 → 번호 블록. 모르는 이름이면 ParseError."""
    out = []
    for b in named:
        k, _, v = b.partition(":")
        if k in ("move_to", "release"):
            if v.startswith("#") and v[1:].isdigit(): v = f"d{int(v[1:])}"
            elif v == "펜": v = sc["pen_home"]
            elif v in sc["docks"]: v = sc["docks"][v]
            else: raise ParseError(f"장면에 없는 자리: {v}")
        elif k == "arm" and v not in sc["arms"]: raise ParseError(f"장면에 없는 팔: {v}")
        out.append(f"{k}:{v}" if v else k)
    return out

def to_blocks(canon, sc):
    """정리문 → 블록 목록. 문법에 안 맞으면 ParseError."""
    canon = canon.strip()
    if canon == NONE: return []
    state = {}
    out = []
    for seg in canon.split(" / "):
        out += _segment(seg.strip(), sc, state)
    return out

def squash(blocks):
    """동작이 같은 중복을 뺀다 (10-30): 이미 그 팔인데 또 팔 고르기 · 방금 놓은(간) 자리로 또 이동.
    'A팔: X→Y / A팔: Y→Z' 와 'A팔: X→Y 옮긴 다음 Z' 가 같은 블록이 된다."""
    out = []; arm = None; at = None; holding = False
    for b in blocks:
        k, _, v = b.partition(":")
        if k == "arm":
            if v == arm: continue
            arm = v; at = None                     # 다른 팔로 바뀌면 그 팔 자리는 모른다
        elif k == "move_to":
            if v == at and not holding: continue   # 펜을 든 채 가는 이동은 남긴다 (같은 자리에 다시 놓기)
            at = v
        elif k == "grab": holding = True
        elif k == "release": at = v; holding = False
        out.append(b)
    return out

def to_app(blocks):
    """정형 블록 → 앱 블록 (@goto:d10 …). arm/wait 는 앱 쪽에서 새로 받는다."""
    out = []
    for b in blocks:
        k, _, v = b.partition(":")
        if k == "grab": out.append("@grasp:ppap")
        elif k in TO_APP: out.append(TO_APP[k] + (":" + v if v else ""))
        else: out.append("@" + b)
    return out

if __name__ == "__main__":
    sc = parse_scene("팔 A=빨강 B=파랑 | 보라=d2 빨강=d10 노랑=d12 초록=d14 | 펜=초록@d2")
    for c in ["보라색 도킹의 초록 펜을 빨간색으로 옮긴 다음 3번 도킹으로 옮겨",
              "A팔: 보라색 도킹의 펜을 노란색으로 옮겨 / 10초 기다려 / B팔: 노란색 도킹의 펜을 초록색으로 옮겨",
              "펜을 노란색으로 옮겨", "그리퍼 열어", "(명령 아님)", "B팔: 노란색 도킹의 펜을 잡아"]:
        print(c, "\n  →", " ".join(to_blocks(c, sc)))
