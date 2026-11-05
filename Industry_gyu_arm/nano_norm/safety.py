"""말 정리기 안전 검사 (계산) — AI 출력을 그대로 믿지 않는다.

통과 못 하면 원문을 그대로 RACHEL 에 넘긴다 (정리기가 틀려도 앱이 지금보다 나빠지지 않게).
  ① 색·번호를 지어내지 않았나: 출력의 색·번호 순서가 원문 언급 순서의 '부분 수열' 이어야 한다
     (정정·반복으로 빠지는 건 괜찮고, 새로 생기거나 순서가 바뀌면 안 된다)
  ② 정해진 꼴인가 (말정리기_설계.md 3절)
  ③ 명령을 지우지 않았나: 원문에 색 + 옮기는 말이 있는데 '(명령 아님)' 이면 의심
"""
import re

FORMS = {"빨간": "빨강", "빨강": "빨강", "붉은": "빨강", "레드": "빨강",
         "주황": "주황", "오렌지": "주황", "주홍": "주황", "주화앙": "주황",
         "노란": "노랑", "노랑": "노랑", "옐로": "노랑",
         "초록": "초록", "녹색": "초록", "그린": "초록", "키록": "초록",
         "파란": "파랑", "파랑": "파랑", "블루": "파랑",
         "보라": "보라", "퍼플": "보라", "자주": "보라"}
_PAT = "|".join(sorted(FORMS, key=len, reverse=True))

def mentions(t):
    """색·번호 언급 순서 (연속 중복 제거).
    머뭇거림·띄어쓰기 붕괴('노....란색', '보 라색')로 색이 쪼개지므로 점·빈칸·물결을 빼고 찾는다."""
    t = re.sub(r"[\s.~…]+", "", t)
    t = t.replace("자주황", "_주황").replace("자주화", "_주화")      # 빈칸을 빼면 '정정하자 주황' 이 '자주(보라)' 로 읽힌다
    found = [(m.start(), FORMS[m.group()]) for m in re.finditer(_PAT, t)]
    found += [(m.start(), f"#{m.group(1)}") for m in re.finditer(r"(\d+)\s*번", t)]
    found.sort(); out = []
    for _, x in found:
        if not out or out[-1] != x: out.append(x)
    return out

def _subseq(a, b):
    it = iter(b)
    return all(x in it for x in a)

CANON = [
    re.compile(r"^(?:\S+ 도킹의 |\d+번 도킹의 )?(?:\S+ )?펜을 (?:\S+(?: 도킹)?(?:으로|로) 옮긴 다음 )*\S+(?: 도킹)?(?:으로|로) 옮겨$"),
    re.compile(r"^\S+(?: 도킹)?의 펜을 잡아$|^\S+(?: 도킹)?에 놓아줘$"),
    re.compile(r"^(그리퍼 열어|그리퍼 닫아|내려|올려|멈춰|다 지워|미리보기|빠르게|천천히|\(명령 아님\))$"),
]
MOVE_WORDS = ("옮", "이동", "놓", "꽂", "넣", "갖다", "보내", "두")

def check(src, out):
    """(통과?, 이유)."""
    out = (out or "").strip()
    if not out: return False, "빈 출력"
    if not any(p.match(out) for p in CANON): return False, "정해진 꼴 아님"
    if out == "(명령 아님)":
        if mentions(src) and any(w in src for w in MOVE_WORDS):
            return False, "명령을 지운 것 같음"
        return True, ""
    if not _subseq(mentions(out), mentions(src)):
        return False, f"색·번호가 원문과 다름 {mentions(out)} ⊄ {mentions(src)}"
    if len(out) > len(src) + 20: return False, "원문보다 김"
    return True, ""
