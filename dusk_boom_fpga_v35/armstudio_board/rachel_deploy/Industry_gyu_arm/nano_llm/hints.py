"""색·번호 언급 -> 도킹 id 힌트 (결정론적 전처리).

색→도킹 조회는 사전 찾기라 계산으로 완벽히 된다. 신경망에 외우게 하지 않는다.
문장에 나온 순서대로 id 를 나열해 주면, 모델은 '어느 것이 출발/경유/도착인가'
라는 언어 구조 판단만 하면 된다. 같은 함수를 앱 서버에서도 그대로 쓴다.
"""
import re

# 색 표기 변형 -> 표준색
# 색 표기 변형 -> 표준색.
#   실사용에서 "오렌지 색으로 옮기자" 가 통째로 무시됐다 —
#   목록에 없는 말은 힌트에 안 잡히고, 그러면 모델도 못 찾는다.
#   긴 것부터 매칭되게 정렬해서 쓴다 ("빨간색" 이 "빨간" 보다 먼저).
FORMS = {"빨간":"빨강","빨강":"빨강","붉은":"빨강","레드":"빨강",
         "주황":"주황","오렌지":"주황","주홍":"주황",
         "노란":"노랑","노랑":"노랑","옐로":"노랑","노르스름":"노랑",
         "초록":"초록","녹색":"초록","그린":"초록","연두":"초록",
         "파란":"파랑","파랑":"파랑","블루":"파랑","푸른":"파랑",
         "보라":"보라","퍼플":"보라","자주":"보라"}
PEN_WORDS = ("펜", "부품", "물체", "펜대")

def build(cmap, home, pcol, text):
    """cmap: {dock_id: 색}, home: 펜이 있는 도킹, pcol: 펜 색.
    반환: 문장에 언급된 순서의 도킹 id 목록 (예: "d7 d13 d2")."""
    c2d = {c: d for d, c in cmap.items()}
    found = []                      # (위치, id)
    # 긴 표기부터 찾아야 "오렌지" 가 통째로 잡힌다
    _pat = "|".join(sorted(FORMS, key=len, reverse=True))
    for m in re.finditer(_pat, text):
        std = FORMS[m.group()]
        tail = text[m.end():m.end()+5]
        if any(w in tail for w in PEN_WORDS):
            if std == pcol: found.append((m.start(), home))   # "초록 펜" = 펜 위치
            continue                                          # 색이 달라도 펜 얘기
        if std in c2d: found.append((m.start(), c2d[std]))
    for m in re.finditer(r"(\d+)\s*번", text):
        d = f"d{m.group(1)}"
        if d in cmap: found.append((m.start(), d))
    found.sort()
    out = []
    for _, d in found:
        if not out or out[-1] != d: out.append(d)             # 연속 중복만 제거
    return " ".join(out)
