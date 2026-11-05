#!/usr/bin/env python3
"""FIXER 회귀 검사 — 고쳐야 할 것은 고치고, 건드리면 안 될 것은 그대로.

  python test_fixer.py
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fixer
fixer.load()
GOOD = [  # 고쳐야 하는 것
 ('보라색 도킹 스테이션에 있는 카페 스테이션에 있는 초록색 펜을 잡아서 빨간색으로 이동한 다음에 주황 운동한 다음에 주황색으로 다시 옮기는 거 하고 싶어.','겹침중복'),
 ('어 그 초록색 색 색 색 펜을 음 잡아서 빨간 도킹이로 옮겨줘.','반복+잔말'),
 ('토킹 스테이선에 있는 팬이 잡아써 빨깐 자리로 옴기는 거','낱말오인'),
]
KEEP = [  # 건드리면 안 되는 것
 '초록 펜을 빨간 도킹으로 옮겨줘',
 '노란색 자리에 있는 부품 잡아서 파란 도킹으로 옮긴 다음에 다시 보라색으로 옮겨줘',
 '빨간 도킹으로 옮긴 다음에 노란 도킹으로 옮기고 파란 도킹으로 옮겨줘',
 '하강','상승','그리퍼 열어','초록 펜 잡아',
 '2번 도킹에 있는 펜을 잡아서 5번 도킹으로 옮겨줘',
 '주임','감사합니다','아 그 뭐지','네 알겠습니다',
]
ok=0; tot=0
print('[고쳐야 하는 것]')
for c,why in GOOD:
    r=fixer.fix(c); tot+=1
    good = r!=c
    ok+=good
    print(f'  {"OK " if good else "★안고침"} ({why}) {r[:56]}')
print('[건드리면 안 되는 것]')
for c in KEEP:
    r=fixer.fix(c); tot+=1
    good = r==c
    ok+=good
    print(f'  {"OK " if good else "★바뀜"} {c[:40]!r}' + ('' if good else f' -> {r[:40]!r}'))
print()
print(f'{ok}/{tot} 통과')
