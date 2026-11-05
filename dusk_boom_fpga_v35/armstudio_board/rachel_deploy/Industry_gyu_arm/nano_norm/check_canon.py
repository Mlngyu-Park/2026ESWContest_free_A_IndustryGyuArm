#!/usr/bin/env python3
"""말 정리기 '정답 꼴' 이 RACHEL 에서 제대로 블록이 되는가 (말투 엔진이 만든 정답 표본으로).

  python check_canon.py 2000          # 앱의 rachel_cpp/rachel + rachel.bin

장면은 정답 문장에서 거꾸로 만든다. RACHEL 은 도킹 번호 2~14 만 배웠으므로 그 범위만 쓴다.
"""
import os, re, subprocess, sys, random
from collections import Counter
HERE = os.path.dirname(os.path.abspath(__file__)); APP = os.path.dirname(HERE)
sys.path.insert(0, APP); sys.path.insert(0, HERE)
from nano_llm import hints
import gen_norm as G

random.seed(5)
CI = {'빨간색': '빨강', '주황색': '주황', '노란색': '노랑', '초록색': '초록', '파란색': '파랑', '보라색': '보라'}
CID = {'빨강': 'd3', '주황': 'd5', '노랑': 'd7', '초록': 'd9', '파랑': 'd11', '보라': 'd13'}
PADJ = {'빨간': '빨강', '주황': '주황', '노란': '노랑', '초록': '초록', '파란': '파랑', '보라': '보라'}

def pid(tok):
    m = re.match(r'(\d+)번 도킹', tok)
    return f'd{m.group(1)}' if m else CID[CI[tok]]

def main():
    want_n = int(sys.argv[1]) if len(sys.argv) > 1 else 1000
    p = subprocess.Popen([f'{APP}/rachel_cpp/rachel', f'{APP}/rachel_cpp/rachel.bin', '--serve'],
                         stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    assert p.stdout.readline().strip() == b'READY'
    res = Counter(); bad = []; n = 0
    while n < want_n:
        s, c, k = G.sample()
        if k not in ('1단계', '2단계', '3~5단계', '정정'): continue
        if any(not (2 <= int(x) <= 14) for x in re.findall(r'(\d+)번', c)): continue
        start = None
        mn = re.match(r'^(\d+)번 도킹의 ', c); mm = re.match(r'^(\S+?) 도킹의 ', c)
        if mn: start = f'd{mn.group(1)}'
        elif mm: start = CID[CI[mm.group(1)]]
        rest = c.split('펜을 ', 1)[1]
        stops = [pid(x.strip()) for x in re.split(r'(?:으로|로) 옮긴 다음 |(?:으로|로) 옮겨$', rest) if x.strip()]
        pw = re.search(r'의 (\S+) 펜을|^(\S+) 펜을', c)
        pcol = PADJ.get((pw.group(1) or pw.group(2))) if pw else None
        pcol = pcol or '초록'
        home = start or random.choice([d for d in CID.values() if d != stops[0]])
        docks = list(dict.fromkeys([home] + stops))
        used = set(); cmap = {}
        for d in docks:                         # 번호 도킹은 색이 겹치지 않게 아무 색이나
            col = next((k2 for k2, v in CID.items() if v == d), None)
            if col is None or col in used:
                col = random.choice([x for x in CID if x not in used] or list(CID))
            used.add(col); cmap[d] = col
        hdr = ' '.join(f'{col}={d}' for d, col in cmap.items()) + f' | 펜={pcol}@{home}'
        h = hints.build(cmap, home, pcol, c)
        pr = (f'{hdr}\n힌트 {h}' if h else hdr) + f'\n{c}\n>'
        p.stdin.write(pr.replace('\n', '\x01').encode() + b'\n'); p.stdin.flush()
        raw = p.stdout.readline().decode('utf-8', 'replace')
        bl = re.findall(r'@(?:goto|grasp|release):[A-Za-z0-9_#\-]+', raw)
        want = [f'@goto:{home}']
        for d in stops: want += ['@grasp:ppap', f'@goto:{d}', f'@release:{d}']
        ok = bl == want
        key = f"{len(stops)}단계" + (' 출발있음' if start else ' 출발없음')
        res[(key, ok)] += 1; n += 1
        if not ok and len(bad) < 5: bad.append((c, hdr, h, bl[:9]))
    p.stdin.write(b'QUIT\n'); p.stdin.flush(); p.wait()
    tot = sum(v for (k2, o), v in res.items() if o)
    print(f'전체 {tot}/{n} ({tot / n * 100:.1f}%)')
    for k2 in sorted({k2 for k2, _ in res}):
        print(f'  {k2:16s} {res[(k2, True)]}/{res[(k2, True)] + res[(k2, False)]}')
    for b in bad: print(' X', b[0], '|', b[1], '| 힌트', b[2], '|', b[3])

if __name__ == '__main__':
    main()
