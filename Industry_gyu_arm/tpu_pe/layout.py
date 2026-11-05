#!/usr/bin/env python3
"""
PE 2개 만능기판 배치도 + 배선도 생성기

WIRING.md 의 핀 정보를 그대로 코드로 옮긴 것이다.
핀 번호는 전부 공식 데이터시트로 확인했다 (2026-09-14):
  SST39SF010A : Microchip DS20005022F p.3 Figure 2-1
  74HC283     : Philips Rev.03 p.4 Fig 5
  74HC574     : Nexperia Rev.10 p.3 Table 2
  74HC74      : Nexperia Rev.9 p.3

쓰는 법:
    python3 layout.py            # layout.svg + 배선목록.txt 생성
    python3 layout.py --check    # 배선 검증만

만능기판은 2.54mm(0.1인치) 격자다. 좌표 (col,row) 는 구멍 번호다.
"""
import io, sys, os

HERE = os.path.dirname(os.path.abspath(__file__))
PITCH = 2.54  # mm, 만능기판 구멍 간격

# ─────────────────────────────────────────────────────────────
# 1. 칩 정의 — 핀 이름 → 핀 번호 (데이터시트 확인됨)
# ─────────────────────────────────────────────────────────────
CHIPS = {
    # SST39SF010A DIP-32, 600mil (12칸 폭)
    'SST39SF010': {
        'pins': 32, 'width': 12,
        'map': {
            'NC1':1, 'A16':2, 'A15':3, 'A12':4, 'A7':5, 'A6':6, 'A5':7, 'A4':8,
            'A3':9, 'A2':10, 'A1':11, 'A0':12, 'DQ0':13, 'DQ1':14, 'DQ2':15,
            'VSS':16, 'DQ3':17, 'DQ4':18, 'DQ5':19, 'DQ6':20, 'DQ7':21,
            'CE#':22, 'A10':23, 'OE#':24, 'A11':25, 'A9':26, 'A8':27,
            'A13':28, 'A14':29, 'NC30':30, 'WE#':31, 'VDD':32,
        },
        'vcc':'VDD', 'gnd':'VSS',
    },
    # 74HC574 DIP-20, 300mil (3칸 폭)
    '74HC574': {
        'pins': 20, 'width': 3,
        'map': {
            'OE#':1, 'D0':2, 'D1':3, 'D2':4, 'D3':5, 'D4':6, 'D5':7, 'D6':8,
            'D7':9, 'GND':10, 'CP':11, 'Q7':12, 'Q6':13, 'Q5':14, 'Q4':15,
            'Q3':16, 'Q2':17, 'Q1':18, 'Q0':19, 'VCC':20,
        },
        'vcc':'VCC', 'gnd':'GND',
    },
    # 74HC283 DIP-16, 300mil
    '74HC283': {
        'pins': 16, 'width': 3,
        'map': {
            'S2':1, 'B2':2, 'A2':3, 'S1':4, 'A1':5, 'B1':6, 'CIN':7, 'GND':8,
            'COUT':9, 'S4':10, 'B4':11, 'A4':12, 'S3':13, 'A3':14, 'B3':15,
            'VCC':16,
        },
        'vcc':'VCC', 'gnd':'GND',
    },
    # 74HC74 DIP-14, 300mil  ★2번 FF 는 CP=11, D=12 (1번과 반대!)
    '74HC74': {
        'pins': 14, 'width': 3,
        'map': {
            '1RD':1, '1D':2, '1CP':3, '1SD':4, '1Q':5, '1nQ':6, 'GND':7,
            '2nQ':8, '2Q':9, '2SD':10, '2CP':11, '2D':12, '2RD':13, 'VCC':14,
        },
        'vcc':'VCC', 'gnd':'GND',
    },
    '74HC244': {
        'pins': 20, 'width': 3,
        'map': {
            '1OE#':1, '1A1':2, '2Y4':3, '1A2':4, '2Y3':5, '1A3':6, '2Y2':7,
            '1A4':8, '2Y1':9, 'GND':10, '2A1':11, '1Y4':12, '2A2':13,
            '1Y3':14, '2A3':15, '1Y2':16, '2A4':17, '1Y1':18, '2OE#':19,
            'VCC':20,
        },
        'vcc':'VCC', 'gnd':'GND',
    },
}


class Part:
    """기판 위에 놓인 칩 하나. (col,row) 는 1번 핀의 구멍 위치."""
    def __init__(self, ref, kind, col, row, note=''):
        self.ref, self.kind, self.col, self.row, self.note = ref, kind, col, row, note
        self.spec = CHIPS[kind]

    @property
    def rows(self):
        return self.spec['pins'] // 2

    def pin_num(self, name):
        n = self.spec['map'].get(name)
        if n is None:
            raise KeyError(f"{self.ref}({self.kind}) 에 '{name}' 핀이 없다")
        return n

    def hole(self, pin):
        """DIP 핀번호 → 기판 구멍 (col,row).
        1번이 좌상단, 아래로 내려가 절반에서 오른쪽 열로 올라온다."""
        half = self.rows
        if pin <= half:
            return (self.col, self.row + pin - 1)
        return (self.col + self.spec['width'], self.row + (2 * half - pin))

    def hole_of(self, name):
        return self.hole(self.pin_num(name))

    def bbox(self):
        return (self.col, self.row, self.col + self.spec['width'], self.row + self.rows - 1)


# ─────────────────────────────────────────────────────────────
# 2. 배치 — 만능기판 위 실제 자리
#    신호가 왼→오른쪽으로 흐르게 놓았다. x 가 흐르는 방향이다.
# ─────────────────────────────────────────────────────────────
def build():
    P = []
    def add(*a, **k):
        p = Part(*a, **k); P.append(p); return p

    parts = {}
    # 공통부 — 왼쪽 끝
    parts['CLK']  = add('U_CLK', '74HC74',  2,  2, '클럭 2분주')
    parts['BUF']  = add('U_BUF', '74HC244', 2, 12, '클럭 버퍼')

    # PE0 / PE1 — 같은 모양을 오른쪽으로 복사
    # ROM 은 DIP-32 라 16행을 먹는다. 그 아래로 가산기를 깐다.
    for i, base in enumerate((10, 34)):
        t = f'PE{i}'
        parts[f'{t}_W']   = add(f'{t}_U2', '74HC574', base,      2,  'w 래치')
        parts[f'{t}_X']   = add(f'{t}_U3', '74HC574', base,      14, 'x 래치')
        parts[f'{t}_ROM'] = add(f'{t}_U1', 'SST39SF010', base+5, 2,  '곱셈 LUT')
        # 가산기 4개를 2×2 로 — ROM 아래, 캐리는 U4→U5→U6→U7
        for j in range(4):
            parts[f'{t}_ADD{j}'] = add(f'{t}_U{4+j}', '74HC283',
                                        base + (0 if j % 2 == 0 else 6),
                                        26 + (j // 2) * 10,
                                        f'가산기 b{j*4}~{j*4+3}')
        parts[f'{t}_PL'] = add(f'{t}_U8', '74HC574', base+13, 26, 'psum 하위')
        parts[f'{t}_PH'] = add(f'{t}_U9', '74HC574', base+13, 36, 'psum 상위')
    return P, parts


# ─────────────────────────────────────────────────────────────
# 3. 배선 — WIRING.md 의 표를 그대로
# ─────────────────────────────────────────────────────────────
def build_nets(pt):
    """(from_part, from_pin, to_part, to_pin, 그룹) 목록"""
    N = []
    def w(a, ap, b, bp, g):
        N.append((a, ap, b, bp, g))

    # 클럭 분주: /Q 를 자기 D 로 되먹임
    clk = pt['CLK']
    w(clk,'1nQ', clk,'1D',  '클럭')      # 1단 2분주
    w(clk,'1nQ', clk,'2CP', '클럭')      # 1단 → 2단 클럭
    w(clk,'2nQ', clk,'2D',  '클럭')      # 2단 2분주

    for i in range(2):
        t = f'PE{i}'
        W, X, R = pt[f'{t}_W'], pt[f'{t}_X'], pt[f'{t}_ROM']
        A = [pt[f'{t}_ADD{j}'] for j in range(4)]
        PL, PH = pt[f'{t}_PL'], pt[f'{t}_PH']

        # ① w 래치 → ROM 주소 상위 A8~A15
        for b in range(8):
            w(W, f'Q{b}', R, f'A{8+b}', f'{t} w→ROM')

        # ② x 래치 → ROM 주소 하위 A0~A7
        for b in range(8):
            w(X, f'Q{b}', R, f'A{b}', f'{t} x→ROM')

        # ③ 위상 신호 → ROM A16
        w(clk, '1Q', R, 'A16', f'{t} 위상')

        # ④ ROM 출력 → 가산기 A 입력 (하위 8비트는 ADD0/ADD1)
        for b in range(8):
            tgt = A[b // 4]
            w(R, f'DQ{b}', tgt, f'A{b % 4 + 1}', f'{t} ROM→가산기')

        # ⑤ 캐리 체인
        w(A[0], 'CIN', None, 'GND', f'{t} 캐리')
        for j in range(3):
            w(A[j], 'COUT', A[j+1], 'CIN', f'{t} 캐리')

        # ⑥ 가산기 출력 → psum 래치
        for b in range(16):
            src = A[b // 4]
            dst = PL if b < 8 else PH
            w(src, f'S{b % 4 + 1}', dst, f'D{b % 8}', f'{t} 가산기→psum')

        # ⑦ psum_in → 가산기 B 입력
        if i == 0:
            for b in range(16):
                w(A[b // 4], f'B{b % 4 + 1}', None, 'GND', f'{t} psum_in=0')
        else:
            prevL, prevH = pt['PE0_PL'], pt['PE0_PH']
            for b in range(16):
                src = prevL if b < 8 else prevH
                w(src, f'Q{b % 8}', A[b // 4], f'B{b % 4 + 1}', f'{t} psum_in')

        # ⑧ ROM 제어
        w(R, 'CE#', None, 'GND', f'{t} ROM제어')
        w(R, 'OE#', None, 'GND', f'{t} ROM제어')
        w(R, 'WE#', None, 'VCC', f'{t} ROM제어')

        # ⑨ 래치 제어
        for L in (W, X, PL, PH):
            w(L, 'OE#', None, 'GND', f'{t} 래치제어')
        for L in (X, PL, PH):
            w(clk, '2Q', L, 'CP', f'{t} 클럭')

    # ⑩ PE0 x_out → PE1 x_in (시스톨릭 핵심)
    for b in range(8):
        w(pt['PE0_X'], f'Q{b}', pt['PE1_X'], f'D{b}', '★ x 흐름')

    # 74HC74 셋/리셋 고정
    for p in ('1RD','1SD','2RD','2SD'):
        w(clk, p, None, 'VCC', '클럭')
    return N


# ─────────────────────────────────────────────────────────────
# 4. SVG 그리기
# ─────────────────────────────────────────────────────────────
COLORS = {
    'w→ROM':'#d62728', 'x→ROM':'#1f77b4', 'ROM→가산기':'#2ca02c',
    '캐리':'#ff7f0e', '가산기→psum':'#9467bd', 'psum_in':'#8c564b',
    '★ x 흐름':'#e377c2', '클럭':'#17becf', 'ROM제어':'#7f7f7f',
    '래치제어':'#bcbd22', 'psum_in=0':'#7f7f7f', '위상':'#17becf',
}

def color_of(group):
    for k, v in COLORS.items():
        if group.endswith(k):
            return v
    return '#333'


def svg(parts, nets, path):
    S = 14.0          # 구멍 하나 = 14px
    MX, MY = 70, 120  # 여백
    cols = max(p.bbox()[2] for p in parts) + 4
    rows = max(p.bbox()[3] for p in parts) + 4
    W, H = MX*2 + cols*S, MY + MY//2 + rows*S

    def X(c): return MX + c*S
    def Y(r): return MY + r*S

    o = []
    o.append(f'<svg xmlns="http://www.w3.org/2000/svg" width="{W:.0f}" height="{H:.0f}" '
             f'viewBox="0 0 {W:.0f} {H:.0f}" font-family="NanumGothic, Noto Sans CJK KR, DejaVu Sans, sans-serif">')
    o.append(f'<rect width="{W:.0f}" height="{H:.0f}" fill="#fbfbf7"/>')

    # 제목
    o.append(f'<text x="{MX}" y="42" font-size="26" font-weight="bold" fill="#111">'
             f'PE 2개 — 만능기판 배치도</text>')
    o.append(f'<text x="{MX}" y="66" font-size="14" fill="#555">'
             f'구멍 {cols}×{rows} (2.54mm 격자) = 약 {cols*2.54:.0f}×{rows*2.54:.0f}mm '
             f'· 칩 {len(parts)}개 · 배선 {len(nets)}가닥</text>')
    o.append(f'<text x="{MX}" y="86" font-size="13" fill="#a00" font-weight="bold">'
             f'핀 번호는 전부 공식 데이터시트 확인됨 (DS20005022F / Philips Rev.03 / Nexperia Rev.10·9)</text>')

    # 만능기판 구멍
    o.append('<g fill="#ddd">')
    for c in range(cols):
        for r in range(rows):
            o.append(f'<circle cx="{X(c):.0f}" cy="{Y(r):.0f}" r="1.6"/>')
    o.append('</g>')

    # 전원 레일
    for r, (lab, col) in ((rows-2, ('+5V', '#c00')), (rows-1, ('GND', '#000'))):
        o.append(f'<line x1="{X(0):.0f}" y1="{Y(r):.0f}" x2="{X(cols-1):.0f}" y2="{Y(r):.0f}" '
                 f'stroke="{col}" stroke-width="3.5" opacity="0.8"/>')
        o.append(f'<text x="{X(0)-42:.0f}" y="{Y(r)+5:.0f}" font-size="13" '
                 f'font-weight="bold" fill="{col}">{lab}</text>')

    # 칩 몸통
    for p in parts:
        x0, y0, x1, y1 = p.bbox()
        px, py = X(x0), Y(y0)
        pw, ph = (x1-x0)*S, (y1-y0)*S
        o.append(f'<rect x="{px-7:.0f}" y="{py-7:.0f}" width="{pw+14:.0f}" height="{ph+14:.0f}" '
                 f'rx="4" fill="#3a3a3a" opacity="0.72" stroke="#000"/>')
        # 1번 핀 표시 (반원 홈)
        o.append(f'<circle cx="{px:.0f}" cy="{py:.0f}" r="4" fill="#fff" opacity="0.9"/>')
        o.append(f'<text x="{px+pw/2:.0f}" y="{py+ph/2-4:.0f}" font-size="12" fill="#fff" '
                 f'text-anchor="middle" font-weight="bold">{p.ref}</text>')
        o.append(f'<text x="{px+pw/2:.0f}" y="{py+ph/2+10:.0f}" font-size="9" fill="#bbb" '
                 f'text-anchor="middle">{p.kind}</text>')
        o.append(f'<text x="{px+pw/2:.0f}" y="{py-12:.0f}" font-size="10" fill="#333" '
                 f'text-anchor="middle">{p.note}</text>')
        # 핀 번호
        for n in range(1, p.spec['pins']+1):
            hx, hy = p.hole(n)
            left = n <= p.rows
            tx = X(hx) + (-11 if left else 11)
            o.append(f'<text x="{tx:.0f}" y="{Y(hy)+3:.0f}" font-size="7" fill="#666" '
                     f'text-anchor="{"end" if left else "start"}">{n}</text>')

    # 배선
    o.append('<g stroke-width="1.7" fill="none" opacity="0.9">')
    for a, ap, b, bp, g in nets:
        if b is None:
            continue
        x1, y1 = a.hole_of(ap); x2, y2 = b.hole_of(bp)
        c = color_of(g)
        mx = (X(x1) + X(x2)) / 2
        o.append(f'<path d="M {X(x1):.0f} {Y(y1):.0f} L {mx:.0f} {Y(y1):.0f} '
                 f'L {mx:.0f} {Y(y2):.0f} L {X(x2):.0f} {Y(y2):.0f}" stroke="{c}"/>')
    o.append('</g>')

    # 범례
    ly = H - 44
    o.append(f'<text x="{MX}" y="{ly-8:.0f}" font-size="12" font-weight="bold" fill="#111">배선 색</text>')
    seen, lx = [], MX
    for g in ('w→ROM','x→ROM','ROM→가산기','캐리','가산기→psum','psum_in','★ x 흐름','클럭'):
        c = COLORS[g]
        o.append(f'<line x1="{lx:.0f}" y1="{ly+8:.0f}" x2="{lx+22:.0f}" y2="{ly+8:.0f}" '
                 f'stroke="{c}" stroke-width="3"/>')
        o.append(f'<text x="{lx+27:.0f}" y="{ly+12:.0f}" font-size="11" fill="#333">{g}</text>')
        lx += 27 + len(g)*11 + 18
    o.append('</svg>')
    io.open(path, 'w', encoding='utf-8').write('\n'.join(o))


# ─────────────────────────────────────────────────────────────
# 5. 배선 목록 (납땜하면서 하나씩 지워가는 용도)
# ─────────────────────────────────────────────────────────────
def netlist(parts, nets, path):
    L = []
    L.append("PE 2개 — 납땜 배선 목록")
    L.append("=" * 64)
    L.append("핀 번호 전부 공식 데이터시트 확인됨 (2026-09-14)")
    L.append("한 줄 납땜할 때마다 [ ] 에 표시하세요.\n")

    L.append("[ 칩 배치 ]")
    L.append(f"{'칩':<10}{'종류':<14}{'1번핀 위치':<14}{'역할'}")
    L.append("-" * 64)
    for p in sorted(parts, key=lambda q: (q.col, q.row)):
        L.append(f"{p.ref:<10}{p.kind:<14}({p.col:>2},{p.row:>2}) 칸{'':<3}{p.note}")

    # 전원
    L.append(f"\n[ 전원 — 먼저 한다. 칩 {len(parts)}개 × 2가닥 = {len(parts)*2}가닥 ]")
    L.append("-" * 64)
    for p in sorted(parts, key=lambda q: (q.col, q.row)):
        v, g = p.spec['vcc'], p.spec['gnd']
        L.append(f"[ ] {p.ref:<10} {p.pin_num(v):>2}번({v}) → +5V 레일")
        L.append(f"[ ] {p.ref:<10} {p.pin_num(g):>2}번({g}) → GND 레일")
    L.append(f"\n[ ] 0.1uF 커패시터 {len(parts)}개 — 칩마다 VCC-GND 사이, 칩 바로 옆에")
    L.append("[ ] 10uF 전해 2~4개 — 기판 전원 입구")

    # 그룹별 신호선
    groups = {}
    for a, ap, b, bp, g in nets:
        groups.setdefault(g, []).append((a, ap, b, bp))

    total = 0
    for g in sorted(groups, key=lambda k: (k.split()[0], k)):
        items = groups[g]
        L.append(f"\n[ {g} ]  {len(items)}가닥")
        L.append("-" * 64)
        for a, ap, b, bp in items:
            src = f"{a.ref} {a.pin_num(ap):>2}번({ap})"
            if b is None:
                dst = f"→ {bp} 레일"
            else:
                dst = f"→ {b.ref} {b.pin_num(bp):>2}번({bp})"
            L.append(f"[ ] {src:<26}{dst}")
            total += 1

    L.append("\n" + "=" * 64)
    L.append(f"신호선 {total}가닥 + 전원 {len(parts)*2}가닥 = 총 {total + len(parts)*2}가닥")
    L.append(f"납땜 지점 약 {(total + len(parts)*2) * 2}곳")
    io.open(path, 'w', encoding='utf-8').write('\n'.join(L) + '\n')
    return total


# ─────────────────────────────────────────────────────────────
# 6. 검증 — 배치와 배선이 말이 되는지
# ─────────────────────────────────────────────────────────────
def veroroute(parts, nets, path):
    """VeroRoute 가 읽는 Protel1/Tango 넷리스트를 만든다.

    형식 (VeroRoute 개발자 설명 그대로):
        부품:  [ 다음 6줄 — RefDes / Value / Footprint / 빈줄 3개 ]
        연결:  ( 다음 넷이름, 그리고 RefDes-핀번호 들 )

    풋프린트는 VeroRoute 가 아는 이름만 쓴다: DIP14, DIP16, DIP20, DIP32.

    ★ 전원은 여기서 진짜 넷으로 펼친다.
      배선목록.txt 는 "+5V 레일" 이라고만 적어도 사람이 알아보지만,
      VeroRoute 는 어느 핀이 전원인지 일일이 알려줘야 한다."""
    FOOT = {14:'DIP14', 16:'DIP16', 20:'DIP20', 32:'DIP32'}
    L = []

    # ── 부품 ──
    # ★ 줄 순서: RefDes / 풋프린트 / 값
    #   (VeroRoute 예제 examples/tinycad_netlist.net 로 확인함.
    #    포럼 글에는 값이 먼저라고 돼 있으나 실제 예제는 풋프린트가 먼저다.)
    for p in sorted(parts, key=lambda q: (q.col, q.row)):
        L += ['[', p.ref, FOOT[p.spec['pins']], p.kind, '', '', '', ']', '']

    # 0.1uF 디커플링 — 칩마다 하나
    caps = []
    for p in sorted(parts, key=lambda q: (q.col, q.row)):
        ref = 'C' + p.ref.replace('_', '')
        caps.append((ref, p))
        L += ['[', ref, 'CAP_CERAMIC', '100n', '', '', '', ']', '']

    # ── 넷 모으기 ──
    from collections import defaultdict
    nodes = defaultdict(list)      # 넷이름 → [(ref, 핀번호)]

    # 전원: 모든 칩의 VCC/GND + 커패시터 양끝
    for p in parts:
        nodes['VCC'].append((p.ref, p.pin_num(p.spec['vcc'])))
        nodes['GND'].append((p.ref, p.pin_num(p.spec['gnd'])))
    for ref, p in caps:
        nodes['VCC'].append((ref, 1))
        nodes['GND'].append((ref, 2))

    # 신호선 — 같은 출력에서 나온 것끼리 한 넷으로 묶는다
    sig = defaultdict(list)
    for a, ap, b, bp, g in nets:
        if b is None:
            # GND/VCC 로 묶이는 입력핀 (캐리 시작, psum_in=0, OE# 등)
            nodes[bp].append((a.ref, a.pin_num(ap)))
        else:
            sig[(a.ref, ap)].append((b.ref, b.pin_num(bp)))

    for (aref, ap), dests in sig.items():
        src = None
        for p in parts:
            if p.ref == aref:
                src = p
        name = f'{aref}_{ap}'.replace('#', 'n')
        nodes[name] = [(aref, src.pin_num(ap))] + dests

    # ── 넷 출력 ──
    for name in sorted(nodes, key=lambda k: (k not in ('VCC','GND'), k)):
        pins = nodes[name]
        L.append('(')
        L.append(name)
        seen = set()
        for ref, pin in pins:
            k = f'{ref}-{pin}'
            if k not in seen:
                L.append(k)
                seen.add(k)
        L += [')', '']

    io.open(path, 'w', encoding='utf-8').write('\n'.join(L) + '\n')
    return len(nodes), len(parts) + len(caps)


def check(parts, nets):
    errs, warns = [], []

    # 칩끼리 겹치는가
    for i, a in enumerate(parts):
        ax0, ay0, ax1, ay1 = a.bbox()
        for b in parts[i+1:]:
            bx0, by0, bx1, by1 = b.bbox()
            if not (ax1 < bx0 or bx1 < ax0 or ay1 < by0 or by1 < ay0):
                errs.append(f"칩 겹침: {a.ref} 과 {b.ref}")

    # 핀 이름이 실제로 있는가 + 같은 입력에 두 번 물리는가
    driven = {}
    for a, ap, b, bp, g in nets:
        try:
            a.pin_num(ap)
            if b is not None:
                b.pin_num(bp)
        except KeyError as e:
            errs.append(str(e)); continue
        if b is not None:
            key = (b.ref, bp)
            if key in driven and driven[key] != (a.ref, ap):
                errs.append(f"입력 충돌: {b.ref} {bp} 에 {driven[key]} 와 ({a.ref},{ap}) 둘 다")
            driven[key] = (a.ref, ap)

    # 모든 칩이 전원에 물리는가 (전원은 레일이라 nets 에 없음 — 존재만 확인)
    for p in parts:
        for k in ('vcc', 'gnd'):
            try:
                p.pin_num(p.spec[k])
            except KeyError:
                errs.append(f"{p.ref} 전원핀 정의 없음")

    # ROM 주소선 17개가 전부 연결됐는가
    for i in range(2):
        rom = None
        for p in parts:
            if p.ref == f'PE{i}_U1':
                rom = p
        if rom:
            need = {f'A{b}' for b in range(17)}
            got = {bp for a, ap, b, bp, g in nets if b is rom and bp.startswith('A')}
            miss = need - got
            if miss:
                errs.append(f"PE{i} ROM 주소선 빠짐: {sorted(miss)}")

    return errs, warns


def main():
    parts, pt = build()
    nets = build_nets(pt)
    errs, warns = check(parts, nets)

    print(f"칩 {len(parts)}개, 배선 {len(nets)}건")
    if errs:
        print("\n!!! 문제 발견 !!!")
        for e in errs:
            print("  -", e)
    else:
        print("검증 통과: 칩 안 겹침, 핀 이름 전부 실재, 입력 충돌 없음, ROM 주소선 17개 완비")
    for w in warns:
        print("  주의:", w)

    if '--check' in sys.argv:
        return 1 if errs else 0

    svg_path = os.path.join(HERE, 'layout.svg')
    txt_path = os.path.join(HERE, '배선목록.txt')
    svg(parts, nets, svg_path)
    n = netlist(parts, nets, txt_path)
    vr_path = os.path.join(HERE, 'pe2.net')
    nn, npart = veroroute(parts, nets, vr_path)
    print(f"\n  {svg_path}")
    print(f"  {txt_path}  (신호 {n}가닥 + 전원 {len(parts)*2}가닥)")
    print(f"  {vr_path}   VeroRoute 용 — 부품 {npart}개, 넷 {nn}개")
    return 1 if errs else 0


if __name__ == '__main__':
    sys.exit(main())
