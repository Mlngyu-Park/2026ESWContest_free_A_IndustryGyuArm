#!/usr/bin/env python3
"""
PE 2개 (새 모양: 플래시 2개/PE, 캐리세이브) 를 KYLE 언어로 쓴다 → pe2.txt

★ 이 파일이 "AI 가 배선하는" 부분이다.
   여기에는 연결(pin REF[핀:넷])만 적는다. 전압·시간은 적지 않는다.
   검증은 sol.py 가 chips.txt 숫자로 따로 한다.

새 모양 (옛 모양과의 차이):
  - 플래시 LO/HI 2개가 같은 주소(w,x)를 받아 16비트 곱을 한 번에 낸다
  - 가산기 캐리를 하위(U4→U5)·상위(U6→U7) 두 줄로 나눈다 (4단 → 2단)
  - 74HC244 로 클럭을 나눠 뿌린다 (옛 배선에선 244 가 붙어 있지 않았다 → 플로팅)
  - 74HC04 로 오실레이터/MEGA 수동클럭 중 하나를 골라 74HC74 에 넣는다 (HDR3 점퍼)
  - 74HC04 안 쓰는 입력은 GND (CMOS 입력을 띄우면 전류가 샌다)
"""
import io, os, sys

HERE = os.path.dirname(os.path.abspath(__file__))
OUT  = os.environ.get('OUT') or os.path.join(HERE, 'pe2.txt')   # OUT=경로 로 다른 파일에 쓸 수 있다 (space.py 가 쓴다)

# ── 기판 설정 (환경변수로 조절: BOARD_W, BOARD_H, LAYERS) ──
BW = int(os.environ.get('BOARD_W', 84))      # 가로 구멍
BH = int(os.environ.get('BOARD_H', 62))      # 세로 구멍
LAYERS = int(os.environ.get('LAYERS', 1))    # 층수 (아파트 구조)
PES    = int(os.environ.get('PES', 2))       # PE 개수 (기판 한 장에 몇 개 올릴지)

L = []          # 출력 줄
pins = {}       # (ref, pinnum) -> net  (같은 다리에 두 번 붙이면 즉시 오류)
groups = {}     # net -> group
nnet = [100]    # 숫자 네트 카운터 (100부터. 0~99 는 사람이 손으로 쓸 때 쓰라고 비워둠)

def newnet():
    nnet[0] += 1
    return nnet[0]

def P(ref, pin, net, group=None):
    key = (ref, pin)
    if key in pins and pins[key] != net:
        raise SystemExit(f"!! {ref}[{pin}] 에 네트 {pins[key]} 와 {net} 둘 다 → 배선 실수")
    pins[key] = net
    if group:
        groups[net] = group

# 핀 이름 → 번호 (chips.txt 와 동일. 여기서 다시 적는 건 AI 가 '자기 배선'을 명시하기 위함)
HC574 = dict(OE=1, D=[2,3,4,5,6,7,8,9], GND=10, CP=11, Q=[19,18,17,16,15,14,13,12], VCC=20)
HC283 = dict(A=[5,3,14,12], B=[6,2,15,11], S=[4,1,13,10], CIN=7, COUT=9, GND=8, VCC=16)
FLASH = dict(A=[12,11,10,9,8,7,6,5, 27,26,23,25,4,28,29,3, 2,30,1],   # A0..A18
             DQ=[13,14,15,17,18,19,20,21], VSS=16, VDD=32, CE=22, OE=24, WE=31)
HC74  = dict(RD1=1, D1=2, CP1=3, SD1=4, Q1=5, nQ1=6, GND=7, nQ2=8, Q2=9, SD2=10, CP2=11, D2=12, RD2=13, VCC=14)
HC244 = dict(OE1=1, A1=[2,4,6,8], Y1=[18,16,14,12], OE2=19, A2=[17,15,13,11], Y2=[3,5,7,9], GND=10, VCC=20)
HC04  = dict(A=[1,3,5,9,11,13], Y=[2,4,6,8,10,12], GND=7, VCC=14)
OSC   = dict(EN=1, GND=7, OUT=8, VCC=14)
MEGA  = dict(D=[1,2,3,4,5,6,7,8], WCP0=9, WCP1=10, XCP=11, MCLK=12, RST=13,
             P=[14,15,16,17,18,19,20,21], PSEL=22, X0=23, GND=24, VCC=25)

# ───────────────────────── 부품 배치 ─────────────────────────
# 층 배분: 2층 이상이면 PE 를 층마다 하나씩 (같은 자리를 위아래로) → 기판이 옆으로 안 커진다
if LAYERS >= 2:
    PE_BASE = (12, 12)[:PES]; PE_LAYER = (1, 2)[:PES]      # 같은 자리를 층만 달리해서 쓴다
else:
    PE_BASE = (12, 50)[:PES]; PE_LAYER = (1, 1)[:PES]
parts = []      # (ref, chip, col, row, rot)
def part(ref, chip, col, row, rot=0, layer=1):
    parts.append((ref, chip, col, row, rot, layer))

# 공통부 (왼쪽)
part('OSC',   'OSC_FULL', 2, 2)
part('U_INV', '74HC04',   2, 12)
part('JP_CLK','HDR3',     2, 22)
part('U_CLK', '74HC74',   2, 26)
part('U_BUF', '74HC244',  2, 36)
part('MEGA',  'MEGA',     2, max(1, BH - 16))   # 핀헤더 30핀 (2열×15행). 가장자리 커넥터라 기판 아래쪽에 고정
NEAR = {}; FIXED = {'MEGA'}    # 배치 힌트 (KYLE 의 near / fixed 토큰으로 나간다)

# PE 2개 — 같은 모양 복사
for i, base in enumerate(PE_BASE):
    t = f'PE{i}'
    part(f'{t}_U2',  '74HC574',    base,    2, 0, PE_LAYER[i])    # w 래치
    part(f'{t}_U3',  '74HC574',    base,    14, 0, PE_LAYER[i])   # x 래치
    part(f'{t}_U1L', 'SST39SF040', base+7,  2, 0, PE_LAYER[i])    # LUT 하위
    part(f'{t}_U1H', 'SST39SF040', base+16, 2, 0, PE_LAYER[i])    # LUT 상위
    # 가산기는 산 74HC283 그대로. sol.py R11 (플래시 VOH 2.4V < HC VIH 3.15V) 은
    #   DQ 선마다 풀업 저항 4.7kΩ→VCC 로 해결한다 (아래 '풀업' 절). 부품 교체 불가 조건.
    ADDER = '74HC283'
    part(f'{t}_U4',  ADDER,        base,    26, 0, PE_LAYER[i])   # 가산기 b0-3
    part(f'{t}_U5',  ADDER,        base+7,  26, 0, PE_LAYER[i])   # b4-7
    part(f'{t}_U6',  ADDER,        base,    36, 0, PE_LAYER[i])   # b8-11
    part(f'{t}_U7',  ADDER,        base+7,  36, 0, PE_LAYER[i])   # b12-15
    part(f'{t}_U8',  '74HC574',    base+15, 26, 0, PE_LAYER[i])   # psum 하위
    part(f'{t}_U9',  '74HC574',    base+15, 36, 0, PE_LAYER[i])   # psum 상위

# ───────────────────────── 전원 ─────────────────────────
for ref, chip, *_ in parts:
    if chip == '74HC574': P(ref, HC574['VCC'], 'VCC'); P(ref, HC574['GND'], 'GND')
    if chip in ('74HC283','74HCT283'): P(ref, HC283['VCC'], 'VCC'); P(ref, HC283['GND'], 'GND')
    if chip == 'SST39SF040': P(ref, FLASH['VDD'], 'VCC'); P(ref, FLASH['VSS'], 'GND')
    if chip == '74HC74':  P(ref, HC74['VCC'], 'VCC');  P(ref, HC74['GND'], 'GND')
    if chip == '74HC244': P(ref, HC244['VCC'], 'VCC'); P(ref, HC244['GND'], 'GND')
    if chip == '74HC04':  P(ref, HC04['VCC'], 'VCC');  P(ref, HC04['GND'], 'GND')
    if chip == 'OSC_FULL':P(ref, OSC['VCC'], 'VCC');   P(ref, OSC['GND'], 'GND')
    if chip == 'MEGA':    P(ref, MEGA['VCC'], 'VCC');  P(ref, MEGA['GND'], 'GND')

# 디커플링 — 칩마다 0.1uF, 전원 입구 10uF ×4
caps = []
ci = 0
WIDTH = {'74HC574':3,'74HC283':3,'74HCT283':3,'74HC74':3,'74HC244':3,'74HC04':3,'SST39SF040':6,'OSC_FULL':3}
for ref, chip, col, row, rot, layer in parts:
    if chip in WIDTH:
        ci += 1
        cref = f'C{ci}'
        # VCC 핀은 우상단 (col+폭, row). 캡을 그 바로 옆 열(col+폭+1)에 세로로 → VCC 리드 2.5mm (R17 대책)
        caps.append((cref, 'CAP100n', col + WIDTH[chip] + 1, row, 0, layer)); NEAR[cref] = ref
        P(cref, 1, 'VCC'); P(cref, 2, 'GND')
for k in range(4):
    cref = f'CB{k+1}'
    caps.append((cref, 'CAP10u', max(6, BW - 12) + k*3, max(1, BH - 2), 0, 1)); FIXED.add(cref)   # 오른쪽 아래 (MEGA 는 왼쪽)
    P(cref, 1, 'VCC'); P(cref, 2, 'GND')

# ───────────────────────── 클럭 계통 ─────────────────────────
# OSC → 04.1A ; 04.1Y → JP.1        (오실레이터 쪽)
# MEGA.MCLK → 04.2A ; 04.2Y → JP.3  (수동 클럭 쪽)
# JP.2 → 74HC74 1CP                 (점퍼로 하나 선택)
n = newnet(); P('OSC', OSC['OUT'], n, '클럭'); P('U_INV', HC04['A'][0], n)
n = newnet(); P('U_INV', HC04['Y'][0], n, '클럭'); P('JP_CLK', 1, n)
n = newnet(); P('MEGA', MEGA['MCLK'], n, '클럭'); P('U_INV', HC04['A'][1], n)
n = newnet(); P('U_INV', HC04['Y'][1], n, '클럭'); P('JP_CLK', 3, n)
n = newnet(); P('JP_CLK', 2, n, '클럭'); P('U_CLK', HC74['CP1'], n)
# 04 안 쓰는 입력 3A~6A → GND
for a in HC04['A'][2:]:
    P('U_INV', a, 'GND')
P('OSC', OSC['EN'], 'NC')

# 74HC74 : 새 모양은 LUT 를 2번 읽지 않으므로 위상 신호가 필요 없다.
#          1단만 2분주로 쓴다 (오실레이터 24M → 12MHz PE 클럭). 2단은 예비(입력 고정).
n = newnet(); P('U_CLK', HC74['nQ1'], n, '클럭'); P('U_CLK', HC74['D1'], n)     # 2분주 되먹임
for p in ('RD1','SD1','RD2','SD2'):
    P('U_CLK', HC74[p], 'VCC')
P('U_CLK', HC74['D2'], 'GND'); P('U_CLK', HC74['CP2'], 'GND')                 # 2단 미사용: 입력 고정
P('U_CLK', HC74['Q2'], 'NC'); P('U_CLK', HC74['nQ2'], 'NC')

# 시스템 클럭 = 74HC74 1Q → 244 로 나눠 뿌림
#   1A0→1Y0 : PE0 (x래치·psum 2개 = 3개)
#   1A1→1Y1 : PE1 (3개)
#   나머지 입력은 GND, 출력 NC
SYSCLK = newnet(); P('U_CLK', HC74['Q1'], SYSCLK, '클럭')
P('U_BUF', HC244['A1'][0], SYSCLK); P('U_BUF', HC244['A1'][1], SYSCLK)
CLK_PE = [newnet(), newnet()]
P('U_BUF', HC244['Y1'][0], CLK_PE[0], '클럭'); P('U_BUF', HC244['Y1'][1], CLK_PE[1], '클럭')
for a in HC244['A1'][2:] + HC244['A2']:
    P('U_BUF', a, 'GND')
for y in HC244['Y1'][2:] + HC244['Y2']:
    P('U_BUF', y, 'NC')
P('U_BUF', HC244['OE1'], 'GND'); P('U_BUF', HC244['OE2'], 'GND')

# ───────────────────────── PE 내부 ─────────────────────────
psum_prev = None          # 앞 PE 의 psum_out 16 네트
xin_nets  = None          # 앞 PE 의 x_out 8 네트
WBUS = [newnet() for _ in range(8)]      # MEGA w/x 데이터 버스 (양 PE w래치 D 공통)
for b in range(8):
    P('MEGA', MEGA['D'][b], WBUS[b], 'MEGA버스')

for i in range(PES):
    t = f'PE{i}'
    W, X, LO, HI = f'{t}_U2', f'{t}_U3', f'{t}_U1L', f'{t}_U1H'
    A = [f'{t}_U4', f'{t}_U5', f'{t}_U6', f'{t}_U7']
    PL, PH = f'{t}_U8', f'{t}_U9'

    # w 래치: D ← MEGA 버스, CP ← MEGA WCPn, Q → LO/HI A8~A15
    for b in range(8):
        P(W, HC574['D'][b], WBUS[b])
    n = newnet(); P('MEGA', MEGA['WCP0' if i == 0 else 'WCP1'], n, 'MEGA제어'); P(W, HC574['CP'], n)
    P(W, HC574['OE'], 'GND')
    for b in range(8):
        n = newnet(); P(W, HC574['Q'][b], n, 'w→ROM')
        P(LO, FLASH['A'][8+b], n); P(HI, FLASH['A'][8+b], n)

    # x 래치: D ← (PE0: MEGA 버스 / PE1: 앞 PE x_out), CP ← 클럭, Q → LO/HI A0~A7 + 다음 PE
    for b in range(8):
        P(X, HC574['D'][b], WBUS[b] if i == 0 else xin_nets[b])
    P(X, HC574['CP'], CLK_PE[i]); P(X, HC574['OE'], 'GND')
    xout = []
    for b in range(8):
        n = newnet(); P(X, HC574['Q'][b], n, 'x→ROM')
        P(LO, FLASH['A'][b], n); P(HI, FLASH['A'][b], n)
        xout.append(n)
    xin_nets = xout
    if i == 0:
        # PE0 x 래치 D 는 w 래치와 같은 MEGA 버스를 공유한다 (시분할: WCP/XCP 로 구분)
        pass

    # 플래시 제어: CE#/OE# GND, WE# VCC, A16~A18 GND (안 쓰는 주소)
    for R in (LO, HI):
        P(R, FLASH['CE'], 'GND'); P(R, FLASH['OE'], 'GND'); P(R, FLASH['WE'], 'VCC')
        for a in FLASH['A'][16:]:
            P(R, a, 'GND')

    # 플래시 DQ → 가산기 A 입력.  LO→U4,U5 / HI→U6,U7
    #   + 풀업 저항: DQ 선마다 RES4k7 한쪽은 DQ 네트, 반대쪽은 VCC
    pbase = PE_BASE[i]                       # ★ 이 PE 의 기준 열 (예전엔 마지막 PE 의 base 가 남아 두 PE 저항이 겹쳤다 — R23 이 잡음)
    RCOL = pbase + 24                        # HI 플래시(pbase+16..22) 와 그 캡(pbase+23) 오른쪽
    for b in range(8):
        n = newnet(); P(LO, FLASH['DQ'][b], n, 'ROM→가산기'); P(A[b//4],   HC283['A'][b%4], n)
        rr = f'{t}_RL{b}'; parts.append((rr, 'RES4k7', RCOL + (b%4), 2 + (b//4)*5, 0, PE_LAYER[i])); P(rr, 1, n); P(rr, 2, 'VCC')
        n = newnet(); P(HI, FLASH['DQ'][b], n, 'ROM→가산기'); P(A[b//4+2], HC283['A'][b%4], n)
        rr = f'{t}_RH{b}'; parts.append((rr, 'RES4k7', RCOL + (b%4), 12 + (b//4)*5, 0, PE_LAYER[i])); P(rr, 1, n); P(rr, 2, 'VCC')

    # 캐리 세이브: U4→U5 (하위), U6→U7 (상위). 두 줄 다 CIN 은 GND 에서 시작
    P(A[0], HC283['CIN'], 'GND')
    n = newnet(); P(A[0], HC283['COUT'], n, '캐리'); P(A[1], HC283['CIN'], n)
    P(A[1], HC283['COUT'], 'NC')                      # 하위 8비트 넘침은 버림 (검증 항목)
    P(A[2], HC283['CIN'], 'GND')
    n = newnet(); P(A[2], HC283['COUT'], n, '캐리'); P(A[3], HC283['CIN'], n)
    P(A[3], HC283['COUT'], 'NC')

    # 가산기 S → psum 래치 D
    for b in range(16):
        n = newnet(); P(A[b//4], HC283['S'][b%4], n, '가산기→psum')
        P(PL if b < 8 else PH, HC574['D'][b%8], n)
    for Lr in (PL, PH):
        P(Lr, HC574['CP'], CLK_PE[i]); P(Lr, HC574['OE'], 'GND')

    # psum 래치 Q → (다음 PE 의 가산기 B) 또는 (마지막이면 MEGA)
    pout = []
    for b in range(16):
        n = newnet(); P(PL if b < 8 else PH, HC574['Q'][b%8], n, 'psum_out')
        pout.append(n)
    # 이 PE 의 가산기 B 입력 ← 앞 PE 의 psum_out (첫 PE 는 GND = 0 에서 시작)
    for b in range(16):
        P(A[b//4], HC283['B'][b%4], psum_prev[b] if psum_prev else 'GND')
    psum_prev = pout

# 마지막 PE 의 psum_out 하위 8비트 → MEGA 로 (상위 8비트는 PSEL 로 다음에; 지금은 NC)
for b in range(8):
    P('MEGA', MEGA['P'][b], psum_prev[b])
for b in range(8, 16):
    pass   # psum_prev[8..15] 는 PH.Q → 아직 아무 데도 안 감 → sol.py 가 "드라이버만 있음" 으로 알려줄 것
P('MEGA', MEGA['XCP'], 'NC'); P('MEGA', MEGA['RST'], 'NC'); P('MEGA', MEGA['PSEL'], 'NC'); P('MEGA', MEGA['X0'], 'NC')
for k in range(26, 31):
    P('MEGA', k, 'NC')

# ───────────────────────── 전원 버스 행 (R16 대책) ─────────────────────────
# 바닥 레일 하나에 매달면 접지선이 100mm 를 넘어 바운스가 여유를 넘는다 (sol.py R16).
# → 칩들의 GND/VCC 핀 행을 모아 3행(7.6mm) 안에 버스 선을 하나씩 깐다.
PKG_ROWS = {'74HC574':10,'74HC283':8,'74HC74':7,'74HC244':10,'74HC04':7,'SST39SF040':16,'OSC_FULL':7}
GND_PIN  = {'74HC574':10,'74HC283':8,'74HC74':7,'74HC244':10,'74HC04':7,'SST39SF040':16,'OSC_FULL':7}
def cluster(rows, gap=3):
    rows=sorted(set(rows)); out=[]
    for r in rows:
        if out and r-out[-1][-1] <= gap: out[-1].append(r)
        else: out.append([r])
    return [int(round(sum(c)/len(c))) for c in out]
gnd_rows = cluster([row + GND_PIN[chip]-1 for ref,chip,col,row,rot,layer in parts if chip in GND_PIN])
vcc_rows = cluster([row for ref,chip,col,row,rot,layer in parts if chip in GND_PIN])   # VCC 핀은 1번 핀 행(우상단)
buses = {'GND': [r+1 for r in gnd_rows], 'VCC': [max(0,r-1) for r in vcc_rows]}   # GND 는 핀 바로 아래 행, VCC 는 바로 위 행

# ───────────────────────── AI 의 전압 예상 (expect) ─────────────────────────
# ★ 여기 숫자는 내(AI)가 '이럴 것이다'라고 적는 값이다. sol.py 는 이걸 보지 않고
#   chips.txt 로 따로 계산한 뒤 R15 에서 대조한다. 틀리면 내가 틀린 것이다.
expects = {'VCC': (5.0, 5.0, None), 'GND': (0.0, 0.0, None)}
drivers = {}          # net -> 드라이버 부품 종류 (내가 배선하며 아는 것)
for (ref, pin), net in pins.items():
    chip = next(c for r, c, *_ in parts + caps if r == ref)
    if chip in ('74HC574','74HC283','74HC74','74HC244','74HC04') and ((chip=='74HC574' and pin in HC574['Q']) or (chip=='74HC283' and pin in HC283['S']+[HC283['COUT']]) or (chip=='74HC74' and pin in (5,6,8,9)) or (chip=='74HC244' and pin in HC244['Y1']+HC244['Y2']) or (chip=='74HC04' and pin in HC04['Y'])):
        drivers[net] = 'HC'
    elif chip == 'SST39SF040' and pin in FLASH['DQ']:
        drivers[net] = 'FLASH'
    elif chip == 'MEGA' and pin in MEGA['D'] + [MEGA['WCP0'], MEGA['WCP1'], MEGA['MCLK']]:
        drivers[net] = 'MEGA'
    elif chip == 'OSC_FULL' and pin == OSC['OUT']:
        drivers[net] = 'OSC'
for net, kind in drivers.items():
    if net in ('VCC','GND','NC'): continue
    if kind == 'HC':    expects[net] = (4.4, 0.1, None)          # 74HC 출력, 가벼운 CMOS 부하
    if kind == 'FLASH': expects[net] = (4.9, 0.4, 1.06e-3)      # 풀업 4.7k 로 HIGH 는 VCC 근처, LOW 때 1.06mA 흡수
    if kind == 'MEGA':  expects[net] = (4.2, 0.7, None)          # ATmega2560 출력
    if kind == 'OSC':   expects[net] = (4.0, 0.5, None)

# ───────────────────────── 층간 연결 (아파트) ─────────────────────────
stacks = []
# 층이 여러 개면, 층을 넘는 네트를 모아 '층 경계'마다 핀헤더를 세운다.
# ★ 중간층은 위·아래 각각 자기 칸을 쓴다 (2층이면 1-2 용 한 줄, 2-3 용 또 한 줄).
if LAYERS >= 2:
    layer_of = {}
    for ref, chip, col, row, rot, layer in parts + caps: layer_of[ref] = layer
    cross = {}    # (lo,hi) -> [net...]
    for net in sorted({n for (r,pn),n in pins.items() if n not in ('VCC','GND','NC')}, key=str):
        ls = sorted({layer_of.get(r,1) for (r,pn),n in pins.items() if n == net})
        if len(ls) < 2: continue
        for a, b in zip(ls, ls[1:]): cross.setdefault((a,b), []).append(net)
    # 전원도 모든 층에 올려야 한다
    for a in range(1, LAYERS):
        cross.setdefault((a,a+1), [])
        cross[(a,a+1)] = ['VCC','GND'] + cross[(a,a+1)]
    si = 0
    # 이미 쓰인 칸 (부품 풋프린트) — 층별로
    occupied = {}
    WID = {'74HC574':3,'74HC283':3,'74HCT283':3,'74HC74':3,'74HC244':3,'74HC04':3,'SST39SF040':6,'OSC_FULL':3,'CAP100n':0,'CAP10u':0,'RES4k7':0,'MEGA':1}
    ROWS= {'74HC574':10,'74HC283':8,'74HCT283':8,'74HC74':7,'74HC244':10,'74HC04':7,'SST39SF040':16,'OSC_FULL':7,'CAP100n':1,'CAP10u':1,'RES4k7':4,'MEGA':15}
    for ref, chip, col, row, rot, lay in parts + caps:
        w = WID.get(chip, 1); h = ROWS.get(chip, 2)
        for x in range(col-1, col+w+2):
            for y in range(row-1, row+h+1):
                occupied.setdefault(lay, set()).add((x, y))
    def free_spot(lo, need):
        """lo 층과 lo+1 층 모두 비어 있는 세로 need 칸 자리를 찾는다"""
        for col in range(1, BW-1):
            for row in range(1, BH-need-1):
                cells = {(col, row+k) for k in range(need)}
                if any(cells & occupied.get(Lz, set()) for Lz in (lo, lo+1)): continue
                return col, row
        return None, None
    # 여유 통로: 배선이 '잠깐 다른 층으로 돌아가기' 위한 빈 핀. 네트를 안 물린 채 둔다.
    SPARE = int(os.environ.get('SPARE_VIAS', 20))
    for (lo, hi), nets in sorted(cross.items()):
        nets = list(nets) + [None] * SPARE
        while nets:
            take = nets[:20]; nets = nets[20:]
            col, row = free_spot(lo, 20 if len(take) > 8 else 8)
            if col is None:
                print(f"  ! {lo}-{hi}층 핀헤더 자리를 못 찾음 (기판을 키우거나 층을 늘릴 것)")
                break
            si += 1; sref = f'J{si}'
            kind = 'HDR20' if len(take) > 8 else 'HDR8'
            span = 20 if kind == 'HDR20' else 8          # 헤더가 실제로 차지하는 세로 칸
            stacks.append((sref, kind, col, row, lo, hi))
            for k, net in enumerate(take, 1):
                if net is not None: P(sref, k, net, '층간')   # None = 여유 통로 (빈 핀)
            for Lz in (lo, hi):
                for k in range(span):
                    occupied.setdefault(Lz, set()).update({(col-1, row+k), (col, row+k), (col+1, row+k)})

# ───────────────────────── 파일 쓰기 ─────────────────────────
L.append('#KYLE 1')
L.append(f'title   PE {PES}개 — 플래시 2개/PE, 캐리세이브, 클럭 버퍼 ({BW}x{BH} 구멍, {LAYERS}층)')
L.append(f'board   perf {BW} {BH}' + (f' layers {LAYERS}' if LAYERS>1 else ''))
L.append('include chips.txt')
L.append('wire    0.5mm')        # 배선 굵기 (물리 값이라 UI 가 아니라 KYLE 문장)
L.append('stitch  6')            # 바느질 간격 (칸)
L.append('share   1')            # 점당 선 수 (1 = 겹침 없음)
L.append('')
L.append('supply  VCC 5.0V max 5A')
L.append('supply  GND 0V')
L.append('')
L.append('# ── 부품 (열 행 은 1번 핀 구멍, 2.54mm 격자) ──')
for ref, chip, col, row, rot, layer in parts + caps:
    L.append(f'part    {ref:<9}{chip:<12}at {col:>2} {row:>2}' + (f'  rot {rot}' if rot else '') + (f'  layer {layer}' if layer != 1 else '') + (f'  near {NEAR[ref]}' if ref in NEAR else '') + ('  fixed' if ref in FIXED else ''))
if stacks:
    L.append('')
    L.append('# ── 층간 연결 핀헤더 (아파트). 중간층은 위·아래가 서로 다른 열을 쓴다 ──')
    for sref, kind, col, row, lo, hi in stacks:
        L.append(f'stack   {sref} {kind} at {col} {row} layers {lo}-{hi}')
L.append('')
L.append('# ── 그룹 색 ──')
COLORS = {'w→ROM':'#d62728','x→ROM':'#1f77b4','ROM→가산기':'#2ca02c','캐리':'#ff7f0e',
          '가산기→psum':'#9467bd','psum_out':'#8c564b','클럭':'#17becf','MEGA버스':'#e377c2','MEGA제어':'#bcbd22'}
for g, c in COLORS.items():
    L.append(f'color   {g:<12}{c}')
L.append('')
L.append('# ── 포인터 연결  REF[핀:넷]  (AI 가 쓰는 유일한 부분) ──')
cur = None
_order = [p[0] for p in parts + caps] + [st[0] for st in stacks]
for (ref, pin), net in sorted(pins.items(), key=lambda kv: (_order.index(kv[0][0]) if kv[0][0] in _order else 9999, kv[0][1])):
    if ref != cur:
        L.append(f'# {ref}')
        cur = ref
    L.append(f'pin     {ref}[{pin}:{net}]')
L.append('')
L.append('# ── 네트 그룹 ──')
for net, g in sorted(groups.items(), key=lambda kv: str(kv[0])):
    L.append(f'group   {net:<6}{g}')
L.append('')
L.append('# ── 전원 버스 선이 지나는 행 (각 전원 핀은 가장 가까운 버스에 짧게) ──')
for net, rows in buses.items():
    L.append(f'bus     {net} rows ' + ','.join(map(str, rows)))
L.append('')
L.append('# ── AI 의 전압 예상: expect <넷> <HIGH V> <LOW V> [<LOW 흡수전류>]  — sol.py R15 가 데이터시트로 대조 ──')
for net, (hi, lo, cur) in sorted(expects.items(), key=lambda kv: str(kv[0])):
    L.append(f'expect  {str(net):<6}{hi} {lo}' + (f' {cur*1e3:.2f}mA' if cur is not None else ''))

L.append('')
L.append('# ── 회로 시험: sim 이름 … end. simulate.py 가 pin + chips.txt(func/reg) 만으로 실제로 돌려 out 과 대조 (sol.py R30) ──')
L.append('# ROM 은 python3 simulate.py --mulrom . 으로 만든 곱셈표 (주소 = w<<8 | x)')
for nm, w, x in (('mul7x6', 7, 6), ('mul255', 255, 255), ('mul0', 0, 200)):
    pr = w * x
    L += [f'sim     {nm}', '  rom     PE0_U1L mul_lo.bin', '  rom     PE0_U1H mul_hi.bin', '  jumper  JP_CLK B-C',
          f'  in      MEGA.D {w}', '  pulse   MEGA.WCP0', f'  in      MEGA.D {x}', '  clock   MEGA.MCLK 4',
          f'  out     PE0_U1L.DQ {pr & 0xFF}', f'  out     PE0_U1H.DQ {(pr >> 8) & 0xFF}', 'end']
io.open(OUT, 'w', encoding='utf-8').write('\n'.join(L) + '\n')
print(f"{OUT}")
print(f"부품 {len(parts)+len(caps)}개, 핀 연결 {len(pins)}개, 네트 {len(set(pins.values()))}개")
# 생성기의 초기 좌표는 '대략' 이라 기판이 작으면 겹치거나 밖으로 나간다 (R23). AI 는 연결만 적고 자리는 배치기가 잡는다 —
# 그래서 바로 place.py 로 합법 배치까지 한다. NOPLACE=1 이면 건너뛴다 (ITERS 로 담금질 횟수).
if not os.environ.get('NOPLACE'):
    import place
    info = place.run(OUT, OUT, iters=int(os.environ.get('ITERS', '3000')), seed=int(os.environ.get('SEED', '0')))
    print(f"배치기: 비용 {info['cost0']:.0f} → {info['cost']:.0f}, 라이저 {info['risers']}개" + ("  ★ 자리 못 찾은 부품 있음" if info.get('fail_place') else ''))
