#!/usr/bin/env python3
"""M68K 32노드 어레이 에뮬레이터.

목적 두 가지:
  ① 정확도 — 32개로 나눠 계산한 결과가 원본과 같은가?
     (68000 은 16비트 정수 연산이므로 float 와 다를 수 있다)
  ② 시간   — mac_68k.s 의 사이클로 실제 몇 분인가?

실제 68000 명령을 흉내내되, 검증에 필요한 정수 연산 정밀도를 지킨다.

  python emulate.py           # 전체 검증
  python emulate.py --quick   # 빠른 확인
"""
import json, struct, sys, time
import numpy as np

import math
CLK      = 8_000_000    # --clk 로 변경
T_MAC    = 100          # mac_68k.s 언롤4 실측 카운트
T_RING   = 40           # 링 1바이트 전달 (MOVE.B + 핸드셰이크)
NODES    = 32           # --nodes 로 변경
TOPO     = 'tree'       # ring / group / tree
BIN      = "../rachel_cpp/rachel.bin"

# ── 가중치 파일 읽기 (C++ 과 같은 형식) ──────────────────────
def load_bin(path):
    f = open(path, "rb")
    assert f.read(4) == b"RCHL"
    ver, d, L, h, c, v = struct.unpack("<6I", f.read(24))
    vlen, = struct.unpack("<I", f.read(4))
    vb = f.read(vlen)
    itos = []
    p = 0
    while p < len(vb):
        n = vb[p]; p += 1
        itos.append(vb[p:p+n].decode("utf-8", "replace")); p += n
    nt, = struct.unpack("<I", f.read(4))
    T = {}
    for _ in range(nt):
        nl, = struct.unpack("<H", f.read(2))
        nm = f.read(nl).decode()
        cnt, sc = struct.unpack("<If", f.read(8))
        q = np.frombuffer(f.read(cnt), dtype=np.int8)
        T[nm] = (q, sc)
    f.close()
    return dict(dim=d, layers=L, heads=h, ctx=c, vocab=v, itos=itos, T=T)

# ── 68000 이 실제로 하는 정수 MAC ────────────────────────────
def mac_int(w_row, x_q):
    """MULS(16×16→32) + ADD.L 누산. 파이썬 int 로 68000 과 동일 결과."""
    return int(np.dot(w_row.astype(np.int32), x_q.astype(np.int32)))

def quant_act(x, bits=15):
    """활성값을 int16 으로. 68000 은 float 가 없다."""
    m = float(np.abs(x).max()) or 1.0
    s = m / ((1 << bits) - 1)
    return np.round(x / s).astype(np.int32), s

# ── 32노드 분산 행렬-벡터 곱 ────────────────────────────────
def matvec_array(W, sc, x, rows, cols, stats):
    """행을 32등분. 워커 k 는 i≡k(mod 32) 행만 계산한다."""
    xq, xs = quant_act(x)
    # 벡터 연산으로 한 번에 (결과는 68000 정수 MAC 과 동일)
    Wm = W[:rows*cols].reshape(rows, cols).astype(np.int32)
    acc = Wm @ xq.astype(np.int32)
    y = acc.astype(np.float64) * sc * xs
    # 이 행렬을 32노드로 나눴을 때 '가장 많이 맡은 노드' 의 MAC 수
    #   워커 k 는 i≡k(mod 32) 행 담당 -> 행 수를 32로 나눈 올림
    rows_max = (rows + NODES - 1) // NODES
    stats['mac']  += rows * cols
    stats['peak'] += rows_max * cols        # ★ 누적해야 한다 (매 층·매 글자)
    stats['bytes'] += cols * 2 + rows * 4   # 입력 int16 뿌리기 + 결과 int32 수집
    return y

def layernorm(x, g, gs, b, bs):
    mu = x.mean(); va = x.var() + 1e-5
    inv = 1.0 / np.sqrt(va)
    gg = g.astype(np.float64)*gs if g is not None else 1.0
    bb = b.astype(np.float64)*bs if b is not None else 0.0
    return (x - mu) * inv * gg + bb

def gelu(x):
    return 0.5*x*(1.0 + np.tanh(0.7978845608*(x + 0.044715*x**3)))

def run(model, prompt, stats, max_new=200):
    d, L, H = model['dim'], model['layers'], model['heads']
    hd = d // H
    T = model['T']; itos = model['itos']
    stoi = {c: i for i, c in enumerate(itos)}
    def W(n):
        return T.get(n, (np.zeros(0, np.int8), 1.0))

    kcache = [np.zeros((model['ctx'], d)) for _ in range(L)]
    vcache = [np.zeros((model['ctx'], d)) for _ in range(L)]
    out = ""; pos = 0
    toks = [stoi.get(ch, 1) for ch in prompt]

    def step(tok, pos):
        emb, es = W('tok.weight'); pe, ps = W('pos.weight')
        x = emb[tok*d:(tok+1)*d].astype(np.float64)*es \
          + pe[pos*d:(pos+1)*d].astype(np.float64)*ps
        for l in range(L):
            p = f'blocks.{l}.'
            g1, g1s = W(p+'ln1.weight'); b1, b1s = W(p+'ln1.bias')
            h = layernorm(x, g1, g1s, b1, b1s)
            Wq, qs = W(p+'attn.in_proj_weight'); Bq, bqs = W(p+'attn.in_proj_bias')
            qkv = matvec_array(Wq, qs, h, 3*d, d, stats)
            if Bq.size: qkv += Bq.astype(np.float64)*bqs
            q_, k_, v_ = qkv[:d], qkv[d:2*d], qkv[2*d:]
            kcache[l][pos] = k_; vcache[l][pos] = v_
            ho = np.zeros(d)
            for hh in range(H):
                sl = slice(hh*hd, (hh+1)*hd)
                sc_ = kcache[l][:pos+1, sl] @ q_[sl] / np.sqrt(hd)
                a = np.exp(sc_ - sc_.max()); a /= a.sum()
                ho[sl] = a @ vcache[l][:pos+1, sl]
                stats['mac'] += (pos+1)*hd*2
            Wo, os_ = W(p+'attn.out_proj.weight'); Bo, bos = W(p+'attn.out_proj.bias')
            o = matvec_array(Wo, os_, ho, d, d, stats)
            if Bo.size: o += Bo.astype(np.float64)*bos
            x = x + o
            g2, g2s = W(p+'ln2.weight'); b2, b2s = W(p+'ln2.bias')
            h = layernorm(x, g2, g2s, b2, b2s)
            W1, w1s = W(p+'mlp.0.weight'); B1, b1s2 = W(p+'mlp.0.bias')
            m = matvec_array(W1, w1s, h, 4*d, d, stats)
            if B1.size: m += B1.astype(np.float64)*b1s2
            m = gelu(m)
            W2, w2s = W(p+'mlp.2.weight'); B2, b2s2 = W(p+'mlp.2.bias')
            o = matvec_array(W2, w2s, m, d, 4*d, stats)
            if B2.size: o += B2.astype(np.float64)*b2s2
            x = x + o
        gl, gls = W('ln.weight'); bl, bls = W('ln.bias')
        x = layernorm(x, gl, gls, bl, bls)
        emb, es = W('tok.weight')
        return matvec_array(emb, es, x, model['vocab'], d, stats)

    for t in toks:
        logits = step(t, pos); pos += 1
    for _ in range(max_new):
        i = int(np.argmax(logits))
        if i == 2: break
        if i >= 3: out += itos[i]
        logits = step(i, pos); pos += 1
        if pos >= model['ctx']: break
    return out

def main():
    global CLK, NODES, TOPO
    a = sys.argv
    if '--clk'   in a: CLK   = float(a[a.index('--clk')+1])*1e6
    if '--nodes' in a: NODES = int(a[a.index('--nodes')+1])
    if '--topo'  in a: TOPO  = a[a.index('--topo')+1]
    if '--tmac'  in a:
        global T_MAC; T_MAC = int(a[a.index('--tmac')+1])
    quick = '--quick' in sys.argv
    print("M68K 32노드 어레이 에뮬레이터")
    print(f"  클럭 {CLK/1e6:.1f}MHz · MAC {T_MAC}사이클 · 노드 {NODES}개 · {TOPO}\n")
    m = load_bin(BIN)
    print(f"모델 d{m['dim']} L{m['layers']} vocab{m['vocab']}")

    cases = [("보라=d2 빨강=d10 주황=d11 | 펜=초록@d2\n힌트 d2 d10\n"
              "보라색 펜을 빨간 도킹으로 옮겨\n>", 4)]
    if not quick:
        cases.append(("보라=d2 빨강=d10 주황=d11 | 펜=초록@d2\n힌트 d2 d10 d11\n"
                      "보라색 펜을 빨간색으로 옮긴 다음 다시 잡아서 주황색으로 옮겨\n>", 7))

    for prompt, nb in cases:
        stats = dict(mac=0, peak=0, bytes=0)
        t0 = time.time()
        out = run(m, prompt, stats)
        wall = time.time() - t0
        # 실제 68000 시간: 가장 오래 걸리는 노드가 전체를 결정한다
        t_calc = stats['peak'] * T_MAC / CLK
        if   TOPO == 'ring':  hops = NODES
        elif TOPO == 'group': hops = 16 + math.ceil(NODES/16)
        else:                 hops = max(1, math.ceil(math.log2(NODES))) * 2
        t_ring = stats['bytes'] / NODES * hops * T_RING / CLK
        total = t_calc + t_ring
        print(f"\n입력: {prompt.splitlines()[-2][:40]}")
        print(f"출력: {out[:80]}")
        print(f"  블록 수      {out.count('@')}개 (기대 {nb})")
        print(f"  총 MAC       {stats['mac']:,}")
        print(f"  최대 노드    {stats['peak']:,} MAC")
        print(f"  링 전송      {stats['bytes']:,} 바이트 · 홉 {hops}")
        print(f"  ── 68000 32개 예상 ──")
        print(f"  계산         {t_calc/60:.2f}분")
        print(f"  통신         {t_ring:.1f}초 ({t_ring/total*100:.1f}%)")
        print(f"  합계         {total/60:.2f}분")
        print(f"  (에뮬 실행   {wall:.1f}초)")

if __name__ == '__main__':
    main()
