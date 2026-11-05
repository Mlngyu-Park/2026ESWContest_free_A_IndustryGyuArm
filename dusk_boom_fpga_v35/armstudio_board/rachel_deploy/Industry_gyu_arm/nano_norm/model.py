"""FIXER 모델 — RACHEL 과 같은 구조를 그대로 쓴다.

원본: nano_llm/model.py (글자 단위 GPT). 검증된 것을 다시 만들지 않는다.

글자 단위 GPT (decoder-only). 외부 의존성은 torch 뿐.
구조·토크나이저·학습 전부 이 저장소 안에서 만들었다.
"""
import math, torch, torch.nn as nn, torch.nn.functional as F

class Block(nn.Module):
    """트랜스포머 블록 하나.

    ★ 어텐션을 nn.MultiheadAttention 대신 직접 짰다.
      MultiheadAttention 은 마스크를 주면 최적화 경로를 못 타서,
      이 GPU(GB10)에서 스텝당 1,972ms 가 나왔다 (실측).
      scaled_dot_product_attention 은 is_causal 로 같은 일을 하면서
      최적화 커널을 쓴다. 수식은 완전히 같다.
    """
    def __init__(s, d, h, drop):
        super().__init__()
        s.h, s.d = h, d
        s.ln1 = nn.LayerNorm(d)
        s.qkv = nn.Linear(d, 3*d)
        s.proj = nn.Linear(d, d)
        s.drop = drop
        s.ln2 = nn.LayerNorm(d)
        s.mlp = nn.Sequential(nn.Linear(d, 4*d), nn.GELU(), nn.Linear(4*d, d), nn.Dropout(drop))

    def forward(s, x, mask=None):
        B, T, D = x.shape
        a = s.ln1(x)
        q, k, v = s.qkv(a).split(D, dim=2)
        # (B, head, T, head_dim) 으로 나눈다
        q = q.view(B, T, s.h, D // s.h).transpose(1, 2)
        k = k.view(B, T, s.h, D // s.h).transpose(1, 2)
        v = v.view(B, T, s.h, D // s.h).transpose(1, 2)
        o = F.scaled_dot_product_attention(
            q, k, v, is_causal=True,
            dropout_p=s.drop if s.training else 0.0)
        o = o.transpose(1, 2).contiguous().view(B, T, D)
        x = x + s.proj(o)
        return x + s.mlp(s.ln2(x))

class Rachel(nn.Module):
    def __init__(s, vocab, d=320, layers=8, heads=8, ctx=256, drop=0.1):
        super().__init__()
        s.ctx = ctx
        s.tok = nn.Embedding(vocab, d)
        s.pos = nn.Embedding(ctx, d)
        s.blocks = nn.ModuleList(Block(d, heads, drop) for _ in range(layers))
        s.ln = nn.LayerNorm(d)
        s.head = nn.Linear(d, vocab, bias=False)
        s.head.weight = s.tok.weight            # 가중치 공유
        mask = torch.triu(torch.full((ctx, ctx), float("-inf")), 1)
        s.register_buffer("mask", mask, persistent=False)
    def forward(s, idx):
        T = idx.size(1)
        x = s.tok(idx) + s.pos(torch.arange(T, device=idx.device))
        m = s.mask[:T, :T]
        for b in s.blocks: x = b(x, m)
        return s.head(s.ln(x))
    def n_params(s):
        return sum(p.numel() for p in s.parameters())

    @torch.no_grad()
    def generate(s, idx, eos, max_new=180):
        s.eval()
        for _ in range(max_new):
            logits = s(idx[:, -s.ctx:])
            nxt = logits[:, -1].argmax(-1, keepdim=True)   # greedy (temperature 0)
            idx = torch.cat([idx, nxt], 1)
            if nxt.item() == eos: break
        return idx
