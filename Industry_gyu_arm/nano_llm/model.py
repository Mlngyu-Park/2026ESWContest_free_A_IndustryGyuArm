"""RACHEL — Robot Arm Command Handling Embedded Language-model.

글자 단위 GPT (decoder-only). 외부 의존성은 torch 뿐.
구조·토크나이저·학습 전부 이 저장소 안에서 만들었다.
"""
import math, torch, torch.nn as nn, torch.nn.functional as F

class Block(nn.Module):
    def __init__(s, d, h, drop):
        super().__init__()
        s.ln1 = nn.LayerNorm(d); s.attn = nn.MultiheadAttention(d, h, drop, batch_first=True)
        s.ln2 = nn.LayerNorm(d)
        s.mlp = nn.Sequential(nn.Linear(d, 4*d), nn.GELU(), nn.Linear(4*d, d), nn.Dropout(drop))
    def forward(s, x, mask):
        a = s.ln1(x)
        x = x + s.attn(a, a, a, attn_mask=mask, need_weights=False)[0]
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
