#!/usr/bin/env python3
"""실사용 문장 회귀시험 — 새 판이 옛 문장을 다시 틀리지 않는지."""
import json, torch, sys
from model import Rachel
import tokenizer as tk, hints

def _mk(ck, itos):
    """체크포인트에 저장된 구조로 모델을 만든다.
    문맥·차원을 하드코딩하면 판이 바뀔 때마다 깨진다 (실제로 깨졌다)."""
    return Rachel(len(itos),
                  d=ck.get("dim", 320), layers=ck.get("layers", 8),
                  heads=ck.get("heads", 8), ctx=ck.get("ctx", 256))


CASES=[  # (cmap, home, pcol, 문장, 기대 블록수, 기대 마지막 release)
 ({'d2':'보라','d10':'빨강','d11':'주황'},'d2','초록',
  "야 보라색 도킹 스테이션에 있는 거 잡아서 빨간 아 초록색 정정 보라색 도킹 스테이션에 있는 초록색 펜 잡 아서 빨간색으로 옮기는 거 좀 해줘 봐",4,'d10'),
 ({'d2':'보라','d10':'빨강','d11':'주황'},'d2','초록',
  "보라색에 있는 초록색 펜 잡은 다음에 그거 빨간색으로 좀 이동시켜줘",4,'d10'),
 ({'d2':'보라','d10':'빨강','d11':'주황'},'d2','초록',
  "야 보라색에 있는 초록색 펜 잡아서 빨간색으로 옮긴 다음에 다시 그 펜 잡아서 주황색으로 옮기는 거 좀 해보자",7,'d11'),
]
ck=torch.load('rachel.pt',map_location='cpu')
itos=ck['vocab']; stoi={c:i for i,c in enumerate(itos)}
m=_mk(ck, itos); m.load_state_dict(ck['model']); m.eval()
ok=0
for cmap,home,pcol,t,nb,last in CASES:
    hdr=' '.join(f'{c}={d}' for d,c in cmap.items())+f' | 펜={pcol}@{home}'
    h=hints.build(cmap,home,pcol,t)
    p=(f'{hdr}\n힌트 {h}' if h else hdr)+f'\n{t}\n>'
    ids=torch.tensor([tk.encode(stoi,p)])
    got=tk.decode(itos,m.generate(ids,tk.EOS)[0,ids.size(1):].tolist()).strip()
    try:
        bl=json.loads(got[got.find('{'):]).get('blocks',[])
        good=(len(bl)==nb and bl[-1]==f'@release:{last}')
    except Exception: bl=got[:60]; good=False
    ok+=good
    print(('✅' if good else '❌'), f'{len(bl) if isinstance(bl,list) else "?"}블록:', t[:38])
    if not good: print('   ->',bl)
print(f'{ok}/{len(CASES)} 통과')
sys.exit(0 if ok==len(CASES) else 1)
