#!/usr/bin/env python3
"""실제 말로 채점: 말 → [정리기] → 안전 검사 → RACHEL → 블록, 정답과 비교.

  python eval_real.py norm.pt            # 정답: paper_evidence/15_REAL_VOICE_1028.json (장면 B)

정리기 없이(원문 그대로) 와 나란히 보여 준다. 안전 검사에 걸리면 원문을 쓴다.
"""
import json, os, re, subprocess, sys, time
import torch
HERE = os.path.dirname(os.path.abspath(__file__)); APP = os.path.dirname(HERE)
sys.path.insert(0, APP); sys.path.insert(0, HERE)
from nano_llm import hints
import tok_norm as tk, safety
from model import Rachel

ID = {'보라': 'd2', '빨강': 'd10', '주황': 'd11', '노랑': 'd12', '파랑': 'd13', '초록': 'd14'}

def load_norm(path):
    ck = torch.load(path, map_location="cpu")
    itos = ck["vocab"]; stoi = {c: i for i, c in enumerate(itos)}
    m = Rachel(len(itos), d=ck["dim"], layers=ck["layers"], heads=ck["heads"], ctx=ck["ctx"])
    m.load_state_dict(ck["model"]); m.eval()
    dev = "cuda" if torch.cuda.is_available() else "cpu"
    return m.to(dev), stoi, itos, ck["ctx"], dev, ck

@torch.no_grad()
def normalize(m, stoi, itos, ctx, dev, text, limit=200):
    ids = tk.encode(stoi, " ".join(text.split()) + "\n>")[-(ctx - 1):]
    x = torch.tensor([ids], device=dev); out = []
    for _ in range(limit):
        t = int(m(x[:, -ctx:])[0, -1].argmax())
        if t == tk.EOS: break
        out.append(t); x = torch.cat([x, torch.tensor([[t]], device=dev)], 1)
    return tk.decode(itos, out)

def rb(r):
    if r in ('[]', ''): return []
    st = [ID[x] for x in r.split('→')]; b = [f'@goto:{st[0]}']
    for d in st[1:]: b += ['@grasp:ppap', f'@goto:{d}', f'@release:{d}']
    return b

def main():
    m, stoi, itos, ctx, dev, ck = load_norm(sys.argv[1])
    print(f"정리기: {ck.get('step')}걸음 · 검증 글자까지 같음 {ck.get('exact', 0)*100:.1f}%")
    items = json.load(open(f'{APP}/paper_evidence/15_REAL_VOICE_1028.json', encoding='utf-8'))
    p = subprocess.Popen([f'{APP}/rachel_cpp/rachel', f'{APP}/rachel_cpp/rachel.bin', '--serve'],
                         stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    assert p.stdout.readline().strip() == b'READY'
    def rachel(cols, home, t):
        cmap = {ID[c]: c for c in cols}
        hdr = ' '.join(f'{c}={d}' for d, c in cmap.items()) + f' | 펜=초록@{home}'
        h = hints.build(cmap, home, '초록', t)
        pr = (f'{hdr}\n힌트 {h}' if h else hdr) + f'\n{t}\n>'
        p.stdin.write(pr.replace('\n', '\x01').encode() + b'\n'); p.stdin.flush()
        raw = p.stdout.readline().decode('utf-8', 'replace')
        return re.findall(r'@(?:goto|grasp|release):[A-Za-z0-9_#\-]+|@open|@close|@down|@up', raw)
    S = {'원문': [0, 0], '정리기': [0, 0]}; NC = {'원문': [0, 0], '정리기': [0, 0]}
    fell = 0; t_norm = []; rows = []
    for it in items:
        g = it['gB']
        cols = list(dict.fromkeys(['보라', '빨강', '주황'] + (g.split('→') if g != '[]' else [])))
        home = ID[g.split('→')[0]] if g != '[]' else 'd2'
        t0 = time.time(); nt = normalize(m, stoi, itos, ctx, dev, it['text']); t_norm.append(time.time() - t0)
        ok, why = safety.check(it['text'], nt)
        if not ok: fell += 1
        use = nt if ok else it['text']
        for name, txt in (('원문', it['text']), ('정리기', use)):
            if name == '정리기' and ok and nt == '(명령 아님)': bl = []          # 명령 아님 → RACHEL 안 부름
            else: bl = rachel(cols, home, txt)
            good = bl == rb(g)
            box = S if (it['cmd'] == 'O' and g != '[]') else NC
            box[name][0] += good; box[name][1] += 1
            if name == '정리기': rows.append((good, it['text'], nt, why, g))
    p.stdin.write(b'QUIT\n'); p.stdin.flush(); p.wait()
    for name in ('원문', '정리기'):
        a, b = S[name]; c, d = NC[name]
        print(f"{name:6s} 이송 명령 {a}/{b} ({a/b*100:.0f}%) · 명령 아님 {c}/{d}")
    print(f"안전 검사에 걸려 원문 사용 {fell}/{len(items)} · 정리기 평균 {sum(t_norm)/len(t_norm)*1000:.0f}ms")
    print("\n틀린 것 (정리기 출력):")
    for good, src, nt, why, g in rows:
        if not good: print(f"  정답 {g:16s} | {nt[:60]:60s} | {why[:30]} | {src[:50]}")
    json.dump([dict(ok=r[0], text=r[1], norm=r[2], why=r[3], gold=r[4]) for r in rows],
              open(os.path.join(HERE, 'eval_real_out.json'), 'w'), ensure_ascii=False, indent=1)

if __name__ == '__main__':
    main()
