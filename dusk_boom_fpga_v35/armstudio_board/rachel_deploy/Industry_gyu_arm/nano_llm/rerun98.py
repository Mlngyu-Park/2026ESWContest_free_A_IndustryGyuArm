"""실사용 98문장을 v9 로 다시 돌린다 — 앱과 같은 장면 헤더(_rachel_scene)·같은 힌트·같은 C 바이너리."""
import json, os, subprocess, sys, io, contextlib
with contextlib.redirect_stdout(io.StringIO()):
    import arm_studio_web as W
    r=W.h_factory_load('이전 배치')
sc=W._rachel_scene()
cmap,home,pcol,hdr=sc
print("장면:",hdr, "| 도킹", len(W.docking_ids()))
from nano_llm import hints
rows=[json.loads(l) for l in open('llm_usage.jsonl',encoding='utf-8') if l.strip()]
texts=[]
for x in rows:
    t=x['text'].strip().replace('\n',' ')
    if t not in texts: texts.append(t)
p=subprocess.Popen(['rachel_cpp/rachel','rachel_cpp/rachel_v9.bin','--serve'],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.DEVNULL)
assert p.stdout.readline().strip()==b'READY'
out=open('paper_evidence/07_RERUN_V9.jsonl','w',encoding='utf-8')
for i,t in enumerate(texts,1):
    h=hints.build(cmap,home,pcol,t)
    prompt=(f'{hdr}\n힌트 {h}' if h else hdr)+f'\n{t}\n>'
    p.stdin.write(prompt.replace('\n','\x01').encode()+b'\n'); p.stdin.flush()
    got=p.stdout.readline().decode('utf-8','replace').strip()
    out.write(json.dumps({'no':i,'text':t,'hint':h,'v9_raw':got},ensure_ascii=False)+'\n')
p.stdin.write(b'QUIT\n'); p.stdin.flush(); p.wait(); out.close()
print(f"{len(texts)}문장 v9 재실행 완료 -> paper_evidence/07_RERUN_V9.jsonl")
os._exit(0)
