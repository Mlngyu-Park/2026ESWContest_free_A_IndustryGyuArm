"""추론기 바꾼 뒤 확인: 옛 실행파일과 같은 답인지 · 5단계가 끝까지 나오는지 · 긴 입력에 안 죽는지 · 속도."""
import json, subprocess, sys, re, time, os
APP=os.path.dirname(os.path.dirname(os.path.abspath(__file__))); sys.path.insert(0,APP)
from nano_llm import hints
def srv(exe):
    p=subprocess.Popen([exe,f'{APP}/rachel_cpp/rachel.bin','--serve'],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.DEVNULL)
    assert p.stdout.readline().strip()==b'READY'; return p
def ask(p,pr):
    p.stdin.write(pr.replace('\n','\x01').encode()+b'\n'); p.stdin.flush(); return p.stdout.readline().decode('utf-8','replace').strip()
old=srv(sys.argv[1]); new=srv(sys.argv[2])
rows=[]
for l in open(f'{APP}/llm_usage.jsonl',encoding='utf-8'):
    try: rows.append(' '.join(json.loads(l)['text'].split()))
    except Exception: pass
U=list(dict.fromkeys(rows))
cmap={'d2':'보라','d10':'빨강','d11':'주황','d12':'노랑'}; hdr='보라=d2 빨강=d10 주황=d11 노랑=d12 | 펜=초록@d2'
same=0; to=tn=0
for t in U:
    h=hints.build(cmap,'d2','초록',t); pr=(f'{hdr}\n힌트 {h}' if h else hdr)+f'\n{t}\n>'
    t0=time.time(); a=ask(old,pr); to+=time.time()-t0
    t0=time.time(); b=ask(new,pr); tn+=time.time()-t0
    same+= a==b
print(f'실제 말 {len(U)}문장: 같은 답 {same} · 평균 옛 {to/len(U)*1000:.0f}ms / 새 {tn/len(U)*1000:.0f}ms')
c='보라색 도킹의 초록 펜을 빨간색으로 옮긴 다음 주황색으로 옮긴 다음 노란색으로 옮긴 다음 파란색으로 옮긴 다음 초록색으로 옮겨'
cm={'d2':'보라','d3':'빨강','d5':'주황','d7':'노랑','d11':'파랑','d9':'초록'}
hd=' '.join(f'{v}={k}' for k,v in cm.items())+' | 펜=초록@d2'
pr=f"{hd}\n힌트 {hints.build(cm,'d2','초록',c)}\n{c}\n>"
for nm,p in (('옛',old),('새',new)):
    r=ask(p,pr); print(f'5단계 {nm}: {len(r)}자 · 놓기 {r.count("@release")}번 · 끝 {r[-30:]}')
long='야 '+'음 그러니까 '*60+c
pr=f"{hd}\n힌트 {hints.build(cm,'d2','초록',long)}\n{long}\n>"
r=ask(new,pr); print(f'긴 입력 {len(pr)}자: 새 실행파일 죽지 않음 ({len(r)}자 출력)')
for p in (old,new): p.stdin.write(b'QUIT\n'); p.stdin.flush()
