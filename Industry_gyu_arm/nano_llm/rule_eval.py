"""앱의 규칙 파서(h_say ②~⑥)를 그대로 재현해 val.jsonl 앞 200문장에 돌린다.
앱 코드는 상태(S, BRICKS)에 묶여 있어 장면 헤더에서 같은 어휘를 만든다."""
import json, re, collections
_COLOR_WORDS={'빨강':['빨강','빨간','레드'],'주황':['주황','오렌지'],'노랑':['노랑','노란','옐로'],
 '초록':['초록','그린'],'파랑':['파랑','파란','블루'],'보라':['보라','퍼플']}
def parse_scene(sc):
    head=sc.split('\n')[0]; left,right=head.split('|')
    cmap={}
    for tok in left.split():
        c,d=tok.split('='); cmap[d]=c
    m=re.match(r'\s*펜=(\S+)@(\S+)',right); pcol,home=m.group(1),m.group(2)
    return cmap,pcol,home
def vocab(cmap,pcol):
    out=[]
    add=lambda k,oid,kind: out.append((k.replace(' ',''),oid,kind))
    add('ppap','ppap','brick'); add('펜','ppap','brick'); add('부품','ppap','brick'); add('물건','ppap','brick')
    for w in _COLOR_WORDS[pcol]: add(w,'ppap','brick'); add(w+'펜','ppap','brick')
    for did,c in cmap.items():
        n=did[1:]
        for w in (f'{n}번도킹',f'도킹{n}번',f'도킹{n}',f'{n}번'): add(w,did,'dock')
        for w in _COLOR_WORDS[c]: add(w,did,'dock'); add(w+'도킹',did,'dock')
    return out
def rule(text,cmap,pcol,home):
    low=text.lower().replace(' ','')
    has=lambda *ws: any(w in low for w in ws)
    if has('지우','지워','삭제','비우','비워','초기화','처음부터','리셋'): return []
    if has('멈춰','정지','스톱') or has('되돌려','되돌리','취소','언두') or has('배속','속도','느리게','빠르게','천천히'): return []
    ment=[]
    for kw,oid,kind in vocab(cmap,pcol):
        p=low.find(kw)
        while p>=0: ment.append((p,len(kw),oid,kind)); p=low.find(kw,p+len(kw))
    ment.sort(key=lambda m:(m[0],-m[1]))
    used=[False]*len(low); filt=[]
    for p,L,oid,kind in ment:
        if any(used[p:p+L]): continue
        for i in range(p,p+L): used[i]=True
        filt.append((p,oid,kind))
    filt.sort(); seen=set(); order=[]
    for p,oid,kind in filt:
        if oid in seen: continue
        seen.add(oid); order.append((p,oid,kind))
    pens=[o for _,o,k in order if k=='brick']; docks=[o for _,o,k in order if k=='dock']
    pen0='ppap'; dock0=docks[0] if docks else None
    if re.findall(r'(\d+)\s*번',text) and not docks and has('꽂','넣','옮','놓','도킹','잡','이동'): return []
    act=has('잡','옮','꽂','넣','놓','열','닫','내려','내리','하강','올려','올리','상승','이동','로가','으로가','가줘','위로')
    if has('미리보','재생','돌려','보여') and not act: return []
    if has('옮','꽂','넣'):
        if dock0 is None: return []
        return [f'@goto:{home}','@grasp:ppap',f'@goto:{dock0}',f'@release:{dock0}']
    if has('잡'): return [f'@goto:{home}','@grasp:ppap']
    if has('놓'):
        if dock0 is None: return []
        return [f'@goto:{dock0}',f'@release:{dock0}']
    prim=[]
    for ws,item in ((('열',),'@open'),(('닫',),'@close'),(('내려','내리','하강'),'@down'),(('올려','올리','올라','상승'),'@up')):
        for w in ws:
            p=low.find(w)
            if p>=0: prim.append((p,item)); break
    if has('이동','로가','으로가','에가','가줘','가라','위로','올라가'):
        for p,o,k in order: prim.append((p,f'@goto:{o}'))
    prim.sort(); return [it for _,it in prim]

rows=[json.loads(l) for l in open('/home/drboom/2026_project/Industry_gyu_arm/nano_llm/val.jsonl',encoding='utf-8')][:200]
tot=collections.Counter(); ok=collections.Counter(); bad=[]
for d in rows:
    cmap,pcol,home=parse_scene(d['scene']); exp=json.loads(d['answer'])['blocks']
    got=rule(d['text'],cmap,pcol,home)
    k='거절' if not exp else f"{exp.count('@grasp:ppap')}단계"
    tot[k]+=1; hit=(got==exp); ok[k]+=hit
    if not hit and len(bad)<6: bad.append((k,d['text'][:40],exp[:4],got[:4]))
print(f"규칙 파서 200문장: {sum(ok.values())}/200 = {sum(ok.values())/2:.1f}%")
for k in sorted(tot): print(f"  {k:5s} {ok[k]:3d}/{tot[k]:3d} = {ok[k]/tot[k]:.0%}")
print("\n틀린 예:")
for k,t,e,g in bad: print(f"  [{k}] {t}\n     정답 {e}\n     규칙 {g}")
