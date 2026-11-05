#!/usr/bin/env python3
"""분포 외 시험 세트 — Gemma(DGX Spark)가 문장만 만들고, 정답은 우리가 정한다.

지도교수 조건 (2026-09-24)
  1. 학습 생성기(gen_data.py)와 다른 생성기: Gemma. 문장 틀 48개·예문은 프롬프트에 절대 안 넣는다.
  2. 정답은 Gemma 가 아니라 우리가 정한다: 장면과 목표(출발→경유→도착)를 주고 문장만 받는다.
     정답 블록은 생성 전에 이미 정해져 있다 → 채점 자동, 라벨링 편향 없음.
  3. 화자 3명 대신 말투 3종 × 100 = 300. 작업 분포는 val 과 맞춘다 (1단계 ½, 2단계+ ⅓, 나머지 거절·제자리).
  4. 학습 문장과 글자 그대로 겹치면 빼고 다시 채운다. 겹침 비율을 보고한다 (0 이어야 정상).
  5. 이 세트는 RACHEL 학습에 절대 넣지 않는다.

쓰는 법 (Spark LLM 이 localhost:18000 에 떠 있어야 한다):
    python gen_ood.py > ood_300.jsonl
"""
import json, random, sys, urllib.request
sys.path.insert(0, __file__.rsplit('/',1)[0])
from gen_data import scene, C_ADJ, COLORS   # 장면 생성기와 색 이름만 재사용 (문장 틀은 안 쓴다)
import hints

LLM = "http://localhost:18000/v1/chat/completions"
PERSONA = {
 "짧게": "아주 짧게, 명령조로, 군더더기 없이 한 문장.",
 "장황": "설명을 길게 덧붙이며, 중간에 '어', '그', '음' 같은 말이 섞이고, 한 번쯤 말을 고치기도 하는 자연스러운 구어체.",
 "존댓말": "정중한 존댓말로, '~해 주세요', '~부탁드립니다' 같은 표현을 쓰는 한 문장.",
}
DIST = [("1단계",0.50),("2단계",0.17),("3단계",0.08),("4단계",0.05),("5단계",0.03),("거절",0.07),("제자리",0.10)]

def pick_kind():
    r=random.random(); acc=0
    for k,p in DIST:
        acc+=p
        if r<acc: return k
    return "1단계"

def goal(kind, docks, cmap, home, pcol):
    """목표를 우리가 정하고, 사람말로 옮길 '뜻' 과 정답 블록을 만든다."""
    others=[d for d in docks if d!=home]
    col=lambda d: C_ADJ[cmap[d]]
    if kind=="거절":
        absent=[c for c in COLORS if c not in cmap.values()]
        t=random.choice([f"{C_ADJ[absent[0]] if absent else '회색'} 도킹으로 옮기라고 (그런 색 도킹은 없다)",
                         "로봇과 무관한 부탁 (커피, 날씨, 음악 등)"])
        return t, []
    if kind=="제자리":
        a=random.choice([("그리퍼를 열어라",["@open"]),("그리퍼를 닫아라",["@close"]),("팔을 내려라",["@down"]),("팔을 올려라",["@up"])])
        return a
    n=int(kind[0]); path=random.sample(others, min(n,len(others)))
    desc=f"{col(home)} 도킹에 있는 {C_ADJ[pcol]} 펜을 " + " 거쳐서 ".join(f"{col(d)} 도킹" for d in path) + " 으로 옮겨라"
    blocks=[f"@goto:{home}","@grasp:ppap"]
    for i,d in enumerate(path):
        if i: blocks.append("@grasp:ppap")
        blocks += [f"@goto:{d}", f"@release:{d}"]
    return desc, blocks

def ask(persona, desc, cmap):
    table=", ".join(f"{C_ADJ[c]} 도킹" for c in cmap.values())
    body=(f"로봇팔에게 말로 명령하는 사람의 문장을 하나 만들어라.\n"
          f"작업대에는 이런 도킹 스테이션들이 있다: {table}. 펜은 도킹 위에 꽂혀 있다.\n"
          f"말하려는 뜻: {desc}\n말투: {PERSONA[persona]}\n"
          f"색 이름과 순서는 뜻 그대로 지켜라. 문장만 출력하고 따옴표·설명은 붙이지 마라.")
    req=urllib.request.Request(LLM, data=json.dumps({"model":"gemma","messages":[{"role":"user","content":body}],
        "max_tokens":120,"temperature":1.0,"chat_template_kwargs":{"enable_thinking":False}}).encode(),
        headers={"Content-Type":"application/json"})
    r=json.loads(urllib.request.urlopen(req,timeout=120).read())
    return r["choices"][0]["message"]["content"].strip().strip('"').split("\n")[0]

def main(per=100):
    random.seed(20260924)
    train=set(json.loads(l)['text'] for l in open('train.jsonl',encoding='utf-8'))
    made=collections=0; dup=0; rows=[]
    for persona in PERSONA:
        n=0
        while n<per:
            docks,cmap,home,pcol,sc=scene(); kind=pick_kind()
            desc,blocks=goal(kind,docks,cmap,home,pcol)
            try: text=ask(persona,desc,cmap)
            except Exception as e: print("# LLM 실패:",e,file=sys.stderr); continue
            if not text or text in train: dup+=1; continue          # 조건 4
            h=hints.build(cmap,home,pcol,text)
            scene_hdr=f"{sc}\n힌트 {h}" if h else sc
            rows.append({"persona":persona,"kind":kind,"scene":scene_hdr,"text":text,
                         "answer":json.dumps({"blocks":blocks},ensure_ascii=False),"goal":desc})
            print(json.dumps(rows[-1],ensure_ascii=False),flush=True); n+=1
    print(f"# 완료 {len(rows)}문장, 학습과 완전 일치로 버린 것 {dup}건",file=sys.stderr)

if __name__=="__main__":
    main(int(sys.argv[1]) if len(sys.argv)>1 else 100)
