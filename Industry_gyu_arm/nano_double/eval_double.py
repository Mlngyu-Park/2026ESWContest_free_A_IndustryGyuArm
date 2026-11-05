#!/usr/bin/env python3
"""더블 레이첼 채점 — 실제 말 98 (한 팔) + 손으로 쓴 두 팔 30.

  python eval_double.py double.pt [뽑는수=16] [온도=0.7]

세 가지 방식을 나란히:
  한번      : 가장 확률 높은 답 하나 그대로 실행
  한번+검사 : 검사(check.py)에 걸리면 실행하지 않는다 (거부)
  다수결    : 한번 + 여러 번 뽑기 → 검사 통과한 것만 모아 가장 많은 블록 → 없으면 거부
  각각 '맞음 / 틀린 채 실행 / 거부' 로 센다 — 틀린 채 실행이 가장 위험하다.
"""
import json, os, sys, time
from collections import Counter
import torch
HERE = os.path.dirname(os.path.abspath(__file__)); APP = os.path.dirname(HERE)
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(APP, "nano_norm"))
import train_double as T
from model import Rachel
from canon import parse_scene, scene_str, resolve
from check import check, split_out

ID = {'보라': 'd2', '빨강': 'd10', '주황': 'd11', '노랑': 'd12', '파랑': 'd13', '초록': 'd14'}

def load(path):
    ck = torch.load(path, map_location="cpu")
    itos = ck["vocab"]; stoi = {c: i for i, c in enumerate(itos)}
    T.CTX = ck["ctx"]
    m = Rachel(len(itos), d=ck["dim"], layers=ck["layers"], heads=ck["heads"], ctx=ck["ctx"])
    m.load_state_dict(ck["model"]); m.eval().to(T.DEV)
    return m, stoi, itos, ck

def real_items():
    out = []
    for it in json.load(open(f"{APP}/paper_evidence/15_REAL_VOICE_1028.json", encoding="utf-8")):
        g = it["gB"]
        cols = list(dict.fromkeys(['보라', '빨강', '주황'] + (g.split('→') if g != '[]' else [])))
        home = ID[g.split('→')[0]] if g != '[]' else 'd2'
        sc = dict(arms={"A": "빨강", "B": "파랑"}, docks={c: ID[c] for c in cols}, pen="초록", pen_home=home)
        if g == '[]':
            # 이송이 아닌 조작 명령은 그 명령이 정답 ('멈춰 새끼야' → stop, '천천히 해줘' → slower) — 10-30
            t = it["text"].replace(" ", "")
            gold = (["stop"] if any(w in t for w in ("멈춰", "정지", "스톱")) else
                    ["slower"] if any(w in t for w in ("천천히", "느리게")) else
                    ["faster"] if "빠르게" in t else [])
        else:
            st = [ID[x] for x in g.split('→')]; gold = [f"move_to:{st[0]}"]
            for d in st[1:]: gold += ["grab:pen", f"move_to:{d}", f"release:{d}"]
        kind = "이송" if (it["cmd"] == "O" and g != "[]") else "명령아님"
        out.append(dict(text=it["text"], sc=sc, gold=gold, kind=kind))
    return out

def hand_items():
    """손으로 쓴 세트: 1 = 사용자 말투 흉내, 2 = 다른 말투(존댓말·부탁·짧은 명령·번호),
    3 = 5차 결과를 보기 전에 새로 쓴 말씨 (4차/5차 공정 비교용)."""
    from canon import to_blocks
    out = []
    for tag, fn in (("1", "dual_eval_hand.json"), ("2", "dual_eval_hand2.json"), ("3", "dual_eval_hand3.json")):
        p = os.path.join(HERE, fn)
        if not os.path.exists(p): continue
        d = json.load(open(p, encoding="utf-8")); sc = parse_scene(d["scene"])
        out += [dict(text=i["text"], sc=sc, gold=to_blocks(i["canon"], sc), canon=i["canon"],
                     kind=f"세트{tag}" + ("·어려움" if i.get("hard") else "")) for i in d["items"]]
    return out

def run(m, stoi, itos, items, n, temp):
    R = {k: Counter() for k in ("한번", "한번+검사", "다수결")}; rows = []
    for it in items:
        s = scene_str(it["sc"])
        g = T.generate(m, stoi, itos, s, it["text"])[0]
        many = T.generate(m, stoi, itos, s, it["text"], temp=temp, n=n) if n else []
        def verdict(blocks): return "맞음" if blocks == it["gold"] else "틀린 채 실행"
        _, gb = split_out(g)
        try: gb = resolve(gb, it["sc"]) if gb is not None else None
        except Exception: gb = None
        R["한번"][verdict(gb)] += 1
        ok, why, cb = check(it["text"], g, it["sc"])
        R["한번+검사"][verdict(cb) if ok else "거부"] += 1
        votes = Counter()
        for o in [g] + many:
            ok2, _, b2 = check(it["text"], o, it["sc"])
            if ok2: votes[" ".join(b2)] += 1
        if votes:
            best = votes.most_common(1)[0][0]
            vb = best.split() if best else []
            R["다수결"][verdict(vb)] += 1
        else: vb = None; R["다수결"]["거부"] += 1
        rows.append(dict(kind=it["kind"], text=it["text"], out=g, ok=(gb == it["gold"]), why=why,
                         vote_ok=(vb == it["gold"]), votes=len(votes), gold=" ".join(it["gold"])))
    return R, rows

def show(name, R, total):
    for k, c in R.items():
        print(f"  {name:10s} {k:8s} 맞음 {c['맞음']:3d}/{total} ({c['맞음']/total*100:4.0f}%) · "
              f"틀린 채 실행 {c['틀린 채 실행']:3d} · 거부 {c['거부']:3d}")

def main():
    path = sys.argv[1]; n = int(sys.argv[2]) if len(sys.argv) > 2 else 16
    temp = float(sys.argv[3]) if len(sys.argv) > 3 else 0.7
    m, stoi, itos, ck = load(path)
    print(f"{path}: {ck.get('step')}걸음 · 합성 검증 블록까지 같음 {ck.get('exact', 0)*100:.1f}% · 뽑기 {n}개 온도 {temp}")
    t0 = time.time(); allrows = []
    for name, items in (("실제 말", real_items()), ("두 팔(손)", hand_items())):
        kinds = sorted(set(i["kind"] for i in items))
        for k in kinds:
            sub = [i for i in items if i["kind"] == k]
            R, rows = run(m, stoi, itos, sub, n, temp); allrows += rows
            show(f"{name}·{k}", R, len(sub))
    print(f"  ({time.time()-t0:.0f}초)")
    print("\n한번에 틀린 것:")
    for r in allrows:
        if not r["ok"]:
            c, _ = split_out(r["out"])
            print(f"  [{r['kind']}] {'(다수결로 맞춤)' if r['vote_ok'] else ''} {r['text'][:60]}\n      → {c[:110]}  {r['why'][:40]}")
    json.dump(allrows, open(os.path.join(HERE, "eval_out.json"), "w"), ensure_ascii=False, indent=1)

if __name__ == "__main__":
    main()
