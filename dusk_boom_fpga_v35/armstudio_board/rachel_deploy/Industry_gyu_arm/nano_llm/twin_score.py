#!/usr/bin/env python3
"""3단계 성공 채점기 — 디지털 트윈에서 (실물 전송 없음)

  ① 블록 정확도  모델 블록 == 정답 블록                            (문자열 완전 일치)
  ② 계획 성공    검증기(_resolve_item) 통과 + seq_reach 도달 + plan_all 성공 + check_collision 0
  ③ 실행 성공    마지막 이동 구간의 종점 관절각을 fk() 로 풀어 툴 중심(cr_7)이
                 계획기가 겨눈 접근점 goto_point(도킹) 에서 XY·Z 각각 3 mm 안 (tool_pos = cr_7)
                 (END_TOL 6 raw ≈ 3 mm 근거)

배치 대응: 물리 배치에는 보이는 도킹이 3개(#2 #10 #11)뿐이다. 검증셋의 dN 은 출현 순서대로
  물리 도킹에 대응시킨다. 서로 다른 도킹이 4개 이상이면 '배치 밖' 으로 표시하고 ②③ 에서 뺀다.

쓰는 법:  cd Industry_gyu_arm && PYTHONPATH=. conda run -n gyu_arm python -u nano_llm/twin_score.py nano_llm/val_model_v9.jsonl 3000 > nano_llm/twin_score.jsonl
"""
import json, sys, io, contextlib, re, time
import numpy as np
with contextlib.redirect_stdout(io.StringIO()):
    import arm_studio_web as W
    W.h_factory_load('esw_contest')                     # 보라 d2 · 빨강 d10 · 주황 d11 · 펜 초록@d2
W.PROG = lambda *a, **k: None                           # 경로 계산 진행 로그 끄기
PHYS = [d for d in W.docking_ids() if W._env_ent(d) is None or W._env_ent(d).get('visible', True)]
PEN = next(b for b, v in W.BRICKS.items() if v.get('pos') is not None)
TOL = 0.003
HOVER = W.GRASP_FACE_DZ + W.GRASP_DZ_DEFAULT + W.GRASP_LIFT   # 도킹 윗면 -> 이동 블록 종점 높이

def remap(blocks):
    """d-번호 -> 물리 도킹 (출현 순서). 4개 이상이면 None."""
    order = []
    for b in blocks:
        m = re.match(r'@(goto|release):(d\d+)$', b)
        if m and m.group(2) not in order: order.append(m.group(2))
    if len(order) > len(PHYS): return None
    mp = dict(zip(order, PHYS)); out = []
    for b in blocks:
        m = re.match(r'@(goto|release):(d\d+)$', b)
        out.append(f'@{m.group(1)}:{mp[m.group(2)]}' if m else (b.replace(':ppap', ':' + PEN) if ':ppap' in b else b))
    return out

def score(blocks):
    if not blocks:                                    return None, None, '빈 블록'
    mapped = remap(blocks)
    if mapped is None:                                return None, None, '배치 밖(도킹 4개 이상)'
    res = [W._resolve_item(b) for b in mapped]
    if any(r is None for r in res):                   return False, False, '검증기 폐기'
    fx = W.seq_fix_moves(res); fx = fx[0] if isinstance(fx, tuple) else fx
    with W.LOCK: W.S['seq'] = fx
    reach = W.seq_reach()
    if any(not v['ok'] for v in reach.values()):     return False, False, '도달 불가'
    try: segs, err, _ = W.plan_all('simple', 'x-y-z', True)
    except Exception as e:                            return False, False, f'계획 예외 {type(e).__name__}'
    if segs is None:                                  return False, False, f'계획 실패: {str(err)[:40]}'
    for seg in segs:                                  # (라벨, 경로, ...) — 길이가 2 이상
        if W.check_collision(seg[1]):                 return False, False, '충돌 검출'
    last = next((b for b in reversed(fx) if b.startswith(W.RELEASE_PREFIX)), None)
    if last is None or not segs:                      return True, True, ''          # 이송 아닌 명령
    did = last[len(W.RELEASE_PREFIX):]; tgt = W.goto_point(did)          # 계획기가 겨눈 접근점 (소켓 위 여유높이)
    if tgt is None:                                   return True, False, '목표점 없음'
    p = np.asarray(W.tool_pos(segs[-1][1][-1]), float)                   # 툴 중심 cr_7 (j6 원점 아님)
    exy = float(np.hypot(p[0] - tgt[0], p[1] - tgt[1])); ez = abs(float(p[2] - tgt[2]))
    ok = exy <= TOL and ez <= TOL
    return True, ok, '' if ok else f'종점 오차 xy {exy*1000:.1f}mm z {ez*1000:.1f}mm'

if __name__ == '__main__':
    rows = [json.loads(l) for l in open(sys.argv[1], encoding='utf-8')][:int(sys.argv[2])]
    t0 = time.time(); cnt = {}
    for d in rows:
        exp = json.loads(d['answer'])['blocks']
        try: got = json.loads(d['model'])['blocks']
        except Exception: got = re.findall(r'@(?:goto|grasp|release):[A-Za-z0-9_#-]+|@open|@close|@down|@up', d['model'])
        s1 = (got == exp); s2, s3, why = score(got)
        k = ('block_ok' if s1 else 'block_x', s2, s3); cnt[k] = cnt.get(k, 0) + 1
        print(json.dumps({'i': d['i'], 'block_ok': s1, 'plan_ok': s2, 'exec_ok': s3, 'why': why}, ensure_ascii=False), flush=True)
    print(json.dumps({'_summary': {str(k): v for k, v in cnt.items()}, 'n': len(rows), 'sec': round(time.time() - t0, 1)}, ensure_ascii=False))
    sys.stdout.flush()
