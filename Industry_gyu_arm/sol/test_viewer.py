#!/usr/bin/env python3
"""
viewer.html 자동 시험 — 모든 기능을 실제로 눌러보고 상태를 확인한다.
    python3 test_viewer.py            (viewer.html 정적)
    python3 test_viewer.py --server   (serve.py 띄워서 검증/저장까지)
"""
import asyncio, os, sys, json, subprocess, time, shutil
from playwright.async_api import async_playwright
HERE=os.path.dirname(os.path.abspath(__file__))
SHOT=os.path.join('/tmp/claude-1000/-home-drboom-2026-project/abc3193e-5a0b-4ca0-8267-3cffbca067a7/scratchpad')

fails=[]
def check(cond, msg):
    print(("  ok   " if cond else "  FAIL ")+msg)
    if not cond: fails.append(msg)

async def run(url):
    async with async_playwright() as p:
        b=await p.chromium.launch(); pg=await b.new_page(viewport={'width':1500,'height':900})
        errs=[]; pg.on('pageerror',lambda e:errs.append('pageerror: '+str(e))); pg.on('console',lambda m: errs.append('console: '+m.text) if m.type=='error' else None)
        await pg.goto(url); await pg.wait_for_timeout(800)
        ev=lambda js: pg.evaluate(js)
        print("[로드]"); check(await ev("!!window.__sol"), "hook 존재"); check(not errs, "JS 오류 없음 "+str(errs))
        n0=await ev("Object.keys(__sol.D.nets).length"); print("   네트", n0)

        print("[3D/2D/뒤집기/선모양]")
        for js,label in (("document.getElementById('b2d').click(); __sol.S.mode","2d"),("document.getElementById('b3d').click(); __sol.S.mode","3d")):
            check((await ev(js))==label, "모드 "+label)
        await pg.keyboard.press('v'); check((await ev("__sol.S.mode"))=='2d', "키 V → 2D")
        await pg.keyboard.press('f'); check((await ev("__sol.S.flipped"))==True, "키 F → 뒤집힘")
        check('앞면' in (await ev("document.getElementById('bflip').textContent")), "뒤집기 버튼 문구 갱신")
        await pg.keyboard.press('f')
        for bid,mode in (('wStraight','straight'),('wBack','front'),('wMan','manhattan')):
            await pg.click('#'+bid); check((await ev("__sol.S.wire"))==mode, "선모양 "+mode)
        # 뒷면우회에서 자동경로 side 가 back 인지
        # 라우터는 앞/뒤를 오가므로(바느질) '첫 구간의 면' 이 아니라 전체 비율로 본다
        frac=lambda: ev("(()=>{ let f=0,b=0; for(const n in __sol.D.nets){ if(__sol.D.nets[n].supply) continue; (__sol.autoRoute(n)||[]).forEach(s=>{ if(s.fail) return; if(s.side==='front') f++; else b++; }); } return {f,b,front:f/(f+b)}; })()")
        await pg.click('#wBack'); r=await frac(); check(r['front']>0.30, f"앞면 모드: 앞면 비율 {r['front']:.0%} (앞 {r['f']}/뒤 {r['b']})")
        await pg.click('#wMan');  r=await frac(); check(r['front']<0.50, f"기본(뒷면 위주): 앞면 비율 {r['front']:.0%} (앞 {r['f']}/뒤 {r['b']})")
        vias=await ev("(()=>{ let v=0; for(const n in __sol.D.nets){ if(__sol.D.nets[n].supply) continue; (__sol.autoRoute(n)||[]).forEach(s=>{ if(s.viaStart) v++; }); } return v; })()")
        check(vias>0, f"바느질(관통) {vias}곳")
        # 겹침 검사 — 선이 지나는 '모든' 칸을 센다.
        #   · 구멍(정수 좌표)도 센다        (예전엔 건너뛰어서 구멍 위 교차를 놓쳤다)
        #   · 구간의 양 끝도 센다           (예전엔 k=1..nn-1 이라 꺾이는 점에서 만나는 걸 놓쳤다)
        #   · 층을 키에 넣는다              (예전엔 side 만 봐서 다른 층을 같은 칸으로 셌다)
        #   · Set 으로 세어 3가닥 이상도 정확히 잡는다 (예전엔 덮어써서 2로 셌다)
        rules=await ev("""(()=>{ const res=__sol.computeAllRoutes(); let diag=0;
          const corr={}, hole={};
          for(const n in res){ if(__sol.D.nets[n].supply) continue; res[n].forEach(sg=>{ if(sg.fail) return;
            const L=sg.layer||1, pre=L+':'+sg.side+':';
            for(let i=1;i<sg.pts.length;i++){ const [x0,y0]=sg.pts[i-1],[x1,y1]=sg.pts[i];
              if(x0!==x1&&y0!==y1){diag++;continue;}
              const dx=Math.sign(x1-x0),dy=Math.sign(y1-y0),nn=Math.round((Math.abs(x1-x0)+Math.abs(y1-y0))*2);
              for(let k=0;k<=nn;k++){ const c=x0+dx*k/2,r=y0+dy*k/2;
                const m=(Number.isInteger(c)&&Number.isInteger(r))?hole:corr;
                (m[pre+c+','+r]=m[pre+c+','+r]||new Set()).add(n); } } }); }
          const cap=Math.max(1,__sol.S.holeCap|0);
          const overlap=Object.values(corr).filter(v=>v.size>1).length;
          const holeOver=Object.values(hole).filter(v=>v.size>cap).length;
          return {diag, overlap, holeOver}; })()""")
        check(rules['overlap']==0, f"통로에서 선 겹침 0 (실제 {rules['overlap']})")
        # 두 번째 방법(셀 점유와 무관한 순수 기하): 같은 층·같은 면의 두 선분이 실제로 교차하는가
        geo=await ev("""(()=>{ const res=__sol.computeAllRoutes(); const ln=[];
          for(const n in res){ if(__sol.D.nets[n].supply) continue; res[n].forEach(sg=>{ if(sg.fail) return;
            for(let i=1;i<sg.pts.length;i++) ln.push({n, s:sg.side, L:sg.layer||1, a:sg.pts[i-1], b:sg.pts[i]}); }); }
          const x=(p,q,r,t)=>{ const d=(q[0]-p[0])*(t[1]-r[1])-(q[1]-p[1])*(t[0]-r[0]); if(Math.abs(d)<1e-9) return false;
            const u=((r[0]-p[0])*(t[1]-r[1])-(r[1]-p[1])*(t[0]-r[0]))/d, v=((r[0]-p[0])*(q[1]-p[1])-(r[1]-p[1])*(q[0]-p[0]))/d;
            return u>=-1e-9&&u<=1+1e-9&&v>=-1e-9&&v<=1+1e-9; };
          let c=0; for(let i=0;i<ln.length;i++) for(let j=i+1;j<ln.length;j++){ const A=ln[i],B=ln[j];
            if(A.n===B.n||A.L!==B.L||A.s!==B.s) continue; if(x(A.a,A.b,B.a,B.b)) c++; } return c; })()""")
        check(geo==0, f"기하 검사: 같은 면 선분 교차 0 (실제 {geo})")
        ends=await ev("""(()=>{ const res=__sol.computeAllRoutes(); let bad=0; for(const n in res){ if(__sol.D.nets[n].supply) continue; const segs=res[n].filter(s=>!s.fail); if(!segs.length) continue;
          const pinsAt=new Set((__sol.D.nets[n].pins||[]).map(([r,k])=>{const q=__sol.findPin(r,k); return q.col+','+q.row;}));
          segs.forEach(sg=>{ [sg.pts[0], sg.pts[sg.pts.length-1]].forEach(pt=>{ if(pinsAt.has(pt[0]+','+pt[1]) && sg.side!=='back') bad++; }); }); } return bad; })()""")
        check(ends==0, f"핀에 닿는 선 끝은 전부 뒷면 (앞면에서 닿은 것 {ends})")
        # 3D 미배선 고무줄: 네트 선택/해제(스타일 재적용) 뒤에도 빨강 유지
        await ev("__sol.setMode('3d'); __sol.draw()"); await pg.wait_for_timeout(1500)
        redv=await ev("""(()=>{ const g=window.__gl3d; if(!g) return null; const f=g.wireMeshes.filter(m=>m.userData.fail); const n=Object.keys(__sol.D.nets).find(k=>!__sol.D.nets[k].supply);
          __sol.showNet(n); __sol.draw(); __sol.showNet(null); __sol.draw(); return f.length? f.every(m=>m.material.color.getHexString()==='ff2d20') : null; })()""")
        check(redv in (True,None), f"3D 미배선 고무줄이 스타일 재적용 뒤에도 빨강 ({redv})")
        await ev("__sol.setMode('2d'); __sol.draw()")
        await pg.click('#sLift'); await pg.wait_for_timeout(120); check((await ev("__sol.S.lift"))==True and not errs, "칩 뽑아 올리기 토글"); await pg.click('#sLift')

        check(rules['holeOver']==0, f"구멍에서 한도 초과 0 (실제 {rules['holeOver']})")
        check(rules['diag']==0, f"비스듬한 선 0 (실제 {rules['diag']})")
        # 점(구멍) 규칙: 설정한 최대 개수 이하 + 같은 점을 쓰는 선끼리 색이 달라야 한다
        holeJS = """(cap)=>{ __sol.S.holeCap=cap; __sol.bumpGeom(); const res=__sol.computeAllRoutes();
          const hole={}, col={}; let worst=0, clash=0;
          const nc=n=>__sol.netColor(n);   // 그룹색 + 네트별 음영 (한 구멍을 나눠 써도 눈으로 구분됨)
          for(const n in res){ if(__sol.D.nets[n].supply) continue;
            res[n].forEach(sg=>{ if(sg.fail) return; const L=(sg.layer||1)+':'+sg.side;   // ★ 층·면이 다르면 다른 점이다 (앞/뒤는 기판이 갈라놓는다)
              for(let i=1;i<sg.pts.length;i++){ const [x0,y0]=sg.pts[i-1],[x1,y1]=sg.pts[i];
                // ★ 좌표는 반칸(0.5) 단위다. 정수 칸으로 훑으면 2.5→37 같은 구간에서
                //    있지도 않은 정수 좌표가 만들어져 엉뚱한 구멍을 세게 된다.
                const dx=Math.sign(x1-x0),dy=Math.sign(y1-y0),nn=Math.round((Math.abs(x1-x0)+Math.abs(y1-y0))*2);
                for(let k=0;k<=nn;k++){ const c=x0+dx*k/2,r=y0+dy*k/2; if(!Number.isInteger(c)||!Number.isInteger(r)) continue;
                  const kk=L+':'+c+','+r;
                  (hole[kk]=hole[kk]||new Set()).add(n); (col[kk]=col[kk]||new Set()).add(nc(n)); } } }); }
          for(const k in hole){ worst=Math.max(worst,hole[k].size); if(hole[k].size>col[k].size) clash++; }
          return {worst, clash}; }"""
        for cap in (1,2,3):
            h=await pg.evaluate(holeJS, cap)
            check(h['worst']<=cap, f"점당 {cap} 설정 → 한 점 최대 {h['worst']}개")
            check(h['clash']==0, f"점당 {cap}: 같은 색끼리 한 점 공유 0 (실제 {h['clash']})")
        await ev("__sol.S.holeCap=1; __sol.bumpGeom(); __sol.draw()")   # 기본값(겹침 없음)으로 복귀
        tack=await ev("(()=>{ let t=0; for(const n in __sol.D.nets){ if(__sol.D.nets[n].supply) continue; (__sol.autoRoute(n)||[]).forEach(s=>{ if(s.tack) t++; }); } return t; })()")
        check(tack>0, f"긴 직선 중간 고정(바느질) {tack}곳")

        print("[숨기기/슬라이더/색]")
        for cid in ('sChips','sSockets','sCaps','sPins','sWires','sHoles','sLabels','sPower','sPinNo','sPinName'):
            await pg.click('#'+cid); await pg.click('#'+cid)
        check(not errs, "토글 후 JS 오류 없음 "+str(errs[-2:]))
        await ev("const s=document.getElementById('step'); s.value=0; s.dispatchEvent(new Event('input'))"); check((await ev("__sol.S.step"))==0, "납땜 순서 0")
        await ev("const s=document.getElementById('step'); s.value=s.max; s.dispatchEvent(new Event('input'))")
        await ev("const i=document.querySelector('#groupList input[type=color]'); i.value='#123456'; i.dispatchEvent(new Event('input'))")
        check('#123456' in json.dumps(await ev("__sol.colors")), "그룹 색 변경 반영")
        await ev("document.querySelector('#partList input').click()"); check((await ev("__sol.S.hidden.size"))==1, "부품 숨김"); await ev("document.querySelector('#partList input').click()")

        print("[2D 부품 끌기]")
        await ev("__sol.setMode('2d'); __sol.fitView(); __sol.draw()")
        z=await ev("__sol.S.zoom"); check(z>1.2 and z<20, f"fit 줌 {z:.2f}")
        ref=await ev("__sol.D.parts.find(p=>p.pkg==='DIP20').ref")
        before=await ev(f"[__sol.partMap['{ref}'].col,__sol.partMap['{ref}'].row]")
        # 부품 중심 화면좌표
        cxy=await ev(f"""(()=>{{const p=__sol.partMap['{ref}'];const P=__sol.D.pitch;const cs=p.pins.map(q=>q.col),rs=p.pins.map(q=>q.row);const cx=(Math.min(...cs)+Math.max(...cs))/2*P, cy=(Math.min(...rs)+Math.max(...rs))/2*P;const W=document.getElementById('cv').getBoundingClientRect();const S=__sol.S;const bw=__sol.D.board[0]*P,bh=__sol.D.board[1]*P;return [W.width/2+(cx-bw/2)*S.zoom+S.panX, W.height/2+(cy-bh/2)*S.zoom+S.panY];}})()""")
        box=await pg.eval_on_selector('#cv','e=>{const r=e.getBoundingClientRect();return [r.left,r.top]}')
        sx,sy=box[0]+cxy[0], box[1]+cxy[1]
        probe=await ev(f"(__sol.hitPart({cxy[0]},{cxy[1]})||{{}}).ref"); check(probe=='{ref}'.format(ref=ref), f"hitPart 가 {ref} 를 잡음 (실제: {{probe}})".format(probe=probe))
        dz=int(z*3)+1
        await pg.mouse.move(sx,sy); await pg.mouse.down(); await pg.mouse.move(sx+dz,sy+dz,steps=5); await pg.mouse.move(sx+dz+1,sy+dz,steps=2); await pg.mouse.up()
        after=await ev(f"[__sol.partMap['{ref}'].col,__sol.partMap['{ref}'].row]")
        check(after!=before, f"부품 {ref} 이동 {before}→{after}")
        pin1=await ev(f"[__sol.partMap['{ref}'].pins[0].col,__sol.partMap['{ref}'].pins[0].row]"); check(pin1==after, "이동 후 1번 핀 좌표 = 부품 좌표")
        net_ok=await ev(f"__sol.partMap['{ref}'].pins.every(q=>q.net==null||__sol.D.nets[q.net].pins.some(([r,n])=>r==='{ref}'&&n===q.n))"); check(net_ok, "이동해도 연결 유지 (포인터)")

        print("[회전]")
        await ev(f"document.activeElement&&document.activeElement.blur(); __sol.S.hoverPart='{ref}'"); await pg.keyboard.press('r'); check((await ev(f"__sol.partMap['{ref}'].rot"))==90, "R 키 회전 90")
        await ev(f"__sol.rotatePart(__sol.partMap['{ref}']); __sol.rotatePart(__sol.partMap['{ref}']); __sol.rotatePart(__sol.partMap['{ref}'])"); check((await ev(f"__sol.partMap['{ref}'].rot"))==0, "4번 회전 → 0")
        check(pin1==await ev(f"[__sol.partMap['{ref}'].pins[0].col,__sol.partMap['{ref}'].pins[0].row]"), "회전 원복 후 핀 좌표 원복")

        print("[네트 선택: 핀 클릭 / 선 클릭]")
        W_,H_=await ev("[document.getElementById('cv').getBoundingClientRect().width, document.getElementById('cv').getBoundingClientRect().height]")
        h=await ev(f"__sol.hits.find(h=>h.ref&&h.net!=null&&!__sol.D.nets[h.net].supply&&h.x>5&&h.x<{W_}-5&&h.y>25&&h.y<{H_}-40)")
        await pg.mouse.click(box[0]+h['x'], box[1]+h['y']); check((await ev("__sol.S.selNet"))==h['net'], f"핀 클릭 → 네트 {h['net']} 선택")
        check(str(h['net']) in (await ev("document.getElementById('netInfo').textContent")), "네트 정보 패널 갱신")
        await ev("__sol.showNet(null)")
        seg=await ev(f"(()=>{{const s=__sol.segHits.find(s=>!__sol.D.nets[s.net].supply&&Math.hypot(s.x2-s.x1,s.y2-s.y1)>30&&Math.min(s.x1,s.x2)>5&&Math.max(s.x1,s.x2)<{W_}-5&&Math.min(s.y1,s.y2)>25&&Math.max(s.y1,s.y2)<{H_}-40);if(!s)return null;return {{x:(s.x1+s.x2)/2,y:(s.y1+s.y2)/2,net:s.net}};}})()")
        if seg:
            await pg.mouse.click(box[0]+seg['x'], box[1]+seg['y']); check((await ev("__sol.S.selNet"))==seg['net'], f"선 중간 클릭 → 네트 {seg['net']} 선택")
        else: check(False, "선분 후보 없음")

        print("[경로 그리기]")
        await ev("__sol.showNet(__sol.S.selNet)"); net=await ev("__sol.S.selNet")
        await pg.click('#bRoute'); check((await ev("__sol.S.routing"))==True, "경로 모드 진입")
        await pg.mouse.click(box[0]+400, box[1]+300); await pg.mouse.click(box[0]+500, box[1]+300); await pg.mouse.click(box[0]+500, box[1]+400)
        check((await ev("__sol.S.routePts.length"))==3, "경유점 3개")
        await pg.click('#bRoute'); check((await ev(f"(__sol.routes['{net}']||[]).length"))==1, "경로 저장")
        check((await ev(f"(__sol.autoRoute('{net}')[0]||{{}}).manual"))==True, "자동배선이 손경로 사용")
        await pg.click('#bRouteClear'); check((await ev(f"!__sol.routes['{net}']"))==True, "경로 지우기")

        print("[연결 편집]")
        # 서로 다른 두 부품의 NC/out 핀 하나씩 이어본다: 안 붙은 핀 2개 찾기
        free=await ev("(()=>{const out=[];for(const p of __sol.D.parts){for(const q of p.pins){if(q.net==null&&q.kind!=='vcc'&&q.kind!=='gnd'){out.push([p.ref,q.n]);if(out.length==2)return out;}}}return out;})()")
        check(len(free)==2, f"미연결 핀 2개 찾음 {free}")
        if len(free)==2:
            nn=await ev(f"__sol.linkPins({{ref:'{free[0][0]}',pin:{free[0][1]}}},{{ref:'{free[1][0]}',pin:{free[1][1]}}})")
            check(await ev(f"__sol.D.nets['{nn}'].pins.length===2"), f"새 네트 {nn} 에 핀 2개")
            check(await ev(f"__sol.findPin('{free[0][0]}',{free[0][1]}).net==='{nn}'"), "핀 객체 net 갱신")
            await ev(f"__sol.setPinNet('{free[0][0]}',{free[0][1]},null)")
            check(await ev(f"__sol.D.nets['{nn}'].pins.length===1"), "핀 떼기 → 네트에 1개")
            await ev(f"__sol.setPinNet('{free[1][0]}',{free[1][1]},null)")
            check(await ev(f"!__sol.D.nets['{nn}']"), "빈 네트 자동 삭제")
        # 네트 병합: 두 기존 신호 네트의 핀을 이으면 하나로
        pair=await ev("(()=>{const ns=Object.keys(__sol.D.nets).filter(n=>!__sol.D.nets[n].supply&&__sol.D.nets[n].pins.length>=2);return [ns[0],ns[1],__sol.D.nets[ns[0]].pins[0],__sol.D.nets[ns[1]].pins[0],__sol.D.nets[ns[0]].pins.length+__sol.D.nets[ns[1]].pins.length];})()")
        before_n=await ev("Object.keys(__sol.D.nets).length")
        keep=await ev(f"__sol.linkPins({{ref:'{pair[2][0]}',pin:{pair[2][1]}}},{{ref:'{pair[3][0]}',pin:{pair[3][1]}}})")
        check(await ev(f"__sol.D.nets['{keep}'].pins.length")==pair[4], f"네트 병합 {pair[0]}+{pair[1]} → {keep}, 핀 {pair[4]}")
        check(await ev("Object.keys(__sol.D.nets).length")==before_n-1, "병합 후 네트 수 -1")

        print("[연결 편집 — UI 경유 (2D)]")
        await ev("__sol.setMode('2d'); __sol.fitView(); __sol.showNet(null); __sol.S.selPin=null; __sol.draw()")
        W_,H_=await ev("[document.getElementById('cv').getBoundingClientRect().width, document.getElementById('cv').getBoundingClientRect().height]")
        two=await ev(f"(()=>{{const out=[];for(const h of __sol.hits){{ if(!h.ref||h.net!=null) continue; if(out.length&&out[0].ref===h.ref) continue; if(h.x>5&&h.x<{W_}-5&&h.y>25&&h.y<{H_}-40){{ out.push({{x:h.x,y:h.y,ref:h.ref,pin:h.pin}}); if(out.length==2) break; }} }} return out;}})()")
        check(len(two)==2, f"미연결 핀 2개(화면 안) {two and [(a['ref'],a['pin']) for a in two]}")
        if len(two)==2:
            await pg.mouse.click(box[0]+two[0]['x'], box[1]+two[0]['y']); await pg.wait_for_timeout(80)
            check(await ev("__sol.S.selPin&&__sol.S.selPin.ref")==two[0]['ref'], "핀 클릭 → selPin.ref 있음")
            await pg.click('#bLink'); await pg.mouse.click(box[0]+two[1]['x'], box[1]+two[1]['y']); await pg.wait_for_timeout(120)
            n1=await ev(f"__sol.findPin('{two[0]['ref']}',{two[0]['pin']}).net"); n2=await ev(f"__sol.findPin('{two[1]['ref']}',{two[1]['pin']}).net")
            check(n1 is not None and n1==n2, f"UI 로 두 핀 잇기 → 네트 {n1}")
            await ev(f"__sol.setPinNet('{two[0]['ref']}',{two[0]['pin']},null); __sol.setPinNet('{two[1]['ref']}',{two[1]['pin']},null)")
        print("[KYLE 내보내기 / 적용 왕복]")
        await pg.click('#bExport'); txt=await ev("document.getElementById('solText').value"); open(os.path.join(HERE,'.roundtrip.txt'),'w',encoding='utf-8').write(txt)
        check(txt.startswith('#KYLE 1'), "KYLE 헤더"); check(txt.count('\npin ')>400, f"pin 줄 {txt.count(chr(10)+'pin ')}개"); check('part    ' in txt and 'color   ' in txt, "part/color 줄")
        snap=await ev("JSON.stringify(__sol.D.parts.map(p=>[p.ref,p.col,p.row,p.rot,p.pins.map(q=>q.net)]))")
        await ev(f"__sol.partMap['{ref}'].col+=5; __sol.recomputePins(__sol.partMap['{ref}'])")
        await pg.click('#bImport'); snap2=await ev("JSON.stringify(__sol.D.parts.map(p=>[p.ref,p.col,p.row,p.rot,p.pins.map(q=>q.net)]))")
        check(snap==snap2, "내보내기→적용 후 배치·연결 동일 (왕복 무손실)")
        # 되읽은 뒤에도 겹침 0 이어야 한다 (저장된 경로가 칸을 안 잡으면 나머지가 그 위로 지나가 '미배선이 줄어든 척' 한다)
        ov=await ev("""(()=>{ const occ={}; let bad=0; for(const n in __sol.D.nets){ if(__sol.D.nets[n].supply) continue; (__sol.autoRoute(n)||[]).forEach(sg=>{ if(sg.fail||sg.rail) return; const L=sg.layer||1;
          for(let i=1;i<sg.pts.length;i++){ const [x0,y0]=sg.pts[i-1],[x1,y1]=sg.pts[i]; const dx=Math.sign(x1-x0),dy=Math.sign(y1-y0),nn=Math.round((Math.abs(x1-x0)+Math.abs(y1-y0))*2);
            for(let k=0;k<=nn;k++){ const key=L+':'+sg.side+':'+(x0*2+dx*k)+','+(y0*2+dy*k); if(occ[key]&&occ[key]!==n) bad++; occ[key]=n; } } }); } return bad; })()""")
        check(ov==0, f"되읽은 뒤 선 겹침 0 (실제 {ov})")
        check(txt.count('\nroute ')>50 and ' auto ' in txt and '\nbus ' in txt and '\nexpect ' in txt, f"내보내기에 route(auto)/bus/expect 포함 (route {txt.count(chr(10)+'route ')})")
        rr=subprocess.run([sys.executable,os.path.join(HERE,'sol.py'),os.path.join(HERE,'.roundtrip.txt')],capture_output=True,text=True).stdout
        r28=[l for l in rr.split('\n') if 'R28' in l and '정보' in l]; check(bool(r28) and '위반 0건' in r28[0], "sol.py R28 (독립 구현) 이 내보낸 배선을 검사: "+(r28[0][-40:] if r28 else '없음'))

        print("[반응형 · 다크]")
        await pg.set_viewport_size({'width':420,'height':800}); await pg.wait_for_timeout(200); check(not errs, "좁은 화면 오류 없음")
        await pg.emulate_media(color_scheme='dark'); await ev("__sol.draw()"); check(not errs, "다크 테마 오류 없음")
        await pg.set_viewport_size({'width':1500,'height':900}); await pg.wait_for_timeout(200)
        await ev("__sol.setMode('3d'); __sol.S.hoverPart=null; __sol.showNet(null); __sol.draw()")
        await pg.screenshot(path=os.path.join(SHOT,'t_3d.png'))
        await ev("__sol.setMode('2d'); __sol.setFlip(true)"); await pg.screenshot(path=os.path.join(SHOT,'t_2d_back.png'))

        if '--server' in sys.argv:
            print("[서버: 검증/저장]")
            await ev("__sol.setFlip(false)")
            check(not (await ev("document.getElementById('bVerify').disabled")), "검증 버튼 활성")
            await pg.click('#bVerify'); await pg.wait_for_timeout(1500)
            hint=await ev("document.getElementById('srvHint').textContent"); check('갱신' in hint, "검증 다시 실행: "+hint)
            check('MHz' in (await ev("document.getElementById('timing').textContent")), "타이밍 갱신")
            await pg.click('#bSave'); await pg.wait_for_timeout(1500)
            hint=await ev("document.getElementById('srvHint').textContent"); check('저장됨' in hint, "파일 저장: "+hint)
        print("[JS 오류 총합]", errs if errs else "없음"); check(not errs, "JS 오류 0")
        await b.close()

async def main():
    if '--server' in sys.argv:
        # 시험은 고정 설계(fixture_2pe.txt)를 쓴다 — pe2.txt 는 사용자가 기판·PE 수를 바꾸는 작업 파일이라 기준이 될 수 없다
        tmp=os.path.join(HERE,'.pe2_test.txt'); shutil.copy(os.path.join(HERE,'fixture_2pe.txt'),tmp)
        srv=subprocess.Popen([sys.executable,os.path.join(HERE,'serve.py'),tmp],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,env=dict(os.environ,BROWSER='true'))
        time.sleep(1.2)
        try: await run('http://127.0.0.1:8765/')
        finally:
            srv.terminate()
            saved=open(tmp,encoding='utf-8').read(); print("   저장된 파일 pin 줄:", saved.count('\npin '))
            r=subprocess.run([sys.executable,os.path.join(HERE,'sol.py'),tmp],capture_output=True,text=True); print("   저장본 재검증:", [l for l in r.stdout.split('\n') if l.startswith('치명')])
            for f in (tmp,tmp+'.bak'):
                if os.path.exists(f): os.remove(f)
    else:
        # 정적 시험도 '지금 템플릿' 으로 고정 설계를 새로 빌드한다 (예전엔 묵은 viewer.html 을 열어 옛 라우터를 시험했다)
        import sol
        d=sol.parse_design(os.path.join(HERE,'fixture_2pe.txt')); R,ni,tim=sol.verify(d)
        js=os.path.join(HERE,'.test_viewer.json'); html=os.path.join(HERE,'.test_viewer.html')
        sol.export_json(d,R,ni,tim,js); subprocess.run([sys.executable,os.path.join(HERE,'build_viewer.py'),js,html],check=True,capture_output=True)
        try: await run('file://'+html)
        finally:
            for f in (js,html):
                if os.path.exists(f): os.remove(f)
    print("\n결과:", "전부 통과" if not fails else f"실패 {len(fails)}건: "+"; ".join(fails))
    return 0 if not fails else 1
sys.exit(asyncio.run(main()))
