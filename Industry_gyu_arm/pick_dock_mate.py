#!/usr/bin/env python
"""도킹 메이트 선택 도구 — ARM STUDIO 본체와 분리된 1회용 코드.

솔리드웍스 메이트처럼 클릭 2번으로 "펜이 도킹에 어떻게 꽂히는가"를 정한다:
  ① 도킹(위에서 본 모습)에서 소켓 자리 클릭   -> 꽂히는 축 XY (동심)
     · 주황 원 후보를 클릭하거나, 아무 곳이나 자유 클릭해도 됨
     · 도킹 쪽 높이는 '맨 윗면' 자동
  ② 펜(옆모습)에서 원(림) 클릭               -> 그 원이 도킹 맨 윗면 높이에
                                                오도록 꽂히는 깊이 자동 결정
저장하면 dock_mate_fixed.py 에 하드코딩된다.

사용:
    ./pick_mate.sh            # 또는
    conda run -n gyu_arm python pick_dock_mate.py [도킹.STL] [펜.STL]
"""
import glob, http.server, json, math, os, socketserver, sys, threading
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OUT  = os.path.join(HERE, 'dock_mate_fixed.py')
PORT_RANGE = range(7950, 7970)
STL_DIR   = '/home/drboom/2026_project/stl'
BRICK_DIR = '/home/drboom/2026_project/urdf/bricks'

def find_meshes():
    dock = sys.argv[1] if len(sys.argv) > 1 else None
    pen  = sys.argv[2] if len(sys.argv) > 2 else None
    if not dock:
        c = [p for p in glob.glob(os.path.join(STL_DIR, '*.STL'))
             if 'docking' in p.lower()]
        dock = sorted(c)[0] if c else None
    if not pen:
        c = (glob.glob(os.path.join(BRICK_DIR, '*', 'meshes', '*l1*.STL'))
             or glob.glob(os.path.join(BRICK_DIR, '*', 'meshes', '*.STL')))
        pen = sorted(c)[0] if c else None
    return dock, pen

def _fit_circle(p):
    A = np.c_[p[:, 0], p[:, 1], np.ones(len(p))]
    b = (p ** 2).sum(1)
    try:
        s, *_ = np.linalg.lstsq(A, b, rcond=None)
    except Exception:
        return None
    cx, cy = s[0] / 2, s[1] / 2
    r2 = s[2] + cx * cx + cy * cy
    if r2 <= 0:
        return None
    r = float(np.sqrt(r2))
    res = float(np.abs(np.linalg.norm(p - [cx, cy], axis=1) - r).std())
    return float(cx), float(cy), r, res

def detect_circles(path, mul):
    """원통(벽/구멍/원호) 후보: [{cx,cy,r,zmin,zmax}] + 점구름(mm)."""
    import trimesh
    from scipy.cluster.hierarchy import fclusterdata
    m = trimesh.load(path)
    v = np.asarray(m.vertices, float) * mul
    n = m.face_normals
    f = np.asarray(m.faces)
    w = v[np.unique(f[np.abs(n[:, 2]) < 0.35])]
    cands = []
    if len(w) >= 5:
        for t in (3.0, 15.0):               # 촘촘한 구멍 + 성긴 원호 둘 다
            lab = fclusterdata(w[:, :2], t=t, criterion='distance')
            for k in set(lab):
                g = w[lab == k]
                if len(g) < 5:
                    continue
                fc = _fit_circle(g[:, :2])
                if not fc:
                    continue
                subs = [g]
                cx, cy, r, st = fc
                if st > max(0.8, r * 0.1) and len(g) >= 10:
                    ri = np.linalg.norm(g[:, :2] - [cx, cy], axis=1)
                    l2 = fclusterdata(ri.reshape(-1, 1), t=1.2,
                                      criterion='distance')
                    subs = [g[l2 == j] for j in set(l2) if (l2 == j).sum() >= 5]
                for sg in subs:
                    fc2 = _fit_circle(sg[:, :2])
                    if not fc2:
                        continue
                    cx2, cy2, r2_, st2 = fc2
                    if not (0.5 < r2_ < 40) or st2 > max(0.8, r2_ * 0.1):
                        continue
                    cands.append({'cx': round(cx2, 2), 'cy': round(cy2, 2),
                                  'r': round(r2_, 2),
                                  'zmin': round(float(sg[:, 2].min()), 2),
                                  'zmax': round(float(sg[:, 2].max()), 2)})
    # 중복 제거 (중심 1mm · 반지름 0.8mm 이내)
    ded = []
    for c in cands:
        if not any(abs(c['cx'] - d['cx']) < 1 and abs(c['cy'] - d['cy']) < 1
                   and abs(c['r'] - d['r']) < 0.8 for d in ded):
            ded.append(c)
    def cloud(idx):
        q = v[:, idx]
        if len(q) > 4000:
            q = q[np.random.RandomState(0).choice(len(q), 4000, replace=False)]
        return q.round(2).tolist()
    return (ded, cloud([0, 1]), cloud([0, 2]),
            round(float(v[:, 2].min()), 2), round(float(v[:, 2].max()), 2))

PAGE = """<!doctype html><html lang="ko"><head><meta charset="utf-8">
<title>도킹 메이트 선택</title>
<style>
 body{margin:0;background:#0e1116;color:#e7ebf3;font-family:sans-serif;
      display:flex;flex-direction:column;align-items:center;gap:10px;padding:14px}
 h2{margin:2px} .hint{color:#8b93a7;font-size:13px;text-align:center;max-width:900px}
 #panes{display:flex;gap:14px;flex-wrap:wrap;justify-content:center}
 .pane{display:flex;flex-direction:column;align-items:center;gap:6px}
 .pane h3{margin:0;font-size:15px}
 svg{background:#161a22;border:1px solid #262c39;border-radius:10px;
     max-width:44vw;max-height:52vh;cursor:crosshair}
 .cand{fill:none;stroke:#e8720c;stroke-width:2;cursor:pointer}
 .cand:hover{stroke:#ffb066;stroke-width:3.5}
 .cand.sel{stroke:#33ccff;stroke-width:3.5}
 .rim{stroke:#e8720c;stroke-width:2.5;cursor:pointer}
 .rim:hover{stroke:#ffb066;stroke-width:4}
 .rim.sel{stroke:#33ccff;stroke-width:4}
 #cross{stroke:#33ccff;stroke-width:2}
 button{font-size:15px;font-weight:700;padding:9px 24px;border-radius:8px;
        border:0;background:#2f6ae0;color:#fff;cursor:pointer}
 button:disabled{opacity:.4}
 #msg{font-size:14px;color:#34c77b;min-height:20px;text-align:center}
</style></head><body>
<h2>도킹 메이트 (꽂히는 자리) 선택</h2>
<div class="hint">① 도킹(위에서 본 모습): 소켓 자리를 클릭 — 주황 원 후보를
클릭하거나 <b>아무 곳이나 자유 클릭</b> (파란 십자 = 선택 위치, 높이는 맨 윗면 자동)
&nbsp;·&nbsp; ② 펜(옆모습): 도킹 윗면과 같은 높이에 올 <b>원(주황 가로선)</b> 클릭</div>
<div id="panes">
 <div class="pane"><h3>① 도킹 — 위에서 본 모습 (X·Y)</h3>
   <svg id="svd" viewBox="__VBD__"></svg></div>
 <div class="pane"><h3>② 펜 __BRICK__ — 옆모습 (X·Z)</h3>
   <svg id="svp" viewBox="__VBP__"></svg></div>
</div>
<button id="save" disabled>이 메이트로 저장</button>
<div id="msg"></div>
<script>
const D=__DOCK__, P=__PEN__;
const sel={xy:null,p:null};
/* ---- 도킹 위에서 본 모습 ---- */
const svd=document.getElementById('svd');
let s='';
for(const q of D.cloud_xy)
  s+=`<circle cx="${q[0]}" cy="${-q[1]}" r="0.5" fill="#3a4152"/>`;
svd.innerHTML=s;
D.circles.forEach((c,i)=>{
  const el=document.createElementNS('http://www.w3.org/2000/svg','circle');
  el.setAttribute('cx',c.cx);el.setAttribute('cy',-c.cy);el.setAttribute('r',c.r);
  el.setAttribute('class','cand');
  el.addEventListener('click',ev=>{ev.stopPropagation();pickXY(c.cx,c.cy,'원 후보');});
  const t=document.createElementNS('http://www.w3.org/2000/svg','title');
  t.textContent=`중심(${c.cx},${c.cy}) r=${c.r}mm z ${c.zmin}~${c.zmax}`;
  el.appendChild(t);svd.appendChild(el);
});
svd.addEventListener('click',ev=>{
  const pt=svd.createSVGPoint();pt.x=ev.clientX;pt.y=ev.clientY;
  const p=pt.matrixTransform(svd.getScreenCTM().inverse());
  pickXY(Math.round(p.x*10)/10,Math.round(-p.y*10)/10,'자유 클릭');
});
function pickXY(x,y,how){
  sel.xy=[x,y];
  svd.querySelectorAll('#cross,#crossv').forEach(e=>e.remove());
  const mk=(id,x1,y1,x2,y2)=>{const l=document.createElementNS(
    'http://www.w3.org/2000/svg','line');
    l.id=id;l.setAttribute('x1',x1);l.setAttribute('y1',y1);
    l.setAttribute('x2',x2);l.setAttribute('y2',y2);
    l.setAttribute('class','');l.setAttribute('stroke','#33ccff');
    l.setAttribute('stroke-width','2');svd.appendChild(l);};
  mk('cross',x-6,-y,x+6,-y);mk('crossv',x,-y-6,x,-y+6);
  update(how);
}
/* ---- 펜 옆모습 ---- */
const svp=document.getElementById('svp');
let s2='';
for(const q of P.cloud_xz)
  s2+=`<circle cx="${q[0]}" cy="${-q[1]}" r="0.5" fill="#3a4152"/>`;
svp.innerHTML=s2;
const rims=[];
P.circles.forEach(c=>{rims.push({...c,z:c.zmin});
  if(c.zmax-c.zmin>0.5)rims.push({...c,z:c.zmax});});
rims.forEach((r,i)=>{
  const ln=document.createElementNS('http://www.w3.org/2000/svg','line');
  ln.setAttribute('x1',r.cx-r.r);ln.setAttribute('x2',r.cx+r.r);
  ln.setAttribute('y1',-r.z);ln.setAttribute('y2',-r.z);
  ln.setAttribute('class','rim');
  ln.addEventListener('click',()=>{
    svp.querySelectorAll('.rim').forEach(e=>e.classList.remove('sel'));
    ln.classList.add('sel');sel.p=r;update();
  });
  const t=document.createElementNS('http://www.w3.org/2000/svg','title');
  t.textContent=`원 r=${r.r}mm · z=${r.z}mm`;
  ln.appendChild(t);svp.appendChild(ln);
});
function update(how){
  const ok=sel.xy&&sel.p;
  document.getElementById('save').disabled=!ok;
  if(!ok){
    if(sel.xy)msg(`소켓 (${sel.xy[0]}, ${sel.xy[1]})mm 선택됨 (${how||''}) — 이제 펜의 원을 클릭`);
    return;
  }
  const seat=Math.round((sel.p.z-P.zb)*10)/10;
  msg(`소켓 (${sel.xy[0]}, ${sel.xy[1]}) · 펜 원 r=${sel.p.r} z=${sel.p.z}`
     +` → 펜 바닥이 도킹 윗면보다 ${seat}mm 아래로 꽂힘`);
}
function msg(t){document.getElementById('msg').textContent=t;}
document.getElementById('save').onclick=async()=>{
  const r=await fetch('/save',{method:'POST',
    body:JSON.stringify({xy:sel.xy,p:sel.p})});
  msg(await r.text());
  document.getElementById('save').disabled=true;
};
</script></body></html>"""

def main():
    dock, pen = find_meshes()
    if not dock or not pen:
        print(f'메쉬를 못 찾음: 도킹={dock} 펜={pen}'); sys.exit(1)
    print(f'도킹: {dock}\n펜  : {pen}')
    # 도킹 STL = mm 원시값, 브릭 STL = m 단위 (프로젝트 관례)
    d_c, d_xy, d_xz, d_zb, d_zt = detect_circles(dock, 1.0)
    p_c, p_xy, p_xz, p_zb, p_zt = detect_circles(pen, 1000.0)
    print(f'원 후보: 도킹 {len(d_c)}개 / 펜 {len(p_c)}개')
    for tag, cs in (('도킹', d_c), ('펜', p_c)):
        for c in cs:
            print(f"  [{tag}] 중심({c['cx']:7.1f},{c['cy']:7.1f}) "
                  f"r={c['r']:6.2f}  z {c['zmin']:6.1f}~{c['zmax']:6.1f}")

    def vb(cloud, pad=12):
        xs=[q[0] for q in cloud]; ys=[-q[1] for q in cloud]
        return (f"{min(xs)-pad:.0f} {min(ys)-pad:.0f} "
                f"{max(xs)-min(xs)+2*pad:.0f} {max(ys)-min(ys)+2*pad:.0f}")
    brick_name = os.path.basename(os.path.dirname(os.path.dirname(pen)))
    brick_name = brick_name.split('_')[-1]
    page = (PAGE
        .replace('__DOCK__', json.dumps({'circles': d_c, 'cloud_xy': d_xy,
                                         'zb': d_zb, 'zt': d_zt}))
        .replace('__PEN__',  json.dumps({'circles': p_c, 'cloud_xz': p_xz,
                                         'zb': p_zb, 'zt': p_zt}))
        .replace('__VBD__', vb(d_xy)).replace('__VBP__', vb(p_xz))
        .replace('__BRICK__', brick_name)).encode('utf-8')

    done = threading.Event()
    dock_base = os.path.splitext(os.path.basename(dock))[0].replace(' ', '_')

    class H(http.server.BaseHTTPRequestHandler):
        def log_message(self, *a): pass
        def _send(self, code, ctype, body):
            self.send_response(code)
            self.send_header('Content-Type', ctype + '; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers(); self.wfile.write(body)
        def do_GET(self):
            self._send(200, 'text/html', page)
        def do_POST(self):
            if self.path != '/save':
                self._send(404, 'text/plain', b'?'); return
            n = int(self.headers.get('Content-Length', 0))
            m = json.loads(self.rfile.read(n))
            xy, rp = m['xy'], m['p']
            seat = round(rp['z'] - p_zb, 2)     # 펜 바닥->기준원 = 꽂힘 깊이
            with open(OUT, 'w') as fo:
                fo.write(
                    '# pick_dock_mate.py 가 생성한 도킹 메이트 (하드코딩)\n'
                    '# 동심: 펜 중심축 = 도킹 소켓 축 (socket_xy_mm, 도킹 로컬)\n'
                    '# Z 일치: 펜 기준 원(seat_dz_mm)이 도킹 맨 윗면 높이\n'
                    f'# 도킹: {dock}\n# 펜  : {pen}\n'
                    'DOCK_MATE = ' + json.dumps({
                        'brick': brick_name,
                        'dock_file': dock_base,
                        'socket_xy_mm': [xy[0], xy[1]],
                        'seat_dz_mm': seat,
                        'pen_circle_r_mm': rp['r'],
                    }, ensure_ascii=False) + '\n')
            txt = (f"저장 완료 -> {OUT} · 소켓({xy[0]},{xy[1]})mm, "
                   f"꽂힘 깊이 {seat}mm · ARM STUDIO 재시작 시 적용")
            print(txt)
            self._send(200, 'text/plain', txt.encode('utf-8'))
            done.set()

    class TS(socketserver.ThreadingMixIn, http.server.HTTPServer):
        daemon_threads = True; allow_reuse_address = True

    for port in PORT_RANGE:
        try:
            srv = TS(('0.0.0.0', port), H); break
        except OSError:
            continue
    else:
        print('빈 포트 없음'); sys.exit(1)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print(f'\n브라우저에서 여세요:  http://localhost:{port}\n'
          '① 도킹 소켓 자리 클릭(원 후보 또는 자유 클릭) ② 펜 원 클릭 '
          '-> [저장]  (Ctrl+C = 취소)')
    try:
        done.wait(); import time; time.sleep(1.0)
    except KeyboardInterrupt:
        print('\n취소됨 (저장 안 함)')

if __name__ == '__main__':
    main()
