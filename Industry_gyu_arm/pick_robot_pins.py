#!/usr/bin/env python
"""로봇 기준 목심(핀) 선택 도구 — ARM STUDIO 본체와 완전 분리된 1회용 코드.

실행하면 브라우저에 로봇 바닥이 위에서 본 모습으로 뜨고,
목심 후보가 전부 주황색 원으로 표시된다.
그중 딱 2개를 클릭(파란색으로 변함)하고 [저장] 을 누르면
robot_pins_fixed.py 에 하드코딩되어 저장된다.
이후 ARM STUDIO 는 자동 감지 없이 무조건 이 값만 쓴다.

사용:
    ./pick_pins.sh                # 또는
    conda run -n gyu_arm python pick_robot_pins.py [base_link.STL 경로]

다시 고르고 싶으면 이 도구를 다시 실행해 저장하면 덮어쓴다.
"""
import glob, http.server, json, math, os, socketserver, sys, threading
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OUT  = os.path.join(HERE, 'robot_pins_fixed.py')
PORT_RANGE = range(7950, 7970)

# ---------- STL 찾기 (ARM STUDIO 와 똑같은 순서로 base URDF 를 찾는다) ----------
URDF_ROOT = '/home/drboom/2026_project/urdf'
ROBOT_FAMILY = os.environ.get('ARM_ROBOT_FAMILY', 'sts3215')
BASE_URDF_CFG = os.path.join(HERE, 'base_urdf.txt')

def _find_base_urdf():
    p = os.environ.get('ARM_BASE_URDF')
    if p and os.path.exists(p):
        return p
    if os.path.exists(BASE_URDF_CFG):
        p = open(BASE_URDF_CFG).read().strip()
        if p and os.path.exists(p):
            return p
    for pat in (os.path.join(URDF_ROOT, ROBOT_FAMILY, '*', 'urdf', '*.urdf'),
                os.path.join(URDF_ROOT, '*', 'urdf', '*.urdf')):
        cands = [u for u in glob.glob(pat) if not u.endswith('_limits.urdf')]
        if cands:
            break
    if not cands:
        return None
    base = [u for u in cands
            if 'base' in os.path.basename(os.path.dirname(
                   os.path.dirname(u))).lower()
            or 'base' in os.path.basename(u).lower()]
    return max(base or cands, key=os.path.getmtime)

def find_stl():
    if len(sys.argv) > 1:
        return sys.argv[1]
    u = _find_base_urdf()
    if not u:
        return None
    return os.path.join(os.path.dirname(u), '..', 'meshes', 'base_link.STL')

# ---------- 바닥 목심 후보 감지 (arm_studio_web.py 와 같은 방식, 단 전부 표시) ----------
def detect(path):
    import trimesh
    from scipy.cluster.hierarchy import fclusterdata
    m = trimesh.load(path)
    v = np.asarray(m.vertices) * 1000.0          # mm
    n = m.face_normals
    f = np.asarray(m.faces)
    zmin = v[:, 2].min()
    # 옆면(법선이 수평에 가까운 면)의 꼭짓점 중 바닥에서 4mm 이내 = 핀 옆면
    w = v[np.unique(f[np.abs(n[:, 2]) < 0.25])]
    w = w[np.abs(w[:, 2] - zmin) < 4]
    lab = fclusterdata(w[:, :2], t=3.0, criterion='distance')
    pins = []
    for k in set(lab):
        g = w[lab == k]
        if len(g) < 6:
            continue
        c = g[:, :2].mean(0)
        r = np.linalg.norm(g[:, :2] - c, axis=1)
        med = float(np.median(r))
        if 3.0 < med < 7.0:                      # 지름 6~14mm 원기둥 = 후보
            pins.append({'x': round(float(c[0]), 2),
                         'y': round(float(c[1]), 2),
                         'r': round(med, 2)})
    # 바닥면 점(외곽 표시용) - 너무 많으면 샘플링
    base = v[np.abs(v[:, 2] - zmin) < 1.0][:, :2]
    if len(base) > 5000:
        base = base[np.random.RandomState(0).choice(len(base), 5000,
                                                    replace=False)]
    return pins, base.round(2).tolist()

# ---------- 페이지 ----------
PAGE = """<!doctype html><html lang="ko"><head><meta charset="utf-8">
<title>로봇 목심 선택</title>
<style>
 body{margin:0;background:#0e1116;color:#e7ebf3;font-family:sans-serif;
      display:flex;flex-direction:column;align-items:center;gap:12px;padding:16px}
 h2{margin:4px}
 .hint{color:#8b93a7;font-size:14px}
 svg{background:#161a22;border:1px solid #262c39;border-radius:12px;
     max-width:92vw;max-height:74vh;cursor:crosshair}
 .pin{fill:#e8720c;cursor:pointer}
 .pin:hover{fill:#ffb066}
 .pin.sel{fill:#33ccff}
 button{font-size:16px;font-weight:700;padding:10px 26px;border-radius:8px;
        border:0;background:#2f6ae0;color:#fff;cursor:pointer}
 button:disabled{opacity:.4}
 #msg{font-size:15px;color:#34c77b;min-height:22px}
</style></head><body>
<h2>로봇 바닥 목심 선택</h2>
<div class="hint">주황 원 = 감지된 목심 후보 __N__개 · 대칭인 목심 <b>2개</b>를
클릭하고 저장을 누르세요 (다시 클릭하면 해제)</div>
<svg id="sv" viewBox="__VB__"></svg>
<button id="save" disabled>이 2개를 기준 목심으로 저장</button>
<div id="msg"></div>
<script>
const PINS=__PINS__, BASE=__BASE__;
const sv=document.getElementById('sv'), sel=new Set();
// 바닥 점 (회색)
let s='';
for(const p of BASE) s+=`<circle cx="${p[0]}" cy="${-p[1]}" r="0.35" fill="#3a4152"/>`;
sv.innerHTML=s;
// 핀 후보 (주황) - y 는 화면상 뒤집힘 주의
PINS.forEach((p,i)=>{
  const c=document.createElementNS('http://www.w3.org/2000/svg','circle');
  c.setAttribute('cx',p.x); c.setAttribute('cy',-p.y); c.setAttribute('r',p.r);
  c.setAttribute('class','pin'); c.dataset.i=i;
  c.addEventListener('click',()=>{
    if(sel.has(i)){sel.delete(i);c.classList.remove('sel');}
    else{
      if(sel.size>=2){msg('이미 2개 선택됨 - 하나를 해제하세요',1);return;}
      sel.add(i);c.classList.add('sel');
    }
    document.getElementById('save').disabled=(sel.size!==2);
    if(sel.size===2){
      const [a,b]=[...sel].map(k=>PINS[k]);
      const d=Math.hypot(a.x-b.x,a.y-b.y);
      msg(`선택: (${a.x},${a.y}) & (${b.x},${b.y}) - 간격 ${d.toFixed(1)}mm`);
    }
  });
  const t=document.createElementNS('http://www.w3.org/2000/svg','title');
  t.textContent=`(${p.x}, ${p.y}) r=${p.r}mm`; c.appendChild(t);
  sv.appendChild(c);
});
function msg(t,warn){const m=document.getElementById('msg');
  m.textContent=t; m.style.color=warn?'#f5a623':'#34c77b';}
document.getElementById('save').onclick=async()=>{
  const pts=[...sel].map(k=>[PINS[k].x,PINS[k].y]);
  const r=await fetch('/save',{method:'POST',body:JSON.stringify(pts)});
  msg(await r.text());
  document.getElementById('save').disabled=true;
};
</script></body></html>"""

def main():
    stl = find_stl()
    if not stl or not os.path.exists(stl):
        print('base_link.STL 을 찾지 못했습니다. 경로를 인자로 주세요.')
        sys.exit(1)
    print(f'메쉬: {stl}')
    pins, base = detect(stl)
    if not pins:
        print('목심 후보를 하나도 찾지 못했습니다.')
        sys.exit(1)
    print(f'목심 후보 {len(pins)}개 감지:')
    for p in pins:
        print(f"  ({p['x']:8.2f}, {p['y']:8.2f})  반지름 {p['r']:.2f}mm")

    xs = [p[0] for p in base]; ys = [-p[1] for p in base]
    pad = 15
    vb = (f"{min(xs)-pad:.0f} {min(ys)-pad:.0f} "
          f"{max(xs)-min(xs)+2*pad:.0f} {max(ys)-min(ys)+2*pad:.0f}")
    page = (PAGE.replace('__PINS__', json.dumps(pins))
                .replace('__BASE__', json.dumps(base))
                .replace('__VB__', vb)
                .replace('__N__', str(len(pins)))).encode('utf-8')

    done = threading.Event()

    class H(http.server.BaseHTTPRequestHandler):
        def log_message(self, *a): pass
        def _send(self, code, ctype, body):
            self.send_response(code)
            self.send_header('Content-Type', ctype + '; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        def do_GET(self):
            self._send(200, 'text/html', page)
        def do_POST(self):
            if self.path != '/save':
                self._send(404, 'text/plain', b'?'); return
            n = int(self.headers.get('Content-Length', 0))
            pts = json.loads(self.rfile.read(n))
            with open(OUT, 'w') as f:
                f.write('# pick_robot_pins.py 가 생성한 로봇 기준 목심 2점\n'
                        '# (base_link 로컬 좌표, mm) - 이 파일이 있으면\n'
                        '# arm_studio_web.py 는 자동 감지 대신 이 값만 쓴다.\n'
                        f'# 원본 메쉬: {stl}\n'
                        f'ROBOT_PINS_MM = {json.dumps(pts)}\n')
            d = math.hypot(pts[0][0]-pts[1][0], pts[0][1]-pts[1][1])
            txt = (f'저장 완료 -> {OUT}  (간격 {d:.1f}mm) '
                   f'· ARM STUDIO 를 재시작하면 적용됩니다. 이 창은 닫아도 됩니다.')
            print(txt)
            self._send(200, 'text/plain', txt.encode('utf-8'))
            done.set()

    class TS(socketserver.ThreadingMixIn, http.server.HTTPServer):
        daemon_threads = True; allow_reuse_address = True

    for port in PORT_RANGE:
        try:
            srv = TS(('0.0.0.0', port), H)
            break
        except OSError:
            continue
    else:
        print('빈 포트가 없습니다'); sys.exit(1)

    threading.Thread(target=srv.serve_forever, daemon=True).start()
    print(f'\n브라우저에서 여세요:  http://localhost:{port}\n'
          '2개 클릭 -> [저장] 하면 자동으로 끝납니다. (Ctrl+C = 취소)')
    try:
        done.wait()
        import time; time.sleep(1.0)         # 응답 전송 여유
    except KeyboardInterrupt:
        print('\n취소됨 (저장 안 함)')

if __name__ == '__main__':
    main()
