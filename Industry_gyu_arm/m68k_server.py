"""M68K 서버 — 68000 어레이 3D 뷰어의 웹 API.

**이 파일은 start_m68k.sh 전용이다.**
start_pro.sh 는 이 파일을 읽지도, 부르지도 않는다.
여기를 아무리 뜯어고쳐도 원래 앱은 영향을 받지 않는다.

붙이는 법:
    import m68k_server
    m68k_server.attach(app, core, JOB, start_job, STREAMS)
"""
import os, json, struct, threading, subprocess
import urllib.request
from fastapi import Body
from fastapi.staticfiles import StaticFiles
from fastapi.responses import JSONResponse, Response

HERE = os.path.dirname(os.path.abspath(__file__))

# attach() 가 채운다 — 붙이기 전에는 전부 None 이다
app = core = JOB = start_job = STREAMS = None


# M68K-GPU 렌더러 뷰어 (68000 어레이 3D 시뮬레이션)
M68K_DIR = os.path.join(HERE, 'm68k_array', 'gfx')

# ── 68000 렌더러를 상주시켜 마우스 조작에 응답한다 ──────────────
#   PL-68000 은 매 프레임 입력을 읽어 시점을 바꾸고 다시 그린다.
#   여기서는 그 입력을 브라우저 마우스로 받는다. 렌더러 프로세스를
#   띄워 두고 (메쉬 재로딩 없이) 한 줄 쓰면 한 프레임이 돌아온다.
_M68K_PROC = [None]
_M68K_LOCK = threading.Lock()

def _m68k_proc():
    p = _M68K_PROC[0]
    if p is not None and p.poll() is None:
        return p
    exe = os.path.join(M68K_DIR, 'pl68k')
    # 공장 전체(scene.bin)가 있으면 그걸 쓰고, 없으면 로봇만(robot_lo.bin)
    mesh = os.path.join(M68K_DIR, 'scene.bin')
    if not os.path.exists(mesh):
        mesh = os.path.join(M68K_DIR, 'robot_lo.bin')
    if not (os.path.exists(exe) and os.path.exists(mesh)):
        return None
    p = subprocess.Popen([exe, mesh, '--serve'],
                         cwd=M68K_DIR,
                         stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                         stderr=subprocess.PIPE)
    # 렌더러가 메쉬를 다 읽으면 stderr 로 READY 를 보낸다.
    # 그 앞에 메쉬 정보 로그가 몇 줄 나오므로 READY 가 나올 때까지 읽는다.
    for _ in range(40):
        line = p.stderr.readline()
        if not line:
            p.kill(); return None
        if line.strip() == b'READY':
            break
    else:
        p.kill(); return None
    _M68K_PROC[0] = p
    return p

# ROUTE GET /api/m68k/scene
def m68k_scene():
    """뷰어가 팔레트를 만들 때 쓰는 조각 목록 (이름·색·종류)."""
    p = os.path.join(M68K_DIR, 'scene.json')
    if not os.path.exists(p):
        return JSONResponse({'parts': [], 'tri': 0})
    with open(p) as f:
        return JSONResponse(json.load(f))


# ROUTE POST /api/m68k/rebuild
def m68k_rebuild(tri: int = 3000):
    """지금 공장 배치를 다시 구워서 68000 이 읽을 장면으로 만든다.

    로봇 자세·설비 위치는 앱이 이미 푼 값을 그대로 받아쓴다
    (기구학을 여기서 다시 풀지 않는다 — 화면과 어긋나면 안 되니까)."""
    global _M68K_PROC
    script = os.path.join(M68K_DIR, 'build_scene.py')
    if not os.path.exists(script):
        return JSONResponse({'ok': False, 'msg': 'build_scene.py 없음'}, 500)
    try:
        r = subprocess.run([sys.executable, script, str(int(tri))],
                           cwd=M68K_DIR, capture_output=True, timeout=180)
        out = (r.stdout or b'').decode('utf-8', 'replace')
        if r.returncode != 0:
            err = (r.stderr or b'').decode('utf-8', 'replace')
            return JSONResponse({'ok': False, 'msg': (err or out)[-800:]}, 500)
    except Exception as e:
        return JSONResponse({'ok': False, 'msg': str(e)}, 500)
    # 새 장면을 읽히려면 렌더러를 다시 띄운다
    with _M68K_LOCK:
        p = _M68K_PROC[0]
        if p is not None and p.poll() is None:
            try: p.kill()
            except Exception: pass
        _M68K_PROC[0] = None
    return JSONResponse({'ok': True, 'log': out[-1500:]})


def _m68k_push_pose():
    """앱이 계산해 둔 지금 로봇 자세를 렌더러에 밀어 넣는다.

    장면을 다시 굽지 않는다 — 메쉬는 그대로 두고 링크마다 행렬 하나씩만
    보낸다. 그래서 로봇이 움직여도 한 프레임이면 따라잡는다.
    (다시 굽기는 1~2초 걸려서 블록 실행을 못 쫓아간다)

    scene.bin 은 build_scene.py 가 '그때 자세' 를 이미 구워 넣었으므로,
    지금 자세로 옮기려면 (지금자세) x (구울때자세)^-1 을 곱해야 한다."""
    p = _M68K_PROC[0]
    if p is None or p.poll() is not None:
        return False
    meta_p = os.path.join(M68K_DIR, 'scene.json')
    if not os.path.exists(meta_p):
        return False
    try:
        with open(meta_p) as f:
            meta = json.load(f)
        baked = meta.get('link_m') or {}
        if not baked:
            return False
        import urllib.request
        with urllib.request.urlopen('http://127.0.0.1:7870/state', timeout=5) as r:
            st = json.loads(r.read().decode())
        # 조각 순서 = scene.json 의 parts 순서
        idx = {pt['id']: i for i, pt in enumerate(meta.get('parts', []))}
        lines = []
        for lk in st.get('links', []):
            i = idx.get(lk['id'])
            if i is None or lk['id'] not in baked:
                continue
            # scene.bin 의 정점은 '구울 때 자세' 로 이미 옮겨져 있다.
            # 지금 자세로 보내려면 구울 때 자세를 먼저 되돌리고(역행렬)
            # 지금 자세를 적용한다:  D = 지금 x 구울때^-1
            # 이 D 는 z-up 좌표계 기준이라, 메쉬가 y-up 으로 저장돼 있으므로
            # 같은 좌표계로 옮겨서 곱해야 한다:  Zup x D x Zup^-1
            cur = _m4(lk['m'])
            old_m = _m4(baked[lk['id']])
            d = _m4mul(cur, _m4inv(old_m))
            d = _m4mul(_ZUP, _m4mul(d, _ZUPI))
            row = []
            for r_ in range(3):
                # 회전 3개는 그대로, 이동은 m -> 0.1mm
                row += [d[r_][0], d[r_][1], d[r_][2], d[r_][3] * 10000.0]
            lines.append('M %d ' % i + ' '.join('%.6f' % v for v in row))
        if not lines:
            return False
        p.stdin.write(('\n'.join(lines) + '\n').encode())
        p.stdin.flush()
        return True
    except Exception:
        return False


def _m4(v):
    v = list(v)
    if len(v) == 12: v = v + [0, 0, 0, 1]
    return [v[0:4], v[4:8], v[8:12], v[12:16]]

def _m4mul(a, b):
    return [[sum(a[i][k]*b[k][j] for k in range(4)) for j in range(4)]
            for i in range(4)]

def _m4inv(m):
    """회전+평행이동만 있는 행렬의 역 (회전은 전치, 이동은 -R^T t)."""
    r = [[m[j][i] for j in range(3)] for i in range(3)]
    t = [-sum(r[i][k]*m[k][3] for k in range(3)) for i in range(3)]
    return [r[0]+[t[0]], r[1]+[t[1]], r[2]+[t[2]], [0, 0, 0, 1]]

# z-up(로봇) -> y-up(렌더러):  (x,y,z) -> (x, z, -y)
_ZUP  = [[1,0,0,0],[0,0,1,0],[0,-1,0,0],[0,0,0,1]]
_ZUPI = [[1,0,0,0],[0,0,-1,0],[0,1,0,0],[0,0,0,1]]


# ROUTE POST /api/m68k/sim
def m68k_sim():
    """이 창에서 바로 시뮬레이션(3D 미리보기)을 돌린다.

    앱 화면의 [3D 미리보기] 버튼과 같은 것을 부른다 —
    계산은 전부 앱이 하고, 여기서는 시작만 시킨다.
    FORCE_SIM 이 켜지므로 실물 로봇으로는 절대 안 나간다."""
    if JOB['busy']:
        return JSONResponse({'ok': False, 'msg': f"다른 작업({JOB['fn']}) 진행 중"})
    # 앱 UI 가 넘기는 것과 같은 인자다 (pro_ui/app.js 의 runArgs)
    modes = []
    try:
        modes = list(core.MODES) if hasattr(core, 'MODES') else []
    except Exception:
        pass
    mode = modes[0] if modes else '관절 순서대로 (수평유지)'
    return start_job('preview', STREAMS['preview'],
                     ['간편 이동', mode, 'x-z-y', '1.0x', False, False],
                     stream=True)


# ROUTE GET /api/m68k/simstatus
def m68k_simstatus():
    """시뮬레이션 진행 상황 (이 창의 표시용)."""
    return {'busy': bool(JOB['busy']), 'status': JOB.get('status') or '',
            'prog': core.PROG_STATE['text'], 'error': JOB.get('error')}


# ROUTE POST /api/m68k/say
def m68k_say(body: dict = Body(...)):
    """채팅으로 들어온 명령을 블록으로 바꾼다.

    앱의 [말로 만들기] 와 같은 것을 부른다 — 해석은 Spark LLM(Gemma)이
    하고, 못 붙으면 규칙 파서로 넘어간다. 여기서는 전달만 한다."""
    t = str(body.get('text') or '').strip()
    if not t:
        return JSONResponse({'ok': False, 'msg': '빈 명령입니다'})
    try:
        out = core.h_say(t)
    except Exception as e:
        return JSONResponse({'ok': False, 'msg': f'{type(e).__name__}: {e}'})
    r, msg = (out if isinstance(out, tuple) else (out, ''))
    r = r if isinstance(r, dict) else {}
    return {'ok': True, 'msg': str(msg or ''),
            'added': int(r.get('added') or 0),
            'preview': bool(r.get('preview')),
            'blocks': _m68k_blocks()}


def _m68k_blocks():
    """지금 블록(파이프라인) 목록 — 이 창에 보여주려고.

    S['seq'] 는 문자열 리스트다. 사람이 읽는 라벨은 seq_labels() 가
    따로 만든다 (앱 화면이 쓰는 것과 같은 것)."""
    try:
        out = []
        for lab in core.seq_labels():
            kind, _idx = core.seq_index(lab)
            out.append({'label': str(lab), 'kind': str(kind or '')})
        return out
    except Exception:
        return []


# ROUTE GET /api/m68k/blocks
def m68k_blocks():
    return {'blocks': _m68k_blocks()}


# ROUTE POST /api/m68k/blocks_clear
def m68k_blocks_clear():
    """블록 전부 지우기."""
    try:
        core._undo_snapshot('68000 창: 블록 비우기')
        with core.LOCK:
            core.S['seq'] = []
        core._macro_save()
    except Exception as e:
        return JSONResponse({'ok': False, 'msg': str(e)})
    return {'ok': True, 'blocks': _m68k_blocks()}


# ROUTE GET /api/m68k/bench
def m68k_bench():
    """구성마다 실제로 몇 사이클 드는지 재본다.

    노드 수를 바꿔가며 진짜로 그려보고 '가장 느린 노드' 를 받아온다.
    전체 사이클을 개수로 나눈 산수가 아니다 — 화면 띠마다 일이 몰리는
    정도가 달라서, 32개를 써도 32배 빨라지지 않는다."""
    out = []
    with _M68K_LOCK:
        p = _m68k_proc()
        if p is None:
            return JSONResponse({'rows': []})
        for n in (1, 4, 8, 16, 32, 64):
            try:
                p.stdin.write(f'N {n}\n28 -18 160 0\n'.encode())
                p.stdin.flush()
                hdr = p.stdout.read(13)
                if len(hdr) != 13 or hdr[0:1] != b'F':
                    _M68K_PROC[0] = None
                    break
                w, h = struct.unpack('<HH', hdr[1:5])
                cyc = struct.unpack('<q', hdr[5:13])[0]
                need = w * h
                got = 0
                while got < need:
                    chunk = p.stdout.read(need - got)
                    if not chunk:
                        break
                    got += len(chunk)
                out.append({'nodes': n, 'cycles': cyc})
            except Exception:
                _M68K_PROC[0] = None
                break
    return {'rows': out}


# ROUTE GET /api/m68k/frame
def m68k_frame(yaw: float = 28, pitch: float = -18, zoom: float = 160,
               phase: float = 0.0, live: int = 0, nodes: int = 32):
    """시점 하나를 68000 렌더러로 그려서 돌려준다.

    응답 본문 = 320x240 팔레트 바이트 (그대로 canvas 에 올린다).
    실제 68000 이 이 프레임에 쓴 사이클 수는 헤더로 보낸다 —
    브라우저가 그걸로 '8MHz 32개면 몇 fps' 를 계산한다."""
    with _M68K_LOCK:
        p = _m68k_proc()
        if p is None:
            return Response(content=b'', status_code=503)
        try:
            # 어레이 구성(노드 수)을 알려준다. 렌더러는 화면을 그만큼
            # 가로띠로 나누고, 삼각형도 노드끼리 나눠 맡는다.
            n = max(1, min(64, int(nodes)))
            p.stdin.write(f'N {n}\n'.encode())
            if live:
                _m68k_push_pose()   # 지금 로봇 자세를 먼저 밀어 넣는다
            p.stdin.write(f'{yaw} {pitch} {zoom} {phase}\n'.encode())
            p.stdin.flush()
            hdr = p.stdout.read(13)
            if len(hdr) != 13 or hdr[0:1] != b'F':
                _M68K_PROC[0] = None
                return Response(content=b'', status_code=503)
            w, h = struct.unpack('<HH', hdr[1:5])
            cycles = struct.unpack('<q', hdr[5:13])[0]
            need = w * h
            buf = b''
            while len(buf) < need:
                chunk = p.stdout.read(need - len(buf))
                if not chunk:
                    _M68K_PROC[0] = None
                    return Response(content=b'', status_code=503)
                buf += chunk
        except Exception:
            _M68K_PROC[0] = None
            return Response(content=b'', status_code=503)
    return Response(content=buf, media_type='application/octet-stream',
                    headers={'X-Cycles': str(cycles),
                             'X-W': str(w), 'X-H': str(h),
                             'Cache-Control': 'no-store'})

def attach(fastapi_app, core_module, job_dict, start_job_fn, streams):
    """앱에 68000 API 를 붙인다. start_m68k.sh 로 띄울 때만 부른다.

    start_pro.sh 로 띄우면 이 함수가 아예 안 불리므로,
    /api/m68k/* 도 /m68k 도 존재하지 않는다."""
    global app, core, JOB, start_job, STREAMS
    app, core, JOB, start_job, STREAMS = (
        fastapi_app, core_module, job_dict, start_job_fn, streams)

    if os.path.isdir(M68K_DIR):
        app.mount('/m68k', StaticFiles(directory=M68K_DIR), name='m68k')

    # 엔드포인트 등록 (데코레이터 대신 명시적으로 — 붙일 때만 생기게)
    app.add_api_route('/api/m68k/scene',      m68k_scene,      methods=['GET'])
    app.add_api_route('/api/m68k/rebuild',    m68k_rebuild,    methods=['POST'])
    app.add_api_route('/api/m68k/sim',        m68k_sim,        methods=['POST'])
    app.add_api_route('/api/m68k/simstatus',  m68k_simstatus,  methods=['GET'])
    app.add_api_route('/api/m68k/say',        m68k_say,        methods=['POST'])
    app.add_api_route('/api/m68k/blocks',     m68k_blocks,     methods=['GET'])
    app.add_api_route('/api/m68k/blocks_clear', m68k_blocks_clear, methods=['POST'])
    app.add_api_route('/api/m68k/bench',      m68k_bench,      methods=['GET'])
    app.add_api_route('/api/m68k/frame',      m68k_frame,      methods=['GET'])
    print('[M68K] 68000 어레이 3D 뷰어 활성 -> /m68k/viewer.html')
    return True
