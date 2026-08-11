#!/usr/bin/env python
"""ARM STUDIO PRO — 새 디자인 UI 백엔드.

기존 arm_studio_web.py 를 '라이브러리'로 import 해서 로봇 제어/IK/상태 로직을
전부 재사용한다 (기존 파일은 수정하지 않음). 화면은 pro_ui/ 의 순수
HTML/CSS/JS 가 담당하고, 이 서버는 REST API 만 제공한다.

실행:  ./start_pro.sh   (또는 conda run -n gyu_arm python arm_studio_pro.py)
"""
import os, sys, math, json, shutil, tempfile, threading, traceback

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import arm_studio_web as core          # __main__ 가드 덕분에 Gradio 는 안 뜬다

from fastapi import FastAPI, Body, File, Form, UploadFile
from fastapi.responses import FileResponse, HTMLResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
import uvicorn

UI_DIR = os.path.join(HERE, 'pro_ui')
app = FastAPI(title='ARM STUDIO PRO')

# ================= 직렬화 =================
def norm(v):
    """핸들러 반환값 → JSON. gr.update dict 는 kwargs 만 남기고,
    plotly Figure 등 직렬화 불가 객체는 None (3D 는 뷰어 서버가 담당)."""
    if v is None or isinstance(v, (str, int, float, bool)):
        return v
    if isinstance(v, dict):
        if v.get('__type__'):
            return {'__update__': {k: norm(x) for k, x in v.items()
                                   if k != '__type__'}}
        return {k: norm(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [norm(x) for x in v]
    return None

def status_of(out):
    """핸들러 반환 튜플에서 사람이 읽을 상태 문자열을 뽑는다 (보통 마지막)."""
    if isinstance(out, str):
        return out
    if isinstance(out, (list, tuple)):
        for x in reversed(out):
            if isinstance(x, str):
                return x
    return ''

# ================= 목심(peg) 선택 상태 캐시 =================
# h_peg_view/h_peg_height 가 gr.update(choices=...) 로만 목록을 주므로 여기 보관.
PEG = {'on': False, 'h': 80, 'sel': None, 'choices': []}

def a_peg_view(on, h):
    PEG['on'] = bool(on); PEG['h'] = float(h)
    out = core.h_peg_view(bool(on), float(h))
    upd = out[0] if isinstance(out, (list, tuple)) and out else None
    if isinstance(upd, dict) and 'choices' in upd:
        PEG['choices'] = list(upd['choices'])
        PEG['sel'] = upd.get('value')
    if not on:
        PEG['choices'] = []; PEG['sel'] = None
    return status_of(out)

def a_peg_height(h):
    PEG['h'] = float(h)
    out = core.h_peg_height(float(h))
    upd = out[0] if isinstance(out, (list, tuple)) and out else None
    if isinstance(upd, dict) and 'choices' in upd:
        PEG['choices'] = list(upd['choices'])
        PEG['sel'] = upd.get('value')
    return status_of(out)

def a_peg_pick(name):
    PEG['sel'] = name or None
    return core.h_peg_pick(name)

def a_select_pose(name):
    core.FT['pose'] = name                 # 미세조정 대상 = 선택한 자세
    core.h_select(name)                    # 3D 미리보기 publish
    return f'자세 선택: {name}'

# ================= 작업(job) — 로봇을 움직이는 긴 호출 =================
JOB = {'busy': False, 'fn': None, 'status': '', 'result': None,
       'error': None, 'seq': 0}
JLOCK = threading.Lock()

def start_job(name, fn, args, stream=False):
    with JLOCK:
        if JOB['busy']:
            return {'ok': False, 'error': f"다른 작업({JOB['fn']}) 진행 중"}
        JOB.update(busy=True, fn=name, status='시작...', result=None, error=None)
        JOB['seq'] += 1
        jid = JOB['seq']
    def work():
        try:
            if stream:                     # h_preview / h_run 은 제너레이터
                for out in fn(*args):
                    s = status_of(out)
                    if s:
                        JOB['status'] = s
                JOB['result'] = JOB['status']
            else:
                out = fn(*args)
                JOB['status'] = status_of(out) or '완료'
                JOB['result'] = norm(out)
        except Exception as e:
            JOB['error'] = f'{type(e).__name__}: {e}'
            traceback.print_exc()
        finally:
            JOB['busy'] = False
    threading.Thread(target=work, daemon=True).start()
    return {'ok': True, 'job': jid}

# ================= 호출 레지스트리 =================
SYNC = {
    'select_pose':   a_select_pose,
    'set_sliders':   core.h_slider,            # (name, d1..d5)
    'pose_grip':     core.h_grip,              # (name, open_bool)
    'cal_robot':     core.h_cal_robot,         # (라벨)
    'cal_torque_off': core.h_cal_torque_off,
    'ft_step':       core.h_ft_step,           # (pose, 'J3', '1', ±1)
    'ft_grip':       core.h_ft_grip,           # (pose, open_bool)
    'ft_apply':      core.h_apply_nearest,
    'ft_reset':      core.h_ft_reset,
    'ft_save':       core.h_ft_save,
    'dup':           core.h_duplicate,         # (name, newname)
    'del':           core.h_delete,
    'save':          core.h_save,
    'load':          core.h_load,
    'seq_add':       core.h_seq_add,
    'seq_grip':      core.h_seq_grip,          # (open_bool)
    'seq_del':       core.h_seq_del,           # (label)
    'seq_move':      core.h_seq_move,          # (label, ±1)
    'seq_grasp':     core.h_seq_grasp,         # (brick)
    'seq_release':   core.h_seq_release,       # (dock)
    'seg_mode':      core.h_seg_mode,
    'seg_ax':        core.h_seg_axorder,
    'seg_rot':       core.h_seg_rotmode,
    'seg_prefer':    core.h_seg_prefer,
    'ign_save':      core.h_ign_save,          # (label, [ids])
    'exec_robot':    core.h_exec_robot,
    'robot_status':  core.h_robot_status,
    'stop':          core.h_stop,
    'torque_off':    core.h_torque_off,
    'macro_rec':     core.h_macro_rec,
    'macro_clear':   core.h_macro_clear,
    'simple_set':    core.h_simple_set,        # (prefer라벨, clear, extra)
    'map_xy':        core.h_map_xy,            # (label, "fx,fy")
    'map_undo':      core.h_map_undo,
    'map_clear':     core.h_map_clear,
    'path_save':     core.h_path_save,
    'path_del':      core.h_path_del,
    'env':           core.h_env,               # ([ids])
    'plate_edit':    core.h_plate_edit,
    'plate_remove':  core.h_plate_remove,
    'plate_refresh': core.h_plate_refresh,
    'robot_move_mode': core.h_robot_move_mode,
    'robot_home':    core.h_robot_home,
    'peg_pick':      a_peg_pick,
    'set_name':      core.h_set_name,          # (oid, name, color)
    'place':         lambda b, d: core.place_brick_on_docking(b, d)[1],
}
JOBS = {
    'ft_goto':       core.h_ft_goto,
    'cal_capture':   core.h_cal_capture_zero,
    'peg_view':      a_peg_view,
    'peg_height':    a_peg_height,
    'peg_goto':      core.h_peg_goto,
    'peg_save_pose': core.h_peg_save_pose,
    'grasp':         core.do_grasp,
    'release':       core.do_release,
    'macro_go':      core.h_macro_go,
    'rec_toggle':    core.h_rec_toggle,
    'add_path':      core.h_add_path,
}
STREAMS = {
    'preview': core.h_preview,   # (kind, mode, axorder, speed, loop, coll)
    'run':     core.h_run,
}

@app.post('/api/call')
def api_call(body: dict = Body(...)):
    fn = body.get('fn'); args = body.get('args') or []
    try:
        if fn in SYNC:
            out = SYNC[fn](*args)
            return {'ok': True, 'status': status_of(out), 'outputs': norm(out)}
        if fn in JOBS:
            return start_job(fn, JOBS[fn], args, stream=False)
        if fn in STREAMS:
            return start_job(fn, STREAMS[fn], args, stream=True)
        return JSONResponse({'ok': False, 'error': f'unknown fn: {fn}'}, 400)
    except Exception as e:
        traceback.print_exc()
        return JSONResponse({'ok': False,
                             'error': f'{type(e).__name__}: {e}'}, 500)

@app.get('/api/job')
def api_job():
    return {'busy': JOB['busy'], 'fn': JOB['fn'], 'status': JOB['status'],
            'prog': core.PROG_STATE['text'], 'error': JOB['error'],
            'result': JOB['result'] if not JOB['busy'] else None}

# ================= 상태 스냅샷 =================
@app.get('/api/meta')
def api_meta():
    joints = []
    for j in range(5):
        lo, hi = core.jr_limits(j)
        joints.append({'label': f'j{j+1}',
                       'min': round(math.degrees(lo), 1),
                       'max': round(math.degrees(hi), 1)})
    return {
        'joints': joints,
        'modes': core.MODES, 'mode_simple': core.MODE_SIMPLE,
        'axorders': list(core.AXIS_ORDERS), 'rots': list(core.ROT_MODES),
        'macro_slots': core.MACRO_SLOTS,
        'exec_labels': list(core._EXEC_LABELS),
        'exec_map': core._EXEC_LABELS,
        'cal_labels': list(core._CAL_KEYS),
        'cal_map': core._CAL_KEYS,
        'speeds': ['0.5x', '1.0x', '1.5x', '2.0x', '3.0x', '4.0x'],
        'steps': ['0.5', '1', '2', '5'],
        'viewer_port': core.VIEWER_PORT,
        'rviz_ok': bool(getattr(core, 'RVIZ_WEB_OK', False)), 'rviz_port': 6080,
        'peg_h_max': int(getattr(core, 'PEG_H_MAX', 250)),
        'project_file': core.PROJECT_FILE,
    }

@app.get('/api/state')
def api_state(sel: str = '', pose: str = ''):
    S = core.S
    poses = []
    for p in S['poses']:
        ang = p.get('angles', [0.0]*6)
        poses.append({'name': p['name'],
                      'deg': [round(math.degrees(a), 1) for a in ang[:5]]})
    seq = []
    for lab in core.seq_labels():
        kind, idx = core.seq_index(lab)
        seq.append({'label': lab, 'kind': kind or 'brick'})
    seg = None; ign_sel = None
    if sel:
        kind, i = core.seq_index(sel)
        if kind == 'move':
            a, b = core._sel_seg(sel)
            if a and b:
                k = core.seg_key(a, b)
                seg = {'a': a, 'b': b,
                       'mode': S.get('modes', {}).get(k, ''),
                       'ax': S.get('axorders', {}).get(k, ''),
                       'rot': core.ROT_LABEL.get(
                           S.get('rotmodes', {}).get(k, 'blend'), '동시(기본)'),
                       'prefer': S.get('simple_seg', {}).get(k, {}).get('prefer', ''),
                       'path_pts': len(S.get('paths2d', {}).get(k, [])),
                       'rec': len(S.get('moves', {}).get(k, []))}
        elif kind == 'pose' and i is not None and i < len(S['seq']):
            ign_sel = S.setdefault('ignore_obs', {}).get(S['seq'][i], [])
    ft_off = ''
    if pose:
        try:
            ft_off = core._off_str(core.get_offset(pose))
        except Exception:
            ft_off = ''
    return {
        'poses': poses, 'seq': seq, 'seg': seg, 'ign_sel': ign_sel,
        'ft_off': ft_off,
        'macros': [core._macro_btn_label(s) for s in core.MACRO_SLOTS],
        'env': core.env_ids(), 'env_on': core.env_visible_ids(),
        'obstacles': core.obstacle_ids(),
        'plates': core.added_tile_labels(), 'plate_info': core.h_plate_list(),
        'robot_pos': core._robot_pos_str(),
        'bricks': list(core.BRICKS), 'docking': core.docking_ids(),
        'names': S.get('names', {}),
        'exec': core.EXEC['mode'], 'cal': core.CAL['robot'],
        'cal_joints': [f'J{i+1}'
                       for i in range(core.robot_njoints(core.CAL['robot']))],
        'running': core.RUNNING.is_set(),
        'prog': core.PROG_STATE['text'],
        'busy': JOB['busy'], 'job_fn': JOB['fn'] if JOB['busy'] else None,
        'peg': dict(PEG),
        'simple': S.setdefault('simple',
                               {'prefer': 'x', 'clear_cm': 5.0, 'extra_cm': 5.0}),
    }

# ================= STL 썸네일 (위에서 본 실루엣 SVG) =================
_THUMB = {}
@app.get('/api/thumb/{oid}')
def api_thumb(oid: str):
    from fastapi.responses import Response
    if oid in _THUMB:
        return Response(_THUMB[oid], media_type='image/svg+xml')
    try:
        import numpy as np, trimesh
        path, scale = core.MESH_REG[oid]
        m = trimesh.load(path)
        v = np.asarray(m.vertices, float) * scale
        f = np.asarray(m.faces, int)
        if len(f) > 600:                        # 무거운 메쉬는 줄여서
            import fast_simplification as fs
            v2, f2 = fs.simplify(v.astype(np.float32), f, target_count=600)
            v, f = np.asarray(v2, float), np.asarray(f2, int)
        lo = v.min(0); hi = v.max(0)
        w = max(hi[0]-lo[0], 1e-6); h = max(hi[1]-lo[1], 1e-6)
        tris = []
        for tri in f:
            p = v[tri]
            pts = ' '.join(f"{(x-lo[0])/w*100:.1f},{(hi[1]-y)/h*100:.1f}"
                           for x, y, _ in p)
            tris.append(f'<polygon points="{pts}"/>')
        col = '#9aa4b2'
        for e in core.ENV_SHOW:
            if e['id'] == oid: col = e.get('color', col)
        svg = (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 100 100" '
               f'preserveAspectRatio="xMidYMid meet">'
               f'<g fill="{col}" fill-opacity="0.9" stroke="none">'
               + ''.join(tris) + '</g></svg>')
        _THUMB[oid] = svg
        return Response(svg, media_type='image/svg+xml')
    except Exception as e:
        return Response(f'<svg xmlns="http://www.w3.org/2000/svg"/>',
                        media_type='image/svg+xml')

@app.get('/api/map', response_class=HTMLResponse)
def api_map(sel: str = ''):
    return core.render_map2d(sel or None)

# ================= 파일 업로드 (URDF 폴더 / STL) =================
@app.post('/api/upload')
def api_upload(files: list[UploadFile] = File(...), paths: str = Form('')):
    rels = json.loads(paths) if paths else [f.filename for f in files]
    tmp = tempfile.mkdtemp(prefix='asp_up_')
    saved = []
    for f, rel in zip(files, rels):
        rel = (rel or f.filename).lstrip('/').replace('..', '_')
        dst = os.path.join(tmp, rel)
        os.makedirs(os.path.dirname(dst) or tmp, exist_ok=True)
        with open(dst, 'wb') as w:
            shutil.copyfileobj(f.file, w)
        saved.append(dst)
    exts = {os.path.splitext(s)[1].lower() for s in saved}
    if '.urdf' in exts:                       # URDF 패키지 → IK (오래 걸림)
        return start_job('add_path', core.h_add_path, [tmp])
    class _F:                                  # gradio File 흉내 (.name 속성)
        def __init__(self, p): self.name = p
    out = core.h_upload([_F(s) for s in saved])
    return {'ok': True, 'status': status_of(out)}

# ================= 정적 파일 =================
@app.get('/', response_class=HTMLResponse)
def index():
    return FileResponse(os.path.join(UI_DIR, 'index.html'))

app.mount('/ui', StaticFiles(directory=UI_DIR), name='ui')

if __name__ == '__main__':
    for port in range(7900, 7920):
        try:
            print(f'ARM STUDIO PRO: http://localhost:{port}')
            uvicorn.run(app, host='0.0.0.0', port=port, log_level='warning')
            break
        except (OSError, SystemExit):
            print(f'port {port} busy, trying {port+1}...')
