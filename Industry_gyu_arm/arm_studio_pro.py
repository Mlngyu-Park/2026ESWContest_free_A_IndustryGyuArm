#!/usr/bin/env python
"""ARM STUDIO PRO — 새 디자인 UI 백엔드.

기존 arm_studio_web.py 를 '라이브러리'로 import 해서 로봇 제어/IK/상태 로직을
전부 재사용한다 (기존 파일은 수정하지 않음). 화면은 pro_ui/ 의 순수
HTML/CSS/JS 가 담당하고, 이 서버는 REST API 만 제공한다.

실행:  ./start_pro.sh   (또는 conda run -n gyu_arm python arm_studio_pro.py)
"""
import os, re, sys, math, json, shutil, tempfile, threading, traceback
import urllib.request, urllib.parse
import rp_led                 # RP2040 NeoPixel: AI 추론 중 깜빡임 (보드 없으면 아무 일 없음)
import time
import subprocess

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import arm_studio_web as core          # __main__ 가드 덕분에 Gradio 는 안 뜬다

from fastapi import FastAPI, Body, File, Form, UploadFile, Request
from fastapi.responses import FileResponse, HTMLResponse, JSONResponse, Response
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
    sw = core._ensure_pose_arm(name)       # 11-05: 그 자세의 팔로 (자세마다 팔이 정해져 있다)
    core.h_select(name)                    # 3D 미리보기 publish
    return f'자세 선택: {name}' + (f' — 지금 팔 {core.arm_label(sw)}' if sw else '')

def a_seq_insert(item, at):
    """블록 팔레트 드래그 앤 드랍: 파이프라인의 원하는 위치에 블록 삽입.
    item = 자세 이름 | '@open' | '@close' | '@grasp:<물체>' | '@release:<도킹>'
    at   = S['seq'] 기준 삽입 위치 (0 = 맨 앞)"""
    core._undo_snapshot('블록 삽입')
    item = str(item)
    ok = (item in core.pose_names() or item in ('@open', '@close')
          or item.startswith('@grasp:') or item.startswith('@release:')
          or item.startswith('@goto:') or item in ('@down','@up')
          or _meta_ok(item))
    if not ok:
        return f'알 수 없는 블록: {item}'
    with core.LOCK:
        seq = core.S['seq']
        pos = max(0, min(int(at), len(seq)))
        seq.insert(pos, item)
    try:
        with open(core.PROJECT_FILE, 'w') as f:
            json.dump(core.S, f, indent=1)
    except Exception:
        pass
    # 자세/이동 블록이면 3D 미리보기 (goto 는 즉석 IK 자세)
    if not core.RUNNING.is_set():
        try:
            q = core.get_angles(item)
            if q is not None:
                core.ros_publish(q)
        except Exception:
            pass
    return f'블록 삽입: {item} (위치 {pos+1})'

def _seq_save():
    try:
        with open(core.PROJECT_FILE, 'w') as f:
            json.dump(core.S, f, indent=1)
    except Exception:
        pass

def _seq_idx(label):
    """블록 라벨('■ 3. base', '◈ 1. 잡기: ppap' 등) -> S['seq'] 인덱스.
    기존 core.seq_index 는 ◈ 블록을 못 읽어서 여기서 직접 판다."""
    m = re.match(r'^[■◆◈➜▼▲◉⧗]\s*(\d+)\.', str(label))
    return int(m.group(1)) if m else None

def _meta_ok(item):
    """두 팔 블록 (10-30): '@arm:A' (설정에 있는 팔) · '@wait:초' (0~600)."""
    if item.startswith('@arm:'):
        return item[5:] in core.arms_cfg()
    if item.startswith('@wait:arm:'):         # 11-03: 'A팔이 끝날 때까지' 기다리기
        return item[len('@wait:arm:'):] in core.arms_cfg()
    if item.startswith('@wait:'):
        try: return 0 <= float(item[6:]) <= 600
        except ValueError: return False
    return False

def a_seq_del(label):
    """블록 삭제 - ◈잡기/놓기 포함 모든 종류. ┃이동은 기존 로직(녹화 삭제)."""
    core._undo_snapshot('블록 삭제')
    lab = str(label)
    if lab.startswith('┃'):
        return status_of(core.h_seq_del(lab))
    i = _seq_idx(lab)
    if i is None:
        return '삭제할 블록을 못 찾았습니다'
    with core.LOCK:
        seq = core.S['seq']
        if not 0 <= i < len(seq):
            return '삭제할 블록을 못 찾았습니다'
        seq.pop(i)
    _seq_save()
    return f'블록 삭제: {lab}'

def a_seq_move(label, d):
    """▲▼ 한 칸 이동 - ◈ 블록 포함."""
    core._undo_snapshot('블록 이동')
    i = _seq_idx(str(label))
    if i is None:
        return status_of(core.h_seq_move(label, d))
    j = i + int(d)
    with core.LOCK:
        seq = core.S['seq']
        if not (0 <= i < len(seq) and 0 <= j < len(seq)):
            return '더 움직일 수 없습니다'
        seq[i], seq[j] = seq[j], seq[i]
    _seq_save()
    return '블록을 ' + ('위로' if int(d) < 0 else '아래로')

def a_seq_move_to(label, at):
    """드래그로 파이프라인 안 재배치 - 블록을 위치 at 로."""
    core._undo_snapshot('블록 이동')
    i = _seq_idx(str(label))
    if i is None:
        return '이동할 블록을 못 찾았습니다'
    with core.LOCK:
        seq = core.S['seq']
        if not 0 <= i < len(seq):
            return '이동할 블록을 못 찾았습니다'
        item = seq.pop(i)
        pos = int(at)
        if pos > i:
            pos -= 1
        pos = max(0, min(pos, len(seq)))
        seq.insert(pos, item)
    _seq_save()
    return f'블록 이동 -> 위치 {pos+1}'

def a_arm_edit(L):
    """3D 에서 '지금 팔' 을 L 로 — 로봇 옮기기(핀·목심)·자세가 그 팔에 적용된다 (10-30)."""
    if core.RUNNING.is_set(): return '실행 중에는 바꿀 수 없습니다'
    L = str(L).strip().upper()
    if L not in core.arms_cfg(): return f'설정에 없는 팔: {L}'
    core.arm_switch(L); _seq_save()
    return f'3D 지금 팔: {core.arm_label(L)} — 로봇 옮기기는 이 팔에 적용됩니다'

def a_arms_set(spec):
    """팔 이름·색 설정 (2026-10-30). 'A=빨강 B=파랑 C=노랑' — 말·블록에서 '빨간 팔' 로 부른다.
    색은 빨강·주황·노랑·초록·파랑·보라 중 하나, 팔마다 달라야 한다."""
    COLS = ('빨강', '주황', '노랑', '초록', '파랑', '보라')
    out = {}
    for kv in str(spec or '').replace(',', ' ').split():
        if '=' not in kv: return f'형식: A=빨강 B=파랑 (지금: {kv})'
        k, v = kv.split('=', 1); k = k.strip().upper(); v = v.strip()
        if not (len(k) == 1 and 'A' <= k <= 'Z'): return f'팔 이름은 A~Z 한 글자: {k}'
        if v not in COLS: return f'색은 {"·".join(COLS)} 중 하나: {v}'
        out[k] = v
    if not out: return '형식: A=빨강 B=파랑'
    if len(set(out.values())) != len(out): return '팔마다 색이 달라야 합니다 (말로 구분하려면)'
    with core.LOCK:
        core.S['arms'] = out
    _seq_save()
    return '팔 설정: ' + ' · '.join(f'{k}팔={v} 팔' for k, v in out.items())

def a_seq_set(label, item):
    """이중 블록의 슬롯 교체: 그 블록의 사물/대상만 바꾼다."""
    core._undo_snapshot('슬롯 교체')
    i = _seq_idx(str(label))
    item = str(item)
    ok = (item in core.pose_names() or item in ('@open', '@close')
          or item.startswith('@grasp:') or item.startswith('@release:')
          or item.startswith('@goto:') or item in ('@down','@up')
          or _meta_ok(item))
    if i is None or not ok:
        return '교체할 수 없습니다'
    with core.LOCK:
        seq = core.S['seq']
        if not 0 <= i < len(seq):
            return '교체할 블록을 못 찾았습니다'
        seq[i] = item
    _seq_save()
    if item in core.pose_names() and not core.RUNNING.is_set():
        try:
            core.ros_publish(core.get_angles(item))
        except Exception:
            pass
    return f'슬롯 교체: {item}'

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
    'level_trim':    core.h_level_trim,        # ('+0.5'|'-0.5'|'reset'|'status') 지금 팔 그리퍼 수평 (실물 j4 영점, 11-05)
    'ft_grip':       core.h_ft_grip,           # (pose, open_bool)
    'ft_apply':      core.h_apply_nearest,
    'ft_reset':      core.h_ft_reset,
    'ft_save':       core.h_ft_save,
    'cam_load':      core.h_cam_load,          # (영상 경로) 카메라 동기화 — 영상 분석
    'cam_ctl':       core.h_cam_ctl,           # play/pause/fp/rot/x+.../reset
    'cam_status':    core.h_cam_status,
    'live_ctl':      core.h_live_ctl,          # 📱 휴대폰 카메라: rot/x±/y±/fp/view/reset
    'live_status':   core.h_live_status,
    'calmap':        core.h_calmap,            # 🗺 보정 지도 켜기/끄기 ('status' = 상태만)
    'calmap_ok':     core.h_calmap_ok,         # ✅ 보정 성공 (지금 꽂힌 도킹 자리 기록)
    'cam_fit_plates': core.h_cam_fit_plates,   # 영상 판 크기대로 바닥판 더하기
    'dup':           core.h_duplicate,         # (name, newname)
    'del':           core.h_delete,
    'save':          core.h_save,
    'load':          core.h_load,
    'seq_add':       core.h_seq_add,
    'seq_insert':    a_seq_insert,
    'seq_grip':      core.h_seq_grip,          # (open_bool)
    'seq_del':       a_seq_del,                # (label) - ◈ 포함 전 종류
    'seq_move':      a_seq_move,               # (label, ±1) - ◈ 포함
    'seq_move_to':   a_seq_move_to,            # (label, at) - 드래그 재배치
    'seq_set':       a_seq_set,                # (label, item) - 슬롯 교체
    'arms_set':      a_arms_set,               # ('A=빨강 B=파랑') - 팔 이름·색 (10-30)
    'arm_edit':      a_arm_edit,               # ('B') - 3D 에서 옮길/보는 팔 (10-30)
    'seq_grasp':     core.h_seq_grasp,         # (brick)
    'seq_release':   core.h_seq_release,       # (dock)
    'llm_engine':    core.h_llm_engine,
    'say':           core.h_say,               # (텍스트/음성 명령 문장)
    'blk_to_pose':   core.h_block_to_pose,     # (블록 라벨) 미세조정용 자세 등록
    'seq_clear':     core.h_seq_clear,         # 블록 전부 삭제
    'robot_list':    core.h_robot_list,        # 쓸 수 있는 로봇 목록
    'robot_pick':    core.h_robot_pick,        # 로봇 바꾸기 (바로 적용)
    'home_zero':     core.h_home_zero,         # 로봇 원위치 (URDF 기본 자세)
    'joy_table':     core.h_joy_table_build,
    'joy_chunk':     core.h_joy_chunk,         # 조종표 주변 조각 (브라우저용)
    'joy_apply':     core.h_joy_apply,         # 브라우저가 찾은 자세 반영
    'head_build':    core.h_head_build,        # 닭 머리 표 미리 만들기
    'head_reload':   core.h_head_reload,       # 표를 다시 읽기 (새로 만든 뒤)   # 조종용 IK 표 미리계산
    'joy_start':     core.h_joy_start,         # 조이콘 조종 시작
    'joy_stop':      core.h_joy_stop,
    'joy_move':      core.h_joy_move,          # (dx,dy,dz,grip) 3D 만 움직임
    'seq_seek':      core.h_seq_seek,          # 타임바: 그 블록 자세로 즉시 점프
    'seg_mode':      core.h_seg_mode,
    'seg_ax':        core.h_seg_axorder,
    'seg_rot':       core.h_seg_rotmode,
    'seg_prefer':    core.h_seg_prefer,
    'ign_save':      core.h_ign_save,          # (label, [ids])
    'exec_robot':    core.h_exec_robot,
    'robot_status':  core.h_robot_status,
    'stop':          core.h_stop,
    'pause':         core.h_pause,         # 11-06 일시 정지 / 계속 (같은 버튼)
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
    'env_drag':      core.h_env_drag,          # (id | '') 드래그 시작/끝
    'plate_edit':    core.h_plate_edit,
    'plate_remove':  core.h_plate_remove,
    'plate_refresh': core.h_plate_refresh,
    'robot_move_mode': core.h_robot_move_mode,
    'robot_home':    core.h_robot_home,
    'peg_pick':      a_peg_pick,
    'set_name':      lambda oid,name,color=None: (_THUMB.pop(oid,None),
                     core.h_set_name(oid,name,color))[1],   # 색 바꾸면 썸네일 재생성
    'place':         lambda b, d: (core._undo_snapshot('물체 배치'),
                                   core.place_brick_on_docking(b, d)[1])[1],
    'undo':          lambda: core.undo_last()[1],
    'factory_save':  core.h_factory_save,
    'factory_load':  core.h_factory_load,
    'factory_reset': core.h_factory_reset,
    'factory_del':   core.h_factory_del,
    'robot_suggest': core.h_robot_suggest,
    'env_delete':    lambda oid: (core._undo_snapshot('삭제'),
                                  core.env_delete(oid)[1])[1],
    'robot_apply_suggest': core.h_robot_apply_suggest,
}
JOBS = {
    'ft_goto':       core.h_ft_goto,
    'cal_capture':   core.h_cal_capture_zero,
    'zero_diag':     core.h_zero_diag,        # 진단(읽기 전용)
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
    'seq_goto':  core.h_seq_goto,        # 타임바: 이 블록까지 재생
    'preview_from': core.h_preview_from, # 타임바 지점부터 이어재생
    'preview': core.h_preview,   # (kind, mode, axorder, speed, loop, coll)
    'preview_upto': core.h_preview_upto,   # (S['seq'] 인덱스, 속도)
    'run':     core.h_run,
}

@app.post('/api/call')
def api_call(body: dict = Body(...)):
    fn = body.get('fn'); args = body.get('args') or []
    try:
        if fn in SYNC:
            if fn == 'say':
                # 말로 만들기(RACHEL 등 AI 추론) 동안 RP2040 LED 초록/파랑 깜빡임
                with rp_led.busy():
                    out = SYNC[fn](*args)
            else:
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
    return _api_job()

# ===== 음성 인식 =====
#   1순위 Spark whisper-server (정확하지만 터널이 필요)
#   2순위 WHISPER DUSK (whisper_dusk/, 이 노트북에서 혼자 돈다)
#         ears  = Moonshine 한국어 27MB (받아쓰기)
#         fixer = 자작 7.5MB (다듬기)
#
#   2순위가 좀 덜 정확해도 괜찮다 — 뒤에서 hints.py 가 색·번호를
#   정규식으로 잡고, RACHEL 이 오염된 말("도킹"->"토킹")을 견디게
#   학습돼 있다. 실측으로 7가지 오염 형태를 전부 통과했다.
STT_URL = 'http://127.0.0.1:18090'      # spark_link.sh 가 열어주는 터널

_LOCAL_STT = [None]                      # 늦게 한 번만 올린다 (48MB)

_FIXER = [None]


def _fixer():
    """WHISPER DUSK 의 다듬기(자작 7.5M) — 지저분한 말을 고친다.

    없으면 조용히 건너뛴다. 있으면 WHISPER DUSK 결과에만 적용한다 —
    Spark(DGX) 결과는 이미 깨끗해서 손댈 이유가 없다.
    """
    if _FIXER[0] is not None:
        return _FIXER[0] or None
    try:
        sys.path.insert(0, os.path.join(HERE, 'whisper_dusk', 'fixer'))
        import fixer as _f
        if not _f.ready():
            _FIXER[0] = False
            return None
        _FIXER[0] = _f
        print('[WHISPER DUSK] 다듬기 모델을 올렸습니다')
        return _f
    except Exception as e:
        print(f'[WHISPER DUSK] 다듬기를 못 올렸습니다 (건너뜁니다): {e}')
        _FIXER[0] = False
        return None


def _polish(txt):
    """받아쓴 결과를 다듬는다. 실패하면 원문 그대로."""
    f = _fixer()
    if f is None or not txt:
        return txt, None
    try:
        t0 = time.time()
        out = f.fix(txt)
        ms = (time.time() - t0) * 1000
        if out != txt:
            print(f'[WHISPER DUSK] 다듬기 {ms:.0f}ms  {txt[:36]} -> {out[:36]}')
        return out, {'before': txt, 'after': out, 'ms': round(ms)}
    except Exception as e:
        print(f'[WHISPER DUSK] 다듬기 실패 (원문 사용): {e}')
        return txt, None


def _local_stt():
    """로컬 Moonshine 모듈. 없으면 None (그러면 Spark 만 쓴다)."""
    if _LOCAL_STT[0] is not None:
        return _LOCAL_STT[0] or None
    try:
        sys.path.insert(0, os.path.join(HERE, 'whisper_dusk', 'ears'))
        import stt as _s
        if not _s.ready():
            _LOCAL_STT[0] = False; return None
        _LOCAL_STT[0] = _s
        return _s
    except Exception as e:
        print(f'[STT] 로컬 모델 못 올림 (Spark 만 씁니다): {e}')
        _LOCAL_STT[0] = False
        return None

# ── STT 엔진 고르기 ────────────────────────────────────────
#   'auto'      Spark 먼저, 안 되면 Moonshine (기본, 예전 동작)
#   'spark'     Spark 만 쓴다 (DGX, 정확하지만 네트워크 필요)
#   'moonshine' 이 노트북 것만 쓴다 (27M, 네트워크 없어도 됨)
#   'both'      둘 다 돌려서 둘 다 로그에 남긴다 (디버깅용).
#               화면에는 Spark 것을 쓰고, 안 되면 Moonshine 것을 쓴다.
STT_PICK = ['auto']

@app.post('/api/stt_engine')
def api_stt_engine(body: dict = Body(...)):
    want = (body or {}).get('engine') or 'auto'
    if want not in ('auto', 'spark', 'moonshine', 'both'):
        return {'ok': False, 'error': f'모르는 엔진: {want}'}
    if want == 'moonshine' and _local_stt() is None:
        return {'ok': False, 'error': 'WHISPER DUSK 가 없습니다 (whisper_dusk/ears)'}
    STT_PICK[0] = want
    print(f'[STT] 엔진을 {want} 로 바꿨습니다')
    return {'ok': True, 'engine': want}

@app.get('/api/stt_engine')
def api_stt_engine_get():
    return {'ok': True, 'engine': STT_PICK[0],
            'have_moonshine': _local_stt() is not None}

# ── 뷰어 중계 (/viewer/*) ──────────────────────────────────
#   왜 필요한가
#     3D 뷰어는 7870 번 포트로 따로 뜬다. 그런데 이 PC 의 윈도우가
#     7781~7880 을 통째로 예약해 놔서(netsh 의 excludedportrange),
#     WSL 의 localhost 자동 전달이 그 구간을 못 쓴다.
#     -> WSL 안에서는 열리는데 윈도우 브라우저에서는 "연결할 수 없음".
#
#   그래서 이미 열려 있는 7900 으로 받아서 뷰어에 대신 물어본다.
#   사용자는 localhost:7900 하나만 알면 된다 — 뷰어 포트가 몇 번으로
#   잡히든 상관없다.
@app.api_route('/viewer{rest:path}',
               methods=['GET', 'POST', 'OPTIONS'])
async def viewer_proxy(rest: str, request: Request):
    port = getattr(core, 'VIEWER_PORT', 7870)
    path = rest or '/'
    if not path.startswith('/'):
        path = '/' + path
    # ★ 다시 인코딩해서 보낸다 (2026-10-20) ★
    #   rest 는 이미 풀린 값이라 '#'(%23) 이 그대로 들어 있다. 그대로 보내면 '#2' 가
    #   주소의 '페이지 안 위치' 로 잘려 나가, 복제본(docking_station-5#2) 삭제·이동·
    #   핀 배치가 전부 숨은 원본(docking_station-5) 에 걸렸다.
    path = urllib.parse.quote(path, safe="/,:=&@!$'()*+;~")
    if request.url.query:
        path += '?' + request.url.query
    url = f'http://127.0.0.1:{port}{path}'
    try:
        data = await request.body() if request.method == 'POST' else None
        req = urllib.request.Request(url, data=data, method=request.method)
        with urllib.request.urlopen(req, timeout=30) as r:
            body = r.read()
            ctype = r.headers.get('Content-Type', 'application/octet-stream')
        # 뷰어 안의 절대 경로를 중계 경로로 바꿔준다 (/state -> /viewer/state)
        if 'text/html' in ctype:
            txt = body.decode('utf-8', 'replace')
            # ★ 뷰어가 부르는 경로는 전부 여기 있어야 한다 ★ (2026-10-20)
            #   빠지면 7900 의 없는 주소로 가서 404 — 화면엔 아무 오류도 안 뜬다.
            #   (공장 꾸미기 드래그앤드롭·우클릭 이동/삭제·핀 배치가 전부 이걸로 죽어 있었다)
            for a in ('/state', '/tbl', '/mesh', '/stl', '/camframe',
                      '/envdrag', '/envdel', '/envmv', '/armmv', '/rpin', '/rpeg',   # /armmv = 팔 우클릭 옮기기 (11-03)
                      '/peg', '/pedit', '/click', '/undo', '/emoff', '/kin',
                      '/calpt'):                                                      # /calpt = 보정 지도 점 클릭 (11-04)
                txt = txt.replace(f"'{a}", f"'/viewer{a}")
                txt = txt.replace(f'"{a}', f'"/viewer{a}')
                txt = txt.replace(f'`{a}', f'`/viewer{a}')
            body = txt.encode()
        return Response(content=body, media_type=ctype,
                        headers={'Access-Control-Allow-Origin': '*',
                                 'Cache-Control': 'no-store'})
    except Exception as e:
        return Response(content=f'뷰어에 연결하지 못했습니다 ({port}): {e}'.encode(),
                        status_code=502, media_type='text/plain; charset=utf-8')

@app.get('/api/stt_ping')
def api_stt_ping():
    """음성 인식이 가능한지 (마이크 버튼이 방식 결정에 사용).

    Spark 가 죽어도 로컬 Moonshine 이 있으면 ok 다 — 브라우저 내장
    인식으로 떨어지지 않게 한다 (그건 중간에 멋대로 끊긴다)."""
    try:
        urllib.request.urlopen(STT_URL + '/', timeout=1.5)
        return {'ok': True, 'engine': 'spark'}
    except Exception:
        pass
    if _local_stt() is not None:
        return {'ok': True, 'engine': 'moonshine'}
    return {'ok': False}

@app.post('/api/stt')
async def api_stt(request: Request):
    """브라우저 녹음(WAV) -> Spark whisper-server -> 한국어 텍스트."""
    wav = await request.body()
    if len(wav) < 1000:
        return {'ok': False, 'error': '녹음이 너무 짧습니다'}
    b = '----armstudio-stt'
    parts = []
    parts.append((f'--{b}\r\nContent-Disposition: form-data; '
                  'name="file"; filename="mic.wav"\r\n'
                  'Content-Type: audio/wav\r\n\r\n').encode() + wav + b'\r\n')
    for k, v in (('language', 'ko'), ('response_format', 'json')):
        parts.append((f'--{b}\r\nContent-Disposition: form-data; '
                      f'name="{k}"\r\n\r\n{v}\r\n').encode())
    body = b''.join(parts) + f'--{b}--\r\n'.encode()

    pick = STT_PICK[0]

    # 말하는 중 미리보기는 비교를 건너뛴다 (매초 두 번 돌면 느리다)
    if pick == 'both' and request.query_params.get('preview'):
        pick = 'auto'      # 되는 쪽 하나로 — Spark 가 죽어도 미리보기는 나온다

    def _spark():
        req = urllib.request.Request(
            STT_URL + '/inference', data=body,
            headers={'Content-Type': f'multipart/form-data; boundary={b}'})
        out = json.loads(urllib.request.urlopen(req, timeout=60).read())
        return (out.get('text') or '').strip()

    # ── 둘 다 돌려보기 (디버깅) ────────────────────────────
    #   같은 소리를 두 엔진에 똑같이 넣고, 결과를 나란히 찍는다.
    #   무엇이 다른지 눈으로 바로 보라고 만든 것이다.
    if pick == 'both':
        sp_txt = sp_err = None
        t0 = time.time()
        try:
            sp_txt = _spark()
            sp_ms = (time.time() - t0) * 1000
        except Exception as e:
            sp_err = f'{type(e).__name__}: {e}'
            sp_ms = (time.time() - t0) * 1000

        mo_txt = mo_err = None
        mo_raw = mo_fx = None
        m = _local_stt()
        if m is None:
            mo_err = '로컬 모델 없음'
            mo_ms = 0.0
        else:
            t1 = time.time()
            try:
                mo_raw = m.from_wav_bytes(wav)
                mo_txt, mo_fx = _polish(mo_raw)
                mo_ms = (time.time() - t1) * 1000
            except Exception as e:
                mo_err = f'{type(e).__name__}: {e}'
                mo_ms = (time.time() - t1) * 1000

        print('[STT 비교] ────────────────────────────────')
        print(f'  DGX Spark  {sp_ms:6.0f}ms : '
              f'{sp_txt if sp_txt is not None else "실패 — " + str(sp_err)}')
        if mo_fx:
            print(f'  W.DUSK 원본 {mo_ms:6.0f}ms : {mo_raw}')
            print(f'  W.DUSK 다듬 {mo_fx["ms"]:6d}ms : {mo_txt}')
        else:
            print(f'  WHISPER DUSK {mo_ms:6.0f}ms : '
                  f'{mo_txt if mo_txt is not None else "실패 — " + str(mo_err)}')
        if sp_txt and mo_txt:
            print(f'  같은가     : {"예" if sp_txt == mo_txt else "아니오"}')
        print('────────────────────────────────────────')

        # 화면에는 Spark 를 쓰고, 없으면 로컬 것을 쓴다
        use, eng = (sp_txt, 'spark') if sp_txt else (mo_txt, 'moonshine')
        if not use:
            return {'ok': False,
                    'error': f'둘 다 실패 (Spark: {sp_err} / 로컬: {mo_err})'}
        return {'ok': True, 'text': use, 'engine': eng,
                'compare': {'spark': sp_txt, 'spark_ms': round(sp_ms),
                            'spark_err': sp_err,
                            'moonshine': mo_txt, 'moonshine_ms': round(mo_ms),
                            'moonshine_err': mo_err,
                            'moonshine_raw': mo_raw,
                            'fixed': mo_fx}}

    # 이 노트북 것만 쓰라고 골랐으면 Spark 는 아예 안 부른다
    if pick == 'moonshine':
        m = _local_stt()
        if m is None:
            return {'ok': False, 'error': 'WHISPER DUSK 가 없습니다 (whisper_dusk/ears)'}
        try:
            t0 = time.time()
            txt = m.from_wav_bytes(wav)
            print(f'[WHISPER DUSK] 받아쓰기 {(time.time()-t0)*1000:.0f}ms: {txt[:40]}')
            txt, fx = _polish(txt)        # 자작 FIXER 로 다듬는다
            r = {'ok': True, 'text': txt, 'engine': 'moonshine'}
            if fx:
                r['fixed'] = fx
            return r
        except Exception as e:
            return {'ok': False, 'error': f'로컬 STT 실패: {e}'}

    try:
        t0 = time.time()
        txt = _spark()
        print(f'[STT] DGX Spark {(time.time()-t0)*1000:.0f}ms: {txt[:60]}')
        return {'ok': True, 'text': txt, 'engine': 'spark'}
    except Exception as spark_err:
        # ★ 'spark' 를 콕 집어 골랐으면 말없이 바꿔치지 않는다.
        #   어느 엔진이 썼는지 모르면 결과를 비교할 수가 없다.
        if pick == 'spark':
            return {'ok': False,
                    'error': f'Spark 연결 실패: {spark_err}'}
        # auto: Spark 가 안 되면 로컬 Moonshine 으로
        m = _local_stt()
        if m is None:
            return {'ok': False,
                    'error': f'{type(spark_err).__name__}: {spark_err}'}
        try:
            t0 = time.time()
            txt = m.from_wav_bytes(wav)
            print(f'[STT] Moonshine 로컬 {(time.time()-t0)*1000:.0f}ms: {txt[:40]}')
            return {'ok': True, 'text': txt, 'engine': 'moonshine'}
        except Exception as e:
            return {'ok': False, 'error': f'로컬 STT 실패: {e}'}

def _api_job():
    return {'busy': JOB['busy'], 'fn': JOB['fn'], 'status': JOB['status'],
            'prog': core.PROG_STATE['text'], 'error': JOB['error'],
            'result': JOB['result'] if not JOB['busy'] else None}

# ================= 상태 스냅샷 =================
@app.get('/api/meta')
def api_meta():
    joints = []
    # 그리퍼를 뺀 팔 관절만 (로봇마다 축 수가 다르다: SO-ARM 6 / UR15 8)
    for j in range(core.GRIP_IDX):
        lo, hi = core.jr_limits(j)
        joints.append({'label': core.jname(j),
                       'dead': core._dead_joint(j),     # 모터 없는 고정 관절 — 슬라이더 숨김 (값 0 유지)
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
        # ★ 뷰어를 7900 을 거쳐 연다 — 윈도우가 예약한 포트 구간
        #   (7781~7880) 때문에 7870 직결은 브라우저에서 막힌다.
        'viewer_url': '/viewer/',
        'phone_urls': phone_urls(),
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
                      'deg': [round(math.degrees(a), 1) for a in ang[:5]],
                      'arm': p.get('arm')})          # 11-05: 어느 팔 자세인지 (없으면 예전 A팔 자세)
    seq = []
    for lab in core.seq_labels():
        kind, idx = core.seq_index(lab)
        # 블록의 대상 사물 id (알약/슬롯을 '사용자 지정 색'으로 칠하기 위해)
        tgt = None
        mlab = re.match(r'^[■◆◈➜▼▲]\s*(\d+)\.', lab)
        if mlab:
            i2 = int(mlab.group(1))
            it2 = core.S['seq'][i2] if 0 <= i2 < len(core.S['seq']) else ''
            for pref in ('@grasp:', '@release:', '@goto:'):
                if isinstance(it2, str) and it2.startswith(pref):
                    tgt = it2[len(pref):]
        seq.append({'label': lab, 'kind': kind or 'brick', 'target': tgt})
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
        'arms': core.arms_cfg(), 'real_arm': core.REAL_ARM, 'active_arm': core.ACTIVE_ARM[0],
        'level_trim': core.S.get('level_trim') or {},
        'ft_off': ft_off,
        'macros': [core._macro_btn_label(s) for s in core.MACRO_SLOTS],
        'env': core.env_ids(), 'env_on': core.env_visible_ids(),
        'ports': core.scan_serial(),   # USB 자동 감지 (by-id)
        'obstacles': core.obstacle_ids(),
        'plates': core.added_tile_labels(), 'plate_info': core.h_plate_list(),
        'headgen': core.HEAD_GEN[0],      # 닭 머리 표 세대 (바뀌면 조각 재수신)
        'run_at': {**core.RUN_AT,
                   'elapsed': (round(time.time()-core.RUN_AT['t0'],1)
                               if core.RUN_AT.get('running') else 0.0)},
        'seq_exp': core.seq_expand(),
        'cur_q': [round(math.degrees(a),1) for a in core.CURRENT_Q],
        'seek_blk': core.SEQ_SEEK.get('blk',-1),
        'robot_pos': core._robot_pos_str(),
        'rmove_on': bool(core.ROBOT_MOVE.get('on')),
        'factory_presets': list((core.S.get('factory_presets') or {}).keys()),
        'last_preset': core.LAST_PRESET[0],          # 11-05: 지금 배치 (비우고 저장 = 여기에 덮어쓰기)
        'reach': core.seq_reach(),
        # 카드 색 = 실제 3D 색과 동기화용 (names 가 색의 원본) / 배치된 물체
        'env_colors': {e['id']:
                       ((core.S.get('names', {}).get(e['id']) or {})
                        .get('color')) or e.get('color')
                       for e in core.ENV_SHOW},
        'bricks_placed': [b for b, v in core.BRICKS.items()
                          if v.get('pos') is not None],
        'bricks': list(core.BRICKS), 'docking': core.docking_ids(),
        'names': S.get('names', {}),
        'exec': core.EXEC['mode'], 'cal': core.CAL['robot'],
        'cal_joints': core.cal_joint_labels(core.CAL['robot']),
        'running': core.RUNNING.is_set(),
        'paused': core.PAUSE.is_set(), 'pause_wait': core.PAUSE_STATE['on'],   # 11-06 일시 정지
        'run_active': core.RUN_ACTIVE[0],
        'estop_held': core.ESTOP['held'], 'estop_port': bool(core.ESTOP['port']),   # 11-06 아두이노 비상정지
        'prog': core.PROG_STATE['text'],
        'busy': JOB['busy'], 'job_fn': JOB['fn'] if JOB['busy'] else None,
        'peg': dict(PEG),
        'simple': S.setdefault('simple',
                               {'prefer': 'x', 'clear_cm': 5.0, 'extra_cm': 5.0}),
    }

# ================= STL 썸네일 (입체: 등각투영 + 면 음영 SVG) =================
_THUMB = {}
@app.get('/api/thumb/{oid}')
def api_thumb(oid: str):
    from fastapi.responses import Response
    if oid in _THUMB:
        return Response(_THUMB[oid], media_type='image/svg+xml')
    try:
        import math, numpy as np, trimesh
        path, scale = core.MESH_REG[oid]
        m = trimesh.load(path)
        v = np.asarray(m.vertices, float) * scale
        f = np.asarray(m.faces, int)
        if len(f) > 900:                        # 무거운 메쉬는 줄여서
            import fast_simplification as fs
            v2, f2 = fs.simplify(v.astype(np.float32), f, target_count=900)
            v, f = np.asarray(v2, float), np.asarray(f2, int)
        nrm = trimesh.Trimesh(vertices=v, faces=f, process=False).face_normals
        # 등각투영: 위 30도에서 대각으로 내려다본 모습
        c30, s30 = math.cos(math.radians(30)), math.sin(math.radians(30))
        px = (v[:, 0] - v[:, 1]) * c30
        py = (v[:, 0] + v[:, 1]) * s30 - v[:, 2]
        view = np.array([1.0, 1.0, 1.0]) / math.sqrt(3)   # 카메라 쪽
        light = np.array([0.3, 0.8, 0.52])                # 위-옆 조명
        light /= np.linalg.norm(light)
        vis = nrm @ view > 0                              # 뒷면 제거
        fv = f[vis]
        depth = v[:, 0] + v[:, 1] + v[:, 2]
        order = np.argsort(depth[fv].mean(axis=1))        # 먼 면부터 (painter)
        col = '#9aa4b2'
        for e in core.ENV_SHOW:
            if e['id'] == oid: col = e.get('color', col)
        r0, g0, b0 = (int(col[i:i+2], 16) for i in (1, 3, 5))
        sh = nrm[vis] @ light
        lo = np.array([px.min(), py.min()])
        span = max(float(np.ptp(px)), float(np.ptp(py)), 1e-6)
        tris = []
        for k in order:
            tri = fv[k]
            b = 0.38 + 0.62 * max(0.0, float(sh[k]))     # 음영 밝기
            c = f'rgb({int(r0*b)},{int(g0*b)},{int(b0*b)})'
            pts = ' '.join(
                f"{(px[i]-lo[0])/span*100:.1f},{(py[i]-lo[1])/span*100:.1f}"
                for i in tri)
            tris.append(f'<polygon points="{pts}" fill="{c}"/>')
        svg = (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="-4 -4 108 108" '
               f'preserveAspectRatio="xMidYMid meet"><g stroke="none">'
               + ''.join(tris) + '</g></svg>')
        _THUMB[oid] = svg
        return Response(svg, media_type='image/svg+xml')
    except Exception:
        return Response('<svg xmlns="http://www.w3.org/2000/svg"/>',
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
    # ★ 캐시 금지 — 안 그러면 브라우저가 옛 화면을 계속 쓴다.
    #   실제로 STT 선택 상자에 새 항목을 넣었는데도 옛 목록이 떠서,
    #   고른 값이 서버 설정을 도로 덮어쓰는 일이 있었다.
    return FileResponse(os.path.join(UI_DIR, 'index.html'),
                        headers={'Cache-Control': 'no-store, must-revalidate'})

@app.get('/joycon', response_class=HTMLResponse)
def index_joycon():
    """조이콘 진단 화면 — 브라우저에 어떤 값으로 들어오는지 확인용.
    조이콘은 Bluetooth 게임패드라 usbipd 마운트가 필요 없다
    (윈도우 브라우저가 직접 읽어 서버로 보낸다)."""
    return FileResponse(os.path.join(UI_DIR, 'joycon.html'))

@app.get('/ball', response_class=HTMLResponse)
def page_ball():
    """조이콘 테스트 — 3D 공. 로봇과 무관하게 입력만 확인한다."""
    return FileResponse(os.path.join(UI_DIR, 'ball.html'))

@app.get('/imu', response_class=HTMLResponse)
def index_imu():
    """조이콘 IMU 실험 — 3D 공을 기울기로 굴려본다.
    WebHID 로 조이콘을 직접 열어 자이로/가속도 6축을 읽는다
    (표준 게임패드 API 는 IMU 를 주지 않는다)."""
    return FileResponse(os.path.join(UI_DIR, 'imu.html'))

# ★ /ui 의 js·css 도 매번 확인하게 한다 (2026-10-20) ★
#   index.html 만 no-store 였어서, app.js 를 고쳐도 브라우저가 옛 파일을 계속 썼다
#   (자세 삭제 '두 번 묻기' 가 안 나타남). no-cache = 바뀌었을 때만 새로 받는다(ETag).
@app.middleware('http')
async def _ui_no_cache(request: Request, call_next):
    resp = await call_next(request)
    if request.url.path.startswith('/ui/'):
        resp.headers['Cache-Control'] = 'no-cache'
    return resp

app.mount('/ui', StaticFiles(directory=UI_DIR), name='ui')

# ── 📱 휴대폰 카메라 (2026-10-22) ──────────────────────────────
#   아이폰 브라우저는 https 에서만 카메라를 연다 -> 같은 앱을 https(7943) 로도 띄운다.
#   인증서는 자체 서명이라 처음 한 번 '안전하지 않음' 경고에서 [계속] 을 눌러야 한다.
PHONE_PORT = 7943
CERT_DIR = os.path.join(HERE, 'certs')


@app.get('/phone', response_class=HTMLResponse)
def page_phone():
    return FileResponse(os.path.join(UI_DIR, 'phone.html'),
                        headers={'Cache-Control': 'no-store'})


@app.post('/api/live/frame')
async def api_live_frame(request: Request):
    data = await request.body()
    lv = core._livecam()
    if lv is None:
        return JSONResponse({'ok': False, 'msg': '휴대폰 카메라 모듈이 없습니다'}, 500)
    if len(data) < 1000:
        return {'ok': False, 'msg': '프레임이 비었습니다'}
    return lv.push(data)


def _lan_ips():
    """이 컴퓨터의 LAN 주소들 (휴대폰이 접속할 주소)."""
    ips = []
    try:
        out = subprocess.run(['hostname', '-I'], capture_output=True, text=True, timeout=3).stdout
        ips = [x for x in out.split() if '.' in x and not x.startswith(('127.', '172.17.'))]
    except Exception:
        pass
    return ips


def phone_urls():
    import socket
    host = socket.gethostname()
    return ([f'https://{ip}:{PHONE_PORT}/phone' for ip in _lan_ips()]
            + [f'https://{host}.mshome.net:{PHONE_PORT}/phone'])


def _ensure_cert():
    """자체 서명 인증서 (없으면 만든다). 주소가 바뀌어도 경고만 같고 동작은 한다."""
    crt, key = os.path.join(CERT_DIR, 'phone.crt'), os.path.join(CERT_DIR, 'phone.key')
    if os.path.exists(crt) and os.path.exists(key):
        return crt, key
    import socket
    os.makedirs(CERT_DIR, exist_ok=True)
    host = socket.gethostname()
    san = ','.join([f'DNS:{host}', f'DNS:{host}.mshome.net', 'DNS:localhost', 'IP:127.0.0.1']
                   + [f'IP:{ip}' for ip in _lan_ips()])
    r = subprocess.run(['openssl', 'req', '-x509', '-newkey', 'rsa:2048', '-nodes', '-days', '3650',
                        '-keyout', key, '-out', crt, '-subj', '/CN=ARM STUDIO phone camera',
                        '-addext', f'subjectAltName={san}'], capture_output=True, text=True)
    if r.returncode != 0:
        raise RuntimeError(r.stderr.strip()[-200:])
    return crt, key


# ── 터널용 작은 서버 (7944) ── 휴대폰 페이지와 영상 받기, 이 두 개'만' 있다.
#   devtunnel 로 인터넷에 열 때 이것만 연다 — 앱 전체(로봇 조작)를 열면 주소만 알면 누구나 조작한다.
PHONE_TUNNEL_PORT = 7944
phone_app = FastAPI()
phone_app.get('/phone', response_class=HTMLResponse)(page_phone)
phone_app.get('/', response_class=HTMLResponse)(page_phone)
phone_app.post('/api/live/frame')(api_live_frame)


def start_phone_tunnel_server():
    try:
        cfg = uvicorn.Config(phone_app, host='127.0.0.1', port=PHONE_TUNNEL_PORT, log_level='warning')
        threading.Thread(target=uvicorn.Server(cfg).run, daemon=True).start()
        print(f'📱 휴대폰 카메라(터널용, 카메라 기능만): http://127.0.0.1:{PHONE_TUNNEL_PORT}/phone'
              f'  ->  devtunnel host -p {PHONE_TUNNEL_PORT} --allow-anonymous')
    except Exception as e:
        print(f'📱 터널용 서버 시작 실패: {e}')


def start_phone_https():
    """같은 앱을 https 로 하나 더 띄운다 (휴대폰 카메라용). 실패해도 앱은 그대로."""
    try:
        crt, key = _ensure_cert()
        cfg = uvicorn.Config(app, host='0.0.0.0', port=PHONE_PORT, log_level='warning',
                             ssl_certfile=crt, ssl_keyfile=key)
        threading.Thread(target=uvicorn.Server(cfg).run, daemon=True).start()
        for u in phone_urls():
            print(f'📱 휴대폰 카메라: {u}')
    except Exception as e:
        print(f'📱 휴대폰 카메라 https 시작 실패: {e}')

# ── 68000 어레이 3D 뷰어 (start_m68k.sh 전용) ──────────────────
#   이 앱은 68000 코드를 갖고 있지 않다. 별도 파일 m68k_server.py 에 있고,
#   환경변수 ARM_STUDIO_LAUNCHER=m68k 일 때만 붙인다.
#   start_pro.sh 로 띄우면 이 블록은 그냥 지나간다 — /api/m68k/* 도,
#   /m68k 도 존재하지 않는다. 68000 쪽을 아무리 고쳐도 여기는 안 바뀐다.
if os.environ.get('ARM_STUDIO_LAUNCHER') == 'm68k':
    try:
        import m68k_server
        m68k_server.attach(app, core, JOB, start_job, STREAMS)
    except Exception as _e:
        print(f'[M68K] 붙이기 실패 (앱은 그대로 동작): {_e}')

# ── 🖥 라떼판다 USB 웹캠·마이크 (2026-11-04) ── 코드는 panda_devices.py 에. 없거나 실패해도 앱은 그대로.
try:
    import panda_devices
    panda_devices.attach(app, core)
except Exception as _e:
    print(f'[판다 장치] 붙이기 실패 (앱은 그대로 동작): {_e}')

# ===== 새 디자인 (비교용) — 기존 화면(/)은 그대로 두고 /new 로 따로 =====
UINEW_DIR = os.path.join(HERE, 'ui_new')
if os.path.isdir(UINEW_DIR):
    @app.get('/new', response_class=HTMLResponse)
    def index_new():
        return FileResponse(os.path.join(UINEW_DIR, 'index.html'))

    app.mount('/uinew', StaticFiles(directory=UINEW_DIR), name='uinew')

# ===== 새 디자인 (비교용) — 기존 화면은 그대로 두고 /v2 로 따로 띄운다 =====
UI2_DIR = os.path.join(HERE, 'ui2')
if os.path.isdir(UI2_DIR):
    @app.get('/v2', response_class=HTMLResponse)
    def index_v2():
        return FileResponse(os.path.join(UI2_DIR, 'index.html'))
    app.mount('/ui2', StaticFiles(directory=UI2_DIR), name='ui2')

if __name__ == '__main__':
    start_phone_https()
    start_phone_tunnel_server()
    for port in range(7900, 7920):
        try:
            print(f'ARM STUDIO PRO: http://localhost:{port}')
            uvicorn.run(app, host='0.0.0.0', port=port, log_level='warning')
            break
        except (OSError, SystemExit):
            print(f'port {port} busy, trying {port+1}...')
