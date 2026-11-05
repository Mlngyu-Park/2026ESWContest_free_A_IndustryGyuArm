#!/usr/bin/env python
"""조종용 자세표 만들기 — 관절 기준 전수 탐색 (CPU 여러 코어 동시 사용).

왜 관절 기준인가:
  · 좌표(공간) 기준 표는 로봇을 옮기는 순간 전부 무효가 된다.
    관절값은 로봇 자신의 것이라 어디에 놓든 그대로 쓸 수 있다.
    (표의 키도 **로봇 기준 좌표**로 담아 이동/회전에 영향받지 않는다)
  · 관절 -> 좌표는 FK(정기구학)라 IK 보다 수백 배 빠르다.
    그래서 '가능한 자세'를 통째로 훑을 수 있다.

빠르게 만드는 방법:
  · URDF 에서 기구학을 직접 읽어 행렬 연산으로 FK 를 푼다
    (ikpy 를 한 자세씩 호출하는 것보다 수십 배 빠르다)
  · **GPU** 가 있으면 수십만 자세를 한 번에 계산한다 (43분 -> 1분)
  · GPU 가 없으면 **CPU 코어 수만큼** 나눠서 동시에 돌린다

중단하고 이어서 하기:
  Ctrl+C 로 멈춰도 지금까지 만든 표가 저장된다.
  다시 실행하면 남은 부분부터 이어서 만든다 (진행 상황을 함께 저장).

사용:
    ./build_joy_table.sh            이어서 만들기 (기본)
    ./build_joy_table.sh --fresh    처음부터 새로
"""
import os, sys, time, json, math, signal
import xml.etree.ElementTree as ET
from multiprocessing import Pool, cpu_count

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

SAVE_EVERY_SEC = 300   # 중간 저장 간격 (5분)
TILT_MAX = 8.0         # 끝단이 이 각도 안이어야 '바닥과 평행'(변형 물컵 규칙)
# 2026-08-24: 조이콘 조종에서 표 적중률이 10% 밖에 안 됐다(20칸 중 2칸).
#   -> 표에 없으면 매번 IK(25ms)를 풀어 반응이 136ms 로 느려진다.
#   좌표 격자만 줄이면 빈칸이 생기므로(채울 자세가 모자람) 관절 격자도
#   함께 줄인다. 15도 + 10mm 조합이 시간(약 3~4시간) 대비 현실적.
STEP_DEG = 15.0        # 관절 격자 간격
POS_GRID_MM = 10.0     # 좌표 격자 (20 -> 10mm, 적중률 8배)
CHUNK = 40000          # 한 번에 묶어 계산할 자세 수

_STOP = False


# ---------- URDF 를 행렬 연산용으로 읽기 ----------
def load_chain(urdf_path, tool_off):
    """조인트 순서대로 (원점 이동, 원점 회전, 회전축) 을 뽑는다."""
    root = ET.parse(urdf_path).getroot()
    joints = []
    for j in root.findall('joint'):
        if (j.get('type') or '').lower() not in ('revolute', 'continuous'):
            continue
        o = j.find('origin')
        xyz = np.array([float(x) for x in
                        ((o.get('xyz') if o is not None else None) or '0 0 0').split()])
        rpy = np.array([float(x) for x in
                        ((o.get('rpy') if o is not None else None) or '0 0 0').split()])
        a = j.find('axis')
        ax = np.array([float(x) for x in
                       ((a.get('xyz') if a is not None else None) or '0 0 1').split()])
        n = np.linalg.norm(ax)
        joints.append({'xyz': xyz, 'rpy': rpy, 'axis': ax / (n if n > 1e-9 else 1.0),
                       'name': j.get('name')})
    return joints, np.asarray(tool_off, float)


def rpy_mat(rpy):
    a, b, c = rpy
    ca, sa, cb, sb, cc, sc = (math.cos(a), math.sin(a), math.cos(b),
                              math.sin(b), math.cos(c), math.sin(c))
    Rx = np.array([[1, 0, 0], [0, ca, -sa], [0, sa, ca]])
    Ry = np.array([[cb, 0, sb], [0, 1, 0], [-sb, 0, cb]])
    Rz = np.array([[cc, -sc, 0], [sc, cc, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


# ---------- GPU 로 FK 한꺼번에 ----------
_TORCH = None
def _get_torch():
    """GPU 를 쓸 수 있으면 torch 를 준비한다 (없으면 None -> CPU 로)."""
    global _TORCH
    if _TORCH is not None:
        return _TORCH if _TORCH is not False else None
    try:
        import torch
        if torch.cuda.is_available():
            _TORCH = torch
            return torch
    except Exception:
        pass
    _TORCH = False
    return None


def _fk_gpu_t(joints, tool_off, Qt, torch, dev):
    """이미 GPU 에 올라간 자세 텐서로 FK (CPU 왕복 없음)."""
    N = Qt.shape[0]
    P = torch.zeros((N, 3), dtype=torch.float32, device=dev)
    R = torch.eye(3, dtype=torch.float32, device=dev).expand(N, 3, 3).clone()
    for k, j in enumerate(joints):
        R0 = torch.as_tensor(rpy_mat(j['rpy']), dtype=torch.float32, device=dev)
        xyz = torch.as_tensor(j['xyz'], dtype=torch.float32, device=dev)
        P = P + torch.einsum('nij,j->ni', R, xyz)
        R = R @ R0
        ax = torch.as_tensor(j['axis'], dtype=torch.float32, device=dev)
        th = Qt[:, k]
        c = torch.cos(th)[:, None, None]; s_ = torch.sin(th)[:, None, None]
        K = torch.zeros((3, 3), dtype=torch.float32, device=dev)
        K[0, 1], K[0, 2] = -ax[2], ax[1]
        K[1, 0], K[1, 2] = ax[2], -ax[0]
        K[2, 0], K[2, 1] = -ax[1], ax[0]
        I = torch.eye(3, dtype=torch.float32, device=dev)
        R = R @ (I * c + K * s_ + torch.outer(ax, ax) * (1 - c))
    t = torch.as_tensor(tool_off, dtype=torch.float32, device=dev)
    return P + torch.einsum('nij,j->ni', R, t), R


def fk_batch_gpu(joints, tool_off, Q):
    """GPU 에서 자세 수십만 개의 FK 를 한 번에 계산한다.
    CPU 보다 훨씬 빠르다 (행렬 곱을 통째로 병렬 처리)."""
    torch = _get_torch()
    dev = torch.device('cuda')
    N = Q.shape[0]
    Qt = torch.as_tensor(Q, dtype=torch.float32, device=dev)
    P = torch.zeros((N, 3), dtype=torch.float32, device=dev)
    R = torch.eye(3, dtype=torch.float32, device=dev).expand(N, 3, 3).clone()
    for k, j in enumerate(joints):
        R0 = torch.as_tensor(rpy_mat(j['rpy']), dtype=torch.float32, device=dev)
        xyz = torch.as_tensor(j['xyz'], dtype=torch.float32, device=dev)
        P = P + torch.einsum('nij,j->ni', R, xyz)
        R = R @ R0
        ax = torch.as_tensor(j['axis'], dtype=torch.float32, device=dev)
        th = Qt[:, k]
        c = torch.cos(th)[:, None, None]
        s = torch.sin(th)[:, None, None]
        K = torch.zeros((3, 3), dtype=torch.float32, device=dev)
        K[0, 1], K[0, 2] = -ax[2], ax[1]
        K[1, 0], K[1, 2] = ax[2], -ax[0]
        K[2, 0], K[2, 1] = -ax[1], ax[0]
        I = torch.eye(3, dtype=torch.float32, device=dev)
        Rj = I * c + K * s + torch.outer(ax, ax) * (1 - c)
        R = R @ Rj
    t = torch.as_tensor(tool_off, dtype=torch.float32, device=dev)
    P = P + torch.einsum('nij,j->ni', R, t)
    return P, R


def fk_batch(joints, tool_off, Q):
    """자세 여러 개(Q: N×축수)의 툴 위치와 끝단 회전을 한 번에 계산.
    반환: 위치 N×3, 회전 N×3×3 (로봇 기준)"""
    N = Q.shape[0]
    P = np.zeros((N, 3))
    R = np.tile(np.eye(3), (N, 1, 1))
    for k, j in enumerate(joints):
        R0 = rpy_mat(j['rpy'])
        P = P + np.einsum('nij,j->ni', R, j['xyz'])
        R = R @ R0
        # 축 회전 (로드리게스)
        ax = j['axis']
        th = Q[:, k]
        c, s = np.cos(th), np.sin(th)
        K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
        Rj = (np.eye(3)[None, :, :] * c[:, None, None]
              + K[None, :, :] * s[:, None, None]
              + np.outer(ax, ax)[None, :, :] * (1 - c)[:, None, None])
        R = R @ Rj
    P = P + np.einsum('nij,j->ni', R, tool_off)
    return P, R


# ---------- 병렬 작업 ----------
_W = {}


def _init(urdf, tool_off, axes_list, end_axis, plate_top, grid):
    _W['j'], _W['t'] = load_chain(urdf, tool_off)
    _W['axes'] = [np.asarray(a) for a in axes_list]
    _W['end'] = np.asarray(end_axis, float)
    _W['plate'] = plate_top
    _W['grid'] = grid


def _combo_at(idx):
    """일련번호 -> 관절 조합 (전체를 메모리에 안 올리려고)"""
    axes = _W['axes']
    out = np.empty((len(idx), len(axes)))
    rem = idx.copy()
    for k in range(len(axes) - 1, -1, -1):
        n = len(axes[k])
        out[:, k] = axes[k][rem % n]
        rem //= n
    return out


def _work(rng):
    """일련번호 구간을 계산해 {칸: (기울기, 관절값)} 로 돌려준다."""
    lo, hi = rng
    idx = np.arange(lo, hi, dtype=np.int64)
    Q = _combo_at(idx)
    P, R = fk_batch(_W['j'], _W['t'], Q)
    keep = P[:, 2] >= _W['plate'] + 0.02          # 바닥 아래 제외
    if not keep.any():
        return {}
    P, R, Q = P[keep], R[keep], Q[keep]
    v = np.einsum('nij,j->ni', R, _W['end'])      # 끝단 회전축 방향
    nv = np.linalg.norm(v, axis=1)
    nv[nv < 1e-9] = 1.0
    tilt = np.degrees(np.arcsin(np.clip(np.abs(v[:, 2]) / nv, -1, 1)))
    g = _W['grid']
    K = np.round(P / g).astype(np.int32)
    out = {}
    for i in range(len(Q)):
        k = (int(K[i, 0]), int(K[i, 1]), int(K[i, 2]))
        t = float(tilt[i])
        if t > TILT_MAX:              # 기울어진 칸은 버린다
            continue
        old = out.get(k)
        if old is None or t < old[0]:
            out[k] = (t, [round(float(x), 5) for x in Q[i]])
    return out


def run_gpu(joints, tool_off, axes, end_axis, plate_top, grid,
            total, start_at, table, best, save_cb):
    """GPU 로 전 구간을 처리한다. 반환: 마지막까지 진행한 위치."""
    torch = _get_torch()
    BIG = 400000                      # GPU 는 한 번에 크게 묶을수록 유리
    # 중간 저장 간격(초). 짧으면 저장만 하다 시간을 다 쓴다.
    # 5분마다면 3시간 작업에 36번 -> 중단해도 최대 5분치만 잃는다.
    dev = torch.device('cuda')
    axes_t = [torch.as_tensor(a, dtype=torch.float32, device=dev) for a in axes]
    sizes = [len(a) for a in axes]
    end_t = torch.as_tensor(end_axis, dtype=torch.float32, device=dev)
    t0 = time.time(); done = start_at; last_save = start_at
    last_save_t = time.time()
    while done < total and not _STOP:
        hi = min(done + BIG, total)
        idx = torch.arange(done, hi, dtype=torch.int64, device=dev)
        # 일련번호 -> 관절 조합
        Q = torch.empty((hi - done, len(axes)), dtype=torch.float32, device=dev)
        rem = idx.clone()
        for k in range(len(axes) - 1, -1, -1):
            Q[:, k] = axes_t[k][(rem % sizes[k]).long()]
            rem = rem // sizes[k]
        P, R = _fk_gpu_t(joints, tool_off, Q, torch, dev)
        keep = P[:, 2] >= plate_top + 0.02
        if keep.any():
            P, R, Q = P[keep], R[keep], Q[keep]
            v = torch.einsum('nij,j->ni', R, end_t)
            nv = torch.linalg.norm(v, dim=1).clamp(min=1e-9)
            tilt = torch.rad2deg(torch.arcsin((v[:, 2].abs() / nv).clamp(0, 1)))
            K = torch.round(P / grid).to(torch.int32)
            # 같은 칸은 기울기가 가장 작은 것만 남긴다 (GPU 정렬 활용)
            order = torch.argsort(tilt)
            K, Q2, tl = K[order], Q[order], tilt[order]
            Kc = K.cpu().numpy(); Qc = Q2.cpu().numpy(); Tc = tl.cpu().numpy()
            seen = set()
            for i in range(len(Kc)):
                key = (int(Kc[i, 0]), int(Kc[i, 1]), int(Kc[i, 2]))
                if key in seen:
                    continue
                seen.add(key)
                t = float(Tc[i])
                if t > TILT_MAX:      # 변형 물컵 규칙에 못 미치는 칸은 버린다
                    continue
                if key not in best or t < best[key]:
                    best[key] = t
                    table[key] = [round(float(x), 5) for x in Qc[i]]
        done = hi
        el = time.time() - t0
        frac = max(1, done - start_at) / max(1, total - start_at)
        print(f'   {min(100,int(frac*100)):>3}%  ·  {len(table):,}칸  ·  '
              f'남은 시간 약 {(el/frac-el)/60:.0f}분   ', end='\r', flush=True)
        # 저장은 '시간 기준' 으로. 표가 100MB 를 넘으면 한 번 저장에 수십 초가
        # 걸려서, 자세 개수로 잡으면 2000번 넘게 저장하다 하루를 다 쓴다.
        if time.time() - last_save_t >= SAVE_EVERY_SEC:
            _t = time.time()
            save_cb(done); last_save = done
            last_save_t = time.time()
            if last_save_t - _t > 5:
                print(f'\n  (표가 커져 저장에 {last_save_t-_t:.0f}초 걸립니다)')
    return done


def _lock_or_exit():
    """같은 표를 두 개가 동시에 만들면 파일이 깨진다. 하나만 돌게 막는다."""
    import atexit
    lk = os.path.join(os.path.dirname(os.path.abspath(__file__)), '.joytbl.lock')
    try:
        fd = os.open(lk, os.O_CREAT | os.O_EXCL | os.O_WRONLY)
    except FileExistsError:
        try:
            old_pid = int(open(lk).read().strip() or 0)
            os.kill(old_pid, 0)          # 살아 있으면 예외가 안 난다
            print(f'이미 만들고 있습니다 (프로세스 {old_pid}). '
                  '멈추려면: pkill -f build_joy_table.py')
            sys.exit(1)
        except (ValueError, ProcessLookupError, PermissionError, OSError):
            os.remove(lk)                # 예전에 죽은 흔적 -> 치우고 진행
            fd = os.open(lk, os.O_CREAT | os.O_EXCL | os.O_WRONLY)
    os.write(fd, str(os.getpid()).encode()); os.close(fd)
    atexit.register(lambda: os.path.exists(lk) and os.remove(lk))


def main():
    fresh = '--fresh' in sys.argv
    _lock_or_exit()
    print('앱 로직 로드 중... (20초쯤)')
    import arm_studio_web as m

    tbl_file = m._JOY_TBL_FILE
    prog_file = tbl_file.replace('.json', '_progress.json')

    print()
    print('=' * 62)
    print(f"  로봇 : {os.path.basename(m.BASE_URDF)}  ({m.NJOINT}축)")
    print(f"  저장 : {tbl_file}")
    _t = _get_torch()
    if _t is not None:
        print(f"  계산 : GPU  {_t.cuda.get_device_name(0)}")
    else:
        print(f"  계산 : CPU {cpu_count()}개 코어 (GPU 를 못 찾았습니다)")
    print(f"  환경 : {os.path.basename(os.path.dirname(os.path.dirname(sys.executable)))}")
    print('=' * 62)

    m.JOY_GRID = POS_GRID_MM / 1000.0
    table, best = {}, {}
    start_at = 0
    if not fresh:
        n = m.joy_table_load()
        if n:
            table = dict(m.JOY_TABLE)
            # 기존 칸의 기울기는 다시 재지 않는다 (수십만 칸이면 몇 분씩 걸림).
            # 새 칸이 더 좋으면 덮어쓰도록 넉넉한 값으로 둔다.
            best = dict.fromkeys(table, 99.0)
            if abs(m.JOY_GRID-POS_GRID_MM/1000.0)>1e-9:
                print(f'  (기존 표는 격자 {m.JOY_GRID*1000:.0f}mm — '
                      f'지금은 {POS_GRID_MM:.0f}mm 라 섞지 않고 새로 만듭니다)')
                table={}; best={}
                m.JOY_GRID=POS_GRID_MM/1000.0
            else:
                print(f'기존 표 {n:,}칸 불러옴')
        try:
            with open(prog_file) as f:
                pr = json.load(f)
            if pr.get('urdf') == os.path.basename(m.BASE_URDF) \
               and pr.get('step') == STEP_DEG:
                start_at = int(pr.get('done', 0))
                print(f'이어서 시작: {start_at:,} 번째부터')
            else:
                print(f"격자가 {pr.get('step')}도 -> {STEP_DEG:.0f}도 로 바뀌어 "
                      '처음부터 만듭니다 (기존 칸은 그대로 둡니다)')
        except Exception:
            pass
    else:
        try:
            os.remove(prog_file)
        except Exception:
            pass
        print('처음부터 새로 만듭니다 (--fresh)')

    # 축별 각도 목록
    axes = []
    for k in range(m.NJOINT):
        lo, hi = m.jr_limits(k)
        lo = max(lo, -math.pi); hi = min(hi, math.pi)
        n = max(2, int(round(math.degrees(hi - lo) / STEP_DEG)) + 1)
        axes.append(np.linspace(lo, hi, n))
    total = 1
    for a in axes:
        total *= len(a)
    print(f'관절 격자 {STEP_DEG:.0f}도  ·  '
          + '×'.join(str(len(a)) for a in axes) + f' = {total:,}가지')
    if start_at >= total:
        print('이미 다 만들었습니다. 처음부터 하려면 --fresh')
        return
    print()

    def on_sigint(*_):
        global _STOP
        _STOP = True
        print('\n중단 요청 — 지금까지 만든 것을 저장하고 종료합니다...')
    signal.signal(signal.SIGINT, on_sigint)

    def save(done):
        m.JOY_TABLE.clear(); m.JOY_TABLE.update(table)
        m.joy_table_save()
        try:
            with open(prog_file, 'w') as f:
                json.dump({'urdf': os.path.basename(m.BASE_URDF),
                           'step': STEP_DEG, 'done': int(done),
                           'total': int(total)}, f)
        except Exception:
            pass

    t0 = time.time()
    done = start_at
    if _t is not None:
        j, to = load_chain(m.BASE_URDF, m.TOOL_OFF)
        done = run_gpu(j, to, [a for a in axes],
                       getattr(m, 'END_AXIS_LOCAL', np.array([0., -1., 0.])),
                       m.PLATE_TOP, m.JOY_GRID, total, start_at,
                       table, best, save)
        save(min(done, total))
        _finish(m, table, best, tbl_file, t0, done, total)
        return
    print('  GPU 없음 — CPU 여러 코어로 진행합니다')
    ranges = [(s, min(s + CHUNK, total))
              for s in range(start_at, total, CHUNK)]
    with Pool(processes=max(1, cpu_count() - 1),
              initializer=_init,
              initargs=(m.BASE_URDF, m.TOOL_OFF, [a.tolist() for a in axes],
                        getattr(m, 'END_AXIS_LOCAL', np.array([0., -1., 0.])),
                        m.PLATE_TOP, m.JOY_GRID)) as pool:
        for res in pool.imap_unordered(_work, ranges, chunksize=1):
            done += CHUNK
            for k, (t, q) in res.items():
                if k not in best or t < best[k]:
                    best[k] = t
                    table[k] = q
            if _STOP:
                break
            el = time.time() - t0
            frac = max(1, done - start_at) / max(1, total - start_at)
            eta = el / frac - el
            print(f'   {min(100,int(frac*100)):>3}%  ·  {len(table):,}칸  ·  '
                  f'남은 시간 약 {eta/60:.0f}분', end='\r', flush=True)
            if done % (CHUNK * 20) < CHUNK:
                save(min(done, total))

    save(min(done, total))
    _finish(m, table, best, tbl_file, t0, done, total)


def _finish(m, table, best, tbl_file, t0, done, total):
    lv = sum(1 for v in best.values() if v < 8)
    try:
        sz = os.path.getsize(tbl_file) / 1024 / 1024
    except Exception:
        sz = 0
    print()
    print()
    print('=' * 62)
    if _STOP:
        print(f'  중단됨 — 여기까지 저장했습니다 ({done:,}/{total:,})')
        print('  다시 실행하면 이어서 만듭니다')
    else:
        print(f'  완료 — 전체 {len(table):,}칸  '
              f'({(time.time()-t0)/60:.1f}분, {sz:.1f}MB)')
    print(f'  그중 {lv:,}칸은 끝단이 바닥과 평행 (변형 물컵 규칙)')
    print('  * 로봇을 옮기거나 돌려도 이 표는 그대로 유효합니다')
    print('=' * 62)


if __name__ == '__main__':
    main()
