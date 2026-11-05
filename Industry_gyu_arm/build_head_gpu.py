#!/usr/bin/env python3
"""닭 머리 표 만들기 — GPU 판 (수만 점을 한 번에 푼다)

CPU 판(build_head_table.py)은 ikpy 로 한 점씩 푼다(점당 33ms).
여기서는 IK 를 '오차를 줄이는 최적화' 로 보고 torch 로 수만 점을 동시에
경사하강한다 -> 점당 0.5ms (66배).

  오차 = |툴위치 - 목표|  +  |l7 방향 - base 방향|
  end 축은 0 으로 고정 (닭 머리: l7 자신이 방향을 만들어야 한다)

  ./build_head_gpu.sh           ±300mm
  ./build_head_gpu.sh 1200      전 범위 (약 2시간)
  ./build_head_gpu.sh 1200 --fresh
중단해도 저장되고, 다시 실행하면 없는 칸만 이어서 만든다.
"""
import os, sys, time, math, json
import numpy as np

BATCH = 60000          # 한 번에 GPU 로 보낼 점 수
ITERS = 600            # 경사하강 반복 (body 모드는 조금 더 필요)
POS_TOL = 0.006        # 위치 허용 (m)
ROT_TOL = 2.0          # 방향 허용 (도)


def main():
    rng_mm = float(sys.argv[1]) if len(sys.argv) > 1 and not sys.argv[1].startswith('-') else 300.0
    fresh = '--fresh' in sys.argv
    # --hours N : 그 시간이 지나면 지금까지 푼 것을 저장하고 멈춘다.
    # (밤새 돌릴 때 정해둔 시각에 깔끔히 끝나게)
    limit_h = 0.0
    for a in sys.argv:
        if a.startswith('--hours='):
            try: limit_h = float(a.split('=',1)[1])
            except Exception: pass
    print('앱 로직 로드 중... (20초쯤)')
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import torch
    import arm_studio_web as m
    if not torch.cuda.is_available():
        print('GPU 를 못 찾았습니다. build_head_table.sh (CPU) 를 쓰세요.')
        return
    dev = torch.device('cuda')
    NJ = m.NJOINT

    # ---- URDF 관절 정의를 GPU 로 ----
    XYZ, RPY, AX = [], [], []
    for jn in m.JOINTS:
        b = m.BASE_DEF[jn]
        XYZ.append(b['xyz']); RPY.append(b['rpy']); AX.append(b['axis'])
    XYZ = torch.tensor(np.array(XYZ), dtype=torch.float32, device=dev)
    AX = torch.tensor(np.array(AX), dtype=torch.float32, device=dev)
    AX = AX / AX.norm(dim=1, keepdim=True)

    def rpy_mat(r, p, y):
        cr,sr,cp,sp,cy,sy = (math.cos(r),math.sin(r),math.cos(p),
                             math.sin(p),math.cos(y),math.sin(y))
        return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                         [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                         [-sp,   cp*sr,          cp*cr]])
    R0 = torch.tensor(np.array([rpy_mat(*r) for r in RPY]),
                      dtype=torch.float32, device=dev)
    TOOL = torch.tensor(np.asarray(m.TOOL_OFF), dtype=torch.float32, device=dev)
    # j1(맨 아래 축)이 +z 로 도는지 -z 로 도는지 (URDF 마다 다르다)
    J1S = -1.0 if np.asarray(m.BASE_DEF[m.JOINTS[0]]['axis'], float)[2] < 0 else 1.0
    _b = np.asarray(m.tool_pos([0.0]*NJ), float)
    J1REF = math.atan2(_b[1], _b[0])          # base 자세의 방위각
    LIM = torch.tensor(np.array([m.jr_limits(j) for j in range(NJ)]),
                       dtype=torch.float32, device=dev)

    def fk(Q):
        N = Q.shape[0]
        R = torch.eye(3, device=dev).expand(N,3,3).contiguous()
        P = torch.zeros(N,3, device=dev)
        R7 = None
        I = torch.eye(3, device=dev)
        for k in range(NJ):
            P = P + torch.einsum('nij,j->ni', R, XYZ[k])
            R = R @ R0[k]
            a = AX[k]
            c = torch.cos(Q[:,k])[:,None,None]; s = torch.sin(Q[:,k])[:,None,None]
            K = torch.zeros(3,3, device=dev)
            K[0,1],K[0,2] = -a[2], a[1]
            K[1,0],K[1,2] =  a[2],-a[0]
            K[2,0],K[2,1] = -a[1], a[0]
            R = R @ (I*c + K*s + torch.outer(a,a)*(1-c))
            if k == NJ-2: R7 = R.clone()
        return P + torch.einsum('nij,j->ni', R, TOOL), R7

    def _tr_body(Q, R7):
        """닭 머리 오차용 trace.

        ★ body 모드 ★ 목표 방향 = base 방향을 j1 만큼 돌린 것.
        "팔이 도는 만큼 주둥이도 같이 돈다(닭이 몸을 돌리듯). 손목은 안 쓴다."
        예전(world 모드)에는 방향을 월드에 못박아 -Y 만 보게 했다. 그러면
        팔이 돌 때마다 j5(손목)로 되돌려야 해서 손목을 90도씩 썼고
        (실측: 옛 표는 예외 없이 j1+j5=0), 뒤쪽 공간이 통째로 막혔다.
        여기서는 Rz(j1)^T @ R7 의 trace 를 쓴다."""
        a = J1S*Q[:,0]
        ca, sa = torch.cos(a), torch.sin(a)
        return ca*(R7[:,0,0]+R7[:,1,1]) + sa*(R7[:,1,0]-R7[:,0,1]) + R7[:,2,2]

    def solve(T, Qinit=None):
        N = T.shape[0]
        if Qinit is not None:
            Q = Qinit.clone()
        else:
            # j1 을 목표 방향으로 미리 돌려둔다. 여기서 출발해야 잘 풀린다.
            Q = torch.zeros(N, NJ, device=dev)
            Tn = T.detach().cpu().numpy()
            a0 = np.arctan2(Tn[:,1], Tn[:,0]) - J1REF
            a0 = (a0 + np.pi) % (2*np.pi) - np.pi
            Q[:,0] = torch.tensor(a0/J1S, dtype=torch.float32, device=dev)
        Q.requires_grad_(True)
        opt = torch.optim.Adam([Q], lr=0.05)
        for _ in range(ITERS):
            opt.zero_grad()
            P, R7 = fk(Q)
            e_pos = ((P - T)**2).sum(1)
            tr = _tr_body(Q, R7)
            loss = (e_pos*300.0 + (3.0-tr)*60.0).sum()
            loss.backward(); opt.step()
            with torch.no_grad():
                Q.data[:, NJ-1] = 0.0                # end 고정
                Q.data = torch.max(torch.min(Q.data, LIM[:,1]), LIM[:,0])
        with torch.no_grad():
            P, R7 = fk(Q)
            perr = (P-T).norm(dim=1)
            tr = _tr_body(Q, R7).clamp(-1,3)
            rerr = torch.acos(((tr-1)/2).clamp(-1,1))*180/math.pi
        return Q.detach(), perr, rerr

    # ---- 만들 칸 목록 ----
    g = m._HEAD_GRID[0]
    p0 = np.array([float(v) for v in m.tool_pos([0.0]*NJ)])
    k0 = tuple(int(round(p0[i]/g)) for i in range(3))
    R = int(round(rng_mm/1000.0/g))
    # 이어서 하기: 저장은 npz 로 하므로 npz 를 읽어야 한다.
    # (예전엔 head_cache_load() 로 JSON 만 봐서, 이어서 돌리면 0칸으로
    #  보고 전부 다시 풀었다)
    m._HEAD_CACHE.clear()
    have = 0
    done = set()
    if not fresh:
        have = m.head_load_any()
        c = m._HEAD_ARR['code']
        if c is not None and len(c):
            OFF, MM = m._HK_OFF, m._HK_M
            _kz = (c % MM) - OFF; _r = c // MM
            _ky = (_r % MM) - OFF; _kx = (_r // MM) - OFF
            done = set(zip(_kx.tolist(), _ky.tolist(), _kz.tolist()))

    todo = []
    for kz in range(k0[2]-R, k0[2]+R+1):
        if kz*g < m.PLATE_TOP+0.02: continue
        for ky in range(k0[1]-R, k0[1]+R+1):
            for kx in range(k0[0]-R, k0[0]+R+1):
                if (kx,ky,kz) not in done:
                    todo.append((kx,ky,kz))

    def _save():
        """이진(npz)으로 저장한다.

        JSON 은 2,337만칸에서 쓰기 192초 / 1.8GB (실측). 빌더는 중간에도
        여러 번 저장하므로 그대로 두면 계산 시간을 통째로 잡아먹는다.
        npz 는 같은 양이 3초 / 0.9GB — 64배 빠르다.
        head_npz_merge() 는 _HEAD_CACHE 의 새 칸만 기존 표에 합치고
        dict 를 비우므로, 메모리도 계속 늘지 않는다."""
        n = m.head_npz_merge()
        if not n:                      # 새로 푼 칸이 없으면 merge 가 0 을 준다
            c = m._HEAD_ARR['code']
            n = len(c) if c is not None else 0
        return n

    print()
    print('='*62)
    print(f'  로봇 : {os.path.basename(m.BASE_URDF)}')
    print(f'  GPU  : {torch.cuda.get_device_name(0)}')
    print(f'  중심 : {[int(v*1000) for v in p0]}mm · 범위 ±{rng_mm:.0f}mm')
    print(f'  기존 : {have:,}칸 · 새로 풀 칸 {len(todo):,}개')
    print('='*62)
    if not todo:
        print('이미 다 만들어져 있습니다.'); return

    t0 = time.time(); made = 0
    for i in range(0, len(todo), BATCH):
        chunk = todo[i:i+BATCH]
        T = torch.tensor(np.array(chunk, dtype=np.float32)*g,
                         dtype=torch.float32, device=dev)
        Q, perr, rerr = solve(T)
        good = ((perr < POS_TOL) & (rerr < ROT_TOL)).cpu().numpy()
        Qc = Q.cpu().numpy()
        for j, kk in enumerate(chunk):
            if good[j]:
                m._HEAD_CACHE[kk] = [round(float(x),4) for x in Qc[j]]
                made += 1
            # 못 푼 칸은 넣지 않는다. npz 로 저장하면 dict 가 비워지므로
            # None 표시는 어차피 남지 않는다 (메모리만 먹는다).
        el = time.time()-t0
        frac = min(1.0, (i+len(chunk))/len(todo))
        print(f'   {frac*100:>3.0f}%  ·  새로 {made:,}칸  ·  '
              f'남은 시간 약 {(el/frac-el)/60:.0f}분   ', end='\r', flush=True)
        if (i//BATCH) % 4 == 3: _save()
        if limit_h and el > limit_h*3600:
            print()
            print(f'   정해둔 {limit_h:.1f}시간이 지나 여기까지만 저장합니다.')
            break

    n = _save()
    print()
    print('='*62)
    print(f'  완료 — 표 {n:,}칸 (이번에 {made:,}칸 추가) · {(time.time()-t0)/60:.1f}분')
    print(f'  저장 : {m._head_npz_file()}')
    print('  * 앱을 다시 켜면 자동으로 불러옵니다')
    print('='*62)


if __name__ == '__main__':
    main()
