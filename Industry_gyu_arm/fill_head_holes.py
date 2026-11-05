#!/usr/bin/env python3
"""닭 머리 표의 '구멍' 메우기

GPU 판은 0 자세에서 출발해 푼다. 대부분 잘 풀리지만, 사방이 뚫려 있는데
그 칸 하나만 못 푼 곳이 생긴다(측정: 3,471칸). 조종 중 그 칸을 지나면
표에 없다고 판단해 서버 IK 로 떨어지고, 거기서 뚝 끊긴다.

여기서는 '이웃 칸의 자세' 에서 출발해 다시 푼다.
이웃은 이미 정답이므로 몇 발짝만 가면 되고, 훨씬 잘 수렴한다.
채워질 게 없을 때까지 반복한다(한 겹 채우면 그 옆도 이웃이 생긴다).
"""
import os, sys, time, math
import numpy as np

ITERS = 1200           # 이웃에서 출발하니 오래 돌려도 싸다
POS_TOL = 0.006
ROT_TOL = 2.0
NB = [(1,0,0),(-1,0,0),(0,1,0),(0,-1,0),(0,0,1),(0,0,-1)]


def main():
    if '--help' in sys.argv or '-h' in sys.argv:
        print(__doc__); return
    # 이웃 몇 개가 채워져 있으면 '구멍' 으로 볼 것인가.
    # 3 이 기본 — 표 가장자리나 모서리는 원래 이웃이 3개뿐이라 정상이다.
    # 4 로 하면 한가운데 뚫린 것만 메우고 가장자리는 놓친다(실측: 29,724칸 놓침).
    min_nb = int(sys.argv[1]) if len(sys.argv) > 1 else 3
    print('앱 로직 로드 중... (20초쯤)')
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import torch
    import arm_studio_web as m
    if not torch.cuda.is_available():
        print('GPU 를 못 찾았습니다.'); return
    dev = torch.device('cuda'); NJ = m.NJOINT

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
    J1S = -1.0 if np.asarray(m.BASE_DEF[m.JOINTS[0]]['axis'], float)[2] < 0 else 1.0
    LIM = torch.tensor(np.array([m.jr_limits(j) for j in range(NJ)]),
                       dtype=torch.float32, device=dev)

    def fk(Q):
        N = Q.shape[0]
        R = torch.eye(3, device=dev).expand(N,3,3).contiguous()
        P = torch.zeros(N,3, device=dev); R7 = None
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
        """body 모드: 목표 방향 = base 방향을 j1 만큼 돌린 것.
        (팔이 도는 만큼 주둥이도 같이 돈다 — 손목은 안 쓴다)"""
        a = J1S*Q[:,0]
        ca, sa = torch.cos(a), torch.sin(a)
        return ca*(R7[:,0,0]+R7[:,1,1]) + sa*(R7[:,1,0]-R7[:,0,1]) + R7[:,2,2]

    def solve(T, Qinit):
        Q = Qinit.clone().requires_grad_(True)
        # 이웃에서 출발하므로 보폭을 작게 — 멀리 튀지 않게
        opt = torch.optim.Adam([Q], lr=0.01)
        for _ in range(ITERS):
            opt.zero_grad()
            P, R7 = fk(Q)
            e_pos = ((P - T)**2).sum(1)
            tr = _tr_body(Q, R7)
            loss = (e_pos*300.0 + (3.0-tr)*60.0).sum()
            loss.backward(); opt.step()
            with torch.no_grad():
                Q.data[:, NJ-1] = 0.0
                Q.data = torch.max(torch.min(Q.data, LIM[:,1]), LIM[:,0])
        with torch.no_grad():
            P, R7 = fk(Q)
            perr = (P-T).norm(dim=1)
            tr = _tr_body(Q, R7).clamp(-1,3)
            rerr = torch.acos(((tr-1)/2).clamp(-1,1))*180/math.pi
        return Q.detach(), perr, rerr

    # 이진 표(npz)를 읽고 dict 로 펼친다. 구멍 찾기는 dict 가 편하다.
    have = m.head_load_any()
    g = m._HEAD_GRID[0]
    if not have:
        print('표가 없습니다. 먼저 build_head_gpu.sh 로 만드세요.'); return
    code = m._HEAD_ARR['code']; val = m._HEAD_ARR['val']
    OFF, MM = m._HK_OFF, m._HK_M
    kz = (code % MM) - OFF; rr = code // MM
    ky = (rr % MM) - OFF;   kx = (rr // MM) - OFF
    C = {(int(a), int(b), int(c)): v
         for a, b, c, v in zip(kx.tolist(), ky.tolist(), kz.tolist(),
                               val.tolist())}
    print(f'표 {have:,}칸 불러옴 · grid {g*1000:.0f}mm')

    total_made = 0
    rounds = int(sys.argv[2]) if len(sys.argv) > 2 else 40
    for rnd in range(1, rounds + 1):
        # 구멍 찾기: 값이 없는데 이웃이 min_nb 개 이상 '진짜 값' 인 칸
        good = {k for k, v in C.items() if v}
        holes = {}
        for k in good:
            for d in NB:
                n = (k[0]+d[0], k[1]+d[1], k[2]+d[2])
                if C.get(n): continue
                holes.setdefault(n, []).append(k)
        holes = {k: v for k, v in holes.items() if len(v) >= min_nb}
        if not holes:
            print(f'{rnd}회차 — 메울 구멍이 없습니다.'); break
        print(f'{rnd}회차 — 구멍 {len(holes):,}칸 (이웃 {min_nb}개 이상)')

        keys = list(holes.keys())
        T = torch.tensor(np.array(keys, dtype=np.float32)*g,
                         dtype=torch.float32, device=dev)
        # 출발 자세 = 이웃들 자세의 평균
        Qi = np.array([np.mean([C[n] for n in holes[k]], axis=0) for k in keys],
                      dtype=np.float32)
        Qi = torch.tensor(Qi, device=dev)
        t0 = time.time()
        Q, perr, rerr = solve(T, Qi)
        ok = ((perr < POS_TOL) & (rerr < ROT_TOL)).cpu().numpy()
        Qc = Q.cpu().numpy(); made = 0
        for j, kk in enumerate(keys):
            if ok[j]:
                C[kk] = [round(float(x), 4) for x in Qc[j]]; made += 1
        total_made += made
        print(f'   메움 {made:,}/{len(keys):,}칸 '
              f'({made*100.0/max(1,len(keys)):.0f}%) · {time.time()-t0:.0f}초')
        if made:
            # 새로 푼 칸만 _HEAD_CACHE 에 넣고 이진 표에 합쳐 저장한다
            m._HEAD_CACHE.clear()
            for j, kk in enumerate(keys):
                if ok[j]: m._HEAD_CACHE[kk] = list(C[kk])
            m.head_npz_merge()
        if made == 0: break
        if made < len(keys) * 0.02:      # 2% 밑이면 남은 건 진짜 못 가는 자리
            print('   (성공률이 바닥 — 남은 칸은 도달 불가로 봅니다)')
            break

    n = len(m._HEAD_ARR['code']) if m._HEAD_ARR['code'] is not None else 0
    print('='*62)
    print(f'  완료 — 표 {n:,}칸 (구멍 {total_made:,}칸 메움)')
    print(f'  저장 : {m._head_npz_file()}')
    print('='*62)


if __name__ == '__main__':
    main()
