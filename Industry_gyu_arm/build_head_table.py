#!/usr/bin/env python3
"""닭 머리 표 만들기 — 조이콘 조종용 (한 번 만들면 계속 씀)

왜 필요한가:
  조이콘으로 움직일 때 매번 서버에 IK 를 물으면 왕복(40ms)이 곧 화면
  속도가 되어 끊긴다. 미리 풀어 둔 표를 브라우저에 주면 왕복 없이
  60Hz 로 즉시 그린다 (공 굴리기와 같은 구조).

왜 병렬이 되나:
  ik_head 는 씨앗(초기 자세)에 따라 해가 달라지는데, '이웃 칸의 표 값'을
  씨앗으로 주면 순차로 이어 푼 것과 같은 해가 나온다 (40/40 동일 확인).
  그래서 z 층별로 나눠 여러 코어가 동시에 풀 수 있다.

  ./build_head_table.sh            (기본 ±300mm)
  ./build_head_table.sh 500        ±500mm
  중단해도 지금까지 만든 건 저장되고, 다시 실행하면 이어서 한다.
"""
import os, sys, time, json, math, signal
import numpy as np
from multiprocessing import Pool, cpu_count

RANGE_MM = 300.0          # 기본 범위 (base 툴 위치 기준 ±)
_W = {}
_STOP = [False]


def _init(urdf):
    """워커마다 앱 로직을 한 번씩 올린다."""
    os.environ['ARM_BASE_URDF'] = urdf
    os.environ['ARM_NO_VIEWER'] = '1'
    import arm_studio_web as m
    _W['m'] = m


def _layer(args):
    """z 층 하나를 푼다. 층 안에서는 순차(이웃 씨앗)로 이어 푼다."""
    kz, k0, R, g, seed0 = args
    m = _W['m']
    out = []
    seed = list(seed0)
    for ky in range(k0[1] - R, k0[1] + R + 1):
        for kx in range(k0[0] - R, k0[0] + R + 1):
            pt = (kx * g, ky * g, kz * g)
            if pt[2] < m.PLATE_TOP + 0.02:
                continue
            q = m.ik_head(pt, [seed])
            if q is None:
                continue
            seed = q
            out.append((kx, ky, kz, [round(float(x), 4) for x in q]))
    return out


def main():
    rng = float(sys.argv[1]) if len(sys.argv) > 1 else RANGE_MM
    fresh = '--fresh' in sys.argv
    print('앱 로직 로드 중... (20초쯤)')
    os.environ['ARM_NO_VIEWER'] = '1'
    import arm_studio_web as m

    g = m._HEAD_GRID[0]
    p0 = m.tool_pos([0.0] * m.NJOINT)          # base 자세의 툴 위치가 중심
    k0 = tuple(int(round(float(p0[i]) / g)) for i in range(3))
    R = int(round(rng / 1000.0 / g))
    tot = (2 * R + 1) ** 3

    have = 0 if fresh else m.head_cache_load()
    print()
    print('=' * 62)
    print(f'  로봇 : {os.path.basename(m.BASE_URDF)}')
    print(f'  중심 : {[int(v*1000) for v in p0]}mm · 범위 ±{rng:.0f}mm')
    print(f'  격자 : {g*1000:.0f}mm · 칸 {tot:,}개')
    print(f'  코어 : {cpu_count()}개')
    if have:
        print(f'  기존 : {have:,}칸 (이어서 만듭니다)')
    print('=' * 62)

    zs = [kz for kz in range(k0[2] - R, k0[2] + R + 1)]
    seed0 = [0.0] * m.NJOINT
    t0 = time.time()
    done = 0

    def save():
        n = m.head_cache_save()
        return n

    signal.signal(signal.SIGINT, lambda *_: (_STOP.__setitem__(0, True),
                                             print('\n중단 요청 — 저장하고 끝냅니다...')))
    with Pool(processes=max(1, cpu_count() - 2),
              initializer=_init, initargs=(m.BASE_URDF,)) as pool:
        args = [(kz, k0, R, g, seed0) for kz in zs]
        for cells in pool.imap_unordered(_layer, args):
            for kx, ky, kz, q in cells:
                m._HEAD_CACHE[(kx, ky, kz)] = q
            done += 1
            el = time.time() - t0
            frac = done / len(zs)
            print(f'   {frac*100:>3.0f}%  ·  {len(m._HEAD_CACHE):,}칸  ·  '
                  f'남은 시간 약 {(el/frac-el)/60:.0f}분   ', end='\r', flush=True)
            if done % 8 == 0:
                save()
            if _STOP[0]:
                pool.terminate()
                break

    n = save()
    print()
    print('=' * 62)
    print(f'  완료 — {n:,}칸  ({(time.time()-t0)/60:.1f}분)')
    print(f'  저장 : {m._head_cache_file()}')
    print('  * 앱을 다시 켜면 자동으로 불러옵니다')
    print('=' * 62)


if __name__ == '__main__':
    main()
