"""카메라 판(2×4 타일) -> 앱 바닥판 맞춤 (2026-11-03).

★ 바닥판은 고정 (사용자) ★ 앱 기본 바닥판(arm_studio_web.PLATE_LAYOUT, 2×4) = 카메라 판 창(목심 BI×BJ = 20×40) 전체.
  판 창의 긴 쪽이 앱 배치의 긴 쪽에 오는 두 방향(그대로 / 180°)만 된다. 2×4 체크무늬는 180° 돌려도 똑같아 보이므로
  둘 중 '앱에 지금 있는 로봇·도킹 자리' 와 더 맞는 쪽을 고른다 (= 이전 자리 유지).
  예전(3×3 판)엔 4방향 × 타일 이동을 다 따져 앱 판을 더하기도 했다 — 이제 판은 늘 그 8장이라 더하지 않는다.

앱(arm_studio_web)을 불러오지 않고 시험할 수 있게 계산만 여기 둔다.
"""
import math

import numpy as np


def rz(k):
    """z 축 90°×k 회전 (3×3)."""
    a = k * math.pi / 2
    c, s = round(math.cos(a)), round(math.sin(a))
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def whole_board(lay, ni, nj, tile=10):
    """판이 다 보였나 — 판 창이 판 가장자리와 맞는다고 믿을 수 있나.
    구멍은 판 위에만 있으므로, 양 끝 줄 타일이 다 보였으면 창이 판에서 미끄러질 수 없다."""
    tiles = {tuple(t) for t in (lay.get('tiles') or [])}
    us = {u for u, _ in tiles}
    vs = {v for _, v in tiles}
    return len(tiles) >= 5 and {0, ni // tile - 1} <= us and {0, nj // tile - 1} <= vs


def fit(lay, layout, tile_origin, pitch, plate_size, plate_top, foot, fwd, app_docks):
    """카메라 판 -> 앱 월드 (R, t).

    lay        livecam.layout() — 'size'(BI,BJ) · 'tiles' · 'robot'(칸) · 'dockings'(칸 'center')
    layout     앱 기본 바닥판 칸 [(ix, iy)]
    tile_origin(t) -> 그 칸 판의 왼쪽 아래 (x, y) m
    foot       앱 로봇 받침 (x, y) m,  fwd = 로봇 팔이 뻗는 방향 (단위벡터)
    app_docks  앱 도킹 자리 [(x, y)] m
    반환 {'R','t','k','cons','face'} 또는 None (판이 덜 보였거나, 방향을 가를 로봇·도킹이 안 보임 = 기다림)."""
    if not lay:
        return None
    ni, nj = lay.get('size') or (20, 40)
    tile = int(round(plate_size / pitch))
    if not whole_board(lay, ni, nj, tile):
        return None
    rob = lay.get('robot')
    docks = list(lay.get('dockings') or [])
    if rob is None and not docks:
        return None
    xs = [t[0] for t in layout]
    ys = [t[1] for t in layout]
    nx, ny = max(xs) - min(xs) + 1, max(ys) - min(ys) + 1
    nu, nv = ni // tile, nj // tile
    if (nx, ny) == (nu, nv):
        ks = (0, 2)                     # 판 i -> 앱 x
    elif (nx, ny) == (nv, nu):
        ks = (1, 3)                     # 판 i -> 앱 y
    else:
        return None                     # 앱 바닥판 배치와 카메라 판 크기가 다르다
    x0, y0 = tile_origin((min(xs), min(ys)))
    g = np.array([x0 + pitch / 2, y0 + pitch / 2])       # 앱 바닥판 첫 목심
    ext = np.array([[a * pitch, b * pitch, 0.0] for a in (0, ni - 1) for b in (0, nj - 1)])
    foot = np.asarray(foot, float)[:2]
    fwd = np.asarray(fwd, float)[:2]
    best = None
    for k in ks:
        R = rz(k)
        mn = (ext @ R.T)[:, :2].min(0)
        t = np.array([g[0] - mn[0], g[1] - mn[1], plate_top])
        cons = 0.0                       # 이전 자리와 얼마나 다른가 (로봇 받침·도킹, 각 40cm 까지만)
        if rob is not None:
            w = R @ np.array([rob[0] * pitch, rob[1] * pitch, 0.0]) + t
            cons += min(0.4, math.hypot(w[0] - foot[0], w[1] - foot[1]))
        if app_docks:
            for d in docks:
                w = R @ np.array([d['center'][0] * pitch, d['center'][1] * pitch, 0.0]) + t
                cons += min(0.4, min(math.hypot(w[0] - x, w[1] - y) for x, y in app_docks))
        cen = R @ np.array([(ni - 1) / 2 * pitch, (nj - 1) / 2 * pitch, 0.0]) + t
        inward = cen[:2] - foot
        face = float(np.dot(inward / (np.linalg.norm(inward) + 1e-9), fwd))
        key = (-round(cons, 3), round(face, 2))
        if best is None or key > best[0]:
            best = (key, R, t, k, cons, face)
    _, R, t, k, cons, face = best
    return {'R': R, 't': t, 'k': k, 'cons': cons, 'face': face}
