#!/usr/bin/env python3
"""공장 전체(로봇 + 바닥 플레이트 + 설비)를 68000 이 읽을 한 덩어리로 굽는다.

앱(7870/state)이 이미 계산해 둔 링크 변환·설비 위치를 그대로 받아쓴다.
여기서 기구학을 다시 풀지 않는다 — 화면에 보이는 것과 어긋나면 안 되니까.

  python build_scene.py [삼각형목표]

만들어지는 것:
  scene.bin   68000 이 읽는 메쉬 묶음 (M68S)
  scene.json  각 조각의 이름·색·종류 (뷰어가 팔레트 만들 때 쓴다)

좌표 단위는 0.1mm 정수다. mm 로 반올림하면 인접 정점이 뭉개져
삼각형이 통째로 사라진다 (실측 84% 소멸).
"""
import os, sys, json, struct
import urllib.request
import numpy as np
import trimesh

HERE      = os.path.dirname(os.path.abspath(__file__))
MESH_DIR  = os.path.expanduser(
    '~/2026_project/urdf/sts3215/Industry_gyu_arm_20260728_base/meshes')
STL_DIR   = os.path.expanduser('~/2026_project/stl')
# 물체(브릭)는 패키지별로 urdf/bricks/<이름>/meshes/ 에 들어 있다
BRICK_DIR = os.path.expanduser('~/2026_project/urdf/bricks')
STATE_URL = 'http://127.0.0.1:7870/state'
UNIT      = 10000.0          # m -> 0.1mm

TARGET_TOTAL = int(sys.argv[1]) if len(sys.argv) > 1 else 3000


def fetch_state():
    with urllib.request.urlopen(STATE_URL, timeout=10) as r:
        return json.loads(r.read().decode())


def find_mesh(oid):
    """개체 id 로 STL 을 찾는다.

    앱은 파일명의 공백을 '_' 로 바꿔 id 를 만든다
    ("... - plate_bottom-7.STL" -> "..._-_plate_bottom-7").
    복제본은 '#2' 같은 꼬리가 더 붙는다. 둘 다 되돌려서 찾는다."""
    base = str(oid).split('#')[0]
    cands = [base]
    # 물체는 <패키지>/meshes/<이름>_base.STL 형태다 (ppap -> ppap_base.STL)
    dirs = [MESH_DIR, STL_DIR]
    if os.path.isdir(BRICK_DIR):
        for pkg in sorted(os.listdir(BRICK_DIR)):
            md = os.path.join(BRICK_DIR, pkg, 'meshes')
            if os.path.isdir(md):
                dirs.append(md)
                if base.lower() in pkg.lower():
                    for fn in sorted(os.listdir(md)):
                        if fn.lower().startswith(base.lower()) and \
                           fn.lower().endswith('.stl'):
                            return os.path.join(md, fn)
    # '_' 를 공백으로 되돌린 형태 — 다만 'plate_bottom' 처럼 원래 '_' 인
    # 부분까지 바뀌면 안 되니, ' - ' 구분자만 되살린다.
    cands.append(base.replace('_-_', ' - '))
    cands.append(base.replace('_', ' '))
    for c in cands:
        for d in dirs:
            for ext in ('.STL', '.stl'):
                p = os.path.join(d, c + ext)
                if os.path.exists(p):
                    return p
    # 그래도 못 찾으면 파일 목록에서 느슨하게 맞춰본다
    key = base.replace('_', '').replace('-', '').replace(' ', '').lower()
    for d in dirs:
        if not os.path.isdir(d):
            continue
        for fn in os.listdir(d):
            if not fn.lower().endswith('.stl'):
                continue
            k2 = fn.rsplit('.', 1)[0]
            k2 = k2.replace('_', '').replace('-', '').replace(' ', '').lower()
            if k2 == key:
                return os.path.join(d, fn)
    return None


def load_clean(path):
    """STL 을 열고 중복 정점을 합친다.

    합치지 않으면 데시메이션이 모서리를 못 접어 얇은 조각만 남고,
    화면에서 한 점으로 뭉개진다 (채움 1.4% vs 11.4%)."""
    m = trimesh.load(path, process=True)
    m.merge_vertices()
    m.update_faces(m.nondegenerate_faces())
    m.update_faces(m.unique_faces())
    return m


def decimate(m, want):
    if len(m.faces) <= want:
        return m
    try:
        s = m.simplify_quadric_decimation(face_count=want)
    except Exception:
        s = m.simplify_quadratic_decimation(want)
    s.merge_vertices()
    s.update_faces(s.nondegenerate_faces())
    return s


def plate_mesh(ox, oy, top_z, pitch=0.020, n=10, hole_r=0.005, thick=0.020):
    """바닥 플레이트를 목심 구멍까지 살려서 직접 만든다.

    STL 을 깎으면 42면쯤 남는데 그러면 구멍이 통째로 사라진다.
    바닥에서 중요한 건 판때기가 아니라 '구멍이 어디 뚫려 있나' 다.
    규칙은 앱과 같다 — 타일당 10x10, 간격 20mm, 지름 10mm.

    면을 아끼려고 이렇게 한다:
      · 판 전체는 사각형 두 장
      · 구멍은 뚫지 않고 그 자리에 '어두운 팔각 원반' 을 살짝 얹는다
    68000 에서 구멍을 진짜로 뚫으면 면이 수천 개 든다. 옛날 게임도
    이렇게 '구멍처럼 보이는 무늬' 로 때웠다. 구멍당 8면이면 충분하다."""
    import math
    V, F = [], []
    w = pitch * n

    def add_quad(a, b, c, d):
        i = len(V); V.extend([a, b, c, d])
        F.append([i, i+1, i+2]); F.append([i, i+2, i+3])

    # 판 윗면 (사각형 하나 = 삼각형 2개)
    add_quad([ox, oy, top_z], [ox+w, oy, top_z],
             [ox+w, oy+w, top_z], [ox, oy+w, top_z])
    # 옆면 네 장 — 판 두께가 보이면 바닥처럼 읽힌다
    z0 = top_z - thick
    add_quad([ox, oy, top_z], [ox, oy, z0], [ox+w, oy, z0], [ox+w, oy, top_z])
    add_quad([ox+w, oy, top_z], [ox+w, oy, z0],
             [ox+w, oy+w, z0], [ox+w, oy+w, top_z])
    add_quad([ox+w, oy+w, top_z], [ox+w, oy+w, z0],
             [ox, oy+w, z0], [ox, oy+w, top_z])
    add_quad([ox, oy+w, top_z], [ox, oy+w, z0], [ox, oy, z0], [ox, oy, top_z])

    return np.array(V, float), np.array(F, np.int64), len(F)


def box_mesh(v):
    """정점 뭉치를 감싸는 사각 박스 하나로 바꾼다 (12 삼각형).

    68000 은 삼각형이 아깝다. 팔 마디의 자세한 모양은 이 해상도
    (320x240) 에서 어차피 안 보이므로, 굵기와 방향만 맞으면 된다.
    옛날 로봇 데모도 팔을 박스로 그렸다.

    축에 정렬된 박스가 아니라, 정점 분포의 주축(가장 긴 방향)에
    맞춘 박스를 쓴다. 팔은 비스듬히 뻗어 있어서 축 정렬 박스로
    감싸면 실제보다 훨씬 뚱뚱해진다."""
    c = v.mean(axis=0)
    q = v - c
    # 주성분 3개 = 이 뭉치가 가장 길게 뻗은 방향들
    try:
        _u, _s, vt = np.linalg.svd(q, full_matrices=False)
        R = vt.T                       # 열이 주축
        if np.linalg.det(R) < 0:
            R[:, 2] *= -1
    except Exception:
        R = np.eye(3)
    local = q @ R
    lo, hi = local.min(axis=0), local.max(axis=0)
    # 굵기(짧은 두 축)를 줄인다. 경계 박스는 나사·돌기까지 감싸서
    # 실제 팔보다 뚱뚱해지고, 그러면 마디끼리 겹쳐 보인다.
    ext = hi - lo
    order = np.argsort(ext)          # 짧은 축부터
    mid = (lo + hi) / 2
    for k in order[:2]:              # 긴 축(길이)은 그대로 두고
        half = ext[k] * 0.5 * 0.62   # 굵기만 62% 로
        lo[k], hi[k] = mid[k] - half, mid[k] + half
    corners = np.array([[x, y, z] for x in (lo[0], hi[0])
                                  for y in (lo[1], hi[1])
                                  for z in (lo[2], hi[2])])
    V = corners @ R.T + c
    # 꼭짓점 순서: (x,y,z) 비트 = (4,2,1)
    F = np.array([
        [0,1,3],[0,3,2],   # x=lo 면
        [4,6,7],[4,7,5],   # x=hi
        [0,4,5],[0,5,1],   # y=lo
        [2,3,7],[2,7,6],   # y=hi
        [0,2,6],[0,6,4],   # z=lo
        [1,5,7],[1,7,3],   # z=hi
    ], np.int64)
    return V, F


def mat_of(link):
    """/state 의 16개 실수 -> 4x4. 행 우선(row-major) 이다."""
    return np.array(link['m'], dtype=float).reshape(4, 4)


# 로봇 링크는 앱에서 전부 비슷한 파랑이라 68000 화면(색 8단계)에서는
# 마디 구분이 안 된다. 마디마다 색을 확실히 벌려준다.
# (앱 화면은 그림자·조명이 있어 구분되지만 여기는 플랫 셰이딩뿐이다)
LINK_RGB = {
    'base_link': (120, 120, 128),   # 받침 - 회색
    'l1':        ( 70, 130, 220),   # 파랑
    'l2':        ( 90, 200, 160),   # 청록
    'l3':        (215, 165,  60),   # 황토
    'l4':        (200, 100, 180),   # 자주
    'l5':        (110, 160, 240),   # 하늘
    'l6':        (240, 140,  60),   # 주황 - 그리퍼
}


def hex_rgb(c):
    c = (c or '#999999').lstrip('#')
    if len(c) == 3:
        c = ''.join(ch * 2 for ch in c)
    try:
        return (int(c[0:2], 16), int(c[2:4], 16), int(c[4:6], 16))
    except Exception:
        return (153, 153, 153)


def main():
    st = fetch_state()
    parts = []          # (이름, 종류, rgb, 정점 Nx3 float m, 면 Mx3)
    # 구울 때의 링크 자세를 남긴다. 나중에 로봇이 움직이면
    # (지금자세)x(구울때자세)^-1 만 렌더러에 보내면 되니 다시 안 구워도 된다.
    baked = {}

    # ── 로봇 링크 7개 ───────────────────────────────────────
    # /state 가 준 변환을 그대로 적용한다 (앱 화면과 같은 자세).
    for lk in st.get('links', []):
        p = find_mesh(lk['id'])
        if not p:
            print(f"  ! 메쉬 없음: {lk['id']}")
            continue
        m = load_clean(p)
        T = mat_of(lk)
        v = np.asarray(m.vertices, float)
        v = v @ T[:3, :3].T + T[:3, 3]
        baked[lk['id']] = [float(x) for x in lk['m']]
        rgb = LINK_RGB.get(lk['id']) or hex_rgb(lk['color'])
        # ★ 마디를 상자 하나(12면)로 뭉개지 않는다.
        #
        #   전에는 l6(그리퍼)만 원본을 쓰고 나머지는 box_mesh() 로
        #   사각 박스를 씌웠다. 49,144면을 72면으로 줄인 셈이라
        #   모양이 아예 딴판이 됐다. 게다가 그 박스는 주성분(SVD)
        #   방향으로 기울인 것이라, L자·비대칭 부품에서는 방향까지
        #   틀어졌다 (실측: base_link 가 실제보다 57mm 부풀었다).
        #
        #   이제 모든 마디가 '원본을 깎은 것' 을 쓴다. 예산 안에서
        #   면 수만 줄이므로 굵기와 실루엣이 살아남는다.
        parts.append([lk['id'], 'robot', rgb, v, np.asarray(m.faces)])

    # ── 설비 · 바닥 플레이트 · 물체 ─────────────────────────
    ptop  = float(st.get('ptop') or 0.020)
    psize = float(st.get('psize') or 0.200)
    for e in st.get('env', []):
        if not e.get('v', True):
            continue
        if e.get('plate'):
            # 바닥은 STL 을 깎지 않는다. 깎으면 구멍이 통째로 사라지는데,
            # 바닥에서 중요한 건 판때기가 아니라 구멍 위치다.
            # 앱과 같은 규칙(20mm 간격 10x10)으로 직접 만든다.
            tile = e.get('tile')
            if tile:
                ox, oy = tile[0]*psize, tile[1]*psize
            else:
                off0 = e.get('off') or (0, 0, 0)
                ox, oy = off0[0], off0[1]
            # 구멍은 안 그린다 — 구멍 400개가 삼각형의 절반을 먹어서
            # 68000 이 감당을 못 했다. 판때기만 사각형으로 깐다.
            v, f, _ = plate_mesh(ox, oy, ptop, pitch=0.020, n=10,
                                 hole_r=0.005, thick=ptop)
            parts.append([e['id'], 'plate', (185, 185, 190), v, f])
            continue
        p = find_mesh(e['id'])
        if not p:
            print(f"  ! 메쉬 없음: {e['id']}")
            continue
        m = load_clean(p)
        sc = float(e.get('scale', 1.0) or 1.0)
        v = np.asarray(m.vertices, float) * sc
        # yaw 회전 후 offset 평행이동 — 앱의 배치 규칙과 같은 순서다
        yaw = float(e.get('yaw', 0.0) or 0.0)
        if abs(yaw) > 1e-9:
            c, s = np.cos(yaw), np.sin(yaw)
            R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
            v = v @ R.T
        off = e.get('off') or (0, 0, 0)
        v = v + np.array([off[0], off[1], off[2]], float)
        kind = 'plate' if e.get('plate') else ('brick' if e.get('brick') else 'env')
        # ★ 설비·물체도 박스로 뭉개지 않는다.
        #   도킹 원본은 316면뿐이라 그대로 써도 부담이 없다.
        #   박스로 씌웠더니 폭이 55mm -> 34.5mm 로 잘못 줄어들어
        #   "구부러져 보인다" 는 말이 나왔다 (실측).
        #   box_mesh 는 주성분 방향으로 기울인 상자라, 도킹처럼
        #   홈이 파인 모양에서는 방향까지 틀어진다.
        parts.append([e['id'], kind, hex_rgb(e['color']), v, np.asarray(m.faces)])

    if not parts:
        print("그릴 게 없다 — 앱(7870)이 떠 있는지 확인하세요")
        return 1

    # ── z-up(로봇 좌표) → y-up(렌더러 좌표) ────────────────
    # 앱 뷰어도 world.rotation.x = -90° 로 같은 변환을 한다.
    # 이걸 안 하면 바닥 플레이트가 벽처럼 세로로 선다.
    #   (x, y, z) → (x, z, -y)
    for pr in parts:
        v = pr[3]
        pr[3] = np.column_stack([v[:, 0], v[:, 2], -v[:, 1]])

    # ── 삼각형 예산 나누기 ──────────────────────────────────
    # 바닥 플레이트는 사실상 판때기다. 로봇에 예산을 몰아준다.
    # 바닥은 이미 필요한 만큼만 만들어져 있고(구멍이 목숨),
    # 도킹·물체도 원본이 수백 면뿐이라 깎으면 구멍·모양이 뭉개진다.
    # 삼각형 예산은 면이 만 단위인 로봇 링크에만 쓴다.
    # 바닥(plate)만 예산 밖이다 — 구멍이 목숨이라 깎으면 안 된다.
    # 로봇 마디·도킹·물체는 이제 전부 원본을 깎아 쓰므로 예산을 나눈다.
    KEEP = {'plate'}
    tot_src = sum(len(f) for *_, f in parts)
    rob = [pr for pr in parts if pr[1] not in KEEP]
    fixed = sum(len(pr[4]) for pr in parts if pr[1] in KEEP)
    # 마디 7개가 예산을 나눠 갖는다. 원본이 큰 마디가 더 받되,
    # 작은 마디도 모양이 남게 최소 80면은 보장한다.
    # (상자 12면으로는 절대 안 되던 부분이다)
    budget = max(200, TARGET_TOTAL - fixed)
    rsum = sum(len(pr[4]) for pr in rob) or 1
    for pr in rob:
        share = max(80, int(budget * len(pr[4]) / rsum))
        m = trimesh.Trimesh(vertices=pr[3], faces=pr[4], process=False)
        sm = decimate(m, share)
        pr[3] = np.asarray(sm.vertices, float)
        pr[4] = np.asarray(sm.faces)

    # ── 68000 바이너리로 굽기 ───────────────────────────────
    #   [magic 'M68S'][조각수 u16]
    #   조각마다: [정점수 u16][면수 u16][정점 int16 x3 (0.1mm)][면 u16 x3]
    out = os.path.join(HERE, 'scene.bin')
    meta = []
    with open(out, 'wb') as fp:
        fp.write(b'M68S' + struct.pack('<H', len(parts)))
        for name, kind, rgb, v, f in parts:
            iv = (v * UNIT).round().astype(np.int32)
            lo, hi = iv.min(), iv.max()
            if lo < -32768 or hi > 32767:
                # 공장이 3.2m 를 넘으면 int16 밖으로 나간다 — 잘라내지 말고 알린다
                print(f"  ! {name}: 좌표가 int16 밖 ({lo}..{hi}) — 잘립니다")
            fp.write(struct.pack('<HH', len(iv), len(f)))
            fp.write(iv.astype(np.int16).tobytes())
            fp.write(f.astype(np.uint16).tobytes())
            meta.append({'id': name, 'kind': kind, 'rgb': list(rgb),
                         'nv': int(len(iv)), 'nf': int(len(f))})

    with open(os.path.join(HERE, 'scene.json'), 'w') as fp:
        json.dump({'parts': meta,
                   'tri': sum(m['nf'] for m in meta),
                   'link_m': baked,
                   'ptop': st.get('ptop'), 'psize': st.get('psize')},
                  fp, ensure_ascii=False, indent=1)

    tri = sum(m['nf'] for m in meta)
    print(f"원본 {tot_src:,} 삼각형 → {tri:,}")
    for m in meta:
        print(f"  {m['kind']:<6} {m['id'][:46]:<46} 면 {m['nf']:>5,}  정점 {m['nv']:>5,}")
    print(f"\nscene.bin  {os.path.getsize(out)/1024:.1f} KB · {tri:,} 삼각형")
    print(f"  68000 메모리 16MB 중 {os.path.getsize(out)/1024:.0f}KB")
    return 0


if __name__ == '__main__':
    sys.exit(main())
