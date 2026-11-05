#!/usr/bin/env python3
"""로봇 STL 을 68000 이 감당할 수준으로 줄인다.

PL-68000 이 8MHz 한 개로 실시간을 내는 규모는 수백~수천 폴리곤이다.
우리 로봇은 49,144 삼각형 — 40배 줄여야 한다.

  python simplify.py [목표삼각형수]
"""
import os, sys, struct
import numpy as np
import trimesh

MESH_DIR = os.path.expanduser(
    '~/2026_project/urdf/sts3215/Industry_gyu_arm_20260728_base/meshes')
LINKS = ['base_link','l1','l2','l3','l4','l5','l6']

# URDF 조인트 원점 (부모→자식, m). rpy 는 전부 0 이라 평행이동뿐이다.
# STL 은 각 링크의 로컬 좌표계로 저장돼 있어서, 그대로 그리면 팔이 흩어진다.
# 부모를 따라 누적한 오프셋을 미리 더해 '조립된 자세' 로 만든다.
JOINT_ORIGIN = [
    (0.240050000000002, -0.2348,             0.0924),              # j1 base→l1
    (0.0182000000000001, 0.0306,             0.0491999999999997),  # j2 l1→l2
    (0.0119,             0.028,              0.112569978235762),   # j3 l2→l3
    (-0.00290000000000018, 0.134899814673214, 0.00519999999999987),# j4 l3→l4
    (-0.0270999999999996, 0.0591000000000002, 0.0),                # j5 l4→l5
    (0.0201999999999996, 0.0244000000000003, -0.0231),             # j6 l5→l6
]

def link_offsets():
    """각 링크의 절대 원점 (base_link 기준, m)."""
    offs = [(0.0, 0.0, 0.0)]
    for dx, dy, dz in JOINT_ORIGIN:
        px, py, pz = offs[-1]
        offs.append((px + dx, py + dy, pz + dz))
    return offs
TARGET_TOTAL = int(sys.argv[1]) if len(sys.argv) > 1 else 1200

def main():
    metas = []
    total_src = 0
    for ln in LINKS:
        p = os.path.join(MESH_DIR, ln + '.STL')
        # process=True 로 열어야 STL 의 중복 정점이 하나로 합쳐진다.
        # 합치지 않으면 면들이 서로 떨어진 조각이라 데시메이션이
        # 모서리를 못 접고 얇은 조각(sliver)만 잔뜩 남는다.
        m = trimesh.load(p, process=True)
        m.merge_vertices()
        m.update_faces(m.nondegenerate_faces())
        m.update_faces(m.unique_faces())
        total_src += len(m.faces)
        metas.append((ln, m))
    print(f"원본 합계 {total_src:,} 삼각형 → 목표 {TARGET_TOTAL:,}")

    out = {}
    offs = link_offsets()
    for li, (ln, m) in enumerate(metas):
        share = max(40, int(TARGET_TOTAL * len(m.faces) / total_src))
        try:
            s = m.simplify_quadric_decimation(face_count=share)
        except Exception:
            s = m.simplify_quadratic_decimation(share)
        # 줄이고 나서도 남은 얇은 조각을 걷어낸다 — 화면에서 한 점으로
        # 뭉개지는 면은 사이클만 먹고 그림에 보탬이 안 된다.
        s.merge_vertices()
        s.update_faces(s.nondegenerate_faces())
        # 68000 은 정수만 다룬다. mm 단위로 반올림하면 로봇이 작아서(440mm)
        # 인접 정점이 같은 값으로 뭉개져 삼각형이 통째로 사라진다.
        # 0.1mm 단위로 저장한다 — int16 범위(±3276.7mm)에 넉넉히 들어간다.
        # 링크 로컬좌표 → 조립된 절대좌표
        ox, oy, oz = offs[li]
        v = ((s.vertices + np.array([ox, oy, oz])) * 10000).round().astype(np.int32)
        f = s.faces.astype(np.int32)
        out[ln] = (v, f)
        print(f"  {ln:<10}{len(m.faces):>7,} → {len(f):>5,}  "
              f"정점 {len(v):>5,}  크기 {(len(v)*6+len(f)*6)/1024:.1f}KB")

    # 68000 이 읽을 바이너리로 저장
    #   좌표 단위는 0.1mm
    #   [magic 'M68G'][링크수 u16]
    #   링크마다: [정점수 u16][면수 u16][정점 int16 x3...][면 u16 x3...]
    with open('robot_lo.bin','wb') as fp:
        fp.write(b'M68G' + struct.pack('<H', len(LINKS)))
        for ln in LINKS:
            v, f = out[ln]
            fp.write(struct.pack('<HH', len(v), len(f)))
            fp.write(v.astype(np.int16).tobytes())
            fp.write(f.astype(np.uint16).tobytes())
    sz = os.path.getsize('robot_lo.bin')
    tot_f = sum(len(f) for _, f in out.values())
    print(f"\nrobot_lo.bin  {sz/1024:.1f} KB · {tot_f:,} 삼각형")
    print(f"  68000 메모리에 여유롭게 들어간다 (16MB 중 {sz/1024:.0f}KB)")

if __name__ == '__main__':
    main()
