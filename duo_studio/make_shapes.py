#!/usr/bin/env python3
"""부딪힘 계산용 모양 (11-01) — python3 make_shapes.py → web/assets/shapes.json

  링크마다 (링크 좌표)
    pts : 겉면에 고르게 뿌린 점 (손은 3mm, 나머지 8mm 간격)
    vox : 속까지 채운 5mm 칸 (겉에서 한 칸 깎음 = 5mm 까지는 봐준다) — 무엇이 몸 '속' 에 들어갔는지 볼 때
  화면(app.js)·Node 검사(test_*.mjs)·블록 실행기(world.js)가 같이 쓴다. prep.py 다음에 돌린다.
"""
import base64, json, os, re, sys
import numpy as np
import trimesh
from scipy.ndimage import binary_erosion

HERE = os.path.dirname(os.path.abspath(__file__))
ASSET = os.path.join(HERE, "web", "assets", sys.argv[2]) if len(sys.argv) > 2 else os.path.join(HERE, "web", "assets")
FINE = re.compile(sys.argv[1] if len(sys.argv) > 1 else r"hand|fngr|wr1")   # 촘촘히 볼 링크
PITCH = 0.005

def rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])

def main():
    model = json.load(open(os.path.join(ASSET, "robot.json")))
    out, npts = {}, 0
    for name, L in model["links"].items():
        ms = []
        for v in L["visuals"]:
            m = trimesh.load(os.path.join(ASSET, v["mesh"]), force="mesh")
            m.apply_scale(v.get("scale", [1, 1, 1])[0])
            T = np.eye(4); T[:3, :3] = rpy(*v["rpy"]); T[:3, 3] = v["xyz"]; m.apply_transform(T)
            ms.append(m)
        if not ms: continue
        m = trimesh.util.concatenate(ms)
        step = 0.003 if FINE.search(name) else 0.008
        n = max(30, int(m.area / (step * step)))
        p, _ = trimesh.sample.sample_surface_even(m, n, radius=step * 0.7, seed=1)
        p = np.vstack([p, m.vertices[::max(1, len(m.vertices) // 200)]])
        V = m.voxelized(PITCH).fill()
        occ = binary_erosion(V.matrix, iterations=1)
        lo = V.transform[:3, 3]                                  # 칸 (0,0,0) 의 가운데
        bits = np.packbits(occ.astype(np.uint8).ravel())
        out[name] = dict(pts=np.round(p, 4).ravel().tolist(), vox=dict(lo=np.round(lo, 5).tolist(), pitch=PITCH,
                         dims=list(occ.shape), bits=base64.b64encode(bits.tobytes()).decode()), filled=int(occ.sum()))
        npts += len(p)
    json.dump(out, open(os.path.join(ASSET, "shapes.json"), "w"), separators=(",", ":"))
    print(f"링크 {len(out)} · 점 {npts:,} · {os.path.getsize(os.path.join(ASSET, 'shapes.json')) / 1e6:.1f}MB")

if __name__ == "__main__":
    main()
