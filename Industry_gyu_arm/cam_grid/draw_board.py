"""누적된 판을 위에서 본 그림으로 — 칸 색(파/흰), 본 구멍(검정 점), 30칸 테두리."""
import cv2, numpy as np
def draw(B, path, cell=16):
    lo, hi, span = B.extent()
    L = np.array(B.window())               # 판 = 가장 많이 본 30×30 창
    img = np.full((32 * cell, 32 * cell, 3), 60, np.uint8)
    for (i, j), v in B.color.items():
        if not B.inside((i, j)): continue
        x, y = (i - L[0] + 1) * cell, (j - L[1] + 1) * cell
        if 0 <= x < img.shape[1] - cell and 0 <= y < img.shape[0] - cell:
            c = (200, 110, 30) if v['B'] >= v['W'] else (235, 235, 235)
            cv2.rectangle(img, (x, y), (x + cell - 1, y + cell - 1), c, -1)
    for (i, j) in B.holes:
        if not B.inside((i, j)): continue
        x, y = (i - L[0] + 1) * cell, (j - L[1] + 1) * cell
        cv2.circle(img, (x, y), 3, (0, 0, 0), -1)
    cv2.rectangle(img, (cell, cell), (31 * cell, 31 * cell), (0, 0, 255), 2)   # 30칸 창
    tp = B.tile_phase()                                                          # 타일 경계 (초록)
    for k in range(-4, 5):
        x = (tp['p'] + 10 * k - L[0] + 1) * cell - cell // 2
        y = (tp['q'] + 10 * k - L[1] + 1) * cell - cell // 2
        if 0 <= x < img.shape[1]: cv2.line(img, (x, 0), (x, img.shape[0]), (0, 200, 0), 2)
        if 0 <= y < img.shape[0]: cv2.line(img, (0, y), (img.shape[1], y), (0, 200, 0), 2)
    cv2.putText(img, f"tile phase {tp['p']},{tp['q']}  agree {tp['일치율']:.1%}", (4, 26), 0, 0.4, (255, 255, 255), 1)
    n_in = sum(B.inside(p) for p in B.holes)
    cv2.putText(img, f"holes {n_in}/900 inside board", (4, 12), 0, 0.4, (255, 255, 255), 1)
    cv2.imwrite(path, img)
