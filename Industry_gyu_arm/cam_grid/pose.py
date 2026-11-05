"""격자 호모그래피 -> 카메라 위치·방향 (판 기준).

판 좌표계 (단위 m)
  원점 = 판 30×30 창의 첫 목심 (i0, j0)
  x = i 방향, y = j 방향 (격자 한 칸 = 20 mm), z = 판에서 위로

초점거리를 모를 때 (휴대폰 영상 등)
  평평한 격자는 실제로 '직각이고 가로세로 간격이 같다'. 이 두 조건이
  호모그래피에서 초점거리 f 를 정해 준다 (Zhang 의 보정 조건, 주점=화면 가운데 가정).
  프레임마다 구해 중앙값을 쓰고, 프레임끼리 얼마나 일정한지로 검산한다.
"""
import numpy as np

PITCH = 0.020


def focal_from_H(H, w, h):
    """한 프레임의 호모그래피로 초점거리 추정. 못 구하면 None."""
    T = np.array([[1, 0, -w / 2], [0, 1, -h / 2], [0, 0, 1.0]])
    A = T @ H
    a, b = A[:, 0], A[:, 1]
    out = []
    den = a[2] * b[2]
    if abs(den) > 1e-12:
        f2 = -(a[0] * b[0] + a[1] * b[1]) / den                       # 직각 조건
        if f2 > 0: out.append(np.sqrt(f2))
    den = a[2] ** 2 - b[2] ** 2
    if abs(den) > 1e-12:
        f2 = -(a[0] ** 2 + a[1] ** 2 - b[0] ** 2 - b[1] ** 2) / den    # 같은 간격 조건
        if f2 > 0: out.append(np.sqrt(f2))
    return float(np.mean(out)) if out else None


def K_of(f, w, h):
    return np.array([[f, 0, w / 2], [0, f, h / 2], [0, 0, 1.0]])


def camera_pose(H, K, i0, j0):
    """H(격자 i,j -> 픽셀) -> 판 좌표계에서 카메라 위치 C(m)와 회전 R(카메라->판).
    반환: C, R_wc, (r1·r2 직교 오차)"""
    S = np.array([[1 / PITCH, 0, i0], [0, 1 / PITCH, j0], [0, 0, 1.0]])   # 판 m -> 격자
    A = np.linalg.inv(K) @ H @ S
    lam = 2.0 / (np.linalg.norm(A[:, 0]) + np.linalg.norm(A[:, 1]))
    r1, r2, t = A[:, 0] * lam, A[:, 1] * lam, A[:, 2] * lam
    if t[2] < 0: r1, r2, t = -r1, -r2, -t                                  # 판이 카메라 앞에 있어야
    ortho = abs(float(r1 @ r2))
    R = np.column_stack([r1, r2, np.cross(r1, r2)])
    U, _, Vt = np.linalg.svd(R); R = U @ Vt                                # 가장 가까운 회전
    C = -R.T @ t                                                           # 카메라 중심 (판 좌표)
    Rwc = R.T                                                              # 카메라 -> 판
    if C[2] < 0:                                                           # z 는 판 위쪽이 + 가 되게
        F = np.diag([1, 1, -1.0]); C = F @ C; Rwc = F @ Rwc @ np.diag([1, -1, -1.0])
    return C, Rwc, ortho


def angles(Rwc):
    """사람이 읽기 쉬운 각도: 내려다보는 각(수평 기준), 방위각."""
    z = Rwc[:, 2]                          # 카메라가 보는 방향 (판 좌표)
    tilt = np.degrees(np.arcsin(-z[2]))    # 아래로 몇 도
    yaw = np.degrees(np.arctan2(z[1], z[0]))
    return float(tilt), float(yaw)
