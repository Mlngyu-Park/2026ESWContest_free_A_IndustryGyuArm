# pick_dock_mate.py 가 생성한 도킹 메이트 (하드코딩)
# 동심: 펜 중심축 = 도킹 소켓 축 (socket_xy_mm, 도킹 로컬)
# Z 일치: 펜 기준 원(seat_dz_mm)이 도킹 맨 윗면 높이
# 도킹: /home/drboom/2026_project/stl/Industry_gyu_arm_20260727 - docking_station-5.STL
# 펜  : /home/drboom/2026_project/urdf/bricks/Industry_gyu_arm_20260809_ppap/meshes/ppap_l1.STL
DOCK_MATE = {"brick": "ppap", "dock_file": "Industry_gyu_arm_20260727_-_docking_station-5", "socket_xy_mm": [270, 62.5], "seat_dz_mm": 38.96, "pen_circle_r_mm": 17.43}
