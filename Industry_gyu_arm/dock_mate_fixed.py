# pick_dock_mate.py 가 생성한 도킹 메이트 (하드코딩)
# 동심: 펜 중심축 = 도킹 소켓 축 (socket_xy_mm, 도킹 로컬)
# Z 일치: 펜 바닥이 도킹 맨 윗면에서 seat_dz_mm 아래 (꽂힘 깊이)
# 도킹: /home/drboom/2026_project/stl/Industry_gyu_arm_20260727 - docking_station-5.STL
# 펜  : /home/drboom/2026_project/urdf/bricks/Industry_gyu_arm_20260809_ppap/meshes/ppap_l1.STL
# 11-05 새 펜 (사용자: 전체 82mm, 턱 지름 35mm·두께 10mm, 턱 위 몸통 45mm) — 턱이 도킹 윗면 위에 '얹힌다'(거치, 파고들지 않음)
#   -> 턱 아랫면 = 도킹 윗면, 아래 몸통 27mm 가 소켓 안, 꼭대기 = 도킹 윗면 + 55mm
#   3D 는 temp/PPAP_new.STL(93.5mm)에서 도킹 안 아래쪽만 줄여 82mm 로 맞춘 것 (최종 STL 은 나중에 받기로)
#   예전 펜(139.5mm): seat_dz 38.96, r 17.43
DOCK_MATE = {"brick": "ppap", "dock_file": "Industry_gyu_arm_20260727_-_docking_station-5", "socket_xy_mm": [270, 62.5], "seat_dz_mm": 27.0, "pen_circle_r_mm": 17.5}
