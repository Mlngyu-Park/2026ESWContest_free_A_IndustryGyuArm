#!/usr/bin/env python3
"""영상 속 카메라 위치를 RViz 디지털 트윈에 띄운다 — 박민규님이 직접 실행.

    ./cam_rviz.sh shots/영상.mp4            (RViz 창까지 같이 띄움)

RViz 에 보이는 것 (기준 좌표계 'board' = 판의 첫 목심, z 는 판 위쪽)
  초록 점      판 900개 목심
  빨강 선      판 테두리 (30×30 = 600 mm)
  노랑 선      타일 경계
  하늘색 선    판 가운데 → 카메라 (방향 = 내려다보는 각 θ, 방위각 φ)
  주황 공      카메라 위치 (선 위의 점. 거리는 초점거리 추정 오차 약 ±2% 만큼 흔들릴 수 있다)
  주황 사각뿔  카메라가 보는 범위
  글자         θ · φ · 거리 · 높이
  영상 창     초록 점을 덧그린 영상 (OpenCV 가 그린 것, Esc 로 끝)
  1인칭 패널  RViz 'camera view' — 카메라 눈으로 본 3D 장면을 실제 영상 위에 겹친다.
              RViz 가 그린 초록 점이 영상 속 구멍 위에 떨어지면 디지털 트윈이 실물과 맞는 것

카메라 렌즈 값을 몰라도 된다 — 격자가 직각·같은 간격이라는 사실로 영상에서 초점거리를 구한다.
  자홍 원기둥  카메라가 읽은 로봇 자리 (위치만)
  색 원기둥    카메라가 읽은 도킹 (위치 + 색). 영상이 흐르며 보이는 만큼 생기고, 뽑으면 사라진다
"""
import sys, time
import cv2, numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import Image, CameraInfo
import array
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation

from grid import Board
from pose import PITCH, focal_from_H, K_of, camera_pose, angles
from cam import board_overlay
import camsync

FRAME = 'board'
SHOW = '--noshow' not in sys.argv
N = 30
SIDE = (N - 1) * PITCH                     # 첫 목심 ~ 마지막 목심 = 580 mm


def analyse(src):
    """앱과 같은 분석(camsync.analyse): 판 위치 · 초점거리 · 카메라 자세 · 판 위 물건(시간순)."""
    d = camsync.analyse(src)
    print(f"판 창 {d['window']} · 초점거리 {d['f']:.0f}px · {len(d['frames'])}프레임 {d['fps']:.0f}fps · 판 {len(d['tiles'])}타일")
    # 물건 판정은 앱과 똑같이 (깜빡임 막기 · 로봇 받침 다수결) — 판 좌표 그대로 쓰도록 원점 0
    cs = camsync.CamSync(); cs.data = d; cs.jpgs = [None] * len(d['frames']); cs.ready = True; cs.playing = False
    return d['frames'], d['Hs'], d['poses'], d['window'], d['K'], d['fps'], cs


def P(x, y, z=0.0):
    p = Point(); p.x, p.y, p.z = float(x), float(y), float(z); return p


def mk(ns, mid, typ, rgba, scale):
    m = Marker(); m.header.frame_id = FRAME; m.ns = ns; m.id = mid; m.type = typ; m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x = m.scale.y = m.scale.z = float(scale)
    m.color.r, m.color.g, m.color.b, m.color.a = (float(v) for v in rgba)
    return m


def board_markers():
    out = []
    pts = mk('holes', 0, Marker.POINTS, (0, 1, 0, 1), 0.006)
    pts.points = [P(i * PITCH, j * PITCH) for i in range(N) for j in range(N)]
    out.append(pts)
    e = -PITCH / 2; E = SIDE + PITCH / 2
    edge = mk('edge', 0, Marker.LINE_STRIP, (1, 0, 0, 1), 0.004)
    edge.points = [P(e, e), P(E, e), P(E, E), P(e, E), P(e, e)]
    out.append(edge)
    tiles = mk('tiles', 0, Marker.LINE_LIST, (1, 1, 0, 1), 0.003)
    for k in (10, 20):
        v = (k - 0.5) * PITCH
        tiles.points += [P(v, e), P(v, E), P(e, v), P(E, v)]
    out.append(tiles)
    return out


def camera_markers(C, Rwc, K, wh):
    out = []
    ctr = np.array([SIDE / 2, SIDE / 2, 0.0])
    ray = mk('ray', 0, Marker.LINE_LIST, (0.2, 0.8, 1, 1), 0.004)
    d = C - ctr; L = np.linalg.norm(d)
    ray.points = [P(*ctr), P(*(ctr + d / L * max(L * 1.3, 0.3)))]     # 카메라 너머까지 길게
    out.append(ray)
    cam = mk('camera', 0, Marker.SPHERE, (1, 0.5, 0, 1), 0.03)
    cam.pose.position = P(*C); out.append(cam)
    w, h = wh; Ki = np.linalg.inv(K); depth = 0.12
    corners = [Rwc @ (Ki @ np.array([u, v, 1.0])) * depth + C for u, v in ((0, 0), (w, 0), (w, h), (0, h))]
    fr = mk('frustum', 0, Marker.LINE_LIST, (1, 0.5, 0, 1), 0.003)
    for k in range(4):
        fr.points += [P(*C), P(*corners[k]), P(*corners[k]), P(*corners[(k + 1) % 4])]
    out.append(fr)
    theta = np.degrees(np.arcsin(d[2] / L)); phi = np.degrees(np.arctan2(d[1], d[0]))
    txt = mk('label', 0, Marker.TEXT_VIEW_FACING, (1, 1, 1, 1), 0.03)
    txt.pose.position = P(*(C + [0, 0, 0.06]))
    txt.text = f'theta {theta:.1f} deg  phi {phi:.1f} deg\ndist {L * 1000:.0f} mm  height {C[2] * 1000:.0f} mm'
    out.append(txt)
    return out, theta, phi, L


def object_markers(cs, k):
    """카메라가 읽은 로봇(자홍 원기둥 · 위치만)과 도킹(그 색 원기둥 · 위치 + 색)."""
    cs.k_pause = k
    lay = cs.layout((0.0, 0.0), 0.0)          # 원점 0 · 회전 0 → 판 좌표 그대로
    out = []
    items = ([('robot', lay['robot'], '#ff00ff', 0.12, 0.10)] if lay and lay['robot'] else [])
    items += [('docking', d['xy'], d['hex'], 0.07, 0.115) for d in (lay['dockings'] if lay else [])]
    for i in range(12):                          # 개수가 줄면 남은 것을 지워야 해서 늘 12개를 보낸다
        m = mk('objects', i, Marker.CYLINDER, (0, 0, 0, 0), 0.01)
        if i < len(items):
            kind, xy, hx, dia, hgt = items[i]
            r, g, b = (int(hx[1:3], 16) / 255, int(hx[3:5], 16) / 255, int(hx[5:7], 16) / 255)
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 0.75
            m.scale.x = m.scale.y = dia; m.scale.z = hgt
            m.pose.position = P(xy[0], xy[1], hgt / 2)
        else:
            m.action = Marker.DELETE
        out.append(m)
    return out


class CamRviz(Node):
    def __init__(self, data, loop=True):
        super().__init__('cam_grid_rviz')
        self.frames, self.Hs, self.poses, (self.i0, self.j0), self.K, fps, self.cs = data
        self.loop = loop; self.k = 0
        self.tf = TransformBroadcaster(self)
        self.pm = self.create_publisher(MarkerArray, '/cam_grid/markers', 10)
        # 1인칭: RViz 의 Camera 표시가 이 영상 + 렌즈 값 + TF 'camera' 로 카메라 눈의 장면을 그린다
        self.pi = self.create_publisher(Image, '/cam_grid/image', 10)
        self.pc = self.create_publisher(CameraInfo, '/cam_grid/camera_info', 10)
        self.scale = 0.5                                   # RViz 로 보내는 영상 크기 (렌즈 값도 같은 비율로)
        self.board = board_markers()
        self.create_timer(1.0 / fps, self.tick)

    def publish_camera(self, f, stamp):
        """원본 영상(덧그림 없음) + 렌즈 값. RViz 가 그린 목심 점이 영상의 구멍 위에 떨어지면 트윈이 맞는 것."""
        s = self.scale
        img = cv2.resize(f, (int(f.shape[1] * s), int(f.shape[0] * s)))
        im = Image(); im.header.stamp = stamp; im.header.frame_id = 'camera'
        im.height, im.width = img.shape[:2]; im.encoding = 'bgr8'; im.step = img.shape[1] * 3
        im.data = array.array('B', np.ascontiguousarray(img).tobytes())   # array.array 는 검사 없이 바로 들어간다
        ci = CameraInfo(); ci.header = im.header; ci.width, ci.height = im.width, im.height
        K = self.K.copy(); K[:2] *= s
        ci.k = [float(v) for v in K.ravel()]
        ci.p = [float(K[0, 0]), 0.0, float(K[0, 2]), 0.0, 0.0, float(K[1, 1]), float(K[1, 2]), 0.0, 0.0, 0.0, 1.0, 0.0]
        ci.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        ci.d = [0.0] * 5; ci.distortion_model = 'plumb_bob'
        self.pc.publish(ci); self.pi.publish(im)

    def tick(self):
        if self.k >= len(self.frames):
            if not self.loop: rclpy.shutdown(); return
            self.k = 0
        f, H, ps = self.frames[self.k], self.Hs[self.k], self.poses[self.k]
        now = self.get_clock().now().to_msg()
        ma = MarkerArray(); ma.markers = list(self.board) + object_markers(self.cs, self.k)
        if ps is not None:
            C, Rwc = ps
            cm, th, ph, L = camera_markers(C, Rwc, self.K, (f.shape[1], f.shape[0]))
            ma.markers += cm
            t = TransformStamped(); t.header.stamp = now; t.header.frame_id = FRAME; t.child_frame_id = 'camera'
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, C)
            q = Rotation.from_matrix(Rwc).as_quat()
            t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = map(float, q)
            self.tf.sendTransform(t)
            img = board_overlay(f, H, self.i0, self.j0)
            cv2.putText(img, f'theta {th:.1f}  phi {ph:.1f}  dist {L * 1000:.0f}mm', (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 200, 0), 2)
        else:
            img = f
        for m in ma.markers: m.header.stamp = now
        self.pm.publish(ma)
        self.publish_camera(f, now)
        # 영상은 RViz 로 보내지 않고 창으로 띄운다 (ROS 메시지로 바꾸는 데 프레임당 40~100ms 가 걸려 24fps 를 못 따라간다)
        if SHOW:                                         # --noshow 면 영상 창을 안 띄운다 (RViz 만)
            cv2.imshow('camera + board (green dots)', img)
            if (cv2.waitKey(1) & 0xFF) == 27: rclpy.shutdown(); return
        self.k += 1


def main():
    if len(sys.argv) < 2: sys.exit(__doc__)
    data = analyse(sys.argv[1])
    rclpy.init()
    node = CamRviz(data, loop='--once' not in sys.argv)
    print('RViz 로 보내는 중 — Ctrl+C 로 끝')
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    except Exception as e:
        if rclpy.ok(): print(f'오류: {type(e).__name__}: {e}')
    finally:
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()
