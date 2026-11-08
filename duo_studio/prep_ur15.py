#!/usr/bin/env python3
"""UR15 (도색 때 쓰던 로컬 모델) → DUO STUDIO 책꽂이용 (11-02).

  python3 prep_ur15.py  → web/assets/ur15/robot.json + meshes/*.stl
원본: ../urdf/ur15_20260823 (사용자 SolidWorks 내보내기). j1~j6 = 팔, j7·end = 도색 끝장치(분사기) → 뺀다.
집게: 원본에 없어 직접 만든 임시 평행 집게를 l6 공구 원판에 붙인다 (상자 모양, 손가락 두 개가 마주 보고 미끄러짐, 벌림 0~17cm).
  집게 좌표: z = 손 끝 방향(원판에서 밖으로), y = 손가락이 닫히는 축, x = y×z. 잡는 점(tcp) = 손가락 끝에서 2cm 안쪽.
"""
import json, os, shutil, xml.etree.ElementTree as ET
import numpy as np, trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "..", "urdf", "ur15_20260823")
OUT = os.path.join(HERE, "web", "assets", "ur15")
KEEP = ["base_link", "l1", "l2", "l3", "l4", "l5", "l6"]
FLANGE_Y = -0.057                                   # l6 좌표의 공구 원판 면 (메쉬 끝)
BASE = dict(x=0.06, y=0.11, z=0.075)                # 집게 몸통
FING = dict(x=0.022, y=0.012, z=0.10)               # 손가락 (몸통 끝에서 앞으로 0.10)
STROKE = 0.085                                      # 손가락 하나가 가운데에서 벌어지는 거리 (0 = 닫힘)
COLORS = {"base_link": "#5f6b7a", "l1": "#cfd5dd", "l2": "#cfd5dd", "l3": "#cfd5dd", "l4": "#5f6b7a", "l5": "#cfd5dd", "l6": "#5f6b7a"}

def xyz(s): return [float(v) for v in (s or "0 0 0").split()]

def box_stl(path, ext, center):
    m = trimesh.creation.box(extents=ext); m.apply_translation(center); m.export(path)

def main():
    os.makedirs(os.path.join(OUT, "meshes"), exist_ok=True)
    root = ET.parse(os.path.join(SRC, "urdf", "ur15_20260823.urdf")).getroot()
    links, joints = {}, []
    for l in root.findall("link"):
        n = l.get("name")
        if n not in KEEP: continue
        shutil.copy(os.path.join(SRC, "meshes", n + ".STL"), os.path.join(OUT, "meshes", n + ".stl"))
        links[n] = dict(visuals=[dict(mesh=f"meshes/{n}.stl", xyz=[0, 0, 0], rpy=[0, 0, 0], scale=[1, 1, 1], color=COLORS[n])], color=COLORS[n])
    for j in root.findall("joint"):
        c = j.find("child").get("link")
        if c not in KEEP: continue
        o = j.find("origin"); a = j.find("axis"); lim = j.find("limit")
        joints.append(dict(name=j.get("name"), type=j.get("type"), parent=j.find("parent").get("link"), child=c,
                           xyz=xyz(o.get("xyz")), rpy=xyz(o.get("rpy")), axis=xyz(a.get("xyz")),
                           lower=float(lim.get("lower")), upper=float(lim.get("upper"))))
    # 집게: l6 원판에 고정 (집게 z = l6 의 -y → roll +90°: y→z 로 돌리면 z 가 -y)
    box_stl(os.path.join(OUT, "meshes", "grip_base.stl"), [BASE["x"], BASE["y"], BASE["z"]], [0, 0, BASE["z"] / 2])
    box_stl(os.path.join(OUT, "meshes", "grip_finger.stl"), [FING["x"], FING["y"], FING["z"]], [0, FING["y"] / 2, FING["z"] / 2])
    links["grip_base"] = dict(visuals=[dict(mesh="meshes/grip_base.stl", xyz=[0, 0, 0], rpy=[0, 0, 0], scale=[1, 1, 1], color="#2b2f36")], color="#2b2f36")
    for side, sgn in (("l", 1), ("r", -1)):
        links[f"grip_finger_{side}"] = dict(visuals=[dict(mesh="meshes/grip_finger.stl", xyz=[0, 0, 0], rpy=[0, 0, 0] if sgn > 0 else [0, 0, np.pi],
                                                         scale=[1, 1, 1], color="#3a414c")], color="#3a414c")
        joints.append(dict(name=f"grip_{side}", type="prismatic", parent="grip_base", child=f"grip_finger_{side}",
                           xyz=[0, 0, BASE["z"]], rpy=[0, 0, 0], axis=[0, sgn, 0], lower=0.0, upper=STROKE))
    links["tcp"] = dict(visuals=[], color="#000000")
    joints.append(dict(name="grip_mount", type="fixed", parent="l6", child="grip_base", xyz=[0, FLANGE_Y, 0], rpy=[np.pi / 2, 0, 0], axis=[1, 0, 0], lower=None, upper=None))
    joints.append(dict(name="tcp", type="fixed", parent="grip_base", child="tcp", xyz=[0, 0, BASE["z"] + FING["z"] - 0.02], rpy=[0, 0, 0], axis=[1, 0, 0], lower=None, upper=None))
    json.dump(dict(name="ur15_with_gripper", root="base_link", links=links, joints=joints), open(os.path.join(OUT, "robot.json"), "w"), ensure_ascii=False, indent=1)
    print("링크", len(links), "관절", len(joints))

if __name__ == "__main__":
    main()
