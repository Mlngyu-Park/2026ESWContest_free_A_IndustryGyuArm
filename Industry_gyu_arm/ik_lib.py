#!/usr/bin/env python3
"""
IK 공용 모듈 (5자유도). URDF origin/axis 를 직접 읽어 ikpy 체인을 수동 구성.
ikpy 자동파싱이 fixed joint/그리퍼 분기를 오인하는 문제를 회피.

체인: base_link -(joint_1..5)-> link_5 -(joint_6 fixed)-> link_6(엔드이펙터)
활성 관절: joint_1~5 (5개).
"""
import math
import numpy as np
import xml.etree.ElementTree as ET
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink

URDF = "/home/drboom/2026_project/Industry_gyu_arm_20260719/urdf/Industry_gyu_arm_limits.urdf"

# 팔을 이루는 조인트 순서 (엔드이펙터=link_6까지)
ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

MOTORS = {0: 'j1', 1: 'j2', 2: 'j3', 3: 'j4', 4: 'j5'}
ZERO_RAW = {0: 2370, 1: 1320, 2: 998, 3: 1948, 4: 2575}  # 2026-07-20 기본자세 재설정
# measure_limits 기반 rad 한계 (j1~j5)
LIMITS_RAD = {0: (-1.697, 1.841), 1: (-1.565, 2.094), 2: (-1.132, 2.568),
              3: (-1.694, 1.655), 4: (-1.549, 1.612)}


def _floats(s):
    return [float(x) for x in s.split()]


def build_chain():
    root = ET.parse(URDF).getroot()
    joints = {j.get('name'): j for j in root.findall('joint')}

    links = [OriginLink()]   # base_link
    active = [False]
    for jn in ARM_JOINTS:
        j = joints[jn]
        o = j.find('origin')
        xyz = _floats(o.get('xyz')) if o is not None and o.get('xyz') else [0, 0, 0]
        rpy = _floats(o.get('rpy')) if o is not None and o.get('rpy') else [0, 0, 0]
        typ = j.get('type')
        if typ == 'revolute':
            ax = _floats(j.find('axis').get('xyz'))
            lim = j.find('limit')
            bounds = (float(lim.get('lower')), float(lim.get('upper'))) if lim is not None else (-math.pi, math.pi)
            links.append(URDFLink(name=jn, origin_translation=xyz,
                                  origin_orientation=rpy, rotation=ax, bounds=bounds,
                                  joint_type='revolute'))
            active.append(True)
        else:  # fixed (joint_6) : 위치 오프셋만 (rotation/translation 모두 None)
            links.append(URDFLink(name=jn, origin_translation=xyz,
                                  origin_orientation=rpy, rotation=None,
                                  translation=None, joint_type='fixed'))
            active.append(False)
    return Chain(name="arm", links=links, active_links_mask=active)


CHAIN = build_chain()


def raw_to_rad(motor_id, raw):
    return (raw - ZERO_RAW[motor_id]) * (math.pi / 2048.0)


def rad_to_raw(motor_id, rad):
    return int(ZERO_RAW[motor_id] + rad * 2048.0 / math.pi)


def full_from_arm(arm_q5):
    """j1~j5 rad 리스트 -> ikpy 전체 관절벡터."""
    full = [0.0] * len(CHAIN.links)
    ji = 0
    for k in range(len(CHAIN.links)):
        if CHAIN.active_links_mask[k]:
            full[k] = arm_q5[ji]; ji += 1
    return full


def arm_from_full(full):
    return [full[k] for k in range(len(CHAIN.links)) if CHAIN.active_links_mask[k]]


def fk(arm_q5):
    """j1~j5 rad -> 엔드이펙터 XYZ (m)."""
    T = CHAIN.forward_kinematics(full_from_arm(arm_q5))
    return T[:3, 3]


def fk_full(arm_q5):
    """j1~j5 rad -> 4x4 변환행렬 (위치+방향 전체)."""
    return CHAIN.forward_kinematics(full_from_arm(arm_q5))


# 기본자세(모든 관절 0)의 그리퍼 방향(회전행렬)을 고정 목표로 사용.
HOME_ORIENT = fk_full([0, 0, 0, 0, 0])[:3, :3]


def ik(target_xyz, seed_q5, keep_orient=True):
    """목표 XYZ -> j1~j5 rad.
    keep_orient=True 면 그리퍼 방향을 기본자세 방향으로 유지 (비틀림 방지).
    5자유도라 위치+방향을 완벽히 다 못 맞추므로 ikpy가 최소오차 해를 찾음."""
    seed = full_from_arm(seed_q5)
    if keep_orient:
        sol = CHAIN.inverse_kinematics(
            target_position=target_xyz,
            target_orientation=HOME_ORIENT,
            orientation_mode="all",
            initial_position=seed,
        )
    else:
        sol = CHAIN.inverse_kinematics(target_position=target_xyz, initial_position=seed)
    return arm_from_full(sol)


if __name__ == '__main__':
    import sys
    sys.stdout.reconfigure(line_buffering=True)
    print("=== 수동 구성 체인 ===")
    for k, l in enumerate(CHAIN.links):
        print(f"  [{k}] {l.name:10s} active={CHAIN.active_links_mask[k]}")
    print("활성 관절:", sum(CHAIN.active_links_mask))
    # 영점(모든 관절 0)에서 FK
    p0 = fk([0, 0, 0, 0, 0])
    print(f"\n[영점 자세] 엔드이펙터 XYZ = ({p0[0]:.4f}, {p0[1]:.4f}, {p0[2]:.4f})  거리 {np.linalg.norm(p0)*1000:.0f}mm")
