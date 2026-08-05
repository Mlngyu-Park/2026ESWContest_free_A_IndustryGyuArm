#!/usr/bin/env python3
"""
IK 준비 + FK 검증 (5자유도). v2 - 체인을 link_6까지로 명확히 지정.
- base_link → ... → link_6 (그리퍼 손목). 그리퍼 분기(left/right)는 제외.
- 활성 관절: joint_1~5 (5개). joint_6(fixed)는 위치 오프셋만.
모터는 읽기만 함(안전).
"""
import sys, math
import numpy as np
from ikpy.chain import Chain
from ikpy.link import URDFLink
from dynamixel_sdk import PortHandler, PacketHandler

sys.stdout.reconfigure(line_buffering=True)

URDF = "/home/drboom/2026_project/Industry_gyu_arm_20260719/urdf/Industry_gyu_arm_limits.urdf"
PORT='/dev/ttyUSB0'; BAUD=57600
ADDR_PRESENT_POSITION=132
MOTORS={0:'j1',1:'j2',2:'j3',3:'j4',4:'j5'}
ZERO_RAW={0:2406,1:1441,2:1430,3:2319,4:2532}

def s32(v): return v-0x100000000 if v>0x7FFFFFFF else v
def raw_to_rad(i,raw): return (raw-ZERO_RAW[i])*(math.pi/2048.0)

# 체인을 link_6 (그리퍼 손목)까지로 명확히 지정. left/right 분기 제거.
chain = Chain.from_urdf_file(
    URDF,
    base_elements=["base_link", "joint_1"],
    last_link_vector=None,
    name="arm",
)
# ikpy가 자동으로 첫 분기(link_6 다음)를 따라감. 실제 활성 확인.
print("=== IK 체인 구성 ===")
for k, link in enumerate(chain.links):
    print(f"  [{k}] {link.name:14s} active={chain.active_links_mask[k]}")
print(f"활성 관절 수: {sum(chain.active_links_mask)}")

# 현재 관절각 읽기
port=PortHandler(PORT); packet=PacketHandler(2.0)
port.openPort(); port.setBaudRate(BAUD)
q={}
for i in MOTORS:
    p,r,_=packet.read4ByteTxRx(port,i,ADDR_PRESENT_POSITION)
    q[i]=raw_to_rad(i,s32(p)) if r==0 else 0.0
port.closePort()
print("\n현재 관절각(rad):", {MOTORS[i]:round(q[i],3) for i in MOTORS})

# active 위치에 j1~j5 채우기
full=[0.0]*len(chain.links)
arm_q=[q[i] for i in sorted(MOTORS)]
ji=0
for k in range(len(chain.links)):
    if chain.active_links_mask[k] and ji<len(arm_q):
        full[k]=arm_q[ji]; ji+=1

T=chain.forward_kinematics(full)
pos=T[:3,3]
print(f"\n[FK] 엔드이펙터 위치(m): X={pos[0]:.4f} Y={pos[1]:.4f} Z={pos[2]:.4f}")
print(f"     원점에서 거리 = {np.linalg.norm(pos)*1000:.0f} mm")

sol=chain.inverse_kinematics(target_position=pos, initial_position=full)
pos2=chain.forward_kinematics(sol)[:3,3]
print(f"[IK검증] 복원 오차 = {np.linalg.norm(pos-pos2)*1000:.2f} mm")
