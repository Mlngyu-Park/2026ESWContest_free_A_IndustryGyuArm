#!/usr/bin/env python3
"""
STS3215 레지스터 덤프 (읽기 전용, 절대 안 움직임).
토크도 건드리지 않고 그냥 읽기만 합니다.
"""
import sys
import scservo_sdk.port_handler as _ph
_ph.LATENCY_TIMER=16
from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS

sys.stdout.reconfigure(line_buffering=True)
PORT='/dev/ttyACM0'; BAUD=1000000
IDS=[1,2,3,4,5,6]

# 관심 레지스터 (STS/SMS 계열 통용)
REGS=[
 (5,1,'ID'),(6,1,'Baud'),(9,2,'Min Angle Limit'),(11,2,'Max Angle Limit'),
 (16,1,'Max Torque(L)'),(17,1,'Max Torque(H)'),
 (33,1,'Mode'),(40,1,'Torque Enable'),(41,1,'Acc'),(42,2,'Goal Position'),
 (44,2,'Goal Time'),(46,2,'Goal Speed'),(48,2,'Torque Limit'),(55,1,'Lock'),
 (56,2,'Present Position'),(58,2,'Present Speed'),(60,2,'Present Load'),
 (62,1,'Voltage'),(63,1,'Temp'),(65,1,'Error'),(66,1,'Moving'),
]

port=PortHandler(PORT)
if not port.openPort(): print("포트 실패"); raise SystemExit
port.setBaudRate(BAUD)
packet=PacketHandler(0)

for i in IDS:
    model,res,_=packet.ping(port,i)
    if res!=COMM_SUCCESS:
        print(f"ID {i}: 응답 없음"); continue
    print(f"\n=== ID {i} (model {model}) ===")
    for addr,size,name in REGS:
        if size==1: v,r,_=packet.read1ByteTxRx(port,i,addr)
        else:       v,r,_=packet.read2ByteTxRx(port,i,addr)
        print(f"  [{addr:3d}] {name:20s} = {v if r==COMM_SUCCESS else '읽기실패'}")
port.closePort()
print("\n(읽기만 했습니다 - 모터 상태 변경 없음)")
