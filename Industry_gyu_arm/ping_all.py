#!/usr/bin/env python3
"""버스에 있는 모터 ID를 ping으로 스캔 (ID 0~9)."""
from dynamixel_sdk import PortHandler, PacketHandler

port = PortHandler('/dev/ttyUSB0')
packet = PacketHandler(2.0)
port.openPort()
port.setBaudRate(57600)

print("ping 스캔 (ID 0~9):")
for dxl_id in range(10):
    model, r, e = packet.ping(port, dxl_id)
    if r == 0:
        print(f"  ID {dxl_id}: 응답 O  (model={model})")
    else:
        print(f"  ID {dxl_id}: 무응답")
port.closePort()
