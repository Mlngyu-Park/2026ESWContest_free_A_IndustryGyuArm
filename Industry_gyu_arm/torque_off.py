#!/usr/bin/env python3
"""6개 모터 토크 OFF (손으로 돌릴 수 있게)."""
from dynamixel_sdk import PortHandler, PacketHandler
port=PortHandler('/dev/ttyUSB0'); packet=PacketHandler(2.0)
port.openPort(); port.setBaudRate(57600)
for i in range(6):
    packet.write1ByteTxRx(port,i,64,0)
print("토크 OFF (ID 0~5). 손으로 base 자세 맞추세요.")
port.closePort()
