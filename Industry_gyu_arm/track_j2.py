#!/usr/bin/env python3
"""
j2(ID1) 실시간 추적. 손으로 j2를 돌리며 경계(0/4095)에서 벗어나게 한다.
목표: raw가 1500~2500 (중앙 근처)로 오면 OK. Ctrl+C 종료.
"""
import time
from dynamixel_sdk import PortHandler, PacketHandler

port = PortHandler('/dev/ttyUSB0')
packet = PacketHandler(2.0)
port.openPort()
port.setBaudRate(57600)


def s32(v):
    return v - 0x100000000 if v > 0x7FFFFFFF else v


print('j2 실시간 추적 — j2를 손으로 천천히 돌리세요. (Ctrl+C 종료)')
print('목표: raw 1500~2500 (중앙). 아래 OK가 뜨면 멈추세요.')
print('=' * 50)
try:
    while True:
        p, r, _ = packet.read4ByteTxRx(port, 1, 132)
        if r == 0:
            p = s32(p)
            norm = p % 4096
            if 1500 <= norm <= 2500:
                status = '  ✅ OK! (중앙권, 여기서 멈춰도 됨)'
            elif norm >= 3950 or norm <= 150:
                status = '  ⚠️ 경계! 더 돌리세요'
            else:
                status = '  (이동중)'
            print(f'\r  j2 raw = {norm:5d}{status}        ', end='', flush=True)
        time.sleep(0.1)
except KeyboardInterrupt:
    print('\n종료')
finally:
    port.closePort()
