#!/usr/bin/env python3
"""j2(ID1) 정밀 진단: Operating Mode / Homing Offset / Pos Limit / Present Position."""
from dynamixel_sdk import PortHandler, PacketHandler

port = PortHandler('/dev/ttyUSB0')
packet = PacketHandler(2.0)
port.openPort()
port.setBaudRate(57600)


def s32(v):
    return v - 0x100000000 if v > 0x7FFFFFFF else v


ID = 1  # j2
mode_map = {1: 'Velocity', 3: 'Position(단일 0~4095)', 4: 'Extended(멀티턴)',
            5: 'Current-Pos', 16: 'PWM'}

fields = [
    ("Operating Mode",   11, 1),
    ("Homing Offset",    20, 4),
    ("Max Pos Limit",    48, 4),
    ("Min Pos Limit",    52, 4),
    ("Torque Enable",    64, 1),
    ("Present Position", 132, 4),
]

print(f"===== j2 (ID {ID}) 정밀 진단 =====")
for name, addr, size in fields:
    if size == 1:
        v, r, _ = packet.read1ByteTxRx(port, ID, addr)
    elif size == 2:
        v, r, _ = packet.read2ByteTxRx(port, ID, addr)
    else:
        v, r, _ = packet.read4ByteTxRx(port, ID, addr)
    if r != 0:
        print(f"  {name:<18}: 읽기실패 (r={r})")
    else:
        val = s32(v)
        extra = ''
        if name == "Operating Mode":
            extra = '  -> ' + mode_map.get(val, f'알수없음({val})')
        print(f"  {name:<18}: {val}{extra}")

port.closePort()
