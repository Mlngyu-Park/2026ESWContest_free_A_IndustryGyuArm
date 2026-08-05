#!/usr/bin/env python3
"""전 모터 토크 즉시 OFF + 온도 확인."""
import time
import scservo_sdk.port_handler as _ph
_ph.LATENCY_TIMER=16
from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS
port=PortHandler('/dev/ttyACM0'); port.openPort(); port.setBaudRate(1000000)
p=PacketHandler(0)
for i in range(1,7):
    for _ in range(3):
        try:
            if p.write1ByteTxRx(port,i,40,0)[0]==COMM_SUCCESS: break
        except Exception: pass
        time.sleep(0.03)
print("전 모터 토크 OFF 완료\n")
print("ID  위치   전압    온도")
for i in range(1,7):
    def r(a,sz=2):
        for _ in range(4):
            time.sleep(0.05)
            try:
                v,rr,_=(p.read2ByteTxRx(port,i,a) if sz==2 else p.read1ByteTxRx(port,i,a))
                if rr==COMM_SUCCESS: return v
            except Exception: pass
            time.sleep(0.04)
        return None
    pos=r(56); volt=r(62,1); temp=r(63,1); load=r(60)
    hot=" <-- 뜨거움!" if (temp is not None and temp>=55) else ""
    print(f"{i}   {pos}   {volt/10.0 if volt else '?'}V   {temp}C{hot}")
port.closePort()
