#!/usr/bin/env python3
"""Dynamixel 스캔: ID/모델 확인.  usbipd 로 U2D2 연결 후 실행.
사용: conda run -n gyu_arm python dxl_scan.py"""
from dynamixel_sdk import PortHandler, PacketHandler
from dxl_profile import PORT, BAUD

MODEL={1020:'XM430', 1080:'XC430', 1060:'XL430'}
p=PortHandler(PORT)
if not p.openPort(): raise SystemExit(f"포트 열기 실패: {PORT} (usbipd 연결 확인)")
p.setBaudRate(BAUD)
pk=PacketHandler(2.0)
print(f"{PORT} @ {BAUD} 스캔...")
found=0
for i in range(10):
    model,r,_=pk.ping(p,i)
    if r==0:
        pos,_,_=pk.read4ByteTxRx(p,i,132)
        print(f"  ID {i}: {MODEL.get(model,model)}  현재위치 raw={pos}")
        found+=1
print(f"{found}개 발견" if found else "모터 없음 - 전원/케이블/usbipd 확인")
p.closePort()
