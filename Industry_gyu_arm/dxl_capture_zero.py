#!/usr/bin/env python3
"""Dynamixel 영점(base 자세) 캡처.
토크를 끄니 손으로 로봇을 base 자세(그리퍼 수평!)로 잡고 Enter.
결과는 dxl_zero.py 에 저장된다.
사용: conda run -n gyu_arm python dxl_capture_zero.py"""
import time
from dynamixel_sdk import PortHandler, PacketHandler
from dxl_profile import PORT, BAUD, J2ID

p=PortHandler(PORT)
if not p.openPort(): raise SystemExit(f"포트 열기 실패: {PORT}")
p.setBaudRate(BAUD)
pk=PacketHandler(2.0)

ids=sorted(set(J2ID))
for i in ids: pk.write1ByteTxRx(p,i,64,0)          # 토크 OFF
print("토크 OFF - 손으로 로봇을 base 자세로 잡으세요.")
print("  * 그리퍼가 지면과 수평이 되게! (물컵 규칙의 기준 자세)")
try:
    while True:
        vals={}
        for i in ids:
            v,r,_=pk.read4ByteTxRx(p,i,132)
            vals[i]=v if r==0 else '?'
        print('  '+' '.join(f"ID{i}:{vals[i]}" for i in ids),end='\r')
        time.sleep(0.15)
except KeyboardInterrupt:
    pass
input("\n자세를 잡았으면 Enter (Ctrl+C 는 취소): ")
zero={}
for i in ids:
    v,r,_=pk.read4ByteTxRx(p,i,132)
    if r!=0: raise SystemExit(f"ID{i} 읽기 실패 - 다시 시도")
    zero[i]=int(v)
with open(__file__.replace('dxl_capture_zero.py','dxl_zero.py'),'w') as f:
    f.write("# Dynamixel(옛 로봇) 영점 raw - dxl_capture_zero.py 로 캡처됨\n")
    f.write(f"# 캡처 시각: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
    f.write(f"ZERO_RAW={zero}\n")
    f.write("VERIFIED=True\n")
print(f"저장됨: dxl_zero.py  {zero}")
p.closePort()
