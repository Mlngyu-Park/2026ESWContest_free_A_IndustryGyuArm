#!/usr/bin/env python3
"""
Feetech STS3215 모터 스캔 (읽기 전용, 안전).
- 여러 baudrate로 ID 0~20 핑 -> 발견 시 모델/위치/전압/온도 출력

실행: conda run --no-capture-output -n gyu_arm python sts_scan.py
"""
import sys
import scservo_sdk.port_handler as _ph
_ph.LATENCY_TIMER=16         # SDK 기본값 (안정)
from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS

PORT='/dev/ttyACM0'
BAUDS=[1000000]              # STS3215 기본. 다른 baud도 볼 땐 아래 줄로 교체
# BAUDS=[1000000, 500000, 115200, 57600, 250000, 128000, 38400]
ID_RANGE=range(1,11)

# STS/SMS 레지스터 (Feetech STS 시리즈)
ADDR_PRESENT_POSITION=56    # 2byte
ADDR_PRESENT_VOLTAGE=62     # 1byte (0.1V)
ADDR_PRESENT_TEMP=63        # 1byte (C)

sys.stdout.reconfigure(line_buffering=True)
port=PortHandler(PORT)
if not port.openPort():
    print(f"포트 열기 실패: {PORT}"); raise SystemExit
packet=PacketHandler(0)     # protocol_end=0 (STS/SMS는 little endian)

print(f"포트 {PORT} 열림. 스캔 시작...\n")
found={}
for baud in BAUDS:
    if not port.setBaudRate(baud):
        print(f"baudrate {baud}: 설정 실패"); continue
    hits=[]
    for i in ID_RANGE:
        model,res,err=packet.ping(port,i)
        if res==COMM_SUCCESS:
            hits.append((i,model))
    if hits:
        found[baud]=hits
        print(f"=== baudrate {baud} : {len(hits)}개 발견 ===")
        for i,model in hits:
            pos,r1,_=packet.read2ByteTxRx(port,i,ADDR_PRESENT_POSITION)
            volt,r2,_=packet.read1ByteTxRx(port,i,ADDR_PRESENT_VOLTAGE)
            temp,r3,_=packet.read1ByteTxRx(port,i,ADDR_PRESENT_TEMP)
            p=pos if r1==COMM_SUCCESS else '?'
            v=f"{volt/10.0:.1f}V" if r2==COMM_SUCCESS else '?'
            t=f"{temp}C" if r3==COMM_SUCCESS else '?'
            print(f"  ID {i:2d} | model {model} | pos {p} | {v} | {t}")
        print()
    else:
        print(f"baudrate {baud}: 없음")

if not found:
    print("\n모터를 찾지 못했습니다. 확인할 것:")
    print("  1) 12V 전원이 배럴잭에 연결되어 있는지 (USB만으론 모터 안 켜짐)")
    print("  2) 보드 점퍼가 'B USB = SERVO' 위치인지")
    print("  3) 모터 케이블이 보드 servo 포트(D/V/G)에 꽂혀 있는지")
else:
    print("정리:")
    for b,h in found.items():
        print(f"  baudrate {b}: ID {[i for i,_ in h]}")
port.closePort()
