#!/usr/bin/env python3
"""
Feetech STS3215 모터 ID 부여 (EEPROM 저장, 전원 꺼도 유지).

사용법:
  1) 보드에 모터를 '하나만' 연결 (다른 모터는 케이블 분리)
  2) conda run --no-capture-output -n gyu_arm python sts_set_id.py <새ID>
     예: python sts_set_id.py 6        # 그리퍼를 ID 6으로
  3) 다음 모터로 케이블 옮기고 반복

주의: 버스에 모터가 2개 이상이면 실행 거부 (충돌 방지).
SO-ARM101 ID 규칙: 1=어깨회전 2=어깨상하 3=팔꿈치 4=손목상하 5=손목회전 6=그리퍼
"""
import sys, time
import scservo_sdk.port_handler as _ph
_ph.LATENCY_TIMER=16         # SDK 기본값 (안정)
from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS

PORT='/dev/ttyACM0'; BAUD=1000000
ADDR_LOCK=55           # 0=EEPROM 쓰기 잠금해제, 1=잠금
ADDR_ID=5              # 1byte
ADDR_PRESENT_POSITION=56

if len(sys.argv)<2:
    print(__doc__); raise SystemExit
try:
    NEW_ID=int(sys.argv[1])
except ValueError:
    print("새 ID는 숫자여야 합니다 (1~6)"); raise SystemExit
if not (0<=NEW_ID<=253):
    print("ID 범위 오류 (1~6 권장)"); raise SystemExit

sys.stdout.reconfigure(line_buffering=True)
port=PortHandler(PORT)
if not port.openPort(): print(f"포트 실패: {PORT}"); raise SystemExit
port.setBaudRate(BAUD)
packet=PacketHandler(0)

# 1) 버스 스캔 (1~10만, 빠르게)
print("스캔 중...")
found=[]
for i in range(1,11):
    model,res,err=packet.ping(port,i)
    if res==COMM_SUCCESS: found.append((i,model))

if not found:
    print("모터를 못 찾았습니다. 케이블/전원 확인하세요."); port.closePort(); raise SystemExit
if len(found)>1:
    print(f"버스에 모터가 {len(found)}개 있습니다: {[i for i,_ in found]}")
    print("ID 부여는 '한 개만' 연결한 상태에서 하세요. 중단합니다.")
    port.closePort(); raise SystemExit

old_id,model=found[0]
pos,_,_=packet.read2ByteTxRx(port,old_id,ADDR_PRESENT_POSITION)
print(f"발견: ID {old_id} (model {model}, pos {pos})")

if old_id==NEW_ID:
    print(f"이미 ID {NEW_ID} 입니다. 변경 불필요.")
    port.closePort(); raise SystemExit

# 2) EEPROM 잠금 해제 -> ID 변경 -> 잠금
print(f"ID {old_id} -> {NEW_ID} 로 변경 중...")
r,_=packet.write1ByteTxRx(port,old_id,ADDR_LOCK,0)      # unlock
if r!=COMM_SUCCESS: print("EEPROM 잠금해제 실패"); port.closePort(); raise SystemExit
time.sleep(0.05)
r,_=packet.write1ByteTxRx(port,old_id,ADDR_ID,NEW_ID)   # write new id
time.sleep(0.1)
r2,_=packet.write1ByteTxRx(port,NEW_ID,ADDR_LOCK,1)     # lock (새 ID로)
time.sleep(0.05)

# 3) 검증
model2,res,err=packet.ping(port,NEW_ID)
if res==COMM_SUCCESS:
    pos2,_,_=packet.read2ByteTxRx(port,NEW_ID,ADDR_PRESENT_POSITION)
    print(f"성공: 이제 ID {NEW_ID} (model {model2}, pos {pos2})")
    print("EEPROM에 저장됨 - 전원 꺼도 유지됩니다.")
else:
    print("검증 실패: 새 ID로 응답이 없습니다. 케이블/전원 확인 후 재시도하세요.")
port.closePort()
