#!/usr/bin/env python3
"""
j6(그리퍼, ID6) 0점 랩어라운드 해결 v2. 토크를 절대 켜지 않음 (안 움직임).

문제(측정 확인됨): 그리퍼 범위 3804~5309 가 0/4095 경계를 걸침
  -> 위치가 튀어 보이고, 목표를 쓰면 먼 쪽으로 크게 회전

해결:
  1) 손으로 그리퍼 왕복 -> 범위 측정 (랩 보정 추적)
  2) Position Offset(주소31, +-2047)으로 범위를 경계에서 멀리 이동
     - 기존 오프셋 반영, 부호 방향은 '써보고 검증' (틀리면 반대로 재시도)
  3) Min/Max Angle Limit 을 새 범위 +-여유로 설정 (펌웨어 차단)

실행: python sts_fix_gripper.py
"""
import sys, time, signal
import scservo_sdk.port_handler as _ph
_ph.LATENCY_TIMER=16
from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS

sys.stdout.reconfigure(line_buffering=True)
PORT='/dev/ttyACM0'; BAUD=1000000
GID=6
ADDR_TORQUE=40; ADDR_PRESENT=56
ADDR_MIN_ANGLE=9; ADDR_MAX_ANGLE=11
ADDR_OFS=31; ADDR_LOCK=55
MARGIN=40

port=PortHandler(PORT)
if not port.openPort(): print("포트 실패"); raise SystemExit
port.setBaudRate(BAUD)
packet=PacketHandler(0)

def rd(addr,size=2):
    v,r,_=(packet.read2ByteTxRx(port,GID,addr) if size==2 else packet.read1ByteTxRx(port,GID,addr))
    return v if r==COMM_SUCCESS else None
def wr2(addr,v): return packet.write2ByteTxRx(port,GID,addr,v)[0]==COMM_SUCCESS
def wr1(addr,v): return packet.write1ByteTxRx(port,GID,addr,v)[0]==COMM_SUCCESS
def dec_ofs(v):   # 부호-크기(bit11)
    return -(v&0x7FF) if (v&0x800) else (v&0x7FF)
def enc_ofs(v):
    v=int(v)
    return ((-v)&0x7FF)|0x800 if v<0 else (v&0x7FF)
def bye(*_):
    try: wr1(ADDR_TORQUE,0)
    except Exception: pass
    try: port.closePort()
    except Exception: pass
    print("\n종료 (토크 꺼진 상태)"); sys.exit(0)
signal.signal(signal.SIGINT,bye)

wr1(ADDR_TORQUE,0)
p0=rd(ADDR_PRESENT)
if p0 is None: print("ID6 응답 없음"); bye()
ofs_raw=rd(ADDR_OFS); ofs_old=dec_ofs(ofs_raw)
mn=rd(ADDR_MIN_ANGLE); mx=rd(ADDR_MAX_ANGLE)
print(f"ID6 현재: pos={p0}  offset={ofs_old:+d}(raw {ofs_raw})  limit={mn}~{mx}")
print()
print("="*56)
print(" 토크 꺼져 있습니다. 손으로 그리퍼를")
print("   '완전히 열기 <-> 완전히 닫기' 몇 번 왕복 후 Enter")
print("="*56)

import threading
stop=False
last=p0; unwrapped=p0; umin=umax=p0
def track():
    global last,unwrapped,umin,umax
    while not stop:
        p=rd(ADDR_PRESENT)
        if p is not None:
            d=p-last
            if d>2048: d-=4096
            elif d<-2048: d+=4096
            unwrapped+=d; last=p
            umin=min(umin,unwrapped); umax=max(umax,unwrapped)
            print(f"\r  raw={p:4d}  범위=[{umin} ~ {umax}] 폭={umax-umin:4d}   ",end="",flush=True)
        time.sleep(0.05)
t=threading.Thread(target=track,daemon=True); t.start()
try: input()
except KeyboardInterrupt: bye()
stop=True; time.sleep(0.2)

span=umax-umin
print(f"\n\n측정: {umin} ~ {umax} (폭 {span} raw = {span*360/4096:.0f}도)")
if span<50: print("범위가 너무 좁습니다. 다시."); bye()
if span>3000: print("범위 비정상. 다시."); bye()

# 현재 위치(unwrapped) 기준 상대 범위
p_cur=rd(ADDR_PRESENT)
rel_min=umin-unwrapped; rel_max=umax-unwrapped   # rel_min<=0<=rel_max

# 이동량 s: 보고 위치가 s 만큼 이동. 조건:
#   p_cur+s+rel_min-MARGIN >= 10   and   p_cur+s+rel_max+MARGIN <= 4085
s_lo=10+MARGIN-rel_min-p_cur
s_hi=4085-MARGIN-rel_max-p_cur
if s_lo>s_hi: print("범위가 너무 넓어 이동 불가"); bye()
s_center=2048-(p_cur+(rel_min+rel_max)//2)
s=max(s_lo,min(s_hi,s_center))

def try_apply(cand_ofs,expect_shift):
    """오프셋 써보고 위치가 예상대로 이동했는지 확인. 성공=새위치, 실패=None"""
    wr1(ADDR_LOCK,0); time.sleep(0.05)
    wr2(ADDR_OFS,enc_ofs(cand_ofs)); time.sleep(0.1)
    p=rd(ADDR_PRESENT)
    if p is None: return None
    d=(p-p_cur+2048)%4096-2048     # 최단 차이
    return p if abs(d-expect_shift)<60 else None

# 방향 규약 A: new = old + s  /  B: new = old - s
applied=None
for name,cand,exp in [("A",ofs_old+s,s),("B",ofs_old-s,s)]:
    if abs(cand)>2047:
        print(f"규약{name}: 오프셋 {cand:+d} 한계 초과 - 건너뜀"); continue
    print(f"규약{name} 시도: offset {ofs_old:+d} -> {cand:+d} (기대 이동 {s:+d})")
    p_new=try_apply(cand,exp)
    if p_new is not None:
        applied=(cand,p_new); print(f"  성공: pos {p_cur} -> {p_new}"); break
    print("  방향 불일치 - 원복 후 다른 규약 시도")
    wr2(ADDR_OFS,enc_ofs(ofs_old)); time.sleep(0.1)

if applied is None:
    wr2(ADDR_OFS,enc_ofs(ofs_old)); wr1(ADDR_LOCK,1)
    print("\n오프셋 적용 실패 (양쪽 규약 모두). 원복했습니다.")
    print("이 경우 모터 혼을 물리적으로 다시 물리는 방법을 써야 합니다."); bye()

cand,p_new=applied
lo=max(0,min(4095,p_new+rel_min-MARGIN))
hi=max(0,min(4095,p_new+rel_max+MARGIN))
wr2(ADDR_MIN_ANGLE,lo); time.sleep(0.05)
wr2(ADDR_MAX_ANGLE,hi); time.sleep(0.05)
wr1(ADDR_LOCK,1); time.sleep(0.05)

print(f"\n=== 적용 완료 ===")
print(f"  offset: {ofs_old:+d} -> {cand:+d}")
print(f"  pos:    {p_cur} -> {p_new}")
print(f"  limit:  {lo} ~ {hi}")
ok = 10<=lo<hi<=4085 and lo<p_new<hi
print("검증:", "성공 - 경계에서 벗어났고 한계도 걸렸습니다." if ok else "결과 확인 필요 - 값을 알려주세요.")
print("이제 손으로 그리퍼를 움직여 raw 가 lo~hi 안에서만 변하는지 확인해보세요:")
print("  python sts_fix_gripper.py  를 다시 실행해 측정 단계에서 확인 (적용 전 Ctrl+C)")
bye()
