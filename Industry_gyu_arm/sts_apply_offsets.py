#!/usr/bin/env python3
"""
측정된 한계(sts_limits.py) 기반으로:
  1) 0/4095 경계를 걸치는 모터(ID 1,4,6 등)의 Position Offset 을 조정해
     가동 범위를 경계에서 멀리 이동 (6번이 갑자기 돌던 문제의 근본 해결)
  2) 모든 모터에 펌웨어 각도 한계(Min/Max Angle) 설정
     -> 코드에 버그가 있어도 범위 밖으로 물리적으로 못 나감

토크는 절대 켜지 않음 (안 움직임). 오프셋 방향은 써보고 검증(틀리면 반대 재시도).
결과는 sts_limits_applied.py 로 저장.

실행: python sts_apply_offsets.py
"""
import sys, time, signal
import scservo_sdk.port_handler as _ph
_ph.LATENCY_TIMER=16
from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS
from sts_limits import LIMIT_RAW      # 측정값 (연속좌표)

sys.stdout.reconfigure(line_buffering=True)
PORT='/dev/ttyACM0'; BAUD=1000000
ADDR_TORQUE=40; ADDR_PRESENT=56
ADDR_MIN_ANGLE=9; ADDR_MAX_ANGLE=11
ADDR_OFS=31; ADDR_LOCK=55
IDS=[1,2,3,4,5,6]
MARGIN=30
SAFE_LO, SAFE_HI = 50, 4045          # 이동 후 범위가 들어가야 할 안전 구간

port=PortHandler(PORT)
if not port.openPort(): print("포트 실패"); raise SystemExit
port.setBaudRate(BAUD)
packet=PacketHandler(0)

def rd(i,addr,size=2,tries=4):
    for _ in range(tries):
        try:
            v,r,_=(packet.read2ByteTxRx(port,i,addr) if size==2
                   else packet.read1ByteTxRx(port,i,addr))
            if r==COMM_SUCCESS: return v
        except Exception:
            pass
        time.sleep(0.03)
    return None
def wr2(i,addr,v,tries=4):
    for _ in range(tries):
        try:
            if packet.write2ByteTxRx(port,i,addr,int(v))[0]==COMM_SUCCESS: return True
        except Exception: pass
        time.sleep(0.03)
    return False
def wr1(i,addr,v,tries=4):
    for _ in range(tries):
        try:
            if packet.write1ByteTxRx(port,i,addr,int(v))[0]==COMM_SUCCESS: return True
        except Exception: pass
        time.sleep(0.03)
    return False
def dec_ofs(v): return -(v&0x7FF) if (v&0x800) else (v&0x7FF)
def enc_ofs(v):
    v=int(v)
    return ((-v)&0x7FF)|0x800 if v<0 else (v&0x7FF)
def bye(*_):
    for i in IDS: wr1(i,ADDR_TORQUE,0)
    try: port.closePort()
    except Exception: pass
    sys.exit(0)
signal.signal(signal.SIGINT,bye)

for i in IDS: wr1(i,ADDR_TORQUE,0)   # 확실히 토크 OFF
time.sleep(0.2)

# 이전 실행(크래시)에서 ID1 오프셋이 어정쩡할 수 있음 -> 측정 당시 값(+85)으로 복구
RESET_FIRST={1:85}
for i,ov in RESET_FIRST.items():
    cur=dec_ofs(rd(i,ADDR_OFS) or 0)
    if cur!=ov:
        print(f"ID{i}: 오프셋 {cur:+d} -> {ov:+d} 복구 (이전 실행 잔재)")
        wr1(i,ADDR_LOCK,0); time.sleep(0.03)
        wr2(i,ADDR_OFS,enc_ofs(ov)); time.sleep(0.08)
        wr1(i,ADDR_LOCK,1); time.sleep(0.03)

results={}
print("모터별 처리 시작 (토크 OFF 상태, 안 움직임)\n")
for i in IDS:
    mn,mx=LIMIT_RAW[i]
    span=mx-mn
    ofs_old=dec_ofs(rd(i,ADDR_OFS))
    p_cur=rd(i,ADDR_PRESENT)
    if p_cur is None: print(f"ID{i}: 응답 없음 - 건너뜀"); continue
    crossing = mn<SAFE_LO or mx>SAFE_HI
    print(f"ID{i}: 범위 {mn}~{mx} (폭{span})  offset={ofs_old:+d}  pos={p_cur}  "
          +("[경계 걸침 -> 이동 필요]" if crossing else "[정상]"))
    s_applied=0
    if crossing:
        # 이동량 창: mn+s >= SAFE_LO, mx+s <= SAFE_HI
        s_lo=SAFE_LO-mn; s_hi=SAFE_HI-mx
        if s_lo>s_hi:
            print(f"  범위가 너무 넓어 이동 불가 - 건너뜀"); continue
        s_want=2048-(mn+mx)//2
        ok=False
        for conv in (-1,+1):    # 실측: 오프셋 +X -> 위치 -X 이동이므로 -부터 시도
            s=max(s_lo,min(s_hi,s_want))
            cand=ofs_old+conv*s
            if abs(cand)>2047:                   # 한계로 클램프 후 s 재계산
                cand=max(-2047,min(2047,cand))
                s=(cand-ofs_old)*conv
                if not (s_lo<=s<=s_hi):
                    print(f"  규약{'A' if conv>0 else 'B'}: 오프셋 한계로 불가")
                    continue
            wr1(i,ADDR_LOCK,0); time.sleep(0.03)
            wr2(i,ADDR_OFS,enc_ofs(cand)); time.sleep(0.08)
            p_new=rd(i,ADDR_PRESENT)
            d=((p_new-p_cur+2048)%4096)-2048 if p_new is not None else 9999
            if abs(d-s)<60:
                print(f"  오프셋 {ofs_old:+d} -> {cand:+d} 적용 (이동 {d:+d}, 기대 {s:+d}) OK")
                s_applied=s; ok=True; break
            # 방향 불일치 -> 원복 후 반대 규약
            wr2(i,ADDR_OFS,enc_ofs(ofs_old)); time.sleep(0.08)
            print(f"  규약{'A' if conv>0 else 'B'} 불일치(이동 {d:+d}) - 반대 시도")
        if not ok:
            wr2(i,ADDR_OFS,enc_ofs(ofs_old)); wr1(i,ADDR_LOCK,1)
            print(f"  ID{i} 오프셋 실패 - 원복. (이 관절은 나중에 혼 재조립 필요)")
            continue
    # 각도 한계 설정 (이동 반영)
    lo=max(0,min(4095,mn+s_applied-MARGIN))
    hi=max(0,min(4095,mx+s_applied+MARGIN))
    wr1(i,ADDR_LOCK,0); time.sleep(0.03)
    wr2(i,ADDR_MIN_ANGLE,lo); time.sleep(0.03)
    wr2(i,ADDR_MAX_ANGLE,hi); time.sleep(0.03)
    wr1(i,ADDR_LOCK,1); time.sleep(0.03)
    p_fin=rd(i,ADDR_PRESENT)
    results[i]=(lo,hi,p_fin)
    print(f"  한계 설정 {lo}~{hi}  현재 pos={p_fin}\n")

print("="*54)
print("최종 상태:")
lines=["# sts_apply_offsets.py 적용 후 한계 (raw)","LIMIT_RAW={"]
allok=True
for i in IDS:
    if i not in results: print(f"  ID{i}: 실패/건너뜀"); allok=False; continue
    lo,hi,p=results[i]
    inside = p is not None and lo<=p<=hi
    if not inside: allok=False
    print(f"  ID{i}: limit {lo}~{hi}  pos {p}  {'OK' if inside else '확인필요'}")
    lines.append(f"    {i}: ({lo}, {hi}),")
lines.append("}")
open('/home/drboom/2026_project/Industry_gyu_arm/sts_limits_applied.py','w').write("\n".join(lines)+"\n")
print("\n저장: sts_limits_applied.py")
print("전체 성공" if allok else "일부 실패 - 출력을 알려주세요")
bye()
