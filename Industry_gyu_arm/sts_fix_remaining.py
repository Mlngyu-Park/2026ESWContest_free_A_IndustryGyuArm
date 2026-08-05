#!/usr/bin/env python3
"""
ID1, ID3 오프셋 적용 (규약 확정판: 오프셋 변화 = -이동량).
- ID1: 범위 -971~1759 -> +1654 이동 (offset 85 -> -1569)
- ID3: 처짐이 4095 경계를 넘는 것 확인됨. 범위를 2642~4850 으로 보고
       -1698 이동 (offset 85 -> +1783)
- 적용 전 팔이 흔들리지 않는지 확인(정지 검사), 적용 후 이동량 검증
- 토크 절대 안 켬. 한계 설정은 재측정 후 별도로.

실행: python sts_fix_remaining.py   (팔을 건드리지 말고 실행)
"""
import sys, time, signal
import scservo_sdk.port_handler as _ph
_ph.LATENCY_TIMER=16
from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS

sys.stdout.reconfigure(line_buffering=True)
PORT='/dev/ttyACM0'; BAUD=1000000
ADDR_TORQUE=40; ADDR_PRESENT=56; ADDR_OFS=31; ADDR_LOCK=55
ADDR_MIN_ANGLE=9; ADDR_MAX_ANGLE=11

# (모터ID, 기대 원래 offset, 목표 offset, 기대 이동량)
PLAN=[
    (1, 85, -1569, +1654),
    (3, 85, +1783, -1698),
]

port=PortHandler(PORT)
if not port.openPort(): print("포트 실패"); raise SystemExit
port.setBaudRate(BAUD)
packet=PacketHandler(0)

def rd(i,addr,size=2,tries=4):
    for _ in range(tries):
        try:
            time.sleep(0.05)          # 응답 밀림 방지
            v,r,_=(packet.read2ByteTxRx(port,i,addr) if size==2
                   else packet.read1ByteTxRx(port,i,addr))
            if r==COMM_SUCCESS: return v
        except Exception: pass
        time.sleep(0.05)
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
    for i,_,_,_ in PLAN: wr1(i,ADDR_TORQUE,0)
    try: port.closePort()
    except Exception: pass
    sys.exit(0)
signal.signal(signal.SIGINT,bye)

ok_all=True
for i,ofs_expect,ofs_new,move_expect in PLAN:
    wr1(i,ADDR_TORQUE,0)
    ofs_cur=dec_ofs(rd(i,ADDR_OFS) or 0)
    print(f"\nID{i}: 현재 offset {ofs_cur:+d}")
    if ofs_cur==ofs_new:
        print(f"  이미 적용됨 ({ofs_new:+d}) - 통과"); continue
    if ofs_cur!=ofs_expect:
        print(f"  주의: 예상 offset({ofs_expect:+d})과 다름. 예상값으로 복구 후 진행.")
        wr1(i,ADDR_LOCK,0); time.sleep(0.03)
        wr2(i,ADDR_OFS,enc_ofs(ofs_expect)); time.sleep(0.1)
        wr1(i,ADDR_LOCK,1); time.sleep(0.03)
    # 정지 검사: 0.6초 동안 흔들림 < 12 raw
    reads=[]
    for _ in range(6):
        p=rd(i,ADDR_PRESENT)
        if p is not None: reads.append(p)
        time.sleep(0.1)
    if len(reads)<4 or max(reads)-min(reads)>12:
        print(f"  팔이 흔들립니다 ({reads}) - 이 관절 건너뜀. 팔을 고정하고 재실행하세요.")
        ok_all=False; continue
    p_before=reads[-1]
    # 적용
    wr1(i,ADDR_LOCK,0); time.sleep(0.03)
    if not wr2(i,ADDR_OFS,enc_ofs(ofs_new)):
        print("  쓰기 실패"); ok_all=False; continue
    time.sleep(0.12)
    p_after=rd(i,ADDR_PRESENT)
    wr1(i,ADDR_LOCK,1); time.sleep(0.03)
    d=((p_after-p_before+2048)%4096)-2048 if p_after is not None else 9999
    if abs(d-move_expect)<=150:
        print(f"  성공: offset {ofs_expect:+d} -> {ofs_new:+d}, pos {p_before} -> {p_after} (이동 {d:+d})")
    else:
        print(f"  이동량 불일치 (관측 {d:+d}, 기대 {move_expect:+d}) - 원복")
        wr1(i,ADDR_LOCK,0); wr2(i,ADDR_OFS,enc_ofs(ofs_expect)); wr1(i,ADDR_LOCK,1)
        ok_all=False; continue
    # 낡은/잘못된 각도한계 해제 (재측정 후 다시 설정할 것)
    wr1(i,ADDR_LOCK,0); time.sleep(0.03)
    wr2(i,ADDR_MIN_ANGLE,0); time.sleep(0.03)
    wr2(i,ADDR_MAX_ANGLE,4095); time.sleep(0.03)
    wr1(i,ADDR_LOCK,1)
    print(f"  각도한계 임시 해제 (재측정 후 재설정 예정)")

print("\n"+("전체 성공 - 다음: python sts_measure_limits.py 로 전 관절 재측정"
       if ok_all else "일부 실패 - 출력을 알려주세요"))
bye()
