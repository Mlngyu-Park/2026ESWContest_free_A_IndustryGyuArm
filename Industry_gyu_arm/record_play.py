#!/usr/bin/env python3
"""
로봇팔 Record & Play (티칭).
  record : 토크 OFF -> 손으로 움직이는 동안 관절각 녹화 -> 파일 저장
  play   : 저장된 궤적을 안전속도로 재생 (팔이 따라 움직임)

사용:
  python record_play.py record [파일명]     # 기본 motion.json
  python record_play.py play   [파일명]

record: Enter로 녹화 시작, 다시 Enter(또는 Ctrl+C)로 종료.
녹화 중 손으로 팔을 천천히 움직이면 됨.
"""
import sys, time, json, signal, threading
from dynamixel_sdk import PortHandler, PacketHandler

sys.stdout.reconfigure(line_buffering=True)

PORT='/dev/ttyUSB0'; BAUD=57600
ADDR_TORQUE=64; ADDR_P_ACCEL=108; ADDR_P_VEL=112
ADDR_GOAL=116; ADDR_PRESENT=132
# 6개 모터 전부 (팔 5 + 그리퍼 1)
MOTORS=[0,1,2,3,4,5]
RECORD_HZ=20            # 녹화 주기
PLAY_VEL=40; PLAY_ACCEL=15

def s32(v): return v-0x100000000 if v>0x7FFFFFFF else v

port=PortHandler(PORT); packet=PacketHandler(2.0)
if not port.openPort(): print("포트 실패"); raise SystemExit
port.setBaudRate(BAUD)

def get_all():
    pos=[]
    for i in MOTORS:
        p,r,_=packet.read4ByteTxRx(port,i,ADDR_PRESENT)
        pos.append(s32(p) if r==0 else None)
    return pos

def set_torque(on):
    for i in MOTORS:
        packet.write1ByteTxRx(port,i,ADDR_TORQUE,1 if on else 0)


def do_record(fname):
    set_torque(False)
    print("토크 OFF. 팔을 손으로 잡으세요.")
    input("준비되면 Enter → 녹화 시작...")
    print(f"녹화 중! ({RECORD_HZ}Hz) 손으로 움직이세요. 다시 Enter(빈 줄) 또는 Ctrl+C로 종료.")

    frames=[]
    stop=threading.Event()
    def wait_enter():
        try: input()
        except EOFError: pass
        stop.set()
    threading.Thread(target=wait_enter, daemon=True).start()

    t0=time.time()
    try:
        while not stop.is_set():
            pos=get_all()
            if all(p is not None for p in pos):
                frames.append({"t": round(time.time()-t0,3), "pos": pos})
            time.sleep(1.0/RECORD_HZ)
    except KeyboardInterrupt:
        pass

    with open(fname,'w') as f:
        json.dump({"motors":MOTORS,"frames":frames}, f)
    print(f"\n녹화 완료: {len(frames)} 프레임 → {fname}  (총 {frames[-1]['t'] if frames else 0:.1f}초)")


def do_play(fname):
    with open(fname) as f:
        data=json.load(f)
    frames=data["frames"]
    if not frames:
        print("빈 궤적"); return
    print(f"재생: {len(frames)} 프레임, {frames[-1]['t']:.1f}초")

    # 안전속도/가감속 + 토크 ON
    for i in MOTORS:
        packet.write4ByteTxRx(port,i,ADDR_P_ACCEL,PLAY_ACCEL)
        packet.write4ByteTxRx(port,i,ADDR_P_VEL,PLAY_VEL)
        packet.write1ByteTxRx(port,i,ADDR_TORQUE,1)

    # 첫 프레임으로 먼저 이동 (급출발 방지)
    print("시작 위치로 이동 중...")
    first=frames[0]["pos"]
    for idx,i in enumerate(MOTORS):
        packet.write4ByteTxRx(port,i,ADDR_GOAL, first[idx]&0xFFFFFFFF)
    time.sleep(2.0)

    print("재생 시작! (Ctrl+C 중단)")
    t0=time.time()
    try:
        for fr in frames:
            # 녹화 타임스탬프에 맞춰 전송
            dt=fr["t"]-(time.time()-t0)
            if dt>0: time.sleep(dt)
            for idx,i in enumerate(MOTORS):
                packet.write4ByteTxRx(port,i,ADDR_GOAL, fr["pos"][idx]&0xFFFFFFFF)
        print("재생 완료")
    except KeyboardInterrupt:
        print("\n중단")


if __name__=='__main__':
    mode = sys.argv[1] if len(sys.argv)>1 else ""
    fname = sys.argv[2] if len(sys.argv)>2 else "motion.json"
    try:
        if mode=="record": do_record(fname)
        elif mode=="play": do_play(fname)
        else:
            print("사용법: python record_play.py [record|play] [파일명]")
    finally:
        port.closePort()
