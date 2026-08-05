#!/usr/bin/env python3
"""
그리퍼(ID6) 열기/닫기 버튼 UI. 각도 개념 없이 두 위치만 사용.

처음 실행 시 sts_gripper.py 가 없으면 설정 모드:
  - 한계 임시 해제 -> 손으로 닫은 상태 [닫힘 기록] -> 연 상태 [열림 기록]
설정 후에는 [열기] [닫기] 버튼으로 제어.

안전: 토크는 버튼을 누를 때만 켜짐. [토크 끄기] 로 언제든 해제.
실행: python sts_gripper_ui.py
"""
import sys, os, time, threading
import tkinter as tk
from tkinter import messagebox
import scservo_sdk.port_handler as _ph
_ph.LATENCY_TIMER=16
from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS

PORT='/dev/ttyACM0'; BAUD=1000000
GID=6
ADDR_TORQUE=40; ADDR_ACC=41; ADDR_GOAL=42; ADDR_SPEED=46
ADDR_TORQUE_LIMIT=48; ADDR_PRESENT=56; ADDR_MODE=33
ADDR_MIN_ANGLE=9; ADDR_MAX_ANGLE=11; ADDR_LOCK=55
CFG='/home/drboom/2026_project/Industry_gyu_arm/sts_gripper.py'
MARGIN=15
SPEED=300; ACC=10; TQ_LIMIT=250      # 과열 방지 (한계에 막히면 계속 힘을 쓰므로 낮게)

port=PortHandler(PORT)
if not port.openPort():
    print(f"포트 실패: {PORT}"); raise SystemExit
port.setBaudRate(BAUD)
packet=PacketHandler(0)

def rd(addr,tries=4):
    for _ in range(tries):
        try:
            time.sleep(0.05)
            v,r,_=packet.read2ByteTxRx(port,GID,addr)
            if r==COMM_SUCCESS: return v
        except Exception: pass
        time.sleep(0.04)
    return None
def wr2(addr,v):
    for _ in range(4):
        try:
            if packet.write2ByteTxRx(port,GID,addr,int(v))[0]==COMM_SUCCESS: return True
        except Exception: pass
        time.sleep(0.03)
    return False
def wr1(addr,v):
    for _ in range(4):
        try:
            if packet.write1ByteTxRx(port,GID,addr,int(v))[0]==COMM_SUCCESS: return True
        except Exception: pass
        time.sleep(0.03)
    return False

def load_cfg():
    if not os.path.exists(CFG): return None
    ns={}
    exec(open(CFG).read(),ns)
    return ns.get('GRIPPER_CLOSE'), ns.get('GRIPPER_OPEN'), ns.get('GRIPPER_LIMIT')
def save_cfg(c,o):
    lo=max(0,min(c,o)-MARGIN); hi=min(4095,max(c,o)+MARGIN)
    open(CFG,'w').write("# sts_gripper_ui.py 로 저장한 그리퍼 위치 (ID6)\n"
        f"GRIPPER_CLOSE={c}\nGRIPPER_OPEN={o}\nGRIPPER_LIMIT=({lo}, {hi})\n")
    wr1(ADDR_LOCK,0); time.sleep(0.04)
    wr2(ADDR_MIN_ANGLE,lo); time.sleep(0.04)
    wr2(ADDR_MAX_ANGLE,hi); time.sleep(0.04)
    wr1(ADDR_LOCK,1); time.sleep(0.04)
    return lo,hi

wr1(ADDR_TORQUE,0)
cfg=load_cfg()

root=tk.Tk()
root.title("Gripper Control (ID6)")
root.geometry("360x330")
root.configure(bg="#d9d9d9")

pos_var=tk.StringVar(value="pos: --")
msg_var=tk.StringVar(value="")
torque_var=tk.StringVar(value="토크: OFF")

def poll():
    p=rd(ADDR_PRESENT)
    if p is not None: pos_var.set(f"현재 위치: {p}")
    root.after(300,poll)

def set_torque(on):
    if on:
        wr1(ADDR_MODE,0)
        wr2(ADDR_TORQUE_LIMIT,TQ_LIMIT)
        wr1(ADDR_ACC,ACC)
        wr2(ADDR_SPEED,SPEED)
    wr1(ADDR_TORQUE,1 if on else 0)
    torque_var.set("토크: ON" if on else "토크: OFF")

def goto(v,label):
    set_torque(True)
    wr2(ADDR_GOAL,v)
    msg_var.set(f"{label} -> {v}")

# ---------- 설정 모드 ----------
setup_state={'close':None,'open':None}
def start_setup():
    wr1(ADDR_TORQUE,0)
    wr1(ADDR_LOCK,0); time.sleep(0.04)
    wr2(ADDR_MIN_ANGLE,0); time.sleep(0.04)
    wr2(ADDR_MAX_ANGLE,4095); time.sleep(0.04)
    wr1(ADDR_LOCK,1); time.sleep(0.04)
    torque_var.set("토크: OFF (설정 모드)")
    msg_var.set("한계 해제됨. 손으로 그리퍼를 움직이세요.")
def rec_close():
    p=rd(ADDR_PRESENT)
    if p is None: msg_var.set("읽기 실패"); return
    setup_state['close']=p; msg_var.set(f"닫힘 기록: {p}")
def rec_open():
    p=rd(ADDR_PRESENT)
    if p is None: msg_var.set("읽기 실패"); return
    setup_state['open']=p; msg_var.set(f"열림 기록: {p}")
def finish_setup():
    c,o=setup_state['close'],setup_state['open']
    if c is None or o is None:
        messagebox.showwarning("설정","닫힘과 열림을 모두 기록하세요"); return
    if abs(c-o)<50:
        messagebox.showwarning("설정","두 위치가 너무 가깝습니다"); return
    lo,hi=save_cfg(c,o)
    messagebox.showinfo("설정 완료",f"CLOSE={c}\nOPEN={o}\n한계={lo}~{hi}\n\n제어 화면으로 전환합니다.")
    build_control(c,o)

def build_setup():
    for w in frame.winfo_children(): w.destroy()
    tk.Label(frame,text="그리퍼 설정",bg="#d9d9d9",font=("",12,"bold")).pack(pady=4)
    tk.Label(frame,text="1) [한계 해제] 후 손으로 움직입니다\n"
                        "2) 완전히 닫고 [닫힘 기록]\n"
                        "3) 완전히 열고 [열림 기록]\n"
                        "4) [설정 완료]",
             bg="#d9d9d9",justify="left").pack(pady=4)
    tk.Button(frame,text="한계 해제 (시작)",command=start_setup,width=24).pack(pady=3)
    tk.Button(frame,text="닫힘 기록",command=rec_close,width=24).pack(pady=3)
    tk.Button(frame,text="열림 기록",command=rec_open,width=24).pack(pady=3)
    tk.Button(frame,text="설정 완료",command=finish_setup,width=24,
              bg="#3a7",fg="white").pack(pady=6)

# ---------- 제어 모드 ----------
def build_control(c,o):
    for w in frame.winfo_children(): w.destroy()
    tk.Label(frame,text="그리퍼 제어",bg="#d9d9d9",font=("",12,"bold")).pack(pady=4)
    tk.Label(frame,text=f"닫힘={c}   열림={o}",bg="#d9d9d9").pack()
    tk.Button(frame,text="열 기  (OPEN)",command=lambda:goto(o,"열기"),
              width=22,height=2,bg="#4a8fd6",fg="white",
              font=("",11,"bold")).pack(pady=6)
    tk.Button(frame,text="닫 기  (CLOSE)",command=lambda:goto(c,"닫기"),
              width=22,height=2,bg="#d67a4a",fg="white",
              font=("",11,"bold")).pack(pady=6)
    tk.Button(frame,text="토크 끄기 (손으로 움직이기)",
              command=lambda:set_torque(False),width=24).pack(pady=4)
    tk.Button(frame,text="다시 설정",command=build_setup,width=24).pack(pady=2)

frame=tk.Frame(root,bg="#d9d9d9"); frame.pack(fill="both",expand=True,padx=8,pady=6)
tk.Label(root,textvariable=pos_var,bg="#d9d9d9").pack()
tk.Label(root,textvariable=torque_var,bg="#d9d9d9").pack()
tk.Label(root,textvariable=msg_var,bg="#d9d9d9",fg="#333").pack(pady=2)

if cfg and cfg[0] is not None:
    build_control(cfg[0],cfg[1])
    msg_var.set("설정 불러옴")
else:
    build_setup()

def on_close():
    wr1(ADDR_TORQUE,0)
    try: port.closePort()
    except Exception: pass
    root.destroy()
root.protocol("WM_DELETE_WINDOW",on_close)
poll()
root.mainloop()
