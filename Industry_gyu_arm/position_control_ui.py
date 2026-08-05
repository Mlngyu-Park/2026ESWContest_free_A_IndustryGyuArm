#!/usr/bin/env python3
"""
Position 제어 UI (RViz).
- base / 1st / 2nd / 3rd 버튼: 그 자세로 부드럽게 이동
- j1~j6 슬라이더: 각 관절 직접 조절
- 시퀀스 재생: base->1->base->2->base->3 반복
/joint_states 발행 → RViz 로봇이 따라 움직임.

실행: python position_control_ui.py
"""
import sys, time, math, threading
import tkinter as tk
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINT_NAMES=["j1","j2","j3","j4","j5","j6"]

# IK로 구한 각 자세 관절각 (rad)
POSES={
    "base":[0.0,    0.0,    0.0,    0.0, 0.0,    0.0],
    "1st": [-0.26,  -1.2827,-1.8599,0.0,-0.5773,-0.26],
    "2nd": [-0.0375,-1.5374,-2.0274,0.0,-0.49,  -0.0375],
    "3rd": [0.2612, -1.2827,-1.8599,0.0,-0.5773, 0.2612],
}
SEQ=["base","1st","base","2nd","base","3rd"]

# 슬라이더 범위 (rad)
LIMITS=[(-2.0,2.0),(-2.5,1.0),(-2.5,1.0),(-2.0,2.0),(-2.0,1.5),(-2.0,2.0)]

MOVE_DUR=2.0; HZ=30

rclpy.init(); node=Node("position_control_ui")
pub=node.create_publisher(JointState,"joint_states",10)

cur=[0.0]*6           # 현재 관절각
busy=[False]          # 이동 중 플래그
seq_run=[False]       # 시퀀스 재생 중

def publish(q):
    m=JointState(); m.header.stamp=node.get_clock().now().to_msg()
    m.name=list(JOINT_NAMES); m.position=[float(x) for x in q]; pub.publish(m)

def ease(t): return t*t*(3-2*t)

def move_to(target, dur=MOVE_DUR):
    """부드럽게 target 자세로 이동 (블로킹, 스레드에서 호출)."""
    start=list(cur); n=int(dur*HZ)
    for k in range(n+1):
        t=ease(k/n)
        q=[a+(b-a)*t for a,b in zip(start,target)]
        cur[:] = q
        publish(q)
        sync_sliders()
        time.sleep(1.0/HZ)

# ---------- GUI ----------
root=tk.Tk(); root.title("Position Control")
sliders=[]; labels=[]
programmatic=[False]

def on_slide(i,v):
    if programmatic[0]: return
    cur[i]=float(v)
    labels[i].config(text=f"{cur[i]:+.2f} rad ({math.degrees(cur[i]):+.0f}deg)")
    publish(cur)

def sync_sliders():
    programmatic[0]=True
    try:
        for i in range(6):
            sliders[i].set(cur[i])
            labels[i].config(text=f"{cur[i]:+.2f} rad ({math.degrees(cur[i]):+.0f}deg)")
    finally:
        programmatic[0]=False

for i,name in enumerate(JOINT_NAMES):
    f=tk.Frame(root); f.pack(fill="x", padx=8, pady=2)
    tk.Label(f,text=name,width=3,font=("",11,"bold")).pack(side="left")
    lo,hi=LIMITS[i]
    s=tk.Scale(f,from_=lo,to=hi,resolution=0.01,orient="horizontal",length=300,
               showvalue=False,command=lambda v,i=i: on_slide(i,v))
    s.pack(side="left",padx=5); sliders.append(s)
    l=tk.Label(f,text="+0.00 rad (+0deg)",width=18); l.pack(side="left"); labels.append(l)

def goto(name):
    if busy[0] or seq_run[0]: return
    def run():
        busy[0]=True
        status.config(text=f"Moving to {name}...")
        move_to(POSES[name])
        status.config(text=f"At {name}")
        busy[0]=False
    threading.Thread(target=run,daemon=True).start()

pf=tk.Frame(root); pf.pack(pady=8)
for n in ["base","1st","2nd","3rd"]:
    tk.Button(pf,text=n.upper(),width=8,command=lambda n=n: goto(n)).pack(side="left",padx=4)

def toggle_seq():
    if busy[0]: return
    if not seq_run[0]:
        seq_run[0]=True; seqbtn.config(text="[] Stop Seq", bg="#f88")
        def run():
            while seq_run[0]:
                for name in SEQ:
                    if not seq_run[0]: break
                    status.config(text=f"Seq -> {name}")
                    move_to(POSES[name])
                    time.sleep(0.5)
            seqbtn.config(text="> Sequence", bg="#d9d9d9")
            status.config(text="Sequence stopped")
        threading.Thread(target=run,daemon=True).start()
    else:
        seq_run[0]=False

sf=tk.Frame(root); sf.pack(pady=4)
seqbtn=tk.Button(sf,text="> Sequence",width=12,command=toggle_seq); seqbtn.pack(side="left",padx=4)
tk.Button(sf,text="Publish now",width=12,command=lambda: publish(cur)).pack(side="left",padx=4)

status=tk.Label(root,text="Ready",fg="blue"); status.pack(pady=4)

# 주기 발행 (RViz 유지)
def tick():
    publish(cur)
    root.after(100,tick)
tick()

try:
    root.mainloop()
except KeyboardInterrupt:
    pass
finally:
    seq_run[0]=False
    try: node.destroy_node(); rclpy.shutdown()
    except Exception: pass
