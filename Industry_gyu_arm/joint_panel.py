#!/usr/bin/env python3
"""
관절 제어 통합 패널 (한 창).
  - j1~j5 : 슬라이더
  - j6    : 열기/닫기 버튼 (각도 대신 두 위치)

발행:
  /joint_states   (JointState)  -> RViz 및 브리지
  /gripper_cmd    (String)      -> 브리지가 실물 그리퍼 제어

joint_state_publisher_gui 를 대체하므로 launch 에서 그것과 j6_merger 는 제외.
실행(단독): python joint_panel.py
"""
import sys, math, threading
import tkinter as tk
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

JOINTS=['j1','j2','j3','j4','j5','j6']
SLIDER_JOINTS=['j1','j2','j3','j4','j5']
LIMIT_DEG=(-92.0, 92.0)           # URDF ±1.6rad

# 실측 기반 j6 각도 (raw 840 닫힘 -> 0, raw 2060 열림 -> -1.8715)
J6_CLOSE_RAD=0.0
J6_OPEN_RAD=-1.8715

class Panel(Node):
    def __init__(self):
        super().__init__('joint_panel')
        self.js=self.create_publisher(JointState,'joint_states',10)
        self.cmd=self.create_publisher(String,'gripper_cmd',10)
        self.q={j:0.0 for j in JOINTS}
        self.q['j6']=J6_CLOSE_RAD
    def publish(self):
        m=JointState()
        m.header.stamp=self.get_clock().now().to_msg()
        m.name=list(JOINTS)
        m.position=[float(self.q[j]) for j in JOINTS]
        self.js.publish(m)
    def send_cmd(self,c):
        s=String(); s.data=c; self.cmd.publish(s)

def main():
    rclpy.init()
    node=Panel()
    threading.Thread(target=lambda: rclpy.spin(node),daemon=True).start()

    win=tk.Tk()
    win.title("Joint Control")
    win.geometry("360x560")
    win.configure(bg="#f0f0f0")
    msg=tk.StringVar(value="")

    # ---- j1~j5 슬라이더 ----
    tk.Label(win,text="관절 제어",bg="#f0f0f0",font=("",13,"bold")).pack(pady=6)
    body=tk.Frame(win,bg="#f0f0f0"); body.pack(fill="x",padx=10)
    vars={}
    def on_slide(j,val):
        node.q[j]=math.radians(float(val))
        node.publish()
    for j in SLIDER_JOINTS:
        row=tk.Frame(body,bg="#f0f0f0"); row.pack(fill="x",pady=3)
        top=tk.Frame(row,bg="#f0f0f0"); top.pack(fill="x")
        tk.Label(top,text=j,bg="#f0f0f0",width=4,anchor="w",
                 font=("",10,"bold")).pack(side="left")
        v=tk.DoubleVar(value=0.0); vars[j]=v
        ent=tk.Label(top,textvariable=v,bg="#f0f0f0",width=8,anchor="e")
        ent.pack(side="right")
        sc=tk.Scale(row,from_=LIMIT_DEG[0],to=LIMIT_DEG[1],orient="horizontal",
                    resolution=0.5,variable=v,showvalue=False,length=320,
                    bg="#f0f0f0",highlightthickness=0,
                    command=lambda val,jj=j: on_slide(jj,val))
        sc.pack(fill="x")

    def center():
        for j in SLIDER_JOINTS:
            vars[j].set(0.0); node.q[j]=0.0
        node.publish(); msg.set("j1~j5 중앙(0)")
    tk.Button(win,text="j1~j5 전부 0",command=center,width=16).pack(pady=6)

    # ---- j6 그리퍼 버튼 ----
    tk.Frame(win,height=2,bg="#c0c0c0").pack(fill="x",padx=10,pady=8)
    tk.Label(win,text="그리퍼 (j6)",bg="#f0f0f0",font=("",12,"bold")).pack()
    gf=tk.Frame(win,bg="#f0f0f0"); gf.pack(pady=4)
    def grip(open_):
        node.q['j6']=J6_OPEN_RAD if open_ else J6_CLOSE_RAD
        node.publish()
        node.send_cmd('open' if open_ else 'close')
        msg.set("그리퍼 열기" if open_ else "그리퍼 닫기")
    tk.Button(gf,text="열 기",command=lambda:grip(True),
              width=10,height=2,bg="#4a8fd6",fg="white",
              font=("",11,"bold")).pack(side="left",padx=6)
    tk.Button(gf,text="닫 기",command=lambda:grip(False),
              width=10,height=2,bg="#d67a4a",fg="white",
              font=("",11,"bold")).pack(side="left",padx=6)
    tf=tk.Frame(win,bg="#f0f0f0"); tf.pack(pady=4)
    tk.Button(tf,text="토크 OFF",width=11,
              command=lambda:(node.send_cmd('torque_off'),msg.set("그리퍼 토크 OFF"))
              ).pack(side="left",padx=4)
    tk.Button(tf,text="토크 ON",width=11,
              command=lambda:(node.send_cmd('torque_on'),msg.set("그리퍼 토크 ON"))
              ).pack(side="left",padx=4)

    tk.Label(win,textvariable=msg,bg="#f0f0f0",fg="#333").pack(pady=6)

    # 주기 발행 (RViz 가 늦게 떠도 반영되도록)
    def tick():
        node.publish()
        win.after(200,tick)
    win.after(400,tick)

    def on_close():
        try: rclpy.shutdown()
        except Exception: pass
        win.destroy()
    win.protocol("WM_DELETE_WINDOW",on_close)
    win.mainloop()

if __name__=='__main__':
    main()
