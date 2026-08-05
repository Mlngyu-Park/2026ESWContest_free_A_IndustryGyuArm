#!/usr/bin/env python3
"""
그리퍼(j6) 열기/닫기 패널. RViz launch 에 포함되어 함께 뜬다.

- /gripper_cmd (String: open/close/torque_off/torque_on) 발행 -> 브리지가 실물 제어
- /gripper_joint (Float64: j6 관절각 rad) 발행 -> RViz 도 같이 움직임
  (브리지가 이 값을 joint_states 에 합쳐서 발행)

실행(단독): python gripper_panel.py
"""
import sys, threading, math
import tkinter as tk
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64

# 실측 기반 j6 관절각 (sts_zero + sts_gripper 로 계산됨)
#   닫힘 raw 840 -> 0.0 rad,  열림 raw 2060 -> -1.8715 rad
J6_CLOSE_RAD = 0.0
J6_OPEN_RAD  = -1.8715

class Panel(Node):
    def __init__(self):
        super().__init__('gripper_panel')
        self.cmd=self.create_publisher(String,'gripper_cmd',10)
        self.joint=self.create_publisher(Float64,'gripper_joint',10)
    def send(self,cmd,rad=None):
        m=String(); m.data=cmd; self.cmd.publish(m)
        if rad is not None:
            f=Float64(); f.data=float(rad); self.joint.publish(f)

def main():
    rclpy.init()
    node=Panel()
    threading.Thread(target=lambda: rclpy.spin(node),daemon=True).start()

    win=tk.Tk()
    win.title("Gripper (j6)")
    win.geometry("240x225")
    win.configure(bg="#d9d9d9")
    msg=tk.StringVar(value="")

    def do_open():
        node.send("open",J6_OPEN_RAD); msg.set("열기")
    def do_close():
        node.send("close",J6_CLOSE_RAD); msg.set("닫기")
    def do_toff():
        node.send("torque_off"); msg.set("토크 OFF")
    def do_ton():
        node.send("torque_on"); msg.set("토크 ON")

    tk.Label(win,text="그리퍼 j6",bg="#d9d9d9",font=("",12,"bold")).pack(pady=6)
    tk.Button(win,text="열 기",command=do_open,
              width=16,height=2,bg="#4a8fd6",fg="white",
              font=("",11,"bold")).pack(pady=5)
    tk.Button(win,text="닫 기",command=do_close,
              width=16,height=2,bg="#d67a4a",fg="white",
              font=("",11,"bold")).pack(pady=5)
    tk.Button(win,text="토크 OFF",command=do_toff,width=18).pack(pady=2)
    tk.Button(win,text="토크 ON",command=do_ton,width=18).pack(pady=2)
    tk.Label(win,textvariable=msg,bg="#d9d9d9",fg="#333").pack(pady=4)

    # 시작 시 닫힘 상태로 RViz 초기화
    win.after(600,lambda: node.joint.publish(Float64(data=J6_CLOSE_RAD)))

    def on_close():
        try: rclpy.shutdown()
        except Exception: pass
        win.destroy()
    win.protocol("WM_DELETE_WINDOW",on_close)
    win.mainloop()

if __name__=='__main__':
    main()
