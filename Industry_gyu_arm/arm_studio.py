#!/usr/bin/env python3
"""
ARM STUDIO - URDF 자세 캡처 -> RViz 확인 -> 미세조정 -> 시퀀스 -> 실물 실행 통합 UI.

워크플로우:
  1. URDF를 드래그앤드롭(또는 Add URDF) -> 관절각 자동 역산 (축+xyz+메쉬)
  2. 사이드바에서 자세 클릭 -> RViz에 즉시 표시
  3. 가운데 슬라이더로 관절 미세조정 (그리퍼 j6 포함)
  4. Duplicate로 자세 복사 -> j6만 바꿔 그리퍼 열림/닫힘 변형 생성
  5. 오른쪽에서 시퀀스 구성 (base -> 1 -> 2 ...) + 이동방식 선택
  6. PREVIEW(RViz만) / RUN(실물) / STOP(즉시 토크 OFF)

실행:
  터미널1: ros2 launch Industry_gyu_arm_20260724_base_position jog.launch.py
  터미널2: conda run --no-capture-output -n gyu_arm python arm_studio.py

UI 텍스트는 영어 (tkinter 한글 폰트 문제).
"""
import os, sys, json, math, time, signal, threading
import numpy as np
import xml.etree.ElementTree as ET
import struct

import tkinter as tk
from tkinter import ttk, filedialog, messagebox, simpledialog
try:
    from tkinterdnd2 import TkinterDnD, DND_FILES
    HAS_DND=True
except Exception:
    HAS_DND=False

from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# ================= 캘리브레이션 (2026-07-24 확정) =================
PORT='/dev/ttyUSB0'; BAUD=57600
ADDR_TORQUE=64; ADDR_P_ACCEL=108; ADDR_P_VEL=112; ADDR_GOAL=116; ADDR_PRESENT=132
ADDR_DRIVE_MODE=10
SAFE_VEL=25; SAFE_ACCEL=8
PROFILE_MS=110; ACCEL_MS=35

BASE_URDF='/home/drboom/2026_project/urdf/Industry_gyu_arm_20260724_base_position/urdf/Industry_gyu_arm_20260724_base_position.urdf'
JOINTS=['j1','j2','j3','j4','j5','j6']
J2ID=[0,1,2,4,3,5]                                    # j4<->j5 배선 교차
ZERO_RAW={0:1517,1:2483,2:1562,3:2244,4:2545,5:2018}
DIR={0:-1,1:1,2:1,3:1,4:1,5:1}                         # j1 반전
LIMIT_RAW={0:(380,2571),1:(1479,3555),2:(1334,3382),3:(1271,3153),4:(1599,3524),5:(1970,3525)}
PROJECT_FILE='/home/drboom/2026_project/Industry_gyu_arm/arm_studio_project.json'

def s32(v): return v-0x100000000 if v>0x7FFFFFFF else v
def jr_limits(jidx):
    i=J2ID[jidx]; lo,hi=LIMIT_RAW[i]
    a=DIR[i]*(lo-ZERO_RAW[i])*math.pi/2048.0
    b=DIR[i]*(hi-ZERO_RAW[i])*math.pi/2048.0
    return (min(a,b),max(a,b))
def rad_to_raw(jidx,rad):
    i=J2ID[jidx]
    raw=int(ZERO_RAW[i]+DIR[i]*rad*2048.0/math.pi)
    lo,hi=LIMIT_RAW[i]; return max(lo,min(hi,raw)), i

# ================= 기구학 =================
def _R(rpy):
    r,p,y=rpy
    Rx=np.array([[1,0,0],[0,np.cos(r),-np.sin(r)],[0,np.sin(r),np.cos(r)]])
    Ry=np.array([[np.cos(p),0,np.sin(p)],[0,1,0],[-np.sin(p),0,np.cos(p)]])
    Rz=np.array([[np.cos(y),-np.sin(y),0],[np.sin(y),np.cos(y),0],[0,0,1]])
    return Rz@Ry@Rx
def _rot(axis,th):
    a=np.asarray(axis,float); a=a/np.linalg.norm(a)
    x,y,z=a; c,s=np.cos(th),np.sin(th); C=1-c
    return np.array([[c+x*x*C,x*y*C-z*s,x*z*C+y*s],
                     [y*x*C+z*s,c+y*y*C,y*z*C-x*s],
                     [z*x*C-y*s,z*y*C+x*s,c+z*z*C]])
def _parse_urdf(path):
    root=ET.parse(path).getroot(); d={}
    for j in root.findall('joint'):
        o=j.find('origin'); a=j.find('axis')
        rpy=np.array([float(x) for x in (o.get('rpy') or '0 0 0').split()])
        xyz=np.array([float(x) for x in (o.get('xyz') or '0 0 0').split()])
        axis=np.array([float(x) for x in (a.get('xyz') if a is not None else '0 0 1').split()])
        d[j.get('name')]=dict(rpy=rpy,xyz=xyz,axis=axis)
    return d

BASE_DEF=_parse_urdf(BASE_URDF)

def build_chain():
    links=[OriginLink()]; active=[False]
    for k,jn in enumerate(JOINTS):
        b=BASE_DEF[jn]
        links.append(URDFLink(name=jn,origin_translation=list(b['xyz']),
                              origin_orientation=list(b['rpy']),rotation=list(b['axis']),
                              bounds=jr_limits(k),joint_type='revolute'))
        active.append(True)
    return Chain(name='arm',links=links,active_links_mask=active)
CHAIN=build_chain()
def fk(q):
    T=CHAIN.forward_kinematics([0]+list(q)); return T[:3,3].copy(), T[:3,:3].copy()

def _read_stl(path,maxtri=20000):
    with open(path,'rb') as f:
        f.read(80); n=struct.unpack('<I',f.read(4))[0]; n=min(n,maxtri)
        pts=np.zeros((n,3))
        for i in range(n):
            data=f.read(50); pts[i]=struct.unpack('<3f',data[12:24])
    return pts

def solve_pose_from_urdf(urdf_path):
    """자세 URDF -> 관절각 역산 (축+xyz 순차, j6는 메쉬 Kabsch). (angles, warn)"""
    new=_parse_urdf(urdf_path)
    M=np.eye(3); thetas=[]
    for k,nk in enumerate(JOINTS):
        b=BASE_DEF[nk]; a=b['axis']/np.linalg.norm(b['axis'])
        C=_R(b['rpy']).T@M@_R(b['rpy'])
        if k+1<len(JOINTS):
            nk1=JOINTS[k+1]
            pairs=[(BASE_DEF[nk1]['axis']/np.linalg.norm(BASE_DEF[nk1]['axis']),
                    new[nk1]['axis']/np.linalg.norm(new[nk1]['axis']))]
            vb=BASE_DEF[nk1]['xyz']; vn=new[nk1]['xyz']
            if np.linalg.norm(vb)>1e-6:
                pairs.append((vb/np.linalg.norm(vb),vn/np.linalg.norm(vn)))
            num=den=0.0
            for v,t_raw in pairs:
                t=C.T@t_raw
                vp=v-a*np.dot(a,v); tp=t-a*np.dot(a,t)
                if np.linalg.norm(vp)<1e-6: continue
                num+=np.dot(a,np.cross(vp,tp)); den+=np.dot(vp,tp)
            th=math.atan2(num,den) if (abs(num)>1e-12 or abs(den)>1e-12) else 0.0
        else:
            th=0.0
        M=C@_rot(a,th); thetas.append(th)
    warn=""
    # j6: 메쉬 비교
    try:
        base_mesh=os.path.join(os.path.dirname(BASE_URDF),'..','meshes','l6.STL')
        new_mesh=os.path.join(os.path.dirname(urdf_path),'..','meshes','l6.STL')
        vb=_read_stl(base_mesh); vn=_read_stl(new_mesh)
        if len(vb)==len(vn):
            cb=vb.mean(0); cn=vn.mean(0)
            H=(vb-cb).T@(vn-cn); U,S,Vt=np.linalg.svd(H)
            D=np.diag([1,1,np.sign(np.linalg.det(Vt.T@U.T))])
            Rm=Vt.T@D@U.T
            M5=np.eye(3)
            for k,nk in enumerate(JOINTS[:-1]):
                b=BASE_DEF[nk]; a=b['axis']/np.linalg.norm(b['axis'])
                C=_R(b['rpy']).T@M5@_R(b['rpy']); M5=C@_rot(a,thetas[k])
            b6=BASE_DEF['j6']; a6=b6['axis']/np.linalg.norm(b6['axis'])
            C6=_R(b6['rpy']).T@M5@_R(b6['rpy'])
            Rj=C6.T@Rm
            ang=np.arccos(np.clip((np.trace(Rj)-1)/2,-1,1))
            vec=np.array([Rj[2,1]-Rj[1,2],Rj[0,2]-Rj[2,0],Rj[1,0]-Rj[0,1]])
            sign=np.sign(np.dot(vec,a6)) if np.linalg.norm(vec)>1e-9 else 1
            thetas[5]=float(ang*sign)
        else:
            warn="l6 mesh mismatch: j6=0"
    except Exception as e:
        warn=f"j6 mesh solve skipped: {e}"
    # 검증
    Mv=np.eye(3); maxerr=0
    for k,nk in enumerate(JOINTS):
        b=BASE_DEF[nk]; a=b['axis']/np.linalg.norm(b['axis'])
        C=_R(b['rpy']).T@Mv@_R(b['rpy']); Mv=C@_rot(a,thetas[k])
        if k+1<len(JOINTS):
            nk1=JOINTS[k+1]
            e=np.linalg.norm(Mv@BASE_DEF[nk1]['xyz']-new[nk1]['xyz'])*1000
            maxerr=max(maxerr,e)
    if maxerr>2.0: warn=(warn+" | " if warn else "")+f"verify err {maxerr:.1f}mm"
    return [float(t) for t in thetas], warn

# ================= 경로 계획 =================
def plan_joint_seq(qa,qb,steps=50):
    way=[list(qa)]; q=list(qa)
    for j in range(6):
        if abs(qb[j]-q[j])<0.005: q[j]=qb[j]; continue
        st=q[j]
        for k in range(1,steps+1):
            t=k/steps; t=t*t*(3-2*t)
            q[j]=st+(qb[j]-st)*t; way.append(list(q))
    way[-1]=list(qb); return way,0.05
def plan_joint_all(qa,qb,steps=50):
    way=[]
    for k in range(steps+1):
        t=k/steps; t=t*t*(3-2*t)
        way.append([a+(b-a)*t for a,b in zip(qa,qb)])
    way[-1]=list(qb); return way,0.05
def _ee_common(qa,qb):
    p0,r0=fk(qa); p1,r1=fk(qb)
    z0=r0[:,2]; z1=r1[:,2]
    cz=np.clip(np.dot(z0,z1),-1,1); angz=np.arccos(cz)
    axz=np.cross(z0,z1)
    axz=axz/np.linalg.norm(axz) if np.linalg.norm(axz)>1e-9 else np.array([1,0,0.0])
    return p0,r0,p1,z0,axz,angz
def plan_ee_line(qa,qb):
    p0,r0,p1,z0,axz,angz=_ee_common(qa,qb)
    d=p1-p0; L=np.linalg.norm(d)
    if L<0.003: return plan_joint_all(qa,qb)
    n=max(10,int(L/0.003)); way=[list(qa)]; q=list(qa)
    for k in range(1,n+1):
        t=k/n; te=t*t*(3-2*t)
        pt=p0+d*te; zt=_rot(axz,angz*te)@z0
        sol=CHAIN.inverse_kinematics(target_position=pt,target_orientation=zt,
                                     orientation_mode='Z',initial_position=[0]+q)
        qn=list(sol[1:]); pchk,_=fk(qn)
        if np.linalg.norm(pchk-pt)>0.004: return None
        if max(abs(a-b) for a,b in zip(qn,q))>math.radians(15): return None
        way.append(qn); q=qn
    way[-1]=list(qb); return way,0.08
def plan_ee_axis(qa,qb,order):
    p0,r0,p1,z0,axz,angz=_ee_common(qa,qb)
    d=p1-p0; total=sum(abs(d[i]) for i in range(3))
    if total<0.003: return plan_joint_all(qa,qb)
    way=[list(qa)]; q=list(qa); cur=p0.copy(); traveled=0.0
    for ai in order:
        dist=d[ai]
        if abs(dist)<0.002:
            cur[ai]+=dist; traveled+=abs(dist); continue
        n=max(6,int(abs(dist)/0.003))
        for k in range(1,n+1):
            t=k/n; te=t*t*(3-2*t)
            pt=cur.copy(); pt[ai]+=dist*te
            frac=(traveled+abs(dist)*te)/total
            zt=_rot(axz,angz*frac)@z0
            sol=CHAIN.inverse_kinematics(target_position=pt,target_orientation=zt,
                                         orientation_mode='Z',initial_position=[0]+q)
            qn=list(sol[1:]); pchk,_=fk(qn)
            if np.linalg.norm(pchk-pt)>0.004: return None
            if max(abs(a-b) for a,b in zip(qn,q))>math.radians(15): return None
            way.append(qn); q=qn
        cur[ai]+=dist; traveled+=abs(dist)
    way[-1]=list(qb); return way,0.08
AXIS_ORDERS={'x-y-z':[0,1,2],'x-z-y':[0,2,1],'y-x-z':[1,0,2],
             'y-z-x':[1,2,0],'z-x-y':[2,0,1],'z-y-x':[2,1,0]}
MODES=['Joint sequential (j1..j6)','Joint together','EE straight line','EE axis-by-axis']

# ================= 로봇 =================
class Robot:
    def __init__(self):
        self.port=None; self.packet=None
    def connect(self):
        if self.port: return True
        from dynamixel_sdk import PortHandler, PacketHandler
        p=PortHandler(PORT)
        if not p.openPort(): return False
        p.setBaudRate(BAUD)
        self.port=p; self.packet=PacketHandler(2.0)
        return True
    def time_profile(self,on):
        for i in ZERO_RAW: self.packet.write1ByteTxRx(self.port,i,ADDR_TORQUE,0)
        for i in ZERO_RAW:
            dm,r,_=self.packet.read1ByteTxRx(self.port,i,ADDR_DRIVE_MODE)
            if r!=0: continue
            ndm=(dm|0x04) if on else (dm & ~0x04)
            if ndm!=dm: self.packet.write1ByteTxRx(self.port,i,ADDR_DRIVE_MODE,ndm)
        for i in ZERO_RAW:
            if on:
                self.packet.write4ByteTxRx(self.port,i,ADDR_P_ACCEL,ACCEL_MS)
                self.packet.write4ByteTxRx(self.port,i,ADDR_P_VEL,PROFILE_MS)
            else:
                self.packet.write4ByteTxRx(self.port,i,ADDR_P_ACCEL,SAFE_ACCEL)
                self.packet.write4ByteTxRx(self.port,i,ADDR_P_VEL,SAFE_VEL)
    def torque(self,on):
        for i in ZERO_RAW: self.packet.write1ByteTxRx(self.port,i,ADDR_TORQUE,1 if on else 0)
    def send(self,q):
        for jidx in range(6):
            raw,i=rad_to_raw(jidx,q[jidx])
            self.packet.write4ByteTxRx(self.port,i,ADDR_GOAL,raw & 0xFFFFFFFF)
    def read(self):
        q=[0.0]*6
        for jidx in range(6):
            i=J2ID[jidx]
            p,r,_=self.packet.read4ByteTxRx(self.port,i,ADDR_PRESENT)
            if r==0: q[jidx]=DIR[i]*(s32(p)-ZERO_RAW[i])*math.pi/2048.0
        return q
    def estop(self):
        try:
            self.torque(False); self.time_profile(False)
        except Exception: pass

# ================= UI =================
class App:
    def __init__(self):
        rclpy.init()
        self.node=Node('arm_studio')
        self.pub=self.node.create_publisher(JointState,'joint_states',10)
        self.robot=Robot()
        self.poses=[{'name':'base','angles':[0.0]*6}]
        self.seq=['base']
        self.stop_flag=threading.Event()
        self.running=False
        self._updating=False

        Root=TkinterDnD.Tk if HAS_DND else tk.Tk
        self.root=Root()
        self.root.title('ARM STUDIO')
        self.root.geometry('980x560')
        self.root.configure(bg='#d9d9d9')
        self._build_ui()
        self.load(silent=True)
        self.refresh_poses()
        self.select_pose(0)
        self.root.protocol('WM_DELETE_WINDOW',self.on_close)

    # ---------- UI 구성 ----------
    def _build_ui(self):
        top=ttk.Frame(self.root); top.pack(fill='x',padx=6,pady=4)
        ttk.Button(top,text='Save',command=self.save).pack(side='left')
        ttk.Button(top,text='Load',command=self.load).pack(side='left',padx=4)
        self.status=tk.StringVar(value='Ready. Drop URDF or Add URDF.')
        ttk.Label(top,textvariable=self.status).pack(side='left',padx=12)

        main=ttk.Frame(self.root); main.pack(fill='both',expand=True,padx=6,pady=4)

        # 왼쪽: 자세 목록
        left=ttk.LabelFrame(main,text=' POSES ')
        left.pack(side='left',fill='y',padx=(0,6))
        self.pose_list=tk.Listbox(left,width=22,height=18,exportselection=False)
        self.pose_list.pack(fill='y',expand=True,padx=4,pady=4)
        self.pose_list.bind('<<ListboxSelect>>',lambda e:self.on_pose_click())
        if HAS_DND:
            self.pose_list.drop_target_register(DND_FILES)
            self.pose_list.dnd_bind('<<Drop>>',self.on_drop)
            ttk.Label(left,text='(drop .urdf here)').pack()
        bf=ttk.Frame(left); bf.pack(fill='x',padx=4,pady=4)
        ttk.Button(bf,text='Add URDF',command=self.add_urdf).grid(row=0,column=0,sticky='ew')
        ttk.Button(bf,text='Duplicate',command=self.dup_pose).grid(row=0,column=1,sticky='ew')
        ttk.Button(bf,text='Rename',command=self.rename_pose).grid(row=1,column=0,sticky='ew')
        ttk.Button(bf,text='Delete',command=self.del_pose).grid(row=1,column=1,sticky='ew')

        # 가운데: 관절 조정
        mid=ttk.LabelFrame(main,text=' JOINT ANGLES (deg) - live to RViz ')
        mid.pack(side='left',fill='both',expand=True,padx=(0,6))
        self.sliders=[]; self.spins=[]
        for j in range(6):
            lo,hi=jr_limits(j)
            row=ttk.Frame(mid); row.pack(fill='x',padx=6,pady=3)
            name=f'j{j+1}'+(' (gripper)' if j==5 else '')
            ttk.Label(row,text=name,width=11).pack(side='left')
            var=tk.DoubleVar(value=0.0)
            sl=tk.Scale(row,from_=math.degrees(lo),to=math.degrees(hi),
                        orient='horizontal',resolution=0.1,variable=var,
                        showvalue=False,length=300,bg='#d9d9d9',
                        command=lambda v,jj=j:self.on_slider(jj))
            sl.pack(side='left',fill='x',expand=True)
            sp=ttk.Spinbox(row,from_=math.degrees(lo),to=math.degrees(hi),
                           increment=0.5,width=8,textvariable=var,
                           command=lambda jj=j:self.on_slider(jj))
            sp.bind('<Return>',lambda e,jj=j:self.on_slider(jj))
            sp.pack(side='left',padx=4)
            self.sliders.append(var); self.spins.append(sp)
        self.ee_label=tk.StringVar(value='EE: -')
        ttk.Label(mid,textvariable=self.ee_label).pack(pady=4)

        # 오른쪽: 시퀀스/실행
        right=ttk.LabelFrame(main,text=' SEQUENCE ')
        right.pack(side='left',fill='y')
        self.seq_list=tk.Listbox(right,width=24,height=10,exportselection=False)
        self.seq_list.pack(padx=4,pady=4)
        sf=ttk.Frame(right); sf.pack(fill='x',padx=4)
        ttk.Button(sf,text='Add >',command=self.seq_add).grid(row=0,column=0,sticky='ew')
        ttk.Button(sf,text='Remove',command=self.seq_del).grid(row=0,column=1,sticky='ew')
        ttk.Button(sf,text='Up',command=lambda:self.seq_move(-1)).grid(row=1,column=0,sticky='ew')
        ttk.Button(sf,text='Down',command=lambda:self.seq_move(1)).grid(row=1,column=1,sticky='ew')
        of=ttk.Frame(right); of.pack(fill='x',padx=4,pady=6)
        ttk.Label(of,text='Move mode').grid(row=0,column=0,sticky='w')
        self.mode=ttk.Combobox(of,values=MODES,state='readonly',width=24)
        self.mode.current(0); self.mode.grid(row=1,column=0,sticky='ew')
        ttk.Label(of,text='Axis order (EE axis)').grid(row=2,column=0,sticky='w')
        self.axis_order=ttk.Combobox(of,values=list(AXIS_ORDERS),state='readonly',width=24)
        self.axis_order.set('x-z-y'); self.axis_order.grid(row=3,column=0,sticky='ew')
        ttk.Label(of,text='Speed').grid(row=4,column=0,sticky='w')
        self.speed=ttk.Combobox(of,values=['0.5x','1.0x','1.5x','2.0x'],state='readonly',width=24)
        self.speed.set('1.0x'); self.speed.grid(row=5,column=0,sticky='ew')
        self.loop_var=tk.BooleanVar(value=False)
        ttk.Checkbutton(of,text='Loop',variable=self.loop_var).grid(row=6,column=0,sticky='w')
        rf=ttk.Frame(right); rf.pack(fill='x',padx=4,pady=4)
        ttk.Button(rf,text='PREVIEW (RViz)',command=self.preview).pack(fill='x',pady=2)
        tk.Button(rf,text='RUN ROBOT',bg='#3a7',fg='white',command=self.run_robot).pack(fill='x',pady=2)
        tk.Button(rf,text='STOP',bg='#c33',fg='white',command=self.stop).pack(fill='x',pady=2)
        ttk.Button(rf,text='Torque OFF',command=self.torque_off).pack(fill='x',pady=2)

    # ---------- 자세 관리 ----------
    def refresh_poses(self):
        self.pose_list.delete(0,'end')
        for p in self.poses: self.pose_list.insert('end',p['name'])
        self.seq_list.delete(0,'end')
        for n in self.seq: self.seq_list.insert('end',n)
    def cur_pose_idx(self):
        s=self.pose_list.curselection()
        return s[0] if s else None
    def select_pose(self,idx):
        self.pose_list.selection_clear(0,'end')
        self.pose_list.selection_set(idx); self.on_pose_click()
    def on_pose_click(self):
        idx=self.cur_pose_idx()
        if idx is None: return
        self._updating=True
        for j,v in enumerate(self.poses[idx]['angles']):
            self.sliders[j].set(round(math.degrees(v),1))
        self._updating=False
        self.publish_current()
    def on_slider(self,j):
        if self._updating: return
        idx=self.cur_pose_idx()
        if idx is None: return
        self.poses[idx]['angles'][j]=math.radians(self.sliders[j].get())
        self.publish_current()
    def publish_current(self):
        idx=self.cur_pose_idx()
        if idx is None: return
        q=self.poses[idx]['angles']
        m=JointState(); m.header.stamp=self.node.get_clock().now().to_msg()
        m.name=list(JOINTS); m.position=[float(x) for x in q]
        self.pub.publish(m)
        p,_=fk(q)
        self.ee_label.set(f'EE: x={p[0]*1000:.1f}  y={p[1]*1000:.1f}  z={p[2]*1000:.1f} mm')
    def add_urdf(self,path=None):
        if path is None:
            path=filedialog.askopenfilename(filetypes=[('URDF','*.urdf')])
        if not path: return
        try:
            angles,warn=solve_pose_from_urdf(path)
        except Exception as e:
            messagebox.showerror('URDF','Solve failed: %s'%e); return
        name=os.path.basename(path).replace('.urdf','')
        for tag in ['Industry_gyu_arm_20260724_','Industry_gyu_arm_']:
            name=name.replace(tag,'')
        base=name; k=2
        while any(p['name']==name for p in self.poses):
            name=f'{base}_{k}'; k+=1
        self.poses.append({'name':name,'angles':angles})
        self.refresh_poses(); self.select_pose(len(self.poses)-1)
        deg=', '.join(f'{math.degrees(a):+.1f}' for a in angles)
        self.status.set(f'Added {name}: [{deg}]'+(f'  WARN: {warn}' if warn else ''))
    def on_drop(self,ev):
        for f in self.root.tk.splitlist(ev.data):
            if f.lower().endswith('.urdf'): self.add_urdf(f)
    def dup_pose(self):
        idx=self.cur_pose_idx()
        if idx is None: return
        src=self.poses[idx]
        name=simpledialog.askstring('Duplicate','New name:',initialvalue=src['name']+'_copy')
        if not name: return
        self.poses.append({'name':name,'angles':list(src['angles'])})
        self.refresh_poses(); self.select_pose(len(self.poses)-1)
        self.status.set(f'Duplicated -> {name} (adjust j6 for gripper etc.)')
    def rename_pose(self):
        idx=self.cur_pose_idx()
        if idx is None: return
        old=self.poses[idx]['name']
        if old=='base': messagebox.showinfo('Rename','base cannot be renamed'); return
        name=simpledialog.askstring('Rename','New name:',initialvalue=old)
        if not name: return
        self.poses[idx]['name']=name
        self.seq=[name if s==old else s for s in self.seq]
        self.refresh_poses(); self.select_pose(idx)
    def del_pose(self):
        idx=self.cur_pose_idx()
        if idx is None: return
        if self.poses[idx]['name']=='base':
            messagebox.showinfo('Delete','base cannot be deleted'); return
        name=self.poses[idx]['name']
        del self.poses[idx]
        self.seq=[s for s in self.seq if s!=name]
        self.refresh_poses()

    # ---------- 시퀀스 ----------
    def seq_add(self):
        idx=self.cur_pose_idx()
        if idx is None: return
        self.seq.append(self.poses[idx]['name'])
        self.refresh_poses()
    def seq_del(self):
        s=self.seq_list.curselection()
        if not s: return
        del self.seq[s[0]]; self.refresh_poses()
    def seq_move(self,d):
        s=self.seq_list.curselection()
        if not s: return
        i=s[0]; j=i+d
        if 0<=j<len(self.seq):
            self.seq[i],self.seq[j]=self.seq[j],self.seq[i]
            self.refresh_poses(); self.seq_list.selection_set(j)

    # ---------- 계획/실행 ----------
    def get_angles(self,name):
        for p in self.poses:
            if p['name']==name: return list(p['angles'])
        return None
    def plan_segment(self,qa,qb):
        m=self.mode.get()
        if m==MODES[0]: return plan_joint_seq(qa,qb)
        if m==MODES[1]: return plan_joint_all(qa,qb)
        if m==MODES[2]: return plan_ee_line(qa,qb)
        return plan_ee_axis(qa,qb,AXIS_ORDERS[self.axis_order.get()])
    def plan_all(self):
        if len(self.seq)<2:
            messagebox.showinfo('Sequence','Need at least 2 poses in sequence'); return None
        segs=[]
        for a,b in zip(self.seq[:-1],self.seq[1:]):
            qa=self.get_angles(a); qb=self.get_angles(b)
            r=self.plan_segment(qa,qb)
            if r is None:
                messagebox.showerror('Plan',f'IK path failed: {a} -> {b}\nTry joint mode.')
                return None
            segs.append((f'{a} -> {b}',)+r)
        return segs
    def _speed(self): return float(self.speed.get().replace('x',''))
    def publish_q(self,q):
        m=JointState(); m.header.stamp=self.node.get_clock().now().to_msg()
        m.name=list(JOINTS); m.position=[float(x) for x in q]; self.pub.publish(m)

    def preview(self):
        if self.running: return
        segs=self.plan_all()
        if not segs: return
        self.stop_flag.clear(); self.running=True
        threading.Thread(target=self._preview_thread,args=(segs,),daemon=True).start()
    def _preview_thread(self,segs):
        sp=self._speed()
        while True:
            for label,way,dt in segs:
                self.status.set('Preview: '+label)
                for q in way:
                    if self.stop_flag.is_set(): self.running=False; return
                    self.publish_q(q); time.sleep(dt/sp)
                time.sleep(0.5)
            if not self.loop_var.get(): break
        self.status.set('Preview done'); self.running=False

    def run_robot(self):
        if self.running: return
        segs=self.plan_all()
        if not segs: return
        if not messagebox.askyesno('RUN','Move REAL robot?\nSTOP = instant torque off.'):
            return
        if not self.robot.connect():
            messagebox.showerror('Robot','Cannot open %s'%PORT); return
        self.stop_flag.clear(); self.running=True
        threading.Thread(target=self._run_thread,args=(segs,),daemon=True).start()
    def _run_thread(self,segs):
        sp=self._speed()
        try:
            self.robot.time_profile(True); self.robot.torque(True)
            # 현재 위치 -> 첫 자세 (관절 순서대로, 안전)
            q0=self.robot.read()
            first=self.get_angles(self.seq[0])
            way,dt=plan_joint_seq(q0,first)
            self.status.set('Moving to first pose...')
            for q in way:
                if self.stop_flag.is_set(): raise KeyboardInterrupt
                self.robot.send(q); self.publish_q(q); time.sleep(dt/sp)
            time.sleep(0.5)
            while True:
                for label,way,dt in segs:
                    self.status.set('RUN: '+label)
                    for q in way:
                        if self.stop_flag.is_set(): raise KeyboardInterrupt
                        self.robot.send(q); self.publish_q(q); time.sleep(dt/sp)
                    time.sleep(0.5)
                if not self.loop_var.get(): break
            self.status.set('RUN done (torque ON). Use Torque OFF to release.')
        except KeyboardInterrupt:
            self.robot.estop(); self.status.set('STOPPED - torque off')
        except Exception as e:
            self.robot.estop(); self.status.set('ERROR: %s - torque off'%e)
        finally:
            self.running=False

    def stop(self):
        self.stop_flag.set()
        if self.robot.port:
            self.robot.estop()
        self.status.set('STOP - torque off')
    def torque_off(self):
        if self.robot.connect():
            self.robot.estop(); self.status.set('Torque OFF')
    # ---------- 저장/불러오기 ----------
    def save(self):
        data={'poses':self.poses,'seq':self.seq}
        with open(PROJECT_FILE,'w') as f: json.dump(data,f,indent=1)
        self.status.set('Saved -> '+PROJECT_FILE)
    def load(self,silent=False):
        try:
            with open(PROJECT_FILE) as f: data=json.load(f)
            self.poses=data['poses']; self.seq=data['seq']
            if not any(p['name']=='base' for p in self.poses):
                self.poses.insert(0,{'name':'base','angles':[0.0]*6})
            self.refresh_poses()
            if not silent: self.status.set('Loaded')
        except FileNotFoundError:
            if not silent: messagebox.showinfo('Load','No project file yet')
    def on_close(self):
        self.stop_flag.set()
        if self.robot.port:
            self.robot.estop()
        try: rclpy.shutdown()
        except Exception: pass
        self.root.destroy()

if __name__=='__main__':
    app=App()
    app.root.mainloop()
