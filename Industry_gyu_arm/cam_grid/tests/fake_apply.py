import sys, math, time
sys.path.insert(0,'cam_grid'); import camsync
src=open('arm_studio_web.py',encoding='utf-8').read()
code=src[src.index('CAM_TRACKS=[]'):src.index('def _cam_apply_loop():')]
ENV={'dock':{'id':'dock','visible':False}}; NAMES={}; POS={}
G=dict(math=math,time=time,PLATE_TOP=0.02,BASE_OFF=[0.0,0.0],BASE_YAW=[0.7],PEG_PITCH=0.02)
G['docking_ids']=lambda: list(ENV); G['_env_ent']=lambda o: ENV.get(o)
G['docking_top']=lambda d: (*POS[d],0.115) if d in POS else None
G['_names']=lambda: NAMES; G['_snap_color']=lambda c: c
G['disp_name']=lambda o: o.split('#')[-1] if '#' in o else o
def set_robot_pose(x,y,yaw): G['BASE_OFF'][0]=x; G['BASE_OFF'][1]=y
def env_clone(b): n=f'dock#{len(ENV)+1}'; ENV[n]={'id':n,'visible':True}; return n
def set_name(o,color=None): NAMES.setdefault(o,{})['color']=color
def move_env(o,x,y,snap=True): POS[o]=(round(x/0.02)*0.02,round(y/0.02)*0.02); return True,''
def env_delete(o): ENV.pop(o); POS.pop(o,None); return True,o
G.update(set_robot_pose=set_robot_pose,env_clone=env_clone,set_name=set_name,move_env=move_env,env_delete=env_delete)
G['_plates_origin']=lambda: (-0.09,-0.09)
import numpy as np
G['np']=np; G['os']=__import__('os'); G['MESH_DIR']='/nonexistent'
G['robot_pins_local']=lambda: [np.array([0.210,-0.250,0]),np.array([0.270,-0.250,0])]
G['_yawR']=lambda: np.array([[np.cos(G['BASE_YAW'][0]),-np.sin(G['BASE_YAW'][0]),0],[np.sin(G['BASE_YAW'][0]),np.cos(G['BASE_YAW'][0]),0],[0,0,1]])
G['l2w']=lambda p: G['_yawR']()@np.asarray(p,float)+np.array([G['BASE_OFF'][0],G['BASE_OFF'][1],0])
exec(code,G)
S=camsync.SYNC; S.robot_mesh='/home/drboom/2026_project/urdf/sts3215/Industry_gyu_arm_20260728_base/meshes/base_link.STL'; S.load('/home/drboom/2026_project/Industry_gyu_arm/cam_grid/shots/KakaoTalk_20261006_123735928.mp4')
while S.busy: time.sleep(0.5)
S.ctl('pause'); last=None
for loop in range(2):                           # 영상이 반복 재생되는 것까지
    for k in range(0,94,3):
        S.k_pause=k; G['_cam_apply_once'](S)
        if G['CAM_APPLY']['last']!=last:
            last=G['CAM_APPLY']['last']; print(f"{loop+1}회차 프레임 {k:3d}: {last[9:]}")
print("\n최종 앱 배치:  로봇", [round(v*1000) for v in G['BASE_OFF']],"mm")
for o in ENV:
    if ENV[o]['visible']: print(f"  {o}  색 {NAMES.get(o,{}).get('color')}  자리 {[round(v*1000) for v in POS.get(o,(0,0))]}mm")
