"""
RACHEL 10장의 '덜 하는 것' 을 비전에 적용:
매 프레임 신경망을 돌리지 않고 추적(계산)으로 메우면 실효 fps 가 얼마나 되나.
"""
import time, statistics
import numpy as np
print("="*92)
print("추적으로 메우기 - 신경망을 매 프레임 안 돌린다면")
print("="*92)

def t(fn, runs=200, warm=20):
    for _ in range(warm): fn()
    ts=[]
    for _ in range(runs):
        t0=time.perf_counter(); fn(); ts.append((time.perf_counter()-t0)*1000)
    return statistics.median(ts)

# --- 추적 비용: 등속 칼만 (상자 N개)
class Track:
    def __init__(self,n):
        self.x=np.random.rand(n,8).astype(np.float32)   # cx,cy,w,h,vx,vy,vw,vh
        self.P=np.tile(np.eye(8,dtype=np.float32),(n,1,1))
        F=np.eye(8,dtype=np.float32); F[0,4]=F[1,5]=F[2,6]=F[3,7]=1
        self.F=F; self.Q=np.eye(8,dtype=np.float32)*0.01
    def predict(self):
        self.x=self.x@self.F.T
        self.P=self.F@self.P@self.F.T+self.Q

for n in (10, 30, 60):
    tr=Track(n)
    ms=t(tr.predict)
    print(f"  칼만 예측 {n:3d}개 상자                        {ms:6.3f} ms  → {1000/ms:8.0f}fps 가능")

# --- 헝가리안 대신 IoU 탐욕 매칭 (검출 프레임에서만)
def match(nd=30, nt=30):
    d=np.random.rand(nd,4).astype(np.float32); d[:,2:]+=d[:,:2]
    tk=np.random.rand(nt,4).astype(np.float32); tk[:,2:]+=tk[:,:2]
    x1=np.maximum(d[:,None,0],tk[None,:,0]); y1=np.maximum(d[:,None,1],tk[None,:,1])
    x2=np.minimum(d[:,None,2],tk[None,:,2]); y2=np.minimum(d[:,None,3],tk[None,:,3])
    inter=np.clip(x2-x1,0,None)*np.clip(y2-y1,0,None)
    a=(d[:,2]-d[:,0])*(d[:,3]-d[:,1]); b=(tk[:,2]-tk[:,0])*(tk[:,3]-tk[:,1])
    return inter/(a[:,None]+b[None,:]-inter)
ms_m=t(lambda: match(30,30), runs=100)
print(f"  IoU 매칭 30x30 (검출 프레임에서만)             {ms_m:6.3f} ms")

# --- 실효 fps 계산
print("\n" + "-"*92)
print("실효 fps: 신경망을 N 프레임에 한 번만 돌리고 나머지는 추적")
print("-"*92)
NN_MS   = 16.3     # int8 6.1MB 모델 416 실측
POST_MS = 7.3      # NMS 6.5 + 디코딩 0.8 (검출 프레임에서만)
PRE_MS  = 1.4      # 리사이즈 1.2 + uint8 0.2
TRK_MS  = 0.05     # 추적 프레임 비용
print(f"  (검출 프레임 = 전처리 {PRE_MS} + 신경망 {NN_MS} + 후처리 {POST_MS} = {PRE_MS+NN_MS+POST_MS:.1f}ms)")
print(f"  (추적 프레임 = {TRK_MS}ms)\n")
print(f"  {'주기':>6s} {'평균 프레임시간':>14s} {'실효fps':>9s}   {'판정':<28s}")
for N in (1,2,3,4,6):
    avg=((PRE_MS+NN_MS+POST_MS) + (N-1)*TRK_MS)/N
    fps=1000/avg
    v = "✅ 60fps 넘김" if fps>=60 else ("△ 30~60" if fps>=30 else "✗ 부족")
    print(f"  {N:3d}프레임 {avg:12.2f}ms {fps:9.1f}   {v:<28s}")
print("\n  ※ 파이썬 후처리 기준. C++ 로 옮기면 후처리가 1ms 아래로 내려가 더 좋아진다.")
