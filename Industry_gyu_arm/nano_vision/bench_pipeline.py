"""
신경망 말고 '나머지' 가 얼마나 먹는지.
RACHEL 8장: 좋은 것만 보고하지 않는다 -> 파이프라인 전체를 잰다.
"""
import time, statistics
import numpy as np, torch, torch.nn as nn
torch.set_num_threads(8); torch.manual_seed(0)

def t(fn, runs=50, warm=10):
    for _ in range(warm): fn()
    ts=[]
    for _ in range(runs):
        t0=time.perf_counter(); fn(); ts.append((time.perf_counter()-t0)*1000)
    return statistics.median(ts), statistics.mean(ts), max(ts)

def show(tag, r):
    med,avg,hi=r
    print(f"  {tag:46s} 중앙 {med:6.2f}ms  평균 {avg:6.2f}  최대 {hi:6.2f}")
    return med

print("="*96)
print("파이프라인 각 단계 실측  (신경망 밖에서 나가는 시간)")
print("="*96)

H,W = 720,1280                      # 흔한 웹캠 원본
frame = np.random.randint(0,255,(H,W,3), dtype=np.uint8)
total = 0.0

print(f"\n원본 프레임 {W}x{H} uint8 → 모델 입력 416x416 float32 까지")

# 1) 리사이즈 (numpy 만으로: 최근접 이웃)
def resize_nn():
    yi = (np.arange(416)*H//416); xi=(np.arange(416)*W//416)
    return frame[yi][:,xi]
total += show("1) 리사이즈 416x416 (최근접, numpy)", t(resize_nn))

small = resize_nn()

# 2) BGR->RGB + HWC->CHW + /255  (전처리의 진짜 비용)
def to_tensor():
    x = small[:,:,::-1].astype(np.float32) * (1/255.0)
    x = np.ascontiguousarray(x.transpose(2,0,1))
    return torch.from_numpy(x).unsqueeze(0)
total += show("2) 색변환+전치+정규화 (float32)", t(to_tensor))

# 2b) int8 모델이면 uint8 그대로 넣을 수 있어 더 쌈
def to_tensor_u8():
    x = np.ascontiguousarray(small[:,:,::-1].transpose(2,0,1))
    return torch.from_numpy(x).unsqueeze(0)
show("2b) 같은 것, uint8 유지 (int8 모델용)", t(to_tensor_u8))

# 3) NMS (복잡한 장면 = 후보가 많음)
try:
    from torchvision.ops import nms as tvnms
    HAVE_TV=True
except Exception:
    HAVE_TV=False

def nms_np(boxes, scores, thr=0.45):
    idx = scores.argsort()[::-1]; keep=[]
    x1,y1,x2,y2 = boxes[:,0],boxes[:,1],boxes[:,2],boxes[:,3]
    areas=(x2-x1)*(y2-y1)
    while idx.size>0:
        i=idx[0]; keep.append(i)
        if idx.size==1: break
        xx1=np.maximum(x1[i],x1[idx[1:]]); yy1=np.maximum(y1[i],y1[idx[1:]])
        xx2=np.minimum(x2[i],x2[idx[1:]]); yy2=np.minimum(y2[i],y2[idx[1:]])
        w=np.maximum(0,xx2-xx1); h=np.maximum(0,yy2-yy1); inter=w*h
        iou=inter/(areas[i]+areas[idx[1:]]-inter)
        idx=idx[1:][iou<=thr]
    return keep

for ncand in (300, 1000, 3000):
    b=np.random.rand(ncand,4).astype(np.float32); b[:,2:]+=b[:,:2]
    s=np.random.rand(ncand).astype(np.float32)
    m=show(f"3) NMS 후보 {ncand:4d}개 (numpy 직접)", t(lambda: nms_np(b,s), runs=20))
    if ncand==1000: total += m

# 4) 디코딩: 격자 출력 -> 상자 좌표 (복잡한 장면일수록 후보가 많다)
raw3 = torch.randn(1, 75, 52, 52); raw4=torch.randn(1,75,26,26); raw5=torch.randn(1,75,13,13)
def decode():
    out=[]
    for r in (raw3,raw4,raw5):
        b,c,h,w = r.shape
        r = r.view(b,3,25,h,w).permute(0,1,3,4,2).reshape(-1,25)
        obj = torch.sigmoid(r[:,4])
        keep = obj > 0.25
        out.append(r[keep])
    return torch.cat(out)
total += show("4) 격자 디코딩 + 임계값 거르기", t(decode, runs=30))

print("\n" + "-"*96)
print(f"  신경망 제외 합계(대표값):  약 {total:.1f} ms")
print(f"  60fps 예산 16.67ms 중 신경망에 남는 시간: {16.67-total:.1f} ms")
print("-"*96)
