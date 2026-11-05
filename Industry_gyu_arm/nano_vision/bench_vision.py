"""
CPU 전용 비전 모델 실측 벤치마크.
RACHEL 8장 교훈: 제일 빠른 것만 보지 않는다 -> 중앙값/평균/최댓값 전부 기록.
"""
import time, statistics, os
import torch, torch.nn as nn

torch.manual_seed(0)

# ---- 스레드 설정: 로봇 제어와 CPU 를 나눠 쓸 것을 고려해 4/8 둘 다 잰다
def set_threads(n):
    torch.set_num_threads(n)

# ---------------------------------------------------------------
# 검출 모델 뼈대 (YOLO 계열과 같은 구조: 백본 + FPN + 헤드)
# 목표 10MB(int8) = 약 1000만 파라미터
# ---------------------------------------------------------------
def cbr(i, o, k=3, s=1):
    return nn.Sequential(
        nn.Conv2d(i, o, k, s, k//2, bias=False),
        nn.BatchNorm2d(o), nn.SiLU(inplace=True))

class DWBlock(nn.Module):
    """깊이별 분리 합성곱 - 모바일 계열의 핵심. 파라미터를 크게 줄인다."""
    def __init__(self, c):
        super().__init__()
        self.dw = nn.Sequential(
            nn.Conv2d(c, c, 3, 1, 1, groups=c, bias=False),
            nn.BatchNorm2d(c), nn.SiLU(inplace=True))
        self.pw = nn.Sequential(
            nn.Conv2d(c, c, 1, bias=False),
            nn.BatchNorm2d(c), nn.SiLU(inplace=True))
    def forward(self, x):
        return x + self.pw(self.dw(x))

class Detector(nn.Module):
    def __init__(self, w=48, nc=20, depth=(2,3,4,2)):
        super().__init__()
        self.stem = cbr(3, w, 3, 2)                    # /2
        self.s2 = nn.Sequential(cbr(w, w*2, 3, 2),     # /4
                                *[DWBlock(w*2) for _ in range(depth[0])])
        self.s3 = nn.Sequential(cbr(w*2, w*4, 3, 2),   # /8
                                *[DWBlock(w*4) for _ in range(depth[1])])
        self.s4 = nn.Sequential(cbr(w*4, w*8, 3, 2),   # /16
                                *[DWBlock(w*8) for _ in range(depth[2])])
        self.s5 = nn.Sequential(cbr(w*8, w*16, 3, 2),  # /32
                                *[DWBlock(w*16) for _ in range(depth[3])])
        # FPN: 큰 물체/작은 물체를 같이 보기 위한 것 (복잡한 장면의 핵심)
        self.up = nn.Upsample(scale_factor=2, mode='nearest')
        self.l4 = cbr(w*16 + w*8, w*8, 1)
        self.l3 = cbr(w*8 + w*4, w*4, 1)
        self.h3 = nn.Conv2d(w*4,  (nc+5)*3, 1)
        self.h4 = nn.Conv2d(w*8,  (nc+5)*3, 1)
        self.h5 = nn.Conv2d(w*16, (nc+5)*3, 1)
    def forward(self, x):
        x = self.stem(x); x = self.s2(x)
        c3 = self.s3(x); c4 = self.s4(c3); c5 = self.s5(c4)
        p4 = self.l4(torch.cat([self.up(c5), c4], 1))
        p3 = self.l3(torch.cat([self.up(p4), c3], 1))
        return self.h3(p3), self.h4(p4), self.h5(c5)

def nparams(m):
    return sum(p.numel() for p in m.parameters())

def bench(model, size, runs=30, warmup=8, threads=8):
    set_threads(threads)
    model.eval()
    x = torch.randn(1, 3, size, size)
    with torch.no_grad():
        for _ in range(warmup):
            model(x)
        ts = []
        for _ in range(runs):
            t0 = time.perf_counter()
            model(x)
            ts.append((time.perf_counter() - t0) * 1000)
    return ts

def report(tag, ts, extra=""):
    med = statistics.median(ts); avg = statistics.mean(ts)
    print(f"  {tag:34s} 중앙 {med:7.1f}ms  평균 {avg:7.1f}ms  "
          f"최소 {min(ts):6.1f}  최대 {max(ts):7.1f}  → {1000/med:5.1f}fps {extra}")
    return med

print("=" * 100)
print("CPU 전용 검출 모델 실측  (Ryzen 7 8845HS / torch float32)")
print("=" * 100)

# ---- 1) 폭(파라미터)을 바꿔가며: 10MB 목표선이 어디인지
print("\n[1] 모델 크기별  (입력 416x416, 8스레드)  ※ 클래스 20종")
for w in (24, 32, 48, 64):
    m = Detector(w=w)
    p = nparams(m)
    ts = bench(m, 416, runs=20, threads=8)
    report(f"w={w:3d}  {p/1e6:5.2f}M ({p/1e6:5.1f}MB int8)", ts)

# ---- 2) 해상도를 바꿔가며: 진짜 병목이 여기라는 것
print("\n[2] 입력 해상도별  (w=48, 약 10MB 모델, 8스레드)  ★ 진짜 병목")
m48 = Detector(w=48)
p48 = nparams(m48)
print(f"    (이 모델 파라미터 {p48/1e6:.2f}M = int8 약 {p48/1e6:.1f}MB)")
base = None
for s in (256, 320, 416, 512, 640):
    ts = bench(m48, s, runs=15, threads=8)
    med = statistics.median(ts)
    if base is None: base = med
    report(f"{s}x{s}", ts, f" (256 대비 {med/base:4.1f}배)")

# ---- 3) 스레드 수: RACHEL 은 OpenMP 를 뺐지만 비전은 반대라는 것
print("\n[3] 스레드 수별  (w=48, 416x416)  ※ RACHEL 과 반대로 나오는지 확인")
for t in (1, 2, 4, 8, 16):
    ts = bench(m48, 416, runs=15, threads=t)
    med = statistics.median(ts)
    print(f"  {t:2d}스레드   중앙 {med:7.1f}ms  → {1000/med:5.1f}fps")

# ---- 4) 분류 모델: 이미지 인식은 얼마나 싼가
print("\n[4] 이미지 인식(분류)  - 검출과 얼마나 차이나는지")
class Classifier(nn.Module):
    def __init__(self, w=48, nc=100):
        super().__init__()
        self.f = nn.Sequential(
            cbr(3, w, 3, 2), cbr(w, w*2, 3, 2), DWBlock(w*2),
            cbr(w*2, w*4, 3, 2), DWBlock(w*4), DWBlock(w*4),
            cbr(w*4, w*8, 3, 2), DWBlock(w*8), DWBlock(w*8),
            cbr(w*8, w*16, 3, 2), DWBlock(w*16),
            nn.AdaptiveAvgPool2d(1), nn.Flatten())
        self.head = nn.Linear(w*16, nc)
        self.emb = w*16
    def forward(self, x):
        return self.head(self.f(x))
c = Classifier()
pc = nparams(c)
for s in (128, 224):
    ts = bench(c, s, runs=20, threads=8)
    report(f"분류 {s}x{s}  {pc/1e6:.2f}M", ts)
