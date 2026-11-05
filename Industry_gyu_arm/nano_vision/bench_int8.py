"""int8 양자화 + 구조 최적화가 얼마나 버는지 실측."""
import time, statistics
import torch, torch.nn as nn
torch.manual_seed(0)
torch.set_num_threads(8)

def cbr(i,o,k=3,s=1):
    return nn.Sequential(nn.Conv2d(i,o,k,s,k//2,bias=False),
                         nn.BatchNorm2d(o), nn.ReLU(inplace=True))
class DW(nn.Module):
    def __init__(self,c):
        super().__init__()
        self.dw=nn.Sequential(nn.Conv2d(c,c,3,1,1,groups=c,bias=False),
                              nn.BatchNorm2d(c),nn.ReLU(inplace=True))
        self.pw=nn.Sequential(nn.Conv2d(c,c,1,bias=False),
                              nn.BatchNorm2d(c),nn.ReLU(inplace=True))
        self.add=nn.quantized.FloatFunctional()
    def forward(self,x): return self.add.add(x, self.pw(self.dw(x)))

class Det(nn.Module):
    def __init__(self,w=48,nc=20,depth=(2,3,4,2)):
        super().__init__()
        self.q=torch.ao.quantization.QuantStub(); self.dq=torch.ao.quantization.DeQuantStub()
        self.stem=cbr(3,w,3,2)
        self.s2=nn.Sequential(cbr(w,w*2,3,2),*[DW(w*2) for _ in range(depth[0])])
        self.s3=nn.Sequential(cbr(w*2,w*4,3,2),*[DW(w*4) for _ in range(depth[1])])
        self.s4=nn.Sequential(cbr(w*4,w*8,3,2),*[DW(w*8) for _ in range(depth[2])])
        self.s5=nn.Sequential(cbr(w*8,w*16,3,2),*[DW(w*16) for _ in range(depth[3])])
        self.up=nn.Upsample(scale_factor=2,mode='nearest')
        self.c4=nn.quantized.FloatFunctional(); self.c3=nn.quantized.FloatFunctional()
        self.l4=cbr(w*16+w*8,w*8,1); self.l3=cbr(w*8+w*4,w*4,1)
        self.h3=nn.Conv2d(w*4,(nc+5)*3,1); self.h4=nn.Conv2d(w*8,(nc+5)*3,1)
        self.h5=nn.Conv2d(w*16,(nc+5)*3,1)
    def forward(self,x):
        x=self.q(x); x=self.stem(x); x=self.s2(x)
        c3=self.s3(x); c4=self.s4(c3); c5=self.s5(c4)
        p4=self.l4(self.c4.cat([self.up(c5),c4],1))
        p3=self.l3(self.c3.cat([self.up(p4),c3],1))
        return self.dq(self.h3(p3)),self.dq(self.h4(p4)),self.dq(self.h5(c5))

def np_(m): return sum(p.numel() for p in m.parameters())
def bench(m,s,runs=20,warm=6):
    m.eval(); x=torch.randn(1,3,s,s)
    with torch.no_grad():
        for _ in range(warm): m(x)
        ts=[]
        for _ in range(runs):
            t0=time.perf_counter(); m(x); ts.append((time.perf_counter()-t0)*1000)
    return statistics.median(ts), statistics.mean(ts), min(ts), max(ts)

def show(tag, r, extra=""):
    med,avg,lo,hi=r
    print(f"  {tag:40s} 중앙 {med:7.1f}ms 평균 {avg:7.1f} 최소 {lo:6.1f} 최대 {hi:7.1f} → {1000/med:5.1f}fps {extra}")
    return med

print("="*104)
print("int8 양자화 실측  (fbgemm = AVX512-VNNI 사용)")
print("="*104)
print(f"\nqengine 사용 가능: {torch.backends.quantized.supported_engines}")
torch.backends.quantized.engine='fbgemm'

for w,size in ((48,416),(64,416),(48,320),(32,416)):
    m=Det(w=w); p=np_(m)
    f32=bench(m,size,runs=12)
    # PTQ
    m.qconfig=torch.ao.quantization.get_default_qconfig('fbgemm')
    torch.ao.quantization.prepare(m,inplace=True)
    with torch.no_grad():
        for _ in range(6): m(torch.randn(1,3,size,size))
    mq=torch.ao.quantization.convert(m,inplace=False)
    i8=bench(mq,size,runs=12)
    print(f"\n▸ w={w} ({p/1e6:.2f}M 파라미터 = int8 {p/1e6:.1f}MB), 입력 {size}x{size}")
    a=show("float32", f32)
    b=show("int8 (fbgemm/VNNI)", i8)
    print(f"  {'':40s} → int8 이 {a/b:.2f}배 빠름")
