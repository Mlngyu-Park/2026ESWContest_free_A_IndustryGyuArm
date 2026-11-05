"""필요 사양 실측: 해상도 · 프레임 간격 · 처리 시간 · 메모리."""
import sys, time, resource, numpy as np, cv2
sys.path.insert(0, '.')
from grid import Board, detect_holes, pitch_of
V = 'shots/KakaoTalk_20261006_123735928.mp4'
c = cv2.VideoCapture(V); F = []
while True:
    ok, f = c.read()
    if not ok: break
    F.append(f)
def run(frames):
    B = Board(); ok = 0; t = time.time(); pit = []
    for f in frames:
        g = B.add(f); ok += g
        if g: pit.append(pitch_of(B.H, f.shape))
    dt = (time.time() - t) / len(frames)
    if not B.holes: return ok, dt, None, None, None
    ck = B.check(); n_in = int(ck['판_안_구멍'].split('/')[0])
    return ok, dt, n_in, ck['전체_범위_칸수'], np.median(pit) if pit else None
print('== 해상도 (원본 1280x720, 94프레임 전부) ==')
for w in (int(sys.argv[1]),) if len(sys.argv)>1 else (1280, 960, 640, 480, 320):
    fr = [cv2.resize(f, (w, int(w * 9 / 16)), interpolation=cv2.INTER_AREA) for f in F]
    ok, dt, n, span, pit = run(fr)
    print(f'{w:4d}x{int(w*9/16):3d}: 맞춘 프레임 {ok}/94  판 안 구멍 {n}/900  범위 {span}  격자간격 {pit and round(pit,1)}px  {dt*1000:.0f}ms/프레임', flush=True)
print('== 프레임 간격 (1280x720, 프레임 건너뛰기 = 카메라가 그만큼 빨리 움직인 것) ==')
for s in (() if len(sys.argv)>1 else (1, 2, 3, 4, 6, 8, 12)):
    fr = F[::s]; ok, dt, n, span, pit = run(fr)
    print(f'{s:2d}프레임마다 (실효 {24/s:4.1f}fps, {len(fr)}장): 맞춘 {ok}/{len(fr)}  판 안 구멍 {n}  범위 {span}', flush=True)
print('최대 메모리(이 시험 전체): %.0f MB' % (resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024))
