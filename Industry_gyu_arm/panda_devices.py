"""라떼판다에 꽂은 USB 웹캠(영상 + 마이크)을 앱에 붙인다 (2026-11-04).

  영상: 휴대폰 카메라와 '같은 길'로 넣는다 — JPEG 한 장씩 core._livecam().push().
        그래서 판 맞춤·도킹 찾기·자동 반영은 휴대폰 때와 똑같이 돈다. 켜고 끄는 건 버튼으로만.
  소리: 브라우저 마이크가 안 될 때(다른 PC 에서 http 로 열었거나, 파이어폭스가 16kHz 녹음을
        못 여는 경우) 판다에서 arecord 로 직접 녹음해 WAV 로 돌려준다. 인식은 기존 /api/stt 그대로.

arm_studio_pro.py 에서 attach(app, core) 한 줄로 붙는다. 실패해도 앱은 그대로 돈다.
"""
import glob, io, os, struct, subprocess, threading, time

CAM_NAME = 'C270'           # /sys/class/video4linux/*/name 에서 이 이름을 찾는다 (없으면 첫 카메라)
CAM_W, CAM_H = 1280, 720    # C270 최대
CAM_FPS = 8                 # 보내는 장 수 (휴대폰은 10 — 판다 CPU 를 아끼려고 조금 낮춤)
CAM_QUALITY = 80
MIC_RATE = 16000            # /api/stt 가 받는 형식 (16kHz 모노 16bit)
MIC_MAX_S = 60              # 버튼을 안 끄고 잊어도 이 이상은 안 쌓는다


def _find_cam():
    """C270 의 /dev/videoN 번호. 한 카메라가 video0(영상)·video1(정보) 두 개를 만드므로 작은 번호."""
    hits = []
    for p in sorted(glob.glob('/sys/class/video4linux/video*')):
        try:
            name = open(os.path.join(p, 'name')).read().strip()
        except Exception:
            continue
        hits.append((CAM_NAME.lower() not in name.lower(), int(p.rsplit('video', 1)[1]), name))
    if not hits:
        return None, None
    hits.sort()
    return hits[0][1], hits[0][2]


class PandaCam:
    def __init__(self, core):
        self.core = core
        self.th = None
        self.stop_ev = threading.Event()
        self.msg = '꺼짐'
        self.n = 0
        self.dark = False

    def on(self):
        return self.th is not None and self.th.is_alive()

    def start(self):
        if self.on():
            return self.status()
        self.stop_ev.clear()
        self.n = 0
        self.msg = '켜는 중'
        self.th = threading.Thread(target=self._run, daemon=True)
        self.th.start()
        time.sleep(0.3)
        return self.status()

    def stop(self):
        self.stop_ev.set()
        if self.th:
            self.th.join(timeout=3)
        self.msg = '꺼짐'
        return self.status()

    def status(self):
        s = f'판다 웹캠: {self.msg}'
        if self.on():
            s += f' · 보낸 장 {self.n}'
            if self.dark:
                s += ' · ⚠ 화면이 거의 까맣습니다 (렌즈 가림/방향 확인)'
        return s

    def _run(self):
        import cv2
        idx, name = _find_cam()
        if idx is None:
            self.msg = '카메라가 없습니다 (USB 웹캠을 꽂아 주세요)'
            return
        cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))   # YUYV 로는 720p 가 5fps 남짓
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)
        if not cap.isOpened():
            self.msg = f'/dev/video{idx} 를 열지 못했습니다 (다른 프로그램이 쓰는 중?)'
            return
        lv = self.core._livecam()
        if lv is None:
            cap.release()
            self.msg = '카메라 모듈(livecam)을 못 불러왔습니다'
            return
        self.msg = f'켜짐 ({name}, /dev/video{idx})'
        print(f'[판다 웹캠] {self.msg}')
        gap = 1.0 / CAM_FPS
        t_next = 0.0
        fails = 0
        try:
            while not self.stop_ev.is_set():
                ok, img = cap.read()          # 늘 읽어서 버퍼를 비운다 (안 그러면 옛 장이 나간다)
                if not ok:
                    fails += 1
                    if fails > 30:
                        self.msg = '영상이 끊겼습니다 (USB 를 다시 꽂고 다시 켜 주세요)'
                        return
                    time.sleep(0.05)
                    continue
                fails = 0
                now = time.time()
                if now < t_next:
                    continue
                t_next = now + gap
                ok, jpg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, CAM_QUALITY])
                if not ok:
                    continue
                if self.n % 40 == 0:
                    self.dark = float(img[::8, ::8].mean()) < 20
                lv.push(jpg.tobytes())
                self.n += 1
        finally:
            cap.release()
            print('[판다 웹캠] 꺼짐')


def _wav(pcm):
    hdr = (b'RIFF' + struct.pack('<I', 36 + len(pcm)) + b'WAVEfmt '
           + struct.pack('<IHHIIHH', 16, 1, 1, MIC_RATE, MIC_RATE * 2, 2, 16)
           + b'data' + struct.pack('<I', len(pcm)))
    return hdr + pcm


class PandaMic:
    """arecord 로 판다 마이크 녹음. 'default' 는 펄스오디오를 거쳐 브라우저와 같이 써도 안 부딪힌다."""
    def __init__(self):
        self.p = None
        self.buf = bytearray()
        self.lock = threading.Lock()
        self.th = None

    def start(self):
        self.stop()
        self.buf = bytearray()
        env = dict(os.environ)
        env.setdefault('XDG_RUNTIME_DIR', f'/run/user/{os.getuid()}')
        try:
            self.p = subprocess.Popen(['arecord', '-q', '-D', 'default', '-f', 'S16_LE',
                                       '-r', str(MIC_RATE), '-c', '1', '-t', 'raw'],
                                      stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, env=env)
        except Exception as e:
            self.p = None
            return {'ok': False, 'error': f'녹음을 시작하지 못했습니다: {e}'}
        self.th = threading.Thread(target=self._pump, args=(self.p,), daemon=True)
        self.th.start()
        return {'ok': True}

    def _pump(self, p):
        cap = MIC_RATE * 2 * MIC_MAX_S
        while True:
            d = p.stdout.read(3200)
            if not d:
                break
            with self.lock:
                if len(self.buf) < cap:
                    self.buf += d

    def wav(self):
        with self.lock:
            return _wav(bytes(self.buf))

    def stop(self):
        p, self.p = self.p, None
        if p is not None:
            p.terminate()
            try:
                p.wait(timeout=2)
            except Exception:
                p.kill()
            if self.th:
                self.th.join(timeout=2)
        return self.wav()


def attach(app, core):
    from fastapi.responses import Response
    cam = PandaCam(core)
    mic = PandaMic()

    @app.post('/api/panda_cam/{cmd}')
    def api_panda_cam(cmd: str):
        if cmd == 'start':
            return {'ok': True, 'status': cam.start()}
        if cmd == 'stop':
            return {'ok': True, 'status': cam.stop()}
        if cmd == 'toggle':
            return {'ok': True, 'status': cam.stop() if cam.on() else cam.start(), 'on': cam.on()}
        return {'ok': True, 'status': cam.status(), 'on': cam.on()}

    @app.post('/api/panda_mic/start')
    def api_panda_mic_start():
        return mic.start()

    @app.get('/api/panda_mic/peek')
    def api_panda_mic_peek():
        return Response(content=mic.wav(), media_type='audio/wav')

    @app.post('/api/panda_mic/stop')
    def api_panda_mic_stop():
        return Response(content=mic.stop(), media_type='audio/wav')

    print('[판다 장치] 웹캠·마이크 연결 준비됨 (/api/panda_cam, /api/panda_mic)')
    return cam, mic
