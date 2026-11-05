#!/usr/bin/env python3
"""tpu_show.py — DUSK_BOOM FPGA 의 HDMI 화면: TPU 가 지금 무엇을 어떻게 곱하고 있는지 (2026-11-05)

  역할: 뮤(라떼판다)가 ARM STUDIO 를 맡고, 이 보드는 '추론 화면' 만 맡는다.
  명령: /run/tpu_cmd 에 한 줄씩 (base64 로 싼 JSON). 뮤의 tpu_bridge 가 USB 시리얼 셸로 덧붙인다.
        {"op":"start","id":n,"kind":"rachel|stt|llm","title":"말 → 블록","model":"더블 레이첼 …"}
        {"op":"prompt","id":n,"text":"…"}   이 보드 TPU 로 같은 문장을 실제로 계산 (rachel_tpu --serve)
        {"op":"text","id":n,"text":"…"}     음성 인식 결과 글자 (출력 칸에 한 글자씩)
        {"op":"done","id":n}                계산 완료 표시 (뮤가 블록을 놓는 순간)
  답:   /dev/ttyPS0 에 '@D {json}' 한 줄 — {"ev":"fpga_done","id":n,"ms":…}
  숫자: 온도 = XADC 실측 + 가동률에 따른 상승(비바도 열저항 11.5°C/W), 전력 = 비바도 전력 분석 + 가동률,
        곱셈·대역폭·가동률 = rachel_tpu 기록(#M, 엔진 클럭 수). 음성·Gemma 는 그 모델 크기대로 그린다.
  시험: python3 tpu_show.py --png out_dir   (화면 없이 몇 장 그려 본다)
"""
import base64
import collections
import json
import math
import mmap
import os
import random
import subprocess
import sys
import threading
import time

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
import pygame  # noqa: E402

W, H = 1280, 720
HERE = os.path.dirname(os.path.abspath(__file__))
CMD_FILE = os.environ.get("TPU_CMD", "/run/tpu_cmd")
TTY = os.environ.get("TPU_TTY", "/dev/ttyPS0")
RACHEL_DIR = os.environ.get("RACHEL_DIR", "/opt/dusk/rachel")
PACE_MS = int(os.environ.get("TPU_PACE_MS", "3700"))
FONT_DIRS = ["/usr/share/fonts/truetype/nanum", "/usr/share/fonts/truetype/dejavu", HERE]
XADC = "/sys/bus/iio/devices/iio:device0"

CLK_MHZ = 148.33
MACS_PER_ENGINE = 32         # 8 칸 × 4 벡터
P_BASE, P_TPU = 1.96, 0.55   # W: PS7 1.546 + 정지 0.135 + MMCM·IO·클럭 0.27 / TPU 가 꽉 돌 때 더하는 몫
THETA = 11.5                 # °C/W (비바도 Effective TJA)


def _c(h):
    h = h.lstrip("#")
    return tuple(int(h[i:i + 2], 16) for i in (0, 2, 4))


BG, PANEL, PANEL2, LINE = _c("0b0e13"), _c("141922"), _c("1b212c"), _c("2a3140")
TXT, DIM, FAINT = _c("e7ebf3"), _c("8b93a7"), _c("4a5263")
ACC, OKC, WARN, HOT = _c("4c8dff"), _c("34c77b"), _c("f5a623"), _c("ff6b4a")
CYAN, VIOLET = _c("38d0e0"), _c("a77bff")


def mix(a, b, t):
    t = max(0.0, min(1.0, t))
    return tuple(int(a[i] + (b[i] - a[i]) * t) for i in range(3))


# =================================================================================================
# 모델 모양 (층마다 곱하는 행렬) — 화면의 층 띠와 '몇 × 몇' 표시
# =================================================================================================
KIND_KO = {"qkv": "어텐션 Q·K·V", "out": "어텐션 출력", "up": "MLP 넓히기", "down": "MLP 좁히기",
           "head": "글자 점수", "xattn": "교차 어텐션", "conv": "소리 특징"}


def model_shape(kind, model):
    """(층 이름 목록, 층마다의 [(종류, 행, 열)]) — rachel 은 실제 기록이 오므로 모양만."""
    m = (model or "").lower()
    if kind == "rachel":
        d = 448
        layers = [f"{i + 1}" for i in range(10)] + ["출력"]
        ops = [[("qkv", 3 * d, d), ("out", d, d), ("up", 4 * d, d), ("down", d, 4 * d)]] * 10 + [[("head", 2502, d)]]
        return layers, ops
    if "whisper" in m:
        d = 1280
        enc = [[("qkv", 3 * d, d), ("out", d, d), ("up", 4 * d, d), ("down", d, 4 * d)]] * 32
        dec = [[("qkv", 3 * d, d), ("xattn", d, d), ("up", 4 * d, d), ("down", d, 4 * d)]] * 4
        return [f"E{i + 1}" for i in range(32)] + [f"D{i + 1}" for i in range(4)] + ["출력"], \
            enc + dec + [[("head", 51866, d)]]
    if "moonshine" in m:
        d = 288
        enc = [[("qkv", 3 * d, d), ("out", d, d), ("up", 4 * d, d), ("down", d, 4 * d)]] * 6
        dec = [[("qkv", 3 * d, d), ("xattn", d, d), ("up", 4 * d, d), ("down", d, 4 * d)]] * 6
        return [f"E{i + 1}" for i in range(6)] + [f"D{i + 1}" for i in range(6)] + ["출력"], \
            enc + dec + [[("head", 32768, d)]]
    d = 2560                                                 # Gemma 급 (말 → 블록, Spark)
    blk = [[("qkv", 3 * d, d), ("out", d, d), ("up", 4 * d, d), ("down", d, 4 * d)]] * 34
    return [f"{i + 1}" for i in range(34)] + ["출력"], blk + [[("head", 262144, d)]]


# =================================================================================================
# 상태 (명령·기록 스레드가 쓰고 그리기가 읽는다)
# =================================================================================================
class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.mode = "idle"            # idle | run | done
        self.kind = ""
        self.title = ""
        self.model = ""
        self.job = 0
        self.t_start = 0.0
        self.t_done = 0.0
        self.layers, self.ops = model_shape("rachel", "")
        self.cur = None               # (층, 종류, 행, 열, 벡터)
        self.layer_done = set()
        self.win = collections.deque()   # (t, 걸린초, 가중치바이트, MAC)
        self.macs = 0
        self.calls = 0
        self.out = ""                 # 출력 칸 글자
        self.pending_text = ""        # 음성: 한 글자씩 흘려 보낼 글
        self.phase = "대기"
        self.real = False             # 이 보드 TPU 가 실제로 계산 중인가
        self.dT = 0.0                 # 온도 상승분
        self.util = 0.0
        self.bw = 0.0
        self.gmacs = 0.0
        self.hist_t = collections.deque(maxlen=120)
        self.fpga_ms = None

    def start(self, j):
        with self.lock:
            self.mode, self.kind = "run", j.get("kind", "rachel")
            self.title, self.model = j.get("title", ""), j.get("model", "")
            self.job = j.get("id", 0)
            self.t_start, self.t_done = time.time(), 0.0
            self.layers, self.ops = model_shape(self.kind, self.model)
            self.cur, self.layer_done = None, set()
            self.win.clear()
            self.macs = self.calls = 0
            self.out = self.pending_text = ""
            self.phase = "준비"
            self.real = False
            self.fpga_ms = None

    def event(self, layer, kind, rows, cols, n, us, nbytes=None, nmacs=None, ncalls=1):
        with self.lock:
            now = time.time()
            if self.cur and layer != self.cur[0] and self.cur[0] is not None:
                self.layer_done.add(self.cur[0])
                if layer > self.cur[0]:            # 기록 간격 사이에 지나간 층도 끝난 것으로
                    self.layer_done.update(range(self.cur[0], layer))
            self.cur = (layer, kind, rows, cols, n)
            b = rows * cols if nbytes is None else nbytes
            m = b * n if nmacs is None else nmacs
            self.win.append((now, us / 1e6, b, m))
            self.macs += m
            self.calls += ncalls


ST = State()
SNAP = [None]                 # 다음 그림을 PNG 로 (확인용)


def tty_send(obj):
    try:
        with open(TTY, "w") as f:
            f.write("\r\n@D " + json.dumps(obj, ensure_ascii=True) + "\r\n")
    except OSError:
        pass


# =================================================================================================
# 실제 계산: rachel_tpu --serve (기록 켬) — 모델은 한 번만 올린다
# =================================================================================================
class RachelTPU:
    def __init__(self):
        self.p = None
        self.lock = threading.Lock()
        self.busy_id = None
        self.t0 = 0.0
        self.prev = (0, 0.0, 0.0, 0.0)

    def _spawn(self):
        exe = os.path.join(RACHEL_DIR, "rachel_tpu")
        w = os.path.join(RACHEL_DIR, "double.bin")
        if not (os.path.exists(exe) and os.path.exists(w)):
            return False
        env = dict(os.environ, RACHEL_TPU="hw", RACHEL_THREADS="2", RACHEL_TRACE="1",
                   RACHEL_PACE_MS=str(PACE_MS), RACHEL_MAXGEN="440")
        self.p = subprocess.Popen([exe, w, "--serve"], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                  stderr=subprocess.PIPE, env=env, cwd=RACHEL_DIR, bufsize=0,
                                  preexec_fn=lambda: os.nice(-10))     # 계산이 화면 그리기보다 먼저
        if self.p.stdout.readline().strip() != b"READY":
            self.p = None
            return False
        threading.Thread(target=self._trace, args=(self.p,), daemon=True).start()
        return True

    def _trace(self, p):
        for raw in iter(p.stderr.readline, b""):
            line = raw.decode("utf-8", "replace").rstrip("\n")
            if not line.startswith("#"):
                continue
            try:
                tag, rest = line[:2], line[3:]
                if tag == "#C":                  # 누적값 → 지난 줄과의 차이를 한 사건으로
                    a = rest.split()
                    calls, us, by, mc = int(a[5]), float(a[6]), float(a[7]), float(a[8])
                    p0 = self.prev
                    if a[2] == "0":                  # 마지막 합계: 숫자만 맞춘다
                        with ST.lock:
                            ST.macs, ST.calls = int(mc), calls
                        self.prev = (calls, us, by, mc)
                        continue
                    ST.event(int(a[0]), a[1], int(a[2]), int(a[3]), int(a[4]), us - p0[1],
                             nbytes=by - p0[2], nmacs=mc - p0[3], ncalls=calls - p0[0])
                    self.prev = (calls, us, by, mc)
                elif tag == "#T":
                    with ST.lock:
                        ST.out += "\n" if rest == "\\n" else rest
                elif tag == "#S":
                    self.prev = (0, 0.0, 0.0, 0.0)
                    with ST.lock:
                        ST.phase = f"입력 {rest}글자 읽는 중"
                elif tag == "#P":
                    with ST.lock:
                        ST.phase = "한 글자씩 생성"
            except (ValueError, IndexError):
                pass

    def ask(self, job, prompt):
        """프롬프트를 이 보드 TPU 로 계산. 끝나면 뮤에 fpga_done."""
        def run():
            with self.lock:
                if self.p is None or self.p.poll() is not None:
                    if not self._spawn():
                        tty_send({"ev": "fpga_done", "id": job, "ms": 0, "err": "no rachel"})
                        return
                with ST.lock:
                    ST.real = True
                    ST.phase = "입력 읽는 중"
                t0 = time.time()
                self.p.stdin.write(prompt.replace("\n", "\x01").encode() + b"\n")
                self.p.stdin.flush()
                self.p.stdout.readline()
                ms = int((time.time() - t0) * 1000)
                with ST.lock:
                    if ST.job == job:
                        ST.fpga_ms = ms
                        ST.phase = "계산 끝 · 결과 넘기는 중"
                tty_send({"ev": "fpga_done", "id": job, "ms": ms})
        threading.Thread(target=run, daemon=True).start()

    def warm(self):
        def go():
            with self.lock:
                if self.p is None:
                    self._spawn()
        threading.Thread(target=go, daemon=True).start()


RT = RachelTPU()


# =================================================================================================
# 음성·Gemma: 그 모델 크기대로 층을 돈다 (이 보드에 그 모델이 없으므로 모양만)
# =================================================================================================
def shape_runner(job, wait_real=0.0):
    if wait_real:                                # 더블 레이첼: 문장이 곧 오면 실제 계산을 보여준다
        time.sleep(wait_real)
        with ST.lock:
            if ST.real or ST.job != job:
                return
    layers, ops = ST.layers, ST.ops
    with ST.lock:
        ST.phase = "소리 특징 → 인코더" if ST.kind == "stt" else "입력 읽는 중"
    total = max(1.0, PACE_MS / 1000.0)
    nl = len(layers)
    t0 = time.time()
    k = 0
    while True:
        with ST.lock:
            if ST.job != job or ST.mode != "run" or ST.real:
                return
        el = time.time() - t0
        nenc = sum(1 for nm in layers if nm.startswith("E"))
        if nenc:                                  # 인코더는 앞 55 %, 디코더·출력은 나머지 (글자마다 디코더를 돈다)
            if el < total * 0.55:
                li = min(nenc - 1, int(el / (total * 0.55) * nenc))
            else:
                ndec = nl - nenc
                li = nenc + int((el - total * 0.55) / 0.12) % ndec
        else:
            li = min(nl - 1, int(el / total * nl)) if el < total else nl - 2 + k % 2
        step = 0.03
        kind, rows, cols = ops[li][k % len(ops[li])]
        n = 4 if li < nl // 2 else 1
        us = step * 1e6 * random.uniform(0.86, 0.97) * (1.0 if el < total else 0.7)
        ST.event(li, kind, rows, cols, n, us, nbytes=int(us * 2370))    # 두 포트 64비트 × 148 MHz
        if ST.kind == "stt" and el < total * 0.55:
            with ST.lock:
                ST.phase = "인코더 · 소리 특징 읽기"
        elif ST.kind == "stt" and k % 2 == 0:
            with ST.lock:
                ST.phase = "디코더 · 글자 생성"
                if ST.pending_text:
                    ST.out += ST.pending_text[0]
                    ST.pending_text = ST.pending_text[1:]
        k += 1
        time.sleep(step)


# =================================================================================================
# 명령 읽기
# =================================================================================================
def handle(j):
    op = j.get("op")
    if op == "start":
        ST.start(j)
        threading.Thread(target=shape_runner, args=(j.get("id", 0), 0.8 if j.get("kind") == "rachel" else 0.0),
                         daemon=True).start()
        tty_send({"ev": "ack", "op": "start", "id": j.get("id", 0)})
    elif op == "prompt":
        if ST.mode != "run" or ST.job != j.get("id", 0):
            ST.start({"op": "start", "id": j.get("id", 0), "kind": "rachel",
                      "title": "말 → 블록", "model": "더블 레이첼"})
        RT.ask(j.get("id", 0), j.get("text", ""))
    elif op == "text":
        with ST.lock:
            ST.pending_text = j.get("text", "")
    elif op == "done":
        with ST.lock:
            if ST.job == j.get("id", ST.job):
                if ST.pending_text:
                    ST.out += ST.pending_text
                    ST.pending_text = ""
                ST.mode, ST.t_done = "done", time.time()
                ST.phase = "계산 완료"
    elif op == "snap":
        SNAP[0] = j.get("path", "/run/tpu_snap.png")
    elif op == "idle":
        with ST.lock:
            ST.mode = "idle"


def cmd_loop():
    pos = None
    while True:
        try:
            sz = os.path.getsize(CMD_FILE)
        except OSError:
            try:
                open(CMD_FILE, "a").close()
                os.chmod(CMD_FILE, 0o666)
            except OSError:
                pass
            time.sleep(0.2)
            continue
        if pos is None or sz < pos:
            pos = sz if pos is None else 0
        if sz > pos:
            with open(CMD_FILE, "rb") as f:
                f.seek(pos)
                data = f.read()
            last = data.rfind(b"\n")
            if last >= 0:
                pos += last + 1
                for line in data[:last].split(b"\n"):
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        handle(json.loads(base64.b64decode(line).decode("utf-8")))
                    except Exception as e:  # noqa: BLE001
                        print("[tpu_show] bad command:", e, line[:60], flush=True)
        time.sleep(0.03)


# =================================================================================================
# 센서
# =================================================================================================
def xadc_temp():
    try:
        raw = int(open(XADC + "/in_temp0_raw").read())
        off = int(open(XADC + "/in_temp0_offset").read())
        sc = float(open(XADC + "/in_temp0_scale").read())
        return (raw + off) * sc / 1000.0
    except (OSError, ValueError):
        return 48.0


def xadc_vccint():
    try:
        return int(open(XADC + "/in_voltage0_vccint_raw").read()) * float(open(XADC + "/in_voltage0_vccint_scale").read()) / 1000
    except (OSError, ValueError):
        return 1.0


# =================================================================================================
# 화면
# =================================================================================================
class FrameBuffer:
    def __init__(self, dev="/dev/fb0"):
        sysd = "/sys/class/graphics/" + os.path.basename(dev)
        self.w, self.h = map(int, open(sysd + "/virtual_size").read().strip().split(","))
        self.stride = int(open(sysd + "/stride").read())
        self.fd = os.open(dev, os.O_RDWR)
        self.mm = mmap.mmap(self.fd, self.stride * self.h, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE)
        try:                                    # 글자 콘솔이 위에 그리지 않게
            import fcntl
            self._tty = os.open("/dev/tty1", os.O_RDWR)
            fcntl.ioctl(self._tty, 0x4B3A, 1)
        except OSError:
            self._tty = None

    def show(self, surf):
        if self.stride == self.w * 4:
            self.mm[0:self.stride * self.h] = surf.get_buffer().raw
        else:
            buf, row = surf.get_buffer().raw, surf.get_width() * 4
            for y in range(self.h):
                self.mm[y * self.stride:y * self.stride + row] = buf[y * row:(y + 1) * row]


class Screen:
    def __init__(self):
        pygame.init()
        import signal                          # SDL 이 SIGTERM 을 가로채므로 되돌린다 (kill 로 끌 수 있게)
        signal.signal(signal.SIGTERM, lambda *_: os._exit(0))
        self.s = pygame.Surface((W, H), 0, 32)
        self.fc = {}
        self.tc = {}
        self.t_amb = xadc_temp()
        self.t_last_sensor = 0.0
        self.vcc = 1.0
        self.flow = 0.0
        self.prev = time.time()
        self.bg = None
        self.bg_key = None

    def font(self, size, bold=False, mono=False):
        k = (size, bold, mono)
        if k not in self.fc:
            names = (["DejaVuSansMono-Bold.ttf", "DejaVuSansMono.ttf"] if mono else
                     (["NanumGothicBold.ttf", "NanumBarunGothicBold.ttf"] if bold else ["NanumGothic.ttf", "NanumBarunGothic.ttf"]))
            f = None
            for d in FONT_DIRS:
                for n in names:
                    p = os.path.join(d, n)
                    if os.path.exists(p):
                        f = pygame.font.Font(p, size)
                        break
                if f:
                    break
            self.fc[k] = f or pygame.font.Font(None, size + 6)
        return self.fc[k]

    def text(self, s, pos, size=16, col=TXT, bold=False, mono=False, anchor="tl"):
        if mono and not s.isascii():             # 고정폭 글꼴엔 한글이 없다
            mono, bold = False, True
        k = (s, size, col, bold, mono)
        img = self.tc.get(k)
        if img is None:
            if len(self.tc) > 2500:
                self.tc.clear()
            img = self.tc[k] = self.font(size, bold, mono).render(s, True, col)
        r = img.get_rect()
        setattr(r, {"tl": "topleft", "tr": "topright", "c": "center", "ml": "midleft", "mr": "midright"}[anchor], pos)
        self.s.blit(img, r)
        return r

    def panel(self, r, title=None):
        pygame.draw.rect(self.s, PANEL, r, border_radius=10)
        pygame.draw.rect(self.s, LINE, r, 1, border_radius=10)
        if title:
            self.text(title, (r[0] + 14, r[1] + 10), 15, DIM, bold=True)

    # ---- 숫자 갱신 ---------------------------------------------------------------------------
    def update_stats(self):
        now = time.time()
        dt = min(0.5, now - self.prev)
        self.prev = now
        with ST.lock:
            while ST.win and now - ST.win[0][0] > 0.6:
                ST.win.popleft()
            busy = sum(w[1] for w in ST.win)
            span = 0.6
            util = min(1.0, busy / span) if ST.mode == "run" else 0.0
            ST.util += (util - ST.util) * min(1.0, dt * 6)
            ST.bw = sum(w[2] for w in ST.win) / span / 1e9 if ST.mode == "run" else ST.bw * 0.8
            ST.gmacs = sum(w[3] for w in ST.win) / span / 1e9 if ST.mode == "run" else ST.gmacs * 0.8
            target = P_TPU * ST.util * THETA
            tau = 3.0 if target > ST.dT else 18.0
            ST.dT += (target - ST.dT) * min(1.0, dt / tau)
        if now - self.t_last_sensor > 1.0:
            self.t_last_sensor = now
            t = xadc_temp()
            if ST.dT < 0.3:                      # 쉬고 있을 때의 센서값을 바탕 온도로
                self.t_amb += (t - self.t_amb) * 0.5
            self.vcc = xadc_vccint()
        self.flow += dt * (0.3 + 3.0 * ST.util)

    # ---- 그리기 ------------------------------------------------------------------------------
    #   보드 CPU 가 약해서(A9 666MHz) 틀·글자·빈 칸은 한 번만 그려 두고(bg), 매 장에는 바뀌는 것만 덧그린다.
    def draw(self):
        self.update_stats()
        with ST.lock:
            mode, title, model, kind = ST.mode, ST.title, ST.model, ST.kind
            cur, layers, ops, done_set = ST.cur, ST.layers, ST.ops, set(ST.layer_done)
            util, bw, gmacs, dT = ST.util, ST.bw, ST.gmacs, ST.dT
            macs, calls, out, phase = ST.macs, ST.calls, ST.out, ST.phase
            t_start, t_done = ST.t_start, ST.t_done
        now = time.time()
        temp = self.t_amb + dT
        power = P_BASE + P_TPU * util
        el = (t_done if mode == "done" else now) - t_start if mode != "idle" else 0.0
        ST.hist_t.append(temp)
        key = (mode == "idle", title, model, tuple(layers))
        if self.bg is None or self.bg_key != key:
            self.bg_key = key
            self.draw_static(mode, title, model, layers)
            self.bg = self.s.copy()
        else:
            self.s.blit(self.bg, (0, 0))
        s = self.s
        chip = {"idle": ("대기", FAINT), "run": ("연산 중", WARN), "done": ("계산 완료", OKC)}[mode]
        cr = pygame.Rect(W - 170, 16, 150, 34)
        col = mix(WARN, _c("ffd27a"), 0.5 + 0.5 * math.sin(now * 6)) if mode == "run" else chip[1]
        pygame.draw.rect(s, mix(PANEL, col, 0.22), cr, border_radius=17)
        pygame.draw.rect(s, col, cr, 2, border_radius=17)
        pygame.draw.circle(s, col, (cr.x + 20, cr.centery), 6)
        self.text(chip[0], (cr.x + 86, cr.centery), 17, col, bold=True, anchor="c")
        self.arch_dyn(mode, cur, util, now)
        self.stats_dyn(mode, temp, power, util, bw, gmacs, macs, calls, el)
        self.layers_dyn(mode, layers, ops, cur, done_set, phase)
        self.output_dyn(mode, out, now)
        return s

    # 자리 (고정)
    ARCH = (16, 80, 800, 420)
    STATS = (832, 80, 432, 420)
    LAYERS = (16, 512, 1248, 92)
    OUTPUT = (16, 614, 1248, 92)

    def _geo(self):
        x, y = self.ARCH[0], self.ARCH[1]
        ddr = pygame.Rect(x + 18, y + 48, 150, 300)
        wb = pygame.Rect(ddr.x + 14, ddr.y + 64, ddr.w - 28, 124)
        xb = pygame.Rect(ddr.x + 14, ddr.bottom - 52, ddr.w - 28, 34)
        engs = [pygame.Rect(x + 316, y + 48 + e * 186, 300, 168) for e in range(2)]
        return x, y, ddr, wb, xb, engs

    def draw_static(self, mode, title, model, layers):
        s = self.s
        s.fill(BG)
        pygame.draw.rect(s, PANEL, (0, 0, W, 64))
        pygame.draw.line(s, LINE, (0, 64), (W, 64))
        self.text("DUSK TPU", (22, 14), 26, TXT, bold=True)
        self.text("XC7Z010 · 곱셈기 64개 · 148 MHz", (24, 44), 12, DIM)
        if mode == "idle":
            self.text("대기 중", (W // 2, 32), 24, DIM, bold=True, anchor="c")
        else:
            self.text(title or "추론", (W // 2, 20), 24, TXT, bold=True, anchor="c")
            self.text(model, (W // 2, 46), 15, CYAN, anchor="c")
        # 구조
        self.panel(self.ARCH, "TPU 구조 · 지금 곱하는 곳")
        x, y, ddr, wb, xb, engs = self._geo()
        pygame.draw.rect(s, PANEL2, ddr, border_radius=8)
        pygame.draw.rect(s, LINE, ddr, 1, border_radius=8)
        self.text("DDR3", (ddr.centerx, ddr.y + 18), 18, TXT, bold=True, anchor="c")
        self.text("512 MB", (ddr.centerx, ddr.y + 40), 13, DIM, anchor="c")
        pygame.draw.rect(s, BG, wb, border_radius=4)
        fill = int(wb.h * 24.4 / 32)
        pygame.draw.rect(s, mix(BG, VIOLET, 0.55), (wb.x, wb.bottom - fill, wb.w, fill), border_radius=4)
        self.text("가중치 int8", (wb.centerx, wb.bottom + 14), 12, DIM, anchor="c")
        self.text("24.4 / 32 MB", (wb.centerx, wb.bottom + 30), 13, TXT, mono=True, anchor="c")
        for e, eng in enumerate(engs):
            py = eng.y + 71
            pygame.draw.line(s, LINE, (ddr.right, py), (eng.x, py), 3)
            self.text(f"HP{e + 1} · 64비트", (x + 241, eng.y + 48), 12, DIM, anchor="c")
            pygame.draw.rect(s, PANEL2, eng, border_radius=8)
            pygame.draw.rect(s, LINE, eng, 1, border_radius=8)
            self.text(f"엔진 {e}", (eng.x + 12, eng.y + 8), 15, TXT, bold=True)
            self.text("8칸 × 4벡터 = 곱셈기 32", (eng.right - 12, eng.y + 10), 12, DIM, anchor="tr")
            gx, gy, cw, ch = eng.x + 16, eng.y + 36, 28, 26
            off = mix(PANEL2, LINE, 0.9)
            for v in range(4):
                for l in range(8):
                    pygame.draw.rect(s, off, (gx + l * (cw + 3), gy + v * (ch + 3), cw, ch), border_radius=3)
            ax = gx + 8 * (cw + 3) + 6
            for v in range(4):
                cy = gy + v * (ch + 3) + ch // 2
                pygame.draw.line(s, LINE, (ax, cy), (ax + 18, cy), 2)
                acc = pygame.Rect(ax + 18, cy - 10, 26, 20)
                pygame.draw.rect(s, BG, acc, border_radius=3)
                self.text("Σ", acc.center, 13, FAINT, bold=True, anchor="c")
            self.text("곱셈기 → 덧셈 나무 → 누산(32비트) → 결과", (eng.x + 16, eng.bottom - 22), 12, DIM, bold=True)
        self.text("지금 곱하는 행렬", (x + 636, y + 52), 14, DIM, bold=True)
        # 상태
        self.panel(self.STATS, "상태")
        sx, sy, sw, sh = self.STATS
        self.text("온도", (sx + 18, sy + 44), 14, DIM)
        pygame.draw.rect(s, BG, (sx + 220, sy + 50, sw - 240, 50), border_radius=4)
        yy = sy + 124
        for name in ("가동률", "전력", "메모리 대역폭", "곱셈 속도"):
            self.text(name, (sx + 18, yy), 14, DIM)
            pygame.draw.rect(s, BG, (sx + 18, yy + 22, sw - 36, 8), border_radius=4)
            yy += 50
        self.text("곱셈 횟수", (sx + 18, yy + 4), 14, DIM)
        self.text("걸린 시간", (sx + 18, yy + 32), 14, DIM)
        # 층 띠
        self.panel(self.LAYERS)
        lx, ly, lw, lh = self.LAYERS
        self.text("층 진행", (lx + 14, ly + 10), 15, DIM, bold=True)
        n = len(layers)
        cw = (lw - 28 - (n - 1) * 3) / n
        for i, nm in enumerate(layers):
            rr = self._cell(i, n, cw)
            pygame.draw.rect(s, PANEL2, rr, border_radius=3)
            if cw > 18:
                self.text(nm, rr.center, 11 if cw < 40 else 13, DIM, anchor="c")
        # 출력
        self.panel(self.OUTPUT)
        ox, oy = self.OUTPUT[0], self.OUTPUT[1]
        self.text("출력", (ox + 14, oy + 10), 15, DIM, bold=True)
        if mode == "idle":
            self.text("말하거나 입력하면 여기서 계산이 시작됩니다", (ox + 14, oy + 44), 16, FAINT)

    def _cell(self, i, n, cw):
        lx, ly = self.LAYERS[0], self.LAYERS[1]
        return pygame.Rect(int(lx + 14 + i * (cw + 3)), ly + 38, max(2, int(cw)), 30)

    def arch_dyn(self, mode, cur, util, now):
        s = self.s
        x, y, ddr, wb, xb, engs = self._geo()
        act = mode == "run" and util > 0.02
        n_vec = cur[4] if (cur and mode == "run") else 0
        fill = int(wb.h * 24.4 / 32)
        if cur and mode == "run":                  # 지금 읽는 층의 가중치 자리
            band = max(3, fill // 11)
            by = wb.bottom - fill + min(fill - band, int(max(0, cur[0]) / 11 * fill))
            pygame.draw.rect(s, VIOLET, (wb.x, by, wb.w, band))
        pygame.draw.rect(s, mix(BG, CYAN, 0.35 if act else 0.12), xb, border_radius=4)
        self.text("입력 벡터 int16", xb.center, 12, TXT, anchor="c")
        for e, eng in enumerate(engs):
            if act:
                py = eng.y + 71
                span = eng.x - ddr.right
                for k in range(6):
                    px = ddr.right + ((self.flow * 140 + k * 26 + e * 13) % span)
                    pygame.draw.circle(s, VIOLET, (int(px), py), 4)
                pygame.draw.rect(s, mix(LINE, WARN, 0.6), eng, 2, border_radius=8)
            gx, gy, cw, ch = eng.x + 16, eng.y + 36, 28, 26
            ax = gx + 8 * (cw + 3) + 6
            for v in range(min(4, n_vec) if act else 0):
                for l in range(8):
                    flick = 0.65 + 0.35 * math.sin(now * 23 + l * 1.7 + v * 2.3 + e * 0.9)
                    rr = pygame.Rect(gx + l * (cw + 3), gy + v * (ch + 3), cw, ch)
                    s.fill(mix(_c("6b4a12"), WARN, flick), rr)
                    self.text("×", rr.center, 13, _c("2a1a00"), bold=True, anchor="c")
                cy = gy + v * (ch + 3) + ch // 2
                pygame.draw.line(s, WARN, (ax, cy), (ax + 18, cy), 2)
                acc = pygame.Rect(ax + 18, cy - 10, 26, 20)
                s.fill(mix(BG, OKC, 0.75), acc)
                self.text("Σ", acc.center, 13, TXT, bold=True, anchor="c")
        mx = x + 636
        if cur and mode != "idle":
            layer, kind, rows, cols, n = cur
            lname = ST.layers[layer] if 0 <= layer < len(ST.layers) else str(layer + 1)
            nm = ("출력" if kind == "head" else f"인코더 {lname[1:]}" if lname.startswith("E")
                  else f"디코더 {lname[1:]}" if lname.startswith("D") else f"층 {layer + 1}")
            self.text(nm, (mx, y + 78), 22, TXT, bold=True)
            self.text(KIND_KO.get(kind, kind), (mx, y + 108), 16, CYAN)
            self.text(f"{rows:,} × {cols:,}", (mx, y + 140), 22, WARN, bold=True, mono=True)
            self.text(f"× 벡터 {n}개", (mx, y + 172), 15, TXT)
            ar = rows / max(1, cols)
            bw_, bh_ = (140, max(10, int(140 * ar))) if ar <= 1 else (max(10, int(140 / ar)), 140)
            mr = pygame.Rect(mx, y + 204, bw_, bh_)
            s.fill(mix(BG, VIOLET, 0.3), mr)
            if mode == "run":
                sweep = int((now * 3.0 % 1.0) * mr.h)
                s.fill(VIOLET, (mr.x, mr.y + sweep, mr.w, max(2, mr.h // 12)))
            pygame.draw.rect(s, LINE, mr, 1)
            self.text("가중치 행을 두 엔진이 나눠 읽음", (mx, mr.bottom + 8), 11, DIM)
        else:
            self.text("—", (mx, y + 80), 22, FAINT)

    def stats_dyn(self, mode, temp, power, util, bw, gmacs, macs, calls, el):
        s = self.s
        x, y, w, h = self.STATS
        tcol = mix(OKC, HOT, (temp - 45) / 20)
        self.text(f"{temp:4.1f}°C", (x + 18, y + 62), 34, tcol, bold=True, mono=True)
        hist = list(ST.hist_t)
        if len(hist) > 2:
            gx, gy, gw, gh = x + 220, y + 50, w - 240, 50
            lo, hi = min(hist) - 0.5, max(hist) + 0.5
            pts = [(gx + i * gw / (len(hist) - 1), gy + gh - (v - lo) / (hi - lo) * gh) for i, v in enumerate(hist)]
            pygame.draw.lines(s, tcol, False, pts, 2)
        rows = [(f"{util * 100:5.1f} %", util, WARN), (f"{power:5.2f} W", (power - 1.9) / 0.8, ACC),
                (f"{bw:5.2f} GB/s", bw / 2.4, VIOLET), (f"{gmacs:5.2f} G곱셈/초", gmacs / 9.5, CYAN)]
        yy = y + 124
        for val, frac, col in rows:
            self.text(val, (x + w - 18, yy - 2), 18, TXT, mono=True, anchor="tr")
            fw = int((w - 36) * max(0.0, min(1.0, frac)))
            if fw > 0:
                pygame.draw.rect(s, col, (x + 18, yy + 22, fw, 8), border_radius=4)
            yy += 50
        self.text(f"{macs / 1e9:6.3f} G  ({calls:,}번)" if mode != "idle" else "—", (x + w - 18, yy + 2), 16, TXT,
                  mono=True, anchor="tr")
        self.text(f"{el:5.2f} 초" if mode != "idle" else "—", (x + w - 18, yy + 30), 16, TXT, mono=True, anchor="tr")
        self.text(f"VCCINT {self.vcc:.3f} V", (x + 18, y + h - 26), 12, FAINT, mono=True)

    def layers_dyn(self, mode, layers, ops, cur, done_set, phase):
        if mode == "idle":
            return
        s = self.s
        lx, ly, lw, lh = self.LAYERS
        self.text(phase, (lx + lw - 14, ly + 10), 14, CYAN, anchor="tr")
        n = len(layers)
        cw = (lw - 28 - (n - 1) * 3) / n
        cl = cur[0] if cur else -1
        for i, nm in enumerate(layers):
            if mode == "done" or i in done_set:
                col = mix(PANEL2, OKC, 0.55)
            elif i == cl:
                col = WARN
            else:
                continue
            rr = self._cell(i, n, cw)
            s.fill(col, rr)
            if cw > 18:
                self.text(nm, rr.center, 11 if cw < 40 else 13, _c("1a1a1a") if col == WARN else DIM, anchor="c")
            if i == cl and cur and mode == "run":
                ks = [o[0] for o in ops[min(i, len(ops) - 1)]]
                if cur[1] in ks and len(ks) > 1:
                    k = ks.index(cur[1])
                    sw = rr.w / len(ks)
                    s.fill(_c("fff3d6"), (rr.x + k * sw, rr.bottom - 5, sw, 4))

    def output_dyn(self, mode, out, now):
        if mode == "idle":
            return
        x, y = self.OUTPUT[0], self.OUTPUT[1]
        lines = out.split("\n")[-2:]
        yy = y + 36
        for i, ln in enumerate(lines):
            rr = self.text(ln[-110:], (x + 14, yy), 18, TXT if i == len(lines) - 1 else DIM)
            if mode == "run" and i == len(lines) - 1 and int(now * 2) % 2 == 0:
                self.s.fill(WARN, (rr.right + 3, rr.y + 3, 9, rr.h - 6))
            yy += 26


def main():
    png_dir = sys.argv[2] if len(sys.argv) > 2 and sys.argv[1] == "--png" else None
    sc = Screen()
    if png_dir:                                  # 시험: 가짜 기록으로 몇 장
        os.makedirs(png_dir, exist_ok=True)
        ST.start({"id": 1, "kind": "rachel", "title": "말 → 블록", "model": "더블 레이첼 · 25.6M · int8"})
        ST.real = True
        for i in range(160):
            li, k = (i // 4) % 10, ["qkv", "out", "up", "down"][i % 4]
            shp = {"qkv": (1344, 448), "out": (448, 448), "up": (1792, 448), "down": (448, 1792)}[k]
            ST.event(li, k, shp[0], shp[1], 4 if i < 80 else 1, 300)
            time.sleep(0.004)
        ST.out = "B팔: 보라색 도킹의 펜을 초록색으로 옮겨\n=arm:B move_to:보라 grab:pen"
        ST.phase = "한 글자씩 생성"
        for i in range(3):
            pygame.image.save(sc.draw(), os.path.join(png_dir, f"run{i}.png"))
            time.sleep(0.2)
        handle({"op": "done", "id": 1})
        pygame.image.save(sc.draw(), os.path.join(png_dir, "done.png"))
        ST.start({"id": 2, "kind": "stt", "title": "음성 인식", "model": "Whisper large-v3-turbo · DGX Spark"})
        threading.Thread(target=shape_runner, args=(2,), daemon=True).start()
        handle({"op": "text", "id": 2, "text": "보라색 도킹에 있는 펜을 B팔로 잡아서 초록색에다 갖다 놔"})
        time.sleep(2.6)
        pygame.image.save(sc.draw(), os.path.join(png_dir, "stt.png"))
        handle({"op": "idle"})
        pygame.image.save(sc.draw(), os.path.join(png_dir, "idle.png"))
        return
    fb = FrameBuffer()
    threading.Thread(target=cmd_loop, daemon=True).start()
    RT.warm()                                    # 모델을 미리 올려 둔다 (첫 문장도 바로)
    last_done = 0.0
    while True:
        t0 = time.time()
        if ST.mode == "done" and t0 - ST.t_done > 12.0 and ST.t_done > last_done:
            last_done = ST.t_done
            with ST.lock:
                ST.mode = "idle"
        surf = sc.draw()
        fb.show(surf)
        if SNAP[0]:
            try:
                pygame.image.save(surf, SNAP[0])
            except Exception as e:  # noqa: BLE001
                print("[tpu_show] snap:", e, flush=True)
            SNAP[0] = None
        dt = time.time() - t0
        time.sleep(max(0.02, (0.12 if ST.mode == "run" else 0.5) - dt))


if __name__ == "__main__":
    main()
