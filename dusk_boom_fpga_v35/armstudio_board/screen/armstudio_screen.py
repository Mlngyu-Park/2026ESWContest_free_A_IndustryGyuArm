#!/usr/bin/env python3
"""armstudio_screen.py — ARM STUDIO 보드 화면 (브라우저 없이 HDMI 에 직접 그린다)

  웹 화면(pro_ui) 의 '② 시뮬레이션' 탭과 같은 모양 · 같은 색으로, 보드에 필요한 것만:
    말로 만들기 (AI 추론 · 음성)  ->  파이프라인 (블록)  ->  ▶ 미리보기 (3D) · 실물 실행 · 정지
  계산은 원본 ARM STUDIO 서버(arm_studio_pro.py :7900, 3D 뷰어 :7870)가 그대로 한다. 이 프로그램은
  웹 화면과 같은 호출(/api/state · /api/call · 뷰어 /state · /mesh/<id>)만 쓴다. 원본은 고치지 않는다.

  화면  : /dev/fb0 (PL 이 HDMI 로 내보내는 1280x720 XRGB8888) — 내용이 바뀔 때만 다시 그린다
  커서  : dusk_cursor (C, 따로 돈다) 가 그린다 — 위치는 /dev/shm/dusk_cursor 로 받는다
  입력  : /dev/input/event* (허브의 키보드 · 마우스 · 터치), 한/영 = 한영키 · 오른쪽 Alt (두벌식)

  python3 armstudio_screen.py                              보드에서 (root)
  python3 armstudio_screen.py --snapshot a.png [--api URL] [--viewer URL]   화면 한 장을 그림으로
"""
import json
import math
import mmap
import os
import re
import select
import struct
import sys
import threading
import time
import urllib.parse
import urllib.request

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
import numpy as np  # noqa: E402
import pygame  # noqa: E402

W, H = 1280, 720
API = os.environ.get("ARM_STUDIO_API", "http://127.0.0.1:7900")
VIEWER = os.environ.get("ARM_STUDIO_VIEWER", "http://127.0.0.1:7870")
FONT_DIRS = ["/usr/share/fonts/truetype/nanum", os.path.dirname(os.path.abspath(__file__))]


# ---- 웹 화면(app.css) 과 같은 색 ----------------------------------------------------------------
def _c(h):
    h = h.lstrip("#")
    return tuple(int(h[i:i + 2], 16) for i in (0, 2, 4))


BG, PANEL, PANEL2, LINE = _c("0e1116"), _c("161a22"), _c("1d222d"), _c("262c39")
TXT, DIM, ACC, ACC2 = _c("e7ebf3"), _c("8b93a7"), _c("4c8dff"), _c("2f6ae0")
OK, WARN, DANGER = _c("34c77b"), _c("f5a623"), _c("ef5350")
SC_MOVE, SC_GRASP, SC_REL, SC_GRIP = _c("4c97ff"), _c("59c059"), _c("ff6680"), _c("ffab19")
CAP_START, CAP_END = _c("3fae49"), _c("5a6273")
ARM_CSS = {"빨강": "e53935", "주황": "fb8c00", "노랑": "fdd835", "초록": "43a047", "파랑": "1e88e5", "보라": "8e24aa"}
TICKER = _c("ffcb47")

# AI 연산 안내 (보드 실측 · TPU 계획) — 화면 위쪽에 표시
AI_INFO = os.environ.get("DUSK_AI_INFO", "AI 더블 레이첼 · FPGA TPU ≈2.7초/문장 (CPU만 ≈14초)")
TPU_INFO = os.environ.get("DUSK_TPU_INFO", "TPU 곱셈기 64 (DSP 66/80) · 2엔진 148MHz · 정답 60/60")


def txt_on(bg):
    return (20, 20, 20) if (bg[0] * 299 + bg[1] * 587 + bg[2] * 114) / 1000 > 160 else (255, 255, 255)


# =================================================================================================
# 서버 (웹 화면과 같은 호출)
# =================================================================================================
class Api:
    def __init__(self, base=API, viewer=VIEWER):
        self.base, self.viewer = base, viewer
        self.meta, self.state, self.view = None, None, None
        self.ver = self.vver = 0
        self.alive = False
        self.msg = "서버를 기다리는 중…"
        self.busy_call = None
        self.meshes = {}

    def _get(self, url, timeout=6):
        with urllib.request.urlopen(url, timeout=timeout) as r:
            return r.read()

    def get_json(self, path, timeout=6):
        return json.loads(self._get(self.base + path, timeout).decode())

    def post(self, path, body, timeout=900):
        req = urllib.request.Request(self.base + path, data=json.dumps(body).encode(),
                                     headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return json.loads(r.read().decode())

    def poll_state(self):
        while True:
            try:
                if self.meta is None:
                    self.meta = self.get_json("/api/meta", 20)
                st = self.get_json("/api/state", 20)
                if st != self.state:
                    self.state = st
                    self.ver += 1
                if not self.alive:
                    self.msg = "준비됐습니다 — 말로 만들기에 할 일을 말하거나 입력하세요"
                self.alive = True
            except Exception:
                self.alive = False
            run = bool((self.state or {}).get("running"))
            time.sleep(0.8 if run else 2.0)               # 서버 CPU 를 아낀다 (보드는 2코어)

    def poll_view(self):
        while True:
            try:
                v = json.loads(self._get(self.viewer + "/state", 6).decode())
                key = (v.get("links"), v.get("env"), v.get("path"))
                if self.view is None or key != (self.view.get("links"), self.view.get("env"), self.view.get("path")):
                    self.view = v
                    self.vver += 1
            except Exception:
                pass
            run = bool((self.state or {}).get("running"))
            time.sleep(0.25 if run else 1.5)

    CACHE = "/var/lib/dusk/meshcache"

    def mesh(self, mid):
        """뷰어의 /mesh/<id> STL (다른 팔 'B~l2' = 같은 메쉬). 변환한 것은 디스크에 남겨 다음 부팅에 바로 쓴다."""
        key = mid.split("~", 1)[1] if "~" in mid else mid
        if key not in self.meshes:
            try:
                data = self._get(self.viewer + "/mesh/" + urllib.parse.quote(key), 20)
                import hashlib
                fn = os.path.join(self.CACHE, hashlib.sha1(data).hexdigest()[:16] + ".npz")
                if R3D is not None and os.path.exists(fn):
                    z = np.load(fn)
                    self.meshes[key] = {"V": z["V"], "F": z["F"], "lo": z["lo"], "hi": z["hi"],
                                        "v": z["V"], "f": z["F"]}
                else:
                    self.meshes[key] = parse_stl(data)
                    if R3D is not None:
                        try:
                            os.makedirs(self.CACHE, exist_ok=True)
                            m = self.meshes[key]
                            np.savez(fn, V=m["V"], F=m["F"], lo=m["lo"], hi=m["hi"])
                        except OSError:
                            pass
            except Exception:
                self.meshes[key] = None
        return self.meshes[key]

    def call(self, fn, *args, after=None):
        if self.busy_call and fn != "stop":
            self.msg = f"'{self.busy_call}' 처리 중입니다"
            return

        def work():
            self.busy_call = fn
            try:
                r = self.post("/api/call", {"fn": fn, "args": list(args)})
                if r.get("ok"):
                    st = r.get("status")
                    if r.get("job"):
                        st = self._wait_job()
                    if st:
                        self.msg = str(st)[:160]
                else:
                    self.msg = "실패: " + str(r.get("error"))[:150]
                if after:
                    after(r)
            except Exception as e:
                self.msg = f"연결 실패: {e}"[:160]
            finally:
                self.busy_call = None

        threading.Thread(target=work, daemon=True).start()

    def _wait_job(self):
        while True:
            time.sleep(0.5)
            j = self.get_json("/api/job")
            if j.get("prog"):
                self.msg = str(j["prog"])[:160]
            if not j.get("busy"):
                return j.get("error") or j.get("status") or "완료"


# =================================================================================================
# 화면 출력 · 커서 · 입력
# =================================================================================================
class Cursor:
    """dusk_cursor 와 나누는 /dev/shm/dusk_cursor: int32 x, y, uint32 frame, hidden."""
    PATH = "/dev/shm/dusk_cursor"

    def __init__(self):
        self.mm = None
        try:
            fd = os.open(self.PATH, os.O_RDWR | os.O_CREAT, 0o666)
            if os.fstat(fd).st_size < 16:
                os.ftruncate(fd, 16)
            self.mm = mmap.mmap(fd, 16)
        except OSError:
            pass

    def pos(self):
        if self.mm is None:
            return None
        return struct.unpack_from("<ii", self.mm, 0)

    def bump(self):
        if self.mm is not None:
            f = struct.unpack_from("<I", self.mm, 8)[0]
            struct.pack_into("<I", self.mm, 8, (f + 1) & 0xFFFFFFFF)


class FrameBuffer:
    def __init__(self, dev="/dev/fb0"):
        sysd = "/sys/class/graphics/" + os.path.basename(dev)
        self.w, self.h = map(int, open(sysd + "/virtual_size").read().strip().split(","))
        self.stride = int(open(sysd + "/stride").read())
        self.fd = os.open(dev, os.O_RDWR)
        self.mm = mmap.mmap(self.fd, self.stride * self.h, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE)
        self.cur = Cursor()
        self._tty = None
        try:                                    # 글자 콘솔(fbcon)이 그 위에 그리지 않게
            import fcntl
            self._tty = os.open("/dev/tty1", os.O_RDWR)
            fcntl.ioctl(self._tty, 0x4B3A, 1)   # KDSETMODE, KD_GRAPHICS
        except OSError:
            self._tty = None

    def show(self, surf):
        self.cur.bump()                         # 커서 프로그램: 밑그림이 바뀐다
        if self.stride == self.w * 4:
            self.mm[0:self.stride * self.h] = surf.get_buffer().raw
        else:
            buf, row = surf.get_buffer().raw, surf.get_width() * 4
            for y in range(self.h):
                self.mm[y * self.stride:y * self.stride + row] = buf[y * row:(y + 1) * row]
        self.cur.bump()                         # 다 썼다 — 커서를 새 그림 위에 다시

    def show_rect(self, surf, rect):
        """surf 의 rect 부분만 화면에 (3D 만 바뀌었을 때)."""
        self.cur.bump()
        x, y, w, h = rect
        sub = surf.subsurface(rect).copy().get_buffer().raw
        row = w * 4
        for i in range(h):
            o = (y + i) * self.stride + x * 4
            self.mm[o:o + row] = sub[i * row:(i + 1) * row]
        self.cur.bump()

    def close(self):
        if self._tty is not None:
            try:
                import fcntl
                fcntl.ioctl(self._tty, 0x4B3A, 0)
            except OSError:
                pass


class Hangul:
    """두벌식 한글 조합 (자판 글쇠 -> 낱자 -> 음절)."""
    CHO = "ㄱㄲㄴㄷㄸㄹㅁㅂㅃㅅㅆㅇㅈㅉㅊㅋㅌㅍㅎ"
    JUNG = "ㅏㅐㅑㅒㅓㅔㅕㅖㅗㅘㅙㅚㅛㅜㅝㅞㅟㅠㅡㅢㅣ"
    JONG = ["", "ㄱ", "ㄲ", "ㄳ", "ㄴ", "ㄵ", "ㄶ", "ㄷ", "ㄹ", "ㄺ", "ㄻ", "ㄼ", "ㄽ", "ㄾ", "ㄿ", "ㅀ",
            "ㅁ", "ㅂ", "ㅄ", "ㅅ", "ㅆ", "ㅇ", "ㅈ", "ㅊ", "ㅋ", "ㅌ", "ㅍ", "ㅎ"]
    KEY = dict(zip("qwertasdfgzxcvQWERTyuiophjklbnmOP",
                   "ㅂㅈㄷㄱㅅㅁㄴㅇㄹㅎㅋㅌㅊㅍㅃㅉㄸㄲㅆㅛㅕㅑㅐㅔㅗㅓㅏㅣㅠㅜㅡㅒㅖ"))
    VV = {("ㅗ", "ㅏ"): "ㅘ", ("ㅗ", "ㅐ"): "ㅙ", ("ㅗ", "ㅣ"): "ㅚ", ("ㅜ", "ㅓ"): "ㅝ", ("ㅜ", "ㅔ"): "ㅞ",
          ("ㅜ", "ㅣ"): "ㅟ", ("ㅡ", "ㅣ"): "ㅢ"}
    CC = {("ㄱ", "ㅅ"): "ㄳ", ("ㄴ", "ㅈ"): "ㄵ", ("ㄴ", "ㅎ"): "ㄶ", ("ㄹ", "ㄱ"): "ㄺ", ("ㄹ", "ㅁ"): "ㄻ",
          ("ㄹ", "ㅂ"): "ㄼ", ("ㄹ", "ㅅ"): "ㄽ", ("ㄹ", "ㅌ"): "ㄾ", ("ㄹ", "ㅍ"): "ㄿ", ("ㄹ", "ㅎ"): "ㅀ",
          ("ㅂ", "ㅅ"): "ㅄ"}
    CC_R = {v: k for k, v in CC.items()}
    VV_R = {v: k for k, v in VV.items()}

    def __init__(self):
        self.text = ""
        self.cho = self.jung = self.jong = ""

    def _syl(self):
        if self.cho and self.jung:
            return chr(0xAC00 + (self.CHO.index(self.cho) * 21 + self.JUNG.index(self.jung)) * 28
                       + self.JONG.index(self.jong))
        return self.cho or self.jung

    def commit(self):
        self.text += self._syl()
        self.cho = self.jung = self.jong = ""

    def value(self):
        return self.text + self._syl()

    def plain(self, ch):
        self.commit()
        self.text += ch

    def jamo(self, j):
        if j in self.JUNG:
            if self.jong:
                split = self.CC_R.get(self.jong)
                if split:
                    self.jong, nxt = split
                else:
                    nxt, self.jong = self.jong, ""
                self.commit()
                self.cho, self.jung = nxt, j
            elif self.jung:
                if (self.jung, j) in self.VV and self.cho:
                    self.jung = self.VV[(self.jung, j)]
                else:
                    self.commit()
                    self.jung = j
            else:
                self.jung = j
        else:
            if self.jong:
                if (self.jong, j) in self.CC:
                    self.jong = self.CC[(self.jong, j)]
                else:
                    self.commit()
                    self.cho = j
            elif self.jung and self.cho and j in self.JONG:
                self.jong = j
            else:
                self.commit()
                self.cho = j

    def backspace(self):
        if self.jong:
            split = self.CC_R.get(self.jong)
            self.jong = split[0] if split else ""
        elif self.jung:
            split = self.VV_R.get(self.jung)
            self.jung = split[0] if split and self.cho else ""
        elif self.cho:
            self.cho = ""
        else:
            self.text = self.text[:-1]

    def clear(self):
        self.text = ""
        self.cho = self.jung = self.jong = ""


class Inputs:
    """evdev: 버튼 · 휠 · 키 · 끌기. 커서 위치는 dusk_cursor 의 것을 쓴다."""
    US = dict(zip(range(2, 14), "1234567890-="))
    US.update(zip(range(16, 28), "qwertyuiop[]"))
    US.update(zip(range(30, 41), "asdfghjkl;'"))
    US.update(zip(range(44, 54), "zxcvbnm,./"))
    US.update({41: "`", 43: "\\", 57: " "})
    SHIFT = dict(zip("1234567890-=[];',./`\\", "!@#$%^&*()_+{}:\"<>?~|"))

    def __init__(self, cursor):
        try:
            import evdev
            self.evdev = evdev
        except ImportError:
            self.evdev = None
        self.cursor = cursor
        self.devs = {}
        self.mx, self.my = W // 2, H // 2
        self.clicks, self.keys, self.wheel = [], [], 0
        self.down = False
        self.drag = [0, 0]
        self.shift = False
        self.hangul = True
        self._scan_t = 0

    def where(self):
        p = self.cursor.pos() if self.cursor else None
        if p:
            self.mx, self.my = p
        return self.mx, self.my

    def scan(self):
        if not self.evdev or time.time() - self._scan_t < 2:
            return
        self._scan_t = time.time()
        for path in self.evdev.list_devices():
            if path in self.devs:
                continue
            try:
                d = self.evdev.InputDevice(path)
                caps = d.capabilities()
                e = self.evdev.ecodes
                if e.EV_REL in caps or e.EV_ABS in caps or e.KEY_A in caps.get(e.EV_KEY, []):
                    self.devs[path] = d
            except OSError:
                pass

    def poll(self, timeout):
        self.scan()
        if not self.devs:
            time.sleep(timeout)
            return
        fds = {d.fd: p for p, d in self.devs.items()}
        try:
            r, _, _ = select.select(list(fds), [], [], timeout)
        except (OSError, ValueError):
            r = []
        e = self.evdev.ecodes
        for fd in r:
            p = fds[fd]
            d = self.devs[p]
            try:
                for ev in d.read():
                    self._event(ev, e)
            except BlockingIOError:
                pass
            except OSError:
                self.devs.pop(p, None)

    def _event(self, ev, e):
        if ev.type == e.EV_REL:
            if self.down and ev.code == e.REL_X:
                self.drag[0] += ev.value
            elif self.down and ev.code == e.REL_Y:
                self.drag[1] += ev.value
            elif ev.code == e.REL_WHEEL:
                self.wheel += ev.value
        elif ev.type == e.EV_KEY:
            if ev.code in (e.BTN_LEFT, e.BTN_TOUCH):
                if ev.value == 1:
                    self.down = True
                    self.drag = [0, 0]
                    self.clicks.append(self.where())
                elif ev.value == 0:
                    self.down = False
            elif ev.code in (e.KEY_LEFTSHIFT, e.KEY_RIGHTSHIFT):
                self.shift = ev.value != 0
            elif ev.value in (1, 2):
                if ev.code in (e.KEY_HANGEUL, e.KEY_RIGHTALT):
                    self.hangul = not self.hangul
                elif ev.code == e.KEY_BACKSPACE:
                    self.keys.append(("bs", None))
                elif ev.code in (e.KEY_ENTER, e.KEY_KPENTER):
                    self.keys.append(("enter", None))
                elif ev.code == e.KEY_ESC:
                    self.keys.append(("esc", None))
                elif ev.code == e.KEY_F2:
                    self.keys.append(("mic", None))
                elif ev.code in self.US:
                    ch = self.US[ev.code]
                    if self.shift:
                        ch = self.SHIFT.get(ch, ch.upper())
                    self.keys.append(("ch", ch))


class Mic:
    """허브의 USB 마이크(웹캠 마이크 포함) -> arecord 16 kHz WAV -> 서버 /api/stt (원본).
    서버는 127.0.0.1:18090 (whisper-server 자리) 로 받아쓴다. STT_HOSTS 는 예비 서버."""
    WAV = "/tmp/dusk_mic.wav"

    def __init__(self):
        self.proc = None
        self.t0 = 0

    @staticmethod
    def card():
        try:
            for ln in open("/proc/asound/cards"):
                ln = ln.strip()
                if ln[:1].isdigit() and "USB" in ln.upper():
                    return int(ln.split()[0])
        except OSError:
            pass
        return None

    def recording(self):
        return self.proc is not None and self.proc.poll() is None

    def start(self):
        import subprocess
        c = self.card()
        if c is None:
            return "마이크가 없습니다 — 허브에 USB 마이크(또는 마이크 달린 웹캠)를 꽂으세요"
        try:
            os.remove(self.WAV)
        except OSError:
            pass
        self.proc = subprocess.Popen(["arecord", "-q", "-D", f"plughw:{c},0", "-f", "S16_LE", "-r", "16000",
                                      "-c", "1", "-t", "wav", "-d", "15", self.WAV])
        self.t0 = time.time()
        return None

    def stop(self):
        import signal
        if self.proc is not None:
            if self.proc.poll() is None:
                self.proc.send_signal(signal.SIGINT)
                try:
                    self.proc.wait(3)
                except Exception:
                    self.proc.kill()
            self.proc = None
        try:
            return open(self.WAV, "rb").read()
        except OSError:
            return b""

    @staticmethod
    def transcribe(api, wav):
        hosts = [api.base] + [h.strip() for h in os.environ.get("STT_HOSTS", "").split(",") if h.strip()]
        last = None
        for h in hosts:
            try:
                req = urllib.request.Request(h + "/api/stt", data=wav, headers={"Content-Type": "audio/wav"})
                with urllib.request.urlopen(req, timeout=120) as r:
                    out = json.loads(r.read().decode())
                if out.get("ok") and (out.get("text") or "").strip():
                    return out["text"].strip(), None
                last = out.get("error") or "못 알아들었습니다"
            except Exception as e:
                last = str(e)
        if last and ("111" in last or "refused" in last.lower() or "18090" in last or "WHISPER" in last):
            last = "보드에 음성 인식 엔진(whisper)이 아직 없습니다 — 설치하면 바로 됩니다"
        return None, last


# =================================================================================================
# 3D (뷰어와 같은 장면: 팔 링크 행렬 · 바닥판 · 설비)
# =================================================================================================
def parse_stl(data):
    if data[:5].lower() == b"solid" and b"facet" in data[:512]:
        pts = [list(map(float, ln.split()[1:4])) for ln in data.decode(errors="ignore").splitlines()
               if ln.strip().startswith("vertex")]
        tri = np.array(pts, dtype=np.float64).reshape(-1, 3, 3)
    else:
        n = int(np.frombuffer(data, np.uint32, 1, 80)[0])
        rec = np.frombuffer(data, np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]), n, 84)
        tri = rec["v"].astype(np.float64)
    v, inv = np.unique(tri.reshape(-1, 3).round(6), axis=0, return_inverse=True)
    f = inv.reshape(-1, 3)
    lo, hi = v.min(0), v.max(0)
    out = {"V": np.ascontiguousarray(v, np.float32), "F": np.ascontiguousarray(f, np.int32), "lo": lo, "hi": hi}
    if R3D is None and len(f) > 260:                   # C 렌더러가 없을 때만 줄인다 (파이썬은 느리다)
        try:
            import fast_simplification
            v, f = fast_simplification.simplify(v, f, target_reduction=1 - 220 / len(f))
        except Exception:
            pass
    out["v"], out["f"] = np.asarray(v, float), np.asarray(f, int)
    return out


def _load_r3d():
    """libr3d.so (C, 깊이 버퍼 + 면 음영) — 원본 STL 을 줄이지 않고 그린다."""
    import ctypes
    for p in (os.path.join(os.path.dirname(os.path.abspath(__file__)), "libr3d.so"), "/opt/dusk/screen/libr3d.so"):
        if os.path.exists(p):
            try:
                lib = ctypes.CDLL(p)
                lib.r3d_mesh.restype = ctypes.c_int
                lib.r3d_mesh.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int,
                                         ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p, ctypes.c_int,
                                         ctypes.c_void_p, ctypes.c_void_p, ctypes.c_uint32]
                return lib
            except OSError:
                pass
    return None


R3D = _load_r3d()


class Scene3D:
    def __init__(self, api):
        self.api = api
        self.yaw, self.pitch, self.dist = math.radians(-125), math.radians(30), 0.95
        self.target = np.array([0.2, -0.2, 0.06])

    def draw(self, surf, rect, coarse=False):
        x0, y0, w, h = rect
        surf.fill(_c("2b2b2b"), rect)
        cy, sy, cp, sp = math.cos(self.yaw), math.sin(self.yaw), math.cos(self.pitch), math.sin(self.pitch)
        Rv = np.array([[cy, sy, 0], [-sy * sp, cy * sp, cp], [sy * cp, -cy * cp, sp]])
        f = 0.95 * h
        tgt, dist = self.target, self.dist

        def proj(P):
            c = (P - tgt) @ Rv.T
            z = np.maximum(dist - c[:, 2], 0.05)
            return np.stack([x0 + w / 2 + f * c[:, 0] / z, y0 + h * 0.5 - f * c[:, 1] / z], 1), z

        for a in np.arange(-1.0, 1.01, 0.1):              # 바닥 격자 10 cm
            for P in (np.array([[a, -1.0, 0], [a, 1.0, 0]]), np.array([[-1.0, a, 0], [1.0, a, 0]])):
                s, _ = proj(P)
                pygame.draw.line(surf, _c("3a3a3a"), s[0], s[1], 1)
        v = self.api.view
        if not v:
            return
        if R3D is not None:
            self._draw_c(surf, rect, v, Rv, f)
            path = v.get("path")
            if path and len(path) > 1:
                s, _ = proj(np.array(path, float))
                pygame.draw.lines(surf, (34, 211, 238), False, s.tolist(), 3)
            return
        polys, depth, cols = [], [], []
        light = np.array([0.35, -0.45, 0.82])
        light /= np.linalg.norm(light)

        def add_mesh(md, T, scale, col):
            P = (md["v"] * scale) @ T[:3, :3].T + T[:3, 3]
            s, z = proj(P)
            fc = md["f"]
            tri = P[fc]
            nrm = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
            nrm /= np.linalg.norm(nrm, axis=1, keepdims=True) + 1e-12
            shade = 0.38 + 0.62 * np.abs(nrm @ light)
            base = np.array(col, float)
            zf = z[fc].mean(1)
            for i in range(0, len(fc), 3 if coarse else 1):
                polys.append(s[fc[i]])
                depth.append(zf[i])
                cols.append(tuple(int(c) for c in base * shade[i]))

        for e in v.get("env", []):
            if not e.get("v", True):
                continue
            yw, o = float(e.get("yaw", 0)), e.get("off", [0, 0, 0])
            T = np.array([[math.cos(yw), -math.sin(yw), 0, o[0]], [math.sin(yw), math.cos(yw), 0, o[1]],
                          [0, 0, 1, o[2]], [0, 0, 0, 1]])
            col = _c(e.get("color") or "#999999")
            md = self.api.mesh(e["id"])
            if md is None:
                continue
            sc = float(e.get("scale", 1.0))
            if e.get("plate"):                             # 바닥판: 같은 크기 · 같은 색 판 (구멍 생략)
                lo, hi = md["lo"] * sc, md["hi"] * sc
                top = hi[2]
                q = np.array([[lo[0], lo[1], top], [hi[0], lo[1], top], [hi[0], hi[1], top], [lo[0], hi[1], top]])
                q = q @ T[:3, :3].T + T[:3, 3]
                s, z = proj(q)
                polys.append(s)
                depth.append(z.mean() + 0.02)              # 판은 늘 맨 뒤 (바닥)
                cols.append(col)
            else:
                add_mesh(md, T, sc, col)
        for ln in v.get("links", []):
            md = self.api.mesh(ln["id"])
            if md is None:
                continue
            add_mesh(md, np.array(ln["m"], float).reshape(4, 4), 1.0, _c(ln.get("color") or "#5b8dbf"))
        for i in np.argsort(depth)[::-1]:
            pygame.draw.polygon(surf, cols[i], polys[i])
        path = v.get("path")
        if path and len(path) > 1:                         # 지금 블록의 예상 경로 (청록)
            s, _ = proj(np.array(path, float))
            pygame.draw.lines(surf, (34, 211, 238), False, s.tolist(), 3)


    def _draw_c(self, surf, rect, v, Rv, foc):
        """원본 STL 전부 (판 구멍까지) — C 렌더러, 깊이 버퍼."""
        x0, y0, w, h = rect
        light = np.array([0.35, -0.45, 0.82])
        light /= np.linalg.norm(light)
        cam = np.ascontiguousarray(np.concatenate([Rv.reshape(-1), self.target, [self.dist, foc, x0 + w / 2,
                                                                                  y0 + h * 0.5], light]), np.float32)
        px = pygame.surfarray.pixels2d(surf)                  # 표면 메모리를 그대로 (잠금)
        rows = px.T
        pitch = surf.get_pitch() // 4
        # 바닥판 · 설비 층은 시점이 같으면 그대로 쓴다 (미리보기 중엔 팔만 다시 그린다)
        env_key = (cam.tobytes(), json.dumps(v.get("env", []), sort_keys=True), surf.get_size())
        if getattr(self, "_env_key", None) == env_key:
            rows[:, :] = self._env_rgb
            zb = self._env_z.copy()
            jobs = []
            self._draw_links(v, rows, zb, pitch, surf, cam)
            del rows, px
            return
        zb = np.full(surf.get_height() * surf.get_width(), np.inf, np.float32)
        jobs = []
        for e in v.get("env", []):
            if not e.get("v", True):
                continue
            yw, o = float(e.get("yaw", 0)), e.get("off", [0, 0, 0])
            sc = float(e.get("scale", 1.0))
            M = np.array([[math.cos(yw) * sc, -math.sin(yw) * sc, 0, o[0]],
                          [math.sin(yw) * sc, math.cos(yw) * sc, 0, o[1]], [0, 0, sc, o[2]]])
            jobs.append((e["id"], M, e.get("color") or "#999999"))
        for mid, M, col in jobs:
            md = self.api.mesh(mid)
            if md is None:
                continue
            M = np.ascontiguousarray(M, np.float32)
            c = _c(col)
            R3D.r3d_mesh(rows.ctypes.data, zb.ctypes.data, surf.get_width(), surf.get_height(), pitch,
                         md["V"].ctypes.data, len(md["V"]), md["F"].ctypes.data, len(md["F"]),
                         M.ctypes.data, cam.ctypes.data, (c[0] << 16) | (c[1] << 8) | c[2])
        self._env_key, self._env_rgb, self._env_z = env_key, rows.copy(), zb.copy()
        self._draw_links(v, rows, zb, pitch, surf, cam)
        del rows, px                                          # 잠금 풀기

    def _draw_links(self, v, rows, zb, pitch, surf, cam):
        for ln in v.get("links", []):
            md = self.api.mesh(ln["id"])
            if md is None:
                continue
            M = np.ascontiguousarray(np.array(ln["m"], float).reshape(4, 4)[:3, :], np.float32)
            c = _c(ln.get("color") or "#5b8dbf")
            R3D.r3d_mesh(rows.ctypes.data, zb.ctypes.data, surf.get_width(), surf.get_height(), pitch,
                         md["V"].ctypes.data, len(md["V"]), md["F"].ctypes.data, len(md["F"]),
                         M.ctypes.data, cam.ctypes.data, (c[0] << 16) | (c[1] << 8) | c[2])


# =================================================================================================
# 화면 (웹 '② 시뮬레이션' 탭 모양)
# =================================================================================================
BLK_RE = re.compile(r"^([■◆◈➜▼▲◉⧗])\s*(\d+)\.\s*(.*)$")


def block_parts(label):
    """app.js blockParts() 와 같은 규칙: 라벨 -> (동작, 슬롯, 종류, 색)."""
    m = BLK_RE.match(label)
    body = m.group(3) if m else label
    k = m.group(1) if m else ""
    if k == "➜":
        return "이동 →", re.sub(r"^이동:\s*", "", body), "obj", SC_MOVE
    if k == "■":
        return "이동 →", body, "pose", SC_MOVE
    if k == "◆":
        return "그리퍼", "열기" if "열" in body else "닫기", "grip", SC_GRIP
    if k == "▼":
        return "하강 ▼", "잡는 높이까지", "down", SC_GRIP
    if k == "▲":
        return "상승 ▲", "접근 높이까지", "down", SC_GRIP
    if k == "◉":
        return "팔", re.sub(r"^팔:\s*", "", body), "arm", SC_GRIP
    if k == "⧗":
        return "기다리기", re.sub(r"^기다리기\s*", "", body), "wait", SC_GRIP
    if body.startswith("잡기"):
        return "잡기", re.sub(r"^잡기:\s*", "", body), "brick", SC_GRASP
    return "놓기", re.sub(r"^놓기:\s*", "", body), "dock", SC_REL


class Screen:
    L = (12, 56, 652, 652)          # 3D 카드
    R = (676, 56, 592, 652)         # 오른쪽 카드

    def __init__(self, api, snapshot=False):
        pygame.font.init()
        self.api = api
        self.surf = pygame.Surface((W, H), 0, 32)
        self.f = {k: self._font(*v) for k, v in dict(t=(14,), s=(12,), xs=(11,), b=(14, True), h=(13, True),
                                                       logo=(17, True), blk=(14, True), btn=(13, True)).items()}
        self.fb = None if snapshot else FrameBuffer()
        self.inp = Inputs(self.fb.cur if self.fb else None)
        self.hg = Hangul()
        self.mic = Mic()
        self.scene = Scene3D(api)
        self.sel = None
        self.scroll = 0
        self.speed_i = 1
        self.loop = False
        self.buttons = []
        self.view_cache = None
        self._cache = {}

    def _font(self, size, bold=False):
        names = ["NanumGothicBold.ttf", "NanumBarunGothicBold.ttf"] if bold else ["NanumGothic.ttf", "NanumBarunGothic.ttf"]
        for d in FONT_DIRS:
            for n in names:
                p = os.path.join(d, n)
                if os.path.exists(p):
                    return pygame.font.Font(p, size)
        return pygame.font.Font(None, size + 4)

    GLYPH = str.maketrans({"➜": "→", "◈": "◆", "✋": "", "✊": "", "🤖": "", "⏱": "", "🎤": "●"})

    def _render(self, font, s, col):
        k = (id(font), s, col)
        img = self._cache.get(k)
        if img is None:
            if len(self._cache) > 1500:
                self._cache.clear()
            img = self._cache[k] = font.render(s, True, col)
        return img

    def text(self, s, pos, font="t", col=TXT, maxw=None, center=False):
        fnt = self.f[font]
        s = str(s).translate(self.GLYPH)
        img = self._render(fnt, s, col)
        if maxw and img.get_width() > maxw:
            while s and fnt.size(s + "…")[0] > maxw:
                s = s[:-1]
            img = self._render(fnt, s + "…", col)
        x, y = pos
        if center:
            x -= img.get_width() // 2
        self.surf.blit(img, (x, y))
        return img.get_width()

    def rr(self, rect, col, rad=8, width=0):
        pygame.draw.rect(self.surf, col, rect, width, border_radius=rad)

    def button(self, label, rect, action, col=PANEL2, fg=TXT, enabled=True, border=None, font="btn"):
        r = pygame.Rect(rect)
        self.rr(r, col if enabled else PANEL2, 7)
        if border:
            self.rr(r, border, 7, 1)
        fnt = self.f[font]
        s = str(label).translate(self.GLYPH)
        if fnt.size(s)[0] > r.w - 10:
            while s and fnt.size(s + "…")[0] > r.w - 10:
                s = s[:-1]
            s += "…"
        img = self._render(fnt, s, fg if enabled else DIM)
        self.surf.blit(img, (r.x + (r.w - img.get_width()) // 2, r.y + (r.h - img.get_height()) // 2))
        if enabled:
            self.buttons.append((r, action))

    def card(self, rect, title=None, sub=None):
        self.rr(rect, PANEL, 10)
        self.rr(rect, LINE, 10, 1)
        if title:
            w = self.text(title, (rect[0] + 10, rect[1] + 8), "h")
            if sub:
                self.text(sub, (rect[0] + 18 + w, rect[1] + 10), "xs", DIM, rect[2] - w - 30)

    # ---- 위 막대 ----------------------------------------------------------------------------------
    def header(self, st):
        S = self.surf
        pygame.draw.rect(S, _c("0b0e13"), (0, 0, W, 44))
        pygame.draw.line(S, LINE, (0, 44), (W, 44))
        pygame.draw.circle(S, ACC, (17, 21), 5)
        self.text("ARM STUDIO", (29, 12), "logo")
        tick = pygame.Rect(150, 7, 470, 30)
        self.rr(tick, _c("05070a"), 6)
        self.rr(tick, LINE, 6, 1)
        prog = self.api.msg if self.api.busy_call or not st.get("prog") else st.get("prog")
        self.text(prog, (tick.x + 10, tick.y + 7), "t", TICKER, tick.w - 20)
        self.text(AI_INFO, (630, 6), "xs", DIM, 330)                 # AI · TPU 안내
        self.text(TPU_INFO, (630, 24), "xs", WARN, 330)
        ports = st.get("ports") or {}
        con = [f"{k.upper()} {os.path.basename(str(v))}" for k, v in ports.items() if v] if isinstance(ports, dict) else []
        pill = ("로봇 " + " · ".join(con)) if con else "로봇 없음 — 시뮬레이션만 가능"
        r = pygame.Rect(968, 9, 140, 26)
        self.rr(r, PANEL2, 13)
        self.text(pill, (r.x + 10, r.y + 6), "xs", OK if con else DIM, r.w - 16)
        self.button("⌂ 원위치", (1114, 7, 74, 30), ("home",), _c("222836"))
        self.button("■ 정지", (1194, 7, 74, 30), ("stop",), _c("1b1416"), DANGER, border=DANGER)

    # ---- 오른쪽: ② 시뮬레이션 ----------------------------------------------------------------------
    def right(self, st):
        x, y, w, h = self.R
        self.card(self.R)
        self.text("② 시뮬레이션", (x + 22, y + 10), "b", ACC)
        pygame.draw.line(self.surf, ACC, (x + 14, y + 32), (x + 122, y + 32), 2)
        pygame.draw.line(self.surf, LINE, (x, y + 34), (x + w, y + 34))
        self.say_card((x + 10, y + 44, w - 20, 138))
        self.pipeline_card((x + 10, y + 190, w - 20, 340), st)
        self.run_card((x + 10, y + 538, w - 20, 106), st)

    def say_card(self, rect):
        x, y, w, h = rect
        self.card(rect, "말로 만들기")
        sel = pygame.Rect(x + 10, y + 30, w - 20, 24)
        self.rr(sel, PANEL2, 5)
        self.text("AI 자동 (보드: 더블 레이첼 → 규칙)", (sel.x + 10, sel.y + 5), "s", TXT)
        rec = self.mic.recording()
        if rec:
            self.button(f"● 듣는 중 {time.time() - self.mic.t0:.0f}초 — 다시 누르면 끝", (x + 10, y + 60, 230, 24),
                        ("mic",), DANGER, font="btn")
        else:
            self.button("● 누르고 말하기 (F2)", (x + 10, y + 60, 150, 24), ("mic",), PANEL2, font="btn")
        mic = Mic.card() is not None
        lx = x + 250 if rec else x + 172
        self.text("음성 인식", (lx, y + 65), "xs", DIM)
        self.text(("마이크 있음 · 보드 whisper" if mic else "마이크 없음 — 허브에 꽂으세요"),
                  (lx + 50, y + 65), "xs", OK if mic else DIM, 250)
        box = pygame.Rect(x + 10, y + 90, w - 84, 26)
        self.rr(box, PANEL2, 5)
        val = self.hg.value()
        tw = self.text(val or "예: 펜 잡아서 3번 도킹에 꽂아", (box.x + 10, box.y + 6), "s", TXT if val else DIM, box.w - 34)
        cx = box.x + 10 + (tw if val else 0) + 1
        pygame.draw.line(self.surf, TXT, (cx, box.y + 5), (cx, box.y + 20), 1)
        self.text("한" if self.inp.hangul else "영", (box.right - 18, box.y + 6), "s", ACC)
        self.button("보내기", (box.right + 6, box.y, 58, 26), ("say",), ACC2, enabled=self.api.alive and bool(val.strip()))
        self.text("말하거나 입력하면 블록이 생기고 바로 3D 미리보기가 됩니다. 예: \"펜 잡아\", \"3번 도킹에 꽂아\", "
                  "\"그리퍼 열어\", \"파이프라인 지워\"", (x + 10, y + 122), "xs", DIM, w - 20)

    def pipeline_card(self, rect, st):
        x, y, w, h = rect
        self.card(rect, "파이프라인", "말로 만든 블록 · 클릭 = 선택")
        bx = x + w - 10
        for lab, act, en in [("전부 삭제", ("seq_clear",), bool(st.get("seq"))), ("삭제", ("seq_del",), bool(self.sel)),
                             ("▼", ("seq_move", 1), bool(self.sel)), ("▲", ("seq_move", -1), bool(self.sel))]:
            bw = 64 if len(lab) > 2 else (40 if len(lab) == 2 else 26)
            bx -= bw
            self.button(lab, (bx, y + 6, bw, 22), act, PANEL2, enabled=self.api.alive and en, font="s")
            bx -= 4
        area = pygame.Rect(x + 8, y + 34, w - 16, h - 42)
        clip = self.surf.get_clip()
        self.surf.set_clip(area)
        items = st.get("seq") or []
        names = st.get("names") or {}
        exp = st.get("seq_exp") or []
        reach = st.get("reach") or {}
        arms = st.get("arms") or {"A": "빨강", "B": "파랑"}
        ra = st.get("run_at") or {}
        cur_arm = next(iter(arms), "A")
        rows = []
        for it in items:
            lab = str(it.get("label", ""))
            am = re.match(r"^◉\s*\d+\.\s*팔:\s*([A-Z])", lab)
            if am:
                cur_arm = am.group(1)
            rows.append((it.get("kind"), lab, it, cur_arm))
        heights = [22 if k == "move" else 34 for k, *_ in rows]
        total = 32 + sum(hh + 6 for hh in heights) + 30
        maxs = max(0, total - area.h)
        self.scroll = max(0, min(self.scroll, maxs))
        yy = area.y + 4 - self.scroll
        self.rr((area.x + 6, yy, area.w - 22, 26), CAP_START, 9)              # 시작 모자 블록
        self.text("▶ 시작", (area.x + 18, yy + 6), "blk", (255, 255, 255))
        yy += 32
        for (kind, lab, it, arm), hh in zip(rows, heights):
            if yy > area.bottom or yy + hh < area.y:                          # 안 보이는 줄은 건너뛴다
                if kind != "move":
                    pass
                yy += hh + 6
                continue
            if kind == "move":
                r = pygame.Rect(area.x + 22, yy, area.w - 40, hh)
                pygame.draw.line(self.surf, CAP_END, (r.x + 4, r.y), (r.x + 4, r.bottom), 2)
                self.text(lab.lstrip("┃ ").strip(), (r.x + 14, r.y + 4), "xs", DIM, r.w - 20)
                if lab == self.sel:
                    self.rr(r, WARN, 4, 1)
                self.buttons.append((r, ("sel", lab)))
                yy += hh + 6
                continue
            act, slot, sk, col = block_parts(lab)
            m = BLK_RE.match(lab)
            bi = int(m.group(2)) if m else -1
            r = pygame.Rect(area.x + 6, yy, area.w - 22, hh)
            running = ra.get("running") and ra.get("blk") == bi
            self.rr(r, col, 6)
            pygame.draw.rect(self.surf, PANEL, (r.x + 18, r.y, 22, 4), border_bottom_left_radius=4,
                             border_bottom_right_radius=4)                    # 위 홈
            pygame.draw.rect(self.surf, col, (r.x + 18, r.bottom, 22, 4), border_bottom_left_radius=4,
                             border_bottom_right_radius=4)                    # 아래 탭
            acol = _c(ARM_CSS.get(arms.get(arm, ""), "8b93a7"))
            badge = f"{arm} {arms.get(arm, '')}".strip()
            bw = self.f["xs"].size(badge)[0] + 10
            self.rr((r.x + 10, r.y + 9, bw, 16), acol, 4)
            self.text(badge, (r.x + 15, r.y + 10), "xs", txt_on(acol))
            ax = r.x + 18 + bw
            ax += self.text(act, (ax, r.y + 9), "blk", (255, 255, 255)) + 8
            scol = None
            tgt = it.get("target")
            if tgt and isinstance(names.get(tgt), dict) and names[tgt].get("color"):
                scol = _c(names[tgt]["color"])
            if scol is None:
                scol = _c("fb8c00") if sk in ("dock", "obj") else tuple(int(c * 0.72) for c in col)
            short = re.sub(r"^Industry_gyu_arm_\d+_-_", "", slot or "◯")
            sw = min(self.f["s"].size(short)[0] + 16, 210)
            self.rr((ax, r.y + 7, sw, 20), scol, 10)
            self.text(short, (ax + 8, r.y + 10), "s", txt_on(scol), sw - 12)
            ax += sw + 8
            if 0 <= bi < len(exp):
                on = ra.get("running") and ra.get("blk") == bi and ra.get("sub")
                for stp in (exp[bi] or {}).get("steps", [])[:4]:
                    sw2 = self.f["xs"].size(stp)[0] + 10
                    if ax + sw2 > r.right - 6:
                        break
                    c2 = WARN if on == stp else tuple(int(c * 0.8) for c in col)
                    self.rr((ax, r.y + 10, sw2, 15), c2, 7)
                    self.text(stp, (ax + 5, r.y + 11), "xs", (255, 255, 255))
                    ax += sw2 + 4
            if lab == self.sel or running:
                self.rr(r.inflate(4, 4), (255, 255, 255), 8, 2 if lab == self.sel else 3)
            rc = reach.get(lab)
            if isinstance(rc, dict) and rc.get("ok") is False:                # 도달 불가 = 빨간 !
                pygame.draw.circle(self.surf, (229, 20, 0), (r.x + 3, r.y + 3), 8)
                pygame.draw.circle(self.surf, (255, 255, 255), (r.x + 3, r.y + 3), 8, 2)
                self.text("!", (r.x + 3, r.y - 5), "b", (255, 255, 255), center=True)
            self.buttons.append((r, ("sel", lab)))
            yy += hh + 6
        self.rr((area.x + 6, yy, area.w - 22, 24), CAP_END, 8)
        self.text("끝", (area.x + 18, yy + 5), "blk", (255, 255, 255))
        if not rows:
            self.text("블록이 없습니다 — 위 '말로 만들기' 에 할 일을 말하거나 입력하세요",
                      (area.x + 10, area.y + 70), "s", DIM, area.w - 20)
        self.surf.set_clip(clip)
        if maxs:                                           # 스크롤 막대
            th = max(24, int(area.h * area.h / total))
            ty = area.y + int((area.h - th) * self.scroll / maxs)
            self.rr((area.right - 6, ty, 4, th), _c("4a5263"), 2)

    def run_card(self, rect, st):
        x, y, w, h = rect
        ports = st.get("ports") or {}
        con = isinstance(ports, dict) and any(ports.values())
        self.card(rect, "실행", "미리보기로 확인한 뒤 실물로" if con else "로봇 USB 꽂으면 자동 인식 (지금은 미리보기만)")
        speeds = (self.api.meta or {}).get("speeds") or ["0.5x", "1.0x", "1.5x", "2.0x"]
        self.speed_i %= len(speeds)
        self.text("속도", (x + 10, y + 40), "s", DIM)
        self.button(speeds[self.speed_i], (x + 44, y + 35, 64, 26), ("speed",), PANEL2, font="s")
        self.button(("☑" if self.loop else "☐") + " 반복", (x + 116, y + 35, 70, 26), ("loop",), PANEL2, font="s")
        busy = bool(st.get("running")) or bool(self.api.busy_call)
        self.button("▶ 미리보기", (x + 196, y + 35, 110, 26), ("preview",), ACC2, enabled=self.api.alive and not busy)
        self.button("실물 실행", (x + 312, y + 35, 100, 26), ("run",), _c("1f7a4c"),
                    enabled=self.api.alive and con and not busy)
        self.button("■ 정지", (x + 418, y + 35, 74, 26), ("stop",), _c("1b1416"), DANGER, border=DANGER)
        ra = st.get("run_at") or {}
        if ra.get("running"):
            done, tot = ra.get("done", 0), ra.get("total", 0)
            stat = f"실행 중 {ra.get('elapsed', 0):.1f}초 · 블록 {done}/{tot}"
            if tot:
                pygame.draw.rect(self.surf, LINE, (x + 10, y + 90, w - 20, 5), border_radius=2)
                pygame.draw.rect(self.surf, ACC, (x + 10, y + 90, int((w - 20) * done / max(1, tot)), 5), border_radius=2)
        else:
            stat = "대기 중 0.0초"
        self.text(stat, (x + 10, y + 70), "s", DIM)

    # ---- 전체 ---------------------------------------------------------------------------------------
    def draw(self):
        st = self.api.state or {}
        self.buttons = []
        self.surf.fill(BG)
        self.header(st)
        self.draw_3d()
        self.right(st)

    def draw_3d(self):
        lx, ly, lw, lh = self.L
        self.card(self.L)
        coarse = self.inp.down and self.inp.mx < lx + lw
        key = (self.api.vver, round(self.scene.yaw, 3), round(self.scene.pitch, 3), round(self.scene.dist, 3), coarse)
        if self.view_cache is None or self.view_cache[0] != key:
            sub = pygame.Surface((lw - 2, lh - 2), 0, 32)
            self.scene.draw(sub, (0, 0, lw - 2, lh - 2), coarse)
            self.view_cache = (key, sub)
        self.surf.blit(self.view_cache[1], (lx + 1, ly + 1))
        v = self.api.view or {}
        self.rr((lx + 10, ly + 10, 210, 24), _c("1a1d24"), 6)
        self.text(f"툴 {v.get('tool') or v.get('ee') or ''}", (lx + 18, ly + 15), "xs", TXT, 200)
        self.text("끌기 = 돌리기 · 휠 = 확대", (lx + 12, ly + lh - 22), "xs", DIM)

    def act(self, a):
        api = self.api
        speeds = (api.meta or {}).get("speeds") or ["1.0x"]
        modes = (api.meta or {}).get("modes") or ["간편 이동"]
        run_args = ("간편 이동", modes[0], "x-z-y", speeds[self.speed_i % len(speeds)], self.loop, False)
        k = a[0]
        if k == "sel":
            self.sel = a[1] if self.sel != a[1] else None
        elif k == "preview":
            api.msg = "미리보기 계산 중… (3D 에서만 움직입니다)"
            api.call("preview", *run_args)
        elif k == "run":
            api.msg = "실물 실행"
            api.call("run", *run_args)
        elif k == "stop":
            threading.Thread(target=lambda: api.post("/api/call", {"fn": "stop", "args": []}), daemon=True).start()
            api.msg = "정지"
        elif k == "home":
            api.call("home_zero", "")
        elif k == "seq_del" and self.sel:
            api.call("seq_del", self.sel)
            self.sel = None
        elif k == "seq_move" and self.sel:
            api.call("seq_move", self.sel, a[1])
        elif k == "seq_clear":
            api.call("seq_clear")
            self.sel = None
        elif k == "speed":
            self.speed_i = (self.speed_i + 1) % len(speeds)
        elif k == "loop":
            self.loop = not self.loop
        elif k == "say":
            t = self.hg.value().strip()
            if t:
                api.msg = "AI 가 블록으로 바꾸는 중… (보드 CPU 라 10~20초)  " + t
                api.call("say", t)
                self.hg.clear()
        elif k == "mic":
            if self.mic.recording():
                wav = self.mic.stop()
                if len(wav) < 8000:
                    api.msg = "녹음이 너무 짧습니다"
                    return
                api.msg = "알아듣는 중…"

                def stt():
                    txt, err = Mic.transcribe(api, wav)
                    if txt:
                        api.msg = "들은 말: " + txt
                        self.hg.clear()
                        self.hg.text = txt
                        self.act(("say",))
                    else:
                        api.msg = "음성 인식 실패: " + str(err)[:120]
                threading.Thread(target=stt, daemon=True).start()
            else:
                api.msg = self.mic.start() or "말씀하세요… (다시 누르면 끝, 최대 15초)"

    def handle_input(self):
        inp = self.inp
        for (cx, cy) in inp.clicks:
            for r, a in reversed(self.buttons):
                if r.collidepoint(cx, cy):
                    self.act(a)
                    break
        inp.clicks = []
        lx, ly, lw, lh = self.L
        mx, my = inp.where()
        if inp.down and lx <= mx < lx + lw and ly <= my < ly + lh and inp.drag != [0, 0]:
            self.scene.yaw -= inp.drag[0] * 0.008
            self.scene.pitch = max(0.05, min(1.45, self.scene.pitch + inp.drag[1] * 0.008))
            inp.drag = [0, 0]
        if inp.wheel:
            if mx < lx + lw:
                self.scene.dist = max(0.4, min(4.0, self.scene.dist * (0.9 ** inp.wheel)))
            else:
                self.scroll = max(0, self.scroll - inp.wheel * 40)
            inp.wheel = 0
        for kind, ch in inp.keys:
            if kind == "ch":
                if inp.hangul and ch in Hangul.KEY:
                    self.hg.jamo(Hangul.KEY[ch])
                else:
                    self.hg.plain(ch)
            elif kind == "bs":
                self.hg.backspace()
            elif kind == "enter":
                self.act(("say",))
            elif kind == "esc":
                self.hg.clear()
            elif kind == "mic":
                self.act(("mic",))
        inp.keys = []

    def loop_forever(self):
        last = last3 = None
        n = 0
        while True:
            t0 = time.time()
            self.handle_input()
            key = (self.api.ver, self.api.msg, self.api.alive, self.api.busy_call, self.hg.value(),
                   self.inp.hangul, self.sel, self.speed_i, self.loop, self.scroll, self.mic.recording(),
                   int(t0 - self.mic.t0) if self.mic.recording() else 0)
            key3 = (self.api.vver, self.scene.yaw, self.scene.pitch, self.scene.dist, self.inp.down)
            if key == last and key3 != last3:              # 3D 만 바뀜 (미리보기 · 돌리기) -> 3D 칸만
                t1 = time.time()
                self.draw_3d()
                self.fb.show_rect(self.surf, self.L)
                last3 = key3
                n += 1
                if n <= 5 or n % 50 == 0:
                    print(f"[screen] 3D redraw {1000 * (time.time() - t1):.0f} ms", flush=True)
            elif key != last or key3 != last3:             # 내용이 바뀔 때만 그린다
                t1 = time.time()
                self.draw()
                self.fb.show(self.surf)
                last, last3 = key, key3
                n += 1
                if n <= 5 or n % 50 == 0:
                    print(f"[screen] redraw {1000 * (time.time() - t1):.0f} ms", flush=True)
            self.inp.poll(max(0.0, (1 / 30 if self.inp.down else 1 / 12) - (time.time() - t0)))


def main():
    args = sys.argv[1:]
    snap = args[args.index("--snapshot") + 1] if "--snapshot" in args else None
    api = Api(args[args.index("--api") + 1] if "--api" in args else API,
              args[args.index("--viewer") + 1] if "--viewer" in args else VIEWER)
    sc = Screen(api, snapshot=bool(snap))
    if snap:
        try:
            api.meta = api.get_json("/api/meta", 30)
            api.state = api.get_json("/api/state", 30)
            api.view = json.loads(api._get(api.viewer + "/state", 30).decode())
            api.alive = True
            api.msg = "준비됐습니다 — 말로 만들기에 할 일을 말하거나 입력하세요"
        except Exception as e:
            api.msg = f"서버 없음: {e}"
        sc.hg.text = "빨간 도킹의 펜을 초록"
        t = time.time()
        sc.draw()
        print(f"draw {1000 * (time.time() - t):.0f} ms")
        pygame.image.save(sc.surf, snap)
        print("saved", snap)
        return
    threading.Thread(target=api.poll_state, daemon=True).start()
    threading.Thread(target=api.poll_view, daemon=True).start()
    try:
        sc.surf.fill(BG)
        img = sc.f["logo"].render("ARM STUDIO — 시작하는 중입니다…", True, TXT)
        sc.surf.blit(img, ((W - img.get_width()) // 2, H // 2 - 10))
        sc.fb.show(sc.surf)
        sc.loop_forever()
    finally:
        sc.fb.close()


if __name__ == "__main__":
    main()
