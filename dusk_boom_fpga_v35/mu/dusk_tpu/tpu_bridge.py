"""tpu_bridge.py — 뮤(라떼판다) → FPGA 보드의 'TPU 추론 화면' 연결 (2026-11-05)

  선: 뮤 USB-A ↔ 보드 CP2102N(USB-UART) ↔ 보드 시리얼 셸(root).
  보내기: 셸에 'echo <base64 JSON> >> /mnt/debian/run/tpu_cmd' 한 줄 → 보드 tpu_show.py 가 읽는다.
  받기:   보드가 같은 선으로 '@D {json}' 을 보낸다 (fpga_done = 보드 TPU 가 같은 문장을 다 계산함).

  규칙 (사용자 11-05): 계산은 뮤가 하지만, 화면의 연산은 최소 4초.
    뮤가 먼저 끝나도 결과(블록·받아쓴 글)를 붙잡아 두고, 4초가 지나고 보드도 끝났을 때
    '계산 완료' 를 보내고 그 순간 결과를 넘긴다. 4초보다 오래 걸리면 끝날 때까지 돈다.
  FPGA 가 안 꽂혀 있으면 아무것도 붙잡지 않는다 (예전과 똑같이 동작).
"""
import base64
import glob
import json
import os
import threading
import time

try:
    import serial
except Exception:          # pyserial 이 없으면 FPGA 화면만 안 움직인다
    serial = None

MIN_S = float(os.environ.get("DUSK_TPU_MIN_S", "4.0"))
MAX_S = float(os.environ.get("DUSK_TPU_MAX_S", "15.0"))     # 보드가 답을 안 줄 때 이만큼만 기다린다
CMD_PATH = "/mnt/debian/run/tpu_cmd"
_LOCK = threading.Lock()   # 한 번에 한 세션 (말로 만들기·음성이 겹치지 않게)
_CUR = [None]


def fpga_ports():
    """보드의 USB-UART (CP2102N) 경로들 — 로봇 포트로 착각하지 않게 빼는 데도 쓴다."""
    out = set()
    for p in glob.glob("/dev/serial/by-id/*CP2102N*") + glob.glob("/dev/serial/by-id/*CP210*"):
        out.add(p)
        out.add(os.path.realpath(p))
    return out


def _port():
    p = sorted(glob.glob("/dev/serial/by-id/*CP2102N*"))
    return p[0] if p else None


class Session:
    def __init__(self, kind, title, model):
        self.kind, self.title, self.model = kind, title, model
        self.id = int(time.time() * 1000) % 1000000000
        self.t0 = time.time()
        self.ser = None
        self.buf = b""
        self.fpga_done = False
        self.fpga_ms = self.fpga_at = None
        self.prompt_sent = False
        self.alive = False
        p = _port() if serial is not None else None
        if not p:
            return
        try:
            self.ser = serial.Serial(p, 115200, timeout=0.05, write_timeout=1.0)
            self.ser.reset_input_buffer()
            self.ser.write(b"\r\n")
            self.alive = True
            self.send({"op": "start", "id": self.id, "kind": kind, "title": title, "model": model})
        except Exception as e:  # noqa: BLE001
            print(f"[TPU 화면] 보드 연결 실패: {e}")
            self.close()

    def send(self, obj):
        if not self.alive:
            return
        b64 = base64.b64encode(json.dumps(obj, ensure_ascii=False).encode("utf-8")).decode()
        try:
            self.ser.write(f"echo {b64} >> {CMD_PATH}\r".encode())
            self.ser.flush()
        except Exception as e:  # noqa: BLE001
            print(f"[TPU 화면] 보내기 실패: {e}")
            self.close()

    def prompt(self, text):
        """보드 TPU 도 같은 문장을 실제로 계산하게 (더블 레이첼)."""
        if self.alive and not self.prompt_sent:
            self.prompt_sent = True
            self.send({"op": "prompt", "id": self.id, "text": text})

    def text(self, t):
        self.send({"op": "text", "id": self.id, "text": t})

    def relabel(self, model):
        self.send({"op": "start", "id": self.id, "kind": self.kind, "title": self.title, "model": model})

    def _poll(self):
        try:
            self.buf += self.ser.read(4096)
        except Exception:  # noqa: BLE001
            return
        while b"\n" in self.buf:
            line, self.buf = self.buf.split(b"\n", 1)
            line = line.strip()
            if not line.startswith(b"@D "):
                continue
            try:
                j = json.loads(line[3:].decode("utf-8", "replace"))
            except ValueError:
                continue
            if j.get("ev") == "fpga_done" and j.get("id") == self.id:
                self.fpga_done = True
                self.fpga_ms = j.get("ms")
                self.fpga_at = time.time() - self.t0

    def finish(self):
        """최소 4초 · 보드 계산 끝 · (최대 MAX_S) 까지 기다렸다가 '계산 완료'."""
        if not self.alive:
            return
        while True:
            self._poll()
            el = time.time() - self.t0
            if el >= MAX_S:
                break
            if el >= MIN_S and (self.fpga_done or not self.prompt_sent):
                break
            time.sleep(0.03)
        self.send({"op": "done", "id": self.id})
        self.close()

    def close(self):
        self.alive = False
        try:
            if self.ser:
                self.ser.close()
        except Exception:  # noqa: BLE001
            pass
        self.ser = None


class session:
    """with session(...) as s: ... — 들어갈 때 시작, 나올 때(예외 포함) finish."""
    def __init__(self, kind, title, model):
        self.args = (kind, title, model)

    def __enter__(self):
        _LOCK.acquire()
        self.s = Session(*self.args)
        _CUR[0] = self.s
        return self.s

    def __exit__(self, *exc):
        try:
            self.s.finish()
        finally:
            _CUR[0] = None
            _LOCK.release()
        return False


def current():
    return _CUR[0]


def poke(obj):
    """세션 없이 보드 화면에 명령 하나 (확인용 캡처 등)."""
    p = _port() if serial is not None else None
    if not p:
        return False
    b64 = base64.b64encode(json.dumps(obj, ensure_ascii=False).encode("utf-8")).decode()
    with serial.Serial(p, 115200, timeout=0.05, write_timeout=1.0) as s:
        s.write(f"\r\necho {b64} >> {CMD_PATH}\r".encode())
        s.flush()
    return True
