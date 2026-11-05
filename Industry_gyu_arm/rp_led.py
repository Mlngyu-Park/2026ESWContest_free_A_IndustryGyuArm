"""RP2040(XIAO) NeoPixel 상태 표시 — AI 추론 중이면 초록/파랑 깜빡임, 평소엔 빨강.

보드 쪽 코드는 rp_led/main.py. 이쪽은 USB 시리얼로 한 줄 명령만 보낸다 (BUSY / IDLE).
보드가 없거나 오류가 나도 조용히 넘어간다 — LED 때문에 앱이 멈추면 안 된다.

    import rp_led
    with rp_led.busy():
        ...추론...
"""
import glob
import os
import threading
import time

try:
    import serial
except Exception:          # pyserial 이 없으면 LED 만 안 켜진다
    serial = None

_LOCK = threading.Lock()
_S = {'ser': None, 'tried': 0.0, 'n': 0}
RETRY_S = 5.0              # 보드를 못 찾았으면 이 간격으로만 다시 찾는다


PORT_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'rp_led_port.txt')


def port():
    """LED 보드의 고정 경로. 없으면 None.
    ① 환경변수 ARM_LED_PORT  ② rp_led_port.txt (한 줄) — MicroPython 보드가 여럿일 때 이걸로 고정
    ③ 없으면 첫 MicroPython 보드(USB VID 2e8a). 로봇 어댑터(CH343 1a86 / FTDI 0403)와는 이름으로 구분된다."""
    for c in (os.environ.get('ARM_LED_PORT'),
              (open(PORT_FILE).read().strip() if os.path.exists(PORT_FILE) else None)):
        if c:
            return c if os.path.exists(c) else None   # 지정한 보드가 없으면 다른 보드를 건드리지 않는다
    for p in sorted(glob.glob('/dev/serial/by-id/*')):
        low = p.lower()
        if 'micropython' in low or '2e8a' in low:
            return p
    return None


def _send(cmd):
    if serial is None:
        return False
    with _LOCK:
        if _S['ser'] is None:
            now = time.time()
            if now - _S['tried'] < RETRY_S:
                return False
            _S['tried'] = now
            p = port()
            if not p:
                return False
            try:
                _S['ser'] = serial.Serial(p, 115200, timeout=0, write_timeout=0.3)
            except Exception:
                _S['ser'] = None
                return False
        try:
            _S['ser'].write((cmd + '\n').encode())
            _S['ser'].flush()
            return True
        except Exception:          # 뽑혔다 꽂힘 등 -> 다음에 다시 연다
            try:
                _S['ser'].close()
            except Exception:
                pass
            _S['ser'] = None
            _S['tried'] = 0.0
            return False


def on():
    """추론 시작. 겹쳐 불려도 마지막 off 때까지 깜빡임 유지."""
    with _LOCK:
        _S['n'] += 1
        first = _S['n'] == 1
    if first:
        _send('BUSY')


def off():
    with _LOCK:
        _S['n'] = max(0, _S['n'] - 1)
        last = _S['n'] == 0
    if last:
        _send('IDLE')


class busy:
    def __enter__(self):
        try:
            on()
        except Exception:
            pass
        return self

    def __exit__(self, *a):
        try:
            off()
        except Exception:
            pass
        return False
