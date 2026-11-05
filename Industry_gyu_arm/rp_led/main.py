# XIAO RP2040 NeoPixel 상태 표시 — ARM STUDIO 연동판 (main.py)
#  - 대기: 빨간불 계속 켜짐
#  - AI(RACHEL) 추론 중 / BOOT 버튼: 초록 <-> 파랑 번갈아 깜빡임
#
# 앱(라떼판다)이 USB 시리얼로 한 줄 명령을 보낸다:
#   BUSY  -> 깜빡임 시작        IDLE -> 빨간불로 (최소 MIN_BUSY_MS 는 깜빡인 뒤)
#   PING  -> PONG 응답          REPL -> 이 프로그램을 끝내고 REPL 로 (코드 고칠 때)
#
# ★ Ctrl+C(0x03) 로 멈추지 않게 했다 ★
#   로봇이 안 꽂혀 있으면 앱이 로봇 명령을 /dev/ttyACM0 (= 이 보드일 수 있음) 로 보낸다.
#   로봇 명령 안에는 0x03 바이트가 흔해서, 그대로 두면 이 프로그램이 멈춘다.
#   코드를 고칠 때는 먼저 'REPL' 을 보내면 Ctrl+C 가 다시 살아난다.
#
# NeoPixel: 데이터 GPIO12, 전원 GPIO11 (1 = 전원 켜짐)
# 작은 사용자 LED(GPIO16/17/25, Active-Low)는 꺼둠

from machine import Pin, Timer
import neopixel
import rp2
import time
import sys
import select
import micropython

# 작은 사용자 LED 끄기
for _p in (16, 17, 25):
    Pin(_p, Pin.OUT, value=1)

Pin(11, Pin.OUT, value=1)  # NeoPixel 전원 ON
np = neopixel.NeoPixel(Pin(12), 1)

BRIGHT = 60          # 밝기 (0~255)
BLINK_MS = 150       # 깜빡임 간격(ms)
DEMO_MS = 3000       # BOOT 버튼을 뗀 뒤 예시 연산(깜빡임) 시간(ms)
MIN_BUSY_MS = 800    # 추론이 아주 빨리 끝나도 이만큼은 깜빡여서 눈에 보이게
MAX_BUSY_MS = 60000  # IDLE 이 안 오면(앱이 죽음 등) 이 시간 뒤 빨간불로

RED = (BRIGHT, 0, 0)
GREEN = (0, BRIGHT, 0)
BLUE = (0, 0, BRIGHT)

_busy = 0
_phase = 0
_timer = Timer()


def _set(color):
    np[0] = color
    np.write()


def _blink(t):
    global _phase
    _phase ^= 1
    _set(GREEN if _phase else BLUE)


def _start_busy():
    global _busy
    _busy += 1
    if _busy == 1:
        _set(GREEN)
        _timer.init(period=BLINK_MS, mode=Timer.PERIODIC, callback=_blink)


def _stop_busy():
    global _busy
    _busy = max(0, _busy - 1)
    if _busy == 0:
        _timer.deinit()
        _set(RED)


class busy:
    """연산 코드를 `with busy():` 로 감싸면 그동안 초록/파랑이 깜빡입니다."""
    def __enter__(self):
        _start_busy()
        return self

    def __exit__(self, *args):
        _stop_busy()


def computation():
    # 예시 연산 (BOOT 버튼 시험용)
    start = time.ticks_ms()
    total = 0
    i = 0
    while time.ticks_diff(time.ticks_ms(), start) < DEMO_MS:
        total += i
        i += 1
    print("result:", total)


# ---- 앱(호스트) 명령 ----
_host = {'on': False, 't0': 0, 'off_req': False}


def _host_busy():
    if not _host['on']:
        _host['on'] = True
        _start_busy()
    _host['t0'] = time.ticks_ms()
    _host['off_req'] = False


def _host_idle_req():
    if _host['on']:
        _host['off_req'] = True


def _host_tick():
    """IDLE 요청은 최소 시간이 지난 뒤 반영, IDLE 이 안 오면 최대 시간 뒤 강제 종료."""
    if not _host['on']:
        return
    el = time.ticks_diff(time.ticks_ms(), _host['t0'])
    if (_host['off_req'] and el >= MIN_BUSY_MS) or el >= MAX_BUSY_MS:
        _host['on'] = False
        _host['off_req'] = False
        _stop_busy()


def main():
    micropython.kbd_intr(-1)          # 0x03 이 와도 멈추지 않게 (위 설명)
    _set(RED)                         # 대기 상태: 빨간불
    print("ready - ARM STUDIO LED (BUSY/IDLE/PING/REPL)")
    poll = select.poll()
    poll.register(sys.stdin, select.POLLIN)
    buf = ''
    while True:
        # ① 앱 명령 읽기 (막히지 않게 있는 만큼만)
        while poll.poll(0):
            ch = sys.stdin.read(1)
            if ch in ('\n', '\r'):
                cmd = buf.strip().upper()
                buf = ''
                if cmd == 'BUSY':
                    _host_busy()
                elif cmd == 'IDLE':
                    _host_idle_req()
                elif cmd == 'PING':
                    print('PONG')
                elif cmd == 'REPL':
                    micropython.kbd_intr(3)   # Ctrl+C 다시 살림
                    _timer.deinit()
                    _set(RED)
                    print('REPL 로 나갑니다')
                    return
            elif ' ' <= ch <= '~':
                buf = (buf + ch)[-16:]      # 로봇 명령 같은 잡음은 버리고 짧게만
        _host_tick()
        # ② BOOT 버튼 시험 (누르고 있는 동안 + 예시 연산이 끝날 때까지 깜빡임)
        if rp2.bootsel_button():
            with busy():
                while rp2.bootsel_button():
                    time.sleep_ms(10)
                computation()
        time.sleep_ms(20)


main()
