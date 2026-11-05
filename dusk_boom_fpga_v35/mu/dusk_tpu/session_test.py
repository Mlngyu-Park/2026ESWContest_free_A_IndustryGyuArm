"""session_test.py — ARM STUDIO 없이 TPU 화면 세션 하나 시험 (문장을 보드 TPU 로, 2초 때 화면 캡처)."""
import sys, time
sys.path.insert(0, '/home/drboom/dusk_tpu')
import tpu_bridge as TB
P = open('/home/drboom/dusk_tpu/prompt1.txt').read().rstrip('\n')
t0 = time.time()
with TB.session('rachel', '말 → 블록', '더블 레이첼 · 25.6M · int8') as s:
    s.prompt(P); print(f'prompt sent at {time.time() - t0:.2f}s')
    time.sleep(2.0)
    s.send({'op': 'snap', 'path': '/run/tpu_snap_run.png'})
print(f'released after {time.time() - t0:.2f}s  fpga_done={s.fpga_done} board_ms={s.fpga_ms} arrived_at={s.fpga_at}')
time.sleep(0.6)
TB.poke({'op': 'snap', 'path': '/run/tpu_snap_done.png'})
