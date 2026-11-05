import os, sys, time
sys.argv = ["x"]
sys.path.insert(0, "/opt/dusk/tpu_show")
import tpu_show as T
sc = T.Screen()
T.ST.start({"id": 1, "kind": "rachel", "title": "말 → 블록", "model": "더블 레이첼"})
for i in range(50): T.ST.event(i % 10, "up", 1792, 448, 4, 300)
for _ in range(3): sc.draw()
t = time.time()
for _ in range(10): sc.draw()
d = (time.time() - t) / 10
raw = sc.s.get_buffer().raw
import mmap
t = time.time()
for _ in range(10): raw = sc.s.get_buffer().raw
g = (time.time() - t) / 10
print(f"draw {d*1000:.0f} ms  get_buffer {g*1000:.0f} ms")
