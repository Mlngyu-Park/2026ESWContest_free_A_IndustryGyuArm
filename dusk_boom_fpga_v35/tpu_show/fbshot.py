"""fbshot.py — 보드 HDMI 화면(/dev/fb0, XRGB8888)을 PNG 로 (확인용)."""
import os, sys
os.environ["SDL_VIDEODRIVER"] = "dummy"
import numpy as np
import pygame
w, h = map(int, open("/sys/class/graphics/fb0/virtual_size").read().strip().split(","))
stride = int(open("/sys/class/graphics/fb0/stride").read())
a = np.frombuffer(open("/dev/fb0", "rb").read(stride * h), np.uint8).reshape(h, stride // 4, 4)[:, :w, [2, 1, 0]]
pygame.image.save(pygame.image.frombuffer(np.ascontiguousarray(a).tobytes(), (w, h), "RGB"),
                  sys.argv[1] if len(sys.argv) > 1 else "/run/fbshot.png")
print("saved")
