# bench_3d.py - time the 3D redraw on the board (meshes cached), C renderer vs nothing
import json, sys, time
sys.path.insert(0, "/opt/dusk/screen"); sys.path.insert(0, "/opt/dusk/py")
import armstudio_screen as A
api = A.Api()
api.meta = api.get_json("/api/meta", 30); api.state = api.get_json("/api/state", 30)
api.view = json.loads(api._get(api.viewer + "/state", 30).decode())
sc = A.Screen(api, snapshot=True)
t = time.time(); sc.draw(); print("first draw (mesh load) %.0f ms" % (1000 * (time.time() - t)))
for i in range(3):
    sc.view_cache = None; sc.scene.yaw += 0.2
    t = time.time(); sc.draw(); print("redraw with 3D %.0f ms" % (1000 * (time.time() - t)))
t = time.time(); sc.draw(); print("redraw without 3D %.0f ms" % (1000 * (time.time() - t)))
import pygame
s2 = pygame.Surface((650, 650), 0, 32)
t = time.time(); sc.scene.draw(s2, (0, 0, 650, 650)); print("3D only %.0f ms" % (1000 * (time.time() - t)))
# robot moved, same camera (preview / real run): plates come from the cache
import copy
for i in range(3):
    v = copy.deepcopy(api.view)
    for ln in v["links"]:
        ln["m"][3] += 0.01 * (i + 1)
    api.view = v; api.vver += 1
    t = time.time(); sc.draw(); print("robot moved, redraw %.0f ms" % (1000 * (time.time() - t)))
