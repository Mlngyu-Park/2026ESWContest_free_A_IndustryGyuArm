"""시험 서버: 앱 없이 뷰어 HTML + 카메라 동기화만 (포트 7991). 앱의 calib.json 을 건드리지 않는다.
    cd cam_grid && conda run -n gyu_arm python tests/viewer_test_server.py
    /test/<cmd> 로 SYNC.ctl(cmd) 를 부른다 (fp, bg, pause ...)"""
import os, sys, json, time, http.server
HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)
import camsync
src = open(os.path.join(HERE, '..', 'arm_studio_web.py'), encoding='utf-8').read()
a = src.index('VIEWER_HTML="""') + len('VIEWER_HTML="""'); HTML = src[a:src.index('"""', a)]
S = camsync.SYNC
print(S.load(os.path.join(HERE, 'shots', 'KakaoTalk_20261006_123735928.mp4')), flush=True)
while S.busy: time.sleep(0.3)
print(S.status, flush=True)
ORIGIN = (-0.09, -0.09)          # 앱 기본 바닥판(타일 0,0 이 GRID_ORG -0.1) 의 첫 목심
class H(http.server.BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def _s(self, c, t, b):
        self.send_response(c); self.send_header('Content-Type', t); self.send_header('Content-Length', str(len(b)))
        self.end_headers(); self.wfile.write(b)
    def do_GET(self):
        p = self.path
        if p == '/': self._s(200, 'text/html', HTML.encode())
        elif p == '/state':
            st = {'links': [], 'env': [], 'ghosts': [], 'pegs': [], 'ptop': 0.02, 'gen': 0, 'psize': 0.2, 'ee': '',
                  'camsync': S.state(ORIGIN, 0.02)}
            self._s(200, 'application/json', json.dumps(st).encode())
        elif p.startswith('/camframe/'): self._s(200, 'image/jpeg', S.jpeg(int(p[10:].split('?')[0])))
        elif p.startswith('/test/'): self._s(200, 'text/plain', S.ctl(p[6:]).encode())
        elif p.startswith('/seek/'): S.k_pause = int(p[6:]); self._s(200, 'text/plain', b'ok')
        elif p == '/kin': self._s(200, 'application/json', b'{}')
        else: self._s(404, 'text/plain', b'')
http.server.ThreadingHTTPServer(('127.0.0.1', 7991), H).serve_forever()
