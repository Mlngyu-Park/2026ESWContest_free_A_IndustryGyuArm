#!/usr/bin/env python3
"""
KYLE 로컬 서버 — 뷰어에서 「검증 다시 실행」이 진짜 sol.py 를 돌리게 한다.

    python3 serve.py            → http://127.0.0.1:8765  브라우저가 열린다 (어느 폴더에서 실행해도 됨)
    python3 serve.py pe2.txt    → 다른 설계 파일
    python3 serve.py --stop     → 떠 있는 서버 끄기 (포트 8765 정리)
    python3 serve.py two_boards.txt --panel B   → 여러 장 파일에서 B 장만 (저장하면 그 장만 되돌려 넣는다)

뷰어(viewer.html)는 그냥 열어도 되지만(정적), 이 서버로 열면:
  - 칩을 옮기거나 연결을 고친 뒤 「검증 다시 실행」 → sol.py 결과가 바로 갱신
  - 「파일로 저장」 → 설계 파일(.txt)이 실제로 덮어써진다 (백업 .bak 남김)

GPU·외부 라이브러리 없음. 파이썬 표준 라이브러리만.
"""
import io, os, sys, json, shutil, webbrowser, threading, time, subprocess, signal, re
from http.server import HTTPServer, BaseHTTPRequestHandler
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sol

HERE = os.path.dirname(os.path.abspath(__file__))
ARGS = [a for a in sys.argv[1:] if not a.startswith('--')]
DESIGN = os.path.abspath(ARGS[0]) if ARGS else os.path.join(HERE, 'pe2.txt')
PANEL = None
if '--panel' in sys.argv: PANEL = sys.argv[sys.argv.index('--panel') + 1]
def _design_text():
    """여러 장 파일이면 --panel 로 고른 장(기본: 첫 장)만 단일 기판 KYLE 로 내준다"""
    global PANEL
    raw = io.open(DESIGN, encoding='utf-8').read()
    d = sol.parse_design(DESIGN, text=raw)
    if not d.panels: return raw
    import panelize
    if PANEL not in d.panels: PANEL = d.panel_order[0]
    return panelize.panel_subdesign(DESIGN, PANEL)

PORT = 8765
LAST_PING = [0.0]        # 브라우저 탭이 살아있다는 신호. 끊기면 서버도 끝난다.

def kill_port_users(port):
    """켤 때 이전에 남은 서버가 포트를 물고 있으면 죽인다 (윈도우/리눅스)."""
    me = os.getpid(); killed = []
    try:
        if os.name == 'nt':
            out = subprocess.run(['netstat','-ano'], capture_output=True, text=True).stdout
            for ln in out.splitlines():
                if f':{port}' in ln and 'LISTENING' in ln:
                    pid = int(ln.split()[-1])
                    if pid != me: subprocess.run(['taskkill','/F','/PID',str(pid)], capture_output=True); killed.append(pid)
        else:
            out = subprocess.run(['ss','-ltnp'], capture_output=True, text=True).stdout
            for ln in out.splitlines():
                if f':{port} ' in ln:
                    for pid in map(int, re.findall(r'pid=(\d+)', ln)):
                        if pid != me: os.kill(pid, signal.SIGTERM); killed.append(pid)
    except Exception as e:
        print('포트 정리 실패(무시):', e)
    if killed: print(f'이전 서버 종료: PID {killed}'); time.sleep(0.5)

class ReusableHTTPServer(HTTPServer):
    allow_reuse_address = True

def build_json_text(text):
    d = sol.parse_design(DESIGN, text=text)
    R, ni, tim = sol.verify(d)
    tmp = os.path.join(HERE, '.serve_tmp.json')
    sol.export_json(d, R, ni, tim, tmp)
    out = io.open(tmp, encoding='utf-8').read()
    os.remove(tmp)
    return out, d

class H(BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def _send(self, code, body, ctype='application/json; charset=utf-8'):
        b = body.encode('utf-8')
        self.send_response(code); self.send_header('Content-Type', ctype); self.send_header('Content-Length', str(len(b))); self.end_headers(); self.wfile.write(b)
    def do_GET(self):
        global PANEL
        if self.path in ('/', '/index.html', '/viewer.html'):
            tpl = io.open(os.path.join(HERE, 'viewer_template.html'), encoding='utf-8').read()
            data, _ = build_json_text(_design_text())
            d0 = sol.parse_design(DESIGN)
            pans = json.dumps(dict(list=d0.panel_order, cur=PANEL or (d0.panel_order[0] if d0.panel_order else None)))
            page = tpl.replace('__DATA__', data.replace('</script', '<\\/script')).replace('__SERVER__', '1').replace('__PANELS__', pans).replace('__FILE__', os.path.basename(DESIGN) + ' (저장 ' + time.strftime('%m-%d %H:%M', time.localtime(os.path.getmtime(DESIGN))) + ', 열람 ' + time.strftime('%H:%M') + ')')
            self._send(200, page, 'text/html; charset=utf-8')
        elif self.path.startswith('/panel?'):
            PANEL = self.path.split('=', 1)[1]; self.send_response(302); self.send_header('Location', '/'); self.end_headers()
        elif self.path == '/ping':
            LAST_PING[0] = time.time(); self._send(200, '{"ok":true}')
        elif self.path == '/quit':
            self._send(200, '{"bye":true}'); threading.Timer(0.3, lambda: os._exit(0)).start()
        elif self.path == '/design.txt':
            self._send(200, _design_text(), 'text/plain; charset=utf-8')
        else:
            self._send(404, '{"error":"not found"}')
    def do_POST(self):
        n = int(self.headers.get('Content-Length', 0)); body = self.rfile.read(n).decode('utf-8')
        try:
            if self.path == '/verify':
                data, _ = build_json_text(body); self._send(200, data)
            elif self.path == '/save':
                d = sol.parse_design(DESIGN, text=body)
                if d.errors:
                    self._send(400, json.dumps({'ok':False,'errors':d.errors}, ensure_ascii=False)); return
                if os.path.exists(DESIGN): shutil.copy(DESIGN, DESIGN + '.bak')
                dm = sol.parse_design(DESIGN)
                if dm.panels:                                   # 여러 장 파일: 이 장만 되돌려 넣는다 (다른 장은 그대로)
                    import panelize
                    info = panelize.merge_panel(DESIGN, PANEL or dm.panel_order[0], body)
                    self._send(200, json.dumps({'ok':True,'path':DESIGN,'backup':DESIGN+'.bak','panel':PANEL,'merged':info}, ensure_ascii=False)); return
                io.open(DESIGN, 'w', encoding='utf-8').write(body)
                self._send(200, json.dumps({'ok':True,'path':DESIGN,'backup':DESIGN+'.bak'}, ensure_ascii=False))
            else:
                self._send(404, '{"error":"not found"}')
        except Exception as e:
            self._send(500, json.dumps({'ok':False,'error':str(e)}, ensure_ascii=False))

def watchdog():
    """브라우저 탭이 닫혀 ping 이 15초 이상 끊기면 서버를 완전히 끝낸다. (처음 60초는 브라우저가 뜨길 기다림)"""
    start = time.time()
    while True:
        time.sleep(3)
        if LAST_PING[0] == 0 and time.time() - start < 60: continue
        if time.time() - max(LAST_PING[0], start) > 15:
            print('브라우저 탭이 닫혀 서버를 종료합니다.'); os._exit(0)

def main():
    if '--stop' in sys.argv:
        kill_port_users(PORT); print('KYLE 서버 종료 (포트 8765 정리됨)'); return
    kill_port_users(PORT)
    srv = ReusableHTTPServer(('127.0.0.1', PORT), H)
    url = f'http://127.0.0.1:{PORT}/'
    print(f"KYLE 서버: {url}   설계: {DESIGN}")
    print("끝내기: 브라우저 탭을 닫거나, 이 창을 닫거나, Ctrl+C")
    if os.environ.get('BROWSER') != 'true': threading.Timer(0.6, lambda: webbrowser.open(url)).start()
    threading.Thread(target=watchdog, daemon=True).start()
    try: srv.serve_forever()
    except KeyboardInterrupt: pass
    finally: os._exit(0)

if __name__ == '__main__':
    main()
