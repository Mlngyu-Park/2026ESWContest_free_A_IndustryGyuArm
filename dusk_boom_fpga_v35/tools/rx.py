# rx.py - receive one file over the serial console (Python 3.4 on the board)
# lines: "D <index> <md5> <base64>" -> answers "A <index>" (ok) or "N <index>" (send again); "END" finishes
import sys, base64, hashlib, os
out, total, want = sys.argv[1], int(sys.argv[2]), sys.argv[3]
part = out + '.part'
f = open(part, 'wb'); n = 0; h = hashlib.sha256()
sys.stdout.write('READY\n'); sys.stdout.flush()
while True:
    line = sys.stdin.readline()
    if not line:
        break
    line = line.strip()
    if not line:
        continue
    if line == 'END':
        break
    ok = False
    try:
        tag, idx, md5, b64 = line.split(' ', 3)
        data = base64.b64decode(b64)
        ok = tag == 'D' and int(idx) == n and hashlib.md5(data).hexdigest() == md5
    except Exception:
        ok = False
    if ok:
        f.write(data); h.update(data); n += 1
        sys.stdout.write('A %d\n' % (n - 1))
    else:
        sys.stdout.write('N %d\n' % n)
    sys.stdout.flush()
f.close()
size = os.path.getsize(part)
if size == total and h.hexdigest() == want:
    os.rename(part, out)
    sys.stdout.write('RX_OK %s %d\n' % (out, size))
else:
    sys.stdout.write('RX_FAIL size=%d sha=%s\n' % (size, h.hexdigest()))
sys.stdout.flush()
