#!/usr/bin/env python3
"""xfer_prep.py SRC DEST_ON_BOARD OUT_LINES [chunk] - chunk a file for tools/send.ps1 (serial console upload)"""
import base64, hashlib, sys
src, dest, out = sys.argv[1:4]
chunk = int(sys.argv[4]) if len(sys.argv) > 4 else 2048
data = open(src, 'rb').read()
with open(out, 'w', newline='\n') as f:
    f.write('#CMD python3 /tmp/rx.py %s %d %s\n' % (dest, len(data), hashlib.sha256(data).hexdigest()))
    for i in range(0, len(data), chunk):
        c = data[i:i + chunk]
        f.write('D %d %s %s\n' % (i // chunk, hashlib.md5(c).hexdigest(), base64.b64encode(c).decode()))
print(src, len(data), 'bytes ->', (len(data) + chunk - 1) // chunk, 'chunks')
