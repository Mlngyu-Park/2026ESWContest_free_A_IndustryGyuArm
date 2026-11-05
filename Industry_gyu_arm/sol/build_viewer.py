#!/usr/bin/env python3
"""viewer_template.html + pe2.json → viewer.html (데이터 내장, 파일 하나로 열림)"""
import io, os, sys, json
HERE = os.path.dirname(os.path.abspath(__file__))
src = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, 'pe2.json')
out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, 'viewer.html')
data = io.open(src, encoding='utf-8').read()
data = data.replace('</script', '<\\/script')          # 안전
tpl = io.open(os.path.join(HERE, 'viewer_template.html'), encoding='utf-8').read()
io.open(out, 'w', encoding='utf-8').write(tpl.replace('__DATA__', data).replace('__SERVER__','0').replace('__PANELS__', 'null').replace('__FILE__', os.path.basename(src) + ' (정적 빌드 ' + __import__('time').strftime('%m-%d %H:%M') + ')'))
print(out, f"{os.path.getsize(out)//1024} KB")
