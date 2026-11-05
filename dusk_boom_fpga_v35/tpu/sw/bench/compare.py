# compare.py — same sentences through two rachel builds/modes, count identical answers
import json, os, subprocess, sys
MODEL = sys.argv[1]
runs = [a.split(':') for a in sys.argv[2:]]          # binary:mode:batch
items = json.load(open(os.path.join(os.path.dirname(__file__), '..', 'emu_compare.json')))
hdr = open(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'rachel_board', 'prompt1.txt')).read().split('\n')[:2]
lines = ['\x01'.join(hdr + [it['text'], '>']) for it in items]
outs = []
for b, mode, batch in runs:
    env = dict(os.environ, RACHEL_TPU=mode, RACHEL_BATCH=batch, RACHEL_THREADS='2')
    p = subprocess.run([b, MODEL, '--serve'], input='\n'.join(lines) + '\nQUIT\n', capture_output=True, text=True, env=env)
    o = p.stdout.split('\n')[1:1 + len(lines)]
    outs.append(o)
for i in range(1, len(outs)):
    same = sum(a == b for a, b in zip(outs[0], outs[i]))
    print(f'{runs[0]} vs {runs[i]}: {same}/{len(lines)} same')
    for a, b in zip(outs[0], outs[i]):
        if a != b: print('   ', a[:70], '|', b[:70])
