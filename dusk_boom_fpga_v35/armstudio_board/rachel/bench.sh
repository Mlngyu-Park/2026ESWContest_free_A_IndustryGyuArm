#!/bin/sh
# bench.sh - double Rachel on the board CPU (2 threads): one sentence, prints answer + time
cd /opt/dusk/rachel
for t in 2 1; do
  echo "== threads $t"
  RACHEL_THREADS=$t ./rachel double.bin "$(cat prompt1.txt)"
done
