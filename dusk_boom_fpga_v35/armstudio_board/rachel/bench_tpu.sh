#!/bin/sh
# bench_tpu.sh - double Rachel: TPU (hw) vs board CPU, same sentence
cd /opt/dusk/rachel
P="$(cat prompt1.txt)"
echo "== TPU"
RACHEL_TPU=hw RACHEL_THREADS=2 ./rachel_tpu double.bin "$P"
echo "== CPU (2 threads)"
RACHEL_TPU=cpu RACHEL_THREADS=2 ./rachel_tpu double.bin "$P"
