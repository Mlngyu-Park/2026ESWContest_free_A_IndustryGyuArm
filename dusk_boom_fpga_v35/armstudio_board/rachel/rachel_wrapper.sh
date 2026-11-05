#!/bin/sh
# rachel (board) — ARM STUDIO 가 부르는 rachel_cpp/rachel 자리에 놓는다 (보드 복사본만, 원본 소스는 그대로)
#   같은 rachel.cpp 에 FPGA TPU 를 붙인 /opt/dusk/rachel/rachel_tpu 를 TPU 모드로 실행한다.
#   TPU 가 없는 비트스트림이면 rachel_tpu 가 알아서 CPU 로 계산한다.
#   예전 CPU 판: rachel_cpp/rachel.cpu_armhf
export RACHEL_TPU="${RACHEL_TPU:-hw}"
export RACHEL_THREADS="${RACHEL_THREADS:-2}"
exec /opt/dusk/rachel/rachel_tpu "$@"
