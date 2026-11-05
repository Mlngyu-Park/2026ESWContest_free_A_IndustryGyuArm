#!/bin/bash
# Icarus Verilog simulation of dusk_boom_v35_top with the PS7 stand-in (DUSK_BOOM robot v3.5)
set -e
cd "$(dirname "$0")"
BIN=${OSSCAD:-$HOME/opt/osscad/oss-cad-suite}/bin
$BIN/iverilog -g2005 -Wall -DSIM -s tb_v35 -o tb_v35.vvp "$@" ../rtl/*.v system_wrapper_sim.v tb_v35.v
$BIN/vvp -n tb_v35.vvp | tee sim.log
grep -c "^PASS" sim.log | xargs echo "PASS lines:"
grep -c "^FAIL" sim.log | xargs echo "FAIL lines:" || true
