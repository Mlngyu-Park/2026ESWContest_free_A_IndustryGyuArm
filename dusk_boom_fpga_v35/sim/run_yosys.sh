#!/bin/bash
# Yosys synth_xilinx check of the PL part (PS7 wrapper as black box) — catches syntax / width / port errors
set -e
cd "$(dirname "$0")"
BIN=${OSSCAD:-$HOME/opt/osscad/oss-cad-suite}/bin
$BIN/yosys -q -l yosys.log -p "read_verilog -lib system_wrapper_bb.v; read_verilog -defer ../rtl/*.v; \
  synth_xilinx -family xc7 -top dusk_boom_v35_top -flatten; \
  check -assert; tee -q -o yosys_stat.txt stat -tech xilinx"
sed -n '/Number of cells/,/Estimated/p' yosys_stat.txt | tail -40
