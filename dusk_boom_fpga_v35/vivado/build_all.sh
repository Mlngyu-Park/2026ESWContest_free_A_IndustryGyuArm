#!/bin/bash
# build_all.sh — DUSK_BOOM v3.5: Vivado build -> FSBL + BOOT.bin -> ../sd_card   (Linux PC, x86)
#   source /tools/Xilinx/Vivado/<ver>/settings64.sh   (and Vitis settings64.sh for xsct)
#   ./build_all.sh
set -e
cd "$(dirname "$0")"
vivado -mode batch -nojournal -nolog -source build.tcl
if command -v xsct >/dev/null; then
    xsct make_boot.tcl
    echo "OK - copy every file in ../sd_card to the microSD (FAT32)"
else
    echo "Vivado build OK, xsct (Vitis) not found:"
    echo "  copy vivado/system.xsa to the DGX and run linux/make_sd_from_xsa.sh there"
fi
