# clock.xdc — PL system clock of the ALINX AC7010C core module (DUSK_BOOM robot v2.3)
#
# Source: ALINX "AC7010C/AC7020C User Manual" Rev 1.0 (2022-09-07), Part 5.2 "PL system clock
# source": "The PL system clock on the AC7010C/AC7020C core board is powered by a 50MHz active
# crystal ... PL Clock pin assignment: 500_CLK -> ZYNQ pin U18".
# U18 = IO_L12P_T1_MRCC_34 on XC7Z010-CLG400 (bank 34, 3.3 V) — clock-capable, checked
# against the Project X-Ray package-pin table for xc7z010clg400-1.
set_property -dict {PACKAGE_PIN U18 IOSTANDARD LVCMOS33} [get_ports pl_clk50]
create_clock -period 20.000 -name pl_clk50 [get_ports pl_clk50]
