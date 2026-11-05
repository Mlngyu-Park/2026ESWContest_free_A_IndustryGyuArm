# build.tcl — DUSK_BOOM robot board v3.5: Zynq PS (robot USB host, Linux) + PL, Vivado on a PC (x86)
#
#   cd fpga_v35/vivado
#   vivado -mode batch -source build.tcl                      (project + bitstream + .xsa)
#   vivado -mode batch -source build.tcl -tclargs project_only (project + block design only, open in GUI)
#
# Output (next to this script):
#   dusk_boom_v35_top.bit   PL bitstream (JTAG, or packed into BOOT.bin)
#   dusk_boom_v35_top.bin   same for Linux fpga_manager / fpgautil
#   system.xsa              hardware platform (PS7 settings + bitstream) -> FSBL (make_boot.tcl)
#                           or U-Boot SPL (../linux/make_sd_from_xsa.sh on the DGX)
#   reports/                timing, utilization, io, drc
#
# Part   : xc7z010clg400-1 (ALINX AC7010C)
# PS7    : ps7_ac7010c.tcl (values from the AC7010C user manual)
# Written for Vivado 2019.2 .. 2024.x (write_hw_platform). NOT run by the author — no x86 Vivado
# host was available; the PL RTL was simulated with Icarus and synthesized with Yosys instead.

set script_dir [file dirname [file normalize [info script]]]
set root       [file normalize [file join $script_dir ..]]
set proj_name  dusk_boom_v35
set proj_dir   [file join $script_dir build]
set part       xc7z010clg400-1
set top        dusk_boom_v35_top
set jobs       4

set project_only 0
if {[llength $argv] > 0 && [lindex $argv 0] eq "project_only"} { set project_only 1 }

source [file join $script_dir ps7_ac7010c.tcl]

create_project -force $proj_name $proj_dir -part $part
set_property target_language Verilog [current_project]
set_property default_lib xil_defaultlib [current_project]

# =====================================================================================
# Block design "system": only the PS7
#   DDR / FIXED_IO        -> top-level PS pins
#   esp_uart (UART0 EMIO) -> esp_uart_txd / esp_uart_rxd
#   GPIO EMIO [15:0]      -> emio_o / emio_t / emio_i (plain buses, no IOBUF in the wrapper)
#   S_AXI_HP0             -> fb_axi + fb_aclk (framebuffer scanout in the PL)
# =====================================================================================
create_bd_design system
set ps [create_bd_cell -type ip -vlnv xilinx.com:ip:processing_system7:5.5 ps7]
apply_bd_automation -rule xilinx.com:bd_rule:processing_system7 \
    -config {make_external "FIXED_IO, DDR" apply_board_preset "0" Master "Disable" Slave "Disable"} $ps
ac7010c_ps7_config $ps

create_bd_intf_port -mode Master -vlnv xilinx.com:interface:uart_rtl:1.0 esp_uart
connect_bd_intf_net [get_bd_intf_pins ps7/UART_0] [get_bd_intf_ports esp_uart]

create_bd_port -dir O -from 15 -to 0 emio_o
create_bd_port -dir O -from 15 -to 0 emio_t
create_bd_port -dir I -from 15 -to 0 emio_i
connect_bd_net [get_bd_pins ps7/GPIO_O] [get_bd_ports emio_o]
connect_bd_net [get_bd_pins ps7/GPIO_T] [get_bd_ports emio_t]
connect_bd_net [get_bd_ports emio_i]    [get_bd_pins ps7/GPIO_I]

# screen: S_AXI_HP0 -> top level "fb_axi" (AXI3 64 bit), clocked by the PL pixel clock (74.25 MHz)
make_bd_intf_pins_external [get_bd_intf_pins ps7/S_AXI_HP0]
set_property name fb_axi [get_bd_intf_ports S_AXI_HP0_0]
create_bd_port -dir I -type clk -freq_hz 74250000 fb_aclk
connect_bd_net [get_bd_ports fb_aclk] [get_bd_pins ps7/S_AXI_HP0_ACLK]
set_property CONFIG.ASSOCIATED_BUSIF {fb_axi} [get_bd_ports fb_aclk]

# TPU (tpu_top.v): registers on M_AXI_GP0 -> "tpu_ctl" (0x43C00000, 64 KB), weights/results on
# S_AXI_HP1 / S_AXI_HP2 -> "tpu_m0" / "tpu_m1" (AXI3 64 bit), all clocked by clk_tpu (148.33 MHz)
make_bd_intf_pins_external [get_bd_intf_pins ps7/M_AXI_GP0]
set_property name tpu_ctl [get_bd_intf_ports M_AXI_GP0_0]
make_bd_intf_pins_external [get_bd_intf_pins ps7/S_AXI_HP1]
set_property name tpu_m0 [get_bd_intf_ports S_AXI_HP1_0]
make_bd_intf_pins_external [get_bd_intf_pins ps7/S_AXI_HP2]
set_property name tpu_m1 [get_bd_intf_ports S_AXI_HP2_0]
create_bd_port -dir I -type clk -freq_hz 148333333 tpu_aclk
connect_bd_net [get_bd_ports tpu_aclk] [get_bd_pins ps7/M_AXI_GP0_ACLK]
connect_bd_net [get_bd_ports tpu_aclk] [get_bd_pins ps7/S_AXI_HP1_ACLK]
connect_bd_net [get_bd_ports tpu_aclk] [get_bd_pins ps7/S_AXI_HP2_ACLK]
set_property CONFIG.ASSOCIATED_BUSIF {tpu_ctl:tpu_m0:tpu_m1} [get_bd_ports tpu_aclk]
assign_bd_address
# TPU registers at the usual first PL address
if {[catch {
    set seg [get_bd_addr_segs -of_objects [get_bd_addr_spaces ps7/Data] -filter {NAME =~ "*tpu_ctl*"}]
    set_property offset 0x43C00000 $seg
    set_property range 64K $seg
    puts "TPU registers: [get_property offset $seg] [get_property range $seg]"
} err]} { puts "WARNING: TPU address: $err" }

validate_bd_design
save_bd_design
set bd_file [get_files system.bd]
generate_target all $bd_file
set wrapper [make_wrapper -files $bd_file -top]
add_files -norecurse $wrapper

# =====================================================================================
# PL sources + constraints
# =====================================================================================
set rtl_files [lsort [glob -directory [file join $root rtl] *.v]]
add_files -norecurse -fileset sources_1 $rtl_files
set_property top $top [get_filesets sources_1]
update_compile_order -fileset sources_1

set xdc_pins   [file join $root constraints dusk_boom_v35_pins.xdc]
set xdc_clock  [file join $root constraints clock.xdc]
set xdc_timing [file join $root constraints timing_v35.xdc]
add_files -norecurse -fileset constrs_1 [list $xdc_pins $xdc_clock $xdc_timing]
set_property used_in_synthesis false [get_files $xdc_timing]
set_property target_constrs_file $xdc_timing [get_filesets constrs_1]

set_property strategy Flow_PerfOptimized_high [get_runs synth_1]
# 7010 has 80 DSP48: 64 TPU multipliers + 1 screen; everything else (adders, counters) in the fabric
set_property STEPS.SYNTH_DESIGN.ARGS.MAX_DSP 66 [get_runs synth_1]
set_property strategy Performance_Explore [get_runs impl_1]

if {$project_only} {
    puts "Project created in $proj_dir (no runs started)."
    return
}

# =====================================================================================
# synthesis / implementation / bitstream
# =====================================================================================
launch_runs synth_1 -jobs $jobs
wait_on_run synth_1
if {[get_property PROGRESS [get_runs synth_1]] ne "100%"} { error "synthesis failed" }

launch_runs impl_1 -to_step write_bitstream -jobs $jobs
wait_on_run impl_1
if {[get_property PROGRESS [get_runs impl_1]] ne "100%"} { error "implementation failed" }

open_run impl_1
set rpt_dir [file join $script_dir reports]
file mkdir $rpt_dir
report_timing_summary -max_paths 20 -report_unconstrained -file [file join $rpt_dir timing_summary.rpt]
report_utilization -hierarchical -file [file join $rpt_dir utilization_hier.rpt]
report_utilization -file [file join $rpt_dir utilization.rpt]
report_clock_utilization -file [file join $rpt_dir clock_utilization.rpt]
report_cdc -details -file [file join $rpt_dir cdc.rpt]
report_io -file [file join $rpt_dir io.rpt]
report_drc -file [file join $rpt_dir drc.rpt]

set bit_src [file join $proj_dir ${proj_name}.runs impl_1 ${top}.bit]
set bit_dst [file join $script_dir ${top}.bit]
file copy -force $bit_src $bit_dst
write_cfgmem -force -format bin -interface SMAPx32 -disablebitswap \
    -loadbit "up 0x0 $bit_dst" [file join $script_dir ${top}.bin]

# hardware platform for the FSBL / U-Boot SPL (contains ps7_init_gpl.c and the bitstream)
set xsa [file join $script_dir system.xsa]
if {[catch {write_hw_platform -fixed -include_bit -force -file $xsa} err]} {
    # Vivado <= 2019.1: .hdf instead of .xsa
    puts "WARNING: write_hw_platform failed ($err) — writing system.hdf instead"
    write_hwdef -force -file [file join $script_dir system.hdf]
}

set wns [get_property STATS.WNS [get_runs impl_1]]
set whs [get_property STATS.WHS [get_runs impl_1]]
puts "================================================================"
puts " DUSK_BOOM robot v3.5 (PS + PL) build done"
puts "   bitstream : $bit_dst"
puts "   platform  : $xsa"
puts "   WNS = $wns ns   WHS = $whs ns   (both must be >= 0)"
puts "   reports   : $rpt_dir"
puts " next: xsct make_boot.tcl   (FSBL + BOOT.bin, needs Vitis)"
puts "   or: copy system.xsa to the DGX -> ../linux/make_sd_from_xsa.sh"
puts "================================================================"
