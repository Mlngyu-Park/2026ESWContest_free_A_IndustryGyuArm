# make_boot.tcl — FSBL from system.xsa + BOOT.bin (FSBL + bitstream + U-Boot), run with Vitis xsct
#
#   cd fpga_v35/vivado
#   xsct make_boot.tcl                 (uses ./system.xsa and ./dusk_boom_v35_top.bit)
#   xsct make_boot.tcl other.xsa
#
# Output: ../sd_card/  = everything for the FAT32 partition of the microSD
#   BOOT.bin     FSBL (DDR / MIO / clocks of the AC7010C) + PL bitstream + U-Boot
#   zImage       Linux kernel with the built-in root file system (dusk_robotd inside)
#   system.dtb   device tree for the AC7010C on the robot board
#
# Needs xsct + bootgen in PATH (Vitis 2019.2 .. 2024.x "Vitis Shell" / settings64). If only Vivado is
# installed (no xsct), use the other path: copy system.xsa to the DGX and run
# ../linux/make_sd_from_xsa.sh — it builds a U-Boot SPL from the same xsa instead of an FSBL.

set script_dir [file dirname [file normalize [info script]]]
set root       [file normalize [file join $script_dir ..]]
set xsa [expr {[llength $argv] > 0 ? [file normalize [lindex $argv 0]] : [file join $script_dir system.xsa]}]
set bit [file join $script_dir dusk_boom_v35_top.bit]
set uboot [file join $root sd u-boot.elf]
set out [file join $root sd_card]
set work [file join $script_dir boot_build]

foreach f [list $xsa $bit $uboot [file join $root sd zImage] [file join $root sd system.dtb]] {
    if {![file exists $f]} { error "missing $f" }
}
file mkdir $out $work

# ---- FSBL ---------------------------------------------------------------------------
# console of the FSBL / standalone BSP = UART1 (CP2102 on the module), not UART0 (robot bridge)
set fsbl_dir [file join $work fsbl]
file delete -force $fsbl_dir
set hw [hsi::open_hw_design $xsa]
if {[catch {
    hsi::create_sw_design fsbl_sw -proc ps7_cortexa9_0 -app zynq_fsbl
    hsi::set_property CONFIG.stdout ps7_uart_1 [hsi::get_os]
    hsi::set_property CONFIG.stdin  ps7_uart_1 [hsi::get_os]
    hsi::generate_app -dir $fsbl_dir -compile
} err]} {
    puts "WARNING: custom FSBL BSP failed ($err) — plain zynq_fsbl template"
    file delete -force $fsbl_dir
    hsi::generate_app -hw $hw -os standalone -proc ps7_cortexa9_0 -app zynq_fsbl -compile -sw fsbl -dir $fsbl_dir
}
set fsbl [file join $fsbl_dir executable.elf]
if {![file exists $fsbl]} { error "FSBL build failed: $fsbl not found" }
hsi::close_hw_design $hw

# ---- BOOT.bin -------------------------------------------------------------------------
set bif [file join $work boot.bif]
set fh [open $bif w]
puts $fh "the_ROM_image:"
puts $fh "\{"
puts $fh "  \[bootloader\] $fsbl"
puts $fh "  $bit"
puts $fh "  $uboot"
puts $fh "\}"
close $fh
# -ignorestderr: bootgen prints warnings on stderr, which would otherwise abort exec
exec -ignorestderr bootgen -arch zynq -image $bif -o [file join $out BOOT.bin] -w on >@ stdout 2>@ stderr

file copy -force [file join $root sd zImage] [file join $root sd system.dtb] $out
if {[file exists [file join $root sd boot.scr]]} { file copy -force [file join $root sd boot.scr] $out }

puts "================================================================"
puts " microSD files ready in $out"
puts "   FAT32 first partition, copy every file to the top folder"
puts "   module J13 -> SD, J5 + J6 caps on (USB HOST), robot adapter -> module USB-A"
puts "================================================================"
