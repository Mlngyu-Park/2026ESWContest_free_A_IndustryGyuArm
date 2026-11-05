# timing_v35.xdc — extra pins, timing exceptions and bitstream options, DUSK_BOOM robot v3.5 + PS
#
# Primary clocks: pl_clk50 (clock.xdc), f_sck / r_sck 25 MHz (dusk_boom_v35_pins.xdc).
# MMCM outputs are derived by Vivado: CLKOUT0 clk_ser · CLKOUT1 clk_pix (CLKOUT2 unused in v3.5).
# The PS7 has no PL clock / AXI in this design (UART0 + GPIO over EMIO only).

# ---- module PL user LED (AC7010C manual 7.1: LED2 = IO_0_34 = R19, 0 = on) ------------------
set_property -dict {PACKAGE_PIN R19 IOSTANDARD LVCMOS33} [get_ports pl_led_n]

# ---- robot UART bridge: ESP_IO1 (ESP -> FPGA) keeps a defined idle level ---------------------
set_property PULLUP TRUE [get_ports fe_io1]

set clk_pix [get_clocks -of_objects [get_pins u_clk/u_mmcm/CLKOUT1]]

# ---- clock-domain crossings (same structure as the v2.3 design) ----------------------------
# SCK -> 50 MHz : rx_byte / rx_first / ev_tgl (data stable 8 SCK periods = 320 ns @ 25 MHz)
set_max_delay -datapath_only -from [get_clocks f_sck]    -to [get_clocks pl_clk50] 10.000
# 50 MHz -> SCK : tx_slot0/1 (written one byte time before being loaded on a falling edge)
set_max_delay -datapath_only -from [get_clocks pl_clk50] -to [get_clocks f_sck]    15.000
# 50 MHz -> pixel domain: quasi-static pattern / colour registers, 2-FF synchronized
set_max_delay -datapath_only -from [get_clocks pl_clk50] -to $clk_pix              10.000
set_false_path -from $clk_pix -to [get_clocks pl_clk50]
# MMCM LOCKED is used as an asynchronous reset of the pixel-domain reset synchronizer
set_false_path -from [get_pins u_clk/u_mmcm/LOCKED]
# F_CS asynchronously resets the SPI bit/byte state
set_false_path -from [get_ports f_cs]

# ---- SPI IO (CPU = master, mode 0) ------------------------------------------------------------
set_property IOB TRUE   [get_ports f_miso]
set_property SLEW FAST  [get_ports f_miso]
set_property DRIVE 12   [get_ports f_miso]
set_output_delay -clock f_sck -max  7.000 [get_ports f_miso]
set_output_delay -clock f_sck -min -1.000 [get_ports f_miso]
set_input_delay  -clock f_sck -clock_fall -max 4.000 [get_ports f_mosi]
set_input_delay  -clock f_sck -clock_fall -min -1.000 [get_ports f_mosi]

# ---- static / slow / asynchronous IO ---------------------------------------------------------
# fe_io0/1 carry the 1 Mbps PS UART0 (EMIO, no PL clock); the rest are GPIO-speed signals.
set_false_path -from [get_ports {hdmi_hpd_f f_aux fe_io* btn1_f btn2_f}]
set_false_path -to   [get_ports {f_irq fe_io* esp_en_f esp_boot_f fled1_f fled2_f pl_led_n stat_led_f}]

# ---- configuration -----------------------------------------------------------------------
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
# unused PL pins: no internal pull (SCCB / DDC lines have external pull-ups on the board)
set_property BITSTREAM.CONFIG.UNUSEDPIN PULLNONE [current_design]
