# ps7_ac7010c.tcl — Zynq PS7 settings for the ALINX AC7010C core module (DUSK_BOOM robot v3.5)
#
# Every value below comes from the ALINX "AC7010C/AC7020C User Manual" (Rev 1.0):
#   5.1  PS clock          33.333 MHz crystal on PS_CLK (E7)
#   4.2  bank voltages     VCCO_MIO0 (bank 500, MIO0-15) = 3.3 V · VCCO_MIO1 (bank 501, MIO16-53) = 1.8 V
#   6.1  QSPI              W25Q256, MIO1 CS · MIO2-5 D0-3 · MIO6 SCK  (MIO8 is NOT the feedback clock here:
#                          on the robot board MIO8 goes to the JX1 header and is a boot-mode strap)
#   6.2  DDR3              2 x H5TQ2G63FFR-RDC (= MT41J128M16HA-125), 128M x 16 each -> 512 MB, 32 bit,
#                          533 MHz, VCCO_DDR 1.5 V
#                          !! NOT MT41K256M16 (that is the 1 GB AC7020C) — fixes the earlier tpu/vivado/build.tcl
#   6.3  Ethernet          JL2121 RGMII on MIO16-27, MDIO MIO52/53, PHY address 1, PHY adds the 2 ns delays
#   6.4  USB               USB3320C ULPI on MIO28-39, PHY reset MIO46 (active low).
#                          Module J5 + J6 jumper caps ON = HOST (USB-A J3) — the robot adapter plugs in there
#   6.5  UART              CP2102 on MIO48 (TX) / MIO49 (RX) = UART1 -> Linux console ttyPS0
#   6.6  SD                MIO40-45, card detect MIO47 (no write-protect pin)
#   6.7  PS LED            MIO0 (0 = on)
# Robot bridge: UART0 on EMIO (-> ESP_IO0/1 in the PL) and 16 EMIO GPIO.
# Screen: S_AXI_HP0 (64 bit) read by the PL framebuffer scanout (clocked by the PL pixel clock).
# TPU: M_AXI_GP0 (registers at 0x43C00000) + S_AXI_HP1 / S_AXI_HP2 (64 bit), clocked by clk_tpu (148.33 MHz).
#
# DDR board delays: ALINX does not publish them for the AC7010C. The values below are the Digilent
# Zybo ones (same XC7Z010-CLG400, same 2 x 128Mx16 32-bit layout, similar compact board) and all three
# DDR training steps (write levelling, read gate, data eye) are switched on, so ps7_init calibrates.
# If ALINX gives you their own AC7010C preset (.tcl), you may source it instead of this file.

proc ac7010c_ps7_config {ps} {
    # --- clocks, banks -------------------------------------------------------------------
    set cfg [list \
        CONFIG.PCW_CRYSTAL_PERIPHERAL_FREQMHZ {33.333333} \
        CONFIG.PCW_APU_PERIPHERAL_FREQMHZ {666.666666} \
        CONFIG.PCW_PRESET_BANK0_VOLTAGE {LVCMOS 3.3V} \
        CONFIG.PCW_PRESET_BANK1_VOLTAGE {LVCMOS 1.8V} \
        CONFIG.PCW_USE_M_AXI_GP0 {1} \
        CONFIG.PCW_USE_S_AXI_HP1 {1} \
        CONFIG.PCW_S_AXI_HP1_DATA_WIDTH {64} \
        CONFIG.PCW_USE_S_AXI_HP2 {1} \
        CONFIG.PCW_S_AXI_HP2_DATA_WIDTH {64} \
        CONFIG.PCW_USE_M_AXI_GP1 {0} \
        CONFIG.PCW_USE_S_AXI_HP0 {1} \
        CONFIG.PCW_S_AXI_HP0_DATA_WIDTH {64} \
        CONFIG.PCW_EN_CLK0_PORT {0} \
        CONFIG.PCW_EN_RST0_PORT {0} \
    ]
    # --- QSPI (no feedback clock) ------------------------------------------------------
    lappend cfg \
        CONFIG.PCW_QSPI_PERIPHERAL_ENABLE {1} \
        CONFIG.PCW_QSPI_GRP_SINGLE_SS_ENABLE {1} \
        CONFIG.PCW_QSPI_GRP_FBCLK_ENABLE {0} \
        CONFIG.PCW_QSPI_PERIPHERAL_FREQMHZ {200}
    # --- Ethernet 0 ----------------------------------------------------------------------
    lappend cfg \
        CONFIG.PCW_ENET0_PERIPHERAL_ENABLE {1} \
        CONFIG.PCW_ENET0_ENET0_IO {MIO 16 .. 27} \
        CONFIG.PCW_ENET0_GRP_MDIO_ENABLE {1} \
        CONFIG.PCW_ENET0_GRP_MDIO_IO {MIO 52 .. 53} \
        CONFIG.PCW_ENET0_PERIPHERAL_FREQMHZ {1000 Mbps} \
        CONFIG.PCW_ENET0_RESET_ENABLE {0}
    # --- USB 0 (host on the module USB-A), PHY reset on MIO46 ---------------------------------
    lappend cfg \
        CONFIG.PCW_USB0_PERIPHERAL_ENABLE {1} \
        CONFIG.PCW_USB0_USB0_IO {MIO 28 .. 39} \
        CONFIG.PCW_USB0_RESET_ENABLE {1} \
        CONFIG.PCW_USB0_RESET_IO {MIO 46} \
        CONFIG.PCW_USB_RESET_ENABLE {1} \
        CONFIG.PCW_USB_RESET_POLARITY {Active Low}
    # --- SD 0 --------------------------------------------------------------------------------
    lappend cfg \
        CONFIG.PCW_SD0_PERIPHERAL_ENABLE {1} \
        CONFIG.PCW_SD0_SD0_IO {MIO 40 .. 45} \
        CONFIG.PCW_SD0_GRP_CD_ENABLE {1} \
        CONFIG.PCW_SD0_GRP_CD_IO {MIO 47} \
        CONFIG.PCW_SD0_GRP_WP_ENABLE {0} \
        CONFIG.PCW_SDIO_PERIPHERAL_FREQMHZ {50}
    # --- UART1 = console (CP2102), UART0 = robot bridge on EMIO ---------------------------------
    lappend cfg \
        CONFIG.PCW_UART1_PERIPHERAL_ENABLE {1} \
        CONFIG.PCW_UART1_UART1_IO {MIO 48 .. 49} \
        CONFIG.PCW_UART1_BAUD_RATE {115200} \
        CONFIG.PCW_UART0_PERIPHERAL_ENABLE {1} \
        CONFIG.PCW_UART0_UART0_IO {EMIO} \
        CONFIG.PCW_UART0_BAUD_RATE {115200} \
        CONFIG.PCW_UART_PERIPHERAL_FREQMHZ {100}
    # --- GPIO: MIO (LED MIO0, USB reset MIO46 ...) + 16 EMIO -------------------------------------
    lappend cfg \
        CONFIG.PCW_GPIO_MIO_GPIO_ENABLE {1} \
        CONFIG.PCW_GPIO_MIO_GPIO_IO {MIO} \
        CONFIG.PCW_GPIO_EMIO_GPIO_ENABLE {1} \
        CONFIG.PCW_GPIO_EMIO_GPIO_IO {16}
    # --- DDR3 -----------------------------------------------------------------------------
    lappend cfg \
        CONFIG.PCW_UIPARAM_DDR_MEMORY_TYPE {DDR 3} \
        CONFIG.PCW_UIPARAM_DDR_BUS_WIDTH {32 Bit} \
        CONFIG.PCW_UIPARAM_DDR_FREQ_MHZ {533.333313} \
        CONFIG.PCW_UIPARAM_DDR_TRAIN_WRITE_LEVEL {1} \
        CONFIG.PCW_UIPARAM_DDR_TRAIN_READ_GATE {1} \
        CONFIG.PCW_UIPARAM_DDR_TRAIN_DATA_EYE {1} \
        CONFIG.PCW_UIPARAM_DDR_BOARD_DELAY0 {0.176} \
        CONFIG.PCW_UIPARAM_DDR_BOARD_DELAY1 {0.159} \
        CONFIG.PCW_UIPARAM_DDR_BOARD_DELAY2 {0.162} \
        CONFIG.PCW_UIPARAM_DDR_BOARD_DELAY3 {0.187} \
        CONFIG.PCW_UIPARAM_DDR_DQS_TO_CLK_DELAY_0 {-0.073} \
        CONFIG.PCW_UIPARAM_DDR_DQS_TO_CLK_DELAY_1 {-0.034} \
        CONFIG.PCW_UIPARAM_DDR_DQS_TO_CLK_DELAY_2 {-0.03} \
        CONFIG.PCW_UIPARAM_DDR_DQS_TO_CLK_DELAY_3 {-0.082}

    # DDR part: the exact AC7010C equivalent first; older Vivado lists may lack it ->
    # MT41K128M16 JT-125 has the same 128M x 16 geometry and DDR3-1600 timing.
    set part_ok 0
    foreach ddr_part {{MT41J128M16 HA-125} {MT41K128M16 JT-125}} {
        if {![catch {set_property CONFIG.PCW_UIPARAM_DDR_PARTNO $ddr_part $ps} err]} {
            puts "INFO: \[ac7010c\] DDR part = $ddr_part"
            set part_ok 1
            break
        }
        puts "WARNING: \[ac7010c\] DDR part '$ddr_part' not accepted: $err"
    }
    if {!$part_ok} { error "no usable DDR3 part name for the AC7010C" }

    set_property -dict $cfg $ps
}
