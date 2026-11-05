// dusk_boom_v35_top.v — DUSK_BOOM robot board v3.5 FPGA top level (2026-11-05)
//   ALINX AC7010C core module (Zynq XC7Z010-1CLG400), PL banks 34/35 at 3.3 V
//
//   PS (ARM, Linux) — block design "system" (../vivado/build.tcl), only the PS7 itself:
//     * DDR3, QSPI, Ethernet, USB0 HOST (module USB-A, J5/J6 caps on), SD0, UART1 = console (CP2102)
//     * UART0 over EMIO  -> robot bridge to the ESP32-S3 on ESP_IO0 / ESP_IO1 (1 Mbps)
//     * GPIO EMIO[15:0]  -> ESP reset / boot, front LEDs, buttons, bridge status, screen (table below)
//     * S_AXI_HP0        <- framebuffer scanout (fb_scanout.v): Linux screen in DDR -> HDMI
//   PL (this file):
//     * SPI slave for the ESP32-S3 (SPI_PROTOCOL.md, ID 0xB0, VERSION 0x35) — spi_regs_v35.v
//     * HDMI 1280x720@60: Linux screen (EMIO 8 = 1) or the v2.3 test pattern (needs F3 on the board)
//     * ESP_IO0..7 (ESP_IO0/1 = robot UART bridge, 2..7 general purpose)
//     * module PL LED (R19): slow blink = bitstream loaded · fast blink = robot bridge running
//                            · steady on = robot USB adapter open
//     * every other v3.5 pin (RP2040 SPI, e-ink, JX headers, EXP) is high-Z
//
// Robot path:  ESP32-S3 IO5 -> ESP_IO1 -> PS UART0 RX -> Linux dusk_robotd -> /dev/ttyUSB0|ttyACM0
//              (servo adapter on the module USB-A) -> Feetech bus -> reply back the same way to
//              ESP32-S3 IO4 via ESP_IO0.
//
// EMIO GPIO (Linux gpio number = zynq_gpio base + 54 + n):
//   n  dir  signal
//   0  out  esp_en_f    1 = hold the ESP32-S3 in reset (QE1 pulls EN low)
//   1  out  esp_boot_f  1 = pull ESP32-S3 IO0 low (download mode on the next reset)
//   2  out  fled1_f     front LED1 (J15-5), 1 = on
//   3  out  fled2_f     front LED2 (J15-6), 1 = on
//   4  in   btn1        front button 1 (J15-3), 1 = pressed
//   5  in   btn2        front button 2 (J15-4), 1 = pressed
//   6  out  robot_open  Linux: robot USB serial adapter is open      (STATUS bit4, LED)
//   7  out  bridge_on   Linux: dusk_robotd runs -> FPGA drives ESP_IO0 (STATUS bit3, LED)
//   8  out  fb_enable   1 = HDMI shows the Linux screen (XRGB8888 1280x720 at 0x1FC00000, stride
//                       5120), 0 = test pattern; switched at a frame boundary
//   9  in   hpd         HDMI hot plug: 1 = a monitor is connected and powered
//  10  in   locked      HDMI pixel clock (MMCM) locked
//  11  in   fb_late     screen fetch was late at least once since EMIO 8 was set (should stay 0)
// An EMIO output only reaches the PL while Linux has set it as an output (GPIO_T = 0); after
// power-on every output above is 0, so the ESP is never held in reset by default.
//
// Port names match ../constraints/dusk_boom_v35_pins.xdc (copy of 로봇판/v3.5/dusk_boom_robot_v35.xdc)
// plus pl_clk50 (clock.xdc) and pl_led_n (timing_v35.xdc). DDR_* / FIXED_IO_* are the fixed PS pins.
`timescale 1ns / 1ps
`default_nettype none

module dusk_boom_v35_top (
    // ---- Zynq PS fixed pins (no xdc needed) ----
    inout  wire [14:0] DDR_addr,
    inout  wire [2:0]  DDR_ba,
    inout  wire        DDR_cas_n,
    inout  wire        DDR_ck_n,
    inout  wire        DDR_ck_p,
    inout  wire        DDR_cke,
    inout  wire        DDR_cs_n,
    inout  wire [3:0]  DDR_dm,
    inout  wire [31:0] DDR_dq,
    inout  wire [3:0]  DDR_dqs_n,
    inout  wire [3:0]  DDR_dqs_p,
    inout  wire        DDR_odt,
    inout  wire        DDR_ras_n,
    inout  wire        DDR_reset_n,
    inout  wire        DDR_we_n,
    inout  wire        FIXED_IO_ddr_vrn,
    inout  wire        FIXED_IO_ddr_vrp,
    inout  wire [53:0] FIXED_IO_mio,
    inout  wire        FIXED_IO_ps_clk,
    inout  wire        FIXED_IO_ps_porb,
    inout  wire        FIXED_IO_ps_srstb,

    // ---- module ----
    input  wire pl_clk50,                 // 50 MHz PL oscillator (U18)
    output wire pl_led_n,                 // module PL user LED (R19), 0 = on

    // ---- ESP32-S3 SPI (ESP = master) ----
    input  wire f_sck,
    input  wire f_mosi,
    output wire f_miso,
    input  wire f_cs,
    output wire f_irq,
    input  wire f_aux,

    // ---- ESP32-S3 lines ----
    inout  wire fe_io0, fe_io1, fe_io2, fe_io3, fe_io4, fe_io5, fe_io6, fe_io7,
    output wire esp_en_f,                 // 1 = ESP reset (QE1)
    output wire esp_boot_f,               // 1 = ESP IO0 low (QE2)

    // ---- XIAO RP2040 SPI (not fitted in the minimal assembly) ----
    inout  wire r_sck, r_mosi, r_miso, r_cs, r_irq,

    // ---- HDMI ----
    output wire hdmi_d0_p, hdmi_d0_n,
    output wire hdmi_d1_p, hdmi_d1_n,
    output wire hdmi_d2_p, hdmi_d2_n,
    output wire hdmi_ck_p, hdmi_ck_n,
    inout  wire hdmi_scl_f,
    inout  wire hdmi_sda_f,
    input  wire hdmi_hpd_f,

    // ---- e-ink (J13 on the robot board), status RGB, front panel ----
    inout  wire epd_din, epd_clk, epd_cs, epd_dc, epd_rst, epd_busy,
    output wire stat_led_f,
    input  wire btn1_f, btn2_f,           // 0 = pressed (10k pull-up on the board)
    output wire fled1_f, fled2_f,         // 1 = on

    // ---- expansion J7 ----
    inout  wire exp0, exp1, exp2, exp3, exp4, exp5, exp6, exp7, exp8, exp9,

    // ---- JX1 / JX2 spare PL pins ----
    inout  wire x_w14, x_v15, x_v12, x_y14, x_u12, x_u13, x_t10, x_w13, x_t12, x_t11,
    inout  wire x_l14, x_l15, x_m14, x_m15, x_n15, x_n16, x_v16, x_w16, x_v17, x_v18,
    inout  wire x_r16, x_y19, x_l16, x_r17, x_k16, x_l17, x_u20, x_j16, x_t17, x_t20,
    inout  wire x_m20, x_r18, x_l20, x_m19, x_p15, x_l19, x_n17, x_p16, x_p18, x_k17,
    inout  wire x_g18
);
    // =====================================================================================
    // Zynq PS (block design wrapper)
    // =====================================================================================
    wire        clk50, clk_pix, clk_ser, clk_cam_unused, mmcm_locked;   // from clkgen below
    wire        uart_tx, uart_rx;
    wire [15:0] emio_o, emio_t, emio_i;

    // screen: AXI3 read master of the framebuffer scanout -> S_AXI_HP0 (clocked by clk_pix)
    wire [31:0] fb_araddr;
    wire [3:0]  fb_arlen, fb_arcache, fb_arqos;
    wire [2:0]  fb_arsize, fb_arprot;
    wire [1:0]  fb_arburst, fb_arlock;
    wire [5:0]  fb_arid;
    wire        fb_arvalid, fb_arready;
    wire [63:0] fb_rdata;
    wire        fb_rvalid, fb_rlast, fb_rready;
    wire [5:0]  fb_rid_unused, fb_bid_unused;
    wire [1:0]  fb_rresp_unused, fb_bresp_unused;
    wire        fb_awready_unused, fb_wready_unused, fb_bvalid_unused;
    wire        fb_late;

    // TPU (tpu_top.v): registers from M_AXI_GP0 (tpu_ctl), weights/results on S_AXI_HP1/HP2 (tpu_m0/m1)
    wire        clk_tpu;
    wire [31:0] tc_awaddr, tc_wdata, tc_araddr, tc_rdata;
    wire [11:0] tc_awid, tc_arid, tc_bid, tc_rid;
    wire [3:0]  tc_arlen;
    wire        tc_awvalid, tc_awready, tc_wlast, tc_wvalid, tc_wready, tc_bvalid, tc_bready;
    wire        tc_arvalid, tc_arready, tc_rlast, tc_rvalid, tc_rready;
    wire [63:0] tm_araddr, tm_awaddr;
    wire [7:0]  tm_arlen, tm_awlen;
    wire [1:0]  tm_arvalid, tm_arready, tm_rvalid, tm_rlast, tm_rready;
    wire [1:0]  tm_awvalid, tm_awready, tm_wlast, tm_wvalid, tm_wready, tm_bvalid, tm_bready;
    wire [127:0] tm_rdata, tm_wdata;

    system_wrapper u_ps (
        .DDR_addr          (DDR_addr),
        .DDR_ba            (DDR_ba),
        .DDR_cas_n         (DDR_cas_n),
        .DDR_ck_n          (DDR_ck_n),
        .DDR_ck_p          (DDR_ck_p),
        .DDR_cke           (DDR_cke),
        .DDR_cs_n          (DDR_cs_n),
        .DDR_dm            (DDR_dm),
        .DDR_dq            (DDR_dq),
        .DDR_dqs_n         (DDR_dqs_n),
        .DDR_dqs_p         (DDR_dqs_p),
        .DDR_odt           (DDR_odt),
        .DDR_ras_n         (DDR_ras_n),
        .DDR_reset_n       (DDR_reset_n),
        .DDR_we_n          (DDR_we_n),
        .FIXED_IO_ddr_vrn  (FIXED_IO_ddr_vrn),
        .FIXED_IO_ddr_vrp  (FIXED_IO_ddr_vrp),
        .FIXED_IO_mio      (FIXED_IO_mio),
        .FIXED_IO_ps_clk   (FIXED_IO_ps_clk),
        .FIXED_IO_ps_porb  (FIXED_IO_ps_porb),
        .FIXED_IO_ps_srstb (FIXED_IO_ps_srstb),
        .esp_uart_rxd      (uart_rx),
        .esp_uart_txd      (uart_tx),
        .emio_i            (emio_i),
        .emio_o            (emio_o),
        .emio_t            (emio_t),
        .fb_aclk           (clk_pix),
        .fb_axi_araddr     (fb_araddr),
        .fb_axi_arburst    (fb_arburst),
        .fb_axi_arcache    (fb_arcache),
        .fb_axi_arid       (fb_arid),
        .fb_axi_arlen      (fb_arlen),
        .fb_axi_arlock     (fb_arlock),
        .fb_axi_arprot     (fb_arprot),
        .fb_axi_arqos      (fb_arqos),
        .fb_axi_arready    (fb_arready),
        .fb_axi_arsize     (fb_arsize),
        .fb_axi_arvalid    (fb_arvalid),
        .fb_axi_rdata      (fb_rdata),
        .fb_axi_rid        (fb_rid_unused),
        .fb_axi_rlast      (fb_rlast),
        .fb_axi_rready     (fb_rready),
        .fb_axi_rresp      (fb_rresp_unused),
        .fb_axi_rvalid     (fb_rvalid),
        // write channels: never used (the PL only reads the screen)
        .fb_axi_awaddr     (32'd0),
        .fb_axi_awburst    (2'b01),
        .fb_axi_awcache    (4'b0011),
        .fb_axi_awid       (6'd0),
        .fb_axi_awlen      (4'd0),
        .fb_axi_awlock     (2'b00),
        .fb_axi_awprot     (3'b000),
        .fb_axi_awqos      (4'd0),
        .fb_axi_awready    (fb_awready_unused),
        .fb_axi_awsize     (3'd3),
        .fb_axi_awvalid    (1'b0),
        .fb_axi_wdata      (64'd0),
        .fb_axi_wid        (6'd0),
        .fb_axi_wlast      (1'b0),
        .fb_axi_wready     (fb_wready_unused),
        .fb_axi_wstrb      (8'd0),
        .fb_axi_wvalid     (1'b0),
        .fb_axi_bid        (fb_bid_unused),
        .fb_axi_bready     (1'b1),
        .fb_axi_bresp      (fb_bresp_unused),
        .fb_axi_bvalid     (fb_bvalid_unused),
        // ---- TPU ----
        .tpu_aclk          (clk_tpu),
        .tpu_ctl_araddr    (tc_araddr),   .tpu_ctl_arburst (),  .tpu_ctl_arcache (),  .tpu_ctl_arid (tc_arid),
        .tpu_ctl_arlen     (tc_arlen),    .tpu_ctl_arlock  (),  .tpu_ctl_arprot  (),  .tpu_ctl_arqos (),
        .tpu_ctl_arready   (tc_arready),  .tpu_ctl_arsize  (),  .tpu_ctl_arvalid (tc_arvalid),
        .tpu_ctl_awaddr    (tc_awaddr),   .tpu_ctl_awburst (),  .tpu_ctl_awcache (),  .tpu_ctl_awid (tc_awid),
        .tpu_ctl_awlen     (),            .tpu_ctl_awlock  (),  .tpu_ctl_awprot  (),  .tpu_ctl_awqos (),
        .tpu_ctl_awready   (tc_awready),  .tpu_ctl_awsize  (),  .tpu_ctl_awvalid (tc_awvalid),
        .tpu_ctl_bid       (tc_bid),      .tpu_ctl_bready  (tc_bready), .tpu_ctl_bresp (2'b00), .tpu_ctl_bvalid (tc_bvalid),
        .tpu_ctl_rdata     (tc_rdata),    .tpu_ctl_rid     (tc_rid),    .tpu_ctl_rlast (tc_rlast),
        .tpu_ctl_rready    (tc_rready),   .tpu_ctl_rresp   (2'b00),     .tpu_ctl_rvalid (tc_rvalid),
        .tpu_ctl_wdata     (tc_wdata),    .tpu_ctl_wid     (),  .tpu_ctl_wlast (tc_wlast), .tpu_ctl_wready (tc_wready),
        .tpu_ctl_wstrb     (),            .tpu_ctl_wvalid  (tc_wvalid),
        .tpu_m0_araddr (tm_araddr[31:0]),  .tpu_m0_arburst (2'b01), .tpu_m0_arcache (4'b0011), .tpu_m0_arid (6'd0),
        .tpu_m0_arlen  (tm_arlen[3:0]),    .tpu_m0_arlock (2'b00),  .tpu_m0_arprot (3'b000),   .tpu_m0_arqos (4'd0),
        .tpu_m0_arready(tm_arready[0]),    .tpu_m0_arsize (3'd3),   .tpu_m0_arvalid(tm_arvalid[0]),
        .tpu_m0_awaddr (tm_awaddr[31:0]),  .tpu_m0_awburst (2'b01), .tpu_m0_awcache (4'b0011), .tpu_m0_awid (6'd0),
        .tpu_m0_awlen  (tm_awlen[3:0]),    .tpu_m0_awlock (2'b00),  .tpu_m0_awprot (3'b000),   .tpu_m0_awqos (4'd0),
        .tpu_m0_awready(tm_awready[0]),    .tpu_m0_awsize (3'd3),   .tpu_m0_awvalid(tm_awvalid[0]),
        .tpu_m0_bid    (),                 .tpu_m0_bready (tm_bready[0]), .tpu_m0_bresp (), .tpu_m0_bvalid (tm_bvalid[0]),
        .tpu_m0_rdata  (tm_rdata[63:0]),   .tpu_m0_rid (), .tpu_m0_rlast (tm_rlast[0]), .tpu_m0_rready (tm_rready[0]),
        .tpu_m0_rresp  (),                 .tpu_m0_rvalid (tm_rvalid[0]),
        .tpu_m0_wdata  (tm_wdata[63:0]),   .tpu_m0_wid (6'd0), .tpu_m0_wlast (tm_wlast[0]), .tpu_m0_wready (tm_wready[0]),
        .tpu_m0_wstrb  (8'hFF),            .tpu_m0_wvalid (tm_wvalid[0]),
        .tpu_m1_araddr (tm_araddr[63:32]), .tpu_m1_arburst (2'b01), .tpu_m1_arcache (4'b0011), .tpu_m1_arid (6'd0),
        .tpu_m1_arlen  (tm_arlen[7:4]),    .tpu_m1_arlock (2'b00),  .tpu_m1_arprot (3'b000),   .tpu_m1_arqos (4'd0),
        .tpu_m1_arready(tm_arready[1]),    .tpu_m1_arsize (3'd3),   .tpu_m1_arvalid(tm_arvalid[1]),
        .tpu_m1_awaddr (tm_awaddr[63:32]), .tpu_m1_awburst (2'b01), .tpu_m1_awcache (4'b0011), .tpu_m1_awid (6'd0),
        .tpu_m1_awlen  (tm_awlen[7:4]),    .tpu_m1_awlock (2'b00),  .tpu_m1_awprot (3'b000),   .tpu_m1_awqos (4'd0),
        .tpu_m1_awready(tm_awready[1]),    .tpu_m1_awsize (3'd3),   .tpu_m1_awvalid(tm_awvalid[1]),
        .tpu_m1_bid    (),                 .tpu_m1_bready (tm_bready[1]), .tpu_m1_bresp (), .tpu_m1_bvalid (tm_bvalid[1]),
        .tpu_m1_rdata  (tm_rdata[127:64]), .tpu_m1_rid (), .tpu_m1_rlast (tm_rlast[1]), .tpu_m1_rready (tm_rready[1]),
        .tpu_m1_rresp  (),                 .tpu_m1_rvalid (tm_rvalid[1]),
        .tpu_m1_wdata  (tm_wdata[127:64]), .tpu_m1_wid (6'd0), .tpu_m1_wlast (tm_wlast[1]), .tpu_m1_wready (tm_wready[1]),
        .tpu_m1_wstrb  (8'hFF),            .tpu_m1_wvalid (tm_wvalid[1])
    );

    // EMIO outputs count only while Linux drives them (GPIO_T = 0)
    wire [15:0] emio_out = emio_o & ~emio_t;
    wire        bridge_on  = emio_out[7];
    wire        robot_open = emio_out[6];
    wire        fb_enable  = emio_out[8];      // Linux screen on HDMI (else the test pattern)

    assign esp_en_f   = emio_out[0];
    assign esp_boot_f = emio_out[1];
    assign fled1_f    = emio_out[2];
    assign fled2_f    = emio_out[3];

    // =====================================================================================
    // Clocks and resets
    // =====================================================================================
    // (clk50, clk_pix, clk_ser, mmcm_locked are declared with the PS wrapper above)

    clkgen u_clk (
        .clk50_in (pl_clk50),
        .clk50    (clk50),
        .clk_pix  (clk_pix),
        .clk_ser  (clk_ser),
        .clk_tpu  (clk_tpu),
        .clk_cam  (clk_cam_unused),
        .locked   (mmcm_locked)
    );

    // power-on reset for the 50 MHz system domain (FF init values come from the bitstream)
    reg [7:0] por_cnt = 8'd0;
    reg       rst50   = 1'b1;
    always @(posedge clk50) begin
        if (por_cnt != 8'hFF) begin
            por_cnt <= por_cnt + 8'd1;
            rst50   <= 1'b1;
        end else begin
            rst50   <= 1'b0;
        end
    end

    // pixel-domain reset: asserted until the MMCM is locked, released synchronously,
    // held 16 clk_pix cycles (OSERDESE2 RST needs >= 2 CLKDIV cycles, released on CLKDIV)
    (* ASYNC_REG = "TRUE" *) reg [1:0] lock_pix_s = 2'b00;
    reg [3:0] rst_pix_cnt = 4'd0;
    reg       rst_pix     = 1'b1;
    always @(posedge clk_pix or negedge mmcm_locked) begin
        if (!mmcm_locked) begin
            lock_pix_s  <= 2'b00;
            rst_pix_cnt <= 4'd0;
            rst_pix     <= 1'b1;
        end else begin
            lock_pix_s <= {lock_pix_s[0], 1'b1};
            if (!lock_pix_s[1])            begin rst_pix_cnt <= 4'd0; rst_pix <= 1'b1; end
            else if (rst_pix_cnt != 4'hF)  begin rst_pix_cnt <= rst_pix_cnt + 4'd1; rst_pix <= 1'b1; end
            else                           rst_pix <= 1'b0;
        end
    end

    // =====================================================================================
    // Status synchronizers (PS GPIO, buttons, HPD, MMCM lock -> 50 MHz)
    // =====================================================================================
    wire hpd_s, locked_s, bridge_s, robot_s, btn1_n_s, btn2_n_s;
    sync_bit            u_hpd_s    (.clk(clk50), .d(hdmi_hpd_f),  .q(hpd_s));
    sync_bit            u_lock_s   (.clk(clk50), .d(mmcm_locked), .q(locked_s));
    sync_bit            u_bridge_s (.clk(clk50), .d(bridge_on),   .q(bridge_s));
    sync_bit            u_robot_s  (.clk(clk50), .d(robot_open),  .q(robot_s));
    sync_bit #(.INIT(1'b1)) u_btn1_s  (.clk(clk50), .d(btn1_f),      .q(btn1_n_s));
    sync_bit #(.INIT(1'b1)) u_btn2_s  (.clk(clk50), .d(btn2_f),      .q(btn2_n_s));

    // PS GPIO inputs: buttons (1 = pressed) + read-back of the outputs
    //   [8] read-back of the screen switch · [9] HDMI hot plug (monitor seen) · [10] HDMI clock locked
    //   [11] screen fetch late (sticky while the screen is on)
    assign emio_i = {4'b0000, fb_late, locked_s, hpd_s, emio_out[8],
                     emio_out[7:6], ~btn2_n_s, ~btn1_n_s, emio_out[3:0]};

    // =====================================================================================
    // SPI slave + register file
    // =====================================================================================
    wire sck_g;
    clkbuf u_sckbuf (.i(f_sck), .o(sck_g));

    wire [7:0] rx_byte, tx_slot0, tx_slot1;
    wire       rx_first, ev_tgl;

    spi_slave u_spi (
        .sck      (sck_g),
        .cs_n     (f_cs),
        .mosi     (f_mosi),
        .miso     (f_miso),
        .rx_byte  (rx_byte),
        .rx_first (rx_first),
        .ev_tgl   (ev_tgl),
        .tx_slot0 (tx_slot0),
        .tx_slot1 (tx_slot1)
    );

    wire [7:0] esp_in, esp_out, esp_dir;
    wire [7:0] scratch, pattern, solid_r, solid_g, solid_b;
    wire       irq;

    spi_regs_v35 u_regs (
        .clk         (clk50),
        .rst         (rst50),
        .rx_byte     (rx_byte),
        .rx_first    (rx_first),
        .ev_tgl      (ev_tgl),
        .tx_slot0    (tx_slot0),
        .tx_slot1    (tx_slot1),
        .hpd         (hpd_s),
        .mmcm_locked (locked_s),
        .bridge_on   (bridge_s),
        .robot_open  (robot_s),
        .btn1        (~btn1_n_s),
        .btn2        (~btn2_n_s),
        .esp_in      (esp_in),
        .scratch     (scratch),
        .pattern     (pattern),
        .solid_r     (solid_r),
        .solid_g     (solid_g),
        .solid_b     (solid_b),
        .irq         (irq),
        .esp_out     (esp_out),
        .esp_dir     (esp_dir)
    );

    reg irq_q = 1'b0;
    always @(posedge clk50) irq_q <= irq;
    assign f_irq = irq_q;

    // =====================================================================================
    // ESP_IO (ESP_IO0/1 = robot UART bridge)
    // =====================================================================================
    esp_io_v35 u_esp (
        .clk       (clk50),
        .out       (esp_out),
        .dir       (esp_dir),
        .in        (esp_in),
        .bridge_on (bridge_on),
        .uart_tx   (uart_tx),
        .uart_rx   (uart_rx),
        .io0 (fe_io0), .io1 (fe_io1), .io2 (fe_io2), .io3 (fe_io3),
        .io4 (fe_io4), .io5 (fe_io5), .io6 (fe_io6), .io7 (fe_io7)
    );

    // =====================================================================================
    // Module PL LED (active low): what state the board is in, without any tool
    // =====================================================================================
    reg [25:0] blink = 26'd0;
    always @(posedge clk50) blink <= blink + 26'd1;
    // 50 MHz / 2^26 = 0.75 Hz slow blink · / 2^23 = 6 Hz fast blink
    wire led_on = robot_s  ? 1'b1 :
                  bridge_s ? blink[22] :
                             blink[25];
    assign pl_led_n = ~led_on;

    // =====================================================================================
    // TPU: int8 x int16 matrix engine for the double-Rachel AI (tpu_top.v, 2 engines x 32 MAC)
    // =====================================================================================
    (* ASYNC_REG = "TRUE" *) reg [1:0] lock_tpu_s = 2'b00;
    reg [3:0] rst_tpu_cnt = 4'd0;
    reg       rst_tpu = 1'b1;
    always @(posedge clk_tpu or negedge mmcm_locked) begin
        if (!mmcm_locked) begin
            lock_tpu_s <= 2'b00; rst_tpu_cnt <= 4'd0; rst_tpu <= 1'b1;
        end else begin
            lock_tpu_s <= {lock_tpu_s[0], 1'b1};
            if (!lock_tpu_s[1])            begin rst_tpu_cnt <= 4'd0; rst_tpu <= 1'b1; end
            else if (rst_tpu_cnt != 4'hF)  begin rst_tpu_cnt <= rst_tpu_cnt + 4'd1; rst_tpu <= 1'b1; end
            else                           rst_tpu <= 1'b0;
        end
    end

    tpu_top u_tpu (
        .clk (clk_tpu), .rst (rst_tpu),
        .s_awaddr (tc_awaddr), .s_awid (tc_awid), .s_awvalid (tc_awvalid), .s_awready (tc_awready),
        .s_wdata (tc_wdata), .s_wlast (tc_wlast), .s_wvalid (tc_wvalid), .s_wready (tc_wready),
        .s_bid (tc_bid), .s_bvalid (tc_bvalid), .s_bready (tc_bready),
        .s_araddr (tc_araddr), .s_arid (tc_arid), .s_arlen (tc_arlen), .s_arvalid (tc_arvalid), .s_arready (tc_arready),
        .s_rdata (tc_rdata), .s_rid (tc_rid), .s_rlast (tc_rlast), .s_rvalid (tc_rvalid), .s_rready (tc_rready),
        .m_araddr (tm_araddr), .m_arlen (tm_arlen), .m_arvalid (tm_arvalid), .m_arready (tm_arready),
        .m_rdata (tm_rdata), .m_rvalid (tm_rvalid), .m_rlast (tm_rlast), .m_rready (tm_rready),
        .m_awaddr (tm_awaddr), .m_awlen (tm_awlen), .m_awvalid (tm_awvalid), .m_awready (tm_awready),
        .m_wdata (tm_wdata), .m_wlast (tm_wlast), .m_wvalid (tm_wvalid), .m_wready (tm_wready),
        .m_bvalid (tm_bvalid), .m_bready (tm_bready)
    );

    // =====================================================================================
    // HDMI 720p60 test pattern (unchanged from the v2.3 top)
    // =====================================================================================
    wire [31:0] ctl_50 = {solid_b, solid_g, solid_r, pattern};
    wire [31:0] ctl_s;
    genvar gi;
    generate
        for (gi = 0; gi < 32; gi = gi + 1) begin : g_ctl
            sync_bit u_s (.clk(clk_pix), .d(ctl_50[gi]), .q(ctl_s[gi]));
        end
    endgenerate

    wire [11:0] hc;
    wire [10:0] vc;
    wire [8:0]  gx;
    wire        de0, hs0, vs0;

    hdmi_timing u_tim (
        .clk (clk_pix), .rst (rst_pix),
        .hc (hc), .vc (vc), .gx (gx), .de (de0), .hs (hs0), .vs (vs0)
    );

    reg [31:0] ctl_pix = 32'd0;
    always @(posedge clk_pix) begin
        if (rst_pix || (vc == 11'd720 && hc == 12'd0)) ctl_pix <= ctl_s;   // frame boundary
    end

    wire [7:0] pat_r, pat_g, pat_b;
    wire       de1, hs1, vs1;

    pattern_gen u_pat (
        .clk     (clk_pix),
        .hc      (hc), .vc (vc), .gx (gx),
        .de_i    (de0), .hs_i (hs0), .vs_i (vs0),
        .pattern (ctl_pix[7:0]),
        .solid_r (ctl_pix[15:8]),
        .solid_g (ctl_pix[23:16]),
        .solid_b (ctl_pix[31:24]),
        .r (pat_r), .g (pat_g), .b (pat_b),
        .de_o (de1), .hs_o (hs1), .vs_o (vs1)
    );

    // ---- Linux screen: framebuffer in DDR (reserved 4 MB at 0x1FC00000) -> HDMI ----------------
    wire fb_en_pix;
    sync_bit u_fb_en_s (.clk(clk_pix), .d(fb_enable), .q(fb_en_pix));

    reg fb_on = 1'b0;                            // display source, switched at a frame boundary
    always @(posedge clk_pix) begin
        if (rst_pix)                              fb_on <= 1'b0;
        else if (vc == 11'd720 && hc == 12'd0)    fb_on <= fb_en_pix;
    end

    wire [7:0] fb_r, fb_g, fb_b;

    fb_scanout #(.BASE(32'h1FC0_0000)) u_fb (
        .clk (clk_pix), .rst (rst_pix), .enable (fb_en_pix),
        .hc (hc), .vc (vc),
        .r (fb_r), .g (fb_g), .b (fb_b), .late (fb_late),
        .araddr (fb_araddr), .arlen (fb_arlen), .arsize (fb_arsize), .arburst (fb_arburst),
        .arlock (fb_arlock), .arcache (fb_arcache), .arprot (fb_arprot), .arqos (fb_arqos),
        .arid (fb_arid), .arvalid (fb_arvalid), .arready (fb_arready),
        .rdata (fb_rdata), .rvalid (fb_rvalid), .rlast (fb_rlast), .rready (fb_rready)
    );

    wire [7:0] pr = fb_on ? fb_r : pat_r;
    wire [7:0] pg = fb_on ? fb_g : pat_g;
    wire [7:0] pb = fb_on ? fb_b : pat_b;

    wire [9:0] tmds0, tmds1, tmds2;
    tmds_encoder u_enc0 (.clk(clk_pix), .rst(rst_pix), .d(pb), .de(de1), .c0(hs1),  .c1(vs1),  .q(tmds0));
    tmds_encoder u_enc1 (.clk(clk_pix), .rst(rst_pix), .d(pg), .de(de1), .c0(1'b0), .c1(1'b0), .q(tmds1));
    tmds_encoder u_enc2 (.clk(clk_pix), .rst(rst_pix), .d(pr), .de(de1), .c0(1'b0), .c1(1'b0), .q(tmds2));

    tmds_serializer u_ser0 (.clk_pix(clk_pix), .clk_ser(clk_ser), .rst(rst_pix), .d(tmds0),           .out_p(hdmi_d0_p), .out_n(hdmi_d0_n));
    tmds_serializer u_ser1 (.clk_pix(clk_pix), .clk_ser(clk_ser), .rst(rst_pix), .d(tmds1),           .out_p(hdmi_d1_p), .out_n(hdmi_d1_n));
    tmds_serializer u_ser2 (.clk_pix(clk_pix), .clk_ser(clk_ser), .rst(rst_pix), .d(tmds2),           .out_p(hdmi_d2_p), .out_n(hdmi_d2_n));
    tmds_serializer u_serc (.clk_pix(clk_pix), .clk_ser(clk_ser), .rst(rst_pix), .d(10'b0000011111), .out_p(hdmi_ck_p), .out_n(hdmi_ck_n));

    // HDMI DDC: not used (high-Z, board level shifters + pull-ups)
    assign hdmi_scl_f = 1'bz;
    assign hdmi_sda_f = 1'bz;

    // =====================================================================================
    // Unused v3.5 pins: high-Z (WS2812 data idles low)
    // =====================================================================================
    assign stat_led_f = 1'b0;

    assign r_sck = 1'bz;  assign r_mosi = 1'bz;  assign r_miso = 1'bz;  assign r_cs = 1'bz;  assign r_irq = 1'bz;

    assign epd_din = 1'bz;  assign epd_clk = 1'bz;  assign epd_cs = 1'bz;
    assign epd_dc  = 1'bz;  assign epd_rst = 1'bz;  assign epd_busy = 1'bz;

    assign exp0 = 1'bz;  assign exp1 = 1'bz;  assign exp2 = 1'bz;  assign exp3 = 1'bz;  assign exp4 = 1'bz;
    assign exp5 = 1'bz;  assign exp6 = 1'bz;  assign exp7 = 1'bz;  assign exp8 = 1'bz;  assign exp9 = 1'bz;

    assign x_w14 = 1'bz;  assign x_v15 = 1'bz;  assign x_v12 = 1'bz;  assign x_y14 = 1'bz;  assign x_u12 = 1'bz;
    assign x_u13 = 1'bz;  assign x_t10 = 1'bz;  assign x_w13 = 1'bz;  assign x_t12 = 1'bz;  assign x_t11 = 1'bz;
    assign x_l14 = 1'bz;  assign x_l15 = 1'bz;  assign x_m14 = 1'bz;  assign x_m15 = 1'bz;  assign x_n15 = 1'bz;
    assign x_n16 = 1'bz;  assign x_v16 = 1'bz;  assign x_w16 = 1'bz;  assign x_v17 = 1'bz;  assign x_v18 = 1'bz;
    assign x_r16 = 1'bz;  assign x_y19 = 1'bz;  assign x_l16 = 1'bz;  assign x_r17 = 1'bz;  assign x_k16 = 1'bz;
    assign x_l17 = 1'bz;  assign x_u20 = 1'bz;  assign x_j16 = 1'bz;  assign x_t17 = 1'bz;  assign x_t20 = 1'bz;
    assign x_m20 = 1'bz;  assign x_r18 = 1'bz;  assign x_l20 = 1'bz;  assign x_m19 = 1'bz;  assign x_p15 = 1'bz;
    assign x_l19 = 1'bz;  assign x_n17 = 1'bz;  assign x_p16 = 1'bz;  assign x_p18 = 1'bz;  assign x_k17 = 1'bz;
    assign x_g18 = 1'bz;
endmodule

// global clock buffer for clocks entering on clock-capable pins (F_SCK)
module clkbuf (
    input  wire i,
    output wire o
);
`ifdef SIM
    assign o = i;
`else
    BUFG u_bufg (.I(i), .O(o));
`endif
endmodule

`default_nettype wire
