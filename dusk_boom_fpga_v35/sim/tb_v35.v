// tb_v35.v — test bench for dusk_boom_v35_top (Icarus Verilog, -DSIM, with system_wrapper_sim.v)
//
//  (a) SPI registers as the ESP firmware sees them: ID 0xB0, VERSION 0x35, FEATURES, STATUS bits from
//      the PS (EMIO), buttons, SCRATCH, burst reads, ESP_IO_DIR read-only bits
//  (b) robot UART bridge, 1 Mbps 8N1: PS UART0 TX -> ESP_IO0 (ESP RX) and ESP_IO1 (ESP TX) -> PS UART0 RX,
//      only while Linux sets bridge_on (EMIO7); high-Z / idle otherwise
//  (c) EMIO GPIO -> esp_en_f / esp_boot_f / front LEDs, read-back on emio_i, module LED states
`timescale 1ns / 1ps
`default_nettype none

module tb_v35;
    // ------------------------------------------------------------------------------------
    // DUT
    // ------------------------------------------------------------------------------------
    reg pl_clk50 = 1'b0;
    always #10 pl_clk50 = ~pl_clk50;

    reg  m_sck = 1'b0, m_mosi = 1'b0, m_cs = 1'b1, f_aux = 1'b0;
    wire f_miso, f_irq;
    wire m_miso;
    assign #(13.0) m_miso = f_miso;          // FPGA clock-to-pad + 470 R + trace (as in tb_top.v)

    // ESP32 side of ESP_IO0..7
    wire [7:0] fe;
    reg  [7:0] esp_drv_en  = 8'h00;
    reg  [7:0] esp_drv_val = 8'h00;
    genvar gi;
    generate
        for (gi = 0; gi < 8; gi = gi + 1) begin : g_esp
            assign fe[gi] = esp_drv_en[gi] ? esp_drv_val[gi] : 1'bz;
        end
    endgenerate
    pullup (fe[1]);                          // PULLUP TRUE on fe_io1 (timing_v35.xdc)

    reg  hdmi_hpd_f = 1'b0, btn1_f = 1'b1, btn2_f = 1'b1;
    wire esp_en_f, esp_boot_f, fled1_f, fled2_f, pl_led_n, stat_led_f;
    wire hdmi_d0_p, hdmi_d0_n, hdmi_d1_p, hdmi_d1_n, hdmi_d2_p, hdmi_d2_n, hdmi_ck_p, hdmi_ck_n;

    dusk_boom_v35_top dut (
        .pl_clk50 (pl_clk50), .pl_led_n (pl_led_n),
        .f_sck (m_sck), .f_mosi (m_mosi), .f_miso (f_miso), .f_cs (m_cs), .f_irq (f_irq), .f_aux (f_aux),
        .fe_io0 (fe[0]), .fe_io1 (fe[1]), .fe_io2 (fe[2]), .fe_io3 (fe[3]),
        .fe_io4 (fe[4]), .fe_io5 (fe[5]), .fe_io6 (fe[6]), .fe_io7 (fe[7]),
        .esp_en_f (esp_en_f), .esp_boot_f (esp_boot_f),
        .hdmi_d0_p (hdmi_d0_p), .hdmi_d0_n (hdmi_d0_n), .hdmi_d1_p (hdmi_d1_p), .hdmi_d1_n (hdmi_d1_n),
        .hdmi_d2_p (hdmi_d2_p), .hdmi_d2_n (hdmi_d2_n), .hdmi_ck_p (hdmi_ck_p), .hdmi_ck_n (hdmi_ck_n),
        .hdmi_hpd_f (hdmi_hpd_f),
        .stat_led_f (stat_led_f), .btn1_f (btn1_f), .btn2_f (btn2_f), .fled1_f (fled1_f), .fled2_f (fled2_f)
    );

    // ------------------------------------------------------------------------------------
    // checks
    // ------------------------------------------------------------------------------------
    integer n_pass = 0, n_fail = 0;
    task check;
        input [511:0] what;
        input [31:0]  got;
        input [31:0]  exp;
        begin
            if (got === exp) begin n_pass = n_pass + 1; $display("PASS %0s = 0x%0h", what, got); end
            else             begin n_fail = n_fail + 1; $display("FAIL %0s = 0x%0h (expected 0x%0h)", what, got, exp); end
        end
    endtask

    // ------------------------------------------------------------------------------------
    // SPI master (mode 0, 8 MHz) exactly like the firmware
    // ------------------------------------------------------------------------------------
    real half = 62.5;
    reg [7:0] txb [0:63];
    reg [7:0] rxb [0:63];
    reg [7:0] rdat [0:63];
    task spi_xfer;
        input integer n;
        integer i, k;
        reg [7:0] r;
        begin
            m_cs = 1'b0;
            for (i = 0; i < n; i = i + 1) begin
                r = 8'h00;
                for (k = 7; k >= 0; k = k - 1) begin
                    m_mosi = txb[i][k];
                    #(half) m_sck = 1'b1;
                    r[k] = m_miso;
                    #(half) m_sck = 1'b0;
                end
                rxb[i] = r;
            end
            #(half) m_cs = 1'b1;
            m_mosi = 1'b0;
            #(200);
        end
    endtask
    task spi_read;
        input [6:0] addr;
        input integer n;
        integer i;
        begin
            txb[0] = {1'b1, addr};
            for (i = 1; i < n + 2; i = i + 1) txb[i] = 8'h00;
            spi_xfer(n + 2);
            for (i = 0; i < n; i = i + 1) rdat[i] = rxb[i + 2];
        end
    endtask
    task spi_wr1;
        input [6:0] addr;
        input [7:0] v;
        begin txb[0] = {1'b0, addr}; txb[1] = v; spi_xfer(2); end
    endtask
    reg [7:0] rd1_v;
    task spi_rd1;
        input [6:0] addr;
        begin spi_read(addr, 1); rd1_v = rdat[0]; end
    endtask

    // ------------------------------------------------------------------------------------
    // UART 1 Mbps 8N1 helpers
    // ------------------------------------------------------------------------------------
    localparam real BIT = 1000.0;
    task ps_uart_send;                       // PS UART0 TX (EMIO) -> PL
        input [7:0] b;
        integer k;
        begin
            dut.u_ps.ps_uart_tx = 1'b0; #(BIT);
            for (k = 0; k < 8; k = k + 1) begin dut.u_ps.ps_uart_tx = b[k]; #(BIT); end
            dut.u_ps.ps_uart_tx = 1'b1; #(BIT);
        end
    endtask
    task esp_uart_send;                      // ESP32 IO5 -> ESP_IO1
        input [7:0] b;
        integer k;
        begin
            esp_drv_en[1] = 1'b1;
            esp_drv_val[1] = 1'b0; #(BIT);
            for (k = 0; k < 8; k = k + 1) begin esp_drv_val[1] = b[k]; #(BIT); end
            esp_drv_val[1] = 1'b1; #(BIT);
        end
    endtask

    // receivers: sample in the middle of each bit
    reg [7:0] esp_rx_buf [0:15];  integer esp_rx_n = 0;   // what the ESP receives on ESP_IO0
    reg [7:0] ps_rx_buf  [0:15];  integer ps_rx_n  = 0;   // what the PS UART0 RX receives
    always @(negedge fe[0]) if (fe[0] === 1'b0) begin : rx_esp
        integer k; reg [7:0] b;
        #(BIT * 1.5);
        for (k = 0; k < 8; k = k + 1) begin b[k] = fe[0]; #(BIT); end
        if (fe[0] === 1'b1 && esp_rx_n < 16) begin esp_rx_buf[esp_rx_n] = b; esp_rx_n = esp_rx_n + 1; end
    end
    wire ps_rxd = dut.u_ps.esp_uart_rxd;
    always @(negedge ps_rxd) if (ps_rxd === 1'b0) begin : rx_ps
        integer k; reg [7:0] b;
        #(BIT * 1.5);
        for (k = 0; k < 8; k = k + 1) begin b[k] = ps_rxd; #(BIT); end
        if (ps_rxd === 1'b1 && ps_rx_n < 16) begin ps_rx_buf[ps_rx_n] = b; ps_rx_n = ps_rx_n + 1; end
    end

    // PS GPIO helpers (Linux: export, direction out, value)
    task ps_gpio_out;
        input integer n;
        input v;
        begin dut.u_ps.ps_gpio_t[n] = 1'b0; dut.u_ps.ps_gpio_o[n] = v; end
    endtask
    task ps_gpio_in;
        input integer n;
        begin dut.u_ps.ps_gpio_t[n] = 1'b1; end
    endtask

    // ------------------------------------------------------------------------------------
    // test sequence
    // ------------------------------------------------------------------------------------
    integer i;
    reg [7:0] ping [0:5];
    reg [7:0] reply [0:5];
    initial begin
        ping[0] = 8'hFF; ping[1] = 8'hFF; ping[2] = 8'h01; ping[3] = 8'h02; ping[4] = 8'h01; ping[5] = 8'hFB;
        reply[0] = 8'hFF; reply[1] = 8'hFF; reply[2] = 8'h01; reply[3] = 8'h02; reply[4] = 8'h00; reply[5] = 8'hFC;

        #6000;      // POR (256 clk50 cycles = 5.1 us) + MMCM lock (SIM model: 2 us)

        // ---- (a) identification ----
        spi_rd1(7'h00); check("ID", rd1_v, 8'hB0);
        spi_rd1(7'h01); check("VERSION", rd1_v, 8'h35);
        spi_rd1(7'h09); check("FEATURES", rd1_v, 8'h01);
        spi_rd1(7'h08); check("STATUS idle (MMCM locked only)", rd1_v, 8'h04);
        spi_rd1(7'h0C); check("old CAM_FRAMES reads 0", rd1_v, 8'h00);
        spi_wr1(7'h02, 8'h5A); spi_rd1(7'h02); check("SCRATCH", rd1_v, 8'h5A);
        spi_read(7'h00, 4);
        check("burst ID", rdat[0], 8'hB0); check("burst VER", rdat[1], 8'h35);
        check("burst SCRATCH", rdat[2], 8'h5A);

        // ---- defaults after power-on: nothing driven towards the ESP ----
        check("esp_en_f default", esp_en_f, 1'b0);
        check("esp_boot_f default", esp_boot_f, 1'b0);
        check("ESP_IO0 high-Z without bridge", fe[0], 1'bz);
        check("PS UART RX idle without bridge", ps_rxd, 1'b1);
        esp_drv_en[1] = 1'b1; esp_drv_val[1] = 1'b0; #500;
        check("PS UART RX still idle while ESP drives 0 (bridge off)", ps_rxd, 1'b1);
        esp_drv_en[1] = 1'b0; #500;
        check("ESP_IO1 pull-up", fe[1], 1'b1);

        // ---- ESP_IO_DIR bits 1:0 are not writable ----
        spi_wr1(7'h21, 8'hA5);
        spi_wr1(7'h22, 8'hFF);
        spi_rd1(7'h22); check("ESP_IO_DIR (bridge off)", rd1_v, 8'hFC);
        #200;
        check("ESP_IO2..7 driven = ESP_IO_OUT", {fe[7:2]}, 6'b101001);
        check("ESP_IO0 still high-Z", fe[0], 1'bz);
        check("ESP_IO1 not driven by FPGA", fe[1], 1'b1);
        spi_rd1(7'h23); check("ESP_IO_IN[7:1] (ESP_IO0 floats)", rd1_v[7:1], 7'b101001_1);
        spi_wr1(7'h22, 8'h00);

        // ---- (c) EMIO: ESP reset / boot / LEDs / buttons ----
        ps_gpio_out(0, 1'b1); #50; check("esp_en_f from EMIO0", esp_en_f, 1'b1);
        ps_gpio_out(0, 1'b0); #50; check("esp_en_f released", esp_en_f, 1'b0);
        ps_gpio_out(1, 1'b1); #50; check("esp_boot_f from EMIO1", esp_boot_f, 1'b1);
        ps_gpio_in(1);        #50; check("esp_boot_f back to 0 when input", esp_boot_f, 1'b0);
        ps_gpio_out(2, 1'b1); ps_gpio_out(3, 1'b1); #50;
        check("front LEDs", {fled2_f, fled1_f}, 2'b11);
        check("emio_i read-back of outputs", dut.u_ps.emio_i[3:0], 4'b1100);
        ps_gpio_out(2, 1'b0); ps_gpio_out(3, 1'b0);
        btn1_f = 1'b0; #200;
        check("emio_i[4] button 1 pressed", dut.u_ps.emio_i[4], 1'b1);
        spi_rd1(7'h08); check("STATUS bit5 button 1", rd1_v, 8'h24);
        btn1_f = 1'b1; btn2_f = 1'b0; #200;
        spi_rd1(7'h08); check("STATUS bit6 button 2", rd1_v, 8'h44);
        btn2_f = 1'b1; #200;

        // ---- module LED ----
        check("LED slow blink = blink[25] (bridge off)", dut.led_on, dut.blink[25]);

        // ---- (b) robot UART bridge ----
        ps_gpio_out(7, 1'b1); #200;
        spi_rd1(7'h08); check("STATUS bit3 bridge running", rd1_v, 8'h0C);
        spi_rd1(7'h22); check("ESP_IO_DIR bit0 = FPGA drives ESP_IO0", rd1_v, 8'h01);
        check("ESP_IO0 idle high (PS UART idle)", fe[0], 1'b1);
        check("LED fast blink = blink[22] (bridge on)", dut.led_on, dut.blink[22]);

        for (i = 0; i < 6; i = i + 1) ps_uart_send(ping[i]);
        #(BIT * 2);
        check("ESP received bytes", esp_rx_n, 6);
        for (i = 0; i < 6; i = i + 1) check("ESP rx byte", esp_rx_buf[i], ping[i]);

        for (i = 0; i < 6; i = i + 1) esp_uart_send(reply[i]);
        esp_drv_en[1] = 1'b0;
        #(BIT * 2);
        check("PS received bytes", ps_rx_n, 6);
        for (i = 0; i < 6; i = i + 1) check("PS rx byte", ps_rx_buf[i], reply[i]);

        ps_gpio_out(6, 1'b1); #200;
        spi_rd1(7'h08); check("STATUS bit4 robot USB open", rd1_v, 8'h1C);
        check("LED steady on (robot open)", pl_led_n, 1'b0);

        // bridge off again: ESP_IO0 back to high-Z
        ps_gpio_in(7); ps_gpio_in(6); #200;
        check("ESP_IO0 high-Z after bridge off", fe[0], 1'bz);
        spi_rd1(7'h08); check("STATUS after bridge off", rd1_v, 8'h04);

        // ---- (d) screen: EMIO 8..11, framebuffer reads on S_AXI_HP0 ----
        check("emio_i[10] HDMI clock locked", dut.u_ps.emio_i[10], 1'b1);
        check("emio_i[9] HPD low (no monitor)", dut.u_ps.emio_i[9], 1'b0);
        hdmi_hpd_f = 1'b1; #200;
        check("emio_i[9] HPD high (monitor)", dut.u_ps.emio_i[9], 1'b1);
        check("no screen reads while EMIO8 = 0", dut.u_ps.ar_count, 0);
        ps_gpio_out(8, 1'b1); #50;
        check("emio_i[8] screen switch read-back", dut.u_ps.emio_i[8], 1'b1);
        #(2 * 1650 * 13.468);                                    // two line times
        check("screen reads started (40 bursts per line)", dut.u_ps.ar_count >= 40, 1'b1);
        check("screen read address inside the 4 MB screen",
              (dut.u_fb.araddr >= 32'h1FC0_0000) && (dut.u_fb.araddr < 32'h1FC0_0000 + 720 * 5120), 1'b1);
        check("no writes on HP0", dut.u_ps.write_attempts, 0);
        check("emio_i[11] fetch never late", dut.u_ps.emio_i[11], 1'b0);
        ps_gpio_out(8, 1'b0); #(2 * 1650 * 13.468);
        i = dut.u_ps.ar_count; #(2 * 1650 * 13.468);
        check("screen reads stop after EMIO8 = 0", dut.u_ps.ar_count, i);

        $display("==== %0d PASS, %0d FAIL ====", n_pass, n_fail);
        $finish;
    end

    initial begin #2_000_000; $display("FAIL timeout"); $finish; end
endmodule

`default_nettype wire
