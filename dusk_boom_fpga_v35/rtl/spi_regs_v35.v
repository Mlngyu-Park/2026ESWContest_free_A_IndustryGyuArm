// spi_regs_v35.v — SPI transaction engine + register file, DUSK_BOOM robot board v3.5
//
// Same SPI protocol as ../../fpga/rtl/spi_regs.v (../../SPI_PROTOCOL.md v1):
//   byte0 = {R/W#, addr[6:0]}  (bit7 = 1 read, 0 write)
//   write : byte1.. = data, address auto-increments
//   read  : byte1 = dummy (ignored by the CPU), byte2.. = data, address auto-increments
//
// Changes against the v2.3 register file (2026-11-05):
//   * VERSION = 0x35 (board v3.5)
//   * camera registers (0x0C..0x13) removed — v3.5 has no camera, they read 0x00
//   * STATUS gets the robot-bridge bits from the Zynq PS (Linux, EMIO GPIO)
//   * new 0x09 FEATURES: what this bitstream offers
//   * ESP_IO0 / ESP_IO1 belong to the robot UART bridge (ESP_IO0 = FPGA -> ESP, ESP_IO1 = ESP -> FPGA);
//     ESP_IO_DIR bits 1:0 cannot be written, bit0 reads 1 while the FPGA drives ESP_IO0
//
// Register map (8-bit):
//   0x00 ID           R   0xB0
//   0x01 VERSION      R   0x35
//   0x02 SCRATCH      RW
//   0x03 HDMI_PATTERN RW  0 colour bars · 1 horizontal gradient · 2 checker · 3 solid (others = bars)
//   0x04 SOLID_R      RW
//   0x05 SOLID_G      RW
//   0x06 SOLID_B      RW
//   0x08 STATUS       R   bit0 HDMI HPD · bit2 HDMI MMCM locked
//                         bit3 robot bridge running (Linux dusk_robotd, FPGA drives ESP_IO0)
//                         bit4 robot USB serial adapter open on the module USB-A port
//                         bit5 front button 1 pressed · bit6 front button 2 pressed
//   0x09 FEATURES     R   bit0 robot UART bridge: ESP_IO1 (ESP IO5, TX) -> Zynq UART0 -> USB-A,
//                              Zynq UART0 -> ESP_IO0 (ESP IO4, RX), 1 Mbps 8N1
//   0x20 IRQ_CTRL     W   bit0=1 -> F_IRQ high · bit1=1 -> F_IRQ low (bit1 wins if both)
//                         (reads back {7'b0, F_IRQ})
//   0x21 ESP_IO_OUT   RW
//   0x22 ESP_IO_DIR   RW  1 = FPGA drives the pin, 0 = input (reset value 0x00) — bits 1:0 read-only
//   0x23 ESP_IO_IN    R   current (synchronized) pin levels
//   all other addresses read 0x00, writes are ignored.
//
// Read prefetch: see spi_regs.v — data for byte j+2 is fetched when byte j has been received.
// Every register read here is side-effect free, so prefetching past the end is harmless.
`timescale 1ns / 1ps
`default_nettype none

module spi_regs_v35 (
    input  wire        clk,
    input  wire        rst,
    // from spi_slave (SCK domain, qualified by ev_tgl)
    input  wire [7:0]  rx_byte,
    input  wire        rx_first,
    input  wire        ev_tgl,
    // to spi_slave
    output reg  [7:0]  tx_slot0,
    output reg  [7:0]  tx_slot1,
    // status inputs (already in this clock domain)
    input  wire        hpd,
    input  wire        mmcm_locked,
    input  wire        bridge_on,
    input  wire        robot_open,
    input  wire        btn1,          // 1 = pressed
    input  wire        btn2,          // 1 = pressed
    input  wire [7:0]  esp_in,
    // control outputs
    output reg  [7:0]  scratch,
    output reg  [7:0]  pattern,
    output reg  [7:0]  solid_r,
    output reg  [7:0]  solid_g,
    output reg  [7:0]  solid_b,
    output reg         irq,
    output reg  [7:0]  esp_out,
    output wire [7:0]  esp_dir        // bits 1:0 always 0 (owned by the robot bridge)
);
    localparam [7:0] ID_VAL   = 8'hB0;
    localparam [7:0] VER_VAL  = 8'h35;
    localparam [7:0] FEAT_VAL = 8'h01;

    // ---- byte event from the SCK domain ------------------------------------------------
    wire ev;
    sync_toggle u_evsync (.clk(clk), .tgl(ev_tgl), .pulse(ev));

    reg        rd;          // current transaction is a read
    reg  [6:0] addr;        // next address (write target / next prefetch address)
    reg        pslot;       // slot to fill with the next prefetch
    reg  [7:2] dir_q;       // writable ESP_IO_DIR bits

    assign esp_dir = {dir_q, 2'b00};

    // ---- read multiplexer ------------------------------------------------------------------
    wire [6:0] raddr = rx_first ? rx_byte[6:0] : addr;
    reg  [7:0] rdata;
    always @* begin
        case (raddr)
            7'h00: rdata = ID_VAL;
            7'h01: rdata = VER_VAL;
            7'h02: rdata = scratch;
            7'h03: rdata = pattern;
            7'h04: rdata = solid_r;
            7'h05: rdata = solid_g;
            7'h06: rdata = solid_b;
            7'h08: rdata = {1'b0, btn2, btn1, robot_open, bridge_on, mmcm_locked, 1'b0, hpd};
            7'h09: rdata = FEAT_VAL;
            7'h20: rdata = {7'b0, irq};
            7'h21: rdata = esp_out;
            7'h22: rdata = {dir_q, 1'b0, bridge_on};
            7'h23: rdata = esp_in;
            default: rdata = 8'h00;
        endcase
    end

    // ---- transaction engine ------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            rd        <= 1'b0;
            addr      <= 7'd0;
            pslot     <= 1'b0;
            tx_slot0  <= 8'h00;
            tx_slot1  <= 8'h00;
            scratch   <= 8'h00;
            pattern   <= 8'h00;
            solid_r   <= 8'h00;
            solid_g   <= 8'h00;
            solid_b   <= 8'h00;
            irq       <= 1'b0;
            esp_out   <= 8'h00;
            dir_q     <= 6'h00;
        end else if (ev) begin
            if (rx_first) begin
                // command byte
                rd <= rx_byte[7];
                if (rx_byte[7]) begin
                    tx_slot0 <= rdata;              // data for byte 2 (addr A)
                    addr     <= rx_byte[6:0] + 7'd1;
                    pslot    <= 1'b1;
                end else begin
                    addr     <= rx_byte[6:0];
                end
            end else if (rd) begin
                // byte j (j >= 1) of a read finished: prefetch addr A+j for byte j+2
                if (pslot) tx_slot1 <= rdata;
                else       tx_slot0 <= rdata;
                pslot <= ~pslot;
                addr  <= addr + 7'd1;
            end else begin
                // write data byte
                case (addr)
                    7'h02: scratch <= rx_byte;
                    7'h03: pattern <= rx_byte;
                    7'h04: solid_r <= rx_byte;
                    7'h05: solid_g <= rx_byte;
                    7'h06: solid_b <= rx_byte;
                    7'h20: begin
                        if (rx_byte[1])      irq <= 1'b0;
                        else if (rx_byte[0]) irq <= 1'b1;
                    end
                    7'h21: esp_out <= rx_byte;
                    7'h22: dir_q   <= rx_byte[7:2];
                    default: ;
                endcase
                addr <= addr + 7'd1;
            end
        end
    end
endmodule

`default_nettype wire
