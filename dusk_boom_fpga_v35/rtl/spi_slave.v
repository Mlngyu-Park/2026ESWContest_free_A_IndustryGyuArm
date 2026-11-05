// spi_slave.v — SCK-domain SPI front end (mode 0, MSB first)
//
// DUSK_BOOM robot board v2.3 · CPU (ESP32-S3 / XIAO) is SPI master, see ../../SPI_PROTOCOL.md
//
// Design (SCK-domain shifting + safe CDC, works up to 25 MHz SCK):
//  * MOSI is sampled on SCK rising edges by flops clocked by SCK itself (F_SCK is on the
//    clock-capable pin N18 = IO_L13P_T2_MRCC_34, routed through a BUFG).
//  * MISO is launched on SCK falling edges (mode 0: master samples on the rising edge),
//    giving a full half period for clock-to-out + board delay.
//  * F_CS high asynchronously resets the bit/byte state, so every transaction starts clean
//    even if a previous one was aborted mid-byte.
//  * Every completed byte is handed to the system clock domain as {rx_byte, rx_first} plus a
//    toggle (ev_tgl).  rx_byte/rx_first are held stable for one whole byte time (8 SCK
//    periods) after the toggle, far longer than the 2..3-cycle synchronizer latency of the
//    50 MHz system domain -> classic "data + toggle-qualifier" CDC, no multi-bit hazard.
//  * Transmit data comes from two system-domain slots (tx_slot0/tx_slot1, double buffer):
//    byte j of a transaction is loaded from slot[j % 2].  The system domain only writes
//    slot[j % 2] for byte j+2 after it has seen byte j complete, i.e. always the slot that
//    is NOT about to be read — so the SCK domain never samples a changing slot.
`timescale 1ns / 1ps
`default_nettype none

module spi_slave (
    input  wire       sck,        // SPI clock (BUFG), idle low
    input  wire       cs_n,       // chip select, active low (async reset of state)
    input  wire       mosi,
    output wire       miso,
    // to the system domain
    output reg  [7:0] rx_byte,    // last completed byte
    output reg        rx_first,   // 1 = it was the first byte (command) of the transaction
    output reg        ev_tgl,     // toggles once per completed byte
    // from the system domain (double-buffered transmit data)
    input  wire [7:0] tx_slot0,
    input  wire [7:0] tx_slot1
);
    reg [2:0] bitcnt;   // bit index within the current byte
    reg [6:0] sr;       // receive shift register
    reg       first;    // current byte is byte 0 of the transaction
    reg       par;      // current byte index (mod 2)

    initial begin
        bitcnt   = 3'd0;  sr = 7'd0;  first = 1'b1;  par = 1'b0;
        rx_byte  = 8'd0;  rx_first = 1'b0;  ev_tgl = 1'b0;
    end

    // ---- rising edge: shift in MOSI -------------------------------------------------------
    always @(posedge sck or posedge cs_n) begin
        if (cs_n) begin
            bitcnt <= 3'd0;
            sr     <= 7'd0;
            first  <= 1'b1;
            par    <= 1'b0;
        end else begin
            bitcnt <= bitcnt + 3'd1;
            sr     <= {sr[5:0], mosi};
            if (bitcnt == 3'd7) begin
                first <= 1'b0;
                par   <= ~par;
            end
        end
    end

    // byte complete (8th rising edge): publish to the system domain.  Not reset by CS so a
    // toggle is never lost; bitcnt==7 already implies CS is low (bitcnt is held 0 by CS).
    always @(posedge sck) begin
        if (bitcnt == 3'd7) begin
            rx_byte  <= {sr, mosi};
            rx_first <= first;
            ev_tgl   <= ~ev_tgl;
        end
    end

    // ---- falling edge: shift out MISO -----------------------------------------------------
    // After the 8th rising edge of byte j-1 the bit counter is 0 again: load byte j from
    // slot[j % 2] (par already toggled to j % 2).  Byte 0 (command) sends zeros.
    (* IOB = "TRUE" *) reg miso_q;
    reg [6:0] txsr;
    initial begin txsr = 7'd0; miso_q = 1'b0; end
    always @(negedge sck or posedge cs_n) begin
        if (cs_n) begin
            txsr   <= 7'd0;
            miso_q <= 1'b0;
        end else if (bitcnt == 3'd0) begin
            {miso_q, txsr} <= par ? tx_slot1 : tx_slot0;
        end else begin
            {miso_q, txsr} <= {txsr, 1'b0};
        end
    end
    assign miso = miso_q;
endmodule

`default_nettype wire
