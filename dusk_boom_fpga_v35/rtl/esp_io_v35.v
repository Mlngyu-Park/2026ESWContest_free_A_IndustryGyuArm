// esp_io_v35.v — the 8 lines between the FPGA and the ESP32-S3 (ESP_IO0..7), board v3.5
//
//   ESP_IO0 (ESP IO4)  robot bridge, FPGA -> ESP : Zynq UART0 TX (EMIO). Driven only while
//                      bridge_on = 1 (Linux dusk_robotd running), high-Z otherwise.
//   ESP_IO1 (ESP IO5)  robot bridge, ESP -> FPGA : goes to Zynq UART0 RX (EMIO). Never driven
//                      by the FPGA. Held at idle (1) towards the PS while bridge_on = 0, so a
//                      floating line can not produce garbage bytes. The pin has a PULLUP (xdc).
//   ESP_IO2..7         general purpose as before: dir[i] = 1 -> FPGA drives out[i],
//                      dir[i] = 0 -> high-Z input (reset default)
//   in                 2-FF synchronized pin levels of all 8 lines (system clock domain)
//
// The UART path is combinational (pad <-> PS EMIO): 1 Mbps needs no PL clock, and the PS UART
// oversamples / synchronizes its RX input itself.
`timescale 1ns / 1ps
`default_nettype none

module esp_io_v35 (
    input  wire       clk,
    input  wire [7:0] out,
    input  wire [7:0] dir,        // bits 1:0 ignored
    output wire [7:0] in,
    // robot UART bridge (Zynq PS UART0 over EMIO)
    input  wire       bridge_on,  // asynchronous (PS GPIO), quasi-static
    input  wire       uart_tx,    // PS UART0 TX  -> ESP_IO0
    output wire       uart_rx,    // ESP_IO1      -> PS UART0 RX
    inout  wire       io0, io1, io2, io3, io4, io5, io6, io7
);
    // general-purpose lines: registered once so the pins never glitch on register writes
    reg [7:2] oe_q = 6'h00;
    reg [7:2] o_q  = 6'h00;
    always @(posedge clk) begin
        oe_q <= dir[7:2];
        o_q  <= out[7:2];
    end

    assign io0 = bridge_on ? uart_tx : 1'bz;
    assign io1 = 1'bz;
    assign io2 = oe_q[2] ? o_q[2] : 1'bz;
    assign io3 = oe_q[3] ? o_q[3] : 1'bz;
    assign io4 = oe_q[4] ? o_q[4] : 1'bz;
    assign io5 = oe_q[5] ? o_q[5] : 1'bz;
    assign io6 = oe_q[6] ? o_q[6] : 1'bz;
    assign io7 = oe_q[7] ? o_q[7] : 1'bz;

    assign uart_rx = bridge_on ? io1 : 1'b1;

    wire [7:0] pins = {io7, io6, io5, io4, io3, io2, io1, io0};
    genvar i;
    generate
        for (i = 0; i < 8; i = i + 1) begin : g_sync
            sync_bit #(.N(2)) u_s (.clk(clk), .d(pins[i]), .q(in[i]));
        end
    endgenerate
endmodule

`default_nettype wire
