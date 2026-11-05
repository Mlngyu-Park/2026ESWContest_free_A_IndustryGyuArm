// tmds_serializer.v — 10:1 TMDS serializer + differential output buffer
//
// Synthesis : OSERDESE2 MASTER + SLAVE cascade (DATA_WIDTH 10, DDR) clocked by
//             clk_ser = 5 x pixel clock (370.83 MHz here; 371.25 MHz for exact 720p60) and clk_pix (CLKDIV),
//             followed by OBUFDS (IOSTANDARD TMDS_33 set in the XDC).
//             d[0] is transmitted first (DVI/HDMI order).
// Simulation: (`define SIM) behavioural DDR shift register with the same bit order.
//             It re-aligns to clk_pix rising edges, so the SIM clock model must deliver
//             clk_pix a few ps after the coincident clk_ser edge (see clkgen.v).
`timescale 1ns / 1ps
`default_nettype none

module tmds_serializer (
    input  wire       clk_pix,
    input  wire       clk_ser,
    input  wire       rst,       // synchronous to clk_pix, active high
    input  wire [9:0] d,
    output wire       out_p,
    output wire       out_n
);
`ifdef SIM
    reg [9:0] sh      = 10'd0;
    reg       pix_q   = 1'b0;
    always @(posedge clk_ser or negedge clk_ser) begin
        pix_q <= clk_pix;
        if (clk_pix && !pix_q) sh <= rst ? 10'd0 : d;     // first bit edge of a new word
        else                   sh <= {1'b0, sh[9:1]};
    end
    assign out_p =  sh[0];
    assign out_n = ~sh[0];
`else
    wire shift1, shift2, oq;

    OSERDESE2 #(
        .DATA_RATE_OQ   ("DDR"),
        .DATA_RATE_TQ   ("SDR"),
        .DATA_WIDTH     (10),
        .INIT_OQ        (1'b0),
        .INIT_TQ        (1'b0),
        .SERDES_MODE    ("MASTER"),
        .SRVAL_OQ       (1'b0),
        .SRVAL_TQ       (1'b0),
        .TBYTE_CTL      ("FALSE"),
        .TBYTE_SRC      ("FALSE"),
        .TRISTATE_WIDTH (1)
    ) u_master (
        .OFB       (),
        .OQ        (oq),
        .SHIFTOUT1 (),
        .SHIFTOUT2 (),
        .TBYTEOUT  (),
        .TFB       (),
        .TQ        (),
        .CLK       (clk_ser),
        .CLKDIV    (clk_pix),
        .D1        (d[0]),
        .D2        (d[1]),
        .D3        (d[2]),
        .D4        (d[3]),
        .D5        (d[4]),
        .D6        (d[5]),
        .D7        (d[6]),
        .D8        (d[7]),
        .OCE       (1'b1),
        .RST       (rst),
        .SHIFTIN1  (shift1),
        .SHIFTIN2  (shift2),
        .T1        (1'b0),
        .T2        (1'b0),
        .T3        (1'b0),
        .T4        (1'b0),
        .TBYTEIN   (1'b0),
        .TCE       (1'b0)
    );

    OSERDESE2 #(
        .DATA_RATE_OQ   ("DDR"),
        .DATA_RATE_TQ   ("SDR"),
        .DATA_WIDTH     (10),
        .INIT_OQ        (1'b0),
        .INIT_TQ        (1'b0),
        .SERDES_MODE    ("SLAVE"),
        .SRVAL_OQ       (1'b0),
        .SRVAL_TQ       (1'b0),
        .TBYTE_CTL      ("FALSE"),
        .TBYTE_SRC      ("FALSE"),
        .TRISTATE_WIDTH (1)
    ) u_slave (
        .OFB       (),
        .OQ        (),
        .SHIFTOUT1 (shift1),
        .SHIFTOUT2 (shift2),
        .TBYTEOUT  (),
        .TFB       (),
        .TQ        (),
        .CLK       (clk_ser),
        .CLKDIV    (clk_pix),
        .D1        (1'b0),
        .D2        (1'b0),
        .D3        (d[8]),
        .D4        (d[9]),
        .D5        (1'b0),
        .D6        (1'b0),
        .D7        (1'b0),
        .D8        (1'b0),
        .OCE       (1'b1),
        .RST       (rst),
        .SHIFTIN1  (1'b0),
        .SHIFTIN2  (1'b0),
        .T1        (1'b0),
        .T2        (1'b0),
        .T3        (1'b0),
        .T4        (1'b0),
        .TBYTEIN   (1'b0),
        .TCE       (1'b0)
    );

    OBUFDS u_obufds (
        .I  (oq),
        .O  (out_p),
        .OB (out_n)
    );
`endif
endmodule

`default_nettype wire
