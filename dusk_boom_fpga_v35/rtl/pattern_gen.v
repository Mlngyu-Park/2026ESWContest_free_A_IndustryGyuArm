// pattern_gen.v — HDMI test patterns (one register stage)
//   pattern 0 : 8 vertical colour bars, 160 px each (white, yellow, cyan, green,
//               magenta, red, blue, black) — 100 % amplitude (0xFF / 0x00)
//   pattern 1 : horizontal grey gradient, value = floor(x / 5) (0..255 over 1280 px)
//   pattern 2 : 32x32 black/white checkerboard (white where x[5] ^ y[5] = 1)
//   pattern 3 : solid colour from SOLID_R/G/B
//   other     : colour bars
`timescale 1ns / 1ps
`default_nettype none

module pattern_gen (
    input  wire        clk,
    input  wire [11:0] hc,
    input  wire [10:0] vc,
    input  wire [8:0]  gx,      // floor(hc/5)
    input  wire        de_i,
    input  wire        hs_i,
    input  wire        vs_i,
    input  wire [7:0]  pattern,
    input  wire [7:0]  solid_r,
    input  wire [7:0]  solid_g,
    input  wire [7:0]  solid_b,
    output reg  [7:0]  r,
    output reg  [7:0]  g,
    output reg  [7:0]  b,
    output reg         de_o,
    output reg         hs_o,
    output reg         vs_o
);
    wire [2:0] bar  = gx[7:5];                 // floor(x / 160)
    wire       chk  = hc[5] ^ vc[5];

    reg [7:0] nr, ng, nb;
    always @* begin
        case (pattern)
            8'd1: begin nr = gx[7:0]; ng = gx[7:0]; nb = gx[7:0]; end
            8'd2: begin nr = {8{chk}}; ng = {8{chk}}; nb = {8{chk}}; end
            8'd3: begin nr = solid_r; ng = solid_g; nb = solid_b; end
            default: begin
                nr = {8{~bar[1]}};
                ng = {8{~bar[2]}};
                nb = {8{~bar[0]}};
            end
        endcase
        if (!de_i) begin nr = 8'd0; ng = 8'd0; nb = 8'd0; end
    end

    always @(posedge clk) begin
        r    <= nr;
        g    <= ng;
        b    <= nb;
        de_o <= de_i;
        hs_o <= hs_i;
        vs_o <= vs_i;
    end
endmodule

`default_nettype wire
