// hdmi_timing.v — video timing generator, default CEA-861 1280x720p60
//   H: 1280 active + 110 front porch + 40 sync + 220 back porch = 1650
//   V:  720 active +   5 front porch +  5 sync +  20 back porch =  750
//   positive H/V sync, VSYNC edges aligned with the HSYNC leading edge (CEA-861 progressive)
// Outputs are registered.  gx = floor(hc / 5) (used for the 1280->256 gradient and the
// 160-pixel colour bars without any divider/multiplier).
`timescale 1ns / 1ps
`default_nettype none

module hdmi_timing #(
    parameter H_ACT  = 1280,
    parameter H_FP   = 110,
    parameter H_SYNC = 40,
    parameter H_BP   = 220,
    parameter V_ACT  = 720,
    parameter V_FP   = 5,
    parameter V_SYNC = 5,
    parameter V_BP   = 20,
    parameter HS_POL = 1,       // 1 = positive sync
    parameter VS_POL = 1
) (
    input  wire        clk,
    input  wire        rst,
    output reg  [11:0] hc,
    output reg  [10:0] vc,
    output reg  [8:0]  gx,
    output reg         de,
    output reg         hs,
    output reg         vs
);
    localparam H_TOT    = H_ACT + H_FP + H_SYNC + H_BP;
    localparam V_TOT    = V_ACT + V_FP + V_SYNC + V_BP;
    localparam HS_START = H_ACT + H_FP;
    localparam HS_END   = H_ACT + H_FP + H_SYNC;
    localparam VS_L0    = V_ACT + V_FP - 1;          // vsync starts at hsync of this line
    localparam VS_L1    = V_ACT + V_FP + V_SYNC - 1; // ... and ends at hsync of this line

    reg [11:0] h;
    reg [10:0] v;
    reg [2:0]  m5;
    reg [8:0]  q5;

    always @(posedge clk) begin
        if (rst) begin
            h <= 12'd0;  v <= 11'd0;  m5 <= 3'd0;  q5 <= 9'd0;
        end else begin
            if (h == H_TOT - 1) begin
                h  <= 12'd0;
                m5 <= 3'd0;
                q5 <= 9'd0;
                v  <= (v == V_TOT - 1) ? 11'd0 : v + 11'd1;
            end else begin
                h <= h + 12'd1;
                if (m5 == 3'd4) begin m5 <= 3'd0; q5 <= q5 + 9'd1; end
                else            m5 <= m5 + 3'd1;
            end
        end
    end

    wire hs_a = (h >= HS_START) && (h < HS_END);
    wire vs_a = ((v == VS_L0) && (h >= HS_START)) ||
                ((v >  VS_L0) && (v <  VS_L1)) ||
                ((v == VS_L1) && (h <  HS_START));
    wire de_a = (h < H_ACT) && (v < V_ACT);

    always @(posedge clk) begin
        if (rst) begin
            hc <= 12'd0; vc <= 11'd0; gx <= 9'd0; de <= 1'b0;
            hs <= (HS_POL == 0); vs <= (VS_POL == 0);
        end else begin
            hc <= h;
            vc <= v;
            gx <= q5;
            de <= de_a;
            hs <= (HS_POL != 0) ? hs_a : ~hs_a;
            vs <= (VS_POL != 0) ? vs_a : ~vs_a;
        end
    end
endmodule

`default_nettype wire
