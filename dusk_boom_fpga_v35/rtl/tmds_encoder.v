// tmds_encoder.v — DVI 1.0 TMDS 8b/10b encoder (2-stage pipeline, latency 2 clk)
//   stage 1: transition minimisation (XOR / XNOR chain) -> q_m[8:0], ones count
//   stage 2: DC balancing with running disparity cnt (reset to 0 during blanking),
//            control tokens during blanking (C1,C0):
//              00 -> 10'b1101010100   01 -> 10'b0010101011
//              10 -> 10'b0101010100   11 -> 10'b1010101011
//   q[0] is the first bit on the wire.
`timescale 1ns / 1ps
`default_nettype none

module tmds_encoder (
    input  wire       clk,
    input  wire       rst,
    input  wire [7:0] d,
    input  wire       de,
    input  wire       c0,
    input  wire       c1,
    output reg  [9:0] q
);
    function [3:0] ones8;
        input [7:0] x;
        ones8 = {3'b0, x[0]} + {3'b0, x[1]} + {3'b0, x[2]} + {3'b0, x[3]} +
                {3'b0, x[4]} + {3'b0, x[5]} + {3'b0, x[6]} + {3'b0, x[7]};
    endfunction

    // ---------------- stage 1 ----------------
    wire [3:0] n1d      = ones8(d);
    wire       use_xnor = (n1d > 4'd4) || ((n1d == 4'd4) && (d[0] == 1'b0));
    wire [8:0] qm;
    assign qm[0] = d[0];
    genvar i;
    generate
        for (i = 1; i < 8; i = i + 1) begin : g_qm
            assign qm[i] = use_xnor ? ~(qm[i-1] ^ d[i]) : (qm[i-1] ^ d[i]);
        end
    endgenerate
    assign qm[8] = ~use_xnor;

    reg [8:0] qm_r  = 9'd0;
    reg [3:0] n1q_r = 4'd0;
    reg       de_r  = 1'b0;
    reg [1:0] c_r   = 2'b00;
    always @(posedge clk) begin
        qm_r  <= qm;
        n1q_r <= ones8(qm[7:0]);
        de_r  <= de;
        c_r   <= {c1, c0};
    end

    // ---------------- stage 2 ----------------
    // disparity of q_m[7:0]: N1 - N0 = 2*N1 - 8   (range -8..+8, always even)
    wire signed [5:0] dq = $signed({1'b0, n1q_r, 1'b0}) - 6'sd8;
    reg  signed [5:0] cnt = 6'sd0;

    always @(posedge clk) begin
        if (rst) begin
            cnt <= 6'sd0;
            q   <= 10'b1101010100;
        end else if (!de_r) begin
            cnt <= 6'sd0;
            case (c_r)
                2'b00: q <= 10'b1101010100;
                2'b01: q <= 10'b0010101011;
                2'b10: q <= 10'b0101010100;
                default: q <= 10'b1010101011;
            endcase
        end else if ((cnt == 6'sd0) || (n1q_r == 4'd4)) begin
            q[9]   <= ~qm_r[8];
            q[8]   <=  qm_r[8];
            q[7:0] <=  qm_r[8] ? qm_r[7:0] : ~qm_r[7:0];
            cnt    <=  qm_r[8] ? (cnt + dq) : (cnt - dq);
        end else if (((cnt > 6'sd0) && (n1q_r > 4'd4)) || ((cnt < 6'sd0) && (n1q_r < 4'd4))) begin
            q[9]   <= 1'b1;
            q[8]   <= qm_r[8];
            q[7:0] <= ~qm_r[7:0];
            cnt    <= cnt + (qm_r[8] ? 6'sd2 : 6'sd0) - dq;
        end else begin
            q[9]   <= 1'b0;
            q[8]   <= qm_r[8];
            q[7:0] <= qm_r[7:0];
            cnt    <= cnt - (qm_r[8] ? 6'sd0 : 6'sd2) + dq;
        end
    end
endmodule

`default_nettype wire
