// cdc.v — clock-domain-crossing helpers (DUSK_BOOM robot board v2.3 FPGA)
`timescale 1ns / 1ps
`default_nettype none

// 2-FF (or N-FF) level synchronizer for single bits / quasi-static bits
module sync_bit #(
    parameter N    = 2,
    parameter INIT = 1'b0
) (
    input  wire clk,
    input  wire d,
    output wire q
);
    (* ASYNC_REG = "TRUE" *) reg [N-1:0] s = {N{INIT}};
    always @(posedge clk) s <= {s[N-2:0], d};
    assign q = s[N-1];
endmodule

// toggle -> pulse synchronizer: every change of `tgl` (source domain) produces
// one `pulse` cycle in the destination domain, 2..3 cycles later.
module sync_toggle (
    input  wire clk,
    input  wire tgl,
    output wire pulse
);
    (* ASYNC_REG = "TRUE" *) reg [1:0] s = 2'b00;
    reg prev = 1'b0;
    always @(posedge clk) begin
        s    <= {s[0], tgl};
        prev <= s[1];
    end
    assign pulse = s[1] ^ prev;
endmodule

`default_nettype wire
