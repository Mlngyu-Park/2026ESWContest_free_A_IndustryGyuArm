// tpu_top.v — DUSK_BOOM TPU: 2 x tpu_mv engines + register file on M_AXI_GP0 (AXI3 slave)
//
//   Registers (byte offsets from 0x43C0_0000, 32 bit):
//     0x000 ID        R   0x44545055 ("DTPU")
//     0x004 CONFIG    R   [7:0] lanes 8, [15:8] vectors 4, [23:16] engines 2, [31:24] version 1
//     0x008 GSTATUS   R   [1:0] busy (engine 1..0), [5:4] done
//     0x00C GSTART    W   bit e = start engine e (all chosen engines start in the same clock)
//     0x100 + 0x40*e  engine e:
//       +0x00 START W  any write starts it         +0x04 STATUS R  bit0 busy, bit1 done
//       +0x08 W_ADDR   +0x0C X_ADDR   +0x10 Y_ADDR (bytes, 128 B aligned)
//       +0x14 K8 (K/8, 1..256)   +0x18 ROWS (1..65535)   +0x1C M (1..4)
//       +0x20 CYCLES R (clocks of the last job)   +0x24 STALLS R (clocks the result FIFO was full)
//   Data formats: see tpu_mv.v. Host side: tpu_hw.h (rachel_tpu.cpp, Linux /dev/mem).
`timescale 1ns / 1ps
`default_nettype none

module tpu_top (
    input  wire        clk,
    input  wire        rst,
    // AXI3 slave (from M_AXI_GP0), 32-bit data, 12-bit ID
    input  wire [31:0] s_awaddr,
    input  wire [11:0] s_awid,
    input  wire        s_awvalid,
    output wire        s_awready,
    input  wire [31:0] s_wdata,
    input  wire        s_wlast,
    input  wire        s_wvalid,
    output wire        s_wready,
    output reg  [11:0] s_bid,
    output reg         s_bvalid,
    input  wire        s_bready,
    input  wire [31:0] s_araddr,
    input  wire [11:0] s_arid,
    input  wire [3:0]  s_arlen,
    input  wire        s_arvalid,
    output wire        s_arready,
    output reg  [31:0] s_rdata,
    output reg  [11:0] s_rid,
    output reg         s_rlast,
    output reg         s_rvalid,
    input  wire        s_rready,
    // two AXI3 64-bit masters (to S_AXI_HP1 / S_AXI_HP2), flattened [e]
    output wire [63:0] m_araddr,  output wire [7:0] m_arlen,  output wire [1:0] m_arvalid,
    input  wire [1:0]  m_arready, input  wire [127:0] m_rdata, input  wire [1:0] m_rvalid,
    input  wire [1:0]  m_rlast,   output wire [1:0] m_rready,
    output wire [63:0] m_awaddr,  output wire [7:0] m_awlen,  output wire [1:0] m_awvalid,
    input  wire [1:0]  m_awready, output wire [127:0] m_wdata, output wire [1:0] m_wlast,
    output wire [1:0]  m_wvalid,  input  wire [1:0] m_wready,  input  wire [1:0] m_bvalid,
    output wire [1:0]  m_bready
);
    // ---- engine registers --------------------------------------------------------------------------
    reg [31:0] r_w [0:1], r_x [0:1], r_y [0:1];
    reg [8:0]  r_k8 [0:1];
    reg [15:0] r_rows [0:1];
    reg [2:0]  r_m [0:1];
    reg [1:0]  go;
    wire [1:0] busy, done;
    wire [31:0] cyc0, cyc1, st0, st1;

    genvar e;
    generate
        for (e = 0; e < 2; e = e + 1) begin : g_eng
            wire [31:0] cyc, stl;
            tpu_mv u_mv (
                .clk(clk), .rst(rst), .start(go[e]),
                .w_addr(r_w[e]), .x_addr(r_x[e]), .y_addr(r_y[e]),
                .k8(r_k8[e]), .rows(r_rows[e]), .mm(r_m[e]),
                .busy(busy[e]), .done(done[e]), .cycles(cyc), .stalls(stl),
                .araddr(m_araddr[e*32 +: 32]), .arlen(m_arlen[e*4 +: 4]), .arsize(), .arburst(), .arcache(),
                .arvalid(m_arvalid[e]), .arready(m_arready[e]),
                .rdata(m_rdata[e*64 +: 64]), .rvalid(m_rvalid[e]), .rlast(m_rlast[e]), .rready(m_rready[e]),
                .awaddr(m_awaddr[e*32 +: 32]), .awlen(m_awlen[e*4 +: 4]), .awsize(), .awburst(), .awcache(),
                .awvalid(m_awvalid[e]), .awready(m_awready[e]),
                .wdata(m_wdata[e*64 +: 64]), .wstrb(), .wlast(m_wlast[e]), .wvalid(m_wvalid[e]),
                .wready(m_wready[e]), .bvalid(m_bvalid[e]), .bready(m_bready[e])
            );
        end
    endgenerate
    assign cyc0 = g_eng[0].cyc;  assign cyc1 = g_eng[1].cyc;
    assign st0  = g_eng[0].stl;  assign st1  = g_eng[1].stl;

    // ---- AXI3 slave: write -------------------------------------------------------------------------
    reg        aw_got, w_got;
    reg [11:0] aw_id;
    reg [11:0] aw_a;
    reg [31:0] w_d;
    assign s_awready = !aw_got && !s_bvalid;
    assign s_wready  = !w_got && !s_bvalid;
    wire wr_do = aw_got && w_got && !s_bvalid;

    always @(posedge clk) begin
        go <= 2'b00;
        if (rst) begin
            aw_got <= 1'b0; w_got <= 1'b0; s_bvalid <= 1'b0;
        end else begin
            if (s_awvalid && s_awready) begin aw_got <= 1'b1; aw_id <= s_awid; aw_a <= s_awaddr[11:0]; end
            if (s_wvalid && s_wready) begin
                if (!w_got) w_d <= s_wdata;          // first beat counts (bursts: rest ignored)
                if (s_wlast) w_got <= 1'b1;
            end
            if (wr_do) begin
                aw_got <= 1'b0; w_got <= 1'b0;
                s_bvalid <= 1'b1; s_bid <= aw_id;
                if (aw_a == 12'h00C) go <= w_d[1:0];
                else if (aw_a[11:8] == 4'h1) begin : wreg
                    integer k;
                    k = aw_a[6];                     // engine 0 at 0x100, engine 1 at 0x140
                    case (aw_a[5:0])
                        6'h00: go[k] <= 1'b1;
                        6'h08: r_w[k] <= w_d;
                        6'h0C: r_x[k] <= w_d;
                        6'h10: r_y[k] <= w_d;
                        6'h14: r_k8[k] <= w_d[8:0];
                        6'h18: r_rows[k] <= w_d[15:0];
                        6'h1C: r_m[k] <= w_d[2:0];
                        default: ;
                    endcase
                end
            end
            if (s_bvalid && s_bready) s_bvalid <= 1'b0;
        end
    end

    // ---- AXI3 slave: read (bursts return the same register len+1 times) ---------------------------
    reg        rd_act;
    reg [3:0]  rd_n;
    reg [11:0] rd_a;
    assign s_arready = !rd_act;
    function [31:0] regval(input [11:0] a);
        reg k;
        begin
            k = a[6];
            case (a)
                12'h000: regval = 32'h44545055;
                12'h004: regval = {8'd1, 8'd2, 8'd4, 8'd8};
                12'h008: regval = {26'd0, done, 2'b00, busy};
                default:
                    if (a[11:8] == 4'h1)
                        case (a[5:0])
                            6'h04: regval = {30'd0, done[k], busy[k]};
                            6'h08: regval = r_w[k];
                            6'h0C: regval = r_x[k];
                            6'h10: regval = r_y[k];
                            6'h14: regval = {23'd0, r_k8[k]};
                            6'h18: regval = {16'd0, r_rows[k]};
                            6'h1C: regval = {29'd0, r_m[k]};
                            6'h20: regval = k ? cyc1 : cyc0;
                            6'h24: regval = k ? st1 : st0;
                            default: regval = 32'd0;
                        endcase
                    else regval = 32'd0;
            endcase
        end
    endfunction

    always @(posedge clk) begin
        if (rst) begin
            rd_act <= 1'b0; s_rvalid <= 1'b0; s_rlast <= 1'b0;
        end else begin
            if (s_arvalid && s_arready) begin
                rd_act <= 1'b1; rd_n <= s_arlen; rd_a <= s_araddr[11:0]; s_rid <= s_arid;
                s_rvalid <= 1'b1; s_rdata <= regval(s_araddr[11:0]); s_rlast <= (s_arlen == 0);
            end else if (s_rvalid && s_rready) begin
                if (s_rlast) begin
                    s_rvalid <= 1'b0; s_rlast <= 1'b0; rd_act <= 1'b0;
                end else begin
                    rd_n <= rd_n - 1;
                    s_rlast <= (rd_n == 1);
                    s_rdata <= regval(rd_a);
                end
            end
        end
    end
endmodule

`default_nettype wire
