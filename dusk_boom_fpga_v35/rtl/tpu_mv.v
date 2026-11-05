// tpu_mv.v — DUSK_BOOM TPU engine: int8 weights x int16 activations, up to 4 vectors at once
//
//   Y[r][m] = sum_k W[r][k] * X[m][k]      r < ROWS, m < M (1..4), k < K = 8*K8 (8..2048)
//
//   One engine owns one AXI3 64-bit port (S_AXI_HPx of the Zynq PS):
//     read  : first X (M*K int16, [m][k] contiguous at X_ADDR) into on-chip buffers, then the
//             weight rows (ROWS*K int8, row-major at W_ADDR) — 8 weights per beat, one beat per clock
//     write : one 16-byte result per row at Y_ADDR + 16*r: int32 Y[r][0..3] (slots >= M are junk)
//   Per beat: 8 weights x M vectors = 32 multipliers (DSP48), 3-level adder tree, 32-bit accumulate.
//   The host keeps |X| <= 8191 (int16 with a per-vector scale, see rachel_tpu.cpp):
//   |Y| <= 2048 * 127 * 8191 < 2^31, the 32-bit accumulator never wraps.
//
//   Alignment rules (host): X_ADDR, W_ADDR, Y_ADDR multiples of 128 B. Reads go out as 128 B bursts
//   (+ one shorter last burst), so starting on a 128 B boundary they never cross 4 KB.
`timescale 1ns / 1ps
`default_nettype none

module tpu_mv #(
    parameter MAXOUT = 8                 // AXI read bursts in flight
) (
    input  wire        clk,
    input  wire        rst,
    // control
    input  wire        start,
    input  wire [31:0] w_addr,
    input  wire [31:0] x_addr,
    input  wire [31:0] y_addr,
    input  wire [8:0]  k8,               // K / 8, 1..256
    input  wire [15:0] rows,             // 1..65535
    input  wire [2:0]  mm,               // vectors 1..4
    output reg         busy,
    output reg         done,
    output reg  [31:0] cycles,
    output reg  [31:0] stalls,
    // AXI3 read master
    output reg  [31:0] araddr,
    output reg  [3:0]  arlen,
    output wire [2:0]  arsize,
    output wire [1:0]  arburst,
    output wire [3:0]  arcache,
    output reg         arvalid,
    input  wire        arready,
    input  wire [63:0] rdata,
    input  wire        rvalid,
    input  wire        rlast,
    output wire        rready,
    // AXI3 write master
    output reg  [31:0] awaddr,
    output reg  [3:0]  awlen,
    output wire [2:0]  awsize,
    output wire [1:0]  awburst,
    output wire [3:0]  awcache,
    output reg         awvalid,
    input  wire        awready,
    output wire [63:0] wdata,
    output wire [7:0]  wstrb,
    output reg         wlast,
    output reg         wvalid,
    input  wire        wready,
    input  wire        bvalid,
    output wire        bready
);
    assign arsize = 3'd3;  assign arburst = 2'b01;  assign arcache = 4'b0011;
    assign awsize = 3'd3;  assign awburst = 2'b01;  assign awcache = 4'b0011;
    assign wstrb  = 8'hFF;
    assign bready = 1'b1;

    // ---- job registers ---------------------------------------------------------------------------
    reg [8:0]  K8;
    reg [15:0] NR;
    reg [2:0]  NM;
    reg [31:0] XA, WA, YA;
    reg  [21:0] x_beats;                              // M * K / 4   (registered: no long path)
    reg  [25:0] w_beats;                              // ROWS * K / 8

    // ---- read address generator: phase 0 = X, phase 1 = W ----------------------------------------
    reg        rd_phase;                              // 0: X, 1: W
    reg [21:0] rd_full;                               // 16-beat bursts still to request
    reg [3:0]  rd_rem;                                // + one last burst of this many beats
    wire       rd_none = (rd_full == 0) && (rd_rem == 0);
    wire [4:0] blen = (rd_full != 0) ? 5'd16 : {1'b0, rd_rem};
    reg [31:0] rd_ptr;
    reg        rd_on;
    reg [3:0]  inflight;
    wire ar_fire = arvalid && arready;
    wire r_fire  = rvalid && rready;
    wire r_end   = r_fire && rlast;

    // ---- data beat counters ------------------------------------------------------------------------
    reg [21:0] xb_left;                               // X beats still to receive
    reg [1:0]  xm;                                    // which vector is arriving
    reg [8:0]  xw;                                    // 128-bit word inside the vector
    reg        xhalf;
    reg [63:0] xlo;
    reg [25:0] wb_left;                               // W beats still to receive
    reg [8:0]  kk;                                    // beat inside the current row

    // ---- output FIFO (128-bit entries, one per row) --------------------------------------------------
    localparam FD = 64;
    (* ram_style = "distributed" *) reg [127:0] ofifo [0:FD-1];
    reg [5:0]   of_wp, of_rp;
    reg [6:0]   of_cnt;
    wire        room = of_cnt < (FD - 24);           // enough space for everything in the pipeline
    reg         rd_phase_rx;                          // phase of the beats now arriving
    assign rready = rd_phase_rx ? room : 1'b1;

    // ---- X buffers: 4 vectors x 256 words x 128 bit -------------------------------------------------
    wire        xwe = r_fire && !rd_phase_rx && xhalf;
    wire [127:0] xrd0, xrd1, xrd2, xrd3;
    tpu_ram256x128 u_xb0 (.clk(clk), .we(xwe && xm == 2'd0), .wa(xw[7:0]), .wd({rdata, xlo}), .ra(kk[7:0]), .rd(xrd0));
    tpu_ram256x128 u_xb1 (.clk(clk), .we(xwe && xm == 2'd1), .wa(xw[7:0]), .wd({rdata, xlo}), .ra(kk[7:0]), .rd(xrd1));
    tpu_ram256x128 u_xb2 (.clk(clk), .we(xwe && xm == 2'd2), .wa(xw[7:0]), .wd({rdata, xlo}), .ra(kk[7:0]), .rd(xrd2));
    tpu_ram256x128 u_xb3 (.clk(clk), .we(xwe && xm == 2'd3), .wa(xw[7:0]), .wd({rdata, xlo}), .ra(kk[7:0]), .rd(xrd3));

    // ---- compute pipeline ---------------------------------------------------------------------------
    // s1: weight beat + X words read
    reg         v1, f1, l1;
    reg [63:0]  w1;
    // RAM output (registered inside the BRAM) arrives 2 clocks after the address: s2 below
    reg  [127:0] xr0, xr1, xr2, xr3;                 // s3: register in front of the multipliers
    wire        wbeat = r_fire && rd_phase_rx;
    always @(posedge clk) begin
        v1  <= wbeat && !rst;
        f1  <= (kk == 9'd0);
        l1  <= (kk == K8 - 9'd1);
        w1  <= rdata;
    end

    // s2: weight beat waits for the RAM (output register), s3: both in registers next to the DSPs
    reg        v1b, f1b, l1b, v1c, f1c, l1c;
    reg [63:0] w1b, w1c;
    always @(posedge clk) begin
        v1b <= v1 && !rst;  f1b <= f1;  l1b <= l1;  w1b <= w1;
        v1c <= v1b && !rst; f1c <= f1b; l1c <= l1b; w1c <= w1b;
        xr0 <= xrd0;  xr1 <= xrd1;  xr2 <= xrd2;  xr3 <= xrd3;
    end
    // s4: 32 products (DSP48)
    reg v2, f2, l2;
    (* use_dsp = "yes" *) reg signed [23:0] p [0:31];
    integer i, m;
    always @(posedge clk) begin
        v2 <= v1c && !rst;  f2 <= f1c;  l2 <= l1c;
        for (i = 0; i < 8; i = i + 1) begin
            p[0*8+i] <= $signed(w1c[i*8 +: 8]) * $signed(xr0[i*16 +: 16]);
            p[1*8+i] <= $signed(w1c[i*8 +: 8]) * $signed(xr1[i*16 +: 16]);
            p[2*8+i] <= $signed(w1c[i*8 +: 8]) * $signed(xr2[i*16 +: 16]);
            p[3*8+i] <= $signed(w1c[i*8 +: 8]) * $signed(xr3[i*16 +: 16]);
        end
    end
    // s3..s5: adder tree per vector (8 -> 4 -> 2 -> 1)
    reg v3, f3, l3, v4, f4, l4, v5, f5, l5;
    // adders stay in the fabric: the 80 DSP48 of the 7010 are for the 64 multipliers (+1 screen)
    (* use_dsp = "no" *) reg signed [24:0] a3 [0:15];
    (* use_dsp = "no" *) reg signed [25:0] a4 [0:7];
    (* use_dsp = "no" *) reg signed [26:0] a5 [0:3];
    always @(posedge clk) begin
        v3 <= v2 && !rst;  f3 <= f2;  l3 <= l2;
        v4 <= v3 && !rst;  f4 <= f3;  l4 <= l3;
        v5 <= v4 && !rst;  f5 <= f4;  l5 <= l4;
        for (i = 0; i < 16; i = i + 1) a3[i] <= p[2*i] + p[2*i+1];
        for (i = 0; i < 8; i = i + 1)  a4[i] <= a3[2*i] + a3[2*i+1];
        for (i = 0; i < 4; i = i + 1)  a5[i] <= a4[2*i] + a4[2*i+1];
    end
    // s6: accumulate; on the last beat of a row the result goes to the FIFO
    (* use_dsp = "no" *) reg signed [31:0] acc [0:3];
    reg               push;
    (* use_dsp = "no" *) reg [127:0] pushd;
    always @(posedge clk) begin
        push <= 1'b0;
        if (v5) begin
            for (m = 0; m < 4; m = m + 1)
                acc[m] <= f5 ? {{5{a5[m][26]}}, a5[m]} : acc[m] + {{5{a5[m][26]}}, a5[m]};
            if (l5) begin
                push  <= 1'b1;
                pushd <= f5 ? {{{5{a5[3][26]}}, a5[3]}, {{5{a5[2][26]}}, a5[2]},
                               {{5{a5[1][26]}}, a5[1]}, {{5{a5[0][26]}}, a5[0]}}
                            : {acc[3] + {{5{a5[3][26]}}, a5[3]}, acc[2] + {{5{a5[2][26]}}, a5[2]},
                               acc[1] + {{5{a5[1][26]}}, a5[1]}, acc[0] + {{5{a5[0][26]}}, a5[0]}};
            end
        end
    end

    // ---- write side: bursts of 8 rows (16 beats); a short burst only at the very end ---------------
    reg [15:0] rows_out;                              // rows handed to the write channel
    reg [15:0] rows_done;                             // rows whose write response came back
    reg [3:0]  wb_n;                                  // rows in the current burst
    reg [4:0]  wbeat_i;
    reg        w_on;
    reg [3:0]  b_pend;
    reg [31:0] wptr;
    reg [15:0] rows_pushed;
    wire       all_pushed = (rows_pushed == NR);
    wire [6:0] avail = of_cnt;
    wire [127:0] of_head = ofifo[of_rp];
    assign wdata = wbeat_i[0] ? of_head[127:64] : of_head[63:0];
    wire w_fire = wvalid && wready;
    wire pop    = w_fire && wbeat_i[0];
    reg [15:0] burst_rows [0:15];                     // rows per outstanding burst (for B)
    reg [3:0]  bq_w, bq_r;

    always @(posedge clk) begin
        if (push) ofifo[of_wp] <= pushd;
    end

    // ---- main control -------------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            busy <= 1'b0; done <= 1'b0; cycles <= 0; stalls <= 0;
            arvalid <= 1'b0; rd_on <= 1'b0; inflight <= 0;
            awvalid <= 1'b0; wvalid <= 1'b0; wlast <= 1'b0; w_on <= 1'b0;
            of_wp <= 0; of_rp <= 0; of_cnt <= 0;
            rows_out <= 0; rows_done <= 0; rows_pushed <= 0; b_pend <= 0; bq_w <= 0; bq_r <= 0;
        end else begin
            // -------- start
            if (start && !busy) begin
                busy <= 1'b1; done <= 1'b0; cycles <= 0; stalls <= 0;
                K8 <= (k8 == 0) ? 9'd1 : k8;
                NR <= rows; NM <= (mm == 0) ? 3'd1 : (mm > 4) ? 3'd4 : mm;
                XA <= x_addr; WA <= w_addr; YA <= y_addr;
                rd_phase <= 1'b0; rd_on <= 1'b0; rd_phase_rx <= 1'b0;
                xm <= 0; xw <= 0; xhalf <= 1'b0; kk <= 0;
                of_wp <= 0; of_rp <= 0; of_cnt <= 0;
                rows_out <= 0; rows_done <= 0; rows_pushed <= 0; b_pend <= 0; bq_w <= 0; bq_r <= 0;
                wptr <= y_addr; inflight <= 0;
            end else if (busy) begin
                cycles <= cycles + 1;
                if (rd_phase_rx && !rready && rvalid) stalls <= stalls + 1;
                // job set-up of the read counters one clock after start
                if (cycles == 0) begin                                  // products (registered)
                    x_beats <= NM * {K8, 1'b0};
                    w_beats <= NR * K8;
                end
                if (!rd_on && !rd_phase && cycles == 1) begin           // then the read counters
                    rd_on <= 1'b1; rd_ptr <= XA; rd_full <= x_beats[21:4]; rd_rem <= x_beats[3:0];
                    xb_left <= x_beats; wb_left <= w_beats;
                end
                // -------- read requests
                if (ar_fire) begin
                    arvalid <= 1'b0;
                end else if (rd_on && !arvalid && !rd_none && inflight < MAXOUT) begin
                    arvalid <= 1'b1; araddr <= rd_ptr; arlen <= blen - 5'd1;
                    rd_ptr <= rd_ptr + {blen, 3'b000};
                    if (rd_full != 0) rd_full <= rd_full - 1;
                    else rd_rem <= 4'd0;
                end else if (rd_on && rd_none && !rd_phase && !arvalid) begin
                    rd_phase <= 1'b1; rd_ptr <= WA;                        // X requested -> weights
                    rd_full <= w_beats[25:4]; rd_rem <= w_beats[3:0];
                end
                case ({ar_fire, r_end})
                    2'b10: inflight <= inflight + 1;
                    2'b01: inflight <= inflight - 1;
                    default: ;
                endcase
                // -------- data beats
                if (r_fire && !rd_phase_rx) begin                       // X
                    if (!xhalf) xlo <= rdata;
                    xhalf <= ~xhalf;
                    if (xhalf) begin
                        if (xw == K8 - 9'd1) begin xw <= 0; xm <= xm + 1; end
                        else xw <= xw + 1;
                    end
                    xb_left <= xb_left - 1;
                    if (xb_left == 1) rd_phase_rx <= 1'b1;
                end
                if (r_fire && rd_phase_rx) begin                        // W
                    wb_left <= wb_left - 1;
                    kk <= (kk == K8 - 9'd1) ? 9'd0 : kk + 9'd1;
                end
                // -------- FIFO bookkeeping
                if (push) begin of_wp <= of_wp + 1; rows_pushed <= rows_pushed + 1; end
                if (pop)  of_rp <= of_rp + 1;
                case ({push, pop})
                    2'b10: of_cnt <= of_cnt + 1;
                    2'b01: of_cnt <= of_cnt - 1;
                    default: ;
                endcase
                // -------- write bursts
                if (!w_on && !awvalid && (avail >= 7'd8 || (all_pushed && avail != 0)) && b_pend < 4'd14) begin
                    wb_n <= (avail >= 7'd8) ? 4'd8 : avail[3:0];
                    awaddr <= wptr; awlen <= ((avail >= 7'd8) ? 5'd16 : {avail[3:0], 1'b0}) - 5'd1;
                    awvalid <= 1'b1; w_on <= 1'b1; wbeat_i <= 0;
                    wptr <= wptr + ((avail >= 7'd8) ? 32'd128 : {avail[3:0], 4'b0000});
                end
                if (awvalid && awready) awvalid <= 1'b0;
                if (w_on && !wvalid && !wlast) begin
                    wvalid <= 1'b1; wlast <= (wb_n == 4'd1 && wbeat_i == 0) ? 1'b0 : 1'b0;
                end
                if (w_fire) begin
                    wbeat_i <= wbeat_i + 1;
                    if (wlast) begin
                        wvalid <= 1'b0; wlast <= 1'b0; w_on <= 1'b0;
                        burst_rows[bq_w] <= {12'd0, wb_n}; bq_w <= bq_w + 1;
                        rows_out <= rows_out + wb_n;
                    end else begin
                        wlast <= (wbeat_i + 5'd2 == {wb_n, 1'b0});
                    end
                end
                case ({w_fire && wlast, bvalid})
                    2'b10: b_pend <= b_pend + 1;
                    2'b01: b_pend <= b_pend - 1;
                    default: ;
                endcase
                if (bvalid) begin rows_done <= rows_done + burst_rows[bq_r]; bq_r <= bq_r + 1; end
                // -------- finished: every row written and acknowledged
                if (rows_done == NR && rd_none && rd_phase && cycles > 3) begin
                    busy <= 1'b0; done <= 1'b1;
                end
            end
        end
    end
endmodule

// 256 x 128 bit simple dual-port RAM (one write, one registered read) -> 4 x RAMB18 / 2 x RAMB36
module tpu_ram256x128 (
    input  wire         clk,
    input  wire         we,
    input  wire [7:0]   wa,
    input  wire [127:0] wd,
    input  wire [7:0]   ra,
    output reg  [127:0] rd
);
    (* ram_style = "block" *) reg [127:0] m [0:255];
    reg [127:0] rq;
    always @(posedge clk) begin                       // 2-clock read: RAM latch + output register
        if (we) m[wa] <= wd;
        rq <= m[ra];
        rd <= rq;
    end
endmodule

`default_nettype wire
