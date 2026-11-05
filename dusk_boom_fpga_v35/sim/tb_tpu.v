// tb_tpu.v — test of tpu_top (2 engines) with an AXI3 DDR model (Icarus)
//   random int8 weights / int16 activations (|x| <= 8191), the real double-Rachel shapes
//   (448x1344 qkv, 448x448, 448x1792, 1792x448, 448x2502 head) + batches of 2..4 vectors,
//   random read latency, random RVALID / WREADY / ARREADY gaps. Every result is compared with a
//   reference computed here. Rows are split between the 2 engines like the host driver does.
`timescale 1ns / 1ps
`default_nettype none

module tb_tpu;
    reg clk = 0, rst = 1;
    always #3.371 clk = ~clk;                     // 148.33 MHz

    // ---- DDR model: 2 MB byte array ------------------------------------------------------------------
    localparam MEM = 2 * 1024 * 1024;
    reg [7:0] mem [0:MEM-1];

    // ---- DUT ---------------------------------------------------------------------------------------
    reg  [31:0] s_awaddr = 0, s_wdata = 0, s_araddr = 0;
    reg         s_awvalid = 0, s_wvalid = 0, s_wlast = 0, s_arvalid = 0, s_bready = 1, s_rready = 1;
    wire        s_awready, s_wready, s_bvalid, s_arready, s_rvalid, s_rlast;
    wire [11:0] s_bid, s_rid;
    wire [31:0] s_rdata;
    wire [63:0] m_araddr, m_awaddr;
    wire [7:0]  m_arlen, m_awlen;
    wire [1:0]  m_arvalid, m_rready, m_awvalid, m_wlast, m_wvalid, m_bready;
    reg  [1:0]  m_arready = 0, m_rvalid = 0, m_rlast = 0, m_awready = 0, m_wready = 0, m_bvalid = 0;
    reg  [127:0] m_rdata = 0;
    wire [127:0] m_wdata;

    tpu_top dut (
        .clk(clk), .rst(rst),
        .s_awaddr(s_awaddr), .s_awid(12'h5A1), .s_awvalid(s_awvalid), .s_awready(s_awready),
        .s_wdata(s_wdata), .s_wlast(s_wlast), .s_wvalid(s_wvalid), .s_wready(s_wready),
        .s_bid(s_bid), .s_bvalid(s_bvalid), .s_bready(s_bready),
        .s_araddr(s_araddr), .s_arid(12'h3C4), .s_arlen(4'd0), .s_arvalid(s_arvalid), .s_arready(s_arready),
        .s_rdata(s_rdata), .s_rid(s_rid), .s_rlast(s_rlast), .s_rvalid(s_rvalid), .s_rready(s_rready),
        .m_araddr(m_araddr), .m_arlen(m_arlen), .m_arvalid(m_arvalid), .m_arready(m_arready),
        .m_rdata(m_rdata), .m_rvalid(m_rvalid), .m_rlast(m_rlast), .m_rready(m_rready),
        .m_awaddr(m_awaddr), .m_awlen(m_awlen), .m_awvalid(m_awvalid), .m_awready(m_awready),
        .m_wdata(m_wdata), .m_wlast(m_wlast), .m_wvalid(m_wvalid), .m_wready(m_wready),
        .m_bvalid(m_bvalid), .m_bready(m_bready)
    );

    integer fails = 0, seed = 11;
    integer gap = 25;                              // % of cycles without data (DDR busy)

    // ---- per-port DDR read / write model --------------------------------------------------------------
    genvar e;
    generate
        for (e = 0; e < 2; e = e + 1) begin : port
            reg [31:0] qa [0:15];
            reg [3:0]  ql [0:15];
            integer qw = 0, qr = 0, beat = 0, lat = 0, i;
            reg [31:0] wa;
            reg [3:0]  wl;
            reg        whave = 0;
            integer    wbeat = 0, bdelay = 0, bpend = 0;
            always @(posedge clk) begin
                // read address
                m_arready[e] <= ($random(seed) % 100) >= 20;
                if (m_arvalid[e] && m_arready[e]) begin
                    qa[qw % 16] = m_araddr[e*32 +: 32]; ql[qw % 16] = m_arlen[e*4 +: 4]; qw = qw + 1;
                    if (m_araddr[e*32 +: 3] != 0) begin fails = fails + 1; $display("FAIL unaligned read %h", m_araddr[e*32 +: 32]); end
                    if ((m_araddr[e*32 +: 32] >> 12) != ((m_araddr[e*32 +: 32] + 8 * m_arlen[e*4 +: 4] + 7) >> 12)) begin
                        fails = fails + 1; $display("FAIL read burst crosses 4 KB at %h", m_araddr[e*32 +: 32]);
                    end
                end
                // read data
                if (m_rvalid[e] && m_rready[e]) begin
                    if (beat == ql[qr % 16]) begin beat = 0; qr = qr + 1; lat = 1 + ($random(seed) & 3); end
                    else beat = beat + 1;
                end
                if (lat > 0) lat = lat - 1;
                if (qr < qw && lat == 0 && (($random(seed) % 100) >= gap || (m_rvalid[e] && !m_rready[e]))) begin
                    m_rvalid[e] <= 1'b1;
                    for (i = 0; i < 8; i = i + 1) m_rdata[e*64 + i*8 +: 8] <= mem[qa[qr % 16] + 8 * beat + i];
                    m_rlast[e] <= (beat == ql[qr % 16]);
                end else begin
                    m_rvalid[e] <= 1'b0;
                end
                // write
                m_awready[e] <= !whave && (($random(seed) % 100) >= 20);
                if (m_awvalid[e] && m_awready[e]) begin
                    wa = m_awaddr[e*32 +: 32]; wl = m_awlen[e*4 +: 4]; whave = 1; wbeat = 0;
                    if (m_awaddr[e*32 +: 3] != 0) begin fails = fails + 1; $display("FAIL unaligned write"); end
                end
                m_wready[e] <= whave && (($random(seed) % 100) >= gap);
                if (m_wvalid[e] && m_wready[e]) begin
                    for (i = 0; i < 8; i = i + 1) mem[wa + 8 * wbeat + i] = m_wdata[e*64 + i*8 +: 8];
                    if (m_wlast[e] != (wbeat == wl)) begin fails = fails + 1; $display("FAIL WLAST beat %0d len %0d", wbeat, wl); end
                    wbeat = wbeat + 1;
                    if (m_wlast[e]) begin whave = 0; bpend = bpend + 1; bdelay = 4; end
                end
                if (bdelay > 0) bdelay = bdelay - 1;
                if (m_bvalid[e] && m_bready[e]) begin m_bvalid[e] <= 0; bpend = bpend - 1; end
                else if (bpend > 0 && bdelay == 0 && !m_bvalid[e]) m_bvalid[e] <= 1;
            end
        end
    endgenerate

    // ---- register access (AXI3 single beats) ------------------------------------------------------------
    task wr(input [31:0] a, input [31:0] d);
        begin
            @(posedge clk);
            s_awaddr <= a; s_awvalid <= 1; s_wdata <= d; s_wvalid <= 1; s_wlast <= 1;
            fork
                begin @(posedge clk); while (!s_awready) @(posedge clk); s_awvalid <= 0; end
                begin @(posedge clk); while (!s_wready) @(posedge clk); s_wvalid <= 0; s_wlast <= 0; end
            join
            while (!s_bvalid) @(posedge clk);
            if (s_bid !== 12'h5A1) begin fails = fails + 1; $display("FAIL BID"); end
            @(posedge clk);
        end
    endtask
    reg [31:0] rd_v;
    task rd(input [31:0] a);
        begin
            @(posedge clk);
            s_araddr <= a; s_arvalid <= 1;
            @(posedge clk); while (!s_arready) @(posedge clk);
            s_arvalid <= 0;
            while (!s_rvalid) @(posedge clk);
            rd_v = s_rdata;
            if (s_rid !== 12'h3C4 || !s_rlast) begin fails = fails + 1; $display("FAIL RID/RLAST"); end
            @(posedge clk);
        end
    endtask

    // ---- one matrix job: rows split between the engines -------------------------------------------------
    localparam XA = 32'h0000_0000, WA = 32'h0001_0000, Y0 = 32'h0018_0000, Y1 = 32'h001C_0000;
    integer K, R, M, r0, m, k, r, ch, cy0, cy1;
    integer ref_v, got;
    task job(input integer kk, input integer rr, input integer mm_);
        begin
            K = kk; R = rr; M = mm_;
            for (m = 0; m < M; m = m + 1)
                for (k = 0; k < K; k = k + 1) begin : fx
                    integer v;
                    v = ($random(seed) % 8192);
                    mem[XA + 2 * (m * K + k)] = v[7:0]; mem[XA + 2 * (m * K + k) + 1] = v[15:8];
                end
            for (r = 0; r < R; r = r + 1)
                for (k = 0; k < K; k = k + 1) mem[WA + r * K + k] = $random(seed);
            // split like the driver: engine 0 rows [0, r0), r0*K multiple of 128
            r0 = R / 2;
            while ((r0 * K) % 128 != 0) r0 = r0 - 1;
            wr(32'h100 + 8'h08, WA);            wr(32'h140 + 8'h08, WA + r0 * K);
            wr(32'h100 + 8'h0C, XA);            wr(32'h140 + 8'h0C, XA);
            wr(32'h100 + 8'h10, Y0);            wr(32'h140 + 8'h10, Y1);
            wr(32'h100 + 8'h14, K / 8);         wr(32'h140 + 8'h14, K / 8);
            wr(32'h100 + 8'h18, r0);            wr(32'h140 + 8'h18, R - r0);
            wr(32'h100 + 8'h1C, M);             wr(32'h140 + 8'h1C, M);
            wr(32'h00C, 3);
            rd(32'h008);
            while (rd_v[1:0] != 0) rd(32'h008);
            if (rd_v[5:4] != 2'b11) begin fails = fails + 1; $display("FAIL done bits %b", rd_v[5:4]); end
            rd(32'h120); cy0 = rd_v;  rd(32'h160); cy1 = rd_v;
            // check every result
            ch = 0;
            for (r = 0; r < R; r = r + 1)
                for (m = 0; m < M; m = m + 1) begin : chk
                    integer base, xv, wv;
                    ref_v = 0;
                    for (k = 0; k < K; k = k + 1) begin
                        xv = $signed({mem[XA + 2 * (m * K + k) + 1], mem[XA + 2 * (m * K + k)]});
                        wv = $signed(mem[WA + r * K + k]);
                        ref_v = ref_v + xv * wv;
                    end
                    base = (r < r0) ? Y0 + 16 * r : Y1 + 16 * (r - r0);
                    got = $signed({mem[base + 4*m + 3], mem[base + 4*m + 2], mem[base + 4*m + 1], mem[base + 4*m]});
                    ch = ch + 1;
                    if (got !== ref_v) begin
                        fails = fails + 1;
                        if (fails < 8) $display("FAIL K%0d R%0d M%0d row %0d m %0d: got %0d want %0d", K, R, M, r, m, got, ref_v);
                    end
                end
            $display("%s K=%0d rows=%0d M=%0d  %0d results  cycles e0 %0d e1 %0d  (%.1f weight bytes/clock)",
                     fails ? "FAIL" : "PASS", K, R, M, ch, cy0, cy1, (1.0 * R * K) / ((cy0 > cy1) ? cy0 : cy1));
        end
    endtask

    initial begin
        repeat (10) @(posedge clk);
        rst = 0;
        repeat (5) @(posedge clk);
        rd(32'h000);
        if (rd_v !== 32'h44545055) begin fails = fails + 1; $display("FAIL ID %h", rd_v); end else $display("PASS ID DTPU");
        rd(32'h004); $display("     CONFIG %h", rd_v);
        job(64, 10, 3);          // small: rows not even, 3 vectors
        job(448, 1344, 1);       // qkv
        job(448, 448, 1);        // out proj
        job(1792, 448, 1);       // mlp.2
        job(448, 1792, 2);       // mlp.0, 2 vectors
        job(448, 2502, 1);       // head (vocab)
        job(448, 448, 4);        // 4-vector batch
        gap = 0;
        job(448, 1344, 1);       // full-speed DDR: peak bytes / clock
        $display("==== %s (%0d fails) ====", fails ? "FAILED" : "ALL PASS", fails);
        $finish;
    end
    integer TMO = 20_000_000;
    initial begin
        if ($value$plusargs("tmo=%d", TMO)) ;
        #(TMO);
        $display("FAIL timeout: e0 busy %b done %b rd_full %0d rd_phase %b rx %b xb_left %0d wb_left %0d infl %0d pushed %0d out %0d done %0d of_cnt %0d w_on %b awv %b",
                 dut.g_eng[0].u_mv.busy, dut.g_eng[0].u_mv.done, dut.g_eng[0].u_mv.rd_full, dut.g_eng[0].u_mv.rd_phase,
                 dut.g_eng[0].u_mv.rd_phase_rx, dut.g_eng[0].u_mv.xb_left, dut.g_eng[0].u_mv.wb_left,
                 dut.g_eng[0].u_mv.inflight, dut.g_eng[0].u_mv.rows_pushed, dut.g_eng[0].u_mv.rows_out,
                 dut.g_eng[0].u_mv.rows_done, dut.g_eng[0].u_mv.of_cnt, dut.g_eng[0].u_mv.w_on, dut.g_eng[0].u_mv.awvalid);
        $finish;
    end
endmodule

`default_nettype wire
