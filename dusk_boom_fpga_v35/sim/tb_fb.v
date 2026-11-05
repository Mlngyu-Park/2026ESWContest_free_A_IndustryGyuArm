// tb_fb.v — unit test of fb_scanout (Icarus): small screen 256 x 6 (H_TOT 400, V_TOT 9),
// AXI3 slave with random ARREADY stalls, random read latency and random RVALID gaps.
// Memory model: the 64-bit word at byte address A is {A + 4, A}, so the pixel at (x, y) must be
// BASE + y * STRIDE + 4 * x (low 24 bits = R G B).  Checks every visible pixel of frames 2..4,
// the "late" flag, AXI burst rules (16 beats, 128 B aligned, no 4 KB crossing) and that no fetch
// starts while enable = 0.
`timescale 1ns / 1ps
`default_nettype none

module tb_fb;
    localparam H_ACT = 256, H_TOT = 400, V_ACT = 6, V_TOT = 9, STRIDE = 1024;
    localparam [31:0] BASE = 32'h1FC0_0000;

    reg clk = 0, rst = 1, enable = 0;
    always #6.734 clk = ~clk;

    reg [11:0] hc = 0;
    reg [10:0] vc = 0;
    always @(posedge clk) begin
        if (hc == H_TOT - 1) begin hc <= 0; vc <= (vc == V_TOT - 1) ? 0 : vc + 1; end
        else hc <= hc + 1;
    end

    wire [7:0] r, g, b;
    wire late;
    wire [31:0] araddr;
    wire [3:0] arlen, arcache, arqos;
    wire [2:0] arsize, arprot;
    wire [1:0] arburst, arlock;
    wire [5:0] arid;
    wire arvalid, rready;
    reg  arready = 0;
    wire [63:0] rdata;
    reg  rvalid_r = 0;
    wire rlast;

    fb_scanout #(.BASE(BASE), .H_ACT(H_ACT), .V_ACT(V_ACT), .V_TOT(V_TOT), .STRIDE(STRIDE)) dut (
        .clk(clk), .rst(rst), .enable(enable), .hc(hc), .vc(vc),
        .r(r), .g(g), .b(b), .late(late),
        .araddr(araddr), .arlen(arlen), .arsize(arsize), .arburst(arburst), .arlock(arlock),
        .arcache(arcache), .arprot(arprot), .arqos(arqos), .arid(arid),
        .arvalid(arvalid), .arready(arready),
        .rdata(rdata), .rvalid(rvalid_r), .rlast(rlast), .rready(rready)
    );

    integer fails = 0, checked = 0, bursts = 0, seed = 7;
    task fail(input [8*80-1:0] what); begin fails = fails + 1; $display("FAIL %0s", what); end endtask

    // ---- AXI slave: queue of accepted bursts, data after a random latency --------------------------
    reg [31:0] qa [0:31];
    integer qw = 0, qr = 0, beat = 0, lat = 0;
    wire [31:0] cur = qa[qr % 32] + 8 * beat;
    assign rdata = {cur + 32'd4, cur};
    assign rlast = rvalid_r && (beat == 15);

    always @(posedge clk) begin
        arready <= ($random(seed) % 4) != 0;                       // 25 % stalls
        if (arvalid && arready) begin
            bursts = bursts + 1;
            if (arlen != 4'd15 || arsize != 3'd3 || arburst != 2'b01) fail("burst format");
            if (araddr[6:0] != 0) fail("burst not 128 B aligned");
            if ((araddr >> 12) != ((araddr + 127) >> 12)) fail("burst crosses 4 KB");
            if (araddr < BASE || araddr >= BASE + V_ACT * STRIDE) fail("address outside the screen");
            qa[qw % 32] <= araddr; qw = qw + 1;
        end
        if (rvalid_r && rready) begin
            if (beat == 15) begin beat = 0; qr = qr + 1; lat = 3 + ($random(seed) & 15); end
            else beat = beat + 1;
        end
        if (lat > 0) lat = lat - 1;
        rvalid_r <= (qr < qw) && (lat == 0) && (($random(seed) % 5) != 0);   // 20 % gaps
    end

    // ---- pixel check: output belongs to (hc, vc) of the previous clock -------------------------------
    reg [11:0] hc1; reg [10:0] vc1; integer frame = 0;
    always @(posedge clk) begin
        hc1 <= hc; vc1 <= vc;
        if (hc == 0 && vc == 0) frame = frame + 1;
    end
    wire [31:0] want = BASE + vc1 * STRIDE + 4 * hc1;
    always @(negedge clk) begin
        if (enable && frame >= 2 && frame <= 4 && hc1 < H_ACT && vc1 < V_ACT) begin
            checked = checked + 1;
            if ({r, g, b} !== want[23:0]) begin
                fails = fails + 1;
                if (fails < 10) $display("FAIL pixel (%0d,%0d) got %06h want %06h", hc1, vc1, {r, g, b}, want[23:0]);
            end
        end
    end

    initial begin
        repeat (20) @(posedge clk);
        rst = 0;
        repeat (3 * H_TOT) @(posedge clk);
        if (bursts != 0) fail("bursts while disabled");
        @(posedge clk); enable = 1;
        wait (frame == 5);
        if (late) fail("late flag set");
        else $display("PASS late flag stays 0");
        enable = 0;
        repeat (V_TOT * H_TOT) @(posedge clk);
        begin : idle
            integer b0; b0 = bursts;
            repeat (2 * V_TOT * H_TOT) @(posedge clk);
            if (bursts != b0) fail("bursts after disable");
            else $display("PASS no fetch after disable");
        end
        if (checked != 3 * V_ACT * H_ACT) fail("pixel count");
        $display("%s %0d pixels checked, %0d bursts, %0d fails", fails == 0 ? "PASS" : "FAIL", checked, bursts, fails);
        $display("==== %s ====", fails == 0 ? "ALL PASS" : "FAILED");
        $finish;
    end
endmodule

`default_nettype wire
