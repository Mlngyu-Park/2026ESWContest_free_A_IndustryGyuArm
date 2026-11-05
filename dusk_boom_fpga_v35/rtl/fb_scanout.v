// fb_scanout.v — framebuffer scanout: Linux draws into DDR, the PL reads it over S_AXI_HP0 and
// feeds the HDMI encoder.  One clock domain: the pixel clock is also the AXI clock (74.25 MHz x
// 64 bit = 594 MB/s on HP0, the 1280x720x4 B x 60 Hz screen needs 221 MB/s).
//
//   format : XRGB8888 little endian (Linux "x8r8g8b8"): byte0 = B, byte1 = G, byte2 = R
//            one 64-bit beat = 2 pixels, pixel x even in [31:0], x odd in [63:32]
//   memory : BASE + y * STRIDE, 128 B AXI3 bursts (16 x 8 B, never cross 4 KB: STRIDE and BASE
//            are multiples of 128)
//   lines  : two line buffers.  While line y is shown from buffer y[0], line y+1 is fetched into
//            the other one (a whole line time = H_TOT clocks).  Line 0 is fetched during the last
//            line of the frame (vertical blanking).
//   enable : only starts new fetches; a fetch that is running always finishes (AXI bursts can not
//            be aborted).  The pixel output is valid one clock after hc/vc (same latency as
//            pattern_gen), so the caller can switch between the two.
//   late   : sticky, set when a fetch was still running when the next one was due (screen tearing,
//            should never happen); cleared while enable = 0.
`timescale 1ns / 1ps
`default_nettype none

module fb_scanout #(
    parameter [31:0] BASE   = 32'h1FC0_0000,
    parameter        H_ACT  = 1280,
    parameter        V_ACT  = 720,
    parameter        V_TOT  = 750,
    parameter        STRIDE = 5120,          // bytes per line in memory
    parameter        MAX_OUT = 4             // AXI read bursts in flight
) (
    input  wire        clk,
    input  wire        rst,
    input  wire        enable,
    input  wire [11:0] hc,
    input  wire [10:0] vc,
    output wire [7:0]  r,
    output wire [7:0]  g,
    output wire [7:0]  b,
    output reg         late,
    // AXI3 read master (to S_AXI_HP0)
    output reg  [31:0] araddr,
    output wire [3:0]  arlen,
    output wire [2:0]  arsize,
    output wire [1:0]  arburst,
    output wire [1:0]  arlock,
    output wire [3:0]  arcache,
    output wire [2:0]  arprot,
    output wire [3:0]  arqos,
    output wire [5:0]  arid,
    output reg         arvalid,
    input  wire        arready,
    input  wire [63:0] rdata,
    input  wire        rvalid,
    input  wire        rlast,
    output wire        rready
);
    localparam BEATS  = H_ACT / 2;           // 64-bit words per line
    localparam BURSTS = BEATS / 16;          // 128 B bursts per line

    assign arlen   = 4'd15;                  // 16 beats
    assign arsize  = 3'd3;                   // 8 bytes
    assign arburst = 2'b01;                  // INCR
    assign arlock  = 2'b00;
    assign arcache = 4'b0011;                // bufferable, modifiable
    assign arprot  = 3'b000;
    assign arqos   = 4'hF;                   // screen first: a late line is visible
    assign arid    = 6'd0;
    assign rready  = 1'b1;

    // ---- line buffers: 2 x 1024 words (only BEATS used per line) ----------------------------
    reg [63:0] lbuf [0:2047];
    reg [63:0] rd_word;
    reg        rd_odd;

    // ---- fetch state ---------------------------------------------------------------------------
    reg        busy;
    reg        wsel;                         // line buffer being filled
    reg [31:0] line_addr;
    reg [7:0]  bursts_sent;
    reg [3:0]  in_flight;
    reg [9:0]  beat;

    wire at_line_start = (hc == 12'd0);
    wire want_line0    = (vc == V_TOT - 1);
    wire want_next     = (vc < V_ACT - 1);
    wire [10:0] next_line = want_line0 ? 11'd0 : vc + 11'd1;
    wire trigger = at_line_start && (want_line0 || want_next);

    wire ar_fire = arvalid && arready;
    wire r_fire  = rvalid && rready;
    wire r_end   = r_fire && rlast;

    always @(posedge clk) begin
        if (rst) begin
            busy        <= 1'b0;
            wsel        <= 1'b0;
            line_addr   <= BASE;
            bursts_sent <= 8'd0;
            in_flight   <= 4'd0;
            beat        <= 10'd0;
            arvalid     <= 1'b0;
            araddr      <= BASE;
            late        <= 1'b0;
        end else begin
            if (!enable) late <= 1'b0;

            // start a line fetch
            if (trigger && enable) begin
                if (busy) begin
                    late <= 1'b1;
                end else begin
                    busy        <= 1'b1;
                    wsel        <= next_line[0];
                    line_addr   <= BASE + next_line * STRIDE;
                    bursts_sent <= 8'd0;
                    beat        <= 10'd0;
                end
            end

            // address channel: keep up to MAX_OUT bursts in flight
            if (ar_fire) begin
                arvalid <= 1'b0;
            end else if (busy && !arvalid && bursts_sent < BURSTS && in_flight < MAX_OUT
                         && !(trigger && enable)) begin
                arvalid <= 1'b1;
                araddr  <= line_addr + {bursts_sent, 7'd0};
            end
            if (ar_fire) bursts_sent <= bursts_sent + 8'd1;

            case ({ar_fire, r_end})
                2'b10:   in_flight <= in_flight + 4'd1;
                2'b01:   in_flight <= in_flight - 4'd1;
                default: ;
            endcase

            // data channel
            if (r_fire) begin
                beat <= beat + 10'd1;
                if (beat == BEATS - 1) busy <= 1'b0;
            end
        end
    end

    always @(posedge clk) begin
        if (r_fire) lbuf[{wsel, beat}] <= rdata;
    end

    // ---- pixel side: word of (hc, vc) one clock later --------------------------------------------
    always @(posedge clk) begin
        rd_word <= lbuf[{vc[0], hc[10:1]}];
        rd_odd  <= hc[0];
    end

    wire [31:0] px = rd_odd ? rd_word[63:32] : rd_word[31:0];
    assign r = px[23:16];
    assign g = px[15:8];
    assign b = px[7:0];
endmodule

`default_nettype wire
