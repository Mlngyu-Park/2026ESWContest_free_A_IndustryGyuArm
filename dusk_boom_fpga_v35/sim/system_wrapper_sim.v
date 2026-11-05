// system_wrapper_sim.v — simulation stand-in for the Vivado block design wrapper (PS7 only)
// Same port list as the wrapper make_wrapper produces for ../vivado/build.tcl.
// The test bench drives the "PS side" through the regs below (hierarchical access):
//   ps_uart_tx  -> esp_uart_txd   (PS UART0 TX over EMIO)
//   esp_uart_rxd                  (PS UART0 RX, sampled by the test bench)
//   ps_gpio_o / ps_gpio_t -> emio_o / emio_t,  emio_i read by the test bench
`timescale 1ns / 1ps
`default_nettype none

module system_wrapper (
    inout  wire [14:0] DDR_addr,
    inout  wire [2:0]  DDR_ba,
    inout  wire        DDR_cas_n,
    inout  wire        DDR_ck_n,
    inout  wire        DDR_ck_p,
    inout  wire        DDR_cke,
    inout  wire        DDR_cs_n,
    inout  wire [3:0]  DDR_dm,
    inout  wire [31:0] DDR_dq,
    inout  wire [3:0]  DDR_dqs_n,
    inout  wire [3:0]  DDR_dqs_p,
    inout  wire        DDR_odt,
    inout  wire        DDR_ras_n,
    inout  wire        DDR_reset_n,
    inout  wire        DDR_we_n,
    inout  wire        FIXED_IO_ddr_vrn,
    inout  wire        FIXED_IO_ddr_vrp,
    inout  wire [53:0] FIXED_IO_mio,
    inout  wire        FIXED_IO_ps_clk,
    inout  wire        FIXED_IO_ps_porb,
    inout  wire        FIXED_IO_ps_srstb,
    input  wire        esp_uart_rxd,
    output wire        esp_uart_txd,
    input  wire [15:0] emio_i,
    output wire [15:0] emio_o,
    output wire [15:0] emio_t,
    // S_AXI_HP0 (AXI3 64 bit) - screen scanout; this stand-in answers reads with data = address
    input  wire        fb_aclk,
    input  wire [31:0] fb_axi_araddr,
    input  wire [1:0]  fb_axi_arburst,
    input  wire [3:0]  fb_axi_arcache,
    input  wire [5:0]  fb_axi_arid,
    input  wire [3:0]  fb_axi_arlen,
    input  wire [1:0]  fb_axi_arlock,
    input  wire [2:0]  fb_axi_arprot,
    input  wire [3:0]  fb_axi_arqos,
    output wire        fb_axi_arready,
    input  wire [2:0]  fb_axi_arsize,
    input  wire        fb_axi_arvalid,
    output wire [63:0] fb_axi_rdata,
    output wire [5:0]  fb_axi_rid,
    output wire        fb_axi_rlast,
    input  wire        fb_axi_rready,
    output wire [1:0]  fb_axi_rresp,
    output wire        fb_axi_rvalid,
    input  wire [31:0] fb_axi_awaddr,
    input  wire [1:0]  fb_axi_awburst,
    input  wire [3:0]  fb_axi_awcache,
    input  wire [5:0]  fb_axi_awid,
    input  wire [3:0]  fb_axi_awlen,
    input  wire [1:0]  fb_axi_awlock,
    input  wire [2:0]  fb_axi_awprot,
    input  wire [3:0]  fb_axi_awqos,
    output wire        fb_axi_awready,
    input  wire [2:0]  fb_axi_awsize,
    input  wire        fb_axi_awvalid,
    input  wire [63:0] fb_axi_wdata,
    input  wire [5:0]  fb_axi_wid,
    input  wire        fb_axi_wlast,
    output wire        fb_axi_wready,
    input  wire [7:0]  fb_axi_wstrb,
    input  wire        fb_axi_wvalid,
    output wire [5:0]  fb_axi_bid,
    input  wire        fb_axi_bready,
    output wire [1:0]  fb_axi_bresp,
    output wire        fb_axi_bvalid,
    // TPU ports (tpu_ctl = M_AXI_GP0 master, tpu_m0/m1 = S_AXI_HP1/HP2 slaves): idle in this stand-in
    input  wire          tpu_aclk,
    output wire [31:0]   tpu_ctl_araddr,
    output wire [1:0]    tpu_ctl_arburst,
    output wire [3:0]    tpu_ctl_arcache,
    output wire [11:0]   tpu_ctl_arid,
    output wire [3:0]    tpu_ctl_arlen,
    output wire [1:0]    tpu_ctl_arlock,
    output wire [2:0]    tpu_ctl_arprot,
    output wire [3:0]    tpu_ctl_arqos,
    input  wire          tpu_ctl_arready,
    output wire [2:0]    tpu_ctl_arsize,
    output wire          tpu_ctl_arvalid,
    output wire [31:0]   tpu_ctl_awaddr,
    output wire [1:0]    tpu_ctl_awburst,
    output wire [3:0]    tpu_ctl_awcache,
    output wire [11:0]   tpu_ctl_awid,
    output wire [3:0]    tpu_ctl_awlen,
    output wire [1:0]    tpu_ctl_awlock,
    output wire [2:0]    tpu_ctl_awprot,
    output wire [3:0]    tpu_ctl_awqos,
    input  wire          tpu_ctl_awready,
    output wire [2:0]    tpu_ctl_awsize,
    output wire          tpu_ctl_awvalid,
    input  wire [11:0]   tpu_ctl_bid,
    output wire          tpu_ctl_bready,
    input  wire [1:0]    tpu_ctl_bresp,
    input  wire          tpu_ctl_bvalid,
    input  wire [31:0]   tpu_ctl_rdata,
    input  wire [11:0]   tpu_ctl_rid,
    input  wire          tpu_ctl_rlast,
    output wire          tpu_ctl_rready,
    input  wire [1:0]    tpu_ctl_rresp,
    input  wire          tpu_ctl_rvalid,
    output wire [31:0]   tpu_ctl_wdata,
    output wire [11:0]   tpu_ctl_wid,
    output wire          tpu_ctl_wlast,
    input  wire          tpu_ctl_wready,
    output wire [3:0]    tpu_ctl_wstrb,
    output wire          tpu_ctl_wvalid,
    input  wire [31:0]   tpu_m0_araddr,
    input  wire [1:0]    tpu_m0_arburst,
    input  wire [3:0]    tpu_m0_arcache,
    input  wire [5:0]    tpu_m0_arid,
    input  wire [3:0]    tpu_m0_arlen,
    input  wire [1:0]    tpu_m0_arlock,
    input  wire [2:0]    tpu_m0_arprot,
    input  wire [3:0]    tpu_m0_arqos,
    output wire          tpu_m0_arready,
    input  wire [2:0]    tpu_m0_arsize,
    input  wire          tpu_m0_arvalid,
    input  wire [31:0]   tpu_m0_awaddr,
    input  wire [1:0]    tpu_m0_awburst,
    input  wire [3:0]    tpu_m0_awcache,
    input  wire [5:0]    tpu_m0_awid,
    input  wire [3:0]    tpu_m0_awlen,
    input  wire [1:0]    tpu_m0_awlock,
    input  wire [2:0]    tpu_m0_awprot,
    input  wire [3:0]    tpu_m0_awqos,
    output wire          tpu_m0_awready,
    input  wire [2:0]    tpu_m0_awsize,
    input  wire          tpu_m0_awvalid,
    output wire [5:0]    tpu_m0_bid,
    input  wire          tpu_m0_bready,
    output wire [1:0]    tpu_m0_bresp,
    output wire          tpu_m0_bvalid,
    output wire [63:0]   tpu_m0_rdata,
    output wire [5:0]    tpu_m0_rid,
    output wire          tpu_m0_rlast,
    input  wire          tpu_m0_rready,
    output wire [1:0]    tpu_m0_rresp,
    output wire          tpu_m0_rvalid,
    input  wire [63:0]   tpu_m0_wdata,
    input  wire [5:0]    tpu_m0_wid,
    input  wire          tpu_m0_wlast,
    output wire          tpu_m0_wready,
    input  wire [7:0]    tpu_m0_wstrb,
    input  wire          tpu_m0_wvalid,
    input  wire [31:0]   tpu_m1_araddr,
    input  wire [1:0]    tpu_m1_arburst,
    input  wire [3:0]    tpu_m1_arcache,
    input  wire [5:0]    tpu_m1_arid,
    input  wire [3:0]    tpu_m1_arlen,
    input  wire [1:0]    tpu_m1_arlock,
    input  wire [2:0]    tpu_m1_arprot,
    input  wire [3:0]    tpu_m1_arqos,
    output wire          tpu_m1_arready,
    input  wire [2:0]    tpu_m1_arsize,
    input  wire          tpu_m1_arvalid,
    input  wire [31:0]   tpu_m1_awaddr,
    input  wire [1:0]    tpu_m1_awburst,
    input  wire [3:0]    tpu_m1_awcache,
    input  wire [5:0]    tpu_m1_awid,
    input  wire [3:0]    tpu_m1_awlen,
    input  wire [1:0]    tpu_m1_awlock,
    input  wire [2:0]    tpu_m1_awprot,
    input  wire [3:0]    tpu_m1_awqos,
    output wire          tpu_m1_awready,
    input  wire [2:0]    tpu_m1_awsize,
    input  wire          tpu_m1_awvalid,
    output wire [5:0]    tpu_m1_bid,
    input  wire          tpu_m1_bready,
    output wire [1:0]    tpu_m1_bresp,
    output wire          tpu_m1_bvalid,
    output wire [63:0]   tpu_m1_rdata,
    output wire [5:0]    tpu_m1_rid,
    output wire          tpu_m1_rlast,
    input  wire          tpu_m1_rready,
    output wire [1:0]    tpu_m1_rresp,
    output wire          tpu_m1_rvalid,
    input  wire [63:0]   tpu_m1_wdata,
    input  wire [5:0]    tpu_m1_wid,
    input  wire          tpu_m1_wlast,
    output wire          tpu_m1_wready,
    input  wire [7:0]    tpu_m1_wstrb,
    input  wire          tpu_m1_wvalid
);
    reg        ps_uart_tx = 1'b1;
    reg [15:0] ps_gpio_o  = 16'h0000;
    reg [15:0] ps_gpio_t  = 16'hFFFF;     // after reset every GPIO is an input (tri-stated)

    assign esp_uart_txd = ps_uart_tx;
    assign emio_o = ps_gpio_o;
    assign emio_t = ps_gpio_t;

    // ---- HP0 read model: accepts an address every cycle, 16-entry queue, data 12 clocks later ----
    reg [31:0] q_addr [0:15];
    reg [3:0]  q_len  [0:15];
    reg [4:0]  q_wr = 0, q_rd = 0;
    reg [3:0]  beat = 0;
    reg [7:0]  wait_c = 0;
    integer    ar_count = 0, write_attempts = 0;
    wire       q_full  = (q_wr - q_rd) == 5'd16;
    wire       q_empty = (q_wr == q_rd);

    assign fb_axi_arready = !q_full;
    assign fb_axi_rvalid  = !q_empty && (wait_c >= 8'd12);
    wire [31:0] cur = q_addr[q_rd[3:0]] + {beat, 3'b000};
    assign fb_axi_rdata   = {cur + 32'd4, cur};
    assign fb_axi_rlast   = fb_axi_rvalid && (beat == q_len[q_rd[3:0]]);
    assign fb_axi_rid     = 6'd0;
    assign fb_axi_rresp   = 2'b00;
    assign fb_axi_awready = 1'b0;
    assign fb_axi_wready  = 1'b0;
    assign fb_axi_bvalid  = 1'b0;
    assign fb_axi_bid     = 6'd0;
    assign fb_axi_bresp   = 2'b00;

    always @(posedge fb_aclk) begin
        if (fb_axi_awvalid || fb_axi_wvalid) write_attempts = write_attempts + 1;
        if (fb_axi_arvalid && fb_axi_arready) begin
            q_addr[q_wr[3:0]] <= fb_axi_araddr;
            q_len[q_wr[3:0]]  <= fb_axi_arlen;
            q_wr <= q_wr + 5'd1;
            ar_count = ar_count + 1;
        end
        if (q_empty)              wait_c <= 8'd0;
        else if (wait_c < 8'd12)  wait_c <= wait_c + 8'd1;
        if (fb_axi_rvalid && fb_axi_rready) begin
            if (fb_axi_rlast) begin beat <= 4'd0; q_rd <= q_rd + 5'd1; wait_c <= 8'd0; end
            else              beat <= beat + 4'd1;
        end
    end
    // TPU: the stand-in never accesses the registers and never answers the memory ports
    assign tpu_ctl_araddr = 0;
    assign tpu_ctl_arburst = 0;
    assign tpu_ctl_arcache = 0;
    assign tpu_ctl_arid = 0;
    assign tpu_ctl_arlen = 0;
    assign tpu_ctl_arlock = 0;
    assign tpu_ctl_arprot = 0;
    assign tpu_ctl_arqos = 0;
    assign tpu_ctl_arsize = 0;
    assign tpu_ctl_arvalid = 0;
    assign tpu_ctl_awaddr = 0;
    assign tpu_ctl_awburst = 0;
    assign tpu_ctl_awcache = 0;
    assign tpu_ctl_awid = 0;
    assign tpu_ctl_awlen = 0;
    assign tpu_ctl_awlock = 0;
    assign tpu_ctl_awprot = 0;
    assign tpu_ctl_awqos = 0;
    assign tpu_ctl_awsize = 0;
    assign tpu_ctl_awvalid = 0;
    assign tpu_ctl_bready = 1'b1;
    assign tpu_ctl_rready = 1'b1;
    assign tpu_ctl_wdata = 0;
    assign tpu_ctl_wid = 0;
    assign tpu_ctl_wlast = 0;
    assign tpu_ctl_wstrb = 0;
    assign tpu_ctl_wvalid = 0;
    assign tpu_m0_arready = 0;
    assign tpu_m0_awready = 0;
    assign tpu_m0_bid = 0;
    assign tpu_m0_bresp = 0;
    assign tpu_m0_bvalid = 0;
    assign tpu_m0_rdata = 0;
    assign tpu_m0_rid = 0;
    assign tpu_m0_rlast = 0;
    assign tpu_m0_rresp = 0;
    assign tpu_m0_rvalid = 0;
    assign tpu_m0_wready = 0;
    assign tpu_m1_arready = 0;
    assign tpu_m1_awready = 0;
    assign tpu_m1_bid = 0;
    assign tpu_m1_bresp = 0;
    assign tpu_m1_bvalid = 0;
    assign tpu_m1_rdata = 0;
    assign tpu_m1_rid = 0;
    assign tpu_m1_rlast = 0;
    assign tpu_m1_rresp = 0;
    assign tpu_m1_rvalid = 0;
    assign tpu_m1_wready = 0;
endmodule
