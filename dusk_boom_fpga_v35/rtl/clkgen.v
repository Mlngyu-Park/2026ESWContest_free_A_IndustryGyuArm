// clkgen.v — clocking for DUSK_BOOM robot board v2.3 (ALINX AC7010C, XC7Z010-1CLG400)
//
// Clock source: the AC7010C core board has a 50 MHz active oscillator wired to PL pin U18
// (IO_L12P_T1_MRCC_34) — ALINX "AC7010C/AC7020C User Manual" Rev 1.0, Part 5.2
// "PL system clock source": signal 500_CLK -> ZYNQ pin U18.  No PS7 / FCLK is needed,
// so the PL design runs even without an FSBL (e.g. JTAG-only configuration).
//
//   clk50   : 50 MHz (BUFG)       system domain: SPI register engine, CDC, camera counters
//   MMCME2  : 50 MHz / 3 x 44.5 = VCO 741.667 MHz  (PFD 16.67 MHz, VCO range 600..1200)
//     CLKOUT0 /2   = 370.83 MHz   clk_ser  (OSERDES CLK, DDR -> 741.67 Mb/s per lane)
//     CLKOUT1 /10  =  74.167 MHz  clk_pix  (OSERDES CLKDIV, video timing)
//     CLKOUT2 /31  =  23.92 MHz   clk_cam  (OV2640 XCLK, forwarded through ODDR)
//     CLKOUT3 /5   = 148.33 MHz   clk_tpu  (TPU, GP0 + HP1/HP2 AXI ports)
//   74.25 MHz exactly is not reachable from 50 MHz with one MMCM and an integer 5:1 ratio
//   (needs M/D = 14.85 = 297/20).  74.1667 MHz is -0.11 % off the 60 Hz CEA-861 value and
//   only -0.012 % off the standard 720p59.94 pixel clock (74.25/1.001 = 74.176 MHz), well
//   inside the +-0.5 % tolerance; frame rate 59.93 Hz.
//   CLKFBOUT_MULT_F = 44.5 is fractional with an EVEN integer part on purpose: for even
//   integer parts the nextpnr-xilinx FASM writer produces the same counter settings as
//   Xilinx XAPP888 (HT=LT=21, FRAC=4, WF_R=WF_F=1, PM_F=2); for odd integer parts
//   (e.g. 59.375) it omits the odd-cycle term and would program a wrong VCO.
//   Integer-only alternative (no fractional counter at all): D=1, M=15 -> 75.0 MHz (+1 %).
//
// Simulation (`define SIM): ideal clocks with the same ratios (bit period 1.348 ns),
// clk_pix delivered 1 ps after the coincident clk_ser edge (see tmds_serializer.v),
// `locked` rises 2 us after time 0.  The test bench may stop the video clocks by setting
// <inst>.sim_run = 0 (speeds up long camera simulations).
`timescale 1ns / 1ps
`default_nettype none

module clkgen (
    input  wire clk50_in,   // pad U18
    output wire clk50,
    output wire clk_pix,
    output wire clk_ser,
    output wire clk_cam,
    output wire clk_tpu,    // 741.67 / 5 = 148.33 MHz: TPU + its AXI ports (tpu_top.v)
    output wire locked
);
`ifdef SIM
    reg ser_r = 1'b0, pix_r = 1'b0, cam_r = 1'b0, lock_r = 1'b0;
    reg sim_run = 1'b1;
    integer e = 0;
    initial begin
        #2000 lock_r = 1'b1;
    end
    initial forever begin
        #1.348;
        if (sim_run) begin
            ser_r = ~ser_r;
            e = e + 1;
            if (e == 5) begin e = 0; pix_r = ~pix_r; end
        end
    end
    initial forever begin
        #20.899 cam_r = ~cam_r;          // 23.92 MHz
    end
    reg tpu_r = 1'b0;
    initial forever begin
        #3.371 tpu_r = ~tpu_r;           // 148.33 MHz
    end
    assign clk_tpu = tpu_r;
    assign clk50   = clk50_in;
    assign clk_ser = ser_r;
    assign #0.001 clk_pix = pix_r;       // 1 ps after the coincident clk_ser edge
    assign clk_cam = cam_r;
    assign locked  = lock_r;
`else
    wire clk50_ibuf, clkfb, ser_u, pix_u, cam_u, tpu_u;

    IBUF u_ibuf (.I(clk50_in), .O(clk50_ibuf));
    BUFG u_bufg50 (.I(clk50_ibuf), .O(clk50));

    // MMCME2_BASE: no CLKIN2/CLKINSEL/DRP pins to tie off (the open-source flow routes
    // tied pins through the interconnect; BASE avoids that and is identical in Vivado).
    // Feedback is the direct CLKFBOUT -> CLKFBIN path (internal compensation): no phase
    // relation to the 50 MHz input is required, only between clk_ser and clk_pix, which
    // both come from this MMCM through matched BUFGs.
    MMCME2_BASE #(
        .BANDWIDTH          ("OPTIMIZED"),
        .STARTUP_WAIT       ("FALSE"),
        .CLKIN1_PERIOD      (20.000),
        .REF_JITTER1        (0.010),
        .DIVCLK_DIVIDE      (3),
        .CLKFBOUT_MULT_F    (44.500),
        .CLKFBOUT_PHASE     (0.000),
        .CLKOUT0_DIVIDE_F   (2.000),
        .CLKOUT0_PHASE      (0.000),
        .CLKOUT0_DUTY_CYCLE (0.500),
        .CLKOUT1_DIVIDE     (10),
        .CLKOUT1_PHASE      (0.000),
        .CLKOUT1_DUTY_CYCLE (0.500),
        .CLKOUT2_DIVIDE     (31),
        .CLKOUT2_PHASE      (0.000),
        .CLKOUT2_DUTY_CYCLE (0.500),
        .CLKOUT3_DIVIDE     (5),
        .CLKOUT3_PHASE      (0.000),
        .CLKOUT3_DUTY_CYCLE (0.500)
    ) u_mmcm (
        .CLKIN1    (clk50),
        .CLKFBIN   (clkfb),
        .CLKFBOUT  (clkfb),
        .CLKFBOUTB (),
        .CLKOUT0   (ser_u),
        .CLKOUT0B  (),
        .CLKOUT1   (pix_u),
        .CLKOUT1B  (),
        .CLKOUT2   (cam_u),
        .CLKOUT2B  (),
        .CLKOUT3   (tpu_u),
        .CLKOUT3B  (),
        .CLKOUT4   (),
        .CLKOUT5   (),
        .CLKOUT6   (),
        .LOCKED    (locked),
        .PWRDWN    (1'b0),
        .RST       (1'b0)
    );

    BUFG u_bufgser (.I(ser_u),   .O(clk_ser));
    BUFG u_bufgpix (.I(pix_u),   .O(clk_pix));
    BUFG u_bufgcam (.I(cam_u),   .O(clk_cam));
    BUFG u_bufgtpu (.I(tpu_u),   .O(clk_tpu));
`endif
endmodule

// Forward a clock to a pin through an ODDR (D1=1, D2=0): avoids routing a global clock
// into general fabric / IOB data paths.
module clk_forward (
    input  wire clk,
    output wire q
);
`ifdef SIM
    assign q = clk;
`else
    ODDR #(
        .DDR_CLK_EDGE ("OPPOSITE_EDGE"),
        .INIT         (1'b0),
        .SRTYPE       ("SYNC")
    ) u_oddr (
        .Q  (q),
        .C  (clk),
        .CE (1'b1),
        .D1 (1'b1),
        .D2 (1'b0),
        .R  (1'b0),
        .S  (1'b0)
    );
`endif
endmodule

`default_nettype wire
