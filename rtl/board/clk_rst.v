// clk_rst.v — board clock buffer/divider + reset release helper
//
// Purpose
//   Own the real-hardware board clock handoff and the synchronous reset
//   release sequence for the platform top.  The default path is a direct
//   BUFG pass-through of the board oscillator.  When CLK_DIVIDE is set to
//   2, 4, or 8 the module switches to BUFGCE_DIV so bring-up can run the
//   core slower without changing the rest of the SoC wiring.  Some board
//   clock sources, notably GT refclk paths that have already gone through
//   BUFG_GT, must bypass this module's clock buffer and use it only for
//   reset release; set INPUT_BUFFERED for that case.
//
// Key interfaces
//   clk_in           — 200 MHz board reference clock.
//   rst_in           — active-high external reset.
//   init_done        — active-high downstream init gate (DDR cal, etc.).
//   dbg_full_rst_in  — active-high JTAG/VIO debug-full-reset request
//                      (task #256).  ORed into the soc_full_rst path so
//                      that downstream `soc_full_rst` consumers see a
//                      single registered driver, not a per-receiver LUT
//                      whose Q→D path regrows the global reset depth.
//   clk_out          — core clock for the SoC.
//   rst_out          — active-high synchronous reset, deasserted after
//                      DEST_SYNC_FF clean clk_out edges once rst_in and
//                      init_done are both high.  Equal to the legacy
//                      `core_rst` (board cold reset only — does NOT
//                      include the JTAG debug-full-reset request).
//   soc_full_rst_out — active-high synchronous reset that asserts on
//                      board cold reset OR JTAG debug-full-reset.
//   core_rst_bank    — `BANK_W`-bit wire bus that mirrors `rst_out`.
//                      Each bit holds the same Q value as `rst_out`.
//                      Driven from the BUFG-buffered broadcast net.
//                      Consumers slice exactly one bit per region.
//   soc_full_rst_bank — same as core_rst_bank, but for soc_full_rst.
//
// Latency
//   Clock pass-through is combinational.  Reset assertion is asynchronous
//   on every stage (xpm_cdc_async_rst async-asserts dest_arst when
//   src_arst rises).  Reset deassertion is synchronised through a
//   DEST_SYNC_FF (=4) FF chain inside the XPM macro and then through one
//   BUFG buffer for global distribution.
//
// Fanout / placement notes (post-vivado_main_d1c9f11 autopsy)
//   The previous implementations chased high-fanout reset broadcasts in
//   user logic — first manual scalar-FF banking with `keep` (the placer
//   still picked 1-2 of the kept scalars as the physical driver), then
//   `MAX_FANOUT` auto-replication on naked FFs (correct for
//   `soc_full_rst_q6` at 16k loads, but `core_rst_q5_reg_rep` at 4k
//   loads still tripped the auto-BUFG insertion code, costing ~2 ns of
//   data-path delay every receive site).
//
//   This revision follows Xilinx UG974 reset-methodology: use the
//   `xpm_cdc_async_rst` parameterized macro for the async-assert /
//   sync-deassert chain (so STA understands the synchroniser
//   semantics — Vivado's auto-replication refuses to touch ASYNC_REG
//   FFs, which the old hand-rolled `rst_pipe` was), then route the
//   broadcast through an EXPLICIT `BUFG`.  An explicit BUFG forces the
//   placer to treat reset arrival as clock-tree skew (balanced by the
//   global clock network) instead of as a data-path delay — that is
//   the ~2 ns of WNS we previously paid every time the placer
//   auto-inserted a BUFG mid-flow.
//
//   References:
//     - Xilinx UG974 (UltraScale Architecture Libraries Guide,
//       xpm_cdc_async_rst section).
//     - AR# 75810 (high-fanout reset distribution best practices).

`default_nettype none

module clk_rst #(
    parameter integer CLK_DIVIDE = 1,
    parameter integer INPUT_BUFFERED = 0,
    // DEST_SYNC_FF for the XPM async-reset synchroniser.  4 mirrors the
    // legacy four-deep `rst_pipe[3:0]` chain; XPM allows 2..10.
    parameter integer DEST_SYNC_FF = 4,
    // Width of the broadcast bus.  Kept as a parameter for source
    // compatibility with downstream consumers that already use the
    // `[BANK_W-1:0]` slice convention.  All bits are driven from the
    // single BUFG-buffered broadcast net.
    parameter integer BANK_W = 8
) (
    input  wire clk_in,
    input  wire rst_in,
    input  wire init_done,
    input  wire dbg_full_rst_in,
    output wire clk_out,
    output wire rst_out,
    output wire soc_full_rst_out,
    output wire [BANK_W-1:0] core_rst_bank,
    output wire [BANK_W-1:0] soc_full_rst_bank
);
    wire clk_buf;
    wire rst_req = rst_in | ~init_done;

    generate
        if (INPUT_BUFFERED != 0) begin : gen_already_buffered
            assign clk_buf = clk_in;
        end else if (CLK_DIVIDE == 1) begin : gen_clk_passthrough
            BUFG u_bufg (
                .I(clk_in),
                .O(clk_buf)
            );
        end else begin : gen_clk_div
            // Reset the divider output while the combined reset request is
            // asserted so divided-clock bring-up starts from a known low
            // phase after reset release.
            BUFGCE_DIV #(
                .BUFGCE_DIVIDE(CLK_DIVIDE)
            ) u_bufgce_div (
                .I(clk_in),
                .CE(1'b1),
                .CLR(rst_req),
                .O(clk_buf)
            );
        end
    endgenerate

    assign clk_out = clk_buf;

    // ─────────────────────────────────────────────────────────────────────
    // Async-reset synchroniser (xpm_cdc_async_rst)
    //
    // src_arst   — combined cold-reset request (board reset || ~init_done).
    // dest_clk   — the buffered core clock the consumers run on.
    // dest_arst  — synchronised reset.  Asserts asynchronously when
    //              src_arst rises; deasserts synchronously DEST_SYNC_FF
    //              cycles after src_arst falls.
    //
    // RST_ACTIVE_HIGH = 1: matches the active-high reset convention used
    // throughout the SoC.  INIT_SYNC_FF = 0: leave the destination FFs
    // un-INIT'd at simulation time (the macro still applies the canonical
    // ASYNC_REG attribute internally so the synchroniser is treated
    // correctly by STA).
    //
    // The XPM macro replaces the old hand-rolled `rst_pipe[3:0]` chain.
    // Sim picks up the behavioural stub from tb/verilator_xilinx_stubs.v
    // (or the unit-tb local stub); Vivado uses the real device library.
    // ─────────────────────────────────────────────────────────────────────
    wire core_rst_unbuf;
    wire soc_full_rst_unbuf;

    xpm_cdc_async_rst #(
        .DEST_SYNC_FF   (DEST_SYNC_FF),
        .INIT_SYNC_FF   (0),
        .RST_ACTIVE_HIGH(1)
    ) u_core_rst_sync (
        .src_arst (rst_req),
        .dest_clk (clk_buf),
        .dest_arst(core_rst_unbuf)
    );

    // soc_full_rst combines the cold reset with the JTAG debug-full-reset
    // request BEFORE the synchroniser so the OR sits inside the reset
    // network rather than at every receiver.  dbg_full_rst_in is sourced
    // from VIO and arrives already registered in the same clock domain
    // (see the upstream sync chain in fpga_top_clocks.vh) so feeding it
    // straight into src_arst is a same-domain merge that the XPM tolerates.
    xpm_cdc_async_rst #(
        .DEST_SYNC_FF   (DEST_SYNC_FF),
        .INIT_SYNC_FF   (0),
        .RST_ACTIVE_HIGH(1)
    ) u_soc_full_rst_sync (
        .src_arst (rst_req | dbg_full_rst_in),
        .dest_clk (clk_buf),
        .dest_arst(soc_full_rst_unbuf)
    );

    // ─────────────────────────────────────────────────────────────────────
    // Explicit BUFG on the broadcast outputs.
    //
    // The placer otherwise picks a regular fabric net for the broadcast,
    // notices the high fanout and inserts a BUFG late — at which point
    // the BUFG's ~2 ns clock-tree-equivalent delay shows up as data-path
    // delay on every receiving FF (because STA is unaware that the BUFG
    // is the broadcast driver, not a real clock).  Instantiating the
    // BUFG here makes Vivado treat the buffered net as a (false-path'd)
    // clock — receivers see balanced skew, not a data-path penalty.
    //
    // The bank ports drive ALL bits from the single BUFG output.  The
    // buses are kept for source-compatibility with consumers that
    // already index `*_bank[N]`; physical replication of the broadcast
    // is done by the global clock network downstream of the BUFG.
    // ─────────────────────────────────────────────────────────────────────
    wire core_rst_buf;
    wire soc_full_rst_buf;

    BUFG u_core_rst_bufg (
        .I(core_rst_unbuf),
        .O(core_rst_buf)
    );

    BUFG u_soc_full_rst_bufg (
        .I(soc_full_rst_unbuf),
        .O(soc_full_rst_buf)
    );

    assign rst_out          = core_rst_buf;
    assign soc_full_rst_out = soc_full_rst_buf;

    assign core_rst_bank     = {BANK_W{core_rst_buf}};
    assign soc_full_rst_bank = {BANK_W{soc_full_rst_buf}};
endmodule

`default_nettype wire
