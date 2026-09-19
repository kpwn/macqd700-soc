// sd_spi_mux.v — Three-way ownership mux for the shared SD SPI master
//
// The SD card is time-multiplexed between three potential masters:
//
//   Phase A (power-on): boot_fsm           — reads ROM sectors 0..8191
//                                             and writes DDR4 @ 0x4000_0000.
//   Phase B (running, select=0): sd_provision — XDMA host-driven, for
//                                             factory/provisioning flows.
//   Phase B (running, select=1): scsi_emu   — NCR 5380 disk emulation
//                                             (SCSI LUN 0, sectors 8192+).
//
// The handoff from A → B is one-way and is signalled externally via
// `boot_done`: once high, the boot_fsm's inputs to this mux are gated off
// and the phase-B owner drives the SPI master.
//
//   Phase B (running, pram_gnt=1): pram_sd — PRAM persistence sector
//                                             (SD LBA 8191), manual only.
//
// Phase-B arbitration is a single-bit select input `b_sel` (1 = SCSI, 0 =
// provision) provided by the top level.  An active provision CS/command
// forces provision ownership while a JTAG SD write is in flight; otherwise
// `b_sel` applies.
//
// `pram_gnt` is a FOURTH phase-B owner and is deliberately shaped
// differently from `prov_active`.  prov_active PREEMPTS: the moment the
// JTAG writer asserts CS it takes the bus out from under whatever SCSI
// was mid-way through.  That is acceptable for a provisioning bitstream
// where the Mac is not running and unacceptable on a live machine.  So
// pram_sd does not get a preempting hook — it raises a request, and the
// TOP LEVEL only raises pram_gnt on a cycle where the SCSI sd_ctrl is
// genuinely idle (rtl/soc/fpga_top_sd.vh).  Everything this mux does with
// pram_gnt is therefore already known not to interrupt a live transfer.
//
// VERIFIED 2026-08-09: the top-level condition really is
// `!core_scsi_busy && !sdj_writer_busy` sampled on the rising edge of the
// grant, so no in-flight sd_ctrl request can have the mux pulled out from
// under it.  But "our sd_ctrl is idle" is NOT "the CARD is idle": a
// CMD24/CMD25 leaves the card in a receive state that only a completed
// block + (for CMD25) a 0xFD stop-tran can leave.  sd_ctrl.v now closes
// that session on every error exit (its S_AB_* states); if you add
// another phase-B owner, that obligation is what makes handing the bus
// over safe — not this mux, and not the busy signal.
// Priority order within phase B: provision > pram > scsi.  Tie pram_gnt
// to 1'b0 (and the pram_* inputs off) in any build without pram_sd.
//
// Contract:
//   - Only the selected master sees non-zero cmd_ready / rsp_valid pulses.
//   - Unselected masters see cmd_ready=0 and rsp_valid=0 — they must wait
//     quietly until they are granted ownership.  The mux does NOT buffer
//     transactions on behalf of unselected masters.
//   - CS is driven by the selected master's cs_n_in.  When no master is
//     selected (transient), CS is forced high (inactive).
//   - fast_mode / hs_mode likewise come from the selected master.
//
// This is a pure combinational mux; no state, no arbitration fairness.

module sd_spi_mux (
    // ── Selection inputs ──────────────────────────────────────────────
    input  wire        boot_done,   // 1 once boot_fsm has finished & released
    input  wire        b_sel,       // 0 = provision, 1 = scsi_emu (phase B)
    input  wire        pram_gnt,    // 1 = pram_sd owns phase B (see header)

    // ── Phase-A: boot_fsm master ─────────────────────────────────────
    input  wire        boot_cmd_valid,
    output wire        boot_cmd_ready,
    input  wire [7:0]  boot_cmd_data,
    output wire        boot_rsp_valid,
    output wire [7:0]  boot_rsp_data,
    input  wire        boot_cs_n_in,
    input  wire        boot_fast_mode,
    input  wire        boot_hs_mode,

    // ── Phase-B option 0: sd_provision (XDMA-driven host port) ───────
    input  wire        prov_cmd_valid,
    output wire        prov_cmd_ready,
    input  wire [7:0]  prov_cmd_data,
    output wire        prov_rsp_valid,
    output wire [7:0]  prov_rsp_data,
    input  wire        prov_cs_n_in,
    input  wire        prov_fast_mode,
    input  wire        prov_hs_mode,

    // ── Phase-B option 1: scsi_emu (NCR 5380 emulation) ──────────────
    input  wire        scsi_cmd_valid,
    output wire        scsi_cmd_ready,
    input  wire [7:0]  scsi_cmd_data,
    output wire        scsi_rsp_valid,
    output wire [7:0]  scsi_rsp_data,
    input  wire        scsi_cs_n_in,
    input  wire        scsi_fast_mode,
    input  wire        scsi_hs_mode,

    // ── Phase-B option 2: pram_sd (PRAM persistence, manual JTAG only) ─
    input  wire        pram_cmd_valid,
    output wire        pram_cmd_ready,
    input  wire [7:0]  pram_cmd_data,
    output wire        pram_rsp_valid,
    output wire [7:0]  pram_rsp_data,
    input  wire        pram_cs_n_in,
    input  wire        pram_fast_mode,
    input  wire        pram_hs_mode,

    // ── Shared SPI master port (drives the single sd_spi instance) ───
    output wire        spi_cmd_valid,
    input  wire        spi_cmd_ready,
    output wire [7:0]  spi_cmd_data,
    input  wire        spi_rsp_valid,
    input  wire [7:0]  spi_rsp_data,
    output wire        spi_cs_n_in,
    output wire        spi_fast_mode,
    output wire        spi_hs_mode
);

    // Two-bit one-hot selector derived from boot_done + b_sel/prov_active.
    // 00 = boot, 01 = provision, 10 = scsi, 11 = unused (same as 10).
    wire prov_active = prov_cmd_valid | ~prov_cs_n_in;
    wire sel_boot = ~boot_done;
    wire sel_prov =  boot_done & (prov_active | ~b_sel);
    wire sel_pram =  boot_done & ~sel_prov & pram_gnt;
    wire sel_scsi =  boot_done & ~sel_prov & ~pram_gnt & b_sel;

    // ── Upstream → sd_spi ─────────────────────────────────────────────
    assign spi_cmd_valid = sel_boot ? boot_cmd_valid
                        : sel_prov ? prov_cmd_valid
                        : sel_pram ? pram_cmd_valid
                        : sel_scsi ? scsi_cmd_valid
                        : 1'b0;

    assign spi_cmd_data  = sel_boot ? boot_cmd_data
                        : sel_prov ? prov_cmd_data
                        : sel_pram ? pram_cmd_data
                        : sel_scsi ? scsi_cmd_data
                        : 8'h00;

    assign spi_cs_n_in   = sel_boot ? boot_cs_n_in
                        : sel_prov ? prov_cs_n_in
                        : sel_pram ? pram_cs_n_in
                        : sel_scsi ? scsi_cs_n_in
                        : 1'b1;                 // inactive

    assign spi_fast_mode = sel_boot ? boot_fast_mode
                        : sel_prov ? prov_fast_mode
                        : sel_pram ? pram_fast_mode
                        : sel_scsi ? scsi_fast_mode
                        : 1'b0;

    assign spi_hs_mode   = sel_boot ? boot_hs_mode
                        : sel_prov ? prov_hs_mode
                        : sel_pram ? pram_hs_mode
                        : sel_scsi ? scsi_hs_mode
                        : 1'b0;

    // ── sd_spi → downstream (only the selected owner sees pulses) ────
    assign boot_cmd_ready = sel_boot & spi_cmd_ready;
    assign boot_rsp_valid = sel_boot & spi_rsp_valid;
    assign boot_rsp_data  = spi_rsp_data;

    assign prov_cmd_ready = sel_prov & spi_cmd_ready;
    assign prov_rsp_valid = sel_prov & spi_rsp_valid;
    assign prov_rsp_data  = spi_rsp_data;

    assign pram_cmd_ready = sel_pram & spi_cmd_ready;
    assign pram_rsp_valid = sel_pram & spi_rsp_valid;
    assign pram_rsp_data  = spi_rsp_data;

    assign scsi_cmd_ready = sel_scsi & spi_cmd_ready;
    assign scsi_rsp_valid = sel_scsi & spi_rsp_valid;
    assign scsi_rsp_data  = spi_rsp_data;

endmodule
