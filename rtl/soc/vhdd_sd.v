// vhdd_sd.v — SD-card provider for the virtual-HDD contract (rtl/vhdd.vh)
//
// Role
// ════
// Presents an SD card as a plain 512-byte-block volume.  Everything
// SD-shaped about the Mac's virtual hard disk lives here or below:
//
//   scsi.v ──vhdd──► vhdd_sd ──► sd_scsi_bridge ──► sd_ctrl ──► sd_spi
//                    (this)      (pb↔core CDC)     (commands)   (SPI)
//
// Two jobs, and nothing else:
//
//  1. Request encoding.  The contract says {req_write, req_multi};
//     sd_ctrl wants a raw SD command class:
//
//        req_write  req_multi   sd_cmd_type
//            0          0       1  CMD17  single-block read
//            0          1       2  CMD18  multi-block read
//            1          0       3  CMD24  single-block write
//            1          1       4  CMD25  multi-block write
//
//  2. The reserved boot window.  SD sectors [0, RESERVED_LBAS) hold the
//     ROM / boot / provisioning assets that boot_fsm reads before the
//     CPU is released; the Mac's disk starts above them, so volume LBA 0
//     is SD sector RESERVED_LBAS.  This bias, and the addressability
//     check that goes with it, are properties of *this* backing store —
//     they are meaningless for, say, a DDR-backed volume, which is why
//     they are here and not in scsi.v.
//
//     The platform must report a capacity that already excludes the
//     window (num_lbas = card_sectors - RESERVED_LBAS, clamped).  See
//     rtl/soc/fpga_top_peripherals.vh, which derives both that clamp and
//     this module's RESERVED_LBAS from one localparam so the two cannot
//     drift.
//
// Everything else on the interface is a straight pass-through: the byte
// streams, the back-pressure, and the busy/done/error handshake all mean
// exactly what sd_ctrl already means by them.
//
// Latency
// ═══════
// Combinational — no clock, no state.  Deliberate: it must not add a
// cycle anywhere on the request or data path, because the master's
// timing (notably "req_lba is valid on the same cycle req_go is high")
// is the pre-refactor timing of the SD path and has to stay that way.
// A future provider that needs state gets its own clock port; this one
// does not need one and does not take one.

`default_nettype none

`include "vhdd.vh"

module vhdd_sd #(
    // SD sectors reserved below the volume for ROM / boot assets.
    // 8192 sectors = 4 MiB.
    parameter [31:0] RESERVED_LBAS = 32'd8192
) (
    // ── vhdd master side (contract in rtl/vhdd.vh) ────────────────────
    input  wire [31:0] num_lbas,

    input  wire [31:0] chk_lba,
    input  wire [23:0] chk_blocks,
    output wire        chk_ok,

    input  wire        req_write,
    input  wire        req_multi,
    input  wire [31:0] req_lba,
    input  wire [15:0] req_block_count,
    input  wire        req_go,

    output wire        busy,
    output wire        done,
    output wire        error,

    output wire        rd_valid,
    output wire [7:0]  rd_data,
    input  wire        rd_ready,

    output wire        wr_ready,
    input  wire        wr_valid,
    input  wire [7:0]  wr_data,
    input  wire        wr_avail,

    // ── SD transport side (sd_ctrl-shaped; via sd_scsi_bridge) ────────
    output wire [2:0]  sd_cmd_type,
    output wire [31:0] sd_lba,
    output wire [15:0] sd_block_count,
    output wire        sd_go,
    input  wire        sd_busy,
    input  wire        sd_done,
    input  wire        sd_error,
    input  wire        sd_rd_valid,
    input  wire [7:0]  sd_rd_data,
    output wire        sd_rd_ready,
    input  wire        sd_wr_ready,
    output wire        sd_wr_valid,
    output wire [7:0]  sd_wr_data,
    output wire        sd_wr_avail
);

    // ── Request encoding ──────────────────────────────────────────────
    localparam [2:0] CT_CMD17 = 3'd1,   // single-block read
                     CT_CMD18 = 3'd2,   // multi-block read
                     CT_CMD24 = 3'd3,   // single-block write
                     CT_CMD25 = 3'd4;   // multi-block write

    assign sd_cmd_type = req_write ? (req_multi ? CT_CMD25 : CT_CMD24)
                                   : (req_multi ? CT_CMD18 : CT_CMD17);

    // NOTE: sd_cmd_type is now a pure function of the request flags, so
    // while no request is outstanding it reads CMD17 rather than the 0
    // ("none") that scsi.v used to park it at.  Nothing downstream can
    // see the difference: sd_scsi_bridge latches cmd_type/lba/block_count
    // only on `pb_go && !pb_busy_internal`, and sd_ctrl only ever samples
    // them behind that latch.  The generic contract has no "no request"
    // encoding by design — `req_go` is what says a request exists.

    assign sd_block_count = req_block_count;
    assign sd_go          = req_go;

    // ── Reserved-window bias ──────────────────────────────────────────
    // Volume LBA 0 lives at SD sector RESERVED_LBAS.  req_lba is already
    // held stable by the master from the cycle req_go is high, so this
    // stays combinational and sd_lba is valid on the same cycle sd_go is.
    assign sd_lba = req_lba + RESERVED_LBAS;

    // ── Addressability probe ──────────────────────────────────────────
    // Same mapper the SCSI target used to instantiate internally, moved
    // down to the layer the reservation actually belongs to.  Its sd_lba
    // output is unused here (the request path above computes the bias for
    // the latched request instead of the probed extent); only `valid` is
    // consumed.
    /* verilator lint_off UNUSEDSIGNAL */
    wire [31:0] chk_lba_phys_unused;
    /* verilator lint_on UNUSEDSIGNAL */

    sd_scsi_lba_mapper #(
        .RESERVED_LBAS(RESERVED_LBAS)
    ) u_lba_mapper (
        .scsi_num_lbas(num_lbas),
        .scsi_lba     (chk_lba),
        .scsi_blocks  (chk_blocks),
        .sd_lba       (chk_lba_phys_unused),
        .valid        (chk_ok)
    );

    // ── Straight pass-through ─────────────────────────────────────────
    assign busy        = sd_busy;
    assign done        = sd_done;
    assign error       = sd_error;

    assign rd_valid    = sd_rd_valid;
    assign rd_data     = sd_rd_data;
    assign sd_rd_ready = rd_ready;

    assign wr_ready    = sd_wr_ready;
    assign sd_wr_valid = wr_valid;
    assign sd_wr_data  = wr_data;
    assign sd_wr_avail = wr_avail;

endmodule

`default_nettype wire
