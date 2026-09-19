// sd_scsi_lba_mapper.v — raw SD-sector mapper for SCSI disk traffic
//
// Purpose:
//   Defines the raw block backing-store contract used by the Mac SCSI disk:
//   SD sectors [0, RESERVED_LBAS) are reserved for ROM/boot/provisioning
//   assets, and SCSI disk LBA 0 starts at SD sector RESERVED_LBAS.  There is
//   no filesystem or partition parser in this path.
//
// Interface:
//   Combinational mapper from a SCSI disk LBA plus transfer block count to
//   a physical SD-sector LBA.  `valid` fails closed for zero-length requests,
//   SCSI-disk capacity overflow, or 32-bit mapped-LBA overflow.
//
// Latency:
//   Combinational.

module sd_scsi_lba_mapper #(
    parameter [31:0] RESERVED_LBAS = 32'd8192       // 4 MiB / 512 B
) (
    // Exposed disk size in sectors.  Runtime input, NOT a parameter: it is
    // derived from the SD card's CSD (see boot_fsm.card_num_lbas) minus the
    // reserved window, so a card swap changes it without a rebuild.
    input  wire [31:0] scsi_num_lbas,
    input  wire [31:0] scsi_lba,
    input  wire [23:0] scsi_blocks,
    output wire [31:0] sd_lba,
    output wire        valid
);

    wire [32:0] mapped_lba_wide;
    wire [33:0] end_lba_wide;
    wire [33:0] mapped_end_lba_wide;
    wire [33:0] disk_lbas_wide;

    assign mapped_lba_wide = {1'b0, scsi_lba} + {1'b0, RESERVED_LBAS};
    assign end_lba_wide    = {2'b0, scsi_lba} + {10'd0, scsi_blocks};
    assign mapped_end_lba_wide = end_lba_wide + {2'b0, RESERVED_LBAS};
    assign disk_lbas_wide  = {2'b0, scsi_num_lbas};

    assign sd_lba = mapped_lba_wide[31:0];
    assign valid  = (scsi_blocks != 24'd0) &&
                    !mapped_lba_wide[32] &&
                    (mapped_end_lba_wide <= 34'h1_0000_0000) &&
                    (end_lba_wide <= disk_lbas_wide);

endmodule
