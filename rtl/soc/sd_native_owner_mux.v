// Share one 100 MHz native SD block engine between ROM boot and runtime SCSI.
// The CPU remains reset until ROM loading completes, so ownership changes
// only after the boot request has ended. Latch the owner at request start so
// a delayed phase change cannot reroute an in-flight byte stream or done.
`default_nettype none
module sd_native_owner_mux (
    input  wire        clk,
    input  wire        rst,
    input  wire        boot_phase_complete,
    input  wire [2:0]  boot_cmd_type,
    input  wire [31:0] boot_lba,
    input  wire [15:0] boot_block_count,
    input  wire        boot_go,
    output wire        boot_busy,
    output wire        boot_done,
    output wire        boot_error,
    output wire        boot_rd_valid,
    output wire [7:0]  boot_rd_data,
    input  wire        boot_rd_ready,
    input  wire [2:0]  scsi_cmd_type,
    input  wire [31:0] scsi_lba,
    input  wire [15:0] scsi_block_count,
    input  wire        scsi_go,
    output wire        scsi_busy,
    output wire        scsi_done,
    output wire        scsi_error,
    output wire        scsi_rd_valid,
    output wire [7:0]  scsi_rd_data,
    input  wire        scsi_rd_ready,
    output wire        scsi_wr_ready,
    input  wire [7:0]  scsi_wr_data,
    input  wire        scsi_wr_avail,
    input  wire        pram_grant,
    input  wire [2:0]  pram_cmd_type,
    input  wire [31:0] pram_lba,
    input  wire [15:0] pram_block_count,
    input  wire        pram_go,
    output wire        pram_busy,
    output wire        pram_done,
    output wire        pram_error,
    output wire        pram_rd_valid,
    output wire [7:0]  pram_rd_data,
    input  wire        pram_rd_ready,
    output wire        pram_wr_ready,
    input  wire [7:0]  pram_wr_data,
    input  wire        pram_wr_avail,
    output wire [2:0]  card_cmd_type,
    output wire [31:0] card_lba,
    output wire [15:0] card_block_count,
    output wire        card_go,
    input  wire        card_busy,
    input  wire        card_done,
    input  wire        card_error,
    input  wire        card_rd_valid,
    input  wire [7:0]  card_rd_data,
    output wire        card_rd_ready,
    input  wire        card_wr_ready,
    output wire [7:0]  card_wr_data,
    output wire        card_wr_avail
);
    localparam [1:0] OWNER_BOOT = 2'd0, OWNER_SCSI = 2'd1,
                     OWNER_PRAM = 2'd2;
    reg [1:0] owner;
    reg boot_reject;
    reg scsi_reject;
    reg pram_reject;
    wire accept_boot = !boot_phase_complete && boot_go && !card_busy;
    wire accept_scsi = boot_phase_complete && !pram_grant && scsi_go && !card_busy;
    wire accept_pram = boot_phase_complete && pram_grant && pram_go && !card_busy;
    assign card_go = accept_boot || accept_scsi || accept_pram;
    assign card_cmd_type = !boot_phase_complete ? boot_cmd_type :
                           pram_grant ? pram_cmd_type : scsi_cmd_type;
    assign card_lba = !boot_phase_complete ? boot_lba :
                      pram_grant ? pram_lba : scsi_lba;
    assign card_block_count = !boot_phase_complete ? boot_block_count :
                              pram_grant ? pram_block_count : scsi_block_count;
    always @(posedge clk) begin
        if (rst) begin
            owner <= OWNER_BOOT;
            boot_reject <= 0;
            scsi_reject <= 0;
            pram_reject <= 0;
        end else begin
            boot_reject <= boot_go && !accept_boot;
            scsi_reject <= scsi_go && !accept_scsi;
            pram_reject <= pram_go && !accept_pram;
            if (card_go) owner <= accept_boot ? OWNER_BOOT :
                                  accept_pram ? OWNER_PRAM : OWNER_SCSI;
        end
    end
    assign boot_busy = card_busy && owner == OWNER_BOOT;
    assign boot_done = (card_done && owner == OWNER_BOOT) || boot_reject;
    assign boot_error = (card_done && owner == OWNER_BOOT && card_error) || boot_reject;
    assign boot_rd_valid = card_rd_valid && owner == OWNER_BOOT;
    assign boot_rd_data = card_rd_data;
    assign scsi_busy = card_busy && owner == OWNER_SCSI;
    assign scsi_done = (card_done && owner == OWNER_SCSI) || scsi_reject;
    assign scsi_error = (card_done && owner == OWNER_SCSI && card_error) || scsi_reject;
    assign scsi_rd_valid = card_rd_valid && owner == OWNER_SCSI;
    assign scsi_rd_data = card_rd_data;
    assign pram_busy = card_busy && owner == OWNER_PRAM;
    assign pram_done = (card_done && owner == OWNER_PRAM) || pram_reject;
    assign pram_error = (card_done && owner == OWNER_PRAM && card_error) || pram_reject;
    assign pram_rd_valid = card_rd_valid && owner == OWNER_PRAM;
    assign pram_rd_data = card_rd_data;
    assign card_rd_ready = owner == OWNER_PRAM ? pram_rd_ready :
                           owner == OWNER_SCSI ? scsi_rd_ready : boot_rd_ready;
    assign scsi_wr_ready = owner == OWNER_SCSI && card_wr_ready;
    assign pram_wr_ready = owner == OWNER_PRAM && card_wr_ready;
    assign card_wr_data = owner == OWNER_PRAM ? pram_wr_data :
                          owner == OWNER_SCSI ? scsi_wr_data : 8'b0;
    assign card_wr_avail = owner == OWNER_PRAM ? pram_wr_avail :
                           owner == OWNER_SCSI && scsi_wr_avail;
endmodule
`default_nettype wire
