// pram_sd.v — manual, JTAG-driven persistence of the Mac's PRAM to SD.
//
// ══════════════════════════════════════════════════════════════════════
// WHAT THIS IS (and, just as importantly, what it is NOT)
// ══════════════════════════════════════════════════════════════════════
//   rtl/mac/rtc.v models the Quadra 700's battery-backed 256-byte
//   parameter RAM.  It already survives `rst` (commit 876de3c), so user
//   settings live across a warm reset — but not across a power cycle or a
//   bitstream reload, because the array's power-on image is a
//   configuration-time `initial` (all-zero under SYNTHESIS).
//
//   This module gives the operator two EXPLICIT, MANUAL operations:
//     * SAVE — snapshot the live PRAM into one reserved SD sector.
//     * LOAD — read that sector back and install it into the live PRAM.
//
//   It is NOT a write-back cache and NOT an autosave.  There is no
//   dirty-tracking, no flush-on-reset, no periodic sync, and above all no
//   path by which the Mac itself can cause an SD write: the ONLY thing
//   that starts a transfer is a host write of a magic word to CTRL over
//   the JTAG-AXI debug aperture.  The 68k has no access to this register
//   file at all — it lives behind AXI_SD_JTAG_BASE, outside the Mac's
//   address map.  Keep it that way; "PRAM writes now hit the SD card" is
//   a card-wear and boot-latency disaster, not a feature.
//
// ══════════════════════════════════════════════════════════════════════
// ON-CARD FORMAT — one 512-byte sector, all multi-byte fields BIG-ENDIAN
// ══════════════════════════════════════════════════════════════════════
//   off    size  field
//   0x000    4   MAGIC    'P','R','M','1'  (0x50 0x52 0x4D 0x31)
//   0x004    2   VERSION  0x0001
//   0x006    2   LENGTH   0x0100  (256 payload bytes)
//   0x008    2   CKSUM    Fletcher-16, see below  (0x008 = B, 0x009 = A)
//   0x00A    6   reserved, written as zero (NOT validated — future use)
//   0x010  256   PRAM bytes 0x00..0xFF, in order
//   0x110  240   zero padding (written as zero, NOT validated)
//
//   CKSUM is Fletcher-16 (mod-256 variant) over the concatenation of
//   bytes 0x000..0x007 and 0x010..0x10F — i.e. magic+version+length and
//   the whole payload, skipping the checksum field itself and the
//   reserved bytes:
//
//       A = 0x0A ; B = 0x5D                 // NON-ZERO seeds, see below
//       for each byte b:  A = (A + b) & 0xFF ;  B = (B + A) & 0xFF
//       CKSUM = (B << 8) | A                // B at 0x008, A at 0x009
//
//   The seeds are deliberately non-zero.  With zero seeds an ALL-ZERO
//   sector — exactly what a blank card, or a card region that was never
//   written, reads back as — produces CKSUM 0x0000, which then MATCHES
//   the all-zero stored field.  A blank sector would have PASSED the
//   checksum.  Non-zero seeds make a zero payload compute to something
//   non-zero against a stored 0x0000, so blank fails the checksum on top
//   of already failing the magic.  Two independent rejections, by design.
//
//   Fletcher rather than a plain additive sum because the payload is 256
//   bytes of small-valued settings where byte transposition is a
//   realistic corruption mode and a plain sum is blind to it.
//
// ══════════════════════════════════════════════════════════════════════
// LOAD FAILURE POLICY — fall back to the POST-RESET DEFAULTS, loudly
// ══════════════════════════════════════════════════════════════════════
//   A LOAD that cannot validate the sector (blank / bad magic / bad
//   version / bad length / bad checksum / SD error) does NOT install the
//   bytes and does NOT leave the array holding whatever was in it.  It
//   asserts `pram_default`, which drives rtc.v's EXISTING `pram_clear`
//   input — the same signal the JTAG `pram-clear` (Cmd-Opt-P-R) command
//   uses.  That is deliberate: there is exactly ONE definition of "the
//   post-reset PRAM contents" in this design and it lives in rtc.v's
//   pram_reset_value().  This module does not carry a second copy that
//   could drift out of sync with it.
//
//   NOTE on what those defaults actually are: rtc.v's pram_reset_value()
//   returns 8'h00 for every address under `SYNTHESIS, and the populated
//   "SCBI" image is a SIM-ONLY opt-in (absent +rtc_populated_pram it is
//   also all-zero).  So on real hardware a failed load lands the array on
//   an all-zero PRAM, which is exactly what a cold bitstream load gives.
//   Whatever that path is, it is the SAME path — do not re-derive it here.
//
//   The failure is always visible: STATUS[7:4] carries a result code
//   naming WHICH check failed, STATUS[3] latches "defaults were applied",
//   and tools/jtag_repl.tcl's `pram-load` prints both.  A silent fallback
//   would be indistinguishable from a successful restore of an all-default
//   image, which is precisely the class of instrument-that-measures-
//   nothing this project keeps getting bitten by.
//
// ══════════════════════════════════════════════════════════════════════
// SD BUS ARBITRATION — never preempt an in-flight SCSI transfer
// ══════════════════════════════════════════════════════════════════════
//   The SD SPI master is shared (see rtl/board/sd_spi_mux.v).  The
//   pre-existing sd_jtag_writer path takes the bus by simply asserting CS
//   (`prov_active`), which PREEMPTS whatever SCSI was doing — fine for a
//   provisioning bitstream where the Mac is not running, catastrophic on
//   a live machine mid-boot.
//
//   This module does not do that.  It raises `sd_req` and waits for
//   `sd_gnt`, which the top level only asserts on a cycle where the SCSI
//   sd_ctrl is idle (see rtl/soc/fpga_top_sd.vh).  Once granted, the
//   grant is held until we drop the request, so our own transfer cannot
//   be preempted either.
//
//   The residual hazard, stated plainly rather than hidden: a SCSI
//   command issued by the Mac AFTER we take the grant finds the SPI mux
//   pointed elsewhere and times out on sd_ctrl's own global request
//   watchdog (~10 s, commit 9786476).  That surfaces as a SCSI error the
//   OS retries — not data corruption, but a real stall.  The documented
//   operating restriction is therefore: halt the CPU around `pram-save` /
//   `pram-load`.  tools/jtag_repl.tcl checks halt status and warns loudly
//   when it is not halted.
//
// ══════════════════════════════════════════════════════════════════════
// BOUNDED RESPONSE
// ══════════════════════════════════════════════════════════════════════
//   Every wait in this FSM has a counter behind it and every counter
//   terminates the operation with a result code:
//     * grant wait          -> ARB_WAIT_LOG2 cycles  -> RES_ARB_TIMEOUT
//     * PRAM CDC handshake  -> CDC_WAIT_LOG2 cycles  -> RES_CDC_TIMEOUT
//     * SD transfer         -> sd_ctrl's own unconditional global request
//                              watchdog -> `error` -> RES_SD_ERR
//   There is no state in which `busy` can stay high forever.  Do not add
//   one.  The SD path in this SoC has already deadlocked a four-way cycle
//   once (SCSI pseudo-DMA <-> CPU <-> sd_ctrl) that only a CPU write
//   could break, with the CPU frozen — that is why sd_ctrl grew an
//   unconditional watchdog and why this module has three.
//
// ══════════════════════════════════════════════════════════════════════
// REGISTER MAP (byte offsets within this module's 256-byte aperture)
// ══════════════════════════════════════════════════════════════════════
//   0x00 IDENT    R   0x50524D31 ('PRM1') — presence probe
//   0x04 CTRL     W   command word, see CMD_* below.  Ignored unless idle.
//   0x08 STATUS   R   [0]     busy
//                     [1]     done (sticky until the next command)
//                     [2]     error (done && result != RES_OK)
//                     [3]     defaults_applied (this op fell back)
//                     [7:4]   result code, RES_*
//                     [11:8]  sd_ctrl err_cause of the last transfer
//                     [12]    sd_gnt   (live)
//                     [13]    sd_req   (live)
//                     [14]    boot_done(live)
//                     [31:16] checksum computed by the last walk
//   0x0C BUFPTR   R/W 0..511 byte pointer into the staging sector buffer
//   0x10 BUFDATA  R   staging byte at BUFPTR; reading auto-increments
//   0x14 LBA      R   the compiled-in PRAM sector LBA (host sanity check)
//   0x18 CKREF    R   [15:0] checksum STORED in the sector by the last
//                     LOAD (0 after SAVE/SNAP) — pairs with STATUS[31:16]
//                     so a mismatch report can print both numbers.
//
// Verilog-2005, synchronous active-high reset.

`default_nettype none

module pram_sd #(
    // Reserved "system persistence" block: SD LBA 8176..8191 (16 sectors,
    // 8 KiB) at the TOP of the 4 MiB reserved window, immediately below
    // the HDD image at LBA 8192.  PRAM takes the last sector, 8191.
    //
    // Anchored at the TOP on purpose.  BOOT_ROM_SECTORS is a `?=` make
    // knob (Makefile:2006, default 2048) and the ROM grows UPWARD from
    // LBA 0, so anything placed just above the ROM would be silently
    // overrun by a larger ROM image.  8191 is independent of ROM size and
    // is bounded above by the equally fixed SD_RESERVED_LBAS = 8192 that
    // boot_fsm.v, vhdd_sd.v, sd_ctrl.v and sd_scsi_lba_mapper.v all agree
    // on.  Sectors 8176..8190 are left free for the next persistent thing
    // so it does not need another layout change.
    parameter [31:0] PRAM_LBA          = 32'd8191,
    parameter [31:0] SYS_PERSIST_BASE  = 32'd8176,   // documentation only
    // Bounded-response budgets, as log2(core_clk cycles).  Overridden
    // small by tb/tb_pram_sd_top.v so the timeout MECHANISM is provable
    // in a short simulation instead of the ~2.7 s the shipping grant
    // budget would cost.
    parameter integer ARB_WAIT_LOG2     = 28,
    parameter integer CDC_WAIT_LOG2     = 16,
    // How long `pram_default` is held. Must cover the 256-pb-clock PRAM
    // sweep plus the 2-FF clear synchronizer before reporting completion
    // or releasing boot autoload. 2^11 core clocks cover 1024 pb clocks
    // at 100 MHz core / 50 MHz pb, or 512 at 200 MHz core / 50 MHz pb.
    // Reduced simulation overrides must preserve this minimum duration.
    parameter integer DEFAULT_HOLD_LOG2 = 11,
    // Opt-in: self-issue one LOAD when boot_done first rises, so a cold
    // boot installs the saved PRAM before the 68k can read it.  Default 0
    // keeps the module strictly manual for every existing instance and tb.
    // This adds no WRITE path -- the module's contract that nothing but an
    // explicit host command can touch the card for a SAVE is unchanged.
    parameter integer AUTOLOAD_ON_BOOT  = 0
) (
    input  wire        clk,
    input  wire        rst,
    // High once boot_fsm has initialised the card and released the SPI
    // bus.  SD-touching commands are refused before this.
    input  wire        boot_done,
    // High from reset until the boot-time LOAD has resolved (or settled as
    // "not applicable").  fpga_top holds the CPU on this so the Mac cannot
    // read PRAM before it is installed -- boot_rom_loaded, which releases
    // the CPU, is the very signal that starts the load.  Tied low when
    // AUTOLOAD_ON_BOOT = 0.
    output wire        autoload_pending,

    // ── AXI-Lite slave (JTAG-AXI debug aperture) ──────────────────────
    input  wire [19:0] s_awaddr,
    input  wire        s_awvalid,
    output wire        s_awready,
    input  wire [31:0] s_wdata,
    input  wire [3:0]  s_wstrb,
    input  wire        s_wvalid,
    output wire        s_wready,
    output wire [1:0]  s_bresp,
    output wire        s_bvalid,
    input  wire        s_bready,
    input  wire [19:0] s_araddr,
    input  wire        s_arvalid,
    output wire        s_arready,
    output wire [31:0] s_rdata,
    output wire [1:0]  s_rresp,
    output wire        s_rvalid,
    input  wire        s_rready,

    // ── SPI byte interface to sd_spi_mux's pram_* port ────────────────
    output wire        spi_cmd_valid,
    input  wire        spi_cmd_ready,
    output wire [7:0]  spi_cmd_data,
    input  wire        spi_rsp_valid,
    input  wire [7:0]  spi_rsp_data,
    output wire        spi_cs_n_in,
    output wire        spi_fast_mode,
    output wire        spi_hs_mode,

    // ── SD bus arbitration (see header) ───────────────────────────────
    output wire        sd_req,
    input  wire        sd_gnt,

    // ── PRAM access, A side of pram_cdc.v ─────────────────────────────
    output reg         pram_req,
    output reg         pram_we,
    output reg  [7:0]  pram_addr,
    output reg  [7:0]  pram_wdata,
    input  wire [7:0]  pram_rdata,
    input  wire        pram_ack,

    // ── "apply post-reset defaults" — drives rtc.v's pram_clear ───────
    output reg         pram_default,

    output wire        pram_sd_busy
);

    // ── sd_ctrl command encodings ─────────────────────────────────────
    localparam [2:0] CT_CMD17 = 3'd1;   // READ_SINGLE_BLOCK
    localparam [2:0] CT_CMD24 = 3'd3;   // WRITE_SINGLE_BLOCK

    // ── Register offsets ──────────────────────────────────────────────
    localparam [5:0]
        REG_CTRL    = 6'h04,
        REG_STATUS  = 6'h08,
        REG_BUFPTR  = 6'h0C,
        REG_BUFDATA = 6'h10,
        REG_LBA     = 6'h14,
        REG_CKREF   = 6'h18;

    localparam [31:0] IDENT_VALUE = 32'h5052_4D31;  // 'PRM1'
    // Boot-time LOAD one-shot.  `autoload_armed` falls on the first attempt
    // so a failed load is never retried in a loop; `autoload_busy` tracks the
    // operation the module started for us, so `pending` can drop the instant
    // it resolves -- including the loud fall-back-to-defaults path, which is
    // a legitimate outcome, not a reason to keep holding the CPU.
    reg autoload_armed_q, autoload_busy_q, boot_done_q;
    wire autoload_en = (AUTOLOAD_ON_BOOT != 0);
    assign autoload_pending = autoload_en && (autoload_armed_q || autoload_busy_q);

    localparam [31:0] CMD_SAVE    = 32'h5052_0001;
    localparam [31:0] CMD_LOAD    = 32'h5052_0002;
    localparam [31:0] CMD_SNAP    = 32'h5052_0003;

    // ── Result codes (STATUS[7:4]) ────────────────────────────────────
    localparam [3:0]
        RES_OK          = 4'd0,
        RES_BAD_MAGIC   = 4'd1,
        RES_BAD_VERSION = 4'd2,
        RES_BAD_LENGTH  = 4'd3,
        RES_BAD_CKSUM   = 4'd4,
        RES_BLANK       = 4'd5,
        RES_SD_ERR      = 4'd6,
        RES_ARB_TIMEOUT = 4'd7,
        RES_NOT_BOOTED  = 4'd8,
        RES_CDC_TIMEOUT = 4'd9,
        RES_BAD_CMD     = 4'd10;

    // ── On-card header constants ──────────────────────────────────────
    localparam [31:0] HDR_MAGIC   = 32'h5052_4D31;  // 'PRM1'
    localparam [15:0] HDR_VERSION = 16'h0001;
    localparam [15:0] HDR_LENGTH  = 16'h0100;       // 256 payload bytes
    localparam [7:0]  CK_SEED_A   = 8'h0A;
    localparam [7:0]  CK_SEED_B   = 8'h5D;
    localparam [8:0]  PAYLOAD_OFF = 9'd16;          // payload at 0x010
    localparam [9:0]  WALK_END    = 10'd272;        // walk covers 0..271

    // ── Operation select ──────────────────────────────────────────────
    localparam [1:0] OP_SAVE = 2'd0, OP_LOAD = 2'd1, OP_SNAP = 2'd2;

    // ── FSM states ────────────────────────────────────────────────────
    localparam [4:0]
        S_IDLE      = 5'd0,
        S_FILL      = 5'd1,   // header + zero pad into the staging buffer
        S_SNAP_REQ  = 5'd2,   // live PRAM -> staging buffer, byte at a time
        S_SNAP_ACK  = 5'd3,
        S_SNAP_REL  = 5'd4,
        S_WALK_A    = 5'd5,   // set staging-buffer read address
        S_WALK_L    = 5'd6,   // one dead cycle: registered array read
        S_WALK_B    = 5'd7,   // consume: header capture + checksum fold
        S_CKST_HI   = 5'd8,   // SAVE/SNAP: park the checksum in the header
        S_CKST_LO   = 5'd9,
        S_ARB       = 5'd10,
        S_SD_GO     = 5'd11,
        S_SD_WAIT   = 5'd12,
        S_SD_REL    = 5'd13,
        S_VERDICT   = 5'd14,
        S_INST_REQ  = 5'd15,  // staging buffer -> live PRAM
        S_INST_L    = 5'd16,  // one dead cycle: registered array read
        S_INST_ACK  = 5'd17,
        S_INST_REL  = 5'd18,
        S_DEFAULT   = 5'd19,
        S_DONE      = 5'd20;

    reg [4:0]  state;
    reg [1:0]  op_q;

    // ── Staging sector buffer ─────────────────────────────────────────
    //
    // Deliberately NOT reset.  A reset loop over 512 entries forces the
    // array into flops (and is a large part of why sd_jtag_writer.v costs
    // ~17.5K LUTs); there is also nothing to protect against, because
    // every operation writes all 512 bytes before anything reads them:
    // SAVE/SNAP via S_FILL + S_SNAP_*, LOAD via the CMD17 sink.  The one
    // read that can precede any write is a host BUFDATA read before the
    // first command, which is a debug convenience and reads whatever the
    // BRAM powered up as.
    (* ram_style = "block" *) reg [7:0] sec_buf [0:511];
    reg [8:0]  buf_waddr;
    reg [7:0]  buf_wdata;
    reg        buf_we;
    reg [8:0]  buf_raddr;
    reg [7:0]  buf_rdata;
    // Second, independent synchronous read port, used ONLY to keep the
    // CMD24 byte stream fed.  Same idea as sd_jtag_writer.v (which is
    // hardware-proven): sd_ctrl's wr_ready pulses are >= 16 core clocks
    // apart (one SPI byte at HS rates), so one cycle of array latency is
    // never on the critical path.
    reg [8:0]  wr_raddr;
    reg [7:0]  wr_byte_q;

    // Both read ports and the write port live in ONE always block so the
    // array stays inferrable as a single dual-port BRAM.
    always @(posedge clk) begin
        if (buf_we) sec_buf[buf_waddr] <= buf_wdata;
        buf_rdata <= sec_buf[buf_raddr];
        wr_byte_q <= sec_buf[wr_raddr];
    end

    // ── AXI-Lite plumbing (same shape as sd_jtag_writer.v) ────────────
    reg        aw_pending_q;
    reg [19:0] awaddr_q;
    reg        w_pending_q;
    reg [31:0] wdata_q;
    reg        bvalid_q;
    reg        rvalid_q;
    reg [31:0] rdata_q;
    reg [8:0]  bufptr_q;

    wire [5:0] wr_reg = awaddr_q[5:0] & 6'h3C;
    wire [5:0] rd_reg = s_araddr[5:0] & 6'h3C;

    assign s_awready = !aw_pending_q && !bvalid_q;
    assign s_wready  = !w_pending_q && !bvalid_q;
    assign s_bresp   = 2'b00;
    assign s_bvalid  = bvalid_q;
    assign s_arready = !rvalid_q;
    assign s_rdata   = rdata_q;
    assign s_rresp   = 2'b00;
    assign s_rvalid  = rvalid_q;

    // ── Status ────────────────────────────────────────────────────────
    reg        busy_q;
    reg        done_sticky_q;
    reg [3:0]  result_q;
    reg        defaults_q;
    reg [3:0]  sd_err_cause_q;
    reg [15:0] ck_calc_q;      // checksum computed by the last walk
    reg [15:0] ck_ref_q;       // checksum found in the sector (LOAD only)

    assign pram_sd_busy = busy_q;

    wire [31:0] status_value = {
        ck_calc_q,                               // [31:16]
        1'b0,                                    // [15]
        boot_done,                               // [14]
        sd_req,                                  // [13]
        sd_gnt,                                  // [12]
        sd_err_cause_q,                          // [11:8]
        result_q,                                // [7:4]
        defaults_q,                              // [3]
        done_sticky_q && (result_q != RES_OK),   // [2] error
        done_sticky_q,                           // [1]
        busy_q                                   // [0]
    };

    // ── Walk / snapshot / install counters ────────────────────────────
    reg [9:0]  idx_q;
    reg [7:0]  ck_a_q, ck_b_q;
    reg [79:0] hdr_q;          // first 10 header bytes, MSB-first
    reg        hdr_nonzero_q;

    // Verdict on the sector the last LOAD read back.  Evaluated in
    // S_VERDICT, where hdr_q / ck_*_q are settled.
    wire [15:0] ck_walk    = {ck_b_q, ck_a_q};
    wire        v_blank    = !hdr_nonzero_q;
    wire        v_magic    = (hdr_q[79:48] != HDR_MAGIC);
    wire        v_version  = (hdr_q[47:32] != HDR_VERSION);
    wire        v_length   = (hdr_q[31:16] != HDR_LENGTH);
    wire        v_cksum    = (hdr_q[15:0]  != ck_walk);
    wire        v_bad      = v_blank | v_magic | v_version | v_length | v_cksum;
    wire [3:0]  v_result   = v_blank   ? RES_BLANK
                           : v_magic   ? RES_BAD_MAGIC
                           : v_version ? RES_BAD_VERSION
                           : v_length  ? RES_BAD_LENGTH
                           : v_cksum   ? RES_BAD_CKSUM
                           :             RES_OK;

    // ── Timeout counters ──────────────────────────────────────────────
    reg [ARB_WAIT_LOG2-1:0]     arb_cnt_q;
    reg [CDC_WAIT_LOG2-1:0]     cdc_cnt_q;
    reg [DEFAULT_HOLD_LOG2-1:0] def_cnt_q;

    // ── sd_ctrl handshake ─────────────────────────────────────────────
    reg        ctrl_go_q;
    reg [2:0]  ctrl_cmd_q;
    wire       ctrl_busy;
    wire       ctrl_done;
    wire       ctrl_error;
    wire [3:0] ctrl_err_cause;
    wire       ctrl_rd_valid;
    wire [7:0] ctrl_rd_data;
    wire       ctrl_wr_ready;

    reg        sd_req_q;
    assign sd_req = sd_req_q;
    // CS is caller-owned; sd_spi_mux only routes it to the pin while we
    // hold the grant, so driving it from the request level is safe.
    assign spi_cs_n_in   = !sd_req_q;
    assign spi_fast_mode = 1'b1;
    assign spi_hs_mode   = 1'b1;

    // Header byte for staging-buffer offset `i` (0..15).  Offsets 8/9 are
    // the checksum and are patched in later by S_CKST_*; 0x0A..0x0F are
    // reserved and written as zero.
    function [7:0] hdr_byte(input [8:0] i);
        case (i[3:0])
            4'd0: hdr_byte = HDR_MAGIC[31:24];
            4'd1: hdr_byte = HDR_MAGIC[23:16];
            4'd2: hdr_byte = HDR_MAGIC[15:8];
            4'd3: hdr_byte = HDR_MAGIC[7:0];
            4'd4: hdr_byte = HDR_VERSION[15:8];
            4'd5: hdr_byte = HDR_VERSION[7:0];
            4'd6: hdr_byte = HDR_LENGTH[15:8];
            4'd7: hdr_byte = HDR_LENGTH[7:0];
            default: hdr_byte = 8'h00;
        endcase
    endfunction

    // Bytes covered by the checksum: 0x000..0x007 and 0x010..0x10F.
    function checksummed(input [9:0] i);
        checksummed = (i <= 10'd7) || (i >= 10'd16);
    endfunction

    always @(posedge clk) begin
        if (rst) begin
            state          <= S_IDLE;
            op_q           <= OP_SAVE;
            autoload_armed_q <= (AUTOLOAD_ON_BOOT != 0);
            autoload_busy_q  <= 1'b0;
            boot_done_q      <= 1'b0;
            aw_pending_q   <= 1'b0;
            awaddr_q       <= 20'd0;
            w_pending_q    <= 1'b0;
            wdata_q        <= 32'd0;
            bvalid_q       <= 1'b0;
            rvalid_q       <= 1'b0;
            rdata_q        <= 32'd0;
            bufptr_q       <= 9'd0;
            busy_q         <= 1'b0;
            done_sticky_q  <= 1'b0;
            result_q       <= RES_OK;
            defaults_q     <= 1'b0;
            sd_err_cause_q <= 4'd0;
            ck_calc_q      <= 16'd0;
            ck_ref_q       <= 16'd0;
            idx_q          <= 10'd0;
            ck_a_q         <= CK_SEED_A;
            ck_b_q         <= CK_SEED_B;
            hdr_q          <= 80'd0;
            hdr_nonzero_q  <= 1'b0;
            arb_cnt_q      <= {ARB_WAIT_LOG2{1'b0}};
            cdc_cnt_q      <= {CDC_WAIT_LOG2{1'b0}};
            def_cnt_q      <= {DEFAULT_HOLD_LOG2{1'b0}};
            ctrl_go_q      <= 1'b0;
            ctrl_cmd_q     <= CT_CMD17;
            sd_req_q       <= 1'b0;
            pram_req       <= 1'b0;
            pram_we        <= 1'b0;
            pram_addr      <= 8'd0;
            pram_wdata     <= 8'd0;
            pram_default   <= 1'b0;
            buf_we         <= 1'b0;
            buf_waddr      <= 9'd0;
            buf_wdata      <= 8'd0;
            buf_raddr      <= 9'd0;
            wr_raddr       <= 9'd0;
        end else begin
            boot_done_q <= boot_done;
            // Self-issue exactly one LOAD the first time the card is ready.
            // Same entry the host's CMD_LOAD takes, so it inherits the whole
            // arbitration/timeout/fallback path rather than duplicating it.
            if (autoload_en && autoload_armed_q && boot_done && !boot_done_q &&
                state == S_IDLE) begin
                autoload_armed_q <= 1'b0;
                autoload_busy_q  <= 1'b1;
                done_sticky_q    <= 1'b0;
                defaults_q       <= 1'b0;
                sd_err_cause_q   <= 4'd0;
                result_q         <= RES_OK;
                ck_calc_q        <= 16'd0;
                ck_ref_q         <= 16'd0;
                op_q             <= OP_LOAD;
                busy_q           <= 1'b1;
                arb_cnt_q        <= {ARB_WAIT_LOG2{1'b0}};
                state            <= S_ARB;
            end
            // Whatever the verdict -- installed, or defaults applied loudly --
            // the boot hold ends when the operation resolves.
            if (autoload_busy_q && state == S_DONE) autoload_busy_q <= 1'b0;
            ctrl_go_q <= 1'b0;
            buf_we    <= 1'b0;

            // ── AXI-Lite write channel ────────────────────────────────
            if (s_awvalid && s_awready) begin
                aw_pending_q <= 1'b1;
                awaddr_q     <= s_awaddr;
            end
            if (s_wvalid && s_wready) begin
                w_pending_q <= 1'b1;
                wdata_q     <= s_wdata;
            end

            if (aw_pending_q && w_pending_q && !bvalid_q) begin
                aw_pending_q <= 1'b0;
                w_pending_q  <= 1'b0;
                bvalid_q     <= 1'b1;
                case (wr_reg)
                    REG_BUFPTR: bufptr_q <= wdata_q[8:0];
                    REG_CTRL: begin
                        if (state == S_IDLE) begin
                            // Every accepted command clears the previous
                            // verdict, so a stale "done" can never be
                            // mistaken for this operation's result.
                            done_sticky_q  <= 1'b0;
                            defaults_q     <= 1'b0;
                            sd_err_cause_q <= 4'd0;
                            result_q       <= RES_OK;
                            ck_calc_q      <= 16'd0;
                            ck_ref_q       <= 16'd0;
                            case (wdata_q)
                                CMD_SAVE: begin
                                    op_q   <= OP_SAVE;
                                    busy_q <= 1'b1;
                                    idx_q  <= 10'd0;
                                    state  <= S_FILL;
                                end
                                CMD_SNAP: begin
                                    op_q   <= OP_SNAP;
                                    busy_q <= 1'b1;
                                    idx_q  <= 10'd0;
                                    state  <= S_FILL;
                                end
                                CMD_LOAD: begin
                                    op_q      <= OP_LOAD;
                                    busy_q    <= 1'b1;
                                    arb_cnt_q <= {ARB_WAIT_LOG2{1'b0}};
                                    state     <= S_ARB;
                                end
                                default: begin
                                    result_q      <= RES_BAD_CMD;
                                    done_sticky_q <= 1'b1;
                                end
                            endcase
                        end
                    end
                    default: ;
                endcase
            end

            if (bvalid_q && s_bready) bvalid_q <= 1'b0;

            // While idle the staging-buffer read port tracks the host
            // pointer, so a BUFDATA read returns the addressed byte.
            // Placed BEFORE the read channel below on purpose: a BUFDATA
            // read overrides this with the POST-increment address in the
            // same cycle it is accepted (see there), and the later
            // assignment has to win.
            if (state == S_IDLE) buf_raddr <= bufptr_q;

            // ── AXI-Lite read channel ─────────────────────────────────
            if (s_arvalid && s_arready) begin
                rvalid_q <= 1'b1;
                case (rd_reg)
                    REG_STATUS: rdata_q <= status_value;
                    REG_BUFPTR: rdata_q <= {23'd0, bufptr_q};
                    REG_BUFDATA: begin
                        rdata_q  <= {24'd0, buf_rdata};
                        bufptr_q <= bufptr_q + 9'd1;
                        // Point the array at the NEXT byte on this very
                        // edge rather than waiting for the S_IDLE tracker
                        // above to notice the incremented pointer.  That
                        // costs one cycle, and the AXI-Lite handshake can
                        // legally present the next AR two cycles after
                        // this one (arready is only gated on !rvalid_q),
                        // which is exactly one cycle too soon — a
                        // back-to-back host read then returns the PREVIOUS
                        // byte and a 512-byte dump comes out shifted.
                        buf_raddr <= bufptr_q + 9'd1;
                    end
                    REG_LBA:   rdata_q <= PRAM_LBA;
                    REG_CKREF: rdata_q <= {16'd0, ck_ref_q};
                    // IDENT (0x00) and anything unmapped.
                    default:   rdata_q <= IDENT_VALUE;
                endcase
            end else if (rvalid_q && s_rready) begin
                rvalid_q <= 1'b0;
            end

            // ── CMD24 byte-stream feeder ──────────────────────────────
            if (ctrl_wr_ready && (wr_raddr < 9'd511))
                wr_raddr <= wr_raddr + 9'd1;

            // ── CMD17 byte-stream sink ────────────────────────────────
            if ((state == S_SD_WAIT) && (op_q == OP_LOAD) && ctrl_rd_valid
                && (idx_q < 10'd512)) begin
                buf_we    <= 1'b1;
                buf_waddr <= idx_q[8:0];
                buf_wdata <= ctrl_rd_data;
                idx_q     <= idx_q + 10'd1;
            end

            // ── Main sequencer ────────────────────────────────────────
            case (state)
                S_IDLE: begin
                    // Nothing to do; CTRL writes above start operations.
                end

                // Lay down the header and the zero padding.  The payload
                // window (16..271) is skipped — S_SNAP_* fills it.
                S_FILL: begin
                    buf_we    <= 1'b1;
                    buf_waddr <= idx_q[8:0];
                    buf_wdata <= (idx_q < 10'd16) ? hdr_byte(idx_q[8:0]) : 8'h00;
                    if (idx_q == 10'd15) begin
                        idx_q <= 10'd272;          // jump over the payload
                    end else if (idx_q == 10'd511) begin
                        idx_q     <= 10'd0;
                        cdc_cnt_q <= {CDC_WAIT_LOG2{1'b0}};
                        state     <= S_SNAP_REQ;
                    end else begin
                        idx_q <= idx_q + 10'd1;
                    end
                end

                // ── Live PRAM -> staging buffer, one byte per handshake ─
                S_SNAP_REQ: begin
                    pram_addr <= idx_q[7:0];
                    pram_we   <= 1'b0;
                    pram_req  <= 1'b1;
                    cdc_cnt_q <= {CDC_WAIT_LOG2{1'b0}};
                    state     <= S_SNAP_ACK;
                end
                S_SNAP_ACK: begin
                    cdc_cnt_q <= cdc_cnt_q + 1'b1;
                    if (pram_ack) begin
                        buf_we    <= 1'b1;
                        buf_waddr <= PAYLOAD_OFF + idx_q[8:0];
                        buf_wdata <= pram_rdata;
                        pram_req  <= 1'b0;
                        cdc_cnt_q <= {CDC_WAIT_LOG2{1'b0}};
                        state     <= S_SNAP_REL;
                    end else if (&cdc_cnt_q) begin
                        pram_req <= 1'b0;
                        result_q <= RES_CDC_TIMEOUT;
                        state    <= S_DONE;
                    end
                end
                S_SNAP_REL: begin
                    cdc_cnt_q <= cdc_cnt_q + 1'b1;
                    if (!pram_ack) begin
                        if (idx_q == 10'd255) begin
                            idx_q         <= 10'd0;
                            ck_a_q        <= CK_SEED_A;
                            ck_b_q        <= CK_SEED_B;
                            hdr_q         <= 80'd0;
                            hdr_nonzero_q <= 1'b0;
                            state         <= S_WALK_A;
                        end else begin
                            idx_q <= idx_q + 10'd1;
                            state <= S_SNAP_REQ;
                        end
                    end else if (&cdc_cnt_q) begin
                        result_q <= RES_CDC_TIMEOUT;
                        state    <= S_DONE;
                    end
                end

                // ── Walk bytes 0..271: capture header, fold checksum ────
                // Three states per byte because sec_buf's read is
                // registered: address out in A, array read completes on
                // the L edge, data readable in B.
                S_WALK_A: begin
                    buf_raddr <= idx_q[8:0];
                    state     <= S_WALK_L;
                end
                S_WALK_L: state <= S_WALK_B;
                S_WALK_B: begin
                    if (idx_q < 10'd10)
                        hdr_q <= {hdr_q[71:0], buf_rdata};
                    if ((idx_q < 10'd16) && (buf_rdata != 8'h00))
                        hdr_nonzero_q <= 1'b1;
                    if (checksummed(idx_q)) begin
                        ck_a_q <= ck_a_q + buf_rdata;
                        ck_b_q <= ck_b_q + ck_a_q + buf_rdata;
                    end
                    if (idx_q == (WALK_END - 10'd1)) begin
                        state <= S_CKST_HI;
                    end else begin
                        idx_q <= idx_q + 10'd1;
                        state <= S_WALK_A;
                    end
                end

                // Publish the computed checksum.  For SAVE/SNAP also write
                // it into the staged header so the sector is
                // self-consistent; for LOAD hand over to the verdict.
                S_CKST_HI: begin
                    ck_calc_q <= ck_walk;
                    if (op_q == OP_LOAD) begin
                        ck_ref_q <= hdr_q[15:0];
                        state    <= S_VERDICT;
                    end else begin
                        buf_we    <= 1'b1;
                        buf_waddr <= 9'd8;
                        buf_wdata <= ck_b_q;
                        state     <= S_CKST_LO;
                    end
                end
                S_CKST_LO: begin
                    buf_we    <= 1'b1;
                    buf_waddr <= 9'd9;
                    buf_wdata <= ck_a_q;
                    if (op_q == OP_SNAP) begin
                        result_q <= RES_OK;
                        state    <= S_DONE;
                    end else begin
                        arb_cnt_q <= {ARB_WAIT_LOG2{1'b0}};
                        state     <= S_ARB;
                    end
                end

                // ── Take the SD bus, but never from a live transfer ─────
                S_ARB: begin
                    if (!boot_done) begin
                        result_q <= RES_NOT_BOOTED;
                        state    <= S_DONE;
                    end else begin
                        sd_req_q  <= 1'b1;
                        arb_cnt_q <= arb_cnt_q + 1'b1;
                        if (sd_gnt) begin
                            state <= S_SD_GO;
                        end else if (&arb_cnt_q) begin
                            sd_req_q <= 1'b0;
                            result_q <= RES_ARB_TIMEOUT;
                            state    <= S_DONE;
                        end
                    end
                end

                S_SD_GO: begin
                    ctrl_cmd_q <= (op_q == OP_LOAD) ? CT_CMD17 : CT_CMD24;
                    ctrl_go_q  <= 1'b1;
                    idx_q      <= 10'd0;
                    wr_raddr   <= 9'd0;
                    state      <= S_SD_WAIT;
                end

                // sd_ctrl carries its own unconditional global watchdog, so
                // `done` is guaranteed to arrive; no local counter needed.
                S_SD_WAIT: begin
                    if (ctrl_done) begin
                        sd_err_cause_q <= ctrl_err_cause;
                        if (ctrl_error) result_q <= RES_SD_ERR;
                        state <= S_SD_REL;
                    end
                end

                S_SD_REL: begin
                    sd_req_q <= 1'b0;
                    if (result_q != RES_OK) begin
                        // An SD error on LOAD still has to leave the array
                        // in a defined state -> defaults.
                        if (op_q == OP_LOAD) begin
                            def_cnt_q <= {DEFAULT_HOLD_LOG2{1'b0}};
                            state     <= S_DEFAULT;
                        end else begin
                            state <= S_DONE;
                        end
                    end else if (op_q == OP_LOAD) begin
                        idx_q         <= 10'd0;
                        ck_a_q        <= CK_SEED_A;
                        ck_b_q        <= CK_SEED_B;
                        hdr_q         <= 80'd0;
                        hdr_nonzero_q <= 1'b0;
                        state         <= S_WALK_A;
                    end else begin
                        state <= S_DONE;
                    end
                end

                // ── Validate what came off the card ─────────────────────
                S_VERDICT: begin
                    result_q <= v_result;
                    if (v_bad) begin
                        def_cnt_q <= {DEFAULT_HOLD_LOG2{1'b0}};
                        state     <= S_DEFAULT;
                    end else begin
                        idx_q     <= 10'd0;
                        cdc_cnt_q <= {CDC_WAIT_LOG2{1'b0}};
                        state     <= S_INST_REQ;
                    end
                end

                // ── Staging buffer -> live PRAM ────────────────────────
                S_INST_REQ: begin
                    buf_raddr <= PAYLOAD_OFF + idx_q[8:0];
                    pram_addr <= idx_q[7:0];
                    cdc_cnt_q <= {CDC_WAIT_LOG2{1'b0}};
                    state     <= S_INST_L;
                end
                S_INST_L: state <= S_INST_ACK;
                S_INST_ACK: begin
                    cdc_cnt_q <= cdc_cnt_q + 1'b1;
                    if (!pram_req) begin
                        // buf_rdata is settled now.  Payload wires and the
                        // request rise together, which satisfies pram_cdc's
                        // MCP contract: the destination does not sample
                        // them until 2 pb_clk edges later.
                        pram_wdata <= buf_rdata;
                        pram_we    <= 1'b1;
                        pram_req   <= 1'b1;
                    end else if (pram_ack) begin
                        pram_req <= 1'b0;
                        pram_we  <= 1'b0;
                        state    <= S_INST_REL;
                    end else if (&cdc_cnt_q) begin
                        pram_req <= 1'b0;
                        pram_we  <= 1'b0;
                        result_q <= RES_CDC_TIMEOUT;
                        state    <= S_DONE;
                    end
                end
                S_INST_REL: begin
                    cdc_cnt_q <= cdc_cnt_q + 1'b1;
                    if (!pram_ack) begin
                        if (idx_q == 10'd255) begin
                            result_q <= RES_OK;
                            state    <= S_DONE;
                        end else begin
                            idx_q <= idx_q + 10'd1;
                            state <= S_INST_REQ;
                        end
                    end else if (&cdc_cnt_q) begin
                        result_q <= RES_CDC_TIMEOUT;
                        state    <= S_DONE;
                    end
                end

                // ── Fall back to rtc.v's OWN post-reset PRAM image ─────
                // pram_default drives the exact same rtc.v input that the
                // JTAG `pram-clear` command uses.  There is deliberately
                // no second definition of "the defaults" in this module.
                S_DEFAULT: begin
                    pram_default <= 1'b1;
                    defaults_q   <= 1'b1;
                    def_cnt_q    <= def_cnt_q + 1'b1;
                    if (&def_cnt_q) begin
                        pram_default <= 1'b0;
                        state        <= S_DONE;
                    end
                end

                S_DONE: begin
                    busy_q        <= 1'b0;
                    done_sticky_q <= 1'b1;
                    sd_req_q      <= 1'b0;
                    pram_req      <= 1'b0;
                    pram_we       <= 1'b0;
                    pram_default  <= 1'b0;
                    state         <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    sd_ctrl u_sd_ctrl (
        .clk           (clk),
        .rst           (rst),
        .crc_check_en  (1'b0),
        .cmd_type      (ctrl_cmd_q),
        .lba           (PRAM_LBA),
        .block_count   (16'd1),
        .go            (ctrl_go_q),
        .rd_valid      (ctrl_rd_valid),
        .rd_data       (ctrl_rd_data),
        .rd_ready      (1'b1),
        .wr_ready      (ctrl_wr_ready),
        .wr_valid      (),
        .wr_data       (wr_byte_q),
        // wr_avail: this caller always has the byte staged before the
        // engine asks for it, so it takes the legacy unpaced write
        // stream unchanged.  See rtl/vhdd.vh / sd_ctrl.v header.
        .wr_avail      (1'b1),
        .spi_cmd_valid (spi_cmd_valid),
        .spi_cmd_ready (spi_cmd_ready),
        .spi_cmd_data  (spi_cmd_data),
        .spi_rsp_valid (spi_rsp_valid),
        .spi_rsp_data  (spi_rsp_data),
        .busy          (ctrl_busy),
        .done          (ctrl_done),
        .error         (ctrl_error),
        .err_cause     (ctrl_err_cause),
        .dbg_rd_crc_calc   (),
        .dbg_rd_crc_recv   (),
        .dbg_last_real_r1  (),
        .dbg_cur_cmd       (),
        .dbg_lba_lat       (),
        .dbg_last_crc7_sent(),
        .dbg_last_write_resp(),
        .dbg_block_idx     (),
        .dbg_last_poll_cnt ()
    );

    /* verilator lint_off UNUSEDSIGNAL */
    wire _unused_pram_sd = &{1'b0, ctrl_busy, s_wstrb, SYS_PERSIST_BASE, 1'b0};
    /* verilator lint_on UNUSEDSIGNAL */

endmodule

`default_nettype wire
