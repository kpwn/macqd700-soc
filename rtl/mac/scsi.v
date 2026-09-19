// MAME reference/excerpt/adaptation attribution: Copyright Olivier Galibert; Patrick Mackinlay.
// MAME-derived portions retain BSD-3-Clause; original contributions are MIT.
// See THIRD_PARTY_NOTICES.md and LICENSES/MAME-BSD-3-Clause.txt.
// scsi.v — NCR 5380 SCSI target emulation (phase-3 full FSM)
//
// Emulates one or two SCSI disk targets (TARGET_ID, plus TARGET_ID_B when
// TARGET_B_EN).  Mac is the initiator; we respond to Selection, parse CDBs,
// stream data from / to a backing store and return Status + Command-Complete.
//
// TWO TARGETS, ONE FSM.  Only one SCSI transaction is live on a bus at a
// time, so the second target is not a second copy of this state machine:
// it is one latched bit, `vh_dev_sel`, sampled at the instant a selection
// succeeds and exported on the vhdd seam so rtl/soc/vhdd_mux.v can route
// the request to the right backing-store provider.  `dev_en` says which
// targets are live right now; a target whose bit is 0 is indistinguishable
// from an absent ID.
//
// Phase FSM (per docs/peripheral_arch.md §SCSI):
//
//   BUS_FREE ─(SEL asserted, ID=0 bit on data bus)─► SELECTION
//   SELECTION ─(Mac drops SEL)─────────────────────► COMMAND
//   COMMAND  ─(CDB bytes drained via REQ/ACK)──────► DATA_IN | DATA_OUT | STATUS
//   DATA_IN  ─(all bytes pulled)───────────────────► STATUS
//   DATA_OUT ─(all bytes pushed)───────────────────► STATUS
//   STATUS   ─(1 byte consumed)────────────────────► MSG_IN
//   MSG_IN   ─(1 byte consumed)────────────────────► BUS_FREE
//   (any state) ─(RST asserted on initiator cmd)───► BUS_FREE
//   (any state) ─(non-live-ID selection)───────────► BUS_FREE (no-device)
//
// Backing store: this module is SCSI protocol only.  Blocks come from a
// *virtual HDD* behind the `vh_*` ports — the contract in rtl/vhdd.vh.
// It knows the volume is addressed in VHDD_BLOCK_BYTES (512 B) blocks
// starting at LBA 0, and nothing else: no SD command codes, no reserved
// boot window, no card geometry, no filesystem.  Whoever provides the
// volume owns all of that (rtl/soc/vhdd_sd.v maps it onto an SD card,
// reserving that card's low sectors for ROM/boot assets).  In sim the
// testbench mocks the provider; on FPGA fpga_top instantiates one.
//
// Supported CDBs (opcode byte 0 of the CDB):
//   0x00 TEST UNIT READY        → 0 B data, GOOD status when medium ready
//   0x03 REQUEST SENSE          → 18 B fixed-format sense data
//   0x08 READ 6                 → N × 512 B from disk
//   0x0A WRITE 6                → N × 512 B to disk
//   0x12 INQUIRY                → 36 B vendor/product/rev
//   0x15 MODE SELECT 6          → dump & GOOD (we don't honour params)
//   0x1A MODE SENSE 6           → minimum "no pages" (4 B header)
//   0x1B START STOP UNIT        → GOOD
//   0x25 READ CAPACITY (10)     → 8 B (last LBA + block size)
//   0x28 READ 10                → N × 512 B from disk
//   0x2A WRITE 10               → N × 512 B to disk
//   default                     → CHECK CONDITION + ILLEGAL REQUEST sense
//
// IRQ: level-sensitive per 5380.  Asserted as soon as STATUS or MSG_IN is
// visible so a ROM polling the phase bits cannot see a ready status byte
// before the interrupt bit/wire.  The post-disconnect latch is cleared by a
// read of register 7 (Reset Interrupt) per 5380 convention.
//
// DRQ: active-high internal DMA request exported for board glue.  It is
// high only while reg 5 bit 6 would read high: a live data phase with REQ
// asserted.  STATUS/MSG_IN completion is reported by END_DMA + IRQ, never
// by leaving DRQ asserted.  Q700 VIA2 PA6 is an active-low latch, so the
// top-level board pin should receive ~drq.
//
// Register map (`pb_addr[8:0]`, low 8 bits preserved by TurboSCSI)
// ┌───┬────────────────────────────┬──────────────────────────────────┐
// │ # │ Read                       │ Write                            │
// ├───┼────────────────────────────┼──────────────────────────────────┤
// │ 0 │ Current SCSI Data          │ Output Data                      │
// │ 1 │ Initiator Command          │ Initiator Command                │
// │ 2 │ Mode                       │ Mode                             │
// │ 3 │ Target Command             │ Target Command                   │
// │ 4 │ Current SCSI Bus Status    │ Select Enable                    │
// │ 5 │ Bus and Status             │ Start DMA Send                   │
// │ 6 │ Input Data  (also DMA RX)  │ Start DMA Target Receive         │
// │ 7 │ Reset Parity / IRQ (clr)   │ Start DMA Initiator Receive      │
// └───┴────────────────────────────┴──────────────────────────────────┘
//
// TurboSCSI front-door shape:
//   0x000..0x0ff  NCR 5380 register aperture (low 3 bits select the reg)
//   0x100..0x101  pseudo-DMA handshake shim
// For now the DMA shim returns deterministic data without implementing the
// full DAFB-mediated pseudo-DMA engine.
//
// Bit layouts (positive logic — 1 == signal asserted on the bus):
//   reg 1 (Initiator Command):
//     [7]=RST [6]=AIP/TEST [5]=LA/DIFF [4]=ACK
//     [3]=BSY [2]=SEL [1]=ATN [0]=DATA_BUS
//   reg 3 (Target Command):
//     [3]=REQ [2]=MSG [1]=C_D [0]=I_O
//   reg 4 (Current SCSI Bus Status, read-only):
//     [7]=RST [6]=BSY [5]=REQ [4]=MSG [3]=C_D [2]=I_O [1]=SEL [0]=DBP
//   reg 5 (Bus and Status, read-only):
//     [7]=END_DMA [6]=DRQ [5]=PARITY_ERR [4]=IRQ
//     [3]=PHASE_MATCH [2]=BUSY_ERROR [1]=ATN [0]=ACK
`include "vhdd.vh"

module scsi #(
    parameter [2:0] TARGET_ID      = 3'd0,
    // Second target ID.  One back-end FSM serves both: only ONE SCSI
    // transaction is live on a bus at a time, so "which device am I
    // talking as" is a single latched bit (vh_dev_sel) sampled at the
    // instant a selection succeeds, not a second copy of the FSM.
    parameter [2:0] TARGET_ID_B    = 3'd1,
    parameter       TARGET_B_EN    = 1'b0,
    parameter TURBOSCSI_C96  = 1'b0,
    // pb_clk cycles per emulated 53C96 chip clock.  The Quadra 700 clocks
    // its NCR53C96 at 50_MHz_XTAL/2 = 25 MHz (MAME macquadra700.cpp:770);
    // our peripheral bus runs at 50 MHz (fpga_top_sd.vh:12), so one chip
    // clock = 2 pb_clk cycles.  Used ONLY by the selection-timeout timer.
    parameter [7:0] C96_CLK_DIV = 8'd2
) (
    input  wire        clk,
    input  wire        rst,
    // Which targets are live on the bus right now.  bit0 = target A
    // (TARGET_ID), bit1 = target B (TARGET_ID_B).  A target whose bit is
    // 0 is not selectable and looks exactly like an absent ID to the
    // initiator (5380: selection ignored; C96: select-timeout shim).
    input  wire [1:0]  dev_en,
    // Volume size we report to the OS, in VHDD_BLOCK_BYTES blocks.
    // RUNTIME INPUT, deliberately not a parameter: it must track the
    // actual backing store (for the SD provider, the card's sector count
    // minus its reserved window).  When this was a hardcoded 32'd1048576
    // and a 700 MB image was provisioned, 188 MB of the HFS volume sat
    // beyond the reported capacity -- including the Alternate MDB in the
    // volume's second-to-last block -- so those reads could never
    // complete and the File Manager hung in a way that looked exactly
    // like a CPU bug (2026-07-28).
    input  wire [31:0] vh_num_lbas,
    // ── Peripheral-bus slave ─────────────────────────────────────────
    input  wire [8:0]  pb_addr,
    input  wire [7:0]  pb_wdata,
    input  wire        pb_wr,
    input  wire        pb_rd,
    output reg  [7:0]  pb_rdata,
    output reg         pb_ack,
    // ── IRQ / DRQ lines ──────────────────────────────────────────────
    output wire        irq,
    output wire        drq,
    // ── Virtual-HDD backing store (contract: rtl/vhdd.vh) ────────────
    // Everything here is backing-store agnostic.  No SD command codes,
    // no reserved-sector bias, no card geometry: the provider owns all
    // of that (rtl/soc/vhdd_sd.v for the SD-card volume).
    //
    // Addressability probe — combinational, live every cycle.  The
    // provider answers whether the extent we are ABOUT to move is
    // reachable on this volume; we gate the transfer on it before the
    // first vh_req_go and re-evaluate it as the extent shrinks.
    output wire [31:0] vh_chk_lba,
    output wire [23:0] vh_chk_blocks,
    input  wire        vh_chk_ok,
    // Request.  vh_req_* are stable from the cycle vh_req_go is high
    // until the next vh_req_go.
    output reg         vh_req_write,        // 0 = read, 1 = write
    output reg         vh_req_multi,        // 0 = single block, 1 = stream
    output reg  [31:0] vh_req_lba,          // volume-relative block
    output reg  [15:0] vh_req_block_count,  // blocks in this request
    output reg         vh_req_go,           // 1-cycle pulse
    input  wire        vh_busy,
    input  wire        vh_done,
    input  wire        vh_error,
    input  wire        vh_rd_valid,     // 1-cycle per byte from the volume
    input  wire [7:0]  vh_rd_data,
    output wire        vh_rd_ready,     // multi-block ring back-pressure:
                                        // low pauses the byte stream when
                                        // the block-sized ring is nearly full
    input  wire        vh_wr_ready,     // 1-cycle when the volume wants a byte
    output wire        vh_wr_valid,     // hi during data-phase (info only)
    output reg  [7:0]  vh_wr_data,
    // REAL write-side back-pressure (the mirror of vh_rd_ready): 1 when
    // we have at least one buffered byte for the provider to take.
    // Holding it low pauses the provider's byte stream at the source.
    output wire        vh_wr_avail,
    // Which volume the live transaction belongs to: 0 = target A,
    // 1 = target B.  Latched when a selection succeeds and held until
    // the NEXT successful selection, so it is stable for the whole of
    // the request it qualifies (including the tail of a multi-block
    // stream that outlives STATUS/MSG_IN).  Routed by rtl/soc/vhdd_mux.v.
    output reg         vh_dev_sel,
    // ── TurboSCSI shim ───────────────────────────────────────────────
    // DAFB +0x24 SCSI bus 1 ctrl register, low 9 bits (m_scsi_ctrl[0]
    // per MAME dafb.cpp:418-422 + 487-528).  bit[7]=DRQ-check on read,
    // bit[8]=DRQ-check on write.  Ignored when TURBOSCSI_C96_EN=0 so
    // the bare-5380 path stays unchanged.  Driven by the DAFB shim
    // (rtl/mac/video.v) at the top of fpga_top.
    input  wire [8:0]  scsi_ctrl_in,
    // 1 on the cycle this pb beat is the LOW half of a HOST 16-bit
    // pseudo-DMA aperture access that the fabric split into two
    // byte-granular pb beats.  Driven by peripheral_bus.v from the
    // SAME condition that picks the odd aperture address
    // (rd_size_q == 3'd1 && rd_scsi_phase_q == 2'd1); every other pb
    // beat — including a genuine standalone BYTE access to the odd
    // aperture byte 0x101 — must leave this LOW so it performs its own
    // DRQ check.  Address parity CANNOT carry this: a byte-granular
    // driver walking 0x100,0x101,0x100,... is indistinguishable from a
    // split word by address alone.  Harnesses with no peripheral_bus
    // in front of scsi.v tie this to 0 — with no splitter, every pb
    // beat IS a standalone access by definition.
    input  wire        pb_dma16_lo_beat,
    // 1 whenever a DMA-shim (pb_addr 0x100/0x101) READ issued THIS cycle
    // is guaranteed NOT to be withheld by the drq_c96 DRQ-check below —
    // i.e. the same condition pb_ack's own withhold-gate evaluates,
    // exported so peripheral_bus.v can avoid firing its one-shot
    // scsi_rd pulse on a cycle where the ack would be silently dropped
    // (see rd_scsi_dma_shim_active's comment in peripheral_bus.v).
    output wire        dma_rd_ready,
    // Write-side analog of dma_rd_ready: 1 whenever a DMA-shim WRITE
    // issued THIS cycle is guaranteed NOT to be withheld by the
    // drq_c96 DRQ-check (mirrors scsi_ctrl_in[8] instead of [7]).
    // Consumed by peripheral_bus.v's wr_scsi_word_active pulse gate —
    // see that wire's comment for why the write side needed its own
    // serializer FSM rather than the read fix's simple pulse gate.
    output wire        dma_wr_ready,
    // ── Debug observability (M-dbg) ───────────────────────────────────
    // scsi.v can return CHECK CONDITION on paths that leave sd_ctrl's
    // error counters untouched and never complete a back-end request --
    // the "READ timeout waiting for backing store" arms (sticky
    // `medium_not_present`, sense key 2 / ASC 3A) and the fail-closed
    // range check (`!vh_chk_ok`, key 5 / ASC 21).  On hardware those are
    // indistinguishable today: the 53C96 trace ring is de-instantiated and
    // nothing else exposes the sense state, so a live CHECK CONDITION
    // cannot be attributed to an arm.  Measured 2026-08-18: the .ASYC00
    // driver takes CHECK CONDITION on READ(10) LBA 2314 x1 while the volume
    // reports ENABLED / 4194304 blocks, then retries 16x and returns ioErr.
    // These outputs exist to name that arm.  Purely observational.
    // The three LIVE inputs to the READ range check (scsi.v:2905 `!vh_chk_ok`).
    // sd_scsi_lba_mapper.v computes
    //   valid = (scsi_blocks != 0) && !mapped[32] && (mapped_end <= 2^32)
    //           && (end_lba <= num_lbas)
    // and vh_chk_blocks is wired to the LIVE xfer_blocks, so a zero block count
    // makes a perfectly good read fail closed with ILLEGAL REQUEST / ASC 0x21.
    // Exposing chk_ok plus its two operands makes that case self-diagnosing
    // instead of needing another build round-trip.
    output wire        dbg_chk_ok,
    output wire [23:0] dbg_xfer_blocks,
    output wire [31:0] dbg_xfer_lba,
    // CTRL[2] of the vhdd CSR, synchronised into pb_clk.  A locked volume
    // answers WRITE(6)/WRITE(10) with CHECK CONDITION / DATA PROTECT rather
    // than touching the card, so the guest sees a write-protected disk and
    // the backing image cannot be modified.
    input  wire        wprot,
    output wire        dbg_medium_not_present,
    output wire [3:0]  dbg_sense_key,
    output wire [7:0]  dbg_sense_asc,
    output wire [7:0]  dbg_check_cond_count,
    // ── 53C96 initiator-state probe (observation only) ────────────────
    // Added 2026-08-19 to resolve the post-boot-fix hang: the ROM's
    // untimed wait-for-INT at 0x40899704 spins forever after ROM
    // 0x40899010/0x40899270 does `W reg2=0xEE; W reg3=0x10` (non-DMA
    // Transfer Information).  Board shows reg3=0x10 dispatched,
    // reg4=0x13 (STATUS+TC0, INT clear), reg7=0x01, static.  Seven
    // RTL-vs-MAME differentials are byte-identical, so the remaining
    // unknowns are internal.  PURELY COMBINATIONAL FAN-OUT of existing
    // state — nothing here drives behaviour.
    //
    //   [83]    c96_sel_stopped   halted in MSG_OUT after the message
    //   [82]    c96_sel_active    select accepted, CDB assembling
    //   [81]    c96_xfer_active   ** live connection (MAME mode==MODE_I) **
    //                             distinguishes 'queued behind slot 0'
    //                             from 'armed but never completed'
    //   [80:79] c96_dma_dir       0=NONE 1=IN 2=OUT
    //   [78:69] c96_accept_pend   chip-accepted, not-yet-drained bytes
    //   [68:52] c96_xfr_left      CPU beats remaining in this chunk
    //   [51]    c96_xfr_armed     ** is the CI_XFER still armed? **
    //   [50]    c96_xfr_dma       DMA form (0x90) vs non-DMA (0x10)
    //   [49:46] c96_xfr_phase     ** phase LATCHED at dispatch **
    //   [45:42] phase             ** live FSM phase **
    //   [41:40] c96_command_pos   command queue depth 0..2
    //   [39:32] c96_command_q     command slot 0 (reg 3 read echo)
    //   [31:24] c96_cmd_q1        command slot 1
    //   [23:19] c96_fifo_pos      FIFO occupancy 0..16 (reg 7)
    //   [18:11] c96_istatus       pending istatus WITHOUT the
    //                             destructive reg 5 read
    //   [10:3]  c96_status_sticky {GROSS,PARITY,_,TCC,TC0,_,_,_}
    //   [2]     c96_irq_pending   IRQ at the source
    //   [1]     t_req             target-side REQ
    //   [0]     c96_nondma_supply target has a byte to give
    //
    // Phase encoding for [49:46] and [45:42] (localparams below):
    //   0 BUS_FREE   1 SELECT     2 COMMAND    3 CMD_EXEC
    //   4 DATA_IN    5 DATA_OUT   6 VH_WAIT_RD 7 VH_WAIT_WR
    //   8 STATUS     9 MSG_IN    10 DISCONNECT 11 RESET
    // NOTE 3/6/7 are OUR internal states; MAME's xfr_phase is masked
    // from the bus (`ctrl & S_PHASE_MASK`) and can never hold them.
    output wire [83:0] dbg_c96_state
);
    localparam [0:0] TURBOSCSI_C96_EN = (TURBOSCSI_C96 != 0);
    localparam TGT_B_LIVE = (TARGET_B_EN != 0);

    // ── Debug: count CHECK CONDITION completions ──────────────────────
    // xfer_status is loaded with 8'h02 by every CHECK CONDITION arm, so a
    // rising edge on (xfer_status == 8'h02) counts them without touching
    // any of those arms.  Saturating, so it never wraps to a lie.
    reg [7:0] dbg_cc_count;
    reg       dbg_cc_seen;
    always @(posedge clk) begin
        if (rst) begin
            dbg_cc_count <= 8'd0;
            dbg_cc_seen  <= 1'b0;
        end else begin
            dbg_cc_seen <= (xfer_status == 8'h02);
            if ((xfer_status == 8'h02) && !dbg_cc_seen && !(&dbg_cc_count))
                dbg_cc_count <= dbg_cc_count + 8'd1;
        end
    end
    assign dbg_chk_ok             = vh_chk_ok;
    assign dbg_xfer_blocks        = xfer_blocks;
    assign dbg_xfer_lba           = xfer_lba;
    assign dbg_medium_not_present = medium_not_present;
    assign dbg_sense_key          = sense_key;
    assign dbg_sense_asc          = sense_asc;
    assign dbg_check_cond_count   = dbg_cc_count;

    // ── Target-ID resolution ──────────────────────────────────────────
    // sel_id_hit(id)   : 1 when `id` names a currently-LIVE target.
    // sel_id_which(id) : 1 when `id` resolves to target B, else 0.
    //
    // Target A wins if both would match the same id (a degenerate
    // TARGET_ID_B == TARGET_ID build).  With TARGET_B_EN = 0 the B term
    // is a constant 0, so sel_id_hit collapses to `dev_en[0] && id ==
    // TARGET_ID` and sel_id_which to a constant 0 — which is why
    // TARGET_B_EN=0 + dev_en=2'b01 is bit-identical to the single-target
    // behaviour these functions replaced.
    function sel_id_hit;
        input [2:0] id;
        begin
            sel_id_hit = (dev_en[0] && (id == TARGET_ID)) ||
                         (TGT_B_LIVE && dev_en[1] && (id == TARGET_ID_B));
        end
    endfunction

    function sel_id_which;
        input [2:0] id;
        begin
            sel_id_which = TGT_B_LIVE && dev_en[1] && (id == TARGET_ID_B) &&
                           !(dev_en[0] && (id == TARGET_ID));
        end
    endfunction

    // The ID this target currently answers as — used when we drive our
    // own ID onto the data bus during reselection.
    wire [2:0] sel_tgt_id = vh_dev_sel ? TARGET_ID_B : TARGET_ID;
    // ══════════════════════════════════════════════════════════════════
    // Register file state
    // ══════════════════════════════════════════════════════════════════
    reg [7:0] r_output_data;   // reg 0 write
    reg [7:0] r_init_cmd;      // reg 1 (both)
    reg [7:0] r_mode;          // reg 2 (both)
    reg [7:0] r_tgt_cmd;       // reg 3 (both)
    // 5380 selection: the initiator drives a bit-per-ID vector on the
    // data bus (not a 3-bit id), so resolve the VECTOR to hit /
    // which-device here rather than through sel_id_hit/sel_id_which.
    // Target A wins if both bits are set.
    wire sel_bus_hit_a = dev_en[0] && r_output_data[TARGET_ID];
    wire sel_bus_hit_b = TGT_B_LIVE && dev_en[1] && r_output_data[TARGET_ID_B];
    wire sel_bus_hit   = sel_bus_hit_a || sel_bus_hit_b;
    wire sel_bus_which = sel_bus_hit_b && !sel_bus_hit_a;
    wire [2:0] sel_bus_id = sel_bus_which ? TARGET_ID_B : TARGET_ID;
    // NCR 5380 reg 4 (Select Enable) and regs 5/6/7 (DMA triggers) are
    // write-only command registers per the 5380 datasheet — the same
    // address read returns Bus Status / Bus and Status / Input Data /
    // Reset-IRQ instead, so there is no software-visible storage.  The
    // pre-merge code latched their write values "for tb readback" but
    // nothing in the design or in tb-scsi consumes the latches; Vivado
    // [Synth 8-6014] removes them.  Reset-IRQ behaviour on a read of
    // reg 7 stays driven from irq_pending / busy_error_pending below.
    // ══════════════════════════════════════════════════════════════════
    // NCR 53C96 register file (Quadra 700 DAFB-exposed aperture)
    // ──────────────────────────────────────────────────────────────────
    // MAME-faithful 1:1 with mame0287/src/devices/machine/ncr53c90.{cpp,h}.
    //
    // Register layout (offset within the chip — DAFB strides each address
    // 16 bytes apart in CPU space, peripheral_bus collapses to 4 bits):
    //   off  read                        write
    //    0   tcounter[7:0]               tcount[7:0]
    //    1   tcounter[15:8]              tcount[15:8]
    //    2   FIFO pop (side-effect)      FIFO push (side-effect)
    //    3   command (echo command_q)    command (queued, runs)
    //    4   status                      bus_id (target id [2:0])
    //    5   istatus (clears IRQ)        select_timeout
    //    6   seq_step                    sync_period
    //    7   fifo_flags (= fifo_pos)     sync_offset
    //    8   config1                     config1
    //    9   —                            clock_conv (write-only)
    //   0a   —                            test (write-only)
    //   0b   config2                     config2
    //   0c   config3                     config3
    //   0f   —                            fifo_align (write-only)
    //
    // Status bit layout (read at off 4):
    //   [2:0] = SCSI bus phase [MSG | C/D | I/O] — derived from `phase`
    //   [3]   = TCC      (transfer counter complete; sticky until istatus rd)
    //   [4]   = TC0      (transfer counter is zero)
    //   [5]   = PARITY   (sticky until istatus rd)
    //   [6]   = GROSS    (sticky until istatus rd)
    //   [7]   = INTR     (mirror of `irq` — 53C90A and later)
    //
    // istatus bit layout (read at off 5; reading also clears IRQ when set):
    //   [0]=SELECTED  [1]=SELECT_ATN  [2]=RESELECTED   [3]=FUNCTION
    //   [4]=BUS       [5]=DISCONNECT  [6]=ILLEGAL      [7]=SCSI_RESET
    // ══════════════════════════════════════════════════════════════════
    reg [15:0] c96_tcount;          // off 0/1 write target
    // 17 bits: a DMA command loaded with tcount=0 transfers 65536 bytes
    // (MAME load_tcounter loads 0 and the decrement wraps through 0xffff;
    // scsi_fuzz finding F4, 2026-08-19).  Register reads expose [15:0].
    reg [16:0] c96_tcounter /* verilator public_flat_rd */;  // off 0/1 read source
    reg [7:0]  c96_fifo [0:15];     // 16-byte FIFO at off 2
    reg [4:0]  c96_fifo_pos;        // 0..16 (bit 4 covers 16)
    reg [7:0]  c96_command_q;       // off 3 read echo
    reg [1:0]  c96_command_pos;     // 0..2 — MAME-faithful queue depth
    reg [7:0]  c96_cmd_q1;          // queued second command (slot 1)
    reg [7:0]  c96_status_sticky;   // {GROSS,PARITY,_,TCC,TC0,_,_,_}
    reg [7:0]  c96_istatus;         // off 5 read source
    reg [7:0]  c96_seq_step;        // off 6 read source
    reg [2:0]  c96_bus_id;          // off 4 write
    reg [7:0]  c96_select_timeout;  // off 5 write
    reg [4:0]  c96_sync_period;     // off 6 write
    reg [3:0]  c96_sync_offset;     // off 7 write
    reg [7:0]  c96_config1;         // off 8
    reg [7:0]  c96_config2;         // off 0xb
    reg [7:0]  c96_config3;         // off 0xc
    // ── 16-bit DMA-port access pairing (MAME dma16_r atomicity) ───────
    // The DAFB TurboSCSI pseudo-DMA aperture is a SIXTEEN-BIT port:
    // macquadra700.cpp:565 maps 0x5000f100..0x5000f101 through
    // dafb_device::turboscsi_dma_r, which checks DRQ exactly ONCE per
    // host access (dafb.cpp:1000-1010) and then pops BOTH bytes
    // atomically via ncr53c94_device::dma16_swap_r ->
    // ncr53c90.cpp:1325-1350.  MAME never re-checks DRQ between the two
    // bytes of one word.
    //
    // Our pb face is byte-granular, so peripheral_bus.v replays one word
    // DMA-shim read as two sequential byte beats, flagging the second
    // one with pb_dma16_lo_beat (and addressing it at the aperture's
    // odd byte 0x101).  The FLAG, not the address, is what this latch
    // keys on: a byte-granular driver walking 0x100,0x101,0x100,...
    // produces the same address parity as a split word, so inferring
    // the pairing from pb_addr[0] would skip the DRQ check on every
    // other byte of such a drain.
    // Re-running the DRQ check on the low half is the divergence that
    // broke ROM boot: with LBTM set (config3 bit 2 — the Q700 ROM writes
    // config3 = 0x04 at PC 0x40899120) the BUSMD_1 DMA_IN formula is
    // `fifo_pos > 1` (ncr53c90.cpp:1387), which is FALSE at the odd
    // intermediate occupancy only a byte-granular split can produce.
    // The last word of every 16-byte chunk was therefore stranded, the
    // pb_ack was withheld forever and the CPU took a bus error at the
    // aperture (HW-confirmed Sad Mac 0F02, vec=2, fa=0x50f4f100, ROM PC
    // 0x4089931c; MAME trace records #1613..#1620 drain the same chunk
    // clean).
    //
    // This latch carries the grant across the pair: it is set when the
    // HIGH half of a DRQ-checked DMA-port read is actually granted, and
    // consumed by its LOW-half partner, restoring MAME's per-ACCESS DRQ
    // check.  It is NOT an exemption for odd aperture addresses: a
    // standalone byte read of 0x101 arrives with pb_dma16_lo_beat LOW
    // and faces the full check (tb_turboscsi's "DMA read held off
    // (0x101) when DRQ low + ctrl[7]=1").
    // 2026-08-19: the same reasoning applies verbatim to the WRITE
    // aperture (dafb.cpp:1039-1053 checks m_drq[bus] once, then hands
    // the whole word to ncr53c94_device::dma16_swap_w), so the latch is
    // now set by a granted high-half WRITE beat too and consumed by its
    // low-half partner.  Without it a DRQ that falls between the two
    // write beats (fifo_pos hitting 15, or a tcount==1 chunk exhausting
    // the counter on beat 0) stranded the second beat and the AXI write
    // only ever terminated through peripheral_bus.v's watchdog as a bus
    // error — the write-side twin of the Sad Mac 0F02 above.
    reg        c96_dma16_hi_granted;
    // ── 16-bit aperture single-byte degradation ───────────────────────
    // MAME's 53C94 does NOT always move two bytes per 16-bit access:
    //
    //   dma16_r (ncr53c90.cpp:1325-1328):
    //       if (fifo_pos < 2) return dma_r() | 0xff00;
    //     -> ONE pop, ONE tcounter decrement, and a literal 0xFF filler
    //        in the other half.  After dma16_swap_r the host word is
    //        {residual, 0xFF}.
    //
    //   dma16_w (ncr53c90.cpp:1352-1357):
    //       if (fifo_pos > 14 || tcounter == 1) { dma_w(data & 0x00ff); return; }
    //     -> ONE push of the FIRST memory byte (the byte-swapped word's
    //        low half), ONE tcounter decrement, and the second byte is
    //        DISCARDED.
    //
    // Both tests are evaluated ONCE, before the access does anything.
    // Our byte-granular replay must therefore decide at the HIGH beat
    // and swallow the LOW beat: this latch carries that decision across
    // the pair exactly like c96_dma16_hi_granted carries the DRQ grant.
    // A swallowed low beat is still ACKed (the host access completes),
    // but performs no FIFO push/pop and no tcounter decrement, and reads
    // back MAME's 0xFF filler.
    //
    // Getting this wrong on the WRITE side is silent data corruption,
    // not a hang: every pseudo-DMA write ending on an odd count would
    // push one byte too many into the transfer.
    reg        c96_dma16_degrade;
    reg        c96_irq_pending;     // mirror status[7] / istatus≠0
    reg [2:0]  c96_clock_conv;      // off 9 write (clock conversion factor)
    // ── Selection timeout ─────────────────────────────────────────────
    // A select of an ID with no responder must end in I_DISCONNECT.  Two
    // independent mechanisms drive it, and BOTH stay armed at once:
    //
    //  (1) `c96_select_timeout_cyc` — the REAL chip timer.  This is the
    //      faithful model: MAME's ncr53c90 arbitration chain, from the
    //      command write to I_DISCONNECT, is
    //          arbitrate()      delay(11)              (ncr53c90.cpp:1076)
    //          ARB_COMPLETE     delay(6)               (:349)
    //          ARB_ASSERT_SEL   delay_cycles(4)        (:359)
    //          ARB_SET_DEST     delay(2)               (:368)
    //          ARB_RELEASE_BUSY delay(8192*sel_timeout) (:385)
    //          ARB_TIMEOUT_BUSY delay(1000)            (:413)
    //          ARB_TIMEOUT_ABORT → istatus |= I_DISCONNECT
    //      with delay(n) == n * (clock_conv ? clock_conv : 8) chip clocks
    //      (:802-806) and delay_cycles(n) == n chip clocks.  So
    //          T_chip = cc * (1019 + 8192*select_timeout) + 4
    //      SCSI Manager 4.3 needs this: it issues CD_SELECT to every ID,
    //      never polls seq_step, and parks the request waiting for the
    //      IRQ.  Without a time-based timeout the bus scan hangs forever.
    //  (2) `c96_select_timeout_polls` — the legacy poll-counted shim.
    //      The Q700 ROM and System 7.0.1's SCSI Manager DO poll seq_step
    //      while waiting, and reach I_DISCONNECT through this path long
    //      before the real timer expires.  Kept as-is (additive) so those
    //      flows are unchanged.
    reg        c96_select_timeout_active;
    reg [7:0]  c96_select_timeout_polls;
    reg [7:0]  c96_select_timeout_limit;
    // Down-counter in 53C96 chip clocks.  Max load is
    // 8*(1019+8192*255)+4 = 16,719,836 → 25 bits.
    reg [24:0] c96_select_timeout_cyc;
    // pb_clk → chip-clock prescaler (see C96_CLK_DIV).
    reg [7:0]  c96_clk_div_cnt;
    wire       c96_chip_tick = (c96_clk_div_cnt == 8'd0);
    // Effective clock-conversion factor: MAME's delay() uses
    // `clock_conv ? clock_conv : 8` (ncr53c90.cpp:804).  device_reset()
    // leaves it at 2 (:251); the Mac SCSI Manager programs 5 for a 25 MHz
    // part (25 MHz / 5 MHz nominal).
    wire [3:0]  c96_cc_eff = (c96_clock_conv == 3'd0)
                             ? 4'd8 : {1'b0, c96_clock_conv};
    // 1019 = 11 (arbitrate) + 6 (ARB_COMPLETE) + 2 (ARB_SET_DEST)
    //        + 1000 (ARB_TIMEOUT_BUSY); all scaled by cc.  The extra +4 is
    // ARB_ASSERT_SEL's delay_cycles(4), which is NOT scaled.
    wire [20:0] c96_sel_to_base = 21'd1019 + {c96_select_timeout, 13'd0};
    wire [24:0] c96_sel_to_load =
        ({4'd0, c96_sel_to_base} * {21'd0, c96_cc_eff}) + 25'd4;
    // ── C96 → back-end bypass control (M3) ────────────────────────────
    // When the C96 sees a CD_SELECT* with bus_id naming a live target
    // (TARGET_ID, or TARGET_ID_B when TARGET_B_EN) and a CDB
    // sitting in the FIFO, it short-circuits the 5380-style REQ/ACK
    // handshake by directly priming `cdb[]`, `cdb_len`, and jumping the
    // back-end FSM into S_CMD_EXEC.  Once the back-end finishes (reaches
    // S_STATUS) we surface I_FUNCTION on istatus + IRQ.
    reg        c96_xfer_active;
    reg        c96_sel_pending;    // CD_SELECT* shortcut fired; raise
                                   // I_FUNCTION|I_BUS + seq=4 once the
                                   // back-end settles in its first bus
                                   // phase (MAME function_bus_complete()).
    reg        c96_xfr_armed;      // CI_XFER (0x10/0x90) issued; raise
                                   // I_BUS on the data→STATUS phase change
                                   // (MAME bus_complete() at INIT_XFR end).
    reg        c96_xfr_dma;        // CI_XFER was the DMA form (0x90) — the
                                   // DAFB pseudo-DMA shim carries the beats
                                   // and DRQ is allowed to assert.  The
                                   // non-DMA form (0x10) drains data-in
                                   // bytes through the 16-byte FIFO.
    // ── Out-phase CI_XFER PARK (MAME ncr53c90.cpp:601-611) ────────────
    // The out-group arm of INIT_XFR (DATA OUT / COMMAND / MSG OUT) is
    //     state = INIT_XFR_SEND_BYTE;
    //     if (fifo_pos == 0) break;    // "can't send if the fifo is empty"
    //     ... send_byte();
    // The `break` leaves the machine in INIT_XFR_SEND_BYTE WITHOUT having
    // called send_byte(), so no delay timer is armed and step() is never
    // re-entered from the chip side.  INIT_XFR_WAIT_REQ — where the
    // "non-dma out: fifo empty" completion at :649 lives — is therefore
    // UNREACHABLE for a transfer that started with an empty FIFO: MAME
    // raises NO interrupt at all and the command keeps queue slot 0.
    // We used to complete it immediately with I_BUS (four sites, all
    // reasoning from :649 while missing the :608-610 early break) —
    // measured divergence, scsi_fuzz seed 27:
    //     rtl  SYNC 3 ... stat=92 ... istat=10   (irq=1)
    //     mame SYNC 3 ... stat=12 ... istat=00   (irq=0)
    // Only a HOST FIFO write can unpark it: both fifo_w() (:869-877) and
    // dma_w() (:1182-1189) end in step(false), which walks
    // INIT_XFR_SEND_BYTE -> INIT_XFR_WAIT_REQ and finds the FIFO no
    // longer empty.  Hence: latched at dispatch, cleared the moment the
    // FIFO is observed non-empty.  With one byte staged the two sides
    // already agreed (verified with the same script + `W 2 aa`), so this
    // flag is the ONLY thing that separates them.
    reg        c96_xfr_out_park;
    // The RESUME is deferred exactly like the c96_xfr_recv_* pull below,
    // and for the same reason: MAME's unpark walks
    // INIT_XFR_SEND_BYTE -> INIT_XFR_WAIT_REQ -> INIT_XFR -> send_byte(),
    // and send_byte() rides a delay() timer that only fires when EMULATED
    // TIME advances.  A host that keeps hammering the chip's registers
    // burns no emulated time at all, so every one of those writes still
    // sees the parked, queue-occupying chip.  Modelling the resume as
    // instantaneous made our chip drop out of the full-queue state early,
    // and the normalization writes that MAME swallows with S_GROSS_ERROR
    // then executed here (scsi_fuzz seed 27, SYNC 5).  Same quiesce
    // constant, same "a DMA-PORT access restarts the countdown" rule —
    // deliberately NOT restarted by plain register reads, so a driver
    // spinning on reg 4 for INTR still sees the transfer resume.
    reg [9:0]  c96_xfr_park_timer;
    // ── Post-ATN_STOP message-drain pacing (same mechanism) ───────────
    // MAME's out-phase transfer sends its FIRST byte SYNCHRONOUSLY inside
    // start_command() (start_command -> INIT_XFR -> send_byte(), all in
    // the command write), and every LATER byte only when the send_byte()
    // delay() timer fires, i.e. only when emulated time advances.  A host
    // that immediately drains the DMA port therefore pops the bytes the
    // chip has NOT yet sent.  Our drain ran free at one byte per cycle,
    // so it emptied the FIFO before the host's pops could see it:
    //   scsi_fuzz seed 39, blind `DR` right after a DMA CI_XFER armed in
    //   MSG_OUT over a 6-byte FIFO (08 00 7f fe 01 00) —
    //     mame      00 7f fe 01 00 00 ...   (exactly one byte sent)
    //     rtl (old) 00 00 00 00 00 00 ...   (all six sent)
    //   and with the RTL-only `GAP 55` removed our old RTL returned
    //   08 00 7f fe 01 00 (none sent) — bracketing MAME's one-byte answer
    //   from both sides, which is what pins the pacing rather than the
    //   drain itself.
    // Same constant and same DMA-port-restart rule as the two timers
    // above.  A driver that stages its message bytes and then waits for
    // INTR without touching the DMA port still drains at one byte per
    // C96_RECV_QUIESCE, which is where the ROM's flows sit.
    reg [9:0]  c96_stop_feed_timer;
    // Did a SEND actually drop ATN?  MAME deasserts ATN inside INIT_XFR
    // only when the byte about to go out is the last one
    // (`remaining_bytes = fifo_pos + (dma ? tcounter : 0) == 1`,
    // ncr53c90.cpp:611-613); the target keeps the bus in MSG_OUT for as
    // long as ATN is up.  A FIFO emptied by HOST POPS (blind dma_r off
    // the aperture) therefore leaves ATN asserted and the target parked
    // in MSG_OUT — where our drain-end hook used to move it to COMMAND
    // unconditionally (scsi_fuzz seeds 32/39: stat 92 vs MAME's 96).
    reg        c96_stop_atn_dropped;
    // ── Deferred DMA CI_XFER receive (STATUS / MSG_IN forms) ──────────
    // MAME's INIT_XFR receive rides recv_byte() → delay_cycles(), a
    // timer that only fires when emulated time advances — so blind
    // dma_w beats issued back-to-back after the arm land in the FIFO
    // BEFORE the received byte, and fifo_push() silently DROPS the byte
    // if they filled it (scsi_fuzz seed 5).  These regs model that: the
    // pull is armed at dispatch and fires only once the DMA port has
    // been quiet for C96_RECV_QUIESCE pb cycles (any shim access
    // restarts the countdown).  512 cycles sits far above a burst's
    // inter-beat gap and any generator GAP (<= 64), and far below a
    // SETTLE, matching where MAME's timer actually fires.  A driver
    // only ever sees the byte arrive a few hundred cycles "late", which
    // is indistinguishable from real chip/bus latency.
    localparam [9:0] C96_RECV_QUIESCE = 10'd512;
    reg        c96_xfr_recv_pend;   // an armed DMA CI_XFER owes one recv
    reg        c96_xfr_recv_isstat; // 1 = status byte (target advances
                                    // to MSG_IN when it fires)
    reg [9:0]  c96_xfr_recv_timer;  // quiesce countdown
    reg [3:0]  c96_xfr_phase;      // Bus phase LATCHED when CI_XFER was
                                   // issued — MAME's `xfr_phase`
                                   // (ncr53c90.h / ncr53c90.cpp:601-651),
                                   // which is what INIT_XFR switches on and
                                   // what INIT_XFR_WAIT_REQ compares the
                                   // live phase against to detect a phase
                                   // change.  Consulted ONLY by the
                                   // STATUS/MSG_IN receive beats added
                                   // 2026-08-08; the DATA_IN and DATA_OUT
                                   // beats deliberately still gate on the
                                   // live `phase`, so this cannot alter any
                                   // hardware-validated path.  Latching
                                   // matters: without it a 0x10 issued in
                                   // DATA_IN would fire a STATUS push the
                                   // moment the target advanced to STATUS,
                                   // silently changing existing behaviour.
    // ── Deferred-CDB select (Q700 ROM boot-scan flow) ─────────────────
    // Golden MAME 0.285 macqd700 trace (2026-07-15): every boot-scan
    // transaction issues DMA|CD_SELECT (0xC1, tcount=1) with an EMPTY
    // FIFO.  MAME's start_command() arbitrates + selects immediately
    // (ncr53c90.cpp:978-989), the target enters COMMAND phase, and only
    // THEN does the ROM stuff the CDB — all but the last byte through
    // FIFO writes, the last byte through the DAFB pseudo-DMA port
    // (ROM 0x40898e86..0x40898eb4).  The ROM gates on seq_step!=0 &&
    // status[2:0]!=0 (ROM 0x40898e12 loop) before pushing bytes.
    reg        c96_sel_active;    // select accepted, CDB assembling
    reg        c96_sel_dma;       // select was DMA form (0xC1) — DRQ
                                  // asserts so the DAFB shim can carry
                                  // the CDB tail (MAME dma_set(DMA_OUT))
    reg [5:0]  c96_sel_len;       // expected CDB length once byte 0 seen
    // ── Progressive CDB drain (System 7.0.1 SCSI Manager flow) ────────
    // The RAM-resident SCSI Manager the System file installs at
    // "Welcome to Macintosh" time uses the SAME deferred-select shape as
    // the ROM (DMA|CD_SELECT, tcount=1, empty FIFO, CDB bytes 0..n-2 via
    // FIFO writes, tail byte via the DAFB pseudo-DMA port) but — unlike
    // the ROM — busy-waits BETWEEN the FIFO writes and the tail byte for
    // the chip to hand the FIFO'd bytes to the target:
    //     moveq #31,d1 ; and.b (0x70,a3),d1 ; beq done   ; FIFO count
    //     moveq #7,d1  ; and.b (0x40,a3),d1               ; phase bits
    //     cmpi.b #2,d1 ; beq loop                         ; still COMMAND
    // On a real 53C96 (and MAME's ncr53c90) the select sequence REQ/ACKs
    // each FIFO byte to the target as it arrives, so the count falls
    // back to 0 almost immediately.  The pre-2026-07-21 model instead
    // ACCUMULATED the bytes in the visible FIFO until the full CDB had
    // arrived — the drain-poll spun forever on fifo_pos==n-1/phase==2
    // (the post-"Welcome to Macintosh" HW boot wedge, parked in low-RAM
    // driver code ~0x29AE0).  Model the drain as instantaneous: bytes
    // land in cdb[c96_sel_idx] directly and the FIFO stays empty.
    reg [5:0]  c96_sel_idx;       // CDB bytes already accepted ("sent")
    // ATN-form deferred select (DMA|CD_SELECT_ATN, empty FIFO): the real
    // chip enters MSG_OUT first and sends exactly one message byte
    // (IDENTIFY) from the FIFO before the target switches to COMMAND.
    // The System SCSI Manager polls status[2:0]==110 (MSG_OUT), writes
    // the IDENTIFY byte to the FIFO, then re-polls for COMMAND.  While
    // this flag is set the status read reports MSG_OUT and the next
    // FIFO byte is consumed as the message (not counted toward the CDB).
    reg        c96_sel_msg_out;
    // ── ATN_STOP halt + Transfer-Information CDB feed (finding F2) ────
    // CD_SELECT_ATN_STOP sends exactly ONE message byte and halts with
    // ATN still asserted: seq=2, I_FUNCTION|I_BUS, the bus reporting
    // MSG_OUT, and the rest of the FIFO retained (MAME
    // DISC_SEL_ATN_SEND_BYTE, ncr53c90.cpp:537-541).  A subsequent
    // Transfer Information drains message bytes (ATN drops before the
    // last one), the target then processes the collected messages
    // (IDENTIFY LUN check) and moves to COMMAND — where the CDB arrives
    // through further Transfer Informations, NOT through bare FIFO
    // pushes (those just stack in the FIFO once the select command has
    // completed).
    reg        c96_sel_stopped;   // halted in MSG_OUT after the message
    reg        c96_sel_atn_stop;  // deferred select was the 0x43 form
    reg [7:0]  c96_stop_msg0;     // first message byte of an ATN_STOP
    // Last IDENTIFY received on this connection (MAME nscsi_full_device
    // latches scsi_identify and each COMMAND checks the LUN itself —
    // bad LUNs consume the whole CDB and only THEN CHECK CONDITION;
    // nscsi_bus.cpp:839-859, nscsi_hd.cpp:220-225).
    reg [2:0]  c96_identify_lun;
    // CI_COMPLETE dispatched while the target is still driving DATA IN:
    // MAME's recv_byte() pulls a byte off the LIVE phase whatever it is
    // (ncr53c90.cpp:1011-1015), so the byte that lands in the FIFO is the
    // next REAL data byte — not the 0x00 an out-phase produces.  The
    // hand-over has to run through the data-supply path, so the dispatch
    // arm only sets this pend flag and c96_cpt_din_beat does the push.
    reg        c96_cpt_din_pend;
    // MSG_IN byte consumed with ACK still asserted (MAME
    // INIT_XFR_RECV_BYTE_NACK / INIT_CPT_RECV_BYTE_NACK): the target
    // cannot present another REQ until CI_MSG_ACCEPT drops ACK, so a
    // further Transfer Information in MSG_IN hangs silently (armed, no
    // interrupt) instead of re-pulling the byte (scsi_fuzz seed 13:
    // MAME answers the 3rd+ 0x10 with istatus 00 forever).
    reg        c96_msgin_ack_held;
    // MAME's dma_dir + dma_command survive their command's completion in
    // ways the fuzzer proved architecturally visible (finding F11): an
    // ILLEGAL command leaves them untouched, so DRQ can stay derived
    // from a STALE direction, and every pseudo-DMA access is a blind
    // dma_r()/dma_w() against the FIFO with tcounter side effects
    // (scsi_ctrl=0 aperture; ncr53c90.cpp:1184-1206).
    localparam [1:0] C96_DIR_NONE = 2'd0,
                     C96_DIR_IN   = 2'd1,
                     C96_DIR_OUT  = 2'd2;
    reg [1:0]  c96_dma_dir;
    reg        c96_last_cmd_dma;   // MAME dma_command: bit 7 of the last
                                   // command that PASSED validity
    reg        c96_drq_stale;      // latched check_drq() for the FIFO-
    reg        c96_drq_norecompute_q; // last cycle's fifo_pos change came
                                   // from FLUSH/RESET (no check_drq in
                                   // MAME; see the recompute block)
                                   // mediated (stale-direction) paths
    reg [4:0]  c96_fifo_pos_q;     // previous-cycle FIFO occupancy —
    reg        c96_tc0_q;          // check_drq re-evaluates whenever a
                                   // FIFO/TC0-touching op ran
    reg        c96_cmd_wait;      // connected, target waiting for a CDB
                                  // fed by Transfer Information
    reg [7:0]  c96_sel_seq_final; // seq_step the select-complete hook
                                  // reports: 4 = everything sent + FIFO
                                  // empty, 2 = residue retained or DMA
                                  // count outstanding (MAME
                                  // DISC_SEL_WAIT_REQ, :548-556)

    // ── ATN-form select carrying an INLINE IdENTIFY byte ──────────────
    // Per the 53C9x contract (and MAME's ncr53c90, this model's cited
    // reference) CD_SELECT_ATN selects, sends ONE message byte popped
    // from the FIFO -- the IDENTIFY -- and only THEN treats the rest of
    // the FIFO as the CDB.  c96_sel_msg_out above models that for the
    // EMPTY-FIFO deferred select the ROM boot scan uses.  It does NOT
    // cover the case where the driver preloads IDENTIFY *and* the CDB
    // together, which is what SCSI Manager 4.3 does once it wants
    // disconnect privileges -- and both non-empty-FIFO select paths then
    // latched the IDENTIFY as cdb[0], decoding e.g. a READ(10) as opcode
    // 0xC0 and rejecting it at the invalid-opcode arm with CHECK
    // CONDITION and no backend dispatch at all.
    //
    // HOW THE MESSAGE BYTE IS IDENTIFIED, and why it is a test and not
    // an unconditional pop: this model never sees the ATN line handshake
    // that tells the real chip a message byte is present, so it infers
    // one from the byte's defining property -- a SCSI message byte has
    // bit 7 set (IDENTIFY is 0x80 | flags), while every CDB opcode Mac OS
    // issues on this path is below 0x80.  Popping unconditionally would
    // corrupt the ATN-form selects that carry a bare CDB, which are
    // already exercised green by tb_scsi_c96_read6.
    //
    // LIMITATION, stated rather than left implied: CD_SELECT_ATN_STOP is
    // still treated as CD_SELECT_ATN once the message byte is stripped.
    // The real chip halts in MSG_OUT after the message and waits for the
    // CDB to be fed separately; modelling that halt is a separate change
    // and nothing observed on this machine depends on it.
    wire       c96_atn_form  = ((c96_cmd_cur & 8'h7f) != CD_SELECT);
    // ATN-form selects send the FIRST FIFO byte as the MSG_OUT byte
    // whatever its value — the real chip has no notion of "looks like an
    // IDENTIFY" (MAME DISC_SEL_ATN_WAIT_REQ + send_byte; scsi_fuzz
    // finding F1: a bare-CDB SELECT_ATN sends the opcode as a garbage
    // message and the target parses the REST as the CDB).
    wire       c96_sel_strip = c96_atn_form && (c96_fifo_pos != 5'd0);
    // IDENTIFY naming a LUN this target doesn't have: the target goes
    // straight to STATUS / CHECK CONDITION without entering COMMAND
    // phase, leaving the CDB in the FIFO (measured against MAME 0.285
    // nscsi_harddisk, scsi_fuzz seed 4).
    wire       c96_sel_badlun = c96_sel_strip && c96_fifo[0][7] &&
                                (c96_fifo[0][2:0] != 3'b000);
    wire [3:0] c96_sel_base  = c96_sel_strip ? 4'd1 : 4'd0;
    wire [7:0] c96_sel_op    = c96_fifo[c96_sel_base];
    // CDB length for the post-strip opcode, as a WIRE rather than a bare
    // c96_cdb_len() call.  Vivado rejects a bit-select applied directly to a
    // function call ("select on function call violates IEEE 1800 syntax",
    // Synth 8-12513) even though Verilator accepts it -- so a synth-only
    // build break is invisible to `make lint` and every testbench.  Selecting
    // off a wire is legal everywhere; keep it that way.
    wire [5:0] c96_sel_cdblen = c96_cdb_len(c96_sel_op);
    wire [4:0] c96_sel_avail = c96_fifo_pos - {1'b0, c96_sel_base};
    // ── DMA CI_XFER chunk bookkeeping ─────────────────────────────────
    // Per MAME (ncr53c90.cpp RECV_WAIT_SETTLE: "tcount is decremented
    // on ACKO, not DACK") the transfer counter decrements as the CHIP
    // accepts bytes from the target — NOT when the CPU drains the DMA
    // port.  The Q700 ROM polls status for TC0 BEFORE draining each
    // 16-byte chunk (ROM 0x408992d2), then expects I_BUS once its DMA
    // reads finish (INIT_XFR_BUS_COMPLETE fires on TC0 + drq low even
    // though the target still sits in the data phase).
    reg [16:0] c96_xfr_left /* verilator public_flat_rd */;    // CPU beats remaining in this chunk
    reg [9:0]  c96_accept_pend /* verilator public_flat_rd */; // chip-accepted, not-yet-drained bytes
    // 53C9x command-register opcodes (subset used by macquadra700)
    localparam [7:0] CM_NOP             = 8'h00;
    localparam [7:0] CM_FLUSH_FIFO      = 8'h01;
    localparam [7:0] CM_RESET           = 8'h02;
    localparam [7:0] CM_RESET_BUS       = 8'h03;
    localparam [7:0] CD_RESELECT        = 8'h40;
    localparam [7:0] CD_SELECT          = 8'h41;
    localparam [7:0] CD_SELECT_ATN      = 8'h42;
    localparam [7:0] CD_SELECT_ATN_STOP = 8'h43;
    localparam [7:0] CD_ENABLE_SEL      = 8'h44;
    localparam [7:0] CD_DISABLE_SEL     = 8'h45;
    localparam [7:0] CD_SELECT_ATN3     = 8'h46;
    localparam [7:0] CT_SEND_STATUS     = 8'h21;
    localparam [7:0] CT_RECV_CMD        = 8'h29;
    localparam [7:0] CT_RECV_CMD_SEQ    = 8'h2b;
    localparam [7:0] CI_XFER            = 8'h10;
    localparam [7:0] CI_COMPLETE        = 8'h11;
    localparam [7:0] CI_MSG_ACCEPT      = 8'h12;
    // Status sticky-bit masks (matches MAME S_GROSS_ERROR/S_PARITY/S_TCC/S_TC0)
    localparam [7:0] S_GROSS  = 8'h40;
    localparam [7:0] S_PARITY = 8'h20;
    localparam [7:0] S_TC0    = 8'h10;
    localparam [7:0] S_TCC    = 8'h08;
    // istatus bits (matches MAME I_*)
    localparam [7:0] I_SCSI_RESET = 8'h80;
    localparam [7:0] I_ILLEGAL    = 8'h40;
    localparam [7:0] I_DISCONNECT = 8'h20;
    localparam [7:0] I_BUS        = 8'h10;
    localparam [7:0] I_FUNCTION   = 8'h08;
    localparam [7:0] I_RESELECTED = 8'h04;
    localparam [7:0] I_SELECT_ATN = 8'h02;
    localparam [7:0] I_SELECTED   = 8'h01;
    // ── Command dispatch plumbing (MAME 2-deep queue; finding F8) ────
    // A command executes either the cycle it is written (queue empty)
    // or the cycle the istatus read retires its predecessor
    // (command_pop_and_chain).  One decoder serves both paths;
    // c96_cmd_cur selects the byte being dispatched.
    wire c96_reg3_wr_now = TURBOSCSI_C96_EN && pb_wr && !pb_dma_shim &&
                           (pb_addr[3:0] == 4'h3);
    // RESET / RESET_BUS bypass the queue — but ONLY when there is a slot
    // to load them into.  MAME command_w (ncr53c90.cpp:887-903) tests
    // `command_pos == 2` FIRST and returns with S_GROSS_ERROR; the
    // `command_pos = 0` reset special-case is reached only afterwards.
    // The datasheet wording ("execute as soon as they are loaded into
    // the top of the Command Register") agrees: a full register loads
    // nothing, so nothing executes.  Ordering these the other way round
    // let a wedged chip be reset out of a full queue, which MAME (and
    // the part) refuse — scsi_fuzz seeds 13/14/22/23.
    wire c96_cmd_is_reset_form = c96_reg3_wr_now &&
         (c96_command_pos != 2'd2) &&
         (((pb_wdata & 8'h7f) == CM_RESET) ||
          ((pb_wdata & 8'h7f) == CM_RESET_BUS));
    wire c96_cmd_dispatch_pop = TURBOSCSI_C96_EN && c96_istatus_rd_now &&
                                (c96_istatus != 8'h00) &&
                                (c96_command_pos == 2'd2);
    // MAME command_w (ncr53c90.cpp:887-905) checks the FULL-QUEUE case
    // FIRST and returns; only then does the RESET/RESET_BUS special case
    // force command_pos = 0.  So a chip already sitting at command_pos ==
    // 2 drops EVERY subsequent write — a CM_RESET included — with
    // S_GROSS_ERROR, and stays wedged until an istatus read pops the
    // queue.  We had the two tests the other way round, so a
    // normalization `W 3 02` un-wedged our chip while MAME stayed at
    // cmd=10:2 / stat=52 (scsi_fuzz seed 27, SYNC 4).
    wire c96_cmd_dispatch_wr  = c96_reg3_wr_now &&
                                (c96_command_pos != 2'd2) &&
                                (c96_cmd_is_reset_form ||
                                 (c96_command_pos == 2'd0));
    wire c96_cmd_dispatch = c96_cmd_dispatch_wr || c96_cmd_dispatch_pop;
    wire [7:0] c96_cmd_cur = c96_cmd_dispatch_pop ? c96_cmd_q1 : pb_wdata;
    // istatus value a dispatched command builds on: the pop path's read
    // has already cleared istatus this same cycle (MAME istatus_r
    // clears, THEN pop_and_chain starts the successor)
    wire [7:0] c96_istatus_base = c96_cmd_dispatch_pop ? 8'h00 : c96_istatus;
    // Connected-as-initiator (MAME mode == MODE_I): from the instant a
    // selection succeeds until disconnect.  A select waiting on its
    // timeout is still disconnected, exactly like MAME (mode flips at
    // ARB_DESKEW_WAIT only when a target answered).
    wire c96_mode_i = c96_xfer_active || c96_sel_active ||
                      c96_sel_stopped || c96_cmd_wait;
    // 53c90a check_valid_command (ncr53c90.cpp:1300), MODE_T pruned —
    // this chip is never selected as a target.  Invalid -> I_ILLEGAL,
    // command still occupies its queue slot until the istatus read.
    function c96_cmd_valid_f;
        input [7:0] cmd;
        input       mode_i;
        begin
            case (cmd[6:4])
                3'd0: c96_cmd_valid_f = (cmd[3:0] <= 4'd3);
                3'd1: c96_cmd_valid_f = mode_i &&
                                        ((cmd[3:0] <= 4'd2)  ||
                                         (cmd[3:0] == 4'd8)  ||
                                         (cmd[3:0] == 4'd10) ||
                                         (cmd[3:0] == 4'd11));
                3'd4: c96_cmd_valid_f = !mode_i && (cmd[3:0] <= 4'd6);
                default: c96_cmd_valid_f = 1'b0;
            endcase
        end
    endfunction
    wire c96_cmd_valid = c96_cmd_valid_f(c96_cmd_cur, c96_mode_i);
    // Execute-and-chain commands: never occupy a queue slot (MAME runs
    // command_pop_and_chain at the end of their start_command arm)
    function c96_cmd_autopop;
        input [7:0] cmd;
        begin
            case (cmd & 8'h7f)
                CM_NOP, CM_FLUSH_FIFO,
                8'h1a, 8'h1b,          // CI_SET_ATN / CI_RESET_ATN
                CD_ENABLE_SEL, CD_DISABLE_SEL:
                    c96_cmd_autopop = 1'b1;
                default:
                    c96_cmd_autopop = 1'b0;
            endcase
        end
    endfunction
    // Observed bus view: reg 4 / reg 5 reads come from the
    // bus_status_read() / bus_and_stat_read() functions below — they
    // derive directly from `phase` + initiator inputs, so the previous
    // bus_status / bus_and_stat shadow registers are redundant and were
    // stripped to remove their dead-FF Vivado warnings.
    reg [7:0] cur_data;        // reg 0 read  (also reg 6 for DMA IN)
    reg       irq_pending;
    reg       end_dma_pending; // completion / end-of-transfer latch
    // ══════════════════════════════════════════════════════════════════
    // Initiator-visible signal extractors
    //   - SEL on initiator cmd -> Mac has asserted SEL
    //   - RST on initiator cmd -> bus reset
    //   - BSY on initiator cmd -> initiator assertion (pre-selection)
    //   - ACK on initiator cmd -> handshake for REQ/ACK
    // ══════════════════════════════════════════════════════════════════
    wire init_rst = r_init_cmd[7];
    wire init_bsy = r_init_cmd[3];
    wire init_sel = r_init_cmd[2];
    wire init_atn = r_init_cmd[1];
    wire init_ack = r_init_cmd[4];
    // ══════════════════════════════════════════════════════════════════
    // Phase FSM
    // ══════════════════════════════════════════════════════════════════
    localparam [3:0]
        S_BUS_FREE    = 4'd0,
        S_SELECT      = 4'd1,    // we asserted BSY, wait for Mac to drop SEL
        S_COMMAND     = 4'd2,    // drain CDB bytes
        S_CMD_EXEC    = 4'd3,    // interpret CDB, maybe kick volume
        S_DATA_IN     = 4'd4,    // pump data from sector buffer to Mac
        S_DATA_OUT    = 4'd5,    // pull data from Mac to sector buffer
        S_VH_WAIT_RD  = 4'd6,    // wait for the provider read to fill buffer
        S_VH_WAIT_WR  = 4'd7,    // wait for the provider write to commit buffer
        S_STATUS      = 4'd8,    // send 1 status byte
        S_MSG_IN      = 4'd9,    // send 1 command-complete message byte
        S_DISCONNECT  = 4'd10,   // release bus → BUS_FREE
        S_RESET       = 4'd11,   // RST line seen; flush everything
        // Disconnect / reselection states — per MAME ncr53c90.cpp:972-989
        // (CD_RESELECT command handler) and ncr53c90.h:153-158 interrupt
        // bits.  Supports MESSAGE OUT: DISCONNECT (0x04) suspend +
        // CD_RESELECT (0x40) resume so Mac OS doesn't hang on long-latency
        // SCSI I/O.  See top-of-file commentary for details.
        S_MSG_OUT     = 4'd12,   // receive 1 message byte from initiator
        S_RESELECT    = 4'd13,   // re-arbitrate, drive ID on data bus
        S_RESELECT_ID = 4'd14,   // ID phase: data bus = (1<<sel_tgt_id)
        S_RESELECT_MSG = 4'd15;  // send IDENTIFY (0x80) and resume DATA_*
    reg [3:0] phase /* verilator public_flat_rd */;
`ifdef VERILATOR
    reg [3:0] phase_prev;
    reg [15:0] dbg_reg4_polls;
    reg [15:0] dbg_reg5_polls;
    reg [15:0] dbg_reg7_polls;
    reg [7:0]  dbg_last_reg4;
    reg [7:0]  dbg_last_reg5;
    reg        dbg_last_irq;
    reg        dbg_select_seen;
`endif
    // ── CDB capture buffer (up to 10 bytes for READ10/WRITE10) ────────
    reg [7:0] cdb [0:9];
    reg [3:0] cdb_idx;
    reg [3:0] cdb_len;   // 6 or 10
    // ── Backing-store block size ──────────────────────────────────────
    // From the vhdd contract (rtl/vhdd.vh): the volume is addressed in
    // VHDD_BLOCK_BYTES-byte blocks, and the sector ring / byte counters /
    // ring high-water mark below are all sized from it.  The 9-bit ring
    // pointers and the explicit 9'd511 wrap tests are NOT parameterised —
    // they assume the same 512 and wrap on their own.
    localparam [15:0] BLOCK_BYTES     = 16'd`VHDD_BLOCK_BYTES;
    localparam [9:0]  RING_FULL       = 10'd`VHDD_BLOCK_BYTES;
    // 16 bytes of slack for the provider's own in-flight pipeline.
    localparam [9:0]  RING_HIGH_WATER = RING_FULL - 10'd16;
    // WRITE-ring arm threshold, evaluated ON a producer beat.  The beat
    // that consults it has ALREADY pushed a byte this cycle, so the
    // occupancy it must reason about is vh_buf_count + 1 and the byte it
    // is deciding to arm would make it + 2.  Comparing the pre-push
    // vh_buf_count against RING_FULL therefore lets exactly one byte too
    // many through — the ring hits 512 and the NEXT beat overwrites
    // sec_buf[vh_drain_ptr].  Measured: ~700 overwrites in a 4-block
    // CMD25 with an otherwise correct guard, one per drain event.
    // The re-arm clause in S_DATA_OUT (fired on a cycle with NO push)
    // correctly compares against RING_FULL instead.
    localparam [9:0]  RING_ARM_MAX    = RING_FULL - 10'd1;
    // ── Sector buffer (single 512-byte) ───────────────────────────────
    // Inferrable BRAM: single port read+write.
    //
    // For multi-block transfers the buffer is treated as a 512-byte
    // ring: provider writes (vh_fill_ptr) and initiator reads (buf_rd_ptr)
    // wrap mod 512 and run concurrently, with vh_buf_count tracking
    // bytes-available for back-pressure.  Single-block paths still
    // operate sequentially (fill → drain) and ignore the counter.
    // 2026-08-01 — sec_buf is now a genuinely inferrable RAM.
    //
    // It used to carry (* ram_style = "block" *) and Vivado ignored that
    // attribute 512 times per synthesis run ([Synth 8-7186] "Applying
    // attribute ram_style = "block" is ignored, object 'sec_buf[N]' is
    // not inferred as ram due to incorrect usage").  The cause was the
    // ~60 CONSTANT-index writes that pre-loaded the canned INQUIRY /
    // REQUEST SENSE / MODE SENSE / READ CAPACITY payloads into
    // sec_buf[0..35] at S_CMD_EXEC, mixed in with the dynamic ports:
    // that write shape has no RAM template, so the whole array fell back
    // to 4,096 discrete flops plus the 512:1 read-mux trees and 512-way
    // write-enable decoders that go with them.  On the 2026-07-31 routed
    // build u_scsi was 6,002 LUTs / 4,732 FFs; sec_buf is 4,096 of those
    // FFs and the bulk of the LUTs.
    //
    // Two changes make the template legal, both behaviour-preserving:
    //   1. the canned payloads moved into the combinational `canned_byte`
    //      ROM below and are muxed onto the port-A read path instead of
    //      being pre-loaded (same bytes, same order, same cycle);
    //   2. every remaining dynamic write funnels through the single
    //      sec_we/sec_wa/sec_wd port applied at the bottom of the main
    //      always block.
    //
    // "distributed", not "block": the port-A read feeding `pb_rd_mux`
    // (the S_DATA_IN pseudo-DMA readback) is genuinely COMBINATIONAL and
    // a BRAM would add a cycle of read latency there.  Two async read
    // ports (buf_rd_ptr, vh_drain_*) + one write port is exactly the
    // Xilinx dual-port distributed-RAM template.
    (* ram_style = "distributed" *) reg [7:0] sec_buf [0:511];
    reg [8:0] buf_rd_ptr;        // byte index for DATA_IN stream to initiator
    reg [8:0] buf_wr_ptr;        // byte index for DATA_OUT stream from initiator
    reg [8:0] vh_fill_ptr;       // byte index for volume→buf fill
    reg [8:0] vh_drain_ptr;      // byte index for buf→volume drain
    reg [9:0] vh_buf_count /* verilator public_flat_rd */;
    // ── Write-ring underflow, sticky per WRITE command ────────────────
    // Set when the provider takes a byte the initiator never supplied
    // (see the vh_buf_count net-update in S_DATA_OUT).  The count itself
    // CLAMPS at 0 rather than wrapping — a wrapped 10-bit count is
    // strictly worse — and clamping ALONE is what made this data-loss
    // event invisible.  This flag is what stops it being absorbed
    // silently: it is REAL rtl (not `ifdef VERILATOR), and it turns the
    // command's completion into CHECK CONDITION / HARDWARE ERROR /
    // WRITE ERROR in S_VH_WAIT_WR.  A retryable I/O error the OS can see
    // beats corrupt data reported as GOOD.
    // Cleared at reset, at bus reset, at C96 select, and at
    // WRITE(6)/WRITE(10) dispatch, so it describes exactly one command.
    reg       vh_wr_underflow /* verilator public_flat_rd */;
                                 // ring-buffer fill level (0..512); used
                                 // for multi-block back-pressure
    // ── REQ/ACK handshake state ───────────────────────────────────────
    // We assert REQ; initiator pulses ACK; we de-assert REQ until ACK drops,
    // then repeat until the whole transfer completes.
    //
    // Target bus phase (MSG/CD/IO) is derived from `phase` directly by
    // bus_status_read() / bus_and_stat_read(); the earlier per-state
    // t_msg / t_cd / t_io latches were redundant and Vivado removed
    // them.  Only t_req survives because the read-mux uses it to gate
    // the REQ bit during the half-cycle gap between drop and re-assert.
    reg t_req;
    reg ack_prev;
    wire ack_rise = init_ack & ~ack_prev;
    wire ack_fall = ~init_ack & ack_prev;
    // ATN edge detection — initiator asserts ATN to enter MSG_OUT phase
    // (per SCSI-2; MAME ncr53c90.cpp uses S_ATN to flag this).  We use
    // a rising edge so a steady ATN line latched during selection
    // doesn't keep retriggering the disconnect path.
    reg atn_prev;
    wire atn_rise = init_atn & ~atn_prev;
    // ── Transfer metrics ──────────────────────────────────────────────
    reg [31:0] xfer_lba;
    reg [23:0] xfer_blocks;      // blocks remaining to transfer
    reg [7:0]  xfer_status;      // 0x00 GOOD / 0x02 CHECK CONDITION
    reg [7:0]  xfer_msg;         // 0x00 COMMAND COMPLETE
    reg [15:0] xfer_bytes_left;  // bytes remaining IN CURRENT PHASE
    reg        vh_kicked;        // 1 once vh_req_go fired in this volume phase
    reg [15:0] vh_wait_ctr;      // timeout guard for absent backing store
    reg [23:0] vh_stuck_ctr;     // busy-independent stuck-supply watchdog
    reg        vh_supply_faulted; // sticky: stuck watchdog fired for this
                                  // transfer — forces vh_ring_staged so a
                                  // gated completion can never park
    reg        busy_error_pending;
    reg        medium_not_present;
    // ── REQUEST SENSE latched state ───────────────────────────────────
    reg [3:0]  sense_key;       // 0=NONE, 5=ILLEGAL REQ, 3=MEDIUM ERR, 6=UA
    reg [7:0]  sense_asc;       // additional sense code
    reg [7:0]  sense_ascq;
    // ── Disconnect / reselection suspend state ────────────────────────
    // Per MAME ncr53c90.cpp:972-989 (CD_RESELECT handler) and the SCSI-2
    // DISCONNECT message protocol: when the initiator asserts ATN during
    // a data phase and sends MSG_OUT 0x04, the target must save the
    // in-flight transfer state, send DISCONNECT back via MSG_IN, then
    // drop BSY and signal I_DISCONNECT (0x20).  When the initiator later
    // issues CD_RESELECT (0x40) the target re-arbitrates, drives its ID
    // on the data bus, and resumes from where it suspended.
    reg        disc_pending;          // 1 == we're holding a suspended xfer
    reg [3:0]  disc_resume_phase;     // S_DATA_IN or S_DATA_OUT
    reg [31:0] disc_xfer_lba;
    reg [23:0] disc_xfer_blocks;
    reg [15:0] disc_xfer_bytes_left;
    reg [8:0]  disc_buf_rd_ptr;
    reg [8:0]  disc_buf_wr_ptr;
    reg [8:0]  disc_vh_fill_ptr;
    reg [8:0]  disc_vh_drain_ptr;
    reg [9:0]  disc_vh_buf_count;
    reg [7:0]  disc_cdb_op;            // suspended CDB opcode (cdb[0])
    reg        disc_req_write;         // suspended vh_req_write
    reg        disc_req_multi;         // suspended vh_req_multi
    reg        disc_vh_kicked;         // suspended vh_kicked
    reg        ident_msg_sent;         // sticky during S_RESELECT_MSG
    reg        msg_out_byte_seen;      // received 1 byte in S_MSG_OUT
    // Pre-computed "last LBA" = NUM_LBAS - 1 (avoids bit-select on expr).
    wire [31:0] last_lba = vh_num_lbas - 32'd1;
    // ══════════════════════════════════════════════════════════════════
    // Canned DATA_IN payloads + sec_buf RAM ports
    // ══════════════════════════════════════════════════════════════════
    // REQUEST SENSE / INQUIRY / MODE SENSE 6 / READ CAPACITY 10 all
    // return a short fixed payload.  Those bytes used to be written into
    // sec_buf[0..35] by ~60 constant-index assignments at S_CMD_EXEC,
    // which is what defeated RAM inference on the whole array (see the
    // sec_buf declaration above).  They are now served straight out of
    // the `canned_byte` ROM on the port-A read path: identical bytes,
    // identical order, identical cycle, and no fill latency.
    reg        canned_active;   // 1 == port A serves canned_byte, not sec_buf
    reg [1:0]  canned_sel;
    localparam [1:0] CANNED_SENSE   = 2'd0,
                     CANNED_INQUIRY = 2'd1,
                     CANNED_MODE6   = 2'd2,
                     CANNED_CAP10   = 2'd3;
    // Snapshots taken at S_CMD_EXEC.  REQUEST SENSE clears sense_* in the
    // very cycle that used to populate sec_buf, so the ROM has to see the
    // PRE-clear values; READ CAPACITY snapshots last_lba for the same
    // "value as of command decode" reason.
    reg [3:0]  sns_key_q;
    reg [7:0]  sns_asc_q;
    reg [7:0]  sns_ascq_q;
    reg [31:0] cap_lba_q;
    // INQUIRY snapshot: the IDENTIFY named a LUN this target does not
    // have.  nscsi_hd.cpp:171-174 answers such an INQUIRY with byte 0 =
    // 0x7F (PERIPHERAL QUALIFIER 011b + PERIPHERAL DEVICE TYPE 1Fh, "not
    // capable of supporting a device on this LUN") instead of the 0x00
    // direct-access type, which is how a SCSI Manager bus scan learns
    // there is nothing behind LUNs 1..7 (scsi_fuzz seed 41).
    reg        inq_bad_lun_q;

    function [7:0] canned_byte;
        input [1:0] sel;
        input [8:0] idx;
        begin
            case (sel)
            // ── 18 B fixed-format REQUEST SENSE data ──────────────────
            CANNED_SENSE: begin
                case (idx)
                    9'd0:    canned_byte = 8'h70;              // response code
                    9'd2:    canned_byte = {4'h0, sns_key_q};
                    9'd7:    canned_byte = 8'd10;              // additional len
                    9'd12:   canned_byte = sns_asc_q;
                    9'd13:   canned_byte = sns_ascq_q;
                    default: canned_byte = 8'h00;
                endcase
            end
            // ── 36 B standard INQUIRY: "APPLE   " / "HD SC" / "0001" ──
            CANNED_INQUIRY: begin
                case (idx)
                    9'd0:                canned_byte = inq_bad_lun_q
                                                       ? 8'h7f   // no dev on LUN
                                                       : 8'h00;  // direct access
                    9'd1:                canned_byte = 8'h00;  // not removable
                    9'd2, 9'd3:          canned_byte = 8'h02;  // SCSI-2, resp fmt
                    9'd4:                canned_byte = 8'd31;  // additional len
                    9'd5, 9'd6, 9'd7:    canned_byte = 8'h00;
                    9'd8:                canned_byte = "A";
                    9'd9:                canned_byte = "P";
                    9'd10:               canned_byte = "P";
                    9'd11:               canned_byte = "L";
                    9'd12:               canned_byte = "E";
                    9'd16:               canned_byte = "H";
                    9'd17:               canned_byte = "D";
                    9'd19:               canned_byte = "S";
                    9'd20:               canned_byte = "C";
                    9'd32, 9'd33, 9'd34: canned_byte = "0";
                    9'd35:               canned_byte = "1";
                    // 13..15, 18, 21..31 are the space padding inside the
                    // vendor / product ID fields.
                    default:             canned_byte = " ";
                endcase
            end
            // ── 4 B MODE SENSE 6 parameter header (no pages) ──────────
            CANNED_MODE6: begin
                case (idx)
                    9'd0:    canned_byte = 8'h03;              // mode data len
                    default: canned_byte = 8'h00;
                endcase
            end
            // ── 8 B READ CAPACITY 10: last LBA (MSB first) + 512 ──────
            default: begin
                case (idx)
                    9'd0:    canned_byte = cap_lba_q[31:24];
                    9'd1:    canned_byte = cap_lba_q[23:16];
                    9'd2:    canned_byte = cap_lba_q[15: 8];
                    9'd3:    canned_byte = cap_lba_q[ 7: 0];
                    9'd6:    canned_byte = 8'h02;              // 512 = 0x0200
                    default: canned_byte = 8'h00;
                endcase
            end
            endcase
        end
    endfunction

    // ── Port A: the DATA_IN stream to the initiator ───────────────────
    // Feeds pb_rd_mux (combinational pseudo-DMA readback), cur_data, and
    // the C96 FIFO fill.  Overlaid with the canned ROM when the current
    // command's payload is one of the four fixed responses.
    wire [7:0] sec_rd_a = canned_active ? canned_byte(canned_sel, buf_rd_ptr)
                                        : sec_buf[buf_rd_ptr];

    // ── Port B: the buf→volume drain stream ───────────────────────────
    // Both drain sites (S_DATA_OUT's concurrent multi-block-write drain
    // and S_VH_WAIT_WR's single-block-write drain) pre-load the NEXT
    // byte on the same cycle they advance vh_drain_ptr, so the read address is the
    // post-advance pointer exactly when the advance fires.  S_VH_WAIT_WR
    // advances on vh_wr_ready alone; S_DATA_OUT additionally requires
    // vh_kicked.  9-bit arithmetic wraps at 512 on its own, so
    // vh_drain_next matches both the explicit ==511 form used in
    // S_DATA_OUT and the bare +1 used in S_VH_WAIT_WR.
    wire [8:0] vh_drain_next = (vh_drain_ptr == 9'd511) ? 9'd0
                                                        : (vh_drain_ptr + 9'd1);
    wire       vh_drain_pre  = vh_wr_ready &&
                               ((phase == S_VH_WAIT_WR) || vh_kicked);
    wire [7:0] sec_rd_b      = sec_buf[vh_drain_pre ? vh_drain_next
                                                    : vh_drain_ptr];

    // ── Single write port ─────────────────────────────────────────────
    // Driven by the `sec_wr_mux` combinational block at the bottom of
    // this file and consumed by the ONE array-write statement at the end
    // of the main always block.  The four dynamic write sites live in
    // mutually exclusive FSM phases, so the mux is a plain priority chain
    // and never drops a write; each original site carries an
    // `ifdef VERILATOR assertion that the mux agrees with it, and every
    // one of those assertions has a positive control (see the landing
    // commit message for which tb catches which site).
    reg        sec_we;
    reg [8:0]  sec_wa;
    reg [7:0]  sec_wd;
    // ── Backing-store addressability probe ────────────────────────────
    // The extent we are about to move, presented live to the provider.
    // It shrinks as blocks retire, and the provider's answer (vh_chk_ok)
    // is consulted every cycle we sit in a backing-store wait state — so
    // this is deliberately the LIVE xfer_* pair, not the latched request.
    assign vh_chk_lba    = xfer_lba;
    assign vh_chk_blocks = xfer_blocks;
    // Watchdog for a backing store that never answers.  Counted ONLY
    // while vh_busy is low: a provider is entitled to take as long as it
    // likes as long as it says it is working (rtl/vhdd.vh).
    localparam [15:0] VH_WAIT_TIMEOUT = 16'd1024;
    // ── STUCK-SUPPLY watchdog (busy-INDEPENDENT) 2026-09-07 ────────────
    // The VH_WAIT_TIMEOUT above and the S_DATA_IN "SUPPLY EXHAUSTED"
    // backstop both REQUIRE `!vh_busy`, and the backstop additionally
    // requires `!t_req` — which the C96 pseudo-DMA path holds HIGH for the
    // whole of a block.  So a provider that stalls mid-stream while
    // ASSERTING vh_busy (and never raising vh_done/vh_error) has NO bound
    // anywhere on the CPU→SCSI→vhdd→SD path: the SoC's ack watchdogs are
    // disabled by owner directive, the SCSI sd_ctrl runs REQ_WDOG_ENABLE=0
    // (2026-08-03 directive), and f8ad8c33's supply back-pressure then
    // withholds the pseudo-DMA beat forever — the CPU pins inside the
    // ROM's blind MOVE.W burst (HW boot-#2, ROM 0x40899664).  Root cause
    // is upstream (a CMD18 read killed by warm_storage_reset with no
    // completion leaves sd_scsi_bridge's pb_busy_internal latched, since
    // the bridge's transfer-loss rescue is coupled to warm_peripheral_
    // reset, a DIFFERENT signal) — that fix is not scsi.v's to make.  What
    // scsi.v OWNS is refusing to pin the CPU forever: a busy-independent
    // contiguous-starvation counter that, on expiry, completes the SCSI
    // command as CHECK CONDITION / NOT READY so the initiator gets its
    // interrupt and the phase leaves S_DATA_IN (which drops
    // c96_shim_rd_starved and releases the withheld beat).  This turns an
    // unrecoverable, debug-dark pin into a bounded, diagnosable error the
    // ROM's SCSI manager can act on.  It does NOT by itself un-stick the
    // bridge — a subsequent read still cannot kick while vh_busy is latched
    // — so it is a safety net, not the supply fix.
    //
    // Counts CONTIGUOUS cycles of "owe the initiator a byte, ring empty,
    // nothing arriving", reset the instant a byte lands (vh_rd_valid) or
    // we leave the read-delivery phases.  The bound is far larger than any
    // legitimate mid-stream inter-byte gap (sd_ctrl's own per-block token
    // timeouts bound a slow-but-clocking card); only a fully stalled
    // supply reaches it.  Shortened under Verilator so the repro exercises
    // the recovery path within a test's tick budget. Performance/card-latency
    // simulations define SCSI_REAL_TIMEOUTS to retain the hardware bound.
`ifdef VERILATOR
`ifdef SCSI_REAL_TIMEOUTS
    localparam [23:0] VH_STUCK_TIMEOUT = 24'd8_000_000;
`else
    localparam [23:0] VH_STUCK_TIMEOUT = 24'd50000;
`endif
`else
    localparam [23:0] VH_STUCK_TIMEOUT = 24'd8_000_000;   // ~160 ms @50 MHz
`endif
    // Decoded request class.  {vh_req_write, vh_req_multi} replaces what
    // used to be a raw SD command code, and these two wires stand in for
    // the "is this a multi-block read/write" tests that were spread
    // through the FSM.
    //
    // 2026-08-07 BUG FIX — System 7.5.3 "Starting up..." hang, HW-measured
    // on bitstream 0xDD5127F3 (tb/vectors/hw_scsi_trace_753_wedge.csv).
    // These tests used to read vh_req_multi/vh_req_write DIRECTLY.  Those
    // are PROVIDER-facing REQUEST descriptors: per rtl/vhdd.vh they are
    // valid on the req_go cycle and "stay stable until the next req_go",
    // so they are assigned ONLY in the READ/WRITE dispatch arms and are
    // deliberately left alone by every command that issues no volume
    // request at all (INQUIRY / REQUEST SENSE / READ CAPACITY / MODE
    // SENSE / TEST UNIT READY).  Reading them as if they described the
    // CURRENT data phase therefore inherited the PREVIOUS command's
    // class: an INQUIRY that followed a multi-block READ ran with
    // vh_multi_read still 1, so c96_avail_bytes (below) reported the
    // drained multi-block ring's occupancy (0) instead of the canned
    // payload's xfer_bytes_left (36).  c96_accept_ev could never fire,
    // tcounter never left 16, S_TC0 never set, the DMA chunk-completion
    // hook never raised I_BUS -- and the interrupt-driven SCSI Manager
    // 4.3, which issues no further register access at all while it waits,
    // waited forever.  Live trace ring, last 22 events before silence:
    //     W4=00, CDB 12 00 00 00 24 00, W3=41, R4=91, R5=18,
    //     W1=00 W0=10 (TC=16), W3=90, R4=01 ... nothing, ever again.
    // MAME has no such coupling: in initiator async DATA IN every byte
    // the target hands over is fifo_push()ed and decrement_tcounter()ed
    // unconditionally (ncr53c90.cpp:471-477), and the only thing that can
    // stop the chip receiving is a full FIFO (:625-627, `if (fifo_pos ==
    // 16) break;`) or the target dropping REQ.  What the chip did on some
    // previous command cannot enter into it.
    //
    // `data_from_ring` is the DATA-PHASE view of the same distinction:
    // "the bytes this data phase moves come from the concurrently-filled
    // 512-byte ring", which is true only for the command that actually
    // asked for a multi-block volume transfer.  It is cleared at the head
    // of S_CMD_EXEC (later NBA in the READ/WRITE arms wins, exactly like
    // canned_active) so it can never outlive its own command.  The
    // vh_req_* ports keep their documented provider contract untouched.
    reg  data_from_ring;
    wire vh_multi_read  = data_from_ring && !vh_req_write;
    wire vh_multi_write = data_from_ring &&  vh_req_write;
    function [7:0] bus_status_read;
        input [3:0] cur_phase;
        input        cur_init_sel;
        input        cur_init_rst;
        input        cur_t_req;
        begin
            if (cur_init_rst) begin
                bus_status_read = 8'h80;
            end else begin
                case (cur_phase)
                    S_BUS_FREE:   bus_status_read = 8'h00;
                    S_SELECT:     bus_status_read = 8'h40 | (cur_init_sel ? 8'h02 : 8'h00);
                S_COMMAND:    bus_status_read = 8'h40 | 8'h08 |
                                                (cur_t_req ? 8'h20 : 8'h00);
                S_CMD_EXEC:   bus_status_read = 8'h40;
                S_DATA_IN:    bus_status_read = 8'h40 | 8'h04 |
                                                (cur_t_req ? 8'h20 : 8'h00);
                S_DATA_OUT:   bus_status_read = 8'h40 |
                                                (cur_t_req ? 8'h20 : 8'h00);
                S_VH_WAIT_RD: bus_status_read = 8'h40;
                S_VH_WAIT_WR: bus_status_read = 8'h40;
                S_STATUS:     bus_status_read = 8'h40 | 8'h08 | 8'h04 |
                                                (cur_t_req ? 8'h20 : 8'h00);
                S_MSG_IN:     bus_status_read = 8'h40 | 8'h10 | 8'h08 | 8'h04 |
                                                (cur_t_req ? 8'h20 : 8'h00);
                // S_MSG_OUT: BSY|MSG|C/D (no I/O — outbound to target);
                // REQ-if-asserted.  Per MAME message-out phase.
                S_MSG_OUT:    bus_status_read = 8'h40 | 8'h10 | 8'h08 |
                                                (cur_t_req ? 8'h20 : 8'h00);
                S_DISCONNECT: bus_status_read = 8'h00;
                // Reselection: BSY held; I/O asserted (MAME ncr53c90.cpp:1071-1077,
                // arbitrate() drives BSY, then target asserts I/O with its ID).
                S_RESELECT,
                S_RESELECT_ID:   bus_status_read = 8'h40 | 8'h04;
                S_RESELECT_MSG:  bus_status_read = 8'h40 | 8'h10 | 8'h08 | 8'h04 |
                                                   (cur_t_req ? 8'h20 : 8'h00);
                default:      bus_status_read = 8'h00;
                endcase
            end
        end
    endfunction
    function [7:0] bus_and_stat_read;
        input [3:0] cur_phase;
        input [2:0] cur_tcmd_phase;
        input       cur_irq_pending;
        input       cur_end_dma_pending;
        input       cur_init_atn;
        input       cur_init_ack;
        input       cur_busy_error;
        input       cur_t_req;
        reg [2:0] cur_bus_phase;
        reg       cur_phase_live;
        begin
            bus_and_stat_read = 8'h00;
            cur_bus_phase = 3'b000;
            cur_phase_live = 1'b0;
            case (cur_phase)
                S_COMMAND: begin
                    cur_bus_phase = 3'b010;  // C/D
                    cur_phase_live = 1'b1;
                end
                S_DATA_IN: begin
                    cur_bus_phase = 3'b001;  // I/O
                    cur_phase_live = 1'b1;
                end
                S_DATA_OUT: begin
                    cur_bus_phase = 3'b000;  // DATA OUT
                    cur_phase_live = 1'b1;
                end
                S_STATUS: begin
                    cur_bus_phase = 3'b011;  // C/D | I/O
                    cur_phase_live = 1'b1;
                end
                S_MSG_IN: begin
                    cur_bus_phase = 3'b111;  // MSG | C/D | I/O
                    cur_phase_live = 1'b1;
                end
                S_MSG_OUT: begin
                    cur_bus_phase = 3'b110;  // MSG | C/D (no I/O — outbound)
                    cur_phase_live = 1'b1;
                end
                S_RESELECT_MSG: begin
                    cur_bus_phase = 3'b111;  // MSG | C/D | I/O — same as MSG_IN
                    cur_phase_live = 1'b1;
                end
                default: begin
                    cur_bus_phase = 3'b000;
                    cur_phase_live = 1'b0;
                end
            endcase
            if (cur_phase == S_DATA_IN || cur_phase == S_DATA_OUT) begin
                bus_and_stat_read = cur_t_req ? 8'h40 : 8'h00;
            end else if (cur_phase == S_COMMAND ||
                         cur_phase == S_STATUS  ||
                         cur_phase == S_MSG_IN) begin
                bus_and_stat_read = 8'h00;
            end
            if (cur_phase_live && (cur_bus_phase == cur_tcmd_phase))
                bus_and_stat_read = bus_and_stat_read | 8'h08;
            if (cur_end_dma_pending)
                bus_and_stat_read = bus_and_stat_read | 8'h80;
            if (cur_irq_pending) bus_and_stat_read = bus_and_stat_read | 8'h10;
            if (cur_busy_error)  bus_and_stat_read = bus_and_stat_read | 8'h04;
            if (cur_init_atn)    bus_and_stat_read = bus_and_stat_read | 8'h02;
            if (cur_init_ack)    bus_and_stat_read = bus_and_stat_read | 8'h01;
        end
    endfunction
    function [15:0] cap_alloc6;
        input [15:0] available;
        input [7:0]  alloc_len;
        begin
            cap_alloc6 = ({8'd0, alloc_len} < available)
                         ? {8'd0, alloc_len}
                         : available;
        end
    endfunction
    // ══════════════════════════════════════════════════════════════════
    // IRQ / DRQ lines
    // ══════════════════════════════════════════════════════════════════
    wire irq_live_5380 = irq_pending || (phase == S_STATUS) || (phase == S_MSG_IN);
    wire completion_live = end_dma_pending ||
                           (phase == S_STATUS) || (phase == S_MSG_IN);
    assign irq = TURBOSCSI_C96_EN ? c96_irq_pending : irq_live_5380;
    // DRQ generation matches NCR 53C9x check_drq() per ncr53c90.cpp:1207-1232
    // (bare 53C90 path) and ncr53c90.cpp:1374-1404 (53C94/C96 with BUSMD_1).
    // In our simplified pipeline the 16B FIFO is replaced by a 512B sec_buf,
    // so we collapse "fifo has data" / "fifo has space" to phase + t_req: a
    // pseudo-DMA beat is precisely the condition under which DRQ should be
    // asserted to the host.  Per MAME dma_set() (called from start_command()
    // only for DMA-form commands), DRQ must stay LOW until the initiator
    // actually issues a DMA transfer command (0x90) — the DAFB DRQ-check
    // (dafb.cpp:1001-1011) spins on this line, so asserting it before the
    // ROM arms the transfer would let pseudo-DMA beats fire out of order.
    // Three DRQ sources in C96 mode (MAME check_drq(), ncr53c90.cpp):
    //   • DMA-form select in progress (dma_set(DMA_OUT) at start_command):
    //     DRQ while the FIFO has space, so the DAFB shim can carry the
    //     CDB tail bytes — the ROM spins on a DAFB DRQ readback before
    //     the pseudo-DMA CDB write (ROM 0x40898ea8).
    //   • DMA data-in chunk: DRQ while the chip holds accepted-but-
    //     undrained bytes and the chunk still owes the CPU beats.
    //   • DMA data-out: phase + REQ (unchanged).
    // 2026-07-15 HW POST-MORTEM (defensive correctness fix, NOT the
    // confirmed root cause of that day's boot wedge — see
    // rtl/soc/peripheral_bus.v's rd_scsi_dma_shim_active comment for the
    // actual confirmed mechanism, a word/byte access-width bug one
    // layer up the stack).  The DATA_IN term below used to also
    // require live `t_req`.  t_req is the legacy bare-5380 REQ line,
    // whose multi-block-read assignment is recomputed after EVERY drained beat
    // from the RING's current refill state (`t_req <= (vh_buf_count >
    // 1) || vh_rd_valid;`) — a signal about whether the NEXT not-yet-
    // accepted byte is coming, not about whether THIS chunk's already-
    // chip-accepted bytes (c96_accept_pend) are sitting in sec_buf
    // ready to drain.  Direct sim instrumentation during this
    // investigation did NOT observe t_req actually glitching low
    // mid-burst under any tested pacing/latency combination, so this
    // was NOT what caused the 2026-07-15 hang.  Kept anyway: gating
    // drq_c96/pseudo_dma_in_beat/data-select on t_req is still
    // semantically wrong (conflates "is a NEW byte coming" with "is
    // THIS already-accepted byte available"), doesn't match MAME's
    // real 53C9x model (already-fetched-from-target bytes are
    // available to the host regardless of the target's current
    // REQ/ACK state for LATER bytes), and is a latent bug worth
    // closing even though it wasn't this session's active trigger.
    // 53C94/96 BUSMD_1 check_drq(), DMA_IN arm (ncr53c90.cpp:1374-1393,
    // macquadra700.cpp sets BUSMD_1):
    //   async (sync_offset==0): drq = fifo_pos > ((config3.2 || !TC0) ? 1 : 0)
    //   sync:                   drq = !TC0 && fifo_pos > 1
    // While dma_dir == DMA_IN every mutation of fifo_pos / TC0 goes
    // through a MAME call site that ends in check_drq() (fifo push/pop/
    // flush, decrement_tcounter, and dir only becomes IN via dispatch
    // paths that recompute), so MAME's LATCHED drq always equals this
    // live formula for the IN direction.  Using it live also removes the
    // one-cycle-late DRQ fall the event-latched c96_drq_stale had:
    // scsi_fuzz seed 5 measured 17 blind dma_r beats accepted where
    // MAME's DRQ falls synchronously on the 16th pop.
    wire c96_tc0_set = (c96_status_sticky & S_TC0) != 8'h00;
    wire c96_drq_fifo_in_live =
        (c96_sync_offset == 4'd0)
            ? (c96_fifo_pos > ((c96_config3[2] || !c96_tc0_set) ? 5'd1
                                                                : 5'd0))
            : (!c96_tc0_set && (c96_fifo_pos > 5'd1));
    wire drq_c96 =
        (c96_sel_active && c96_sel_dma && (c96_fifo_pos != 5'd16)) ||
        // MAME check_drq() result, LATCHED: MAME recomputes drq only
        // when a FIFO/tcounter operation calls check_drq(), NOT when
        // dma_set() changes the direction — so a timed-out DMA select
        // leaves DRQ low until the next FIFO op even though the OUT
        // formula would say otherwise (finding F11, scsi_fuzz seeds
        // 5/14/17).  c96_drq_stale below carries that latched value for
        // the OUT/NONE directions; the IN direction is live (above).
        ((c96_dma_dir == C96_DIR_IN) ? c96_drq_fifo_in_live
                                     : c96_drq_stale) ||
        // post-ATN_STOP DMA message drain / Transfer-Information CDB feed
        (c96_sel_stopped && c96_xfr_armed && c96_xfr_dma &&
         (c96_tcounter != 17'd0)) ||
        (c96_cmd_wait && c96_xfr_armed && c96_xfr_dma &&
         (c96_tcounter != 17'd0)) ||
        // (R1 rework: the old DATA_IN accept_pend/xfr_left term is gone —
        // DMA data-in DRQ is the dir==IN live FIFO formula above, since
        // the accepted bytes now sit in the real c96_fifo.)
        // MAME check_drq() DMA_OUT (ncr53c90.cpp:1391-1393) is
        //     drq = !(status & S_TC0) && fifo_pos < 15
        // The occupancy half was missing entirely: reg 7 advertised
        // "send me more" at any FIFO level, including levels where
        // fifo_push() would silently drop the byte.
        ((phase == S_DATA_OUT) && t_req && c96_xfr_armed && c96_xfr_dma &&
         (c96_tcounter != 17'd0) && (c96_fifo_pos < 5'd15));
    assign drq = TURBOSCSI_C96_EN ? drq_c96 :
                 (((phase == S_DATA_IN) || (phase == S_DATA_OUT)) && t_req);
    // ── Backing-store read-stream back-pressure ───────────────────────
    // The multi-block read stream used to be unpaced — the provider of
    // the day (an SD card over SPI) "could not be paused" — so any drain
    // stall longer than the ring's slack (a VBL/Timer ISR pre-empting the
    // ROM's chunked poll loop mid-read) let the incoming stream overwrite
    // undrained ring bytes past 512 of backlog, and wrap the 10-bit
    // vh_buf_count at 1024.  A wrapped
    // count mid-chunk-drain drops t_req, the ROM's remaining blind
    // DMA-port reads stop counting as beats, c96_xfr_left never reaches
    // zero and the chunk-completion I_BUS never fires — the 2026-07-15
    // HW boot wedge at ROM PC 0x40899322 (status bit-7 poll spinning
    // forever, tcounter==0 over JTAG).
    //
    // Fix: real consumer pacing.  vh_rd_ready is REAL back-pressure in
    // the vhdd contract (rtl/vhdd.vh) — a provider must stop the stream
    // at the source while it is low — so holding it low when the ring is
    // nearly full bounds the backlog.  The 16-byte margin covers the
    // provider's own pipeline (for the SD path: the sd_scsi_bridge CDC
    // ladder plus one in-flight SPI byte).  Multi-block reads only:
    // vh_buf_count is drained (decremented) only on the multi-block ring
    // path — non-ring sources (single-block reads, INQUIRY, ...) fill
    // exactly one fully-buffered block and
    // must not be paced by the stale count.  Also gated on the read
    // actually being live (S_VH_WAIT_RD/S_DATA_IN): if the initiator
    // ever abandons a multi-block read mid-stream (bus reset), the
    // stream must
    // drain to completion into the void — pre-fix behaviour — instead
    // of holding vh_busy forever under a permanent pause.
    //
    // HONEST FOLLOW-UP (same day): a fresh HW test of this exact fix
    // showed the boot wedge at 0x40899322 STILL reproduced, identical
    // PC/phase/symptom.  This ring-overrun close is a real, worthwhile
    // fix in its own right (an unpaced multi-KB-backlog read stream
    // absolutely can wrap vh_buf_count given enough stall), but it was
    // NOT the active trigger for the 2026-07-15 hang.  The confirmed
    // root cause is a word-vs-byte access-width bug one layer up the
    // stack — see rtl/soc/peripheral_bus.v's rd_scsi_dma_shim_active
    // comment.  Left in place as defense-in-depth.
    // ── SUPPLY PRE-STAGING (owner directive 2026-09-07) ───────────────
    // The ROM drains DMA data-in transfers as BLIND MOVE.W bursts with
    // no DRQ check (0x40899628 Duff run / 0x4089931c chunked variant),
    // assuming real-53C96 semantics where the FIFO always outruns the
    // CPU.  Our SD-class supply cannot promise that mid-burst, so the
    // latency must be moved to BEFORE the points that RELEASE the ROM
    // into a burst — the select-complete interrupt and each per-chunk
    // I_BUS — rather than into the middle of the burst as a withheld
    // beat.  Measured (tb-scsi-c96-stuck-supply --measure, SD at 16
    // pb-cyc/byte vs CPU at 26 pb-cyc/word): starvation is mid-block
    // steady-state — the drain outruns the stream and the ring dips
    // empty every few words once the initial headroom is consumed.
    //
    // Mechanism: a multi-block read is "staged" when the ring holds
    // RING_HIGH_WATER bytes (the provider's own pause point), or the
    // whole remaining payload if that is less, or the stream has closed
    // (nothing more will ever come — take what is there).  Staging
    // gates (1) S_VH_WAIT_RD -> S_DATA_IN entry (which is what releases
    // the select-complete interrupt), and (2) the DMA data-in chunk
    // completion I_BUS (which is what releases the ROM into the next
    // chunk's burst).  With the provider streaming during the drain,
    // every burst whose slack loss (chunk_bytes * deficit_ratio) fits
    // inside the headroom never sees a withheld beat at all; the
    // boot-scan 16-byte and 512-byte chunk flows fall in that class.
    // vh_supply_faulted (set by the stuck-supply watchdog) forces the
    // staged condition so a dead provider can never park a gated
    // completion — the command ends as CHECK CONDITION instead.
    //
    // Stage target: RING_HIGH_WATER, collapsing to the remaining payload
    // at the transfer tail so the gate never demands bytes that will
    // never exist.  The tail is only smaller than the high water when we
    // are inside the LAST block (xfer_blocks == 1) with less than
    // RING_HIGH_WATER of it left — any earlier, at least one whole
    // 512-byte block is still owed, which already exceeds the water
    // mark.  No wide adder needed.
    wire [9:0] vh_stage_target =
        ((xfer_blocks != 24'd1) ||
         (xfer_bytes_left > {6'd0, RING_HIGH_WATER}))
            ? RING_HIGH_WATER : xfer_bytes_left[9:0];
    wire vh_stream_closed = vh_kicked && !vh_busy && !vh_rd_valid;
    wire vh_ring_staged = !vh_multi_read ||
                          (vh_buf_count >= vh_stage_target) ||
                          vh_stream_closed ||
                          vh_supply_faulted;
    assign vh_rd_ready = (!vh_multi_read) ||
                         ((phase != S_VH_WAIT_RD) &&
                          (phase != S_DATA_IN)) ||
                         (vh_buf_count < RING_HIGH_WATER);
    assign vh_wr_valid = (phase == S_VH_WAIT_WR);
    // ── Backing-store write-stream back-pressure ──────────────────────
    // Symmetric twin of vh_rd_ready above, and closing the symmetric
    // hole.  The write ring is only *tracked* while the initiator and
    // the provider run concurrently — that is S_DATA_OUT on a
    // multi-block write, blocks 2..N of a CMD25.  Everywhere else the
    // whole payload the provider can still ask for is already sitting
    // in sec_buf:
    //   * single-block writes and MODE SELECT buffer the entire block
    //     before S_VH_WAIT_WR kicks the provider;
    //   * the TAIL of a multi-block write (S_VH_WAIT_WR, entered only
    //     after the last byte was pushed) still owes the provider the
    //     final block, all of it already written — vh_buf_count is not
    //     decremented in that state at all (it carries a residue), so
    //     gating on it there would deadlock a healthy transfer.
    // Hence: available unless we are actively co-running the ring and
    // it is empty.
    //
    // ONE CYCLE OF DELAY IS LOAD-BEARING.  The credit and the byte it
    // refers to have to arrive at the provider together, and the byte's
    // path is one register longer than the count's:
    //     posedge T   initiator beat writes sec_buf[p]; vh_buf_count++
    //     cycle  T+1  sec_buf[p] readable, count already updated
    //                 (a combinational credit would go high HERE)
    //     posedge T+1 vh_wr_data <= sec_rd_b  (the new byte)
    //     cycle  T+2  vh_wr_data finally carries it
    // Both leave this module through identical 2-FF ladders in
    // sd_scsi_bridge, so an undelayed credit reaches sd_ctrl exactly one
    // pb cycle before the data does, and sd_ctrl latches the PREVIOUS
    // byte.  Measured with the combinational form: a 4-block CMD25 with
    // a mid-block producer stall came out with exactly ONE wrong byte,
    // at the resume point.  Registering the credit aligns them.
    //
    // The falling edge is delayed by the same cycle, which is the safe
    // direction and already has slack: the drain path preloads
    // vh_wr_data from vh_drain_next ON the wr_ready cycle, so the NEXT
    // byte is on the wire one cycle BEFORE the decremented count shows
    // up in the credit.
    wire vh_wr_avail_comb = (phase != S_DATA_OUT) || (!vh_multi_write) ||
                            (vh_buf_count != 10'd0);
    reg  vh_wr_avail_q;
    always @(posedge clk) begin
        if (rst) vh_wr_avail_q <= 1'b1;
        else     vh_wr_avail_q <= vh_wr_avail_comb;
    end
    assign vh_wr_avail = vh_wr_avail_q;
    // ══════════════════════════════════════════════════════════════════
    // Peripheral-bus slave: read path (combinational mux, registered out).
    // ══════════════════════════════════════════════════════════════════
    wire pb_dma_shim = (pb_addr == 9'h100) || (pb_addr == 9'h101);
    // Same-cycle-access guards for the selection timer (see its block
    // comment).  A register access that disarms or re-arms the timeout
    // must win over the timer's own expiry in the cycle they collide.
    wire c96_reg_wr_now     = TURBOSCSI_C96_EN && pb_wr && !pb_dma_shim;
    wire c96_istatus_rd_now = TURBOSCSI_C96_EN && pb_rd && !pb_dma_shim &&
                              (pb_addr[3:0] == 4'h5);
    wire c96_seqstep_rd_now = TURBOSCSI_C96_EN && pb_rd && !pb_dma_shim &&
                              (pb_addr[3:0] == 4'h6);
    // Mirrors the read-side term of pb_ack's own withhold-gate below
    // (`TURBOSCSI_C96_EN && pb_dma_shim && scsi_ctrl_in[7] && !drq_c96`)
    // so peripheral_bus.v can hold off its scsi_rd pulse until an ack is
    // guaranteed, instead of firing blind and losing the beat forever
    // when it lands on a cycle where drq_c96 happens to be low.
    // Low half of a 16-bit DMA-port read whose high half was granted —
    // see c96_dma16_hi_granted's declaration.
    wire c96_dma16_lo_inherit = TURBOSCSI_C96_EN && pb_dma_shim &&
                                pb_dma16_lo_beat && c96_dma16_hi_granted;
    // ── SUPPLY STARVATION BACK-PRESSURE (2026-09-06) ──────────────────
    // A blind (scsi_ctrl[7]=0) pseudo-DMA burst can outrun the chip's
    // own supply: measured in tb-scsi-c96-mame-chunk's 32-byte chunk
    // scenario, the host drained the staged 16 bytes and then issued its
    // remaining 16 reads while c96_fifo_pos==0 and the chunk still owed
    // 16 bytes (c96_tcounter==16).  Those reads popped NOTHING and
    // returned the stale fifo[0], so (a) the block was silently
    // corrupted, and (b) the chip then staged its last 16 bytes with no
    // reader left, pinning fifo_pos=16 -> drq high -> the TC0 && !drq
    // completion below can never fire.  That is the boot wedge observed
    // on silicon at ROM 0x40899706 (status=0x11, INT clear, fifo=9).
    //
    // A real 53C96 cannot present this: its FIFO is refilled from the
    // SCSI bus far faster than the 68k can drain it, so a byte is always
    // there.  Our vhdd-backed supply has latency the real part does not,
    // which is an artifact of the emulation, not of the guest.  So hold
    // the beat off until the byte lands, exactly as the DRQ-checked path
    // already does -- this ADDS no new deadlock, because the chunk is
    // armed and the counter says the bytes are still owed and coming.
    wire c96_shim_rd_starved = TURBOSCSI_C96_EN && pb_dma_shim &&
                               (phase == S_DATA_IN) &&
                               c96_xfr_armed && c96_xfr_dma &&
                               (c96_fifo_pos == 5'd0) &&
                               (c96_tcounter != 17'd0);
    wire c96_shim_rd_withhold = (TURBOSCSI_C96_EN && pb_dma_shim &&
                                 scsi_ctrl_in[7] && !drq_c96 &&
                                 !c96_dma16_lo_inherit) ||
                                c96_shim_rd_starved;
    assign dma_rd_ready = !c96_shim_rd_withhold;
    // Write-side analog: mirrors the write term of the same withhold-
    // gate (`TURBOSCSI_C96_EN && pb_dma_shim && scsi_ctrl_in[8] &&
    // !drq_c96`).  Same contract as dma_rd_ready: the pulse-decision in
    // peripheral_bus.v and the ack-decision below consult the identical
    // drq_c96 value on the identical cycle, so a scsi_wr pulse fired
    // while this reads 1 is guaranteed its pb_ack one cycle later.
    //
    // 2026-08-19: gains the SAME c96_dma16_lo_inherit exemption the read
    // side has carried since f63207b.  dafb.cpp:1039-1053 checks
    // m_drq[bus] once per host access and then calls dma16_swap_w, which
    // pushes both bytes with no second consultation, so the low half of
    // a split word write must not re-run the check.  It re-ran it here,
    // and the two reachable ways for DRQ to fall between our beats —
    // fifo_pos reaching 15 (the DMA_OUT formula's own limit) and a
    // tcount==1 chunk exhausting c96_tcounter on beat 0 (the DATA_OUT
    // term at the bottom of drq_c96) — both stranded the second beat
    // with no way to ever fire it: peripheral_bus.v's write serializer
    // re-evaluates this gate on every strobe-bit pulse, so the AXI B
    // response never arrived except through its watchdog.
    wire c96_shim_wr_withhold = TURBOSCSI_C96_EN && pb_dma_shim &&
                                scsi_ctrl_in[8] && !drq_c96 &&
                                !c96_dma16_lo_inherit;
    assign dma_wr_ready = !c96_shim_wr_withhold;
    // MAME's once-per-access single-byte tests (see c96_dma16_degrade).
    // Evaluated combinationally at the HIGH beat, i.e. on exactly the
    // pre-access state MAME tests them against.
    wire c96_dma16_rd_single = (c96_fifo_pos < 5'd2);
    // ── DEGRADE SUPPRESSION WHILE THE SUPPLY IS MERELY LATE (2026-09-07)
    // Shape-(i) boot stall, ROM 0x40899706 (status=0x11, INT clear,
    // fifo=9, twice, identically).  The ROM drains each DMA data-in
    // chunk as a BLIND burst of MOVE.W reads from the DAFB pseudo-DMA
    // port (ROM 0x40899628: 16x `movew %a1@(256),%a2@+`, Duff-
    // dispatched, no DRQ check, no TC0 poll first).  MAME's dma16_r
    // degrades such an access to a single dma_r() when fifo_pos < 2
    // (ncr53c90.cpp:1329) — but on MAME that state is unreachable while
    // the transfer still owes bytes, because recv_byte() refills the
    // FIFO instantly between CPU instructions.  Our supply has real
    // latency (vhdd ring, SD pacing, 1 accept/cycle), so a word beat CAN
    // land at fifo_pos == 1 with tcounter != 0.  Each one popped ONE
    // byte while the host pointer advanced TWO: one byte of the block
    // leaked into FIFO residue (silent corruption), and once TC0 set the
    // residue held drq above the BUSMD_1 threshold so the chunk
    // completion (`TC0 && !drq`, MAME INIT_XFR_BUS_COMPLETE) never
    // fired.  INT never rose; the ROM's untimed INT wait at 0x40899704
    // parked forever.  Arithmetic check against the fingerprint: 256
    // blind words with 9 degraded beats give pops = 2*(256-9)+9 = 503,
    // accepts = 512 (TC0), residue = 9 — exactly the measured state.
    //
    // Fix, same principle as c96_shim_rd_starved (f8ad8c33): the
    // degraded access is an emulation-latency artifact, not guest
    // behaviour, so suppress the degrade while the armed DMA-in
    // transfer still owes bytes (tcounter != 0) AND the supply actually
    // has them (avail != 0 — a genuinely short supply must still
    // degrade exactly like MAME's settled fifo would).  The HIGH beat
    // then pops the staged byte normally and the LOW beat, no longer
    // swallowed, waits out the supply through c96_shim_rd_starved
    // (fifo_pos == 0 && tcounter != 0) and pops the next real byte when
    // it lands.  States where this term changes behaviour are exactly
    // the states MAME cannot settle in, so the scsi_fuzz differential
    // is unaffected by construction (verified by the baseline arm).
    // 2026-09-07 REFINEMENT (starvation-site measurement, realistic
    // pacing): the original `avail != 0` term treated a momentary RING
    // dip as "supply exhausted" and degraded anyway — with the CPU's
    // blind drain outrunning the SD stream the ring dips to 0 roughly
    // every few words, and each dip leaked one byte again (measured:
    // hi-beat singles at fifo=1 tc!=0 avail=0 every ~9 bytes).  "More
    // coming" means the transfer still owes bytes AND the supply is not
    // CLOSED — a live provider (vh_busy / a byte landing this cycle)
    // with an empty ring is LATE, not absent.  A genuinely exhausted
    // supply (provider idle, ring empty) still degrades exactly like
    // MAME's settled FIFO would.
    wire c96_dma16_rd_more_coming = TURBOSCSI_C96_EN &&
                                    (phase == S_DATA_IN) &&
                                    c96_xfr_armed && c96_xfr_dma &&
                                    (c96_tcounter != 17'd0) &&
                                    ((c96_avail_bytes != 16'd0) ||
                                     vh_busy || vh_rd_valid);
    wire c96_dma16_wr_single = (c96_fifo_pos > 5'd14) ||
                               (c96_tcounter == 17'd1);
    // "This beat is the swallowed low half of a degraded 16-bit access."
    // Keyed on the lo-beat flag ALONE, never on the DRQ grant: MAME
    // applies the degradation in both the DRQ-checked and the blind
    // (scsi_ctrl bit clear) dafb branches.
    wire c96_dma16_lo_swallow = TURBOSCSI_C96_EN && pb_dma_shim &&
                                pb_dma16_lo_beat && c96_dma16_degrade;
    // In C96 mode a data-in beat only advances the transfer while a DMA
    // CI_XFER chunk is armed and still owes the CPU bytes (c96_xfr_left).
    // The ROM issues one DMA|CI_XFER per 16-byte chunk and reads exactly
    // tcount bytes; unarmed shim reads must not steal data.  The bare-
    // 5380 path (TURBOSCSI_C96_EN=0) is unchanged.
    // 2026-08-19 R1 REWORK: in C96 mode every shim read is now a plain
    // MAME dma_r() — a pop of the real c96_fifo, where the DMA data-in
    // path stages its accepted bytes (see c96_accept_ev).  This wire is
    // kept for the bare-5380 pointer-advance path (unchanged) and as
    // the debug accept_pend/xfr_left bookkeeping event in C96 mode; no
    // C96 functional path consumes it any more.
    // Both exclude a swallowed low half: MAME's degraded 16-bit access
    // performs exactly ONE dma_r()/dma_w(), so the second beat must move
    // no payload at all.  For the OUT direction this is the data-
    // integrity term — pseudo_dma_out_beat is what routes the byte into
    // sec_buf, so letting a swallowed beat through would write one extra
    // byte into the sector.
    wire pseudo_dma_in_beat  = pb_rd && pb_dma_shim && (phase == S_DATA_IN) &&
                               !c96_dma16_lo_swallow &&
                               (TURBOSCSI_C96_EN
                                ? (c96_fifo_pos != 5'd0)
                                : t_req);
    wire pseudo_dma_out_beat = pb_wr && pb_dma_shim && (phase == S_DATA_OUT) &&
                               !c96_dma16_lo_swallow && t_req;
    // Non-DMA CI_XFER (0x10) in DATA IN: the chip itself REQ/ACKs data-in
    // bytes into the 16-byte FIFO for the CPU to pop at offset 2 (MAME
    // INIT_XFR non-DMA receive path).
    //
    // MAME CONTRACT — ONE BYTE PER COMMAND, then bus_complete():
    //   ncr53c90.cpp:601-635  INIT_XFR / S_PHASE_DATA_IN -> recv_byte()
    //   ncr53c90.cpp:643-661  INIT_XFR_WAIT_REQ:
    //       || (!dma_command && (xfr_phase & S_INP) == S_INP && fifo_pos == 1))
    //              // "non-dma in: every byte"
    //          state = INIT_XFR_BUS_COMPLETE;
    //   ncr53c90.cpp:686-692  INIT_XFR_BUS_COMPLETE -> bus_complete()
    //   ncr53c90.cpp:792-799  bus_complete(): state = IDLE; istatus |= I_BUS
    // So a non-DMA Transfer Information in a data-IN phase moves EXACTLY
    // one byte, leaves fifo_flags reading 0x01, raises I_BUS, and returns
    // the chip to IDLE (i.e. disarms).  The completion + disarm live in
    // the "non-DMA CI_XFER completion" hook further down this file.
    //
    // 2026-08-07 BUG FIX (System 7.5.3 "Starting up..." hang, HW bitstream
    // 0xE80161D3).  This beat used to free-run one byte per idle pb cycle
    // until the FIFO hit 16, with completion only on phase exhaust.  A
    // single 0x10 therefore silently consumed up to 16 bytes of target
    // supply that the driver only ever popped ONE of, starving a later
    // DMA|CI_XFER chunk into a state with no completion path at all:
    // accepts need avail>pend, chunk-I_BUS needs tcounter==0, phase-exit
    // needs a drain beat, drain beats need accept_pend!=0.  Chip frozen at
    // cmd=0x90 status=0x01 tcounter=0x0010, SD back-end idle, INT never
    // rises again, SCSI Manager 4.3 polls forever.
    //   * `c96_fifo_pos == 0` is MAME's "receive while fifo_pos != 1"
    //     read literally for a FIFO that starts empty (the CDB-assembly
    //     path zeroes fifo_pos at dispatch, so it always does).
    //   * availability is the SAME notion the DMA accept path uses
    //     (ring occupancy for multi-block reads, phase remainder for
    //     fully-buffered sources) rather than the legacy `t_req`, which
    //     is a statement about the NEXT byte, not about this one.
    //   * the bus-conflict gate is now narrowed to the accesses that
    //     actually collide in the register block: a reg-2 read (FIFO
    //     pop), any register write (reg-2 push / reg-3 command re-arm),
    //     and a reg-5 istatus read (clears the istatus this beat's
    //     completion sets — the same exclusion c96_accept_ev carries).
    //     The old blanket `!pb_rd && !pb_wr` also blocked the beat on a
    //     gap-free reg-4 status poll, i.e. exactly the loop a driver
    //     waiting for this command's interrupt spins in.
    wire c96_nondma_supply = vh_multi_read ? (vh_buf_count != 10'd0)
                                           : (xfer_bytes_left != 16'd0);
    wire c96_fifo_bus_conflict =
        (pb_rd && !pb_dma_shim && ((pb_addr[3:0] == 4'h2) ||
                                   (pb_addr[3:0] == 4'h5))) ||
        (pb_wr && !pb_dma_shim);
    // Post-ATN_STOP CDB feed event: armed non-DMA Transfer Information
    // in COMMAND phase hands FIFO bytes to the target one per cycle
    // (send_byte pacing is invisible at settled sync points).
    // Post-ATN_STOP DMA message drain: staged FIFO bytes go out one per
    // cycle (no transfer-count cost — MAME's send_byte does not call
    // decrement_tcounter; only dma_w/dma_r do).
    // 2026-08-19 BUG FIX (scsi_fuzz seed 0 + 17 more seeds of the same
    // signature): the last term here used to be a bare `!pb_dma_shim`.
    // pb_dma_shim is an ADDRESS DECODE, not an access strobe — after the
    // final pseudo-DMA beat of a burst the bus can idle with pb_addr
    // still parked at 0x100 (the fuzz harness does exactly that, and
    // nothing forbids the SoC address bus idling there either), which
    // held the drain off FOREVER: the FIFO kept its staged message
    // bytes, the drain-end hook (which needs fifo_pos==0) never fired,
    // and the chip stayed parked in MSG_OUT with no interrupt while
    // MAME sent every staged byte and completed with I_BUS.  Only an
    // ACTUAL shim access (blind dma_r pop / dma_w beat) conflicts with
    // this drain, so qualify the gate with the strobes.
    wire c96_stop_feed = TURBOSCSI_C96_EN && c96_sel_stopped &&
                         c96_xfr_armed && c96_xfr_dma &&
                         (c96_stop_feed_timer == 10'd0) &&
                         (c96_fifo_pos != 5'd0) && !c96_fifo_bus_conflict &&
                         !((pb_rd || pb_wr) && pb_dma_shim);
    wire c96_cmdwait_feed = TURBOSCSI_C96_EN && c96_cmd_wait &&
                            c96_xfr_armed && !c96_xfr_dma &&
                            (phase == S_COMMAND) && !c96_xfr_out_park &&
                            (c96_fifo_pos != 5'd0) &&
                            !c96_fifo_bus_conflict &&
                            !((pb_rd || pb_wr) && pb_dma_shim);
    // 2026-08-19 (scsi_fuzz seed 13 family): the empty-FIFO gate here
    // was widened to FIFO SPACE (MAME's recv, :626-627) so a receive
    // behind prior residue still pulls the target's byte in at the
    // FIFO tail.
    // 2026-09-06: the completion hook below no longer carries a pos==0
    // qualifier — one beat per arm, so MAME's residue FREE-RUN (:650
    // `fifo_pos == 1` after the push never matching) is deliberately
    // NOT modelled any more.  Real-chip semantics: one byte + I_BUS,
    // residue or not.  See the completion hook's divergence box.
    wire c96_fifo_fill_beat = TURBOSCSI_C96_EN && c96_xfr_armed &&
                              !c96_xfr_dma && (phase == S_DATA_IN) &&
                              c96_nondma_supply && (c96_fifo_pos != 5'd16) &&
                              !c96_fifo_bus_conflict;
    // ── CI_COMPLETE (0x11) receive in a live DATA IN phase ────────────
    // MAME CI_COMPLETE (ncr53c90.cpp:1011-1015) runs recv_byte()
    // unconditionally, so RECV_WAIT_SETTLE (:465-482) pushes whatever the
    // TARGET is driving on the data lines.  In DATA IN that is the next
    // real payload byte; only in an out-phase (nobody driving) is it the
    // 0x00 the dispatch arm's fallback pushes.  Exactly ONE byte moves —
    // INIT_CPT_RECV_WAIT_REQ (:578-589) sees phase != MSG_IN, zeroes the
    // command queue and bus_complete()s — hence the pend flag rather than
    // the free-running level condition c96_fifo_fill_beat carries.
    // (scsi_fuzz seeds 41 + 43: MAME serves the INQUIRY / READ(6) byte,
    // the RTL served 0x00.)
    wire c96_cpt_din_beat = TURBOSCSI_C96_EN && c96_cpt_din_pend &&
                            !c96_xfr_armed && (phase == S_DATA_IN) &&
                            c96_nondma_supply && (c96_fifo_pos != 5'd16) &&
                            !c96_fifo_bus_conflict;
    // ── Non-DMA CI_XFER (0x10) in DATA OUT — the SEND mirror ──────────
    // 2026-08-07 BUG FIX (System 7.5.3 stalls at ~85% of the extension
    // parade, HW bitstream 0xC74E3489).  The 53C96 trace ring showed one
    // transaction retried forever — 4096+ events in 70 s — every retry
    // ending on exactly:
    //     R4=90  R3=90  R8=47  W8=c7  W2=ee  W3=10  R4=10
    // i.e. one byte staged in the FIFO (W2) and a NON-DMA Transfer
    // Information (W3=0x10) issued while the bus sits in DATA OUT
    // (status[2:0] == 000), after which INTR never rises again.
    //
    // Until this fix the FIFO-drain half of a non-DMA transfer only
    // existed for data IN (c96_fifo_fill_beat above): NOTHING in this
    // module ever handed a c96_fifo byte to the target.  The armed
    // command therefore had no reachable completion hook at all — the
    // non-DMA hook needs c96_fifo_fill_beat (DATA IN only), the
    // phase-change hook needs phase == S_STATUS (unreachable, because
    // the phase only advances when a byte is consumed), and the DMA
    // chunk hook needs c96_xfr_dma.  Dead end; the driver retried
    // forever.  This is the exact mirror of the data-IN contract added
    // for the same command on 2026-08-06.
    //
    // MAME CONTRACT — DRAIN THE WHOLE FIFO, then bus_complete():
    //   ncr53c90.cpp:601-616  INIT_XFR / S_PHASE_DATA_OUT:
    //       state = INIT_XFR_SEND_BYTE;
    //       if (fifo_pos == 0) break;      // "can't send if the fifo is empty"
    //       send_byte();
    //   ncr53c90.cpp:749-763  send_byte(): data_w(..., fifo_pop())
    //   ncr53c90.cpp:643-651  INIT_XFR_WAIT_REQ:
    //       || (!dma_command && (xfr_phase & S_INP) == 0 && fifo_pos == 0)
    //              // "non-dma out: fifo empty"
    //          state = INIT_XFR_BUS_COMPLETE;
    //       else if phase unchanged -> INIT_XFR again (loop: next byte)
    //   ncr53c90.cpp:686-692  INIT_XFR_BUS_COMPLETE -> bus_complete()
    //   ncr53c90.cpp:792-799  bus_complete(): state = IDLE; istatus |= I_BUS
    // So an out-phase non-DMA Transfer Information sends EVERY staged
    // byte (not one, unlike the data-IN "non-dma in: every byte" form at
    // :650) and raises I_BUS once the FIFO is empty — including the
    // degenerate 0x10-with-empty-FIFO case, which :608-610 skips the
    // send for and still completes.
    //
    // tcounter is deliberately untouched: decrement_tcounter()
    // (ncr53c90.cpp:1234-1237) returns immediately when !dma_command, so
    // a non-DMA transfer never moves the counter in either direction.
    // The data-IN half behaves the same way (its decrement lives in
    // c96_accept_ev, which requires c96_xfr_dma).
    //
    // `t_req` is the target's REQ line, matching MAME's
    // `if(!(ctrl & S_REQ)) break;` at the head of INIT_XFR_WAIT_REQ and
    // the existing pseudo_dma_out_beat gate — so a multi-block write
    // whose ring is full back-pressures this path exactly as it
    // back-pressures the DMA one.  The bus-conflict gate additionally
    // excludes a pseudo-DMA write beat, because both would drive the
    // single sec_buf write port in the same cycle.
    // ── Non-DMA CI_XFER (0x10) in STATUS / MSG IN ─────────────────────
    // 2026-08-08.  Closing the rest of the class rather than waiting to
    // meet instance five on hardware: MAME's INIT_XFR handles
    // S_PHASE_STATUS and S_PHASE_MSG_IN on the SAME arm as
    // S_PHASE_DATA_IN (ncr53c90.cpp:623-633), so a non-DMA Transfer
    // Information in either phase receives one byte into the FIFO and
    // completes.  We had neither, with two DIFFERENT failure modes:
    //   * STATUS — an armed CI_XFER already got I_BUS from the
    //     phase-change hook, but NO byte was ever pushed, so the driver
    //     popped 0x00 at offset 2 instead of the real status.  Silent
    //     wrong data, which is worse than the DATA OUT hang.
    //   * MSG IN — no hook matches at all, so it hangs exactly like the
    //     DATA OUT case did.
    //
    // COMPLETION DIFFERS BETWEEN THE TWO, and this is easy to get wrong:
    //   ncr53c90.cpp:629-630 picks INIT_XFR_RECV_BYTE_NACK (rather than
    //   _ACK) when `xfr_phase == S_PHASE_MSG_IN && (!dma_command || ...)`,
    //   and :676-684 routes NACK -> INIT_XFR_FUNCTION_COMPLETE ->
    //   function_complete() (:782-790), which sets **I_FUNCTION**.
    //   STATUS takes the _ACK path -> INIT_XFR_WAIT_REQ -> the
    //   "non-dma in: every byte" test at :650 -> INIT_XFR_BUS_COMPLETE ->
    //   bus_complete() (:792-799), which sets **I_BUS**.
    // So: STATUS completes I_BUS, MSG IN completes I_FUNCTION.
    //
    // tcounter is untouched here too (:1234-1237 early-returns when
    // !dma_command), matching both the DATA IN and DATA OUT halves.
    //
    // Gated on the LATCHED c96_xfr_phase, not the live phase — see that
    // register's declaration for why that is load-bearing.
    // 2026-08-19 (scsi_fuzz seed 13): the two receives below were gated
    // on an EMPTY FIFO — MAME's recv only requires FIFO SPACE
    // (`if (fifo_pos == 16) break;`, ncr53c90.cpp:626-627) and pushes
    // the byte at the TAIL behind whatever is already staged, so a
    // 0x10 issued in STATUS with CDB residue still in the FIFO must
    // pull the status byte (and complete I_BUS via the phase change to
    // MSG_IN) instead of silently doing nothing forever.
    wire c96_nondma_in_status = TURBOSCSI_C96_EN && c96_xfr_armed &&
                                !c96_xfr_dma &&
                                (c96_xfr_phase == S_STATUS) &&
                                (phase == S_STATUS) &&
                                (c96_fifo_pos != 5'd16) &&
                                !c96_fifo_bus_conflict;
    wire c96_nondma_in_msgin  = TURBOSCSI_C96_EN && c96_xfr_armed &&
                                !c96_xfr_dma &&
                                (c96_xfr_phase == S_MSG_IN) &&
                                (phase == S_MSG_IN) &&
                                (c96_fifo_pos != 5'd16) &&
                                !c96_msgin_ack_held &&
                                !c96_fifo_bus_conflict;
    // The byte the target presents in each phase — the same values
    // CI_COMPLETE preloads into the FIFO for the equivalent flow.
    wire [7:0] c96_nondma_in_byte = (c96_xfr_phase == S_MSG_IN) ? xfer_msg
                                                                : xfer_status;
    wire c96_fifo_out_bus_conflict = c96_fifo_bus_conflict ||
                                     (pb_wr && pb_dma_shim);
    // 2026-08-19 FIX (HW hang after the phase-drift fix landed; board
    // showed reg3=0x90 armed, reg4=0x10 TC0+DATA OUT, reg7=0x01, t_req=1,
    // no IRQ, SD back end idle at 2502 completions).  This used to carry
    // `!c96_xfr_dma`, so NOTHING drained staged FIFO bytes out to the
    // target while a DMA-form CI_XFER was armed.  MAME's send is NOT
    // dma-gated: INIT_XFR's out-group branch (ncr53c90.cpp:603-620) does
    // `if (fifo_pos == 0) break; ... send_byte();` for DATA_OUT /
    // COMMAND / MSG_OUT regardless of dma_command, and the DMA-out
    // completion test at :648 (`dma_command && S_TC0 && fifo_pos == 0`)
    // is only ever reached BECAUSE those sends drain the FIFO.  With the
    // gate in place fifo_pos could never reach 0, so the transfer stayed
    // armed forever with the target still asserting REQ.
    //
    // MEASURED (directed differential, RTL vs live MAME): arm a 512-byte
    // DMA CI_XFER in DATA OUT, then push three bytes through register 2 —
    //   RTL  fifo=3:aabbcc flags=03   (stuck)
    //   MAME fifo=0:       flags=00   (sent to the target)
    // tcounter matched at 512 on both sides, so this is a missing SEND,
    // not a counter desync.
    //
    // DATA INTEGRITY: the popped byte is real write payload — the
    // consumer routes c96_fifo[0] into sec_buf via sec_wr_mux and
    // advances the transfer bookkeeping, exactly as a pseudo-DMA beat
    // does.  It is never merely discarded.  c96_fifo_out_bus_conflict
    // already excludes `pb_wr && pb_dma_shim`, so this beat and
    // pseudo_dma_out_beat can never fire in the same cycle and cannot
    // double-book or drop a byte.
    //
    // The non-DMA COMPLETION (c96_out_send_done) keeps its !c96_xfr_dma
    // gate: the DMA form completes through the TC0 hook instead
    // (`dma_command && S_TC0 && fifo_pos == 0`, MAME :648), which this
    // drain is precisely what makes reachable.
    wire c96_out_send_beat = TURBOSCSI_C96_EN && c96_xfr_armed &&
                               (phase == S_DATA_OUT) &&
                               t_req && !c96_xfr_out_park &&
                               (c96_fifo_pos != 5'd0) &&
                               !c96_fifo_out_bus_conflict;
    // Completion for the same command: MAME's "non-dma out: fifo empty".
    // Two disjoint cases, both of which leave the FIFO empty at the end
    // of this cycle: the FIFO drained to empty at some earlier point, or
    // this cycle drains the last byte.  Held off while the CPU reads
    // istatus, mirroring the exclusion c96_accept_ev carries.
    // NOT included: the 0x10-issued-with-nothing-staged case.  MAME's
    // :608-610 breaks out of INIT_XFR before send_byte() when the FIFO is
    // empty at dispatch, so no timer is armed, INIT_XFR_WAIT_REQ is never
    // entered and :649 never fires — the command parks with no interrupt
    // (c96_xfr_out_park; scsi_fuzz seed 27).
    wire c96_out_send_done = TURBOSCSI_C96_EN && c96_xfr_armed &&
                               !c96_xfr_dma && (phase == S_DATA_OUT) &&
                               t_req && !c96_xfr_out_park &&
                               ((c96_fifo_pos == 5'd0) ||
                                (c96_out_send_beat &&
                                 (c96_fifo_pos == 5'd1))) &&
                               !(pb_rd && !pb_dma_shim &&
                                 (pb_addr[3:0] == 4'h5));
    // C96 status[2:0] reflect the current SCSI bus phase encoded as
    // {MSG, C/D, I/O}.  Derived directly from `phase` so the back-end FSM
    // and the C96 register block share one source of truth.
    function [2:0] c96_phase_bits;
        input [3:0] cur_phase;
        begin
            case (cur_phase)
                S_DATA_OUT:    c96_phase_bits = 3'b000; // OUT: nothing asserted
                S_DATA_IN:     c96_phase_bits = 3'b001; // I/O
                S_COMMAND:     c96_phase_bits = 3'b010; // C/D
                S_STATUS:      c96_phase_bits = 3'b011; // C/D | I/O
                S_MSG_IN:      c96_phase_bits = 3'b111; // MSG | C/D | I/O
                S_MSG_OUT:     c96_phase_bits = 3'b110; // MSG | C/D
                S_RESELECT_MSG:c96_phase_bits = 3'b111; // MSG | C/D | I/O
                default:       c96_phase_bits = 3'b000;
            endcase
        end
    endfunction
    // ── Off-bus back-end states ───────────────────────────────────────
    // S_CMD_EXEC and the two S_VH_WAIT_* states are OURS, not the SCSI
    // bus's: the target is still connected and still holding whatever
    // phase it last drove, but our back-end has stepped aside to decode
    // the CDB or to wait on the backing store.  MAME has no equivalent —
    // status_r() (ncr53c90.cpp:1088-1095) reports the LIVE bus lines, so
    // its phase bits simply never change while a target thinks.
    //
    // They used to fall through c96_phase_bits' `default: 3'b000`, i.e.
    // they reported DATA OUT — the single worst answer available, since
    // it tells the driver the target is asking for data.  A READ whose
    // backing store takes milliseconds (S_CMD_EXEC -> S_VH_WAIT_RD) held
    // that lie for the whole latency, with INTR low.
    function c96_phase_offbus;
        input [3:0] cur_phase;
        begin
            c96_phase_offbus = (cur_phase == S_CMD_EXEC)   ||
                               (cur_phase == S_VH_WAIT_RD) ||
                               (cur_phase == S_VH_WAIT_WR);
        end
    endfunction
    // Bus phase last driven by a real bus state, held across the off-bus
    // states above.  Seeded with COMMAND because the only way into an
    // off-bus state before any bus phase has been driven would be a
    // reset landing mid-select, which the FSM cannot produce.
    reg [2:0] c96_phase_hold;
    always @(posedge clk) begin
        if (rst)
            c96_phase_hold <= 3'b010;               // COMMAND
        else if (!c96_phase_offbus(phase))
            c96_phase_hold <= c96_phase_bits(phase);
    end
    // SCSI CDB length from the opcode group (byte 0): group 1/2 (bits
    // [7:5] == 001/010) are 10-byte CDBs, everything the Mac ROM issues
    // otherwise is 6-byte.
    // Bytes the TARGET consumes for a CDB whose first byte is `op` —
    // the chip itself has no length knowledge; the target REQs until
    // its group decode is satisfied (MAME nscsi_full_device::
    // scsi_command_done, nscsi_bus.cpp:550: groups 0->6, 1/2->10,
    // 5->12, 7->32, and 3/4/6 are complete after ONE byte — which is
    // how an IDENTIFY mistaken for an opcode consumes a single byte
    // and CHECK CONDITIONs, leaving the rest in the FIFO).
    function [5:0] c96_cdb_len;
        input [7:0] op;
        begin
            case (op[7:5])
                3'd0:    c96_cdb_len = 6'd6;
                3'd1,
                3'd2:    c96_cdb_len = 6'd10;
                3'd5:    c96_cdb_len = 6'd12;
                3'd7:    c96_cdb_len = 6'd32;
                default: c96_cdb_len = 6'd1;   // groups 3/4/6
            endcase
        end
    endfunction
    // ── Chip-side DMA-in acceptance ────────────────────────────────────
    // Bytes the current data-in phase can still hand the chip: for
    // multi-block reads only what the volume ring has buffered; for fully-
    // buffered sources (INQUIRY/SENSE/CAPACITY/single-block read) the whole
    // remainder of the phase.  c96_accept_pend is the accepted-but-
    // undrained subset, so (avail > pend) means one more byte can be
    // accepted (tcounter decremented) this cycle.  The event is held off
    // in cycles where the CPU writes the command register (tcounter
    // reload) or reads istatus (status_sticky clear) so the two
    // assignments cannot collide.
    wire [15:0] c96_avail_bytes = (vh_multi_read)
                                  ? {6'd0, vh_buf_count}
                                  : xfer_bytes_left;
    // 2026-08-19 R1 REWORK: an accept now STAGES the byte through the
    // real 16-deep c96_fifo (MAME recv_byte() → fifo_push()), so it is
    // additionally gated on FIFO space — MAME stops receiving at
    // fifo_pos == 16 (ncr53c90.cpp:627 `if (fifo_pos == 16) break;`),
    // which is what keeps TC0 timing honest for tcount > 16 — and on
    // FIFO-access-free cycles so the tail push cannot collide with a
    // same-cycle host pop/push.  The accept consumes the supply
    // directly (buf_rd_ptr / xfer_bytes_left advance at accept, not at
    // CPU drain), so the old `avail > pend` becomes `avail != 0`.
    wire c96_accept_ev = TURBOSCSI_C96_EN && c96_xfr_armed && c96_xfr_dma &&
                         (phase == S_DATA_IN) && (c96_tcounter != 17'd0) &&
                         (c96_avail_bytes != 16'd0) &&
                         (c96_fifo_pos != 5'd16) &&
                         !c96_fifo_bus_conflict &&
                         !((pb_rd || pb_wr) && pb_dma_shim) &&
                         !(pb_rd && !pb_dma_shim && (pb_addr[3:0] == 4'h5));
    // Composed status byte (read at offset 4).  While an ATN-form
    // deferred select owes its message byte, report MSG_OUT (110) —
    // the back-end `phase` reg sits in S_COMMAND for the whole select.
    wire [2:0] c96_phase_bits_eff =
        ((c96_sel_active && c96_sel_msg_out) || c96_sel_stopped)
            ? 3'b110
            : (c96_phase_offbus(phase) ? c96_phase_hold
                                       : c96_phase_bits(phase));
    wire [7:0] c96_status_read =
        c96_status_sticky |
        {5'b00000, c96_phase_bits_eff} |
        (c96_irq_pending ? 8'h80 : 8'h00);
    // ── FIFO Flags (read at offset 7) ──────────────────────────────────
    // MAME ncr53c90_device::fifo_flags_r() (ncr53c90.cpp:1141-1144)
    // returns the live FIFO occupancy, and recv_byte() → fifo_push()
    // (ncr53c90.cpp:471, 848-853) pushes EVERY byte the chip accepts
    // from the target in a DATA IN phase into that same 16-byte FIFO.
    // 53C94/53C96 do not override fifo_flags_r, so on a DMA data-in
    // transfer MAME's offset-7 count climbs to 16 and the host drains
    // it.
    //
    // Our C96 DMA data-in path bypasses c96_fifo entirely: bytes
    // accepted from the target are counted by c96_accept_pend and
    // served out of sec_buf through the DAFB pseudo-DMA shim, so
    // c96_fifo_pos would read 0 for the whole transfer.  Report the
    // accepted-but-undrained count instead, saturated at the real
    // chip's 16-byte FIFO depth (MAME stops receiving at fifo_pos ==
    // 16, ncr53c90.cpp:627, so 16 is the maximum it can ever show).
    // The non-DMA CI_XFER data-in path already stages its bytes in
    // c96_fifo via c96_fifo_fill_beat and is unaffected, as are the
    // CDB-assembly and CI_COMPLETE status/msg uses of c96_fifo_pos.
    //
    // HW 2026-08-01, bitstream 0xE74B330C: System 7.5.3's SCSI driver
    // gates its 16-byte chunked drain on this register — 0x405C2
    // `BTST #4,$70(A3)` (FIFO count >= 16), then eight
    // `MOVE.W $100(A1),(A2)+` beats off the pseudo-DMA port.  With
    // offset 7 stuck at 0 that gate never opened and the driver spun
    // forever at 0x40592..0x405C8 with STAT=0x11 (DATA IN + TC0),
    // cmd=0x90, tcounter=0, DRQ asserted.  Forcing the count to 16
    // over JTAG released it through the rest of the transaction
    // (cmd echo 0x90 → 0x12 CI_MSG_ACCEPT, STAT → 0x10 bus free).
    // The Q700 ROM / System 7.0.1 driver polls only STAT + the DAFB
    // DRQ bit, which is why this stayed latent until 7.5.3.
    // 2026-08-19 R1 REWORK: DMA data-in bytes are now staged through
    // the real c96_fifo (accepts push, host drains pop), so offset 7 is
    // simply the live occupancy — exactly MAME fifo_flags_r().  The
    // accept_pend visibility shim that used to sit here (and satisfied
    // the 7.5.3 BTST #4 gate) is superseded: the real count climbs to
    // 16 the same way, and unlike the shim it also FALLS to 0 when the
    // driver drains through reg 2 or the DMA port — the MacBench "SCSI
    // Information" hang was the driver spinning on `flags & 0x1f == 0`
    // against a count no drain path could decrement.
    wire [4:0] c96_fifo_flags = c96_fifo_pos;
    reg [7:0] pb_rd_mux;
    always @* begin
        if (TURBOSCSI_C96_EN && !pb_dma_shim) begin
            case (pb_addr[3:0])
                4'h0: pb_rd_mux = c96_tcounter[7:0];
                4'h1: pb_rd_mux = c96_tcounter[15:8];
                4'h2: pb_rd_mux = (c96_fifo_pos != 5'd0) ? c96_fifo[0] : 8'h00;
                4'h3: pb_rd_mux = c96_command_q;
                4'h4: pb_rd_mux = c96_status_read;
                4'h5: pb_rd_mux = c96_istatus;
                4'h6: pb_rd_mux = c96_seq_step;
                4'h7: pb_rd_mux = {3'b000, c96_fifo_flags};
                4'h8: pb_rd_mux = c96_config1;
                4'hb: pb_rd_mux = c96_config2;
                4'hc: pb_rd_mux = c96_config3;
                default: pb_rd_mux = 8'hff;
            endcase
        end else if (pb_dma_shim) begin
            // C96 (R1 rework): every shim read is MAME dma_r() — pop of
            // the real FIFO head, where the accepted data-in bytes are
            // now staged.  Bare-5380 keeps the original literal-REQ
            // selection.
            // A swallowed low half reads back MAME's literal 0xFF
            // filler (`dma_r() | 0xff00` byte-swapped into the host's
            // low byte), NOT a duplicate of the residual payload byte.
            pb_rd_mux = c96_dma16_lo_swallow
                        ? 8'hff
                        : (TURBOSCSI_C96_EN
                           ? c96_fifo[0]
                           : (((phase == S_DATA_IN) && t_req) ? sec_rd_a
                                                              : cur_data));
        end else begin
            case (pb_addr[2:0])
                3'd0: pb_rd_mux = cur_data;
                3'd1: pb_rd_mux = r_init_cmd;
                3'd2: pb_rd_mux = r_mode;
                3'd3: pb_rd_mux = r_tgt_cmd;
                3'd4: pb_rd_mux = bus_status_read(phase, init_sel, init_rst,
                                                  t_req);
                3'd5: pb_rd_mux = bus_and_stat_read(phase, r_tgt_cmd[2:0],
                                                    irq_live_5380, completion_live,
                                                    init_atn,
                                                    init_ack,
                                                    busy_error_pending, t_req);
                3'd6: pb_rd_mux = cur_data;        // "Input Data" == bus data
                3'd7: pb_rd_mux = 8'h00;           // reset-irq: returns 0
                default: pb_rd_mux = 8'h00;
            endcase
        end
    end
    // ── Deferred-select CDB byte sink ──────────────────────────────────
    // Called (from the main always block only) for every CDB byte that
    // arrives while c96_sel_active: FIFO writes carry bytes 0..n-2, the
    // DAFB pseudo-DMA port carries the tail (DMA-form select).  Once the
    // opcode-derived length is reached the back-end is dispatched exactly
    // like the immediate CD_SELECT shortcut, and the c96_sel_pending hook
    // raises I_FUNCTION|I_BUS + seq=4 when it settles in its first bus
    // phase (MAME function_bus_complete()).
    task c96_sel_cdb_byte;
        input [7:0] b;
        input [5:0] len;    // CDB length: opcode-derived on byte 0
        input       via_sel; // 1 = select-sequence delivery (completion
                             // raises the select-complete interrupt);
                             // 0 = Transfer-Information delivery after an
                             // ATN_STOP (completion relies on the armed
                             // xfer's phase-change I_BUS hook)
        integer j;
        begin
            if ({1'b0, c96_sel_idx} + 7'd1 >= {1'b0, len}) begin
                // Final CDB byte — cdb[] already holds the accepted
                // prefix (progressive drain); append this byte, zero the
                // tail, and kick the back-end into S_CMD_EXEC.  Bytes
                // past cdb[9] were accepted-and-discarded (only 1-byte-
                // group / junk CDBs are that long; the decode arms never
                // read past byte 9).
                for (j = 0; j < 10; j = j + 1) begin
                    if (j == {26'd0, c96_sel_idx})
                        cdb[j] <= b;
                    else if (j > {26'd0, c96_sel_idx})
                        cdb[j] <= 8'h00;
                end
                cdb_len         <= (len > 6'd10) ? 4'd10 : len[3:0];
                cdb_idx         <= 4'd0;
                c96_fifo_pos    <= 5'd0;
                c96_sel_active  <= 1'b0;
                c96_cmd_wait    <= 1'b0;
                c96_sel_len     <= 6'd0;
                c96_sel_idx     <= 6'd0;
                c96_xfer_active <= 1'b1;
                c96_sel_pending <= via_sel;
                c96_sel_seq_final <= (c96_sel_dma &&
                                      (c96_tcounter > 17'd1))
                                     ? 8'h02 : 8'h04;
                buf_rd_ptr      <= 9'd0;
                buf_wr_ptr      <= 9'd0;
                vh_fill_ptr     <= 9'd0;
                vh_drain_ptr    <= 9'd0;
                vh_buf_count    <= 10'd0;
                vh_wr_underflow <= 1'b0;
                xfer_bytes_left <= 16'd0;
                vh_kicked       <= 1'b0;
                vh_supply_faulted <= 1'b0;
                phase           <= S_CMD_EXEC;
                t_req           <= 1'b0;
            end else begin
                // Progressive drain: the chip REQ/ACKs the byte straight
                // to the target — the CPU-visible FIFO count stays 0 so
                // the System SCSI Manager's drain-poll (fifo_flags&0x1F
                // while phase==COMMAND) terminates.  Do NOT push c96_fifo.
                if (via_sel)
                    c96_seq_step  <= c96_sel_dma ? 8'h03 : 8'h04;
                c96_sel_len       <= len;
                if (c96_sel_idx < 6'd10)
                    cdb[c96_sel_idx[3:0]]  <= b;
                c96_sel_idx       <= c96_sel_idx + 6'd1;
            end
        end
    endtask
    // ══════════════════════════════════════════════════════════════════
    // Main sequential — register writes, phase FSM, volume interface
    // ══════════════════════════════════════════════════════════════════
    integer k;
    always @(posedge clk) begin
        if (rst) begin
            // Register file (live storage only — write-only NCR 5380 regs
            // 4..7 do not latch).
            canned_active <= 1'b0;
            r_output_data <= 8'h00;
            r_init_cmd    <= 8'h00;
            r_mode        <= 8'h00;
            r_tgt_cmd     <= 8'h00;
            // Bus view (cur_data only — bus_status / bus_and_stat are
            // computed combinationally from `phase`).
            cur_data      <= 8'h00;
            irq_pending   <= 1'b0;
            end_dma_pending <= 1'b0;
            c96_tcount         <= 16'h0000;
            c96_tcounter       <= 17'h00000;
            c96_fifo_pos       <= 5'd0;
            c96_command_q      <= 8'h00;
            c96_command_pos    <= 2'd0;
            c96_cmd_q1         <= 8'h00;
            c96_status_sticky  <= 8'h00;
            c96_istatus        <= 8'h00;
            c96_seq_step       <= 8'h00;
            c96_bus_id         <= 3'd0;
            c96_select_timeout <= 8'h00;
            c96_sync_period    <= 5'd0;
            c96_sync_offset    <= 4'd0;
            c96_config1        <= 8'h00;
            c96_config2        <= 8'h00;
            c96_config3        <= 8'h00;
            c96_irq_pending    <= 1'b0;
            c96_clock_conv     <= 3'd2;   // MAME device_reset()
            c96_select_timeout_active <= 1'b0;
            c96_select_timeout_polls  <= 8'h00;
            c96_select_timeout_limit  <= 8'h00;
            c96_select_timeout_cyc    <= 25'd0;
            c96_clk_div_cnt           <= 8'd0;
            c96_xfer_active           <= 1'b0;
            c96_sel_pending           <= 1'b0;
            c96_xfr_armed             <= 1'b0;
            c96_xfr_dma               <= 1'b0;
            c96_xfr_out_park          <= 1'b0;
            c96_xfr_park_timer        <= 10'd0;
            c96_stop_feed_timer       <= 10'd0;
            c96_stop_atn_dropped      <= 1'b0;
            c96_xfr_recv_pend         <= 1'b0;
            c96_xfr_recv_isstat       <= 1'b0;
            c96_xfr_recv_timer        <= 10'd0;
            c96_sel_active            <= 1'b0;
            c96_sel_dma               <= 1'b0;
            c96_sel_len               <= 6'd0;
            c96_sel_idx               <= 6'd0;
            c96_sel_msg_out           <= 1'b0;
            c96_sel_stopped           <= 1'b0;
            c96_sel_atn_stop          <= 1'b0;
            c96_stop_msg0             <= 8'h00;
            c96_identify_lun          <= 3'd0;
            c96_cpt_din_pend          <= 1'b0;
            c96_dma_dir               <= C96_DIR_NONE;
            c96_last_cmd_dma          <= 1'b0;
            c96_drq_stale             <= 1'b0;
            c96_drq_norecompute_q     <= 1'b0;
            c96_msgin_ack_held        <= 1'b0;
            c96_fifo_pos_q            <= 5'd0;
            c96_tc0_q                 <= 1'b0;
            c96_cmd_wait              <= 1'b0;
            c96_sel_seq_final         <= 8'h04;
            c96_xfr_left              <= 17'd0;
            c96_accept_pend           <= 10'd0;
            // FSM
            phase         <= S_BUS_FREE;
            vh_dev_sel    <= 1'b0;
`ifdef VERILATOR
            phase_prev    <= S_BUS_FREE;
            dbg_reg4_polls <= 16'd0;
            dbg_reg5_polls <= 16'd0;
            dbg_reg7_polls <= 16'd0;
            dbg_last_reg4  <= 8'h00;
            dbg_last_reg5  <= 8'h00;
            dbg_last_irq   <= 1'b0;
            dbg_select_seen <= 1'b0;
`endif
            cdb_idx       <= 4'd0;
            cdb_len       <= 4'd6;
            buf_rd_ptr    <= 9'd0;
            buf_wr_ptr    <= 9'd0;
            vh_fill_ptr   <= 9'd0;
            vh_drain_ptr  <= 9'd0;
            vh_buf_count  <= 10'd0;
            vh_wr_underflow <= 1'b0;
            t_req <= 1'b0;
            ack_prev     <= 1'b0;
            atn_prev     <= 1'b0;
            xfer_lba     <= 32'd0;
            xfer_blocks  <= 24'd0;
            xfer_status  <= 8'h00;
            xfer_msg     <= 8'h00;
            xfer_bytes_left <= 16'd0;
            vh_kicked    <= 1'b0;
            vh_wait_ctr  <= 16'd0;
            vh_stuck_ctr <= 24'd0;
            vh_supply_faulted <= 1'b0;
            busy_error_pending <= 1'b0;
            medium_not_present <= 1'b0;
            sense_key  <= 4'd0;
            sense_asc  <= 8'h00;
            sense_ascq <= 8'h00;
            // Disconnect / reselection state
            disc_pending         <= 1'b0;
            disc_resume_phase    <= S_BUS_FREE;
            disc_xfer_lba        <= 32'd0;
            disc_xfer_blocks     <= 24'd0;
            disc_xfer_bytes_left <= 16'd0;
            disc_buf_rd_ptr      <= 9'd0;
            disc_buf_wr_ptr      <= 9'd0;
            disc_vh_fill_ptr     <= 9'd0;
            disc_vh_drain_ptr    <= 9'd0;
            disc_vh_buf_count    <= 10'd0;
            disc_cdb_op          <= 8'h00;
            disc_req_write      <= 1'b0;
            disc_req_multi      <= 1'b0;
            disc_vh_kicked       <= 1'b0;
            ident_msg_sent       <= 1'b0;
            msg_out_byte_seen    <= 1'b0;
            // volume interface
            vh_req_write   <= 1'b0;
            vh_req_multi   <= 1'b0;
            data_from_ring <= 1'b0;
            vh_req_lba         <= 32'd0;
            vh_req_block_count <= 16'd1;
            vh_req_go          <= 1'b0;
            vh_wr_data     <= 8'h00;
            // Peripheral-bus outs
            pb_rdata <= 8'h00;
            pb_ack   <= 1'b0;
            c96_dma16_hi_granted <= 1'b0;
            c96_dma16_degrade    <= 1'b0;
            for (k = 0; k < 10; k = k + 1) cdb[k] <= 8'h00;
            for (k = 0; k < 16; k = k + 1) c96_fifo[k] <= 8'h00;
        end else begin
            // ── Defaults (one-cycle pulses) ──────────────────────────
            vh_req_go  <= 1'b0;
            // Unpark an out-phase CI_XFER once the FIFO has stopped being
            // empty AND the host has left the chip alone long enough for
            // MAME's send_byte() delay() to have fired — see the
            // c96_xfr_park_timer declaration.  Placed among the defaults
            // so the dispatch-time latch further down this block wins.
            if (c96_xfr_out_park && (c96_fifo_pos != 5'd0)) begin
                if (pb_dma_shim && (pb_rd || pb_wr))
                    c96_xfr_park_timer <= C96_RECV_QUIESCE;
                else if (c96_xfr_park_timer != 10'd0)
                    c96_xfr_park_timer <= c96_xfr_park_timer - 10'd1;
                else
                    c96_xfr_out_park <= 1'b0;
            end
            // pb_ack is normally pulsed any time pb_wr or pb_rd asserts.
            // TurboSCSI DRQ-check (DAFB +0x24 bit 7 read / bit 8 write,
            // dafb.cpp:1001-1011 + 1040-1047) holds off DTACK on the DMA
            // window when DRQ is not asserted — MAME emulates that with
            // restart_this_instruction()+spin_until_time(50us).
            //
            // Current contract (T1b reconciliation, 2026-07-23; corrects
            // a stale comment here that described a level-hold scheme
            // peripheral_bus.v never actually implements): peripheral_bus
            // does NOT re-assert pb_rd/pb_wr across the wait.  Instead it
            // withholds the scsi_rd/scsi_wr PULSE itself until this exact
            // condition (dma_shim_gate, mirrored via the dma_rd_ready /
            // dma_wr_ready exports below) is already false, so a pulse is
            // only ever issued on a cycle where drq_c96 is guaranteed
            // high and this withhold branch is guaranteed NOT to fire —
            // see rd_scsi_dma_shim_active / wr_scsi_dma_shim_active in
            // peripheral_bus.v for the pulse-gating mechanism (byte AND
            // word DMA-shim accesses both covered).  This branch is kept
            // as defense-in-depth (it should be dead code for any pulse
            // peripheral_bus actually issues) and to preserve bit-exact
            // behaviour for the bare-5380 path, which never consults the
            // check.  Only the C96 path consults the check; the bare-5380
            // branch keeps ack-on-pulse unchanged for tb-scsi compat.
            if ((pb_rd && c96_shim_rd_withhold) ||
                (pb_wr && c96_shim_wr_withhold)) begin
                pb_ack <= 1'b0;
            end else begin
                pb_ack <= pb_wr | pb_rd;
            end
            // c96_dma16_hi_granted (see its declaration): the grant is
            // recorded on the exact cycle the HIGH half of a DRQ-checked
            // 16-bit DMA-port read is acked, consumed by its LOW-half
            // partner beat, and dropped by any non-shim register access
            // (a command / count write ends the pair unambiguously, and
            // peripheral_bus.v's same-slot interlock guarantees nothing
            // can interleave between the two halves of one word).
            //
            // c96_dma16_degrade (see its declaration) rides the exact
            // same pairing: latched from the pre-access FIFO/tcounter
            // state on the HIGH beat, consumed by the LOW beat.  Unlike
            // the grant it is latched whether or not the DRQ check is
            // enabled, because MAME's dafb reaches dma16_*_w/r through
            // both the checked and the blind branch.
            if (!TURBOSCSI_C96_EN) begin
                c96_dma16_hi_granted <= 1'b0;
                c96_dma16_degrade    <= 1'b0;
            end else if (pb_rd && pb_dma_shim && !c96_shim_rd_withhold) begin
                c96_dma16_hi_granted <= scsi_ctrl_in[7] && !pb_dma16_lo_beat;
                if (!pb_dma16_lo_beat)
                    // Degrade only when MAME's settled model would: a
                    // FIFO short because the supply is LATE (armed
                    // DMA-in, bytes owed and available) must not
                    // swallow the low beat — see
                    // c96_dma16_rd_more_coming's block comment.
                    c96_dma16_degrade <= c96_dma16_rd_single &&
                                         !c96_dma16_rd_more_coming;
            end else if (pb_wr && pb_dma_shim && !c96_shim_wr_withhold) begin
                c96_dma16_hi_granted <= scsi_ctrl_in[8] && !pb_dma16_lo_beat;
                if (!pb_dma16_lo_beat)
                    c96_dma16_degrade <= c96_dma16_wr_single;
            end else if ((pb_rd || pb_wr) && !pb_dma_shim) begin
                c96_dma16_hi_granted <= 1'b0;
                c96_dma16_degrade    <= 1'b0;
            end
            // Track ACK edges every cycle (based on register-file state)
            ack_prev <= init_ack;
            // Track ATN edges for MSG_OUT entry (DISCONNECT etc.).
            atn_prev <= init_atn;
`ifdef VERILATOR
            if (phase != phase_prev) begin
                case (phase)
                    S_SELECT:
                        $display("%0t scsi: phase=SELECTION target_id=%0d select_data=%02x",
                                 $time, sel_tgt_id, r_output_data);
                    S_COMMAND:
                        $display("%0t scsi: phase=COMMAND", $time);
                    S_DATA_IN:
                        $display("%0t scsi: phase=DATA_IN opcode=%02x lba=%08x blocks_left=%0d bytes=%0d drq=1 irq=%0d",
                                 $time, cdb[0], xfer_lba, xfer_blocks,
                                 xfer_bytes_left, irq_live_5380);
                    S_DATA_OUT:
                        $display("%0t scsi: phase=DATA_OUT opcode=%02x lba=%08x blocks_left=%0d bytes=%0d drq=1 irq=%0d",
                                 $time, cdb[0], xfer_lba, xfer_blocks,
                                 xfer_bytes_left, irq_live_5380);
                    S_STATUS:
                        $display("%0t scsi: phase=STATUS status=%02x irq=1 end_dma=1",
                                 $time, xfer_status);
                    S_MSG_IN:
                        $display("%0t scsi: phase=MSG_IN msg=%02x irq=1",
                                 $time, xfer_msg);
                    S_DISCONNECT:
                        $display("%0t scsi: phase=DISCONNECT irq=%0d",
                                 $time, irq_pending);
                    default: ;
                endcase
            end
            phase_prev <= phase;
            if (irq_live_5380 != dbg_last_irq) begin
                $display("%0t scsi: irq=%0d phase=%0d status=%02x end_dma=%0d",
                         $time, irq_live_5380, phase, xfer_status,
                         completion_live);
                dbg_last_irq <= irq_live_5380;
            end
`endif
            // ── Register-file write path ─────────────────────────────
            // C96 path: the DAFB-mediated pseudo-DMA shim at 0x100/0x101
            // passes a single byte to the chip per access; treat each
            // shim write the same way MAME's `dma_w()` does — push to
            // FIFO and decrement the transfer counter (when DMA-armed).
            if (pb_wr && TURBOSCSI_C96_EN && pb_dma_shim &&
                !c96_dma16_lo_swallow) begin
                if (c96_sel_stopped && c96_xfr_armed && c96_xfr_dma) begin
                    // Post-ATN_STOP message-drain beat: the byte goes to
                    // the target as a message (content only matters for
                    // the IDENTIFY already latched in c96_stop_msg0);
                    // the drain ends at tcounter==0 (hook below).
                    if (c96_tcounter != 17'd0) begin
                        c96_tcounter <= c96_tcounter - 17'd1;
                        if (c96_tcounter == 17'd1)
                            c96_status_sticky <= c96_status_sticky | S_TC0;
                    end
                end else if (c96_sel_active ||
                             (c96_cmd_wait && c96_xfr_armed &&
                              c96_xfr_dma)) begin
                    // DMA-form select: the CDB tail arrives through the
                    // DAFB pseudo-DMA port (the ROM programs tcount=1 and
                    // sends the last CDB byte here — ROM 0x40898eb4).
                    // The select's DMA counter covers these bytes, so
                    // decrement tcounter and let TC0 go sticky — the
                    // ROM's post-select status poll expects 0x91
                    // (INTR|TC0|DATA_IN).
                    if (c96_tcounter != 17'd0) begin
                        c96_tcounter <= c96_tcounter - 17'd1;
                        if (c96_tcounter == 17'd1)
                            c96_status_sticky <= c96_status_sticky | S_TC0;
                    end
                    if (c96_sel_msg_out) begin
                        // ATN select: first byte is the MSG_OUT message
                        // (IDENTIFY) — consumed by the target, not part
                        // of the CDB.  The 0x43 form HALTS here (F2).
                        c96_sel_msg_out <= 1'b0;
                        c96_seq_step    <= 8'h02;
                        c96_identify_lun <= pb_wdata[7]
                                            ? pb_wdata[2:0] : 3'd0;
                        if (c96_sel_atn_stop) begin
                            c96_sel_active  <= 1'b0;
                            c96_sel_stopped <= 1'b1;
                            c96_stop_msg0   <= pb_wdata;
                            c96_istatus     <= c96_istatus |
                                               I_FUNCTION | I_BUS;
                            c96_irq_pending <= 1'b1;
                        end
                    end else begin
                        c96_sel_cdb_byte(pb_wdata,
                                         (c96_sel_idx == 6'd0)
                                         ? c96_cdb_len(pb_wdata)
                                         : c96_sel_len, c96_sel_active);
                    end
                end else if (!pseudo_dma_out_beat) begin
                    // FIFO push (mirror MAME dma_w behaviour) — EXCEPT
                    // when this very write is consumed as a data-out
                    // transfer beat.  MAME pushes then immediately pops
                    // via send_byte() (dma_w -> step -> INIT_XFR ->
                    // send_byte), so its settled FIFO is empty; pushing
                    // here as well double-booked the byte and left the
                    // FIFO reading 16 after every completed DMA-out
                    // (scsi_fuzz finding F6, 2026-08-19).  dma_w also
                    // decrements the transfer counter whenever the last
                    // valid command was a DMA form (finding F11).
                    if (c96_fifo_pos != 5'd16) begin
                        c96_fifo[c96_fifo_pos[3:0]] <= pb_wdata;
                        c96_fifo_pos                <= c96_fifo_pos + 5'd1;
                    end
                    if (c96_last_cmd_dma) begin
                        if ((c96_status_sticky & S_TC0) != 8'h00) begin
                            c96_tcounter <= 17'd0;
                        end else if (c96_tcounter != 17'd0) begin
                            c96_tcounter <= c96_tcounter - 17'd1;
                            if (c96_tcounter == 17'd1)
                                c96_status_sticky <= c96_status_sticky
                                                     | S_TC0;
                        end
                    end
                end
            end else if (pb_wr && TURBOSCSI_C96_EN && !pb_dma_shim) begin
                case (pb_addr[3:0])
                    4'h0: c96_tcount[7:0]  <= pb_wdata;
                    4'h1: c96_tcount[15:8] <= pb_wdata;
                    4'h2: begin
                        if (c96_sel_active) begin
                            // Deferred-select CDB byte (the ROM stuffs
                            // bytes 0..n-2 here after issuing CD_SELECT
                            // with an empty FIFO — ROM 0x40898e90).
                            if (c96_sel_msg_out) begin
                                // ATN select: MSG_OUT byte (IDENTIFY),
                                // not part of the CDB.  0x43 halts (F2).
                                c96_sel_msg_out <= 1'b0;
                                c96_seq_step    <= 8'h02;
                                c96_identify_lun <= pb_wdata[7]
                                                    ? pb_wdata[2:0] : 3'd0;
                                if (c96_sel_atn_stop) begin
                                    c96_sel_active  <= 1'b0;
                                    c96_sel_stopped <= 1'b1;
                                    c96_stop_msg0   <= pb_wdata;
                                    c96_istatus     <= c96_istatus |
                                                       I_FUNCTION | I_BUS;
                                    c96_irq_pending <= 1'b1;
                                end
                            end else begin
                                c96_sel_cdb_byte(pb_wdata,
                                                 (c96_sel_idx == 6'd0)
                                                 ? c96_cdb_len(pb_wdata)
                                                 : c96_sel_len, 1'b1);
                            end
                        end else if (c96_fifo_pos != 5'd16) begin
                            // FIFO push from CPU.  Saturates at 16 (per
                            // MAME).
                            c96_fifo[c96_fifo_pos[3:0]] <= pb_wdata;
                            c96_fifo_pos                <= c96_fifo_pos + 5'd1;
                        end
                    end
                    4'h3: begin
                        // Command register — MAME-faithful 2-deep queue
                        // (ncr53c90.cpp command_w / command_pop_and_chain /
                        // istatus_r; scsi_fuzz finding F8, 2026-08-19).
                        //   * a third command is dropped with S_GROSS_ERROR and no
                        //     interrupt — this test comes FIRST and applies to
                        //     EVERY opcode, RESET forms included (see
                        //     c96_cmd_is_reset_form).
                        //   * RESET / RESET_BUS otherwise execute the moment they
                        //     are written and clear the queue (command_w's
                        //     special-case forces command_pos = 0).
                        //   * execute-and-chain commands (NOP, FLUSH, ENABLE_SEL,
                        //     DISABLE_SEL, SET/RESET ATN) never occupy a slot.
                        //   * anything else occupies slot 0 and executes; a second
                        //     command queues in slot 1 and is dispatched by the
                        //     istatus read that retires slot 0
                        //     (command_pop_and_chain).
                        // Dispatch itself lives in the c96_cmd_dispatch block
                        // below this always block's register-access section, so
                        // the write path and the istatus-read pop path share one
                        // decoder.
                        if (c96_command_pos == 2'd2) begin
                            // queue full: dropped, gross error, NO interrupt (MAME
                            // command_w: status |= S_GROSS_ERROR without check_irq
                            // side effects on istatus)
                            c96_status_sticky <= c96_status_sticky | S_GROSS;
                        end else if (c96_cmd_is_reset_form) begin
                            c96_command_q   <= 8'h00;   // reset_disconnect memsets command[]
                            c96_cmd_q1      <= 8'h00;
                            c96_command_pos <= 2'd0;
                        end else if (c96_command_pos == 2'd0) begin
                            c96_command_q   <= pb_wdata;
                            if (!c96_cmd_autopop(pb_wdata))
                                c96_command_pos <= 2'd1;
                        end else if (c96_command_pos == 2'd1) begin
                            c96_cmd_q1      <= pb_wdata;
                            c96_command_pos <= 2'd2;
                        end
                    end
                    4'h4: c96_bus_id         <= pb_wdata[2:0];
                    4'h5: c96_select_timeout <= pb_wdata;
                    4'h6: c96_sync_period    <= pb_wdata[4:0];
                    4'h7: c96_sync_offset    <= pb_wdata[3:0];
                    4'h8: c96_config1        <= pb_wdata;
                    // Clock-conversion factor (MAME clock_w,
                    // ncr53c90.cpp:1173-1176: `clock_conv = data & 0x07`).
                    // Scales every delay() the chip's arbitration chain
                    // takes, so the selection timeout depends on it.
                    4'h9: c96_clock_conv     <= pb_wdata[2:0];
                    // 4'ha = test_w  (write-only, no observable effect yet)
                    4'hb: c96_config2        <= pb_wdata;
                    4'hc: c96_config3        <= pb_wdata;
                    // 4'hf = fifo_align_w (write-only, no observable effect)
                    default: ;
                endcase
            end else if (pb_wr) begin
                if (!pb_dma_shim) begin
                    case (pb_addr[2:0])
                        3'd0: r_output_data <= pb_wdata;
                        3'd1: r_init_cmd    <= pb_wdata;
                        3'd2: r_mode        <= pb_wdata;
                        3'd3: r_tgt_cmd     <= pb_wdata;
                        // 5380 regs 4..7 are write-only command triggers
                        // (Select Enable, Start DMA Send/TgtRx/InitRx);
                        // we don't implement the 5380 pseudo-DMA engine
                        // (the TurboSCSI shim at 0x100/0x101 carries our
                        // bursts), so the writes are accepted-and-dropped.
                        // Adding storage here means a Vivado-stripped FF.
                        default: ;
                    endcase
                end
            end
            // Reset-IRQ register: reading reg 7 clears IRQ (5380 convention)
            if (!TURBOSCSI_C96_EN && pb_rd && (pb_addr == 9'h007)) begin
                irq_pending <= 1'b0;
                busy_error_pending <= 1'b0;
                end_dma_pending <= 1'b0;
            end
            // Blind pseudo-DMA READ (MAME dma_r, ncr53c90.cpp:1193-1206):
            // with the scsi_ctrl=0 aperture every shim read that is NOT
            // an active data-in beat pops the FIFO and — for a DMA-form
            // command with the bus outside DATA IN — decrements the
            // transfer counter, clamping at zero once TC0 is set.
            // (R1 rework: the `!pseudo_dma_in_beat` carve-out is gone —
            // armed data-in beats pop the same real FIFO as blind reads,
            // exactly like MAME dma_r(); only the tcounter side effect
            // below distinguishes the bus phase.)
            if (TURBOSCSI_C96_EN && pb_rd && pb_dma_shim &&
                !c96_dma16_lo_swallow) begin
                if (c96_fifo_pos != 5'd0) begin
                    // MAME fifo_pop() (ncr53c90.cpp:837-845) shifts with
                    // memmove(fifo, fifo+1, --fifo_pos): only the slots
                    // BELOW the new occupancy move; slots >= pos-1 keep
                    // their old contents, so after draining the last
                    // byte fifo[0] goes on reading that byte forever.
                    // Blind dma_r of an empty FIFO returns fifo[0], so
                    // this residue is architecturally visible — the old
                    // shift-all-with-0x00-fill left different garbage
                    // (scsi_fuzz seeds 9/31/75/94: repeated stale bytes
                    // vs MAME's repeated last-popped byte).
                    for (k = 0; k < 15; k = k + 1)
                        if (k + 1 < {27'd0, c96_fifo_pos})
                            c96_fifo[k] <= c96_fifo[k + 1];
                    c96_fifo_pos <= c96_fifo_pos - 5'd1;
                end
                // MAME dma_r() (ncr53c90.cpp:1198-1201):
                //   if ((sync_offset != 0) ||
                //       ((ctrl & S_PHASE_MASK) != S_PHASE_DATA_IN))
                //       decrement_tcounter();
                // The sync disjunct was missing.  It is the DACK half of
                // an EXCLUSIVE pair with the ACKO half in the receive
                // path (:472-477, gated on `sync_offset == 0`): async
                // data-in counts down as the CHIP accepts bytes, sync
                // data-in counts down as the HOST pops them.  We only
                // implemented the async half, so a driver that
                // negotiated a non-zero sync offset (register 7 is
                // fully driver-writable — see the 4'h7 case above, and
                // drq_c96 already honours it) counted down up to a
                // FIFO-depth too early, and the sync IN DRQ formula
                // (`!TC0 && fifo_pos > 1`) then went low with bytes
                // still staged.
                if (c96_last_cmd_dma &&
                    ((c96_sync_offset != 4'd0) || (phase != S_DATA_IN))) begin
                    if ((c96_status_sticky & S_TC0) != 8'h00) begin
                        c96_tcounter <= 17'd0;
                    end else if (c96_tcounter != 17'd0) begin
                        c96_tcounter <= c96_tcounter - 17'd1;
                        if (c96_tcounter == 17'd1)
                            c96_status_sticky <= c96_status_sticky | S_TC0;
                    end
                end
            end
            if (TURBOSCSI_C96_EN && pb_rd && !pb_dma_shim) begin
                // FIFO pop side-effect on offset 2 read.
                if (pb_addr[3:0] == 4'h2) begin
                    if (c96_fifo_pos != 5'd0) begin
                        // MAME fifo_r(): memmove pop — slots >= pos-1
                        // keep their contents (see blind dma_r pop).
                        for (k = 0; k < 15; k = k + 1) begin
                            if (k + 1 < {27'd0, c96_fifo_pos})
                                c96_fifo[k] <= c96_fifo[k + 1];
                        end
                        c96_fifo_pos <= c96_fifo_pos - 5'd1;
                    end
                end
                // istatus read clears IRQ + sticky status (MAME convention).
                else if (pb_addr[3:0] == 4'h5) begin
                    if (c96_irq_pending) begin
                        c96_status_sticky <= c96_status_sticky &
                                             ~(S_GROSS | S_PARITY | S_TCC);
                        c96_istatus       <= 8'h00;
                        c96_seq_step      <= 8'h00;
                    end
                    c96_irq_pending           <= 1'b0;
                    c96_select_timeout_active <= 1'b0;
                    // MAME command_pop_and_chain: a nonzero istatus read
                    // retires the current command; a queued successor is
                    // dispatched this same cycle (c96_cmd_dispatch_pop)
                    if (c96_istatus != 8'h00) begin
                        if (c96_command_pos == 2'd2) begin
                            c96_command_q   <= c96_cmd_q1;
                            c96_command_pos <=
                                (c96_cmd_autopop(c96_cmd_q1) && c96_cmd_valid)
                                ? 2'd0 : 2'd1;
                        end else if (c96_command_pos != 2'd0) begin
                            c96_command_pos <= 2'd0;
                        end
                    end
                end
                // Poll-counted select timeout (mechanism 2 of 2) — after
                // `select_timeout_limit` seq_step polls (offset 6), assert
                // I_DISCONNECT.  The Q700 ROM and the 7.0.1-era SCSI
                // Manager both spin on seq_step while a select is
                // outstanding, so they reach the disconnect this way long
                // before the real chip timer below expires.  SCSI Manager
                // 4.3 does NOT poll — it depends on the timer.
                else if ((pb_addr[3:0] == 4'h6) &&
                         c96_select_timeout_active && !c96_irq_pending) begin
                    if (c96_select_timeout_polls + 8'h01 >=
                        c96_select_timeout_limit) begin
                        c96_istatus     <= I_DISCONNECT;
                        c96_irq_pending <= 1'b1;
                        c96_select_timeout_active <= 1'b0;
                    end
                    c96_select_timeout_polls <= c96_select_timeout_polls + 8'h01;
                end
            end
            // ── C96 command dispatch (write path + istatus-read pop path) ──
            // See the queue comment at the reg-3 write arm.  Placed AFTER
            // the register read side-effects so a command dispatched by the
            // pop path overrides the istatus clear the same read performed
            // (MAME: istatus_r clears, then command_pop_and_chain starts the
            // successor, which may post a fresh interrupt).
            if (TURBOSCSI_C96_EN && c96_cmd_dispatch) begin
                if (!c96_cmd_valid) begin
                    // MAME start_command: check_valid_command failed ->
                    // I_ILLEGAL, command keeps its queue slot
                    c96_istatus     <= c96_istatus_base | I_ILLEGAL;
                    c96_irq_pending <= 1'b1;
                end else begin
                    // MAME start_command: dma_command latches the DMA bit
                    // of every VALID command
                    c96_last_cmd_dma <= c96_cmd_cur[7];
                    // dma_set() per command class (stale otherwise)
                    case (c96_cmd_cur & 8'h7f)
                        CD_SELECT, CD_SELECT_ATN, CD_SELECT_ATN_STOP:
                            c96_dma_dir <= c96_cmd_cur[7]
                                           ? C96_DIR_OUT : C96_DIR_NONE;
                        CI_XFER:
                            c96_dma_dir <= !c96_cmd_cur[7]
                                ? C96_DIR_NONE
                                : (((phase == S_DATA_IN) ||
                                    (phase == S_STATUS) ||
                                    (phase == S_MSG_IN))
                                   ? C96_DIR_IN : C96_DIR_OUT);
                        CI_COMPLETE:
                            c96_dma_dir <= c96_cmd_cur[7]
                                           ? C96_DIR_IN : C96_DIR_NONE;
                        default: ;   // incl. CI_MSG_ACCEPT: unchanged
                    endcase
                    // load_tcounter for DMA-form commands; non-DMA
                    // commands zero the counter (MAME start_command)
                    if (c96_cmd_cur[7]) begin
                        c96_tcounter      <= (c96_tcount == 16'd0)
                                             ? 17'h10000
                                             : {1'b0, c96_tcount};
                        c96_status_sticky <= c96_status_sticky & ~S_TC0;
                    end else begin
                        c96_tcounter <= 17'h00000;
                    end
                        case (c96_cmd_cur & 8'h7f)
                            CM_NOP: begin
                                // No-op (execute-and-chain; queue handled
                                // by the reg-3 arm / pop path)
                            end
                            CM_FLUSH_FIFO: begin
                                c96_fifo_pos    <= 5'd0;
                            end
                            CM_RESET: begin
                                // Soft reset chip — equivalent to
                                // device_reset() in MAME.  Drops irq, clears
                                // sticky, zeroes counters.  Match MAME for
                                // fidelity.
                                c96_tcount         <= 16'h0000;
                                c96_tcounter       <= 17'h00000;
                                c96_status_sticky  <= 8'h00;
                                c96_istatus        <= 8'h00;
                                c96_seq_step       <= 8'h00;
                                c96_fifo_pos       <= 5'd0;
                                // MAME device_reset() memsets the FIFO
                                // CONTENTS, not just the count
                                // (ncr53c90.cpp:248-249) — the residue
                                // is visible through blind dma_r pops
                                // of the empty FIFO (scsi_fuzz seeds
                                // 31/75: repeated stale byte vs 0x00)
                                for (k = 0; k < 16; k = k + 1)
                                    c96_fifo[k] <= 8'h00;
                                c96_irq_pending    <= 1'b0;
                                c96_command_pos    <= 2'd0;
                                c96_cmd_q1         <= 8'h00;
                                c96_clock_conv     <= 3'd2;
                                c96_select_timeout_active <= 1'b0;
                                c96_select_timeout_polls  <= 8'h00;
                                c96_select_timeout_limit  <= 8'h00;
                                c96_select_timeout_cyc    <= 25'd0;
                                c96_xfer_active           <= 1'b0;
                                c96_sel_pending           <= 1'b0;
                                c96_xfr_armed             <= 1'b0;
                                c96_xfr_dma               <= 1'b0;
                                c96_xfr_recv_pend         <= 1'b0;
                                c96_sel_active            <= 1'b0;
                                c96_sel_dma               <= 1'b0;
                                c96_sel_len               <= 6'd0;
                                c96_sel_idx               <= 6'd0;
                                c96_sel_msg_out           <= 1'b0;
                                c96_sel_stopped           <= 1'b0;
                                c96_sel_atn_stop          <= 1'b0;
                                c96_cmd_wait              <= 1'b0;
                                c96_identify_lun          <= 3'd0;
                                c96_xfr_left              <= 17'd0;
                                c96_accept_pend           <= 10'd0;
                                c96_msgin_ack_held        <= 1'b0;
                                // 53C90A clears config2 on reset.
                                // 53C94 clears config3 on reset.
                                // 53C90 keeps config[2:0] only.
                                c96_config1 <= c96_config1 & 8'h07;
                                c96_config2 <= 8'h00;
                                c96_config3 <= 8'h00;
                            end
                            CM_RESET_BUS: begin
                                // Reset SCSI bus — fires SCSI_RESET istatus
                                // unless config1[6] (interrupt-disable on
                                // bus-reset) is set, per MAME.
                                // The FIFO is deliberately NOT flushed:
                                // MAME's CM_RESET_BUS chain
                                // (BUSRESET_WAIT_INT → reset_disconnect)
                                // touches neither fifo_pos nor the
                                // contents — only CM_RESET and
                                // CM_FLUSH_FIFO do (measured 2026-08-19,
                                // fifo retained across a 0x03 in fuzz
                                // scripts).
                                c96_status_sticky  <= 8'h00;
                                c96_seq_step       <= 8'h00;
                                c96_command_pos    <= 2'd0;
                                c96_cmd_q1         <= 8'h00;
                                c96_select_timeout_active <= 1'b0;
                                c96_select_timeout_polls  <= 8'h00;
                                c96_select_timeout_cyc    <= 25'd0;
                                // A SCSI bus reset (RST asserted) forces
                                // every target to Bus Free — this is the
                                // ONLY C96-mode path that can recover the
                                // back-end from an abandoned transfer
                                // (there is no 5380 init_rst register in
                                // C96 mode).  Mirrors the init_rst branch.
                                c96_xfer_active <= 1'b0;
                                c96_sel_pending <= 1'b0;
                                c96_xfr_armed   <= 1'b0;
                                c96_xfr_dma     <= 1'b0;
                                c96_xfr_recv_pend <= 1'b0;
                                c96_sel_active  <= 1'b0;
                                c96_sel_dma     <= 1'b0;
                                c96_sel_len     <= 6'd0;
                                c96_sel_idx     <= 6'd0;
                                c96_sel_msg_out <= 1'b0;
                                c96_sel_stopped <= 1'b0;
                                c96_sel_atn_stop <= 1'b0;
                                c96_cmd_wait    <= 1'b0;
                                c96_identify_lun <= 3'd0;
                                c96_xfr_left    <= 17'd0;
                                c96_accept_pend <= 10'd0;
                                c96_msgin_ack_held <= 1'b0;
                                phase           <= S_BUS_FREE;
                                t_req           <= 1'b0;
                                disc_pending    <= 1'b0;
                                ident_msg_sent  <= 1'b0;
                                msg_out_byte_seen <= 1'b0;
                                // MAME ncr53c90 CM_RESET_BUS only ORs
                                // I_SCSI_RESET in when the interrupt is
                                // ENABLED; when config1[6] disables it the
                                // chip leaves istatus/IRQ ALONE.  Clearing
                                // them here destroyed an interrupt that had
                                // already been posted and not yet read by
                                // software, leaving VIA2's edge-latched
                                // IFR.CB2 set with NO chip-side cause: the
                                // driver ISR chain polls the 53C96, sees
                                // istatus=0, claims nothing and never writes
                                // IFR -- and the level-sensitive via2_irq
                                // then re-fires forever.  irq_agg's strict
                                // priority makes that held ipl=2 starve the
                                // 60 Hz VIA1 tick, so the machine crawls.
                                // Measured 2026-09-14 on two independent
                                // 200 MHz builds: ~247k level-2 exc/s with
                                // the 53C96 completely idle (istatus=0,
                                // phase=BUS_FREE, completions frozen).
                                // Also OR rather than replace, so a reset
                                // interrupt cannot drop other istatus bits.
                                if (!c96_config1[6]) begin
                                    c96_istatus     <= c96_istatus | I_SCSI_RESET;
                                    c96_irq_pending <= 1'b1;
                                end
                            end
                            CI_COMPLETE: begin
                                // Initiator: command-complete sequence —
                                // per MAME INIT_CPT_* the chip pulls the
                                // status byte and the message byte off the
                                // bus into the FIFO, keeps ACK asserted on
                                // the message byte, and fires I_FUNCTION
                                // (function_complete()).  Return the REAL
                                // transaction status/message — the pre-M5
                                // stub pushed hardcoded GOOD/COMPLETE,
                                // which would have masked CHECK CONDITION
                                // from the initiator forever.
                                c96_seq_step    <= 8'h00;
                                c96_dma_dir     <= C96_DIR_NONE;
                                if (phase == S_STATUS) begin
                                    // The canonical sequence: pull the
                                    // status byte and the message byte
                                    // into the FIFO, I_FUNCTION, park in
                                    // MSG_IN until CI_MSG_ACCEPT.
                                    c96_istatus     <= c96_istatus_base
                                                       | I_FUNCTION;
                                    c96_irq_pending <= 1'b1;
                                    if (c96_fifo_pos <= 5'd14) begin
                                        c96_fifo[c96_fifo_pos[3:0]] <= xfer_status;
                                        c96_fifo[c96_fifo_pos[3:0] + 4'd1] <= xfer_msg;
                                        c96_fifo_pos <= c96_fifo_pos + 5'd2;
                                    end else if (c96_fifo_pos == 5'd15) begin
                                        c96_fifo[15] <= xfer_status;
                                        c96_fifo_pos <= 5'd16;
                                    end
                                    if (c96_xfer_active)
                                        phase <= S_MSG_IN;
                                    // msg byte held un-ACKed until
                                    // CI_MSG_ACCEPT (INIT_CPT_RECV_
                                    // BYTE_NACK)
                                    c96_msgin_ack_held <= 1'b1;
                                end else if ((phase == S_MSG_IN) &&
                                             c96_xfer_active) begin
                                    // Status already consumed out of
                                    // band (e.g. a DMA CI_XFER armed in
                                    // STATUS pulled it): only the
                                    // message byte is left.  MAME's
                                    // INIT_CPT_RECV_BYTE_ACK receives it
                                    // and RELEASES ACK (:572-575) — the
                                    // COMMAND COMPLETE message is done,
                                    // the target drops BSY, and the
                                    // step() bus-free detector fires
                                    // I_DISCONNECT (:314-318), NOT the
                                    // I_FUNCTION of the normal
                                    // STATUS-entry path whose msg byte
                                    // is held un-ACKed (measured vs
                                    // MAME 0.285, scsi_fuzz seed 4
                                    // SYNC 4: istat=20, bus free).
                                    c96_istatus     <= c96_istatus_base
                                                       | I_DISCONNECT;
                                    c96_irq_pending <= 1'b1;
                                    if (c96_fifo_pos != 5'd16) begin
                                        c96_fifo[c96_fifo_pos[3:0]] <= xfer_msg;
                                        c96_fifo_pos <= c96_fifo_pos + 5'd1;
                                    end
                                    c96_xfer_active <= 1'b0;
                                    c96_xfr_armed   <= 1'b0;
                                    c96_xfr_dma     <= 1'b0;
                                    c96_sel_pending <= 1'b0;
                                    c96_xfr_left    <= 17'd0;
                                    c96_accept_pend <= 10'd0;
                                    c96_msgin_ack_held <= 1'b0;
                                    phase           <= S_DISCONNECT;
                                    t_req           <= 1'b0;
                                end else if (phase == S_MSG_IN) begin
                                    // Not connected through the C96
                                    // shortcut backend: keep the plain
                                    // message pull.
                                    c96_istatus     <= c96_istatus_base
                                                       | I_FUNCTION;
                                    c96_irq_pending <= 1'b1;
                                    if (c96_fifo_pos != 5'd16) begin
                                        c96_fifo[c96_fifo_pos[3:0]] <= xfer_msg;
                                        c96_fifo_pos <= c96_fifo_pos + 5'd1;
                                    end
                                end else if (TURBOSCSI_C96_EN &&
                                             c96_xfer_active &&
                                             (phase == S_DATA_IN)) begin
                                    // CI_COMPLETE while the target still
                                    // drives DATA IN.  MAME's recv_byte()
                                    // pulls a byte off the LIVE phase, so
                                    // the FIFO gets the next REAL payload
                                    // byte (scsi_fuzz seed 41: the
                                    // INQUIRY byte, seed 43: the READ(6)
                                    // byte) — not the 0x00 of the
                                    // out-phase arm below.  Completion is
                                    // the same INIT_CPT_RECV_WAIT_REQ
                                    // path (ncr53c90.cpp:578-589): phase
                                    // is still != MSG_IN, so
                                    // `command_pos = 0` + bus_complete().
                                    // The push itself is deferred to
                                    // c96_cpt_din_beat, the only place
                                    // that can hand over a target byte
                                    // and advance the supply pointers.
                                    c96_istatus      <= c96_istatus_base
                                                        | I_BUS;
                                    c96_irq_pending  <= 1'b1;
                                    c96_command_pos  <= 2'd0;
                                    c96_cpt_din_pend <= 1'b1;
                                end else begin
                                    // CI_COMPLETE in an out-group phase:
                                    // measured vs MAME 0.285 (scsi_fuzz
                                    // seed 10): ONE 0x00 byte lands in
                                    // the FIFO and the command ends with
                                    // I_BUS, not I_FUNCTION.  MAME's
                                    // INIT_CPT_RECV_WAIT_REQ arm
                                    // (ncr53c90.cpp:583-585) also zeroes
                                    // the command queue on this path
                                    // (`command_pos = 0`).  Nothing is
                                    // driving the data lines in an out
                                    // phase, hence data_r() == 0x00.
                                    c96_istatus     <= c96_istatus_base
                                                       | I_BUS;
                                    c96_irq_pending <= 1'b1;
                                    c96_command_pos <= 2'd0;
                                    if (c96_fifo_pos != 5'd16) begin
                                        c96_fifo[c96_fifo_pos[3:0]] <= 8'h00;
                                        c96_fifo_pos <= c96_fifo_pos + 5'd1;
                                    end
                                end
                            end
                            CI_MSG_ACCEPT: begin
                                // Initiator: send a "message accepted" ACK,
                                // releasing the SCSI bus.  After that the
                                // chip emits I_DISCONNECT.
                                c96_istatus     <=
                                    (c96_xfer_active && (phase == S_MSG_IN))
                                    ? (c96_istatus_base | I_DISCONNECT)
                                    : (c96_istatus_base | I_BUS);
                                c96_irq_pending <= 1'b1;
                                c96_dma_dir     <= C96_DIR_NONE;
                                c96_msgin_ack_held <= 1'b0;  // ACK dropped
                                c96_seq_step    <= 8'd2;  // arbitrary 2 (MAME)
                                // FIFO deliberately untouched (MAME's
                                // CI_MSG_ACCEPT only releases ACK)
                                // Release the C96-held back-end: the
                                // target drops BSY and goes Bus Free
                                // (MAME INIT_MSG_WAIT_REQ → disconnect).
                                // Only a connection parked in MSG_IN
                                // (post-CI_COMPLETE) actually releases.
                                if (c96_xfer_active &&
                                    (phase == S_MSG_IN)) begin
                                    c96_xfer_active <= 1'b0;
                                    c96_xfr_armed   <= 1'b0;
                                    c96_xfr_dma     <= 1'b0;
                                    c96_sel_pending <= 1'b0;
                                    c96_xfr_left    <= 17'd0;
                                    c96_accept_pend <= 10'd0;
                                    phase           <= S_DISCONNECT;
                                    t_req           <= 1'b0;
                                end
                            end
                            CI_XFER: begin
                                // Transfer Information (0x10 non-DMA /
                                // 0x90 DMA) — MAME start_command() →
                                // INIT_XFR.  This is the command the Q700
                                // ROM issues to move the data phase after
                                // a successful select: DMA form arms the
                                // DAFB pseudo-DMA shim (DRQ may assert
                                // from now on), non-DMA form drains
                                // data-in bytes through the FIFO
                                // (c96_fifo_fill_beat).  Completion is
                                // signalled by the phase-change hook at
                                // the bottom of this block: when the
                                // back-end leaves the data phase for
                                // STATUS, istatus gets I_BUS (MAME
                                // bus_complete()).  Previously this
                                // opcode fell into the default arm and
                                // returned I_ILLEGAL — the ROM's boot
                                // disk scan abandoned the transfer right
                                // there, leaving the target parked in
                                // DATA_IN forever.
                                if (c96_xfer_active) begin
                                    c96_xfr_armed <= 1'b1;
                                    c96_xfr_dma   <= c96_cmd_cur[7];
                                    // MAME :606-610 skips the send (and
                                    // so never arms a timer) when the
                                    // FIFO is empty at dispatch — the
                                    // command PARKS with no interrupt.
                                    // See the c96_xfr_out_park comment.
                                    c96_xfr_out_park <=
                                        !c96_cmd_cur[7] &&
                                        ((phase == S_DATA_OUT) ||
                                         (phase == S_COMMAND)  ||
                                         (phase == S_MSG_OUT)) &&
                                        (c96_fifo_pos == 5'd0);
                                    c96_xfr_park_timer <= C96_RECV_QUIESCE;
                                    // DMA form armed in STATUS: the chip
                                    // RECVs the status byte into the
                                    // FIFO (no tcounter cost — ACK-
                                    // decrement is DATA IN only) and the
                                    // target advances to MSG_IN (MAME
                                    // INIT_XFR / S_PHASE_STATUS →
                                    // recv_byte; scsi_fuzz seed 14).
                                    // The recv is DEFERRED, not instant:
                                    // MAME's recv rides a delay_cycles
                                    // timer that only fires when
                                    // emulated time advances, so a burst
                                    // of blind dma_w beats issued right
                                    // after the arm lands in the FIFO
                                    // FIRST, and the status byte is
                                    // DROPPED at push time if they
                                    // filled it (fifo_push on a full
                                    // FIFO discards; measured scsi_fuzz
                                    // seed 5: MAME keeps 7 dma_w bytes
                                    // and no status byte where the
                                    // instant push kept status + 6).
                                    // c96_xfr_recv_* implements exactly
                                    // that: the pull fires only after
                                    // the DMA port has been quiet for
                                    // C96_RECV_QUIESCE cycles.
                                    if (c96_cmd_cur[7] &&
                                        (phase == S_STATUS)) begin
                                        c96_xfr_recv_pend   <= 1'b1;
                                        c96_xfr_recv_isstat <= 1'b1;
                                        c96_xfr_recv_timer  <= C96_RECV_QUIESCE;
                                    end else if (c96_cmd_cur[7] &&
                                                 (phase == S_MSG_IN)) begin
                                        // Same deferral for the message
                                        // byte of a MSG_IN-armed DMA
                                        // transfer.
                                        c96_xfr_recv_pend   <= 1'b1;
                                        c96_xfr_recv_isstat <= 1'b0;
                                        c96_xfr_recv_timer  <= C96_RECV_QUIESCE;
                                    end
                                    // MAME latches xfr_phase at command
                                    // start (INIT_XFR switches on it, and
                                    // INIT_XFR_WAIT_REQ compares the live
                                    // phase against it).  See the
                                    // c96_xfr_phase declaration.
                                    c96_xfr_phase <= phase;
                                    // DMA form: the chunk owes the CPU
                                    // exactly tcount beats; completion
                                    // (I_BUS) fires when tcounter has
                                    // been chip-accepted to 0 AND the
                                    // CPU has drained all beats — see
                                    // the chunk-completion hook below.
                                    c96_xfr_left    <= c96_cmd_cur[7]
                                                       ? ((c96_tcount == 16'd0)
                                                          ? 17'h10000 : {1'b0, c96_tcount})
                                                       : 17'd0;
                                    c96_accept_pend <= 10'd0;
                                end
                                // ── OUT-GROUP: COMMAND / MSG OUT ──────
                                // 2026-08-08.  MAME's INIT_XFR puts
                                // S_PHASE_COMMAND and S_PHASE_MSG_OUT on
                                // the SAME arm as S_PHASE_DATA_OUT
                                // (ncr53c90.cpp:601-616) — the third and
                                // fourth members of the phase class whose
                                // data-OUT member was fixed 2026-08-07.
                                // We implemented neither, and the CPU can
                                // see BOTH phases: a deferred-CDB select
                                // parks the back-end in S_COMMAND with
                                // c96_sel_active set (:2131-2160), and the
                                // ATN form additionally REPORTS MSG_OUT
                                // (110) through c96_phase_bits_eff
                                // (:1288-1291) until the IDENTIFY byte
                                // arrives.  A driver reading reg 4 in
                                // either state and issuing a non-DMA
                                // Transfer Information got NOTHING —
                                // c96_xfer_active is 0 for the whole
                                // deferred select (it is only set at CDB
                                // dispatch, :1410 / :2074), so the arm
                                // above misses, every completion hook
                                // below misses, and no istatus bit is ever
                                // set.  Measured in sim: 20000 reg-4 polls
                                // after W3=0x10, INTR never rises.  Same
                                // silent-forever shape as the DATA OUT
                                // stall of 2026-08-07.
                                //
                                // WHAT MAME DOES, step by step.  The FIFO
                                // is STRUCTURALLY EMPTY in this state:
                                // the progressive-drain byte sink
                                // (c96_sel_cdb_byte, :1421-1428) and the
                                // two write intercepts that call it
                                // (:1628-1652 shim, :1663-1682 reg 2)
                                // deliberately do NOT push c96_fifo, and
                                // the select itself zeroes c96_fifo_pos
                                // (:2146).  So:
                                //   :606-610  state = INIT_XFR_SEND_BYTE;
                                //             "can't send if the fifo is
                                //             empty" -> fifo_pos == 0 ->
                                //             break, NO send_byte()
                                // and that `break` is the WHOLE story: it
                                // leaves INIT_XFR without calling
                                // send_byte(), so NO delay timer is armed
                                // and step() is never re-entered from the
                                // chip side.  INIT_XFR_SEND_BYTE ->
                                // INIT_XFR_WAIT_REQ (:663-666) never runs,
                                // so the ":649 non-dma out: fifo empty"
                                // completion is UNREACHABLE.  MAME simply
                                // PARKS: no byte moved, NO interrupt, and
                                // the command keeps queue slot 0.
                                //
                                // 2026-08-19 CORRECTION (scsi_fuzz seed
                                // 27).  This arm used to raise I_BUS here,
                                // reading :649 without noticing that
                                // :608-610 prevents ever reaching it.
                                // Measured, minimal script (ATN_STOP
                                // select + DMA CI_XFER drain, then a bare
                                // `W 3 10` in COMMAND with an empty FIFO):
                                //   rtl  SYNC 3 stat=92 istat=10 irq=1
                                //   mame SYNC 3 stat=12 istat=00 irq=0
                                // and with ONE byte staged (`W 2 aa`
                                // inserted) the two sides agree exactly.
                                // The arm is therefore GONE; the empty
                                // case now falls through to no state
                                // change at all, which is what MAME does.
                                // c96_xfr_out_park carries the same rule
                                // for the transfers that DO arm (see the
                                // completion hooks lower down).
                                //
                                // NON-DMA ONLY (c96_cmd_cur[7] == 0).  The
                                // DMA form does NOT complete here: :644
                                // needs S_TC0, which a just-reloaded
                                // tcounter has cleared, and :649 requires
                                // !dma_command.  A DMA-form 0x90 in
                                // COMMAND phase is the ROM's own
                                // deferred-select tail flow, and it must
                                // keep waiting for the pseudo-DMA port to
                                // feed the CDB — completing it early would
                                // break boot.
                                //
                                // NOT gated on t_req, unlike
                                // c96_out_send_beat: our deferred select
                                // does not run the 5380 REQ/ACK handshake
                                // at all (t_req is cleared at :2160 and the
                                // byte sink at :1389-1431 replaces it), so
                                // MAME's `if(!(ctrl & S_REQ)) break;` at
                                // the head of INIT_XFR_WAIT_REQ has no
                                // counterpart to wait on here.
                                //
                                // ATN DEASSERT (:611-613) is deliberately
                                // NOT modelled: it fires only when
                                // remaining_bytes == fifo_pos +
                                // (dma ? tcounter : 0) == 1, and this arm
                                // requires fifo_pos == 0 with !dma, i.e.
                                // remaining_bytes == 0 — so MAME would not
                                // deassert either.  Independently, this
                                // module has NO chip-driven ATN: `init_atn`
                                // (:486) is an INPUT read out of the bare-
                                // 5380 register file, whose only write site
                                // (:2205-2210) is unreachable when
                                // TURBOSCSI_C96_EN is 1.  There is nothing
                                // to deassert and nothing that could
                                // observe it.
                                //
                                // (The deferred-select COMMAND-phase arm
                                // that used to sit here is deleted — see
                                // the 2026-08-19 correction above.  The
                                // FIFO is structurally empty throughout
                                // that state, so MAME always parks and so
                                // do we, by doing nothing.)
                                // ── Post-ATN_STOP message drain (F2) ──
                                else if (c96_sel_stopped) begin
                                    if (c96_cmd_cur[7]) begin
                                        // DMA form: staged FIFO bytes
                                        // drain one per cycle (the
                                        // c96_stop_feed hook — no
                                        // tcounter cost, per MAME
                                        // send_byte); further pseudo-DMA
                                        // beats are messages too, and
                                        // the drain ends at tcounter==0.
                                        c96_xfr_armed <= 1'b1;
                                        c96_xfr_dma   <= 1'b1;
                                        c96_xfr_phase <= S_MSG_OUT;
                                        // MAME's FIRST send is inside
                                        // start_command() itself — no
                                        // timer, no emulated time.
                                        c96_stop_feed_timer <= 10'd0;
                                        c96_stop_atn_dropped <= 1'b0;
                                    end else if (c96_fifo_pos != 5'd0) begin
                                        // non-DMA: every staged byte goes
                                        // out as a message, ATN drops
                                        // with the last, the target
                                        // processes the collected
                                        // messages NOW.
                                        c96_fifo_pos    <= 5'd0;
                                        c96_istatus     <= c96_istatus_base
                                                           | I_BUS;
                                        c96_irq_pending <= 1'b1;
                                        c96_sel_stopped <= 1'b0;
                                        // ATN dropped: the target
                                        // processes the collected
                                        // messages (IDENTIFY latches its
                                        // LUN; a bad LUN only matters
                                        // once a CDB executes) and waits
                                        // in COMMAND for a Transfer-
                                        // Information-fed CDB.
                                        c96_cmd_wait    <= 1'b1;
                                        c96_xfer_active <= 1'b1;
                                        c96_sel_idx     <= 6'd0;
                                        c96_sel_len     <= 6'd0;
                                        phase           <= S_COMMAND;
                                        t_req           <= 1'b0;
                                    end else begin
                                        // non-DMA, empty FIFO: nothing
                                        // moves, ATN stays up, still
                                        // halted — and NO interrupt.
                                        // MAME :608-610 breaks out of
                                        // INIT_XFR before send_byte(), so
                                        // no timer is armed and :649 is
                                        // never reached (2026-08-19
                                        // correction, scsi_fuzz seed 27;
                                        // the I_BUS this used to raise was
                                        // the same misreading as the
                                        // deleted COMMAND-phase arm).
                                    end
                                end
                            end
                            CD_ENABLE_SEL: begin
                                // Enable selection/reselection (0x44):
                                // MAME runs command_pop_and_chain() with
                                // no interrupt.  Silent accept — the ROM
                                // issues this between transactions; the
                                // previous default-arm I_ILLEGAL + IRQ
                                // injected a spurious error interrupt
                                // into the driver's command stream.
                            end
                            CD_DISABLE_SEL: begin
                                // Disable selection/reselection (0x45):
                                // MAME fires function_complete() →
                                // I_FUNCTION + dma_set(DMA_NONE) +
                                // check_drq (the drq_stale clear rides
                                // the dispatch-completes hook by the
                                // stale-recompute block).
                                c96_istatus     <= I_FUNCTION;
                                c96_irq_pending <= 1'b1;
                                c96_dma_dir     <= C96_DIR_NONE;
                            end
                            CD_RESELECT: begin
                                // CD_RESELECT — reconnect to a previously-
                                // disconnected target.  Per MAME
                                // ncr53c90.cpp:972-976 + ncr53c90.h:164.
                                // When disc_pending is set, kick the FSM
                                // into S_RESELECT so it drives BSY +
                                // (1<<sel_tgt_id) on the data bus, then
                                // sends IDENTIFY via MSG_IN, then resumes
                                // the suspended data phase.  Drivers see
                                // the chain complete via I_FUNCTION +
                                // I_RESELECTED (0x08|0x04 = 0x0C,
                                // ncr53c90.h:155-156).
                                if (disc_pending) begin
                                    phase           <= S_RESELECT;
                                    c96_istatus     <= I_FUNCTION | I_RESELECTED;
                                    c96_irq_pending <= 1'b1;
                                    c96_seq_step    <= 8'd4; // identify sent
                                end else begin
                                    // No suspended xfer: MAME arbitrates
                                    // and waits (silently, forever) to be
                                    // reselected.  No interrupt; the
                                    // command slot stays occupied until a
                                    // reset retires it.
                                    c96_seq_step    <= 8'h00;
                                end
                            end
                            CD_SELECT_ATN3: begin
                                // CD_SELECT_ATN3 (53c90a tagged-q sel) —
                                // not implemented; per ncr53c90.cpp the
                                // command is "valid" on the 53c90a (it
                                // appears in check_valid_command), but
                                // we don't model tagged queueing.  Return
                                // I_ILLEGAL (0x40, ncr53c90.h:152) so the
                                // driver doesn't silently hang.
                                c96_istatus     <= I_ILLEGAL;
                                c96_irq_pending <= 1'b1;
                                c96_seq_step    <= 8'h00;
                            end
                            CD_SELECT, CD_SELECT_ATN, CD_SELECT_ATN_STOP: begin
                                // Initiator: select target `bus_id`.
                                // Three paths:
                                //   • bus_id names a live target, CDB in FIFO:
                                //     synthetic short-circuit (M3) — load
                                //     cdb[] from c96_fifo, kick back-end
                                //     into S_CMD_EXEC.
                                //   • bus_id names a live target, empty FIFO:
                                //     deferred-CDB select (ROM boot scan).
                                //   • else — ANY absent ID, including 3:
                                //     arm both selection timeouts; the
                                //     chip timer fires I_DISCONNECT after
                                //     the programmed interval, and a
                                //     seq_step-polling driver gets there
                                //     sooner via the poll-counted shim.
                                // MAME start_command: seq=0; a still-
                                // unread istatus is NOT cleared here
                                c96_seq_step    <= 8'h00;
                                c96_select_timeout_polls <= 8'h00;
                                if (sel_id_hit(c96_bus_id) &&
                                    ((c96_cmd_cur & 8'h7f) ==
                                     CD_SELECT_ATN_STOP) &&
                                    (c96_fifo_pos != 5'd0)) begin
                                    // ATN_STOP with a preloaded FIFO:
                                    // send fifo[0] as the message and
                                    // HALT — seq=2, I_FUNCTION|I_BUS,
                                    // bus reporting MSG_OUT, the REST of
                                    // the FIFO retained (finding F2;
                                    // MAME ncr53c90.cpp:537-541).
                                    c96_select_timeout_active <= 1'b0;
                                    c96_select_timeout_limit  <= 8'h00;
                                    vh_dev_sel      <= sel_id_which(c96_bus_id);
                                    // MAME fifo_pop() memmove semantics
                                    // (see blind dma_r pop)
                                    for (k = 0; k < 15; k = k + 1)
                                        if (k + 1 < {27'd0, c96_fifo_pos})
                                            c96_fifo[k] <= c96_fifo[k + 1];
                                    c96_fifo_pos    <= c96_fifo_pos - 5'd1;
                                    c96_stop_msg0   <= c96_fifo[0];
                                    c96_identify_lun <= c96_fifo[0][7]
                                        ? c96_fifo[0][2:0] : 3'd0;
                                    c96_sel_stopped <= 1'b1;
                                    c96_seq_step    <= 8'h02;
                                    c96_istatus     <= c96_istatus_base |
                                                       I_FUNCTION | I_BUS;
                                    c96_irq_pending <= 1'b1;
                                end else if (sel_id_hit(c96_bus_id) &&
                                    (c96_sel_avail != 5'd0) &&
                                    ({2'b00, c96_sel_avail} >=
                                     {1'b0, c96_sel_cdblen})) begin
                                    // Synthetic selection: drive back-end
                                    // straight into S_CMD_EXEC with cdb[]
                                    // primed from the FIFO.  CDB length
                                    // determined from c96_fifo[0][7:5]
                                    // (0b001 / 0b010 = 10-byte; else 6).
                                    c96_select_timeout_active <= 1'b0;
                                    c96_select_timeout_limit  <= 8'h00;
                                    // Latch which volume this transaction
                                    // belongs to at the instant selection
                                    // succeeds (see vh_dev_sel).
                                    vh_dev_sel      <= sel_id_which(c96_bus_id);
                                    c96_xfer_active <= 1'b1;
                                    // Select-complete interrupt is raised
                                    // by the sel_pending hook once the
                                    // back-end settles in its first bus
                                    // phase — I_FUNCTION|I_BUS + seq=4
                                    // (MAME function_bus_complete()).
                                    c96_sel_pending <= 1'b1;
                                    c96_xfr_armed   <= 1'b0;
                                    c96_xfr_dma     <= 1'b0;
                                    // A stripped IDENTIFY latches its
                                    // LUN for this connection; a fresh
                                    // connection with no message keeps
                                    // LUN 0 (nscsi get_lun default)
                                    c96_identify_lun <=
                                        (c96_sel_strip && c96_fifo[0][7])
                                        ? c96_fifo[0][2:0] : 3'd0;
                                    // Consume message + exactly the
                                    // TARGET-determined CDB length;
                                    // everything beyond stays in the
                                    // FIFO (finding F1: MAME leaves the
                                    // residue; seq 4 only when nothing
                                    // is left and no DMA count is
                                    // outstanding, else 2).
                                    for (k = 0; k < 10; k = k + 1)
                                        cdb[k] <= (k < {26'd0,
                                                        c96_sel_cdblen})
                                            ? c96_fifo[c96_sel_base + k[3:0]]
                                            : 8'h00;
                                    cdb_idx <= 4'd0;
                                    cdb_len <= (c96_sel_cdblen
                                                > 6'd10)
                                               ? 4'd10
                                               : c96_sel_cdblen
                                                 [3:0];
                                    // MAME pops message+CDB one byte at
                                    // a time (send_byte → fifo_pop, a
                                    // memmove): slots below the old
                                    // occupancy end up holding the last
                                    // shifted byte replicated, slots at/
                                    // beyond it keep their contents —
                                    // NOT 0x00.  Blind dma_r of the
                                    // drained FIFO exposes the residue
                                    // (scsi_fuzz seeds 9/31/75).
                                    for (k = 0; k < 16; k = k + 1) begin
                                        if ((k + {28'd0, c96_sel_base}
                                               + {26'd0, c96_sel_cdblen})
                                            < {27'd0, c96_fifo_pos})
                                            c96_fifo[k] <=
                                                c96_fifo[k[3:0]
                                                         + c96_sel_base
                                                         + c96_sel_cdblen
                                                           [3:0]];
                                        else if (k < {27'd0, c96_fifo_pos})
                                            c96_fifo[k] <=
                                                c96_fifo[c96_fifo_pos[3:0]
                                                         - 4'd1];
                                    end
                                    c96_fifo_pos <= c96_fifo_pos
                                        - {1'b0, c96_sel_base}
                                        - {c96_sel_cdblen[4:0]};
                                    c96_sel_seq_final <=
                                        ((c96_fifo_pos ==
                                          ({1'b0, c96_sel_base} +
                                           {c96_sel_cdblen[4:0]}))
                                         && !c96_cmd_cur[7])
                                        ? 8'h04 : 8'h02;
                                    // Reset back-end pointers and jump
                                    // into S_CMD_EXEC on the next cycle.
                                    buf_rd_ptr      <= 9'd0;
                                    buf_wr_ptr      <= 9'd0;
                                    vh_fill_ptr     <= 9'd0;
                                    vh_drain_ptr    <= 9'd0;
                                    vh_buf_count    <= 10'd0;
                                    vh_wr_underflow <= 1'b0;
                                    xfer_bytes_left <= 16'd0;
                                    vh_kicked       <= 1'b0;
                                    vh_supply_faulted <= 1'b0;
                                    phase           <= S_CMD_EXEC;
                                    t_req           <= 1'b0;
                                end else if (sel_id_hit(c96_bus_id)) begin
                                    // Deferred-CDB select — the Q700 ROM
                                    // boot-scan flow: DMA|CD_SELECT with
                                    // an EMPTY FIFO.  MAME arbitrates +
                                    // selects immediately; the target
                                    // asserts BSY and enters COMMAND
                                    // phase before any CDB byte exists.
                                    // The ROM gate (0x40898e12) needs
                                    // seq_step!=0 (MAME's empty-fifo
                                    // arbitration hack sets seq=1,
                                    // ncr53c90.cpp:500-509) AND
                                    // status[2:0]!=0 (COMMAND=010)
                                    // before it stuffs the CDB.
                                    c96_select_timeout_active <= 1'b0;
                                    c96_select_timeout_limit  <= 8'h00;
                                    vh_dev_sel     <= sel_id_which(c96_bus_id);
                                    c96_sel_active <= 1'b1;
                                    // fresh connection: LUN 0 until an
                                    // IDENTIFY message says otherwise
                                    c96_identify_lun <= 3'd0;
                                    c96_sel_dma    <= c96_cmd_cur[7];
                                    c96_sel_atn_stop <=
                                        ((c96_cmd_cur & 8'h7f) ==
                                         CD_SELECT_ATN_STOP);
                                    c96_sel_len    <= (c96_sel_avail != 5'd0)
                                        ? c96_sel_cdblen : 6'd0;
                                    // Progressive drain: absorb any
                                    // preloaded FIFO bytes as the
                                    // already-sent CDB prefix so the
                                    // visible FIFO count reads 0 from
                                    // here on (see c96_sel_idx block
                                    // comment).
                                    for (k = 0; k < 10; k = k + 1) begin
                                        if (k < {27'd0, c96_sel_avail})
                                            cdb[k] <= c96_fifo[c96_sel_base + k[3:0]];
                                    end
                                    c96_sel_idx  <= {1'b0, c96_sel_avail};
                                    // MAME pops every preloaded byte
                                    // one at a time — memmove residue:
                                    // last byte replicated below the
                                    // old occupancy (see the bulk
                                    // consume above; blind dma_r
                                    // exposes it)
                                    for (k = 0; k < 16; k = k + 1)
                                        if (k < {27'd0, c96_fifo_pos})
                                            c96_fifo[k] <=
                                                c96_fifo[c96_fifo_pos[3:0]
                                                         - 4'd1];
                                    c96_fifo_pos <= 5'd0;
                                    // ATN-form empty-FIFO select owes a
                                    // MSG_OUT byte (IDENTIFY) before the
                                    // command phase — see c96_sel_msg_out
                                    // block comment.  Plain CD_SELECT
                                    // (the ROM boot-scan flow) goes
                                    // straight to COMMAND, unchanged.
                                    c96_sel_msg_out <=
                                        ((c96_cmd_cur & 8'h7f) != CD_SELECT) &&
                                        (c96_fifo_pos == 5'd0);
                                    // seq: 1 while nothing has been
                                    // sent (MAME's empty-fifo
                                    // arbitration hack, :508); once
                                    // bytes are in flight the settled
                                    // value is 4 for a non-DMA select
                                    // (fifo drained — DISC_SEL_SEND_BYTE
                                    // :568) and 3 for the DMA form with
                                    // its count outstanding
                                    c96_seq_step   <= (c96_sel_avail != 5'd0)
                                                      ? (c96_cmd_cur[7]
                                                         ? 8'h03 : 8'h04)
                                                      : 8'h01;
                                    c96_xfr_armed  <= 1'b0;
                                    c96_xfr_dma    <= 1'b0;
                                    phase          <= S_COMMAND;
                                    t_req          <= 1'b0;
                                end else begin
                                    // Absent target.  Arm BOTH timeout
                                    // paths — see the c96_select_timeout_*
                                    // block comment near the declarations.
                                    // There is deliberately no ID-3 case
                                    // here: this machine has exactly two
                                    // SCSI devices (the SD-backed HDD and
                                    // the DDR RAM disk) and every other ID,
                                    // ID 3 included, is an absent target.
                                    c96_select_timeout_active <= 1'b1;
                                    c96_select_timeout_limit  <=
                                        (c96_select_timeout > 8'd13)
                                          ? ((c96_select_timeout - 8'd13) >> 1)
                                          : 8'h01;
                                    c96_select_timeout_cyc <= c96_sel_to_load;
                                end
                            end
                            8'h1a, 8'h1b: begin
                                // CI_SET_ATN / CI_RESET_ATN: no chip-
                                // driven ATN is modelled (see the CI_XFER
                                // ATN-deassert note); execute-and-chain
                                // with no observable effect, like MAME's
                                // ctrl_w + command_pop_and_chain.
                            end
                            default: begin
                                // Every reachable command is either
                                // decoded above or screened out by
                                // c96_cmd_valid (I_ILLEGAL) before this
                                // case runs.  CI_PAD (0x18/0x98) is the
                                // one valid-but-unmodelled straggler:
                                // MAME pads bytes until TC0; nothing in
                                // this machine issues it.  It occupies
                                // its queue slot silently.
                            end
                        endcase
                end
            end
            // ── Real 53C96 selection timer (mechanism 1 of 2) ─────────
            // Free-running chip-clock down-counter, armed on a select of
            // an ID with no responder.  Expiry is I_DISCONNECT + IRQ,
            // exactly as the poll shim above does it, and exactly as
            // MAME's ARB_TIMEOUT_ABORT does (ncr53c90.cpp:422-431).
            // No seq_step read is required — that is the whole point.
            //
            // Ordering note: this block sits AFTER the register write and
            // read paths in the same always block, so it would otherwise
            // win a same-cycle race against a command write or an istatus
            // read (both of which disarm the timeout using non-blocking
            // assignments, i.e. `c96_select_timeout_active` still reads
            // its old value here).  The explicit guards below hand the
            // cycle to the register access instead.
            if (TURBOSCSI_C96_EN) begin
                c96_clk_div_cnt <= (c96_clk_div_cnt + 8'd1 >= C96_CLK_DIV)
                                   ? 8'd0 : (c96_clk_div_cnt + 8'd1);
                if (c96_select_timeout_active && !c96_irq_pending &&
                    !c96_reg_wr_now && !c96_istatus_rd_now &&
                    !(c96_seqstep_rd_now &&
                      (c96_select_timeout_polls + 8'h01 >=
                       c96_select_timeout_limit)) &&
                    c96_chip_tick) begin
                    if (c96_select_timeout_cyc <= 25'd1) begin
                        c96_istatus               <= I_DISCONNECT;
                        c96_irq_pending           <= 1'b1;
                        c96_select_timeout_active <= 1'b0;
                        c96_select_timeout_cyc    <= 25'd0;
                    end else begin
                        c96_select_timeout_cyc <=
                            c96_select_timeout_cyc - 25'd1;
                    end
                end
            end
            // pb_rdata registered one cycle after request.  Held until
            // the next read — no self-clear.
            if (pb_rd) pb_rdata <= pb_rd_mux;
`ifdef VERILATOR
            // Low-noise ROM bring-up observability: the Quadra ROM polls
            // Current SCSI Bus Status and Bus-and-Status while waiting for
            // REQ/phase/IRQ.  Ignore per-byte REQ/DRQ/ACK chatter, but log
            // phase/IRQ changes and periodic long polls.
            if (pb_rd && (pb_addr == 9'h004)) begin
                dbg_reg4_polls <= dbg_reg4_polls + 16'd1;
                if (((pb_rd_mux & 8'hDF) != (dbg_last_reg4 & 8'hDF)) ||
                    (dbg_reg4_polls[9:0] == 10'h000)) begin
                    $display("%0t scsi: poll reg4 count=%0d value=%02x",
                             $time, dbg_reg4_polls + 16'd1, pb_rd_mux);
                end
                dbg_last_reg4 <= pb_rd_mux;
            end
            if (pb_rd && (pb_addr == 9'h005)) begin
                dbg_reg5_polls <= dbg_reg5_polls + 16'd1;
                if (((pb_rd_mux & 8'hBE) != (dbg_last_reg5 & 8'hBE)) ||
                    (dbg_reg5_polls[7:0] == 8'h00)) begin
                    $display("%0t scsi: poll reg5 count=%0d value=%02x",
                             $time, dbg_reg5_polls + 16'd1, pb_rd_mux);
                end
                dbg_last_reg5 <= pb_rd_mux;
            end
            if (pb_rd && (pb_addr == 9'h007)) begin
                dbg_reg7_polls <= dbg_reg7_polls + 16'd1;
                $display("%0t scsi: poll reg7 reset-irq count=%0d irq=%0d end_dma=%0d busyerr=%0d",
                         $time, dbg_reg7_polls + 16'd1, irq_pending,
                         end_dma_pending, busy_error_pending);
            end
`endif
            // ── RST on initiator command forces BUS_FREE ─────────────
            if (init_rst) begin
                phase        <= S_BUS_FREE;
                vh_dev_sel   <= 1'b0;
                cur_data     <= 8'h00;
                t_req <= 1'b0;
                irq_pending  <= 1'b0;
                end_dma_pending <= 1'b0;
                cdb_idx      <= 4'd0;
                buf_rd_ptr   <= 9'd0;
                buf_wr_ptr   <= 9'd0;
                vh_fill_ptr  <= 9'd0;
                vh_drain_ptr <= 9'd0;
                vh_buf_count <= 10'd0;
                vh_wr_underflow <= 1'b0;
                xfer_blocks  <= 24'd0;
                xfer_bytes_left <= 16'd0;
                vh_kicked    <= 1'b0;
                vh_wait_ctr  <= 16'd0;
                vh_stuck_ctr <= 24'd0;
                vh_supply_faulted <= 1'b0;
                busy_error_pending <= 1'b0;
                medium_not_present <= 1'b0;
                // RST also clears any suspended-disconnect state.
                disc_pending         <= 1'b0;
                ident_msg_sent       <= 1'b0;
                msg_out_byte_seen    <= 1'b0;
            end else begin
                // ── Phase FSM ──────────────────────────────────────
                case (phase)
                    // BUS_FREE: idle.  Trigger Selection when SEL asserted
                    // and the output-data bus shows our ID bit set.  Any
                    // other value → no-device; stay here.
                    S_BUS_FREE: begin
                        cur_data     <= 8'h00;
                        t_req <= 1'b0; 
                        cdb_idx <= 4'd0;
                        if (init_sel && sel_bus_hit &&
                            (r_output_data != 8'h00)) begin
                            // We're being selected.  Latch which of our
                            // target IDs the initiator asked for; that
                            // choice qualifies the whole transaction.
`ifdef VERILATOR
                            if (!dbg_select_seen) begin
                                $display("%0t scsi: selected target_id=%0d select_data=%02x",
                                         $time, sel_bus_id, r_output_data);
                                dbg_select_seen <= 1'b1;
                            end
`endif
                            vh_dev_sel <= sel_bus_which;
                            phase      <= S_SELECT;
                             // BSY + SEL echo
                        end else if (init_sel) begin
`ifdef VERILATOR
                            if (!dbg_select_seen) begin
                                $display("%0t scsi: selection ignored target_id=%0d select_data=%02x",
                                         $time, sel_bus_id, r_output_data);
                                dbg_select_seen <= 1'b1;
                            end
`endif
                        end
`ifdef VERILATOR
                        if (!init_sel) dbg_select_seen <= 1'b0;
`endif
                    end
                    // SELECT: wait for initiator to drop SEL, then enter
                    // COMMAND phase (C/D=1, I/O=0, REQ asserted).
                    S_SELECT: begin
                        if (!init_sel) begin
                            // Enter COMMAND phase (C/D=1, I/O=0)
                            phase        <= S_COMMAND;
                            t_req        <= 1'b1;
                            cdb_idx      <= 4'd0;
                            cdb_len      <= 4'd6;    // decide after opcode
                                        // BSY | REQ | CD
                               // PHASE_MATCH
                            // Sequence-register progression per MAME
                            // ncr53c90.cpp:506-560: seq=3 = "target
                            // responded with BSY in command phase".
                            // Drivers may poll seq during selection.
                            if (TURBOSCSI_C96_EN) begin
                                c96_seq_step <= 8'h03;
                            end
                        end
                    end
                    // COMMAND: REQ/ACK-pulse in each CDB byte.  Drop REQ
                    // on ACK rise; re-assert on ACK fall (unless done).
                    S_COMMAND: begin
                                        // BSY | CD | REQ-if-asserted
                            // PHASE_MATCH
                        if (t_req && ack_rise) begin
                            // Latch the byte from Mac's output data bus.
                            cdb[cdb_idx] <= r_output_data;
                            // After opcode (byte 0) decide CDB length.
                            if (cdb_idx == 4'd0) begin
                                if (r_output_data[7:5] == 3'b001 ||
                                    r_output_data[7:5] == 3'b010) begin
                                    cdb_len <= 4'd10;
                                end else begin
                                    cdb_len <= 4'd6;
                                end
                            end
                            t_req <= 1'b0;    // drop REQ until ACK drops
                        end else if (!t_req && ack_fall) begin
                            // Move to next byte or finish CDB.
                            if (cdb_idx + 4'd1 >= cdb_len) begin
                                // CDB complete; interpret.
`ifdef VERILATOR
                                if (cdb_len == 4'd10) begin
                                    $display("%0t scsi: CDB len=10 bytes=%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
                                             $time, cdb[0], cdb[1], cdb[2],
                                             cdb[3], cdb[4], cdb[5], cdb[6],
                                             cdb[7], cdb[8], cdb[9]);
                                end else begin
                                    $display("%0t scsi: CDB len=6 bytes=%02x %02x %02x %02x %02x %02x",
                                             $time, cdb[0], cdb[1], cdb[2],
                                             cdb[3], cdb[4], cdb[5]);
                                end
`endif
                                phase <= S_CMD_EXEC;
                                t_req <= 1'b0;
                                // seq=4 = "identify/CDB sent" — final step
                                // of selection per MAME ncr53c90.cpp:548.
                                if (TURBOSCSI_C96_EN) begin
                                    c96_seq_step <= 8'h04;
                                end
                            end else begin
                                cdb_idx <= cdb_idx + 4'd1;
                                t_req   <= 1'b1;
                            end
                        end
                    end
                    // CMD_EXEC: decode opcode, pre-load sec_buf for
                    // data-in flows, latch LBA/block count, kick volume.
                    S_CMD_EXEC: begin
                        `ifdef VERILATOR
                            $display("%0t scsi: opcode=%02x", $time, cdb[0]);
                        `endif
                        xfer_status <= 8'h00;         // default GOOD
                        xfer_msg    <= 8'h00;         // COMMAND COMPLETE
                        buf_rd_ptr  <= 9'd0;
                        buf_wr_ptr  <= 9'd0;
                        // Default: DATA_IN reads the real sector buffer.
                        // The four canned-payload opcodes below override
                        // this in the same cycle (later NBA wins).
                        canned_active <= 1'b0;
                        // Same idiom, same cycle: this command's data
                        // phase is NOT fed by the multi-block ring unless
                        // one of the READ/WRITE arms below says so.  This
                        // is the clear that stops the previous command's
                        // request class leaking into this one — see the
                        // data_from_ring block comment near the top.
                        data_from_ring <= 1'b0;
                        if (TURBOSCSI_C96_EN &&
                            (c96_identify_lun != 3'd0) &&
                            ((cdb[0] == 8'h00) || (cdb[0] == 8'h1a))) begin
                            // IDENTIFY named a LUN this target doesn't
                            // have.  2026-08-19 (scsi_fuzz seed 20): the
                            // golden target only enforces the LUN on
                            // TEST UNIT READY and MODE SENSE(6) —
                            // nscsi_hd.cpp:105-230 has exactly three
                            // get_lun() sites and only those two call
                            // bad_lun(); READ(6)/READ(10)/READ CAPACITY/
                            // WRITE answer with the LUN-0 data
                            // regardless, and INQUIRY returns data
                            // flagged with peripheral qualifier 0x7f
                            // (:171 — a device-model payload exclusion
                            // in the fuzzer).  The previous blanket
                            // every-command CHECK CONDITION sent a
                            // wrong-LUN READ CAPACITY to STATUS where
                            // MAME serves DATA IN.  bad_lun() =
                            // CHECK CONDITION / ILLEGAL REQUEST /
                            // LUN NOT SUPPORTED (nscsi_bus.cpp:860).
                            xfer_status <= 8'h02;
                            sense_key   <= 4'd5;   // ILLEGAL REQUEST
                            sense_asc   <= 8'h25;  // LUN NOT SUPPORTED
                            phase       <= S_STATUS;
                            t_req       <= 1'b0;
                        end else
                        case (cdb[0])
                            // ── 0x00 TEST UNIT READY ───────────────
                            8'h00: begin
                                if (medium_not_present || (vh_num_lbas == 32'd0)) begin
                                    xfer_status <= 8'h02;  // CHECK CONDITION
                                    sense_key   <= 4'd2;   // NOT READY
                                    sense_asc   <= 8'h3A;  // medium not present
                                    sense_ascq  <= 8'h00;
                                end
                                end_dma_pending <= 1'b1;
                                phase <= S_STATUS;
                                t_req <= 1'b1;
                            end
                            // ── 0x03 REQUEST SENSE ─────────────────
                            8'h03: begin
                                // Serve 18 B fixed-format sense data from
                                // the canned ROM.  sense_* is cleared a
                                // few lines below, so snapshot it here.
                                canned_active <= 1'b1;
                                canned_sel    <= CANNED_SENSE;
                                sns_key_q     <= sense_key;
                                sns_asc_q     <= sense_asc;
                                sns_ascq_q    <= sense_ascq;
                                xfer_bytes_left <= cap_alloc6(16'd18, cdb[4]);
                                if (cdb[4] == 8'd0) begin
                                    end_dma_pending <= 1'b1;
                                    phase <= S_STATUS;
                                    t_req <= 1'b1;
                                end else begin
                                    phase <= S_DATA_IN;
                                    t_req <= 1'b1;
                                end
                                // Clear sense once read.
                                sense_key  <= 4'd0;
                                sense_asc  <= 8'h00;
                                sense_ascq <= 8'h00;
                            end
                            // ── 0x12 INQUIRY ───────────────────────
                            8'h12: begin
                                // 36-byte standard INQUIRY.  Peripheral
                                // qual=0, dev type 0x00 = DIRECT ACCESS.
                                // Vendor "APPLE   ", product "HD SC ..."
                                // matches Apple SCSI-Manager-friendly
                                // identifiers (mirrors what MAME's
                                // nscsi_harddisk reports for an Apple HD SC).
                                // Byte-for-byte payload lives in
                                // canned_byte(CANNED_INQUIRY, ...).
                                canned_active <= 1'b1;
                                canned_sel    <= CANNED_INQUIRY;
                                // Byte 0 = 0x7F when the IDENTIFY named a
                                // LUN this target does not have
                                // (nscsi_hd.cpp:171-174).
                                inq_bad_lun_q <= TURBOSCSI_C96_EN &&
                                                 (c96_identify_lun != 3'd0);
                                // Supply EXACTLY the allocation length,
                                // zero-padded past the 36 valid bytes —
                                // nscsi_harddisk fills a 148-byte buffer
                                // and honors alloc verbatim, so an
                                // alloc > 36 must keep streaming zeros
                                // or the transfer counters diverge
                                // (scsi_fuzz clean-profile seed 4).
                                xfer_bytes_left <= {8'd0, cdb[4]};
                                if (cdb[4] == 8'd0) begin
                                    end_dma_pending <= 1'b1;
                                    phase <= S_STATUS;
                                    t_req <= 1'b1;
                                end else begin
                                    phase <= S_DATA_IN;
                                    t_req <= 1'b1;
                                end
                            end
                            // ── 0x1A MODE SENSE 6 ──────────────────
                            8'h1A: begin
                                // 4-byte mode parameter header (no pages).
                                canned_active <= 1'b1;
                                canned_sel    <= CANNED_MODE6;
                                xfer_bytes_left <= cap_alloc6(16'd4, cdb[4]);
                                if (cdb[4] == 8'd0) begin
                                    end_dma_pending <= 1'b1;
                                    phase <= S_STATUS;
                                    t_req <= 1'b1;
                                end else begin
                                    phase <= S_DATA_IN;
                                    t_req <= 1'b1;
                                end
                            end
                            // ── 0x1B START STOP UNIT ───────────────
                            8'h1B: begin
                                end_dma_pending <= 1'b1;
                                phase <= S_STATUS;
                                t_req <= 1'b1;
                            end
                            // ── 0x25 READ CAPACITY 10 ──────────────
                            8'h25: begin
                                if (medium_not_present || (vh_num_lbas == 32'd0)) begin
                                    xfer_status <= 8'h02;  // CHECK CONDITION
                                    sense_key   <= 4'd2;   // NOT READY
                                    sense_asc   <= 8'h3A;  // medium not present
                                    sense_ascq  <= 8'h00;
                                    end_dma_pending <= 1'b1;
                                    phase <= S_STATUS;
                                    t_req <= 1'b1;
                                end else begin
                                    // Returns last LBA (MSB-first) + 512 block.
                                    canned_active <= 1'b1;
                                    canned_sel    <= CANNED_CAP10;
                                    cap_lba_q     <= last_lba;
                                    xfer_bytes_left <= 16'd8;
                                    phase <= S_DATA_IN;
                                    t_req <= 1'b1;
                                end
                            end
                            // ── 0x08 READ 6 / 0x28 READ 10 ────────
                            8'h08, 8'h28: begin
                                if (cdb[0] == 8'h08) begin
                                    // READ 6: LBA in cdb[1][4:0], cdb[2], cdb[3]
`ifdef VERILATOR
                                    $display("%0t scsi: request READ6 lba=%08x blocks=%0d",
                                             $time,
                                             {11'd0, cdb[1][4:0], cdb[2], cdb[3]},
                                             (cdb[4] == 8'd0) ? 24'd256 : {16'd0, cdb[4]});
`endif
                                    xfer_lba    <= {11'd0,
                                                    cdb[1][4:0],
                                                    cdb[2], cdb[3]};
                                    xfer_blocks <= (cdb[4] == 8'd0)
                                                    ? 24'd256
                                                    : {16'd0, cdb[4]};
                                end else begin
                                    // READ 10
`ifdef VERILATOR
                                    $display("%0t scsi: request READ10 lba=%08x blocks=%0d",
                                             $time,
                                             {cdb[2], cdb[3], cdb[4], cdb[5]},
                                             {8'd0, cdb[7], cdb[8]});
`endif
                                    xfer_lba    <= {cdb[2], cdb[3],
                                                    cdb[4], cdb[5]};
                                    xfer_blocks <= {8'd0, cdb[7], cdb[8]};
                                end
                                if ((cdb[0] == 8'h28) &&
                                    (cdb[7] == 8'h00) && (cdb[8] == 8'h00)) begin
                                    end_dma_pending <= 1'b1;
                                    phase <= S_STATUS;
                                    t_req <= 1'b1;
                                end else begin
                                    // Kick a volume read for the whole
                                    // transfer.  blocks==1 → single-block
                                    // request (fast path); blocks>1 →
                                    // multi-block request, where the
                                    // provider streams N×512 bytes
                                    // back-to-back and only pulses
                                    // vh_done after the last one.
                                    if (cdb[0] == 8'h08) begin
                                        // READ(6) length is 1 byte; cdb[4]==0
                                        // means 256 blocks.
                                        if (cdb[4] == 8'd1) begin
                                            vh_req_write   <= 1'b0;  // single-block read
                                            vh_req_multi   <= 1'b0;
                                            data_from_ring <= 1'b0;
                                            vh_req_block_count <= 16'd1;
                                        end else begin
                                            vh_req_write   <= 1'b0;  // multi-block read
                                            vh_req_multi   <= 1'b1;
                                            data_from_ring <= 1'b1;
                                            vh_req_block_count <= (cdb[4] == 8'd0)
                                                ? 16'd256
                                                : {8'd0, cdb[4]};
                                        end
                                    end else begin
                                        // READ(10): blocks in cdb[7..8].
                                        if ((cdb[7] == 8'h00) &&
                                            (cdb[8] == 8'h01)) begin
                                            vh_req_write   <= 1'b0;  // single-block read
                                            vh_req_multi   <= 1'b0;
                                            data_from_ring <= 1'b0;
                                            vh_req_block_count <= 16'd1;
                                        end else begin
                                            vh_req_write   <= 1'b0;  // multi-block read
                                            vh_req_multi   <= 1'b1;
                                            data_from_ring <= 1'b1;
                                            vh_req_block_count <= {cdb[7], cdb[8]};
                                        end
                                    end
                                    vh_fill_ptr    <= 9'd0;
                                    buf_rd_ptr     <= 9'd0;
                                    vh_buf_count   <= 10'd0;
                                    vh_kicked      <= 1'b0;
                                    vh_supply_faulted <= 1'b0;
                                    phase          <= S_VH_WAIT_RD;
                                end
                            end
                            // ── 0x0A WRITE 6 / 0x2A WRITE 10 ──────
                            8'h0A, 8'h2A: if (wprot) begin
                                // Locked volume: report it the way a real
                                // target does, so the OS mounts read-only
                                // instead of believing a write succeeded.
                                // ASC 0x27 = WRITE PROTECTED.
                                xfer_status <= 8'h02;  // CHECK CONDITION
                                sense_key   <= 4'd7;   // DATA PROTECT
                                sense_asc   <= 8'h27;
                                sense_ascq  <= 8'h00;
                                end_dma_pending <= 1'b1;
                                phase <= S_STATUS;
                                t_req <= 1'b1;
                            end else begin
                                if (cdb[0] == 8'h0A) begin
`ifdef VERILATOR
                                    $display("%0t scsi: request WRITE6 lba=%08x blocks=%0d",
                                             $time,
                                             {11'd0, cdb[1][4:0], cdb[2], cdb[3]},
                                             (cdb[4] == 8'd0) ? 24'd256 : {16'd0, cdb[4]});
`endif
                                    xfer_lba    <= {11'd0,
                                                    cdb[1][4:0],
                                                    cdb[2], cdb[3]};
                                    xfer_blocks <= (cdb[4] == 8'd0)
                                                    ? 24'd256
                                                    : {16'd0, cdb[4]};
                                end else begin
`ifdef VERILATOR
                                    $display("%0t scsi: request WRITE10 lba=%08x blocks=%0d",
                                             $time,
                                             {cdb[2], cdb[3], cdb[4], cdb[5]},
                                             {8'd0, cdb[7], cdb[8]});
`endif
                                    xfer_lba    <= {cdb[2], cdb[3],
                                                    cdb[4], cdb[5]};
                                    xfer_blocks <= {8'd0, cdb[7], cdb[8]};
                                end
                                if ((cdb[0] == 8'h2A) &&
                                    (cdb[7] == 8'h00) && (cdb[8] == 8'h00)) begin
                                    end_dma_pending <= 1'b1;
                                    phase <= S_STATUS;
                                    t_req <= 1'b1;
                                end else begin
                                    // Enter DATA_OUT; the volume write kicks after
                                    // the first sector is drained into
                                    // sec_buf.  blocks==1 → single-block
                                    // write (fast path, kick happens at
                                    // the end of S_DATA_OUT and we go
                                    // through S_VH_WAIT_WR).  blocks>1 →
                                    // multi-block write (kick happens at the
                                    // end of the FIRST S_DATA_OUT block,
                                    // and S_DATA_OUT loops to receive
                                    // blocks 2..N concurrently with the
                                    // the provider drain).
                                    if (cdb[0] == 8'h0A) begin
                                        if (cdb[4] == 8'd1) begin
                                            vh_req_write   <= 1'b1;  // single-block write
                                            vh_req_multi   <= 1'b0;
                                            data_from_ring <= 1'b0;
                                            vh_req_block_count <= 16'd1;
                                        end else begin
                                            vh_req_write   <= 1'b1;  // multi-block write
                                            vh_req_multi   <= 1'b1;
                                            data_from_ring <= 1'b1;
                                            vh_req_block_count <= (cdb[4] == 8'd0)
                                                ? 16'd256
                                                : {8'd0, cdb[4]};
                                        end
                                    end else begin
                                        if ((cdb[7] == 8'h00) &&
                                            (cdb[8] == 8'h01)) begin
                                            vh_req_write   <= 1'b1;  // single-block write
                                            vh_req_multi   <= 1'b0;
                                            data_from_ring <= 1'b0;
                                            vh_req_block_count <= 16'd1;
                                        end else begin
                                            vh_req_write   <= 1'b1;  // multi-block write
                                            vh_req_multi   <= 1'b1;
                                            data_from_ring <= 1'b1;
                                            vh_req_block_count <= {cdb[7], cdb[8]};
                                        end
                                    end
                                    buf_wr_ptr     <= 9'd0;
                                    vh_drain_ptr   <= 9'd0;
                                    vh_buf_count   <= 10'd0;
                                    vh_wr_underflow <= 1'b0;
                                    vh_kicked      <= 1'b0;
                                    phase <= S_DATA_OUT;
                                    t_req <= 1'b1;
                                    xfer_bytes_left <= BLOCK_BYTES;
                                end
                            end
                            // ── 0x15 MODE SELECT 6 ────────────────
                            8'h15: begin
                                // Accept parameters (length in cdb[4]).
                                if (cdb[4] == 8'd0) begin
                                    phase <= S_STATUS;
                                     t_req <= 1'b1;
                                end else begin
                                    xfer_bytes_left <= {8'd0, cdb[4]};
                                    phase <= S_DATA_OUT;
                                     t_req <= 1'b1;
                                end
                            end
                            // ── default → CHECK CONDITION ────────
                            default: begin
                                xfer_status <= 8'h02;     // CHECK CONDITION
                                sense_key   <= 4'd5;      // ILLEGAL REQUEST
                                sense_asc   <= 8'h20;     // INVALID OPCODE
                                sense_ascq  <= 8'h00;
                                phase <= S_STATUS;
                                t_req <= 1'b1;
                            end
                        endcase
                    end
                    // VH_WAIT_RD: kick the volume read (once, via
                    // vh_kicked); drain vh_rd_valid bytes into sec_buf.
                    //
                    // Single block: wait for vh_done before entering
                    // DATA_IN.
                    //
                    // Multi-block: the provider streams N×512 bytes
                    // back-to-back and vh_done only fires after the whole
                    // run completes — too late for per-block pacing.
                    // Instead transition to DATA_IN as soon as the first
                    // byte is buffered (see the block comment further
                    // down) and let the DATA_IN loop keep draining
                    // vh_rd_valid concurrently while the initiator pulls
                    // the block.  This asymmetry is deliberate.
                    //
                    // vh_error is sticky, so we can check it once outside
                    // the per-block path.
                    S_VH_WAIT_RD: begin
                          // BSY only
                        if (!vh_chk_ok) begin
                            `ifdef VERILATOR
                                $display("%0t scsi: READ out of range lba=%08x blocks=%0d",
                                         $time, xfer_lba, xfer_blocks);
                            `endif
                            xfer_status <= 8'h02;  // CHECK CONDITION
                            sense_key   <= 4'd5;   // ILLEGAL REQUEST
                            sense_asc   <= 8'h21;  // LBA out of range
                            sense_ascq  <= 8'h00;
                            phase <= S_STATUS;
                            t_req <= 1'b1;
                        // Fire vh_req_go one cycle on entry.
                        end else if (!vh_kicked && !vh_busy && !vh_req_go) begin
                            `ifdef VERILATOR
                                if (vh_multi_read) begin
                                    $display("%0t scsi: vhdd multi-read lba=%08x blocks=%0d",
                                             $time, xfer_lba, xfer_blocks);
                                end else begin
                                    $display("%0t scsi: vhdd read lba=%08x blocks=%0d",
                                             $time, xfer_lba, xfer_blocks);
                                end
                            `endif
                            vh_req_lba    <= xfer_lba;
                            vh_req_go     <= 1'b1;
                            vh_kicked <= 1'b1;
                            vh_wait_ctr <= 16'd0;
                        end
                        // Drain vh_rd_valid bytes into buffer.  Pointer
                        // wraps mod 512 so multi-block transfers can keep
                        // streaming while DATA_IN drains earlier blocks.
                        if (vh_rd_valid) begin
                            // sec_buf[vh_fill_ptr] <= vh_rd_data — driven
                            // by the sec_* write port (see sec_wr_mux).
`ifdef VERILATOR
                            if (!(sec_we && (sec_wa == vh_fill_ptr) &&
                                  (sec_wd == vh_rd_data)))
                                $display("%0t scsi: FATAL sec_buf write-port mismatch (S_VH_WAIT_RD)",
                                         $time);
`endif
                            vh_fill_ptr  <= (vh_fill_ptr == 9'd511)
                                ? 9'd0
                                : vh_fill_ptr + 9'd1;
                            // Saturate (never wrap): with rd_ready
                            // back-pressure this cannot be reached; the
                            // guard keeps a future pacing bug a data
                            // error instead of a boot wedge.
                            if (vh_buf_count != 10'd1023)
                                vh_buf_count <= vh_buf_count + 10'd1;
`ifdef VERILATOR
                            if (vh_buf_count >= RING_FULL)
                                $display("%0t scsi: WARN multi-block read ring overrun (S_VH_WAIT_RD) count=%0d",
                                         $time, vh_buf_count);
`endif
                        end
                        // Per-cmd-type completion criterion:
                        //   single-block read: wait for vh_done (existing behavior).
                        //   multi-block read: enter DATA_IN as soon as one block is
                        //          buffered; concurrent drain continues
                        //          inside DATA_IN.
                        if (vh_error && vh_done) begin
                            vh_wait_ctr <= 16'd0;
                            `ifdef VERILATOR
                                $display("%0t scsi: READ error sense=%0d asc=%02x",
                                         $time, 4'd3, 8'h11);
                            `endif
                            xfer_status <= 8'h02;  // CHECK CONDITION
                            sense_key   <= 4'd3;   // MEDIUM ERROR
                            sense_asc   <= 8'h11;  // UNRECOVERED READ
                            sense_ascq  <= 8'h00;
                            end_dma_pending <= 1'b1;
                            phase <= S_STATUS;
                            t_req <= 1'b1;
                        end else if (vh_multi_read) begin
                            // multi-block read: enter DATA_IN as soon as ANY byte is
                            // buffered.  the provider cannot be paused mid-
                            // multi-block read (the SPI stream runs to completion),
                            // so waiting for a FULL 512-byte block here
                            // left the ring with zero headroom — every
                            // byte arriving between DATA_IN entry and the
                            // initiator's first drain overwrote unread
                            // data at the ring head (multi-block reads
                            // corrupted their first bytes).  Entering at
                            // the first byte keeps ~512 bytes of slack
                            // for the initiator's select-complete →
                            // first-chunk latency; the DATA_IN ring
                            // back-pressure/re-arm logic already handles
                            // an underrun-paced drain.
                            if (TURBOSCSI_C96_EN
                                ? ((vh_buf_count >= vh_stage_target) ||
                                   vh_supply_faulted ||
                                   ((vh_buf_count != 10'd0) &&
                                    vh_stream_closed))
                                : ((vh_buf_count != 10'd0) || vh_rd_valid))
                            begin
                                // C96: enter DATA_IN only once the ring
                                // is STAGED (see vh_ring_staged) — the
                                // select-complete INT is the pre-stage
                                // barrier, so the ROM's first blind
                                // burst starts with maximal headroom.
                                // Bare-5380 keeps first-byte entry (its
                                // REQ/ACK handshake self-paces).
                                medium_not_present <= 1'b0;
                                xfer_bytes_left <= BLOCK_BYTES;
                                phase <= S_DATA_IN;
                                t_req <= 1'b1;
                                vh_wait_ctr <= 16'd0;
                            end else if (vh_kicked && !vh_busy) begin
                                if (vh_wait_ctr >= (VH_WAIT_TIMEOUT - 16'd1)) begin
                                    `ifdef VERILATOR
                                        $display("%0t scsi: READ timeout waiting for backing store",
                                                 $time);
                                    `endif
                                    busy_error_pending <= 1'b1;
                                    medium_not_present <= 1'b1;
                                    xfer_status <= 8'h02;  // CHECK CONDITION
                                    sense_key   <= 4'd2;   // NOT READY
                                    sense_asc   <= 8'h3A;  // medium not present
                                    sense_ascq  <= 8'h00;
                                    end_dma_pending <= 1'b1;
                                    phase <= S_STATUS;
                                    t_req <= 1'b1;
                                    vh_wait_ctr <= 16'd0;
                                end else begin
                                    vh_wait_ctr <= vh_wait_ctr + 16'd1;
                                end
                            end else if (vh_busy) begin
                                // CONTRACT FIX: the watchdog above is documented as
                                // "Counted ONLY while vh_busy is low: a provider is entitled to
                                //  take as long as it likes as long as it says it is working."
                                // The counter was only ever cleared when DATA ARRIVED, so short
                                // empty-and-not-busy gaps ACCUMULATED across an entire multi-block
                                // run.  VH_WAIT_TIMEOUT is 1024 cycles (~10us @100MHz) of TOTAL
                                // such gap time, which a long transfer (e.g. a 260-block extent)
                                // can reach while the provider is perfectly healthy -- yielding a
                                // spurious CHECK CONDITION / NOT READY on a good read.
                                // Clearing it whenever the provider asserts busy makes the guard
                                // measure a CONTIGUOUS not-busy period, which is what the comment
                                // says it measures.  A backing store that NEVER answers still
                                // accumulates uninterrupted and still trips the timeout, so the
                                // watchdog keeps its purpose.
                                vh_wait_ctr <= 16'd0;
                            end
                        end else if (vh_done) begin
                            // single-block read completion (single block).
                            vh_wait_ctr <= 16'd0;
                            medium_not_present <= 1'b0;
                            xfer_bytes_left <= BLOCK_BYTES;
                            phase <= S_DATA_IN;
                            t_req <= 1'b1;
                        end else if (vh_kicked && !vh_busy) begin
                            if (vh_wait_ctr >= (VH_WAIT_TIMEOUT - 16'd1)) begin
                                `ifdef VERILATOR
                                    $display("%0t scsi: READ timeout waiting for backing store",
                                             $time);
                                `endif
                                busy_error_pending <= 1'b1;
                                medium_not_present <= 1'b1;
                                xfer_status <= 8'h02;  // CHECK CONDITION
                                sense_key   <= 4'd2;   // NOT READY
                                sense_asc   <= 8'h3A;  // medium not present
                                sense_ascq  <= 8'h00;
                                end_dma_pending <= 1'b1;
                                phase <= S_STATUS;
                                t_req <= 1'b1;
                                vh_wait_ctr <= 16'd0;
                            end else begin
                                vh_wait_ctr <= vh_wait_ctr + 16'd1;
                            end
                        end else if (vh_busy) begin
                            // CONTRACT FIX: the watchdog above is documented as
                            // "Counted ONLY while vh_busy is low: a provider is entitled to
                            //  take as long as it likes as long as it says it is working."
                            // The counter was only ever cleared when DATA ARRIVED, so short
                            // empty-and-not-busy gaps ACCUMULATED across an entire multi-block
                            // run.  VH_WAIT_TIMEOUT is 1024 cycles (~10us @100MHz) of TOTAL
                            // such gap time, which a long transfer (e.g. a 260-block extent)
                            // can reach while the provider is perfectly healthy -- yielding a
                            // spurious CHECK CONDITION / NOT READY on a good read.
                            // Clearing it whenever the provider asserts busy makes the guard
                            // measure a CONTIGUOUS not-busy period, which is what the comment
                            // says it measures.  A backing store that NEVER answers still
                            // accumulates uninterrupted and still trips the timeout, so the
                            // watchdog keeps its purpose.
                            vh_wait_ctr <= 16'd0;
                        end
                    end
                    // DATA_IN: REQ/ACK out each byte from sec_buf to the
                    // initiator; on end of block loop into next sector or
                    // step to STATUS.
                    //
                    // For multi-block read multi-block reads the buffer is treated
                    // as a 512-byte ring: vh_rd_valid pulses for the
                    // NEXT block's bytes keep filling sec_buf (with
                    // vh_fill_ptr wrapping mod 512) while the initiator
                    // drains the current block via buf_rd_ptr (also mod
                    // 512).  At end-of-block we stay in S_DATA_IN,
                    // reset xfer_bytes_left and decrement xfer_blocks —
                    // the provider is already streaming the next block.
                    //
                    // For single-block read single-block (and non-data-read CDBs
                    // like INQUIRY/SENSE) the legacy path: end-of-bytes
                    // → S_STATUS, no concurrent fill.
                    //
                    // vh_buf_count is updated with a NET delta across
                    // both fill (vh_rd_valid) and drain (beat) events
                    // so that coincident pulses don't lose either
                    // increment or decrement.
                    S_DATA_IN: begin
                                        // BSY | IO | REQ-if-asserted
                                        // PHASE_MATCH | DRQ-if-asserted
                        cur_data <= sec_rd_a;
                        // Initiator ATN-rise during DATA_IN means it wants
                        // to send a message (typically MSG_OUT 0x04
                        // DISCONNECT).  Per SCSI-2 spec + MAME's MSG_OUT
                        // phase handling, the target snaps the bus into
                        // MSG_OUT and re-arms REQ.  Save the in-flight
                        // transfer state into the disc_* mirrors so a
                        // subsequent CD_RESELECT (ncr53c90.cpp:972-989)
                        // can resume from this exact byte.
                        if (atn_rise && !disc_pending) begin
                            disc_resume_phase    <= S_DATA_IN;
                            disc_xfer_lba        <= xfer_lba;
                            disc_xfer_blocks     <= xfer_blocks;
                            disc_xfer_bytes_left <= xfer_bytes_left;
                            disc_buf_rd_ptr      <= buf_rd_ptr;
                            disc_buf_wr_ptr      <= buf_wr_ptr;
                            disc_vh_fill_ptr     <= vh_fill_ptr;
                            disc_vh_drain_ptr    <= vh_drain_ptr;
                            disc_vh_buf_count    <= vh_buf_count;
                            disc_cdb_op          <= cdb[0];
                            disc_req_write      <= vh_req_write;
                            disc_req_multi      <= vh_req_multi;
                            disc_vh_kicked       <= vh_kicked;
                            phase                <= S_MSG_OUT;
                            t_req                <= 1'b1;
                            msg_out_byte_seen    <= 1'b0;
                        end
                        // Concurrent provider-side fill (multi-block read only): rd_valid
                        // pulses for blocks 2..N arrive while we're
                        // draining block 1 to the initiator.  Pointer
                        // and per-cycle delta to vh_buf_count handled
                        // here; net update happens at the end of the
                        // case branch below.
                        if (vh_rd_valid && (vh_multi_read)) begin
                            // sec_buf[vh_fill_ptr] <= vh_rd_data — see
                            // sec_wr_mux.
`ifdef VERILATOR
                            if (!(sec_we && (sec_wa == vh_fill_ptr) &&
                                  (sec_wd == vh_rd_data)))
                                $display("%0t scsi: FATAL sec_buf write-port mismatch (S_DATA_IN)",
                                         $time);
`endif
                            vh_fill_ptr <= (vh_fill_ptr == 9'd511)
                                ? 9'd0
                                : vh_fill_ptr + 9'd1;
                        end
                        // Non-DMA CI_XFER: push the served byte into the
                        // C96 FIFO (the CPU pops it at offset 2).  Cannot
                        // collide with the register-write-path FIFO
                        // accesses — the beat is gated on !pb_rd && !pb_wr.
                        // R1 rework: a DMA-in ACCEPT stages its byte
                        // through the real FIFO too (MAME recv_byte →
                        // fifo_push), sharing the non-DMA fill push.
                        // Both events are gated on fifo-access-free
                        // cycles and on fifo_pos != 16.
                        if (c96_fifo_fill_beat || c96_cpt_din_beat ||
                            (TURBOSCSI_C96_EN && c96_accept_ev)) begin
                            c96_fifo[c96_fifo_pos[3:0]] <= sec_rd_a;
                            c96_fifo_pos                <= c96_fifo_pos + 5'd1;
                        end
                        // The CI_COMPLETE receive moves exactly one byte
                        // (MAME INIT_CPT_RECV_WAIT_REQ completes right
                        // after it), so retire the pend flag here.
                        if (c96_cpt_din_beat)
                            c96_cpt_din_pend <= 1'b0;
                        // R1 rework: in C96 mode the supply-side
                        // pointers (buf_rd_ptr / xfer_bytes_left /
                        // block loop / phase exit) advance when the
                        // CHIP accepts a byte off the target — the
                        // same instant tcounter decrements — not when
                        // the CPU drains the staged byte.  Bare-5380
                        // keeps the pseudo-DMA-beat trigger.
                        if ((TURBOSCSI_C96_EN ? c96_accept_ev
                                              : pseudo_dma_in_beat) ||
                            c96_fifo_fill_beat || c96_cpt_din_beat) begin
                            if (xfer_bytes_left <= 16'd1) begin
                                if (cdb[0] == 8'h08 || cdb[0] == 8'h28) begin
                                    if (xfer_blocks > 24'd1) begin
                                        xfer_lba    <= xfer_lba + 32'd1;
                                        xfer_blocks <= xfer_blocks - 24'd1;
                                        if (vh_multi_read) begin
                                            buf_rd_ptr <= (buf_rd_ptr == 9'd511)
                                                ? 9'd0
                                                : buf_rd_ptr + 9'd1;
                                            xfer_bytes_left <= BLOCK_BYTES;
                                            // Ring back-pressure: only keep
                                            // REQ/DRQ up if the next block's
                                            // first byte is already buffered
                                            // (this beat consumes one).  The
                                            // re-arm clause below wakes us
                                            // when the provider stream catches up.
                                            t_req <= (vh_buf_count > 10'd1) ||
                                                     vh_rd_valid;
                                        end else begin
                                            // Defensive — single-block read is single
                                            // block; should never multi-loop.
                                            buf_rd_ptr  <= 9'd0;
                                            vh_fill_ptr <= 9'd0;
                                            vh_kicked   <= 1'b0;
                                            phase       <= S_VH_WAIT_RD;
                                        end
                                    end else begin
                                        end_dma_pending <= 1'b1;
                                        phase <= S_STATUS;
                                         t_req <= 1'b1;
                                    end
                                end else begin
                                    end_dma_pending <= 1'b1;
                                    phase <= S_STATUS;
                                     t_req <= 1'b1;
                                end
                            end else begin
                                buf_rd_ptr      <= (vh_multi_read)
                                    ? ((buf_rd_ptr == 9'd511)
                                        ? 9'd0
                                        : buf_rd_ptr + 9'd1)
                                    : (buf_rd_ptr + 9'd1);
                                xfer_bytes_left <= xfer_bytes_left - 16'd1;
                                // multi-block read ring back-pressure (see above);
                                // non-multi-block sources are fully buffered.
                                t_req           <= (!vh_multi_read) ||
                                                   (vh_buf_count > 10'd1) ||
                                                   vh_rd_valid;
                            end
                        end else if (t_req && ack_rise) begin
                            t_req <= 1'b0;
                        end else if (!t_req && ack_fall) begin
                            if (xfer_bytes_left <= 16'd1) begin
                                // End of current sector / data transfer.
                                if (cdb[0] == 8'h08 || cdb[0] == 8'h28) begin
                                    // More blocks for READ ?
                                    if (xfer_blocks > 24'd1) begin
                                        xfer_lba    <= xfer_lba + 32'd1;
                                        xfer_blocks <= xfer_blocks - 24'd1;
                                        if (vh_multi_read) begin
                                            buf_rd_ptr <= (buf_rd_ptr == 9'd511)
                                                ? 9'd0
                                                : buf_rd_ptr + 9'd1;
                                            xfer_bytes_left <= BLOCK_BYTES;
                                            // Ring back-pressure (byte was
                                            // debited at ack_rise).
                                            t_req <= (vh_buf_count != 10'd0) ||
                                                     vh_rd_valid;
                                        end else begin
                                            // single-block read fallback.
                                            buf_rd_ptr  <= 9'd0;
                                            vh_fill_ptr <= 9'd0;
                                            vh_kicked   <= 1'b0;
                                            phase       <= S_VH_WAIT_RD;
                                        end
                                    end else begin
                                        end_dma_pending <= 1'b1;
                                        phase <= S_STATUS;
                                         t_req <= 1'b1;
                                    end
                                end else begin
                                    // INQUIRY / SENSE / CAPACITY / MODE
                                    end_dma_pending <= 1'b1;
                                    phase <= S_STATUS;
                                     t_req <= 1'b1;
                                end
                            end else begin
                                buf_rd_ptr      <= (vh_multi_read)
                                    ? ((buf_rd_ptr == 9'd511)
                                        ? 9'd0
                                        : buf_rd_ptr + 9'd1)
                                    : (buf_rd_ptr + 9'd1);
                                xfer_bytes_left <= xfer_bytes_left - 16'd1;
                                // Ring back-pressure (byte was debited
                                // at ack_rise); non-multi-block fully buffered.
                                t_req           <= (!vh_multi_read) ||
                                                   (vh_buf_count != 10'd0) ||
                                                   vh_rd_valid;
                            end
                        end else if ((vh_multi_read) && !t_req &&
                                     (xfer_bytes_left != 16'd0) &&
                                     ((vh_buf_count != 10'd0) || vh_rd_valid) &&
                                     !init_ack) begin
                            // Multi-block-read ring-underrun re-arm: the provider stream
                            // caught up — raise REQ/DRQ for the next byte.
                            // Gated on !init_ack so it cannot race the
                            // 5380 ack_fall handshake path above.
                            t_req <= 1'b1;
                        end
                        // Multi-block read, mid-stream volume error: the
                        // provider aborted (for the SD path: CRC, token
                        // timeout, R1 reject, ...) while we were
                        // draining.  Fail the transfer like the
                        // S_VH_WAIT_RD error path instead of starving REQ
                        // forever (vh_error is sticky; vh_done pulses).
                        if ((vh_multi_read) && vh_done && vh_error) begin
                            xfer_status <= 8'h02;  // CHECK CONDITION
                            sense_key   <= 4'd3;   // MEDIUM ERROR
                            sense_asc   <= 8'h11;  // UNRECOVERED READ
                            sense_ascq  <= 8'h00;
                            end_dma_pending <= 1'b1;
                            phase <= S_STATUS;
                            t_req <= 1'b1;
                        end
                        // Net vh_buf_count update for this cycle.
                        //   +1 on vh_rd_valid (multi-block read fill).
                        //   -1 on a consumer byte event:
                        //       * c96_accept_ev (C96 DMA path — R1
                        //         rework: the CHIP-side accept consumes
                        //         the ring byte into c96_fifo), or
                        //       * pseudo_dma_in_beat (bare-5380 path), or
                        //       * c96_fifo_fill_beat (non-DMA CI_XFER), or
                        //       * t_req && ack_rise (REQ/ACK path —
                        //         initiator latched cur_data on ACK).
                        // For non-multi-block paths we leave vh_buf_count
                        // untouched (legacy single-block ignores it).
                        if (vh_multi_read) begin
                            case ({vh_rd_valid,
                                   (TURBOSCSI_C96_EN ? c96_accept_ev
                                                     : pseudo_dma_in_beat) ||
                                   c96_fifo_fill_beat || c96_cpt_din_beat ||
                                   (t_req && ack_rise)})
                                // Saturate (never wrap) — see the
                                // matching guard in S_VH_WAIT_RD.
                                2'b10: vh_buf_count <= (vh_buf_count == 10'd1023)
                                                       ? 10'd1023
                                                       : vh_buf_count + 10'd1;
                                2'b01: vh_buf_count <= (vh_buf_count == 10'd0)
                                                       ? 10'd0
                                                       : vh_buf_count - 10'd1;
                                default: ;     // 00 = no change; 11 = +1-1
                            endcase
`ifdef VERILATOR
                            if (vh_rd_valid && (vh_buf_count >= RING_FULL))
                                $display("%0t scsi: WARN multi-block read ring overrun (S_DATA_IN) count=%0d",
                                         $time, vh_buf_count);
`endif
                        end
                    end
                    // DATA_OUT: REQ/ACK in each byte from initiator into
                    // sec_buf; on end of block kick the volume write (or STATUS
                    // for MODE SELECT).
                    //
                    // For multi-block write multi-block writes the buffer is treated
                    // as a 512-byte ring: initiator pushes via
                    // buf_wr_ptr (mod 512) while the provider drains via
                    // vh_drain_ptr (mod 512) concurrently, after the
                    // first vh_req_go fires.  The kick happens at the end
                    // of block 1 — the provider then streams blocks 1..N
                    // back-to-back + emits the 0xFD stop-tran tail
                    // internally.  We stay in S_DATA_OUT for blocks 2..N
                    // and go to S_VH_WAIT_WR only after all bytes have
                    // been pushed (it then just waits for vh_done).
                    S_DATA_OUT: begin
                                        // BSY | REQ-if-asserted (IO=0)
                                        // PHASE_MATCH | DRQ-if-asserted
                        // Initiator ATN-rise during DATA_OUT — see
                        // matching block in S_DATA_IN above.  Save state,
                        // enter MSG_OUT to receive the DISCONNECT byte.
                        if (atn_rise && !disc_pending) begin
                            disc_resume_phase    <= S_DATA_OUT;
                            disc_xfer_lba        <= xfer_lba;
                            disc_xfer_blocks     <= xfer_blocks;
                            disc_xfer_bytes_left <= xfer_bytes_left;
                            disc_buf_rd_ptr      <= buf_rd_ptr;
                            disc_buf_wr_ptr      <= buf_wr_ptr;
                            disc_vh_fill_ptr     <= vh_fill_ptr;
                            disc_vh_drain_ptr    <= vh_drain_ptr;
                            disc_vh_buf_count    <= vh_buf_count;
                            disc_cdb_op          <= cdb[0];
                            disc_req_write      <= vh_req_write;
                            disc_req_multi      <= vh_req_multi;
                            disc_vh_kicked       <= vh_kicked;
                            phase                <= S_MSG_OUT;
                            t_req                <= 1'b1;
                            msg_out_byte_seen    <= 1'b0;
                        end
                        // Concurrent provider-side drain (multi-block write only).
                        // Mirrors the vh_wr_data preload pattern from
                        // S_VH_WAIT_WR — preload runs even before
                        // vh_kicked so the first wr_ready pulse sees
                        // sec_buf[0] on the bus.  vh_drain_ptr only
                        // advances on actual wr_ready events.
                        if (vh_multi_write) begin
                            if (vh_wr_ready && vh_kicked) begin
                                vh_drain_ptr <= vh_drain_next;
                            end
                            // sec_rd_b already selects vh_drain_next on
                            // exactly the (vh_wr_ready && vh_kicked)
                            // condition above — see its declaration.
                            vh_wr_data <= sec_rd_b;
                        end
                        // A data-out byte arrives either through the
                        // pseudo-DMA port (DMA CI_XFER) or off the head
                        // of the chip FIFO (non-DMA CI_XFER — MAME
                        // send_byte(), ncr53c90.cpp:749-763).  The byte
                        // accounting below (buf_wr_ptr / xfer_bytes_left
                        // / block advance) is identical either way; only
                        // the data source differs, and that lives in
                        // sec_wr_mux.  The two beats are mutually
                        // exclusive by c96_fifo_out_bus_conflict.
                        if (pseudo_dma_out_beat || c96_out_send_beat) begin
                            // sec_buf[buf_wr_ptr] <= pb_wdata /
                            // c96_fifo[0] — see sec_wr_mux.
`ifdef VERILATOR
                            if (!(sec_we && (sec_wa == buf_wr_ptr) &&
                                  (sec_wd == (pseudo_dma_out_beat
                                              ? pb_wdata : c96_fifo[0]))))
                                $display("%0t scsi: FATAL sec_buf write-port mismatch (DATA_OUT pdma)",
                                         $time);
`endif
                            if (xfer_bytes_left <= 16'd1) begin
                                if (cdb[0] == 8'h0A || cdb[0] == 8'h2A) begin
                                    if (vh_multi_write) begin
                                        // multi-block write: stay in S_DATA_OUT for
                                        // additional blocks; kick vh_req_go
                                        // on first block boundary.
                                        if (xfer_blocks > 24'd1) begin
                                            xfer_blocks <= xfer_blocks - 24'd1;
                                            buf_wr_ptr  <= (buf_wr_ptr == 9'd511)
                                                ? 9'd0
                                                : buf_wr_ptr + 9'd1;
                                            xfer_bytes_left <= BLOCK_BYTES;
                                            // Back-pressure: only re-arm
                                            // REQ when the ring is below
                                            // full (the next-byte slot is
                                            // free).  Otherwise DROP it;
                                            // the re-arm clause below
                                            // wakes us up once SPI drains.
                                            //
                                            // The `else` is load-bearing.
                                            // This whole arm is guarded by
                                            // (pseudo_dma_out_beat ||
                                            //  c96_out_send_beat), and
                                            // BOTH of those beats already
                                            // require t_req (see their
                                            // declarations) — so inside
                                            // here t_req is 1 by
                                            // construction and a guard
                                            // that only ever SETS it can
                                            // never deassert REQ.  Without
                                            // the else the initiator keeps
                                            // pushing, buf_wr_ptr (mod
                                            // 512) overwrites undrained
                                            // ring bytes, and the card
                                            // gets the wrong payload at
                                            // the right LBA with GOOD
                                            // status.
                                            if (vh_buf_count < RING_ARM_MAX) begin
                                                t_req <= 1'b1;
                                            end else begin
                                                t_req <= 1'b0;
                                            end
                                        end else begin
                                            // All bytes received.
                                            end_dma_pending <= 1'b1;
                                            phase <= S_VH_WAIT_WR;
                                        end
                                    end else begin
                                        // single-block write path.
                                        vh_drain_ptr   <= 9'd0;
                                        vh_kicked      <= 1'b0;
                                        end_dma_pending <= 1'b1;
                                        phase          <= S_VH_WAIT_WR;
                                    end
                                end else begin
                                    end_dma_pending <= 1'b1;
                                    phase <= S_STATUS;
                                     t_req <= 1'b1;
                                end
                            end else begin
                                buf_wr_ptr      <= (vh_multi_write)
                                    ? ((buf_wr_ptr == 9'd511)
                                        ? 9'd0
                                        : buf_wr_ptr + 9'd1)
                                    : (buf_wr_ptr + 9'd1);
                                xfer_bytes_left <= xfer_bytes_left - 16'd1;
                                // Same fix, mid-block.  `!vh_multi_write`
                                // short-circuits to "always re-arm", so
                                // single-block writes and MODE SELECT
                                // parameter lists (which buffer a whole
                                // block before any kick and never drain
                                // concurrently) keep their previous
                                // behaviour bit for bit.  Only the
                                // multi-block ring can take the else.
                                if ((!vh_multi_write) ||
                                    (vh_buf_count < RING_ARM_MAX)) begin
                                    t_req <= 1'b1;
                                end else begin
                                    t_req <= 1'b0;
                                end
                            end
                        end else if (t_req && ack_rise) begin
                            // sec_buf[buf_wr_ptr] <= r_output_data — see
                            // sec_wr_mux.
`ifdef VERILATOR
                            if (!(sec_we && (sec_wa == buf_wr_ptr) &&
                                  (sec_wd == r_output_data)))
                                $display("%0t scsi: FATAL sec_buf write-port mismatch (DATA_OUT reqack)",
                                         $time);
`endif
                            t_req <= 1'b0;
                        end else if (!t_req && ack_fall) begin
                            if (xfer_bytes_left <= 16'd1) begin
                                // Sector/command data drained.
                                if (cdb[0] == 8'h0A || cdb[0] == 8'h2A) begin
                                    if (vh_multi_write) begin
                                        if (xfer_blocks > 24'd1) begin
                                            xfer_blocks <= xfer_blocks - 24'd1;
                                            buf_wr_ptr  <= (buf_wr_ptr == 9'd511)
                                                ? 9'd0
                                                : buf_wr_ptr + 9'd1;
                                            xfer_bytes_left <= BLOCK_BYTES;
                                            // RING_FULL here, NOT
                                            // RING_ARM_MAX — see the note
                                            // on the mid-block twin below.
                                            if (vh_buf_count < RING_FULL) begin
                                                t_req <= 1'b1;
                                            end
                                        end else begin
                                            end_dma_pending <= 1'b1;
                                            phase <= S_VH_WAIT_WR;
                                        end
                                    end else begin
                                        // single-block write — kick the volume write for the
                                        // single block via S_VH_WAIT_WR.
                                        vh_drain_ptr   <= 9'd0;
                                        vh_kicked      <= 1'b0;
                                        end_dma_pending <= 1'b1;
                                        phase          <= S_VH_WAIT_WR;
                                    end
                                end else begin
                                    // MODE SELECT payload dumped.
                                    end_dma_pending <= 1'b1;
                                    phase <= S_STATUS;
                                     t_req <= 1'b1;
                                end
                            end else begin
                                buf_wr_ptr      <= (vh_multi_write)
                                    ? ((buf_wr_ptr == 9'd511)
                                        ? 9'd0
                                        : buf_wr_ptr + 9'd1)
                                    : (buf_wr_ptr + 9'd1);
                                xfer_bytes_left <= xfer_bytes_left - 16'd1;
                                // ── RING_FULL, and that is CORRECT here ──
                                // The pseudo-DMA/non-DMA beat arm above
                                // uses RING_ARM_MAX because it runs ON the
                                // cycle that pushes a byte, so the
                                // vh_buf_count it reads is the PRE-push
                                // value and understates occupancy by one.
                                // This arm is different, and the
                                // difference is load-bearing rather than
                                // an oversight: the bare-5380 byte is
                                // written to sec_buf on ACK_RISE (see
                                // sec_wr_mux's S_DATA_OUT `t_req &&
                                // ack_rise` arm) and vh_buf_count is
                                // incremented on that same ack_rise cycle
                                // (the net-update case below lists
                                // `t_req && ack_rise` as a push event).
                                // ACK_FALL — this arm — pushes NOTHING; it
                                // only advances buf_wr_ptr for the NEXT
                                // byte.  So vh_buf_count is already the
                                // post-push occupancy, arming would take
                                // it to count+1, and count+1 <= RING_FULL
                                // is exactly `count < RING_FULL`.
                                // Using RING_ARM_MAX here would cap the
                                // bare path one byte short for no reason.
                                // No `else` is needed either: `!t_req` is
                                // a precondition of this arm, so "only
                                // set" already means "leave REQ low".
                                if ((!vh_multi_write) ||
                                    (vh_buf_count < RING_FULL)) begin
                                    t_req <= 1'b1;
                                end
                            end
                        end else if ((vh_multi_write) && !t_req &&
                                     (xfer_bytes_left > 16'd0) &&
                                     (vh_buf_count < RING_FULL) &&
                                     !init_ack) begin
                            // multi-block write re-arm: drain made room in the ring,
                            // and ACK is low — assert REQ to fetch the
                            // next byte.  Skip if init_ack is high
                            // because we'd race with the ack_fall path.
                            t_req <= 1'b1;
                        end
                        // Net vh_buf_count update for multi-block write:
                        //   +1 on initiator byte event (push),
                        //   -1 on vh_wr_ready event (drain).
                        // "Initiator byte event" is whichever of the three
                        // data-out sources moved a byte into sec_buf this
                        // cycle — pseudo-DMA shim beat, non-DMA CI_XFER
                        // FIFO send, or the bare-5380 REQ/ACK handshake.
                        // Missing one here silently desynchronises the
                        // ring occupancy from the bytes actually buffered.
                        if (vh_multi_write) begin
                            case ({pseudo_dma_out_beat ||
                                   c96_out_send_beat ||
                                   (t_req && ack_rise),
                                   vh_wr_ready && vh_kicked})
                                2'b10: vh_buf_count <= vh_buf_count + 10'd1;
                                2'b01: begin
                                    // Clamp, do NOT wrap — but RECORD it.
                                    // Draining an empty ring means the
                                    // provider just took whatever stale
                                    // content sat at sec_buf[vh_drain_ptr]
                                    // and put it on the medium.
                                    if (vh_buf_count == 10'd0) begin
                                        vh_wr_underflow <= 1'b1;
                                    end else begin
                                        vh_buf_count <= vh_buf_count - 10'd1;
                                    end
                                end
                                default: ;
                            endcase
                        end
                        // Kick multi-block write vh_req_go once, after the first 512 bytes
                        // have been buffered (vh_buf_count >= 512 OR all
                        // bytes for a small (1-block-but-using-multi-block write)
                        // transfer; latter doesn't happen because the
                        // decode forces single-block write for blocks==1).
                        if ((vh_multi_write) && !vh_kicked && !vh_busy &&
                            !vh_req_go && vh_chk_ok &&
                            (vh_buf_count >= RING_FULL)) begin
                            `ifdef VERILATOR
                                $display("%0t scsi: vhdd multi-write lba=%08x blocks=%0d",
                                         $time, xfer_lba, xfer_blocks);
                            `endif
                            vh_req_lba    <= xfer_lba;
                            vh_req_go     <= 1'b1;
                            vh_kicked <= 1'b1;
                            vh_wait_ctr <= 16'd0;
                        end
                        // ── Multi-block WRITE, mid-stream volume error ──
                        // The exact mirror of the multi-block READ escape
                        // in S_DATA_IN above: the provider aborted (for
                        // the SD path a rejected data-response token, an
                        // R1 reject, a busy timeout, ...) while we were
                        // still streaming bytes into the ring.
                        //
                        // WHY THIS MUST EXIST, and why it did not have to
                        // before producer back-pressure worked.  When the
                        // provider dies it stops pulsing vh_wr_ready, so
                        // vh_buf_count freezes.  With working
                        // back-pressure the initiator has almost certainly
                        // just parked REQ low on a full ring — that IS the
                        // steady state when the initiator outruns SPI,
                        // which the driver does — and the S_DATA_OUT
                        // re-arm clause needs vh_buf_count < RING_FULL to
                        // fire.  Nothing can ever make that true again.
                        // REQ parks low forever, the initiator waits on a
                        // REQ that never comes, and the SCSI bus is wedged
                        // with no diagnosis.  The SCSI sd_ctrl instance
                        // runs with REQ_WDOG_ENABLE(0)
                        // (rtl/soc/fpga_top_sd.vh) so there is no watchdog
                        // backstop either.  Pre-back-pressure this was
                        // survivable only by accident: t_req stayed stuck
                        // high, the initiator drained xfer_bytes_left,
                        // fell into S_VH_WAIT_WR and terminated on
                        // VH_WAIT_TIMEOUT — corrupt, but terminated.
                        //
                        // Leaving the data phase for STATUS mid-transfer
                        // is the driver-safe way out, not a hack: MAME's
                        // initiator explicitly tolerates it —
                        //   ncr53c90.cpp:652-658, INIT_XFR_WAIT_REQ:
                        //     // check for phase change
                        //     if((ctrl & S_PHASE_MASK) != xfr_phase) {
                        //         command_pos = 0;
                        //         state = INIT_XFR_BUS_COMPLETE;
                        //     }
                        // and INIT_XFR_BUS_COMPLETE runs bus_complete()
                        // (:686-692, :792-799) which raises I_BUS.  Our
                        // own armed-CI_XFER phase-change hook does the
                        // same thing, so the driver gets its interrupt.
                        //
                        // SENSE PRIORITY vs vh_wr_underflow: the provider
                        // error wins and reports MEDIUM ERROR (key 3).
                        // The two cannot double-report — this escape jumps
                        // straight to S_STATUS, so S_VH_WAIT_WR's
                        // `vh_error || vh_wr_underflow` arm never runs for
                        // this transfer — and a provider that died is the
                        // upstream CAUSE of any underflow its death then
                        // produced, so key 3 is the more useful answer.
                        // vh_wr_underflow is cleared here for the same
                        // reason: it describes a consequence, not the
                        // fault.
                        //
                        // Ring state is reset so a subsequent command
                        // cannot inherit a full-looking ring.
                        // Placed LAST in this case arm on purpose: these
                        // assignments must win over the beat arm's own
                        // phase/t_req/vh_buf_count writes in the same
                        // cycle.
                        if ((vh_multi_write) && vh_done && vh_error) begin
`ifdef VERILATOR
                            $display("%0t scsi: multi-block WRITE provider error mid-stream lba=%08x blocks=%0d buf_count=%0d — abandoning to STATUS",
                                     $time, xfer_lba, xfer_blocks,
                                     vh_buf_count);
`endif
                            xfer_status     <= 8'h02;  // CHECK CONDITION
                            sense_key       <= 4'd3;   // MEDIUM ERROR
                            sense_asc       <= 8'h0C;  // WRITE ERROR
                            sense_ascq      <= 8'h00;
                            vh_buf_count    <= 10'd0;
                            vh_drain_ptr    <= 9'd0;
                            buf_wr_ptr      <= 9'd0;
                            vh_kicked       <= 1'b0;
                            vh_wr_underflow <= 1'b0;
                            end_dma_pending <= 1'b1;
                            phase           <= S_STATUS;
                            t_req           <= 1'b1;
                        end
                    end
                    // VH_WAIT_WR: kick the volume write; on wr_ready advance
                    // drain_ptr and pre-load next byte; on vh_done → STATUS.
                    S_VH_WAIT_WR: begin
                          // BSY only
                        if (!vh_chk_ok) begin
                            `ifdef VERILATOR
                                $display("%0t scsi: WRITE out of range lba=%08x blocks=%0d",
                                         $time, xfer_lba, xfer_blocks);
                            `endif
                            xfer_status <= 8'h02;  // CHECK CONDITION
                            sense_key   <= 4'd5;   // ILLEGAL REQUEST
                            sense_asc   <= 8'h21;  // LBA out of range
                            sense_ascq  <= 8'h00;
                            end_dma_pending <= 1'b1;
                            phase <= S_STATUS;
                            t_req <= 1'b1;
                        end else if (!vh_kicked && !vh_busy && !vh_req_go) begin
                            `ifdef VERILATOR
                                $display("%0t scsi: vhdd write lba=%08x blocks=%0d",
                                         $time, xfer_lba, xfer_blocks);
                            `endif
                            vh_req_lba    <= xfer_lba;
                            vh_req_go     <= 1'b1;
                            vh_kicked <= 1'b1;
                            vh_wait_ctr <= 16'd0;
                        end
                        // Drive next byte when the provider pulses wr_ready.
                        // When wr_ready fires, advance drain_ptr AND pre-
                        // load the next byte so the following cycle has
                        // valid data ready.
                        if (vh_wr_ready) begin
                            vh_drain_ptr <= vh_drain_ptr + 9'd1;
                        end
                        // sec_rd_b selects vh_drain_next whenever
                        // vh_wr_ready is high in this state.
                        vh_wr_data <= sec_rd_b;
                        if (vh_done) begin
                            vh_wait_ctr <= 16'd0;
                            // vh_wr_underflow is escalated exactly like a
                            // provider error, and for the same reason: the
                            // block that reached the medium is NOT the
                            // block the initiator sent.  Distinguished by
                            // sense key — 3 MEDIUM ERROR for a provider
                            // failure, 4 HARDWARE ERROR for our own ring
                            // running dry — so a sense dump names which.
                            // Silently returning GOOD here is the actual
                            // sin: it converts a detectable, retryable I/O
                            // error into permanent on-disk corruption.
                            if (vh_error || vh_wr_underflow) begin
                                `ifdef VERILATOR
                                    $display("%0t scsi: WRITE error sense=%0d asc=%02x underflow=%0b",
                                             $time, vh_wr_underflow ? 4'd4 : 4'd3,
                                             8'h0C, vh_wr_underflow);
                                `endif
                                xfer_status <= 8'h02;
                                sense_key   <= vh_wr_underflow ? 4'd4 : 4'd3;
                                sense_asc   <= 8'h0C;    // WRITE ERROR
                                sense_ascq  <= 8'h00;
                                end_dma_pending <= 1'b1;
                                phase <= S_STATUS;
                                t_req <= 1'b1;
                            end else begin
                                medium_not_present <= 1'b0;
                                if (xfer_blocks > 24'd1) begin
                                    xfer_lba    <= xfer_lba + 32'd1;
                                    xfer_blocks <= xfer_blocks - 24'd1;
                                    buf_wr_ptr  <= 9'd0;
                                    vh_drain_ptr <= 9'd0;
                                    vh_kicked    <= 1'b0;
                                    xfer_bytes_left <= BLOCK_BYTES;
                                    phase <= S_DATA_OUT;
                                     t_req <= 1'b1;
                                end else begin
                                    end_dma_pending <= 1'b1;
                                    phase <= S_STATUS;
                                     t_req <= 1'b1;
                                end
                            end
                        end else if (vh_kicked && !vh_busy) begin
                            if (vh_wait_ctr >= (VH_WAIT_TIMEOUT - 16'd1)) begin
                                `ifdef VERILATOR
                                    $display("%0t scsi: WRITE timeout waiting for backing store",
                                             $time);
                                `endif
                                busy_error_pending <= 1'b1;
                                medium_not_present <= 1'b1;
                                xfer_status <= 8'h02;  // CHECK CONDITION
                                sense_key   <= 4'd2;   // NOT READY
                                sense_asc   <= 8'h3A;  // medium not present
                                sense_ascq  <= 8'h00;
                                end_dma_pending <= 1'b1;
                                phase <= S_STATUS;
                                t_req <= 1'b1;
                                vh_wait_ctr <= 16'd0;
                            end else begin
                                vh_wait_ctr <= vh_wait_ctr + 16'd1;
                            end
                        end else if (vh_busy) begin
                            // CONTRACT FIX: the watchdog above is documented as
                            // "Counted ONLY while vh_busy is low: a provider is entitled to
                            //  take as long as it likes as long as it says it is working."
                            // The counter was only ever cleared when DATA ARRIVED, so short
                            // empty-and-not-busy gaps ACCUMULATED across an entire multi-block
                            // run.  VH_WAIT_TIMEOUT is 1024 cycles (~10us @100MHz) of TOTAL
                            // such gap time, which a long transfer (e.g. a 260-block extent)
                            // can reach while the provider is perfectly healthy -- yielding a
                            // spurious CHECK CONDITION / NOT READY on a good read.
                            // Clearing it whenever the provider asserts busy makes the guard
                            // measure a CONTIGUOUS not-busy period, which is what the comment
                            // says it measures.  A backing store that NEVER answers still
                            // accumulates uninterrupted and still trips the timeout, so the
                            // watchdog keeps its purpose.
                            vh_wait_ctr <= 16'd0;
                        end
                    end
                    // STATUS: REQ/ACK one status byte; assert IRQ.
                    // C96 path: when c96_xfer_active is set, the ROM never
                    // drives ack_rise/ack_fall — it advances the exchange
                    // with 53C96 commands instead.  Hold the STATUS phase
                    // (status[2:0]=011, REQ up) until the initiator issues
                    // CI_COMPLETE / CI_MSG_ACCEPT (register-write path);
                    // the c96_xfr_armed hook below already raised I_BUS
                    // when we arrived here from a data phase (MAME
                    // bus_complete() at the end of INIT_XFR).
                    S_STATUS: begin
                        end_dma_pending <= 1'b1;
                        cur_data <= xfer_status;
                        irq_pending <= 1'b1;
                        if (TURBOSCSI_C96_EN && c96_xfer_active) begin
                            // Hold — see comment above.
                        end else if (t_req && ack_rise) begin
                            t_req <= 1'b0;
                        end else if (!t_req && ack_fall) begin
                            // Next: MSG_IN
                            phase <= S_MSG_IN;
                            t_req <= 1'b1;
                        end
                    end
                    // MSG_IN: REQ/ACK one byte (COMMAND COMPLETE=0x00),
                    // keep IRQ asserted, then disconnect.
                    S_MSG_IN: begin
                        cur_data <= xfer_msg;
                        irq_pending <= 1'b1;
                        if (TURBOSCSI_C96_EN && c96_xfer_active) begin
                            // C96-held: parked here by CI_COMPLETE (ACK
                            // asserted on the message byte); released to
                            // S_DISCONNECT by CI_MSG_ACCEPT.
                        end else if (t_req && ack_rise) begin
                            t_req <= 1'b0;
                        end else if (!t_req && ack_fall) begin
                            phase <= S_DISCONNECT;
                        end
                    end
                    // DISCONNECT: drop BSY → BUS_FREE; IRQ held until
                    // initiator reads reg 7.  When disc_pending=1 we
                    // arrived here via the MSG_OUT 0x04 (DISCONNECT)
                    // path: the I_DISCONNECT (0x20) interrupt has
                    // already been latched in the C96 path; in the
                    // bare-5380 path irq_pending stays asserted so the
                    // initiator's IRQ poll sees it, and a later
                    // CD_RESELECT (or initiator-side re-selection
                    // arbitration) re-enters via S_RESELECT.
                    S_DISCONNECT: begin
                        t_req <= 1'b0;
                        phase <= S_BUS_FREE;
                    end
                    // ── MSG_OUT: receive 1 message byte from initiator ──
                    // Per SCSI-2: initiator asserted ATN to interrupt the
                    // data phase; target sent REQ in MSG_OUT phase to
                    // drain the byte.  We support DISCONNECT (0x04) and
                    // (silently consume + ignore) any other message —
                    // notably IDENTIFY (0x80+), which the Mac sends
                    // immediately after Selection on some flows.
                    //
                    // MAME ncr53c90.cpp does not parse the byte itself —
                    // the FIFO does — but the high-level effect is the
                    // same: a 0x04 byte triggers I_DISCONNECT (0x20)
                    // and a bus-free transition.
                    S_MSG_OUT: begin
                        if (t_req && ack_rise) begin
                            // Latch the byte from initiator data bus.
                            cur_data <= r_output_data;
                            msg_out_byte_seen <= 1'b1;
                            t_req <= 1'b0;
                        end else if (!t_req && ack_fall && msg_out_byte_seen) begin
                            if (cur_data == 8'h04) begin
                                // DISCONNECT: send 0x04 back via MSG_IN
                                // and drop BSY.  irq + I_DISCONNECT bit
                                // set when entering S_DISCONNECT below.
                                disc_pending <= 1'b1;
                                xfer_msg     <= 8'h04;
                                phase        <= S_MSG_IN;
                                t_req        <= 1'b1;
                                irq_pending  <= 1'b1;
                                end_dma_pending <= 1'b1;
                                if (TURBOSCSI_C96_EN) begin
                                    // I_DISCONNECT per ncr53c90.h:153.
                                    c96_istatus     <= I_DISCONNECT;
                                    c96_irq_pending <= 1'b1;
                                end
                            end else begin
                                // Non-DISCONNECT messages: consume and
                                // resume the data phase we suspended
                                // (or COMMAND if entered from there in
                                // the future).  msg_out_byte_seen is
                                // cleared on resume so a subsequent
                                // ATN-rise in the resumed phase can fire
                                // again.
                                msg_out_byte_seen <= 1'b0;
                                if (disc_resume_phase == S_DATA_IN ||
                                    disc_resume_phase == S_DATA_OUT) begin
                                    phase <= disc_resume_phase;
                                end else begin
                                    phase <= S_COMMAND;
                                end
                                // Do NOT re-arm REQ onto a full ring.
                                // This is the ATN detour — the initiator
                                // interrupted the data phase with a
                                // non-DISCONNECT message and we go
                                // straight back — so NOTHING was torn
                                // down and the LIVE vh_buf_count /
                                // vh_multi_write are the current truth
                                // (unlike the reselect resume below,
                                // which restores from disc_*).
                                //
                                // Re-arming unconditionally used to be
                                // harmless because there was no producer
                                // back-pressure at all.  Now that REQ is
                                // correctly parked on a full ring,
                                // RING_FULL is the ordinary steady state
                                // whenever the initiator outruns SPI —
                                // which the driver does — so an ATN
                                // detour is MOST likely to land exactly
                                // there, and arming would invite a push
                                // that overwrites an undrained byte.
                                //
                                // RING_FULL (not RING_ARM_MAX) is the
                                // right threshold: no byte is pushed on
                                // this cycle, so vh_buf_count is the true
                                // occupancy and arming permits exactly
                                // one push, to at most RING_FULL.  The
                                // beat that then fires re-evaluates on
                                // its own pre-push count via
                                // RING_ARM_MAX.  Same reasoning as the
                                // S_DATA_OUT re-arm clause, which is also
                                // what wakes us if we leave REQ low here.
                                //
                                // HONEST STATUS: defence in depth, and
                                // UNPROVEN BY TEST.  tb-scsi-sd-e2e's s11
                                // drives this path with the ring measured
                                // at exactly RING_FULL and still passes
                                // with this gate reverted, because the
                                // ATN message handshake itself costs about
                                // one SPI byte-time, so the ring drains
                                // 512 -> 511 during the detour and the
                                // unconditional arm happened to be safe by
                                // one byte.  Kept anyway: a one-byte margin
                                // that depends on provider timing is not a
                                // guarantee, and a provider sitting in a
                                // busy phase across the detour drains
                                // nothing at all.
                                if ((disc_resume_phase == S_DATA_OUT) &&
                                    vh_multi_write &&
                                    !(vh_buf_count < RING_FULL)) begin
                                    t_req <= 1'b0;
                                end else begin
                                    t_req <= 1'b1;
                                end
                            end
                        end
                    end
                    // ── RESELECT: target re-arbitrates and reconnects ──
                    // Per MAME ncr53c90.cpp:972-989 (CD_RESELECT) and
                    // arbitrate() at lines 1071-1077: the chip drives
                    // BSY + (1<<id) on the data bus, waits for the
                    // initiator to acknowledge (its own BSY drop), then
                    // sends an IDENTIFY message via MSG_IN and resumes
                    // the original data phase.  We model the three
                    // sub-states in sequence; each transition is
                    // unconditional in our simplified bus.
                    S_RESELECT: begin
                        // Drive ID on data bus and assert BSY.  The
                        // C96 register-side already raised seq=0 in the
                        // command-decode path.  Move on next cycle.
                        // Drive the ID of the device we are CURRENTLY
                        // acting as, not a fixed TARGET_ID — the
                        // suspended transfer we are reconnecting to
                        // belongs to whichever target was selected.
                        cur_data <= 8'h01 << sel_tgt_id;
                        phase    <= S_RESELECT_ID;
                        t_req    <= 1'b0;
                    end
                    S_RESELECT_ID: begin
                        // Wait one cycle then drop into MSG_IN with
                        // IDENTIFY (0x80 = LUN 0).  This mirrors MAME's
                        // ARB_COMPLETE → DISC_REC_ARBITRATION sequence
                        // (ncr53c90.cpp:1071-1077 + reselect handler).
                        cur_data       <= 8'h80;            // IDENTIFY, LUN 0
                        ident_msg_sent <= 1'b0;
                        phase          <= S_RESELECT_MSG;
                        t_req          <= 1'b1;
                    end
                    S_RESELECT_MSG: begin
                        cur_data <= 8'h80;
                        if (t_req && ack_rise) begin
                            t_req          <= 1'b0;
                            ident_msg_sent <= 1'b1;
                        end else if (!t_req && ack_fall && ident_msg_sent) begin
                            // Restore suspended state and resume the
                            // data phase the initiator was draining.
                            xfer_lba        <= disc_xfer_lba;
                            xfer_blocks     <= disc_xfer_blocks;
                            xfer_bytes_left <= disc_xfer_bytes_left;
                            buf_rd_ptr      <= disc_buf_rd_ptr;
                            buf_wr_ptr      <= disc_buf_wr_ptr;
                            vh_fill_ptr     <= disc_vh_fill_ptr;
                            vh_drain_ptr    <= disc_vh_drain_ptr;
                            vh_buf_count    <= disc_vh_buf_count;
                            vh_req_write    <= disc_req_write;
                            vh_req_multi    <= disc_req_multi;
                            data_from_ring  <= disc_req_multi;
                            vh_kicked       <= disc_vh_kicked;
                            cdb[0]          <= disc_cdb_op;
                            disc_pending    <= 1'b0;
                            phase           <= disc_resume_phase;
                            // Do NOT invite a push onto a full ring.
                            // Resume used to arm REQ unconditionally.
                            // That was harmless while there was no
                            // producer back-pressure at all (the ring was
                            // already being blown past full continuously),
                            // but parking AT RING_FULL is the normal
                            // steady state once back-pressure works and
                            // the initiator outruns SPI — so a disconnect
                            // is now MOST likely to happen exactly when
                            // the ring is full, and re-arming would
                            // overwrite an undrained byte.  Same threshold
                            // as the beat arm (RING_ARM_MAX): the byte
                            // this arms would be pushed by a beat that
                            // then re-evaluates on the pre-push count.
                            // Multi-block WRITE resume only — S_DATA_IN
                            // resume has no such constraint and keeps
                            // arming unconditionally.
                            //
                            // REACHABILITY, so nobody hunts for a test:
                            // this arm is currently DEAD in both shipping
                            // configurations, and the fix is defence in
                            // depth for a future one.  Getting here needs
                            // BOTH disc_pending (set only by an ATN-driven
                            // MSG_OUT 0x04, and r_init_cmd[1]/init_atn is
                            // writable only via the bare-5380 register arm
                            // at scsi.v:2170-2174, which the C96 decode
                            // arms above shadow entirely when
                            // TURBOSCSI_C96_EN=1) AND a CD_RESELECT
                            // command (decoded only inside the C96
                            // register block, scsi.v:1980).  C96=1 can
                            // reselect but can never raise ATN; C96=0 can
                            // raise ATN but has no CD_RESELECT.  The
                            // REACHABLE resume is the S_MSG_OUT ATN-detour
                            // arm below, which is fixed and tested.
                            // RING_FULL for the same reason as there: no
                            // push on this cycle.
                            //
                            // Leaving t_req low here is safe, not a second
                            // wedge: the S_DATA_OUT re-arm clause is
                            // reachable from the resumed state — phase
                            // becomes S_DATA_OUT, vh_multi_write is
                            // rebuilt from the restored data_from_ring /
                            // vh_req_write assigned just above,
                            // xfer_bytes_left is restored non-zero,
                            // init_ack is low (this arm fires on ack_fall),
                            // and the provider's CMD25 is still live
                            // across the disconnect with vh_kicked
                            // restored, so vh_wr_ready keeps pulsing and
                            // drains the ring below the threshold.
                            if ((disc_resume_phase == S_DATA_OUT) &&
                                disc_req_write && disc_req_multi &&
                                !(disc_vh_buf_count < RING_FULL)) begin
                                t_req <= 1'b0;
                            end else begin
                                t_req <= 1'b1;
                            end
                            ident_msg_sent  <= 1'b0;
                        end
                    end
                    default: phase <= S_BUS_FREE;
                endcase
                // ── Supply-exhaustion phase advance (structural backstop) ──
                // A SCSI target does not sit in DATA IN with nothing left
                // to send: once the CDB's byte budget is spent it releases
                // the data phase and drives STATUS.  Every normal exit
                // above is taken on the LAST DRAIN BEAT (xfer_bytes_left
                // <= 1), so it depends on the initiator-side beat
                // accounting still being live.  If that accounting ever
                // leaks — a dropped accept, a lost drain beat, a ring
                // byte the provider never delivered — the phase exit is
                // lost with it and the chip is left armed in DATA IN with
                // no completion path at all: accepts need avail>pend,
                // chunk-I_BUS needs tcounter==0, phase-exit needs a drain
                // beat, drain beats need accept_pend!=0.  That is the
                // interrupt-free wedge measured on HW (bitstream
                // 0xE80161D3).
                //
                // These are genuine phase advances keyed on the target's
                // own remaining supply, NOT timeouts — neither term can
                // be true while the target still has a byte to give:
                //   (a) xfer_bytes_left == 0: the data phase's byte
                //       budget is spent.  The normal paths step it
                //       1 -> STATUS, so 0-in-DATA_IN is by construction
                //       unreachable in a healthy transfer.
                //   (b) multi-block read whose provider has CLOSED the
                //       stream (kicked, no longer busy, no byte landing
                //       this cycle) with an EMPTY ring and bytes still
                //       owed.  Nothing can ever arrive again, so the
                //       phase can only end here.  Reachable: a backing
                //       store that ends a read short without flagging an
                //       error puts the transfer here (tb scn_short_supply).
                //       `!t_req && !init_ack` excludes the bare-5380
                //       REQ/ACK window: that path debits vh_buf_count at
                //       ack_rise but xfer_bytes_left only at ack_fall, so
                //       the ring legitimately reads 0 for the cycles
                //       between them with the last byte still in flight.
                //       Without this the final byte of a multi-block
                //       READ(10) is lost (tb-scsi
                //       test_read10_two_blocks_lba_sequence).  `!ack_fall`
                //       additionally leaves the FSM's own last-byte exit
                //       to the FSM, so a fired backstop always means a
                //       real short read and never shadows a normal one.
                // Either way the transaction completes as a SHORT READ —
                // the armed CI_XFER takes the phase-change I_BUS hook
                // below and the initiator reads STATUS/MSG normally —
                // instead of hanging forever with no interrupt.
                // (`atn_rise && !disc_pending` is excluded so an initiator
                // asking for MSG OUT in the same cycle still wins — the
                // case arm's own phase assignment must not be clobbered.)
                if ((phase == S_DATA_IN) &&
                    ((xfer_bytes_left == 16'd0) ||
                     (vh_multi_read && vh_kicked && !vh_busy &&
                      !vh_rd_valid && (vh_buf_count == 10'd0) &&
                      !t_req && !init_ack && !ack_fall)) &&
                    !(atn_rise && !disc_pending)) begin
`ifdef VERILATOR
                    $display("%0t scsi: SUPPLY EXHAUSTED in DATA_IN — advancing to STATUS (short read) bytes_left=%0d blocks=%0d tcounter=%0d accept_pend=%0d xfr_left=%0d buf_count=%0d",
                             $time, xfer_bytes_left, xfer_blocks,
                             c96_tcounter, c96_accept_pend,
                             c96_xfr_left, vh_buf_count);
`endif
                    end_dma_pending <= 1'b1;
                    phase           <= S_STATUS;
                    t_req           <= 1'b1;
                end
                // ── STUCK-SUPPLY watchdog (busy-INDEPENDENT) ──────────
                // Complements the backstop above, which cannot fire while
                // vh_busy is high (mid-stream stall) or t_req is high
                // (C96 pseudo-DMA holds it up for the whole block).  See
                // VH_STUCK_TIMEOUT's block comment.  Counts contiguous
                // cycles of "multi-block disk read, ring empty, no byte
                // arriving" — reset the instant a byte lands or we leave
                // the read-delivery phases — and on expiry completes the
                // command as CHECK CONDITION / MEDIUM ERROR so the
                // initiator's interrupt fires and the phase leaves
                // S_DATA_IN (dropping c96_shim_rd_starved, releasing the
                // withheld pseudo-DMA beat).  Only a genuinely stalled
                // supply reaches the bound; a slow-but-clocking card's
                // inter-byte gaps are far shorter and reset the counter.
                // 2026-09-07: widened from `vh_buf_count == 0` to
                // `< vh_stage_target` so a provider that dies with
                // RESIDUE in the ring (leaving a staged-gated completion
                // waiting for a refill that never comes) is also
                // bounded.  A healthy stream resets the counter every
                // vh_rd_valid; a ring at/above its stage target does not
                // count at all (that includes the closed-stream tail,
                // where target collapses to the remaining payload).
                if (vh_multi_read &&
                    ((phase == S_DATA_IN) || (phase == S_VH_WAIT_RD)) &&
                    (vh_buf_count < vh_stage_target) && !vh_rd_valid) begin
                    if (vh_stuck_ctr >= (VH_STUCK_TIMEOUT - 24'd1)) begin
`ifdef VERILATOR
                        $display("%0t scsi: STUCK SUPPLY watchdog fired — provider delivered nothing for %0d cycles (vh_busy=%0d phase=%0d buf_count=%0d) — completing as CHECK CONDITION",
                                 $time, vh_stuck_ctr, vh_busy, phase,
                                 vh_buf_count);
`endif
                        xfer_status <= 8'h02;  // CHECK CONDITION
                        sense_key   <= 4'd4;   // HARDWARE ERROR
                        sense_asc   <= 8'h44;  // internal target failure
                        sense_ascq  <= 8'h00;
                        end_dma_pending <= 1'b1;
                        phase           <= S_STATUS;
                        t_req           <= 1'b1;
                        vh_kicked       <= 1'b0;
                        vh_stuck_ctr    <= 24'd0;
                        // Force vh_ring_staged from here on so the gated
                        // chunk completion (if armed) fires I_BUS instead
                        // of parking behind a refill that will never come.
                        vh_supply_faulted <= 1'b1;
                    end else begin
                        vh_stuck_ctr <= vh_stuck_ctr + 24'd1;
                    end
                end else begin
                    vh_stuck_ctr <= 24'd0;
                end
            end
            // ── C96 transfer counter ───────────────────────────────────
            // Data-out: decrement per pseudo-DMA write beat (MAME dma_w
            // → decrement_tcounter).  Cannot conflict with the reg-3
            // write path — a command write and a shim beat are different
            // pb addresses.
            if (TURBOSCSI_C96_EN && pseudo_dma_out_beat &&
                (c96_tcounter != 17'd0)) begin
                c96_tcounter <= c96_tcounter - 17'd1;
                if (c96_tcounter == 17'd1)
                    c96_status_sticky <= c96_status_sticky | S_TC0;
            end
            // Data-in: per MAME (RECV_WAIT_SETTLE — "tcount is
            // decremented on ACKO, not DACK") the counter decrements as
            // the CHIP accepts bytes from the target, NOT when the CPU
            // drains the DMA port.  The Q700 ROM polls status for TC0
            // BEFORE draining each 16-byte chunk (ROM 0x408992d2);
            // decrementing on the CPU's reads deadlocks the boot scan in
            // S_DATA_IN — the exact HW wedge this replaces.  Acceptance
            // runs at 1 byte/cycle while buffered data is available;
            // c96_accept_ev carries the reg-3-write / istatus-read
            // exclusions.
            //
            // ASYNC ONLY.  MAME's recv path decrements here under
            // `(mode == MODE_I) && (sync_offset == 0) && (phase ==
            // DATA_IN)` (ncr53c90.cpp:472-477) — we are always the
            // initiator and c96_accept_ev already pins the phase, so the
            // sync-offset test is the only missing term.  Its partner is
            // the dma_r() decrement above; the two are mutually
            // exclusive in MAME and must stay so here, or a sync
            // transfer would count each byte down twice.
            if (c96_accept_ev && (c96_sync_offset == 4'd0)) begin
                c96_tcounter <= c96_tcounter - 17'd1;
                if (c96_tcounter == 17'd1)
                    c96_status_sticky <= c96_status_sticky | S_TC0;
            end
            // Non-DMA data-out send: pop the byte this beat just handed
            // the target off the head of the FIFO (MAME send_byte() →
            // fifo_pop(), ncr53c90.cpp:749-763 / :837-845).  Cannot
            // collide with the reg-2 read pop or any register write —
            // c96_fifo_out_bus_conflict excludes both — nor with the
            // data-in fill push, which lives in S_DATA_IN.  tcounter is
            // NOT touched: decrement_tcounter() (:1234-1237) returns
            // early for a non-DMA command.
            if (c96_out_send_beat) begin
                // MAME send_byte() → fifo_pop() memmove semantics
                for (k = 0; k < 15; k = k + 1) begin
                    if (k + 1 < {27'd0, c96_fifo_pos})
                        c96_fifo[k] <= c96_fifo[k + 1];
                end
                c96_fifo_pos <= c96_fifo_pos - 5'd1;
            end
            // Non-DMA STATUS / MSG IN receive: push the one byte the
            // target presents (MAME recv_byte() -> fifo_push(),
            // ncr53c90.cpp:846-852).  Mutually exclusive with the send
            // pop above (different phases) and with the reg-2 accesses
            // (c96_fifo_bus_conflict).
            if (c96_nondma_in_status || c96_nondma_in_msgin) begin
                c96_fifo[c96_fifo_pos[3:0]] <= c96_nondma_in_byte;
                c96_fifo_pos                <= c96_fifo_pos + 5'd1;
            end
            // Accepted-but-undrained tracker (+1 accept, -1 CPU beat)
            // and per-chunk CPU-beat countdown.
            // 2026-08-19 R1 REWORK: DEBUG-ONLY from here on — the
            // functional data path stages accepts through c96_fifo and
            // completes on TC0 + the live DRQ formula; these counters
            // only feed the conservation/DUMP instrumentation
            // (tb_scsi_c96_sm43_chunk peeks them, $displays below).
            if (TURBOSCSI_C96_EN) begin
                case ({c96_accept_ev, pseudo_dma_in_beat})
                    2'b10: c96_accept_pend <= c96_accept_pend + 10'd1;
                    2'b01: c96_accept_pend <= (c96_accept_pend == 10'd0)
                                              ? 10'd0
                                              : c96_accept_pend - 10'd1;
                    default: ;     // 00 = no change; 11 = +1-1
                endcase
                if (pseudo_dma_in_beat && (c96_xfr_left != 17'd0))
                    c96_xfr_left <= c96_xfr_left - 17'd1;
            end
            // ── Latched check_drq() (finding F11) ─────────────────────
            // Recompute whenever a FIFO- or TC0-touching operation ran
            // (the previous cycle changed fifo_pos/TC0) or a pseudo-DMA
            // access happened — the sites where MAME calls check_drq().
            // A bare dma_set() (command dispatch) does NOT recompute,
            // which is exactly why a timed-out DMA select keeps DRQ
            // low.  Chip reset forces it low (device_reset).
            //
            // 2026-08-19 (scsi_fuzz seeds 17/40/62): CM_FLUSH_FIFO and
            // device_reset are the two fifo_pos mutations MAME performs
            // WITHOUT a check_drq (`fifo_pos = 0` direct assignment,
            // ncr53c90.cpp:956 / :248) — the previous edge-event here
            // recomputed off them anyway, resurrecting a stale
            // OUT-formula DRQ right after the flush every normalization
            // sequence performs.  c96_drq_norecompute_q masks the
            // pos-change edge those two commands cause.
            if (TURBOSCSI_C96_EN) begin
                c96_fifo_pos_q <= c96_fifo_pos;
                c96_tc0_q      <= (c96_status_sticky & S_TC0) != 8'h00;
                c96_drq_norecompute_q <=
                    c96_cmd_dispatch &&
                    (((c96_cmd_cur & 8'h7f) == CM_RESET) ||
                     ((c96_cmd_cur & 8'h7f) == CM_FLUSH_FIFO));
                if (c96_cmd_dispatch && ((c96_cmd_cur & 8'h7f) == CM_RESET))
                    c96_drq_stale <= 1'b0;
                else if (((c96_fifo_pos != c96_fifo_pos_q) &&
                          !c96_drq_norecompute_q) ||
                         (((c96_status_sticky & S_TC0) != 8'h00)
                          != c96_tc0_q) ||
                         (pb_dma_shim && (pb_rd || pb_wr))) begin
                    case (c96_dma_dir)
                        C96_DIR_IN:
                            // BUSMD_1 IN formula (see c96_drq_fifo_in_live
                            // — drq_c96 consults it live while dir==IN;
                            // this latch only matters as the carried-over
                            // value after a dma_set() switches direction
                            // without a recompute, so keep it in sync)
                            c96_drq_stale <= c96_drq_fifo_in_live;
                        C96_DIR_OUT:
                            c96_drq_stale <=
                                !c96_xfr_armed && !c96_sel_active &&
                                !c96_sel_stopped && !c96_cmd_wait &&
                                ((c96_status_sticky & S_TC0) == 8'h00) &&
                                (c96_fifo_pos < 5'd15);
                        default:
                            c96_drq_stale <= 1'b0;
                    endcase
                end
                // ── MAME completion-path check_drq() ──────────────────
                // function_complete() / function_bus_complete() /
                // bus_complete() all run dma_set(DMA_NONE) followed by
                // check_drq() (ncr53c90.cpp:772-799) — the latched DRQ
                // is forced LOW the moment any command completes.  The
                // dispatch arms that complete instantly in this model
                // (CI_COMPLETE, CI_MSG_ACCEPT, CD_DISABLE_SEL) sit
                // ABOVE the recompute above, so their clear is applied
                // here where it wins the ordering (scsi_fuzz seed 4: a
                // completed DMA-form select left the stale OUT-formula
                // DRQ latched high forever — the MacBench "SCSI
                // Information" hang's DRQ half).  The event hooks below
                // this block carry their own inline clears.
                if (c96_cmd_dispatch && c96_cmd_valid &&
                    (((c96_cmd_cur & 8'h7f) == CI_COMPLETE)   ||
                     ((c96_cmd_cur & 8'h7f) == CI_MSG_ACCEPT) ||
                     ((c96_cmd_cur & 8'h7f) == CD_DISABLE_SEL)))
                    c96_drq_stale <= 1'b0;
            end
            // ── CI_COMPLETE DATA-IN receive: give up if the bus left
            //    DATA IN before the target handed a byte over.  MAME's
            //    recv_byte() is instantaneous in emulated time, so there
            //    is no MAME-side equivalent to strand; this only keeps a
            //    chip/bus reset or a disconnect from carrying the pend
            //    flag into the next connection.
            if (TURBOSCSI_C96_EN && c96_cpt_din_pend &&
                (phase != S_DATA_IN))
                c96_cpt_din_pend <= 1'b0;
            // ── Deferred DMA CI_XFER receive: fire once the DMA port
            //    has quiesced (see the c96_xfr_recv_* declaration) ──────
            if (TURBOSCSI_C96_EN && c96_xfr_recv_pend) begin
                if (pb_dma_shim && (pb_rd || pb_wr)) begin
                    c96_xfr_recv_timer <= C96_RECV_QUIESCE;
                end else if (c96_xfr_recv_timer != 10'd0) begin
                    c96_xfr_recv_timer <= c96_xfr_recv_timer - 10'd1;
                end else if (!c96_fifo_bus_conflict) begin
                    // The byte is ACKed off the bus either way; MAME's
                    // fifo_push() silently discards it when the host's
                    // dma_w burst already filled the FIFO (scsi_fuzz
                    // seed 5: 9 stale + 7 dma_w bytes, no status byte).
                    if (c96_fifo_pos != 5'd16) begin
                        c96_fifo[c96_fifo_pos[3:0]] <=
                            c96_xfr_recv_isstat ? xfer_status : xfer_msg;
                        c96_fifo_pos <= c96_fifo_pos + 5'd1;
                    end
                    if (c96_xfr_recv_isstat && c96_xfer_active) begin
                        phase <= S_MSG_IN;
                        // MAME INIT_XFR_WAIT_REQ (ncr53c90.cpp:646-658):
                        // the conditions are ORDERED — the TC0
                        // completion (`dma_command && TC0 && dir==IN`)
                        // is checked FIRST and goes to BUS_COMPLETE
                        // withOUT touching the queue; only the
                        // phase-change arm below it zeroes command_pos
                        // (echo retained; the next command_w then
                        // dispatches immediately instead of queueing).
                        // scsi_fuzz seed 4 (no TC0: follow-up FLUSH must
                        // run, clearing FIFO + tcounter) vs seed 5 (TC0
                        // set by the dma_w burst: the queued FLUSH stays
                        // queued and a third write gets S_GROSS_ERROR).
                        if (!c96_tc0_set)
                            c96_command_pos <= 2'd0;
                    end
                    c96_xfr_recv_pend <= 1'b0;
                end
            end
            // ── STATUS-armed DMA CI_XFER completion → I_BUS ───────────
            // MAME INIT_XFR_WAIT_REQ: once the recv fired, the bus phase
            // (MSG_IN) differs from the latched xfr_phase (STATUS) →
            // INIT_XFR_BUS_COMPLETE, which waits for DRQ to drop
            // (`if (dma_command && drq) break;`) and then runs
            // bus_complete() — I_BUS + dma_set(DMA_NONE) + check_drq.
            // DRQ here is the BUSMD_1 DMA_IN fifo formula, so the
            // completion lands when the host's blind dma_r pops have
            // drained the FIFO below the threshold (scsi_fuzz seed 4
            // SYNC 3: MAME istat=10/stat=87 where the old always-armed
            // park never interrupted).
            if (TURBOSCSI_C96_EN && c96_xfr_armed && c96_xfr_dma &&
                (c96_xfr_phase == S_STATUS) && (phase == S_MSG_IN) &&
                !c96_xfr_recv_pend && !c96_drq_fifo_in_live &&
                (c96_dma_dir == C96_DIR_IN) &&
                !c96_reg_wr_now && !c96_istatus_rd_now) begin
                c96_xfr_armed   <= 1'b0;
                c96_dma_dir     <= C96_DIR_NONE;
                c96_drq_stale   <= 1'b0;  // completion check_drq (MAME :772-799)
                c96_istatus     <= c96_istatus | I_BUS;
                c96_irq_pending <= 1'b1;
            end
            // ── Post-ATN_STOP DMA message drain: one staged byte per
            //    cycle goes to the target as a message ──────────────────
            if (c96_stop_feed) begin
                // MAME send_byte() → fifo_pop() memmove semantics
                for (k = 0; k < 15; k = k + 1)
                    if (k + 1 < {27'd0, c96_fifo_pos})
                        c96_fifo[k] <= c96_fifo[k + 1];
                c96_fifo_pos <= c96_fifo_pos - 5'd1;
                // The NEXT byte rides send_byte()'s delay() timer — see
                // the c96_stop_feed_timer declaration.
                c96_stop_feed_timer <= C96_RECV_QUIESCE;
                // MAME :611-613 — ATN drops with the LAST message byte.
                if ((c96_fifo_pos == 5'd1) && (c96_tcounter == 17'd0))
                    c96_stop_atn_dropped <= 1'b1;
            end else if (c96_sel_stopped && c96_xfr_armed && c96_xfr_dma) begin
                if (pb_dma_shim && (pb_rd || pb_wr))
                    c96_stop_feed_timer <= C96_RECV_QUIESCE;
                else if (c96_stop_feed_timer != 10'd0)
                    c96_stop_feed_timer <= c96_stop_feed_timer - 10'd1;
            end
            // ── Post-ATN_STOP CDB feed: Transfer Information drains the
            //    FIFO into the CDB, the TARGET deciding the length
            //    (c96_cmd_wait; finding F2 continuation) ────────────────
            if (c96_cmdwait_feed) begin
                c96_sel_cdb_byte(c96_fifo[0],
                                 (c96_sel_idx == 6'd0)
                                 ? c96_cdb_len(c96_fifo[0])
                                 : c96_sel_len, 1'b0);
                // MAME send_byte() → fifo_pop() memmove semantics
                for (k = 0; k < 15; k = k + 1)
                    if (k + 1 < {27'd0, c96_fifo_pos})
                        c96_fifo[k] <= c96_fifo[k + 1];
                c96_fifo_pos <= c96_fifo_pos - 5'd1;
            end
            // Armed non-DMA Transfer Information in COMMAND with nothing
            // LEFT to send: I_BUS, disarm, target keeps waiting (MAME
            // INIT_XFR_WAIT_REQ :649 "non-dma out: fifo empty").
            // !c96_xfr_out_park: a transfer that started with an EMPTY
            // FIFO never sent anything, so MAME never reached
            // INIT_XFR_WAIT_REQ and never raised this (scsi_fuzz seed 27).
            if (TURBOSCSI_C96_EN && c96_cmd_wait && c96_xfr_armed &&
                !c96_xfr_dma && (phase == S_COMMAND) && !c96_xfr_out_park &&
                (c96_fifo_pos == 5'd0) && !c96_cmdwait_feed &&
                !c96_reg_wr_now && !c96_istatus_rd_now) begin
                c96_xfr_armed   <= 1'b0;
                c96_dma_dir     <= C96_DIR_NONE;
                c96_drq_stale   <= 1'b0;  // completion check_drq (MAME :772-799)
                c96_istatus     <= c96_istatus | I_BUS;
                c96_irq_pending <= 1'b1;
            end
            // ── Post-ATN_STOP DMA message drain end (tcounter hit 0):
            //    MAME INIT_XFR_WAIT_REQ -> INIT_XFR_BUS_COMPLETE ->
            //    bus_complete(): I_BUS and the command is done.  Whether
            //    the TARGET moves on is a separate question — it does so
            //    only if ATN was actually deasserted by the last SEND
            //    (:611-613).  A FIFO the host drained through the DMA
            //    aperture instead leaves ATN up and the target parked in
            //    MSG_OUT (c96_stop_atn_dropped; scsi_fuzz seeds 32/39).
            if (TURBOSCSI_C96_EN && c96_sel_stopped && c96_xfr_armed &&
                c96_xfr_dma && (c96_tcounter == 17'd0) &&
                (c96_fifo_pos == 5'd0) && !c96_stop_feed &&
                !c96_reg_wr_now && !c96_istatus_rd_now &&
                !(pb_rd && pb_dma_shim)) begin
                c96_xfr_armed   <= 1'b0;
                c96_xfr_dma     <= 1'b0;
                c96_dma_dir     <= C96_DIR_NONE;
                c96_drq_stale   <= 1'b0;  // completion check_drq (MAME :772-799)
                c96_istatus     <= c96_istatus | I_BUS;
                c96_irq_pending <= 1'b1;
                if (c96_stop_atn_dropped) begin
                    c96_sel_stopped <= 1'b0;
                    c96_cmd_wait    <= 1'b1;
                    c96_xfer_active <= 1'b1;
                    c96_sel_idx     <= 6'd0;
                    c96_sel_len     <= 6'd0;
                    phase           <= S_COMMAND;
                    t_req           <= 1'b0;
                end
            end
            // ── C96 select-complete → I_FUNCTION|I_BUS + seq=4 ─────────
            // MAME function_bus_complete(): the select sequence (arb +
            // selection + CDB send) is complete once the target settles
            // in its first post-command bus phase.  Fired one cycle
            // after the back-end dispatch so the status-register phase
            // bits already show the new phase when the ISR reads them.
            //
            // MAME's DISC_SEL_WAIT_REQ (ncr53c90.cpp:544-552) completes
            // on ANY phase that is not COMMAND — `if ((ctrl &
            // S_PHASE_MASK) != S_PHASE_COMMAND) { ... function_bus_
            // complete(); }` — not just the three data/status phases.  A
            // target that answers a select by going straight to MSG IN
            // (or that our reselect path parks in S_RESELECT_MSG) left
            // c96_sel_pending stuck here forever.  The off-bus states
            // are excluded because MAME additionally requires S_REQ, and
            // they are exactly the states in which no target is driving
            // REQ.
            if (TURBOSCSI_C96_EN && c96_sel_pending &&
                !c96_phase_offbus(phase) &&
                (phase != S_COMMAND) && (phase != S_BUS_FREE) &&
                (phase != S_SELECT) && (phase != S_DISCONNECT) &&
                (phase != S_RESET) && (phase != S_RESELECT) &&
                (phase != S_RESELECT_ID)) begin
                c96_sel_pending <= 1'b0;
                c96_dma_dir     <= C96_DIR_NONE;
                c96_drq_stale   <= 1'b0;  // completion check_drq (MAME :772-799)
                c96_istatus     <= c96_istatus | I_FUNCTION | I_BUS;
                c96_irq_pending <= 1'b1;
                c96_seq_step    <= c96_sel_seq_final;
            end
            // ── C96 non-DMA CI_XFER completion → I_BUS ─────────────────
            // A non-DMA Transfer Information in a data-IN phase moves
            // exactly ONE byte and then completes (I_BUS + disarm),
            // REGARDLESS of prior FIFO residue.
            //
            // ⚠️ DELIBERATE MAME DIVERGENCE (2026-09-06, owner-approved —
            // real-chip semantics beat MAME fidelity, precedent: the
            // RESET-instruction decision).  DO NOT "restore parity"
            // without reading this comment and the boot RCA
            // (memory: scsi-53c96-stall-root-cause-2026-09-06).
            //
            // MAME's completion test (ncr53c90.cpp:650, "non-dma in:
            // every byte") is `fifo_pos == 1` AFTER the push — a receive
            // behind prior residue never matches it, so MAME free-runs
            // pulling target bytes until the FIFO fills, with NO
            // interrupt.  The 2026-08-19 seed-13 change made this RTL
            // MAME-faithful, and that faithfulness is exactly what turned
            // a benign lost race into the permanent 7.5.3 boot wedge at
            // ROM 0x40899704 (measured on silicon with the p143 trace
            // ring): an interrupt-context read-to-clear reg-5 read steals
            // one per-byte completion, the ROM's error path skips the
            // FIFO pop, the next 0x10 arms with 1 byte of residue, and
            // the MAME-faithful model free-runs to fifo_pos=16 with the
            // istatus never rising again.  A real 53C96 moves one byte
            // and interrupts whatever the FIFO held, so the same stolen
            // race costs one benign retry on real hardware.
            // scsi_fuzz seeds that exercise a non-DMA DATA IN 0x10 over
            // residue now EXPECTEDLY diverge from MAME (the seed-13
            // family; see docs/scsi_fuzz.md "Deliberate real-chip
            // divergences").
            else if (TURBOSCSI_C96_EN && c96_fifo_fill_beat) begin
                c96_xfr_armed   <= 1'b0;
                c96_dma_dir     <= C96_DIR_NONE;
                c96_drq_stale   <= 1'b0;  // completion check_drq (MAME :772-799)
                c96_istatus     <= c96_istatus | I_BUS;
                c96_irq_pending <= 1'b1;
            end
            // ── C96 non-DMA CI_XFER in DATA IN, supply exhausted →
            //    I_BUS (the trailing-transfer completion) ───────────────
            // Second face of the same real-chip contract: a non-DMA
            // DATA IN Transfer Information with NOTHING to move must
            // still complete.  Reached when the transfer is armed while
            // the back-end has permanently drained (measured on silicon
            // 2026-09-06, JTAG round 2: xfr_armed=1, xfr_dma=0,
            // phase=DATA_IN, fifo_pos=0, nondma_supply=0 — and
            // reproduced in sim: a mid-stream bus reset + re-select
            // leaves vh_kicked=0, so the supply-exhaustion phase
            // backstop above, whose term (b) REQUIRES vh_kicked, can
            // never advance the phase, and c96_fifo_fill_beat needs
            // supply.  Without this arm the command parks with no byte
            // to move and no interrupt, and the ROM's untimed istatus
            // poll at 0x40899704 spins forever).
            //
            // `!vh_busy && !vh_rd_valid` is load-bearing: a multi-block
            // read whose ring is empty ONLY because the provider is
            // still fetching (SD refill latency) must WAIT — completing
            // there would hand the driver an empty FIFO mid-data and
            // corrupt the read.  A busy provider will deliver, the fill
            // beat fires, and the arm above completes with the byte.
            // `!init_ack` mirrors the backstop's in-flight-byte
            // exclusion for the bare-5380 handshake window.
            // No byte is pushed and tcounter is untouched (non-DMA);
            // the queued command retires through the driver's istatus
            // read exactly as any other bus_complete() does.
            else if (TURBOSCSI_C96_EN && c96_xfr_armed && !c96_xfr_dma &&
                     (phase == S_DATA_IN) && !c96_nondma_supply &&
                     !vh_busy && !vh_rd_valid && !init_ack &&
                     !c96_fifo_bus_conflict) begin
                c96_xfr_armed   <= 1'b0;
                c96_dma_dir     <= C96_DIR_NONE;
                c96_drq_stale   <= 1'b0;  // completion check_drq (MAME :772-799)
                c96_istatus     <= c96_istatus | I_BUS;
                c96_irq_pending <= 1'b1;
            end
            // ── C96 non-DMA CI_XFER in DATA OUT → I_BUS ───────────────
            // MAME ncr53c90.cpp:649 ("non-dma out: fifo empty") →
            // INIT_XFR_BUS_COMPLETE (:686-692) → bus_complete()
            // (:792-799, `state = IDLE; istatus |= I_BUS`).  Unlike the
            // data-IN form above this does NOT stop after one byte —
            // c96_out_send_beat keeps sending while the FIFO has
            // bytes, and this fires only once it is (about to be)
            // empty.  See the c96_out_send_done declaration for the
            // 2026-08-07 HW stall this closes.
            else if (c96_out_send_done) begin
                c96_xfr_armed   <= 1'b0;
                c96_dma_dir     <= C96_DIR_NONE;
                c96_drq_stale   <= 1'b0;  // completion check_drq (MAME :772-799)
                c96_istatus     <= c96_istatus | I_BUS;
                c96_irq_pending <= 1'b1;
            end
            // ── C96 non-DMA CI_XFER in STATUS → I_BUS ─────────────────
            // MAME :650 "non-dma in: every byte" -> INIT_XFR_BUS_COMPLETE
            // (:686-692) -> bus_complete() (:792-799).  MUST sit ahead of
            // the phase-change hook below, which would otherwise claim
            // this cycle and complete the command without ever pushing
            // the status byte — the silent-empty-FIFO bug this fixes.
            else if (c96_nondma_in_status) begin
                c96_xfr_armed   <= 1'b0;
                c96_dma_dir     <= C96_DIR_NONE;
                c96_drq_stale   <= 1'b0;  // completion check_drq (MAME :772-799)
                c96_istatus     <= c96_istatus | I_BUS;
                c96_irq_pending <= 1'b1;
                // The status byte is ACKed and released, so the target
                // advances to MSG_IN.  With prior FIFO residue the
                // completion is MAME's PHASE-CHANGE arm (fifo_pos != 1
                // skips the "every byte" test) — which also zeroes the
                // command queue (`command_pos = 0`, :616); the
                // empty-FIFO case completes through the "every byte"
                // arm and leaves the queue alone (scsi_fuzz seed 13).
                if (c96_xfer_active)
                    phase <= S_MSG_IN;
                if (c96_fifo_pos != 5'd0)
                    c96_command_pos <= 2'd0;
            end
            // ── C96 non-DMA CI_XFER in MSG IN → I_FUNCTION ────────────
            // NOT I_BUS: ncr53c90.cpp:629-630 takes INIT_XFR_RECV_BYTE_NACK
            // for a non-DMA MSG IN, which routes to
            // INIT_XFR_FUNCTION_COMPLETE (:676-684) and function_complete()
            // (:782-790) — `istatus |= I_FUNCTION`.
            else if (c96_nondma_in_msgin) begin
                c96_xfr_armed   <= 1'b0;
                c96_dma_dir     <= C96_DIR_NONE;
                c96_drq_stale   <= 1'b0;  // completion check_drq (MAME :772-799)
                c96_istatus     <= c96_istatus | I_FUNCTION;
                c96_irq_pending <= 1'b1;
                c96_msgin_ack_held <= 1'b1;   // NACK path: ACK stays up
            end
            // ── C96 transfer-info completion → I_BUS ───────────────────
            // MAME bus_complete(): an armed CI_XFER ends when the target
            // changes phase away from the data phase (to STATUS).  The
            // initiator then runs CI_COMPLETE / CI_MSG_ACCEPT.
            else if (TURBOSCSI_C96_EN && c96_xfr_armed &&
                     (phase == S_STATUS) &&
                     // 2026-08-19 FIX.  This used to enumerate
                     // {S_DATA_IN, S_DATA_OUT}.  MAME's arm is GENERAL —
                     // ncr53c90.cpp:653-657 completes on ANY live-vs-
                     // latched mismatch (`(ctrl & S_PHASE_MASK) !=
                     // xfr_phase`) — and c96_xfr_phase latches our
                     // INTERNAL FSM state (`c96_xfr_phase <= phase`),
                     // whose enum includes S_CMD_EXEC / S_VH_WAIT_RD /
                     // S_VH_WAIT_WR: states MAME's xfr_phase can never
                     // hold, because it is masked from the bus lines.
                     // A non-DMA CI_XFER left armed with any latched
                     // value outside the enumerated pair therefore had
                     // NO completion hook at all: measured 9 of 12
                     // latched values wedged, every one of them landing
                     // on exactly the hardware signature (live phase
                     // STATUS, one staged byte, istatus 0x00, no IRQ) —
                     // the ROM's untimed wait at 0x40899704 then spins
                     // forever.  `!= S_STATUS` is the same superset
                     // MAME has while preserving the "armed IN STATUS
                     // pulls bytes and stays armed" case (scsi_fuzz
                     // seed 14), which the equality below still covers.
                     (c96_xfr_phase != S_STATUS) &&
                     // MAME INIT_XFR_WAIT_REQ condition ORDER: the TC0
                     // completion (checked first, :648) wins over the
                     // phase-change arm — a DMA transfer whose count is
                     // exhausted completes through the TC0 hook below
                     // (no queue zero, waits for the FIFO drain);
                     // this arm keeps the non-DMA / short-supply cases.
                     !(c96_xfr_dma && c96_tc0_set) &&
                     // INIT_XFR_BUS_COMPLETE (:688-691): `if
                     // (dma_command && drq) break;` — a DMA short read
                     // whose staged bytes are still in the FIFO holds
                     // I_BUS until the host drains below the BUSMD_1
                     // threshold.  Not an interrupt-free wedge: every
                     // host pop path pops this same FIFO, so the drain
                     // that the driver does anyway releases it.
                     !(c96_xfr_dma && (c96_dma_dir == C96_DIR_IN) &&
                       c96_drq_fifo_in_live)) begin
                // bus_complete on a PHASE CHANGE out of the data phase
                // the transfer was armed in.  A transfer armed IN
                // STATUS/MSG_IN instead pulls bytes and stays armed
                // (MAME INIT_XFR, measured: scsi_fuzz seed 14).
                c96_xfr_armed   <= 1'b0;
                c96_dma_dir     <= C96_DIR_NONE;
                c96_drq_stale   <= 1'b0;  // completion check_drq (MAME :772-799)
                c96_istatus     <= c96_istatus | I_BUS;
                c96_irq_pending <= 1'b1;
                // MAME :616: phase-change completion zeroes the command
                // queue (`command_pos = 0`; echo retained).  The
                // TC0-path chunk completion below does NOT.
                c96_command_pos <= 2'd0;
            end
            // ── C96 DMA chunk completion → I_BUS (no phase change) ─────
            // MAME INIT_XFR_WAIT_REQ → INIT_XFR_BUS_COMPLETE: a DMA
            // transfer-information command ALSO completes when the
            // transfer counter is exhausted (TC0) and the CPU has
            // drained the last byte (drq drops), even though the target
            // still sits in the data phase with more data to move.  The
            // Q700 ROM moves each 512-byte block as 32 chunks of
            // DMA|CI_XFER with tcount=16 and needs this I_BUS interrupt
            // per chunk (ROM 0x40899322 waits for INTR after its 16
            // reads).
            // 2026-07-26: the DATA_IN term used to require
            // `c96_xfr_left == 0` ALONE.  That is NOT what MAME does and it
            // deadlocks.  MAME's condition is TC0 && !drq
            // (ncr53c90.cpp INIT_XFR_BUS_COMPLETE:
            //  `if (dma_command && drq) break; bus_complete();`), and OUR
            // OWN drq for DATA_IN (see drq_c96 above) is
            //     (c96_accept_pend != 0) && (c96_xfr_left != 0)
            // so `!drq` is the DISJUNCTION
            //     (c96_accept_pend == 0) || (c96_xfr_left == 0).
            // Requiring only `c96_xfr_left == 0` therefore misses the case
            // "target has delivered everything (TC0) and the host has taken
            // everything the chip accepted (accept_pend == 0), but the host
            // issued fewer blind DMA-port beats than c96_xfr_left expected".
            // In that state the chip has NOTHING left to hand over, drq is
            // low, MAME fires I_BUS -- and we hung forever waiting for a
            // beat that will never come.  HW-observed 2026-07-26 on build
            // 0x0BC53B30: status=0x11 (DATA_IN + S_TC0), cmd=0x90, tc=0,
            // fifo=0, INTR clear, ROM spinning at 0x408992D2, so the whole
            // I/O never completed, IODone was never called, and the caller
            // spun on ioResult forever at RAM 0x0002E8A4.
            // 2026-08-19 R1 REWORK: with the DMA-in bytes staged through
            // the real c96_fifo, `!drq` IS MAME's condition verbatim —
            // the BUSMD_1 DMA_IN fifo formula (c96_drq_fifo_in_live).
            // The dir==IN arm deliberately has NO phase guard: MAME's
            // INIT_XFR_WAIT_REQ takes the TC0 condition whether the
            // target still sits in the data phase or has already moved
            // to STATUS, and the same wait also finishes a STATUS/
            // MSG_IN-armed pull whose dma_w burst forced TC0 (scsi_fuzz
            // seed 5: I_BUS only once the host drains the full FIFO).
            else if (TURBOSCSI_C96_EN && c96_xfr_armed && c96_xfr_dma &&
                     c96_tc0_set &&
                     ((c96_dma_dir == C96_DIR_IN)
                      // IN: `if (dma_command && drq) break;` — wait for
                      // the host to drain the staged FIFO (and for any
                      // pending recv byte to land/drop first).
                      // 2026-09-07 pre-staging: ALSO wait until the ring
                      // is re-staged for the NEXT chunk's blind burst
                      // (vh_ring_staged) — this I_BUS is what releases
                      // the ROM into that burst, so the refill latency
                      // belongs here, not mid-burst as a withheld beat.
                      // The staged term is forced TRUE by a closed
                      // stream, by non-multi transfers, and by the
                      // stuck-supply watchdog's fault latch, so it can
                      // delay but never park a completion.
                      ? (!c96_drq_fifo_in_live && !c96_xfr_recv_pend &&
                         vh_ring_staged)
                      // OUT (:648 `fifo_pos == 0`): everything pushed
                      // has been sent; fires whether the target still
                      // sits in DATA_OUT or already moved to STATUS
                      : (c96_fifo_pos == 5'd0))) begin
                c96_xfr_armed   <= 1'b0;
                c96_dma_dir     <= C96_DIR_NONE;
                c96_drq_stale   <= 1'b0;  // completion check_drq (MAME :772-799)
                c96_istatus     <= c96_istatus | I_BUS;
                c96_irq_pending <= 1'b1;
            end
        end
        // ── sec_buf single write port ─────────────────────────────────
        // The one and only array write in the module — that single
        // textual write statement is what makes the RAM template legal.
        // sec_we/sec_wa/sec_wd come from `sec_wr_mux` below, which
        // restates the guards of the four dynamic write sites; each of
        // those sites carries an `ifdef VERILATOR assertion that the mux
        // agrees with it, so a drift between the two is a loud sim
        // failure rather than a silent lost store.
        //
        // No reset clause: sec_buf's contents are undefined out of reset,
        // exactly as before.  Nothing reads a byte it has not written
        // first — buf_rd_ptr / vh_drain_ptr only advance behind a fill.
        if (sec_we) sec_buf[sec_wa] <= sec_wd;
    end

    // ══════════════════════════════════════════════════════════════════
    // sec_buf write-port mux
    // ══════════════════════════════════════════════════════════════════
    // Mirrors the guards of the four dynamic sec_buf write sites in the
    // main FSM.  They live in mutually exclusive phases, so this is a
    // plain priority chain and never drops a write.  The `case (phase)`
    // and the `!init_rst` guard match the main block's structure: the
    // phase FSM there sits inside `if (rst) ... else if (init_rst) ...
    // else begin case (phase) ...`.
    //
    // IF YOU ADD A sec_buf WRITE SITE, ADD IT HERE TOO.  The per-site
    // `ifdef VERILATOR assertions will catch you if you forget, but only
    // if the new path is actually exercised by a testbench.
    always @* begin
        sec_we = 1'b0;
        sec_wa = vh_fill_ptr;
        sec_wd = vh_rd_data;
        if (!rst && !init_rst) begin
            case (phase)
                S_VH_WAIT_RD: begin
                    if (vh_rd_valid) begin
                        sec_we = 1'b1;
                        sec_wa = vh_fill_ptr;
                        sec_wd = vh_rd_data;
                    end
                end
                S_DATA_IN: begin
                    // Concurrent multi-block read ring fill while draining to the
                    // initiator.
                    if (vh_rd_valid && (vh_multi_read)) begin
                        sec_we = 1'b1;
                        sec_wa = vh_fill_ptr;
                        sec_wd = vh_rd_data;
                    end
                end
                S_DATA_OUT: begin
                    if (pseudo_dma_out_beat) begin
                        sec_we = 1'b1;
                        sec_wa = buf_wr_ptr;
                        sec_wd = pb_wdata;
                    end else if (c96_out_send_beat) begin
                        // Non-DMA CI_XFER send: the byte comes off the
                        // head of the chip FIFO instead of the
                        // pseudo-DMA port.  Mutually exclusive with the
                        // beat above by c96_fifo_out_bus_conflict.
                        sec_we = 1'b1;
                        sec_wa = buf_wr_ptr;
                        sec_wd = c96_fifo[0];
                    end else if (t_req && ack_rise) begin
                        sec_we = 1'b1;
                        sec_wa = buf_wr_ptr;
                        sec_wd = r_output_data;
                    end
                end
                default: begin
                    sec_we = 1'b0;
                end
            endcase
        end
    end

`ifdef VERILATOR
    // ══════════════════════════════════════════════════════════════════
    // Byte-conservation instrumentation (sim-only observers, no effect
    // on any non-VERILATOR behaviour).  Task: agent/scsi-chunk-wedge-repro.
    //
    // Per-transaction counters, reset at CDB dispatch (entry into
    // S_CMD_EXEC) and reported at transaction end (return to S_BUS_FREE):
    //   cons_supplied — every vh_rd_valid pulse from the provider
    //   cons_counted  — vh_rd_valid pulses that landed in a phase whose
    //                   case-arm counts them into vh_buf_count / sec_buf
    //                   (S_VH_WAIT_RD always; S_DATA_IN only when
    //                   vh_multi_read — mirrors sec_wr_mux exactly)
    //   cons_accepted — c96_accept_ev pulses (chip-side tcounter decrements)
    //   cons_drained  — consumer byte events in S_DATA_IN (the OR of
    //                   pseudo_dma_in_beat / c96_fifo_fill_beat /
    //                   t_req&&ack_rise — one ring decrement per cycle,
    //                   matching the vh_buf_count net-update case)
    //
    // A "CONSERVATION VIOLATION" line fires the moment a provider pulse
    // lands in a phase that does not count it (supplied != counted from
    // that cycle on) — that pulse's byte is lost from the ring's
    // accounting and the final chunk(s) of the transaction starve.
    // A "SATURATION DROP" line fires if a counted-arm pulse could not
    // increment vh_buf_count because it sat at the 10-bit saturation
    // guard (1023) — the other way a fill can go missing.
    reg [31:0] cons_supplied   /* verilator public_flat_rd */;
    reg [31:0] cons_counted    /* verilator public_flat_rd */;
    reg [31:0] cons_accepted   /* verilator public_flat_rd */;
    reg [31:0] cons_drained    /* verilator public_flat_rd */;
    reg [31:0] cons_violations /* verilator public_flat_rd */;
    // Provider-contract / ring-boundary observers:
    //   cons_ready_viol — vh_rd_valid pulses arriving while vh_rd_ready
    //                     is LOW (vhdd contract violation; models the
    //                     CDC in-flight exposure of sd_scsi_bridge)
    //   cons_ring_full  — vh_rd_valid pulses landing with the ring at or
    //                     beyond RING_FULL (overwrite of undrained data)
    reg [31:0] cons_ready_viol /* verilator public_flat_rd */;
    reg [31:0] cons_ring_full  /* verilator public_flat_rd */;
    reg [3:0]  cons_phase_q;
    wire cons_fill_counted = vh_rd_valid &&
                             ((phase == S_VH_WAIT_RD) ||
                              ((phase == S_DATA_IN) && vh_multi_read));
    wire cons_drain_ev = (phase == S_DATA_IN) &&
                         ((TURBOSCSI_C96_EN ? c96_accept_ev
                                            : pseudo_dma_in_beat) ||
                          c96_fifo_fill_beat || c96_cpt_din_beat ||
                          (t_req && ack_rise));

    // ── WRITE-side ring observers (the CMD25 multi-block ring) ────────
    // The read side above has had ring instrumentation since the
    // 2026-07-15 overrun investigation; the WRITE side — the identical
    // ring running the other way, with the initiator as producer and the
    // backing store as consumer — had none, in either direction.  Both
    // failure modes write WRONG BYTES to the card at the CORRECT LBA
    // with GOOD status, so nothing above notices:
    //
    //   cons_wr_over  — a producer byte landed in sec_buf while the ring
    //                   was already at/beyond RING_FULL, i.e. it
    //                   overwrote a byte the provider has not drained.
    //                   Producer back-pressure failed.
    //   cons_wr_under — the provider took a byte while the ring was
    //                   EMPTY, i.e. it re-sent whatever stale content
    //                   sat at vh_drain_ptr from the previous block.
    //                   Consumer back-pressure failed.  The vh_buf_count
    //                   update below deliberately CLAMPS at 0 rather
    //                   than wrapping, which is what made this event
    //                   invisible; the clamp stays (a wrapped 10-bit
    //                   count is strictly worse) and this counter is
    //                   what makes it observable.
    //
    // Both are restricted to S_DATA_OUT + vh_multi_write because that is
    // the only phase in which vh_buf_count is maintained: the tail of a
    // multi-block write drains from S_VH_WAIT_WR with the whole final
    // block already buffered (buf_count residue, never decremented), and
    // single-block writes buffer the entire block before the kick.
    reg [31:0] cons_wr_over  /* verilator public_flat_rd */;
    reg [31:0] cons_wr_under /* verilator public_flat_rd */;
    wire cons_wr_push_ev = (phase == S_DATA_OUT) && vh_multi_write &&
                           (pseudo_dma_out_beat || c96_out_send_beat ||
                            (t_req && ack_rise));
    wire cons_wr_drain_ev = (phase == S_DATA_OUT) && vh_multi_write &&
                            vh_wr_ready && vh_kicked;
    always @(posedge clk) begin
        if (rst) begin
            cons_supplied   <= 32'd0;
            cons_counted    <= 32'd0;
            cons_accepted   <= 32'd0;
            cons_drained    <= 32'd0;
            cons_violations <= 32'd0;
            cons_ready_viol <= 32'd0;
            cons_ring_full  <= 32'd0;
            cons_wr_over    <= 32'd0;
            cons_wr_under   <= 32'd0;
            cons_phase_q    <= S_BUS_FREE;
        end else begin
            cons_phase_q <= phase;
            // Loud, immediate: a provider byte the ring will never count.
            if (vh_rd_valid && !cons_fill_counted) begin
                cons_violations <= cons_violations + 32'd1;
                $display("%0t scsi: CONSERVATION VIOLATION vh_rd_valid in phase=%0d (uncounted) multi=%0b buf_count=%0d tcounter=%0d accept_pend=%0d xfr_left=%0d supplied=%0d counted=%0d",
                         $time, phase, vh_multi_read, vh_buf_count,
                         c96_tcounter, c96_accept_pend, c96_xfr_left,
                         cons_supplied + 32'd1, cons_counted);
            end
            if (cons_fill_counted && (vh_buf_count == 10'd1023)) begin
                cons_violations <= cons_violations + 32'd1;
                $display("%0t scsi: CONSERVATION SATURATION DROP vh_rd_valid at buf_count=1023 phase=%0d",
                         $time, phase);
            end
            // vhdd contract: a provider must stop the stream at the
            // source while rd_ready is low.  Pulses arriving anyway are
            // the CDC in-flight exposure under measurement.
            if (vh_rd_valid && !vh_rd_ready) begin
                cons_ready_viol <= cons_ready_viol + 32'd1;
                $display("%0t scsi: RD_READY CONTRACT VIOLATION vh_rd_valid while rd_ready=0 phase=%0d buf_count=%0d (n=%0d)",
                         $time, phase, vh_buf_count,
                         cons_ready_viol + 32'd1);
            end
            if (vh_rd_valid && (vh_buf_count >= RING_FULL)) begin
                cons_ring_full <= cons_ring_full + 32'd1;
                $display("%0t scsi: RING FULL OVERWRITE vh_rd_valid at buf_count=%0d phase=%0d (n=%0d)",
                         $time, vh_buf_count, phase,
                         cons_ring_full + 32'd1);
            end
            // WRITE ring, producer side: an initiator byte landing on a
            // full ring overwrites undrained data.  A simultaneous drain
            // is excluded — count stays at RING_FULL, nothing is lost.
            if (cons_wr_push_ev && (vh_buf_count >= RING_FULL) &&
                !cons_wr_drain_ev) begin
                cons_wr_over <= cons_wr_over + 32'd1;
                $display("%0t scsi: WR RING OVERRUN initiator byte at buf_count=%0d wr_ptr=%0d drain_ptr=%0d (n=%0d)",
                         $time, vh_buf_count, buf_wr_ptr, vh_drain_ptr,
                         cons_wr_over + 32'd1);
            end
            // WRITE ring, consumer side: the provider took a byte the
            // initiator never supplied.  sec_buf[vh_drain_ptr] still
            // holds the previous block's content at that offset, so the
            // card receives stale bytes at the right LBA with GOOD
            // status.  The count clamp below hides this completely.
            if (cons_wr_drain_ev && (vh_buf_count == 10'd0) &&
                !cons_wr_push_ev) begin
                cons_wr_under <= cons_wr_under + 32'd1;
                $display("%0t scsi: WR RING UNDERFLOW provider byte at buf_count=0 wr_ptr=%0d drain_ptr=%0d (n=%0d)",
                         $time, buf_wr_ptr, vh_drain_ptr,
                         cons_wr_under + 32'd1);
            end
            if ((cons_phase_q != S_CMD_EXEC) && (phase == S_CMD_EXEC)) begin
                // CDB dispatch — fresh transaction; include this cycle's
                // own events in the fresh counts.
                cons_supplied <= vh_rd_valid       ? 32'd1 : 32'd0;
                cons_counted  <= cons_fill_counted ? 32'd1 : 32'd0;
                cons_accepted <= c96_accept_ev     ? 32'd1 : 32'd0;
                cons_drained  <= cons_drain_ev     ? 32'd1 : 32'd0;
            end else begin
                if (vh_rd_valid)       cons_supplied <= cons_supplied + 32'd1;
                if (cons_fill_counted) cons_counted  <= cons_counted  + 32'd1;
                if (c96_accept_ev)     cons_accepted <= cons_accepted + 32'd1;
                if (cons_drain_ev)     cons_drained  <= cons_drained  + 32'd1;
            end
            if ((cons_phase_q != S_BUS_FREE) && (phase == S_BUS_FREE)) begin
                $display("%0t scsi: CONSERVATION transaction end: supplied=%0d counted=%0d accepted=%0d drained=%0d buf_count_residue=%0d tcounter=%0d accept_pend=%0d xfr_left=%0d ready_viol=%0d ring_full=%0d wr_over=%0d wr_under=%0d",
                         $time, cons_supplied, cons_counted, cons_accepted,
                         cons_drained, vh_buf_count, c96_tcounter,
                         c96_accept_pend, c96_xfr_left, cons_ready_viol,
                         cons_ring_full, cons_wr_over, cons_wr_under);
            end
        end
    end
`endif
`ifdef VERILATOR
    // ══════════════════════════════════════════════════════════════════
    // Stranded-interrupt monitor (simulation only)
    // ══════════════════════════════════════════════════════════════════
    //
    // WHY THIS EXISTS
    // ───────────────
    // `irq` feeds VIA2's CB2 input through an inverter
    // (fpga_top_peripherals.vh:1763, `.cb2_in(~scsi_irq_pb)`).  The Mac
    // programs VIA2 PCR = 0x22 — CB2 in INDEPENDENT, NEGATIVE-edge mode —
    // so a 0→1 transition of `irq` LATCHES IFR.CB2 and, per via2.v:313,
    // an ORB access does NOT clear it: only a write to IFR does.  The
    // chip's own IRQ line is a LEVEL, but VIA2 remembers the EDGE.
    //
    // That makes every silent retraction of `c96_irq_pending` — any clear
    // that is NOT the software-visible istatus (register 5) read — a
    // machine-killing bug: the edge outlives the condition, the driver
    // ISR reads istatus, sees 0, claims nothing and never writes IFR, and
    // the level-sensitive via2_irq re-fires forever.  irq_agg's strict
    // priority then starves the 60 Hz VIA1 tick.  Measured twice on
    // silicon (2026-09-14 and 2026-09-15): ~250k-280k level-2 exc/s with
    // the 53C96 completely idle — istatus=0, irq=0, phase=BUS_FREE,
    // SD completions frozen.
    //
    // Only TWO clears are legitimate:
    //   • `rst`            — the module reset (:2433)
    //   • an istatus read  — the software-visible clear (:2932)
    // The only other clear in the whole module is CM_RESET (chip reset,
    // opcode 0x02, :3036).  That one is MAME-faithful — MAME's
    // ncr53c90_device::device_reset() also does `irq = false;
    // m_irq_handler(irq)` — and the Mac driver defends it: measured over
    // a healthy MAME boot, every CM_RESET is bracketed by `IER <- 0x08`
    // (mask CB2) before and `IFR <- 0x88` (clear the flag) after.  So the
    // RTL is left MAME-faithful and this monitor NAMES the site instead,
    // with the hold time, so a firing can be judged: an assertion shorter
    // than one phi2 period (64 pb_clk) cannot have been sampled by VIA2's
    // edge latch and is harmless; a longer one WAS latched and, if CB2 is
    // armed and unguarded at that moment, is a permanent livelock.
    reg        irqmon_d1, irqmon_d2;
    reg        irqmon_rd5_d1, irqmon_rd5_d2;
    reg        irqmon_rst_d1, irqmon_rst_d2;
    reg        irqmon_cmrst_d1, irqmon_cmrst_d2;
    reg [31:0] irqmon_hi_cyc;      // cycles the current assertion has held
    reg [31:0] irqmon_hi_cyc_d1;
    reg [31:0] irqmon_strand_cnt;
    wire       irqmon_istatus_rd = TURBOSCSI_C96_EN && pb_rd && !pb_dma_shim &&
                                   (pb_addr[3:0] == 4'h5);
    // The CM_RESET arm, reconstructed from the dispatch decode rather than
    // poked into the synthesisable case — keeps the RTL untouched.
    wire       irqmon_cm_reset  = TURBOSCSI_C96_EN && c96_cmd_dispatch &&
                                  c96_cmd_valid &&
                                  ((c96_cmd_cur & 8'h7f) == CM_RESET);
    // One phi2 period at PB_CLK_HZ=50 MHz (783 360 Hz VIA timebase).  An
    // IRQ held at least this long has certainly been sampled by VIA2's
    // CB2 edge latch, so retracting it strands a flag in IFR.
    localparam [31:0] IRQMON_PHI2_CYC = 32'd64;
    always @(posedge clk) begin
        irqmon_d1        <= c96_irq_pending;
        irqmon_d2        <= irqmon_d1;
        irqmon_rd5_d1    <= irqmon_istatus_rd;
        irqmon_rd5_d2    <= irqmon_rd5_d1;
        irqmon_rst_d1    <= rst;
        irqmon_rst_d2    <= irqmon_rst_d1;
        irqmon_cmrst_d1  <= irqmon_cm_reset;
        irqmon_cmrst_d2  <= irqmon_cmrst_d1;
        irqmon_hi_cyc    <= c96_irq_pending ? (irqmon_hi_cyc + 32'd1) : 32'd0;
        irqmon_hi_cyc_d1 <= irqmon_hi_cyc;
        if (rst) begin
            irqmon_strand_cnt <= 32'd0;
        end else if (TURBOSCSI_C96_EN && irqmon_d2 && !irqmon_d1 &&
                     !irqmon_rd5_d2 && !irqmon_rst_d2) begin
            irqmon_strand_cnt <= irqmon_strand_cnt + 32'd1;
            $display("%0t scsi: STRANDED IRQ RETRACTION #%0d (%s, %s) -- c96_irq_pending 1->0 with NO istatus read, held %0d cycles; istatus=%02x sticky=%02x phase=%0d cmd_pos=%0d cmd_q=%02x",
                     $time, irqmon_strand_cnt + 32'd1,
                     irqmon_cmrst_d2 ? "CM_RESET :3036" : "UNATTRIBUTED SITE",
                     (irqmon_hi_cyc_d1 >= IRQMON_PHI2_CYC)
                         ? "VIA2-LATCHABLE: >= 1 phi2 period"
                         : "below one phi2 period",
                     irqmon_hi_cyc_d1,
                     c96_istatus, c96_status_sticky, phase,
                     c96_command_pos, c96_command_q);
        end
    end
`endif
    // Probe packing — see the port declaration for the field map.
    assign dbg_c96_state = {c96_sel_stopped,      // [83]
                            c96_sel_active,       // [82]
                            c96_xfer_active,      // [81]
                            c96_dma_dir,          // [80:79]
                            c96_accept_pend,      // [78:69]
                            c96_xfr_left,         // [68:52]
                            c96_xfr_armed,        // [51]
                            c96_xfr_dma,          // [50]
                            c96_xfr_phase,        // [49:46]
                            phase,                // [45:42]
                            c96_command_pos,      // [41:40]
                            c96_command_q,        // [39:32]
                            c96_cmd_q1,           // [31:24]
                            c96_fifo_pos,         // [23:19]
                            c96_istatus,          // [18:11]
                            c96_status_sticky,    // [10:3]
                            c96_irq_pending,      // [2]
                            t_req,                // [1]
                            c96_nondma_supply};   // [0]

endmodule
