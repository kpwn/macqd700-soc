// boot_fsm.v — Power-on ROM loader: SD (SPI) → DDR4 (AXI4-MM master)
//
// Role after the sd-ctrl-unify refactor
//   * Owns the SD initialisation sequence (CMD0/8/55+41/58/6).
//   * Delegates sector data transport to `sd_ctrl.v` (CMD18 one-shot
//     multi-block read of all NUM_SECTORS blocks).
//   * Owns the AXI4-MM write master — streams the rd_valid/rd_data byte
//     stream from sd_ctrl into 32-bit single-beat writes (big-endian
//     packing).  Earlier versions emitted one 128-beat AXI burst per
//     sector, but the first-light FPGA memory path serialises 32-bit
//     traffic through a single-beat narrow-to-wide adapter.
//
// Why CMD18 (single call)?
//   CMD17 per-sector ~164 µs @ 25 MHz SPI × 2048 sectors = ~0.34 s; every
//   sector pays the full 6-byte command + R1-poll + token-wait overhead.
//   CMD18 amortises all of that across the entire transfer: one command
//   frame, one R1 poll, then just per-block 0xFE-token-wait + 512 data +
//   2 CRC.  A single CMD18 run to completion should land closer to ~0.13 s
//   at 25 MHz SPI — 2–3× faster.  At 50 MHz SPI (HS mode, HS_HALF=2) the
//   win is even larger — the SPI byte is now ~160 ns, so the fixed
//   overheads dominate progressively less.
//
//   We do NOT chunk into smaller CMD18 invocations.  SD cards can stream
//   indefinitely on CMD18 until CMD12, and the test suite confirms the
//   card handles arbitrary-length multi-block reads.  If a card rejects
//   long CMD18 runs (hasn't been observed in the wild for the target
//   boards), we'd fall back to multiple NUM_SECTORS/N chunks — leaving a
//   comment here so the future maintainer knows where to poke.
//
// External interface — UNCHANGED from the pre-refactor version.
//   Same SPI byte handshake, same AXI4-MM master, same status outputs.
//
// Internal structure
//   * The SD init sequence is driven by a trimmed FSM carrying over from
//     the pre-refactor module — states ST_DUMMY_* through ST_CMD6_CRC_*.
//   * During init, this module drives the SPI byte sub-FSM directly
//     (bgo/bt → bdone/br).  `sd_ctrl` is held in reset via its `go`
//     input staying low; its spi_cmd_* outputs are muxed away.
//   * Once init is done, we hand the SPI byte interface over to sd_ctrl
//     and fire one `go` pulse for the CMD18 all-sector run.
//   * The AXI writer packs incoming bytes 4-at-a-time (big-endian), queues
//     completed words, and fires one single-beat write per word.
//
// Timing / latency claims are the same as before: one outstanding AXI
// write at a time; SPI is the bandwidth bottleneck.

module boot_fsm #(
    // Committed Q700 ROM is 1 MiB = 2048 512-byte sectors.  The SD image
    // still reserves the first 4 MiB (8192 sectors) before raw/SCSI data;
    // this loader only copies the bytes the current ROM image needs.
    parameter [31:0] NUM_SECTORS   = 32'd2048,
    // Sector index (block address) on the SD image where the ROM payload
    // begins.  Today's first-light layout puts the ROM at sector 0; if a
    // future layout reserves the front of the SD for a ROM-loader header
    // or signature page, bump this and the lba arg below scales correctly
    // for both card classes.
    parameter [31:0] START_SECTOR  = 32'd0,
    parameter [31:0] ROM_BASE_ADDR = 32'h4000_0000,
    parameter [3:0]  AXI_ID        = 4'd0,
    // CPU_M68K040 only (see fpga_top_boot_master.vh): cpu040's
    // instruction fetch bypasses the crossbar's ROM-overlay redirect
    // entirely (Level A / task #269 routes axi_i straight to L2C), so
    // that redirect can no longer be relied on to synthesize ROM
    // content at low addresses at reset -- there is no single point left
    // that is guaranteed to see every path the CPU might reach memory
    // through. Real Q700 hardware never had this problem: one physical
    // bus, one address decoder, every access visible to it. This SoC's
    // boot loader does the equivalent by hand instead: while streaming
    // the ROM image to its native ROM_BASE_ADDR, also write each word a
    // second time at the same relative offset off address 0, so the CPU
    // finds correct ROM content already sitting in real RAM at reset --
    // no runtime redirect logic needed, and correct regardless of which
    // physical path any future routing change sends CPU traffic through.
    parameter        MIRROR_LOW_RAM     = 1'b0,
    // How many bytes of the low mirror to write (and, correspondingly,
    // to exclude from the RAM pre-zero pass below so it doesn't
    // immediately overwrite them with zeros). Matches the real ROM
    // image period (`AXI_ROM_IMAGE_SIZE` in axi_defs.vh), not the
    // larger ROM_SIZE DDR reservation window the old crossbar overlay
    // used -- only the bytes SD actually streams in ever need mirroring.
    parameter [31:0] MIRROR_IMAGE_BYTES = 32'h0010_0000,
    // Pre-zero this many bytes of RAM (starting at 0x0000_0000) after the
    // ROM copy completes, before releasing the CPU.  Default = 4 MiB,
    // matching the default RAM_WINDOW_LG2=22.  Set to 0 to disable the
    // zero pass (e.g. for tb units that don't need deterministic RAM).
    parameter [31:0] ZERO_BYTES    = 32'h0040_0000,
    // Beats-minus-one per RAM pre-zero AXI write burst (32-bit beats).
    // 255 = the AXI4 maximum, 256 beats x 4 B = 1 KiB retired per AW/B
    // round trip.  See the ST_ZERO_* geometry note further down for why
    // that is safe (naturally aligned, so it cannot cross the AXI 4 KiB
    // boundary that nothing between here and the MIG checks or splits) and
    // for what it is and is NOT worth.  Exposed as a parameter, not a
    // localparam, so tb/tb_sd_boot_top.v can sweep the burst shape and so
    // an integrator can dial it back without editing the FSM.
    // 63 (256 B/burst -> wide AWLEN 15 through axi_narrow_to_wide), NOT 255.
    // The fabric is HW-proven only to wide AWLEN=3 for writes; 255 would map
    // to wide AWLEN=63.  A downstream audit found every counter structurally
    // wide enough and the burst is naturally aligned so it cannot cross the
    // AXI 4 KiB boundary (nothing between here and the MIG checks or splits
    // at 4 KiB), but the MEASURED gain of 255 over 63 is only 0.7% -- not
    // worth taking unproven burst depth to hardware for.  Raise it once the
    // fabric is proven deeper.
    parameter [7:0]  ZERO_AWLEN    = 8'd63,

    // Fallback card size (in 512-byte sectors) used only when CMD9/CSD
    // cannot be read or decodes to something implausible.  512 MB, which
    // is the historical hardcoded value the SCSI target used to report.
    parameter [31:0] CARD_LBAS_DEFAULT = 32'd1048576,
    // A decoded capacity at or below this is rejected as garbage: the
    // card must at minimum hold the reserved ROM area plus something.
    parameter [31:0] CARD_LBAS_MIN     = 32'd8192,

    // Second RAM pre-zero pass, over the VRAM pixel aperture instead of
    // low main RAM. Same reasoning as the main ZERO_BYTES pass: give
    // scanout a deterministic (black) framebuffer at reset instead of
    // whatever garbage DRAM happened to power up with, so a boot that
    // never gets far enough to paint anything doesn't show visual noise
    // that looks like a display fault. Disabled by default (0 bytes);
    // the real SoC instantiation (fpga_top_boot_master.vh) enables it
    // with AXI_VRAM_BASE/AXI_VRAM_SIZE. Runs as a second pass AFTER the
    // main ZERO_BYTES pass completes, reusing the same ST_ZERO_AW/W/B
    // burst-write states with a different (base, length) pair -- see
    // zero_pass_vram_q below. VRAM_ZERO_BYTES must be a whole number of
    // (ZERO_AWLEN+1)*4-byte bursts, same constraint as ZERO_BYTES.
    parameter [31:0] VRAM_ZERO_BASE  = 32'd0,
    parameter [31:0] VRAM_ZERO_BYTES = 32'd0,

    // AXI word-writer FIFO depth, log2. Must be declared here (not as a
    // body localparam) so MIRROR_LOW_RAM's instantiation can override
    // it -- see the WORD_FIFO_LOG2 comment at its use site below.
    parameter        WORD_FIFO_LOG2P = 5
) (
    input  wire        clk,
    input  wire        rst,

    // ── Diagnostic overrides (default 0 = current production behavior:
    // send CMD59, never engage HS mode) — added 2026-07-23 alongside the
    // standalone fpga_top_sdmin SD/CRC test harness so ONE bitstream can
    // exercise multiple init-sequence hypotheses via VIO without a
    // resynth per experiment. Tied to 1'b0 in the real SoC
    // (fpga_top_boot_master.vh); only the sdmin harness drives these
    // from VIO probe_out bits.
    //   cfg_skip_cmd59: skip CMD59 entirely, unconditionally force
    //                   sd_crc_enabled=1 instead (tests whether CMD59
    //                   itself — not the CRC math — is implicated,
    //                   since a real SD card likely computes/sends a
    //                   genuine data-block CRC16 on reads regardless of
    //                   the CRC_ON_OFF setting; see ZipCPU sdspi's
    //                   bench/cpp/sdspisim.cpp, which always computes
    //                   real CRC16 with no on/off gating at all).
    //   cfg_force_hs:   allow spi_hs_mode to engage when the card
    //                   accepts CMD6 (reverts the 9521f93 HS-disable
    //                   fix for A/B testing against the same card).
    input  wire        cfg_skip_cmd59,
    input  wire        cfg_force_hs,

    // ── sd_spi byte interface (via sd_spi_mux "boot" port) ───────────
    output reg         spi_cmd_valid,
    input  wire        spi_cmd_ready,
    output reg  [7:0]  spi_cmd_data,
    input  wire        spi_rsp_valid,
    input  wire [7:0]  spi_rsp_data,
    output reg         spi_cs_n,
    output reg         spi_fast_mode,
    output reg         spi_hs_mode,

    // ── AXI4-MM write master (AW/W/B channels) ──────────────────────
    output reg  [3:0]  m_axi_awid,
    output reg  [31:0] m_axi_awaddr,
    output reg  [7:0]  m_axi_awlen,
    output reg  [2:0]  m_axi_awsize,
    output reg  [1:0]  m_axi_awburst,
    output reg         m_axi_awvalid,
    input  wire        m_axi_awready,

    output reg  [31:0] m_axi_wdata,
    output reg  [3:0]  m_axi_wstrb,
    output reg         m_axi_wlast,
    output reg         m_axi_wvalid,
    input  wire        m_axi_wready,

    input  wire [3:0]  m_axi_bid,
    input  wire [1:0]  m_axi_bresp,
    input  wire        m_axi_bvalid,
    output reg         m_axi_bready,

    // ── Status ───────────────────────────────────────────────────────
    // rom_loading is driven through a dedicated registered broadcast stage
    // (rom_loading_q) below.  Pre-broadcast value lives in rom_loading_int
    // and is set/cleared by the main FSM exactly as before.  The extra FF
    // exists solely to give post-place a clean FDRE driver for the high-
    // fanout boot/reset gate (was a LUT5, slack -7.18 ns at fanout 1390).
    output wire        rom_loading,
    output reg         rom_loaded,
    output reg         error,

    // 1 once CMD59 (CRC_ON_OFF) has been accepted by the card during
    // init — the caller (fpga_top_sd.vh) wires this into BOTH sd_ctrl
    // instances' crc_check_en (this module's own, for the CMD18 ROM
    // bulk-load, AND the separate runtime sd_ctrl_scsi instance) since
    // the card's CRC-on-off setting is a whole-session, one-time-set
    // property, not a per-command one.  Stays 0 (backward-compatible
    // legacy behaviour on both consumers) if the card doesn't support
    // or accept CMD59.
    output reg         sd_crc_enabled,

    // Total addressable sectors on the SD card, decoded from the CSD
    // (CMD9) during init.  The SCSI target's reported capacity must be
    // derived from THIS minus the reserved ROM area, never hardcoded:
    // 2026-07-28 the provisioned 700 MB disk image overran a hardcoded
    // 512 MB DISK_NUM_LBAS, leaving 188 MB of the HFS volume (including
    // the Alternate MDB, which lives in the volume's second-to-last
    // block) permanently unreachable.  Reads of those blocks simply
    // never completed, which presents as a File Manager / Device Manager
    // hang and looks exactly like a CPU bug.
    //
    // Stays at CARD_LBAS_DEFAULT if the card does not answer CMD9 — the
    // CSD read is deliberately NON-FATAL so an unsupported or flaky CMD9
    // degrades to the old fixed behaviour instead of failing the boot.
    output reg  [31:0] card_num_lbas,

    // ── Debug taps ───────────────────────────────────────────────────
    output wire [5:0]  dbg_st,
    output wire [3:0]  dbg_cur_cmd,
    output wire [7:0]  dbg_last_r1,
    output wire [7:0]  dbg_last_rx,
    output wire [15:0] dbg_acmd41_try,
    output wire [23:0] dbg_rsp_count,
    output wire [15:0] dbg_sector,
    output wire        dbg_is_sdhc,     // OCR-decoded card class (1=SDHC/SDXC)
    output wire [7:0]  dbg_ocr0,        // raw OCR byte 0 (for board diag)
    output wire [2:0]  dbg_err_cause,   // 0=none,1=CMD0,2=ACMD41 max,
                                          // 3=CMD17/18,4=unknown,
                                          // 5=R1 timeout,6=token timeout,
                                          // 7=BRESP err
    // Whole-run CMD18 retry count (0..CTRL_RETRY_MAX) — added alongside
    // the retry loop itself so a real-HW CRC failure can be diagnosed:
    // does err_cause=3 latch after 0 retries (first attempt, no retry
    // ever ran) or after CTRL_RETRY_MAX (every attempt failed the same
    // way, pointing at something systematic rather than a one-off
    // transient bit error).
    output wire [3:0]  dbg_ctrl_retry,
    // Raw computed-vs-received CRC16 for the failing block (see
    // sd_ctrl.v's dbg_rd_crc_calc/dbg_rd_crc_recv comment) — lets a
    // stuck boot distinguish "card sends a genuine but mismatched CRC"
    // from "card sends a fixed/bogus placeholder regardless of
    // CRC_ON_OFF" (the latter would read back the SAME dbg_rd_crc_recv
    // value across independent boots regardless of actual data content).
    output wire [15:0] dbg_rd_crc_calc,
    output wire [15:0] dbg_rd_crc_recv,
    // sd_ctrl's OWN cur_cmd/lba_lat — named distinctly from this
    // module's own dbg_cur_cmd (its init-phase command tracker, a
    // DIFFERENT register) to avoid confusion between the two.
    output wire [3:0]  dbg_sdctrl_cur_cmd,
    output wire [31:0] dbg_sdctrl_lba_lat,
    output wire [7:0]  dbg_sdctrl_last_crc7_sent,
    output wire [7:0]  dbg_sdctrl_last_poll_cnt,
    // sd_ctrl's OWN detailed error code (ERR_R1_BAD/ERR_R1_TO/ERR_TOK_TO/
    // ERR_DR_BAD/ERR_BUSY_TO/ERR_UNK_CMD/ERR_CRC_BAD — see sd_ctrl.v's
    // localparams) — boot_fsm's own dbg_err_cause=3 only means "sd_ctrl
    // reported SOME error", not specifically a CRC mismatch. Needed to
    // distinguish those cases on the sdmin test harness.
    output wire [3:0]  dbg_ctrl_err_cause,
    output wire [7:0]  dbg_ctrl_last_real_r1,

    // Per-attempt outcome log (2026-07-24): dbg_ctrl_retry/dbg_ctrl_err_cause
    // above only expose the FINAL, frozen state after either a success or
    // full CTRL_RETRY_MAX exhaustion — they cannot show which of the up to
    // 6 whole-run CMD18 attempts (indices 0..CTRL_RETRY_MAX) succeeded or
    // failed, or distinguish "attempt 3 succeeded, then something after it
    // still triggered more retries" from "every attempt failed the same
    // way". Each attempt_log entry is {outcome[3:0], sector[15:0]}: outcome
    // is the sd_ctrl err_cause (0=none,1=R1_BAD,2=R1_TO,3=TOK_TO,4=DR_BAD,
    // 5=BUSY_TO,6=UNK_CMD,7=CRC_BAD) on failure, or 4'hF as a success
    // sentinel; sector is the sector count reached (0 if failure was
    // pre-transfer) when ctrl_done fired for that attempt. Indexed by the
    // attempt's own ctrl_retry value at completion time.
    output wire [19:0] dbg_attempt_log0,
    output wire [19:0] dbg_attempt_log1,
    output wire [19:0] dbg_attempt_log2,
    output wire [19:0] dbg_attempt_log3,
    output wire [19:0] dbg_attempt_log4,
    output wire [19:0] dbg_attempt_log5,

    // ── RAM pre-zero pass: runtime enable ─────────────────────────────
    // ZERO_BYTES is the compile-time SIZE of the pass; this is the
    // per-boot decision to RUN it.  Low = skip straight to ST_DONE.
    //
    // Why it is runtime and not just the parameter: the pass costs a
    // MEASURED 4.06 s of a 4.06 s boot (5/5 reset cycles, +/-2 ms, zero
    // CRC retries) because ZERO_BYTES is the full 256 MiB RAM window.
    // That price buys "a reset presents as a genuine cold boot", which is
    // worth paying on a real cold reset and pure waste on the warm resets
    // used to iterate.  fpga_top drives this from a cold/warm flag that
    // survives the JTAG debug-full reset but not a platform reset.
    //
    // Tie high to get the historical always-zero behaviour.
    input  wire        zero_en
);

    // NUM_SECTORS feeds sd_ctrl.block_count[15:0], dbg_sector[15:0], and
    // the {sector, 9'b0} AXI address offset below.  Catch accidental
    // widening at elaboration instead of wrapping the boot copy.
    localparam [31:0] SD_RESERVED_LBAS = 32'd8192;
    generate
        if (NUM_SECTORS == 32'd0) begin : g_bad_num_sectors_zero
            initial $error("boot_fsm NUM_SECTORS must be non-zero");
        end
        if (NUM_SECTORS > SD_RESERVED_LBAS) begin : g_bad_num_sectors_window
            initial $error("boot_fsm NUM_SECTORS must fit within the 4 MiB reserved SD window");
        end
        if (NUM_SECTORS > 32'd65535) begin : g_bad_num_sectors_width
            initial $error("boot_fsm NUM_SECTORS must fit the 16-bit sector path");
        end
        // The zero pass retires whole bursts and stops as soon as the NEXT
        // burst would run past ZERO_BYTES, so a ZERO_BYTES that is not a
        // whole number of bursts would silently leave the tail unzeroed.
        // Catch it at elaboration rather than on a Sad Mac.
        if ((ZERO_BYTES != 32'd0) &&
            ((ZERO_BYTES % ((32'd1 + {24'd0, ZERO_AWLEN}) * 32'd4)) != 32'd0))
        begin : g_bad_zero_geometry
            initial $error("boot_fsm ZERO_BYTES must be a whole number of (ZERO_AWLEN+1)*4-byte bursts");
        end
        // Same geometry constraint as ZERO_BYTES, for the VRAM pass.
        if ((VRAM_ZERO_BYTES != 32'd0) &&
            ((VRAM_ZERO_BYTES % ((32'd1 + {24'd0, ZERO_AWLEN}) * 32'd4)) != 32'd0))
        begin : g_bad_vram_zero_geometry
            initial $error("boot_fsm VRAM_ZERO_BYTES must be a whole number of (ZERO_AWLEN+1)*4-byte bursts");
        end
    endgenerate

    // ──────────────────────────────────────────────────────────────────
    // Byte-level SPI sub-FSM (used for init only — data transport uses
    // the sub-FSM inside sd_ctrl).
    // ──────────────────────────────────────────────────────────────────
    localparam [1:0]
        BS_IDLE = 2'd0,
        BS_SEND = 2'd1,
        BS_WAIT = 2'd2;

    reg  [1:0] bst;
    reg        bgo;
    reg  [7:0] bt;
    reg        bdone;
    reg  [7:0] br;

    // These are this module's own SPI byte handshake wires (pre-mux).
    reg        init_spi_cmd_valid;
    reg  [7:0] init_spi_cmd_data;

    always @(posedge clk) begin
        if (rst) begin
            bst                 <= BS_IDLE;
            init_spi_cmd_valid  <= 1'b0;
            bdone               <= 1'b0;
            br                  <= 8'h00;
            init_spi_cmd_data   <= 8'hFF;
        end else begin
            bdone <= 1'b0;
            case (bst)
                BS_IDLE: begin
                    init_spi_cmd_valid <= 1'b0;
                    if (bgo) begin
                        init_spi_cmd_data  <= bt;
                        init_spi_cmd_valid <= 1'b1;
                        bst                <= BS_SEND;
                    end
                end
                BS_SEND: begin
                    if (spi_cmd_ready) begin
                        init_spi_cmd_valid <= 1'b0;
                        bst                <= BS_WAIT;
                    end
                end
                BS_WAIT: begin
                    if (spi_rsp_valid) begin
                        br    <= spi_rsp_data;
                        bdone <= 1'b1;
                        bst   <= BS_IDLE;
                    end
                end
                default: bst <= BS_IDLE;
            endcase
        end
    end

    // ──────────────────────────────────────────────────────────────────
    // Main FSM (init + hand-off to sd_ctrl for the CMD18 data pump)
    // ──────────────────────────────────────────────────────────────────
    localparam [5:0]
        ST_RESET       = 6'd0,
        ST_DUMMY_SEND  = 6'd1,
        ST_DUMMY_WAIT  = 6'd2,
        ST_FRAME_SEND  = 6'd3,
        ST_FRAME_WAIT  = 6'd4,
        ST_R1_SEND     = 6'd5,
        ST_R1_WAIT     = 6'd6,
        ST_ECHO_SEND   = 6'd7,
        ST_ECHO_WAIT   = 6'd8,
        ST_OCR_SEND    = 6'd9,
        ST_OCR_WAIT    = 6'd10,
        ST_CMD6_TOK_SEND  = 6'd11,
        ST_CMD6_TOK_WAIT  = 6'd12,
        ST_CMD6_DATA_SEND = 6'd13,
        ST_CMD6_DATA_WAIT = 6'd14,
        ST_CMD6_CRC_SEND  = 6'd15,
        ST_CMD6_CRC_WAIT  = 6'd16,
        ST_CTRL_KICK   = 6'd17,   // fire sd_ctrl.go = 1
        ST_CTRL_RUN    = 6'd18,   // wait for sd_ctrl.done, stream bytes
        ST_AXI_B_WAIT  = 6'd19,   // drain final AXI burst after ctrl done
        // ── RAM pre-zero pass (deterministic cold boot) ──────────────────
        // After ROM copy completes, zero the OS-visible RAM region so the
        // Q700 ROM's RAM-probe stripe-pattern reads are deterministic across
        // power cycles.  Without this, residual DRAM contents differ per
        // cold boot → ROM probes interpret different stripe data → boot
        // path forks non-deterministically (catastrophic in some boots,
        // clean in others — observed during HW bisection 2026-05-03).
        // Zeros 0x0000_0000..ZERO_BYTES-1 via AWLEN=15 (16-beat) bursts.
        ST_ZERO_AW     = 6'd22,   // launch a zero-write burst of ZERO_AWLEN+1 beats
        ST_ZERO_W      = 6'd23,   // stream ZERO_AWLEN+1 zero W beats
        ST_ZERO_B      = 6'd24,   // wait for B response, advance addr
        ST_DONE        = 6'd20,
        ST_ERROR       = 6'd21,
        // Drain in-flight AXI writes from an aborted CMD18 attempt before
        // restarting a whole-run retry (ST_CTRL_KICK) — same gate as
        // ST_AXI_B_WAIT, but loops back into a fresh attempt instead of
        // continuing to the zero-pass/done exit.  Without this, a stale
        // write still queued from the failed attempt could land in DDR
        // AFTER the retry's write to the same address, silently
        // resurrecting the bad data the retry was meant to replace.
        ST_CTRL_RETRY_DRAIN = 6'd25,
        // ── CMD9 / CSD read (0xFE token + 16 CSD bytes + 2 CRC) ──────────
        // Same data-block shape as the CMD6 status read below, just 16
        // bytes instead of 64.  Runs once, right after CMD58/OCR, where
        // the card is fully initialised but we have not yet touched the
        // delicate CMD59/CMD6 ordering.
        ST_CSD_TOK_SEND   = 6'd26,
        ST_CSD_TOK_WAIT   = 6'd27,
        ST_CSD_DATA_SEND  = 6'd28,
        ST_CSD_DATA_WAIT  = 6'd29,
        ST_CSD_CRC_SEND   = 6'd30,
        ST_CSD_CRC_WAIT   = 6'd31;

    // Commands we send during init
    localparam [3:0]
        C_NONE   = 4'd0,
        C_CMD0   = 4'd1,
        C_CMD8   = 4'd2,
        C_CMD55  = 4'd3,
        C_ACMD41 = 4'd4,
        C_CMD58  = 4'd5,
        C_CMD6   = 4'd6,
        // CMD59 (CRC_ON_OFF, arg bit0=1) — enables the card's data-block
        // CRC16, which sd_ctrl.v's crc_check_en gate then relies on to
        // validate reads (and to make sending a real, non-dummy CRC16 on
        // writes meaningful).  Sent once here, after CMD58/OCR and before
        // CMD6/HS-switch; the setting persists for the card's whole
        // power-on session, covering BOTH this module's own CMD18 ROM
        // bulk-load AND the separate runtime sd_ctrl_scsi instance that
        // takes over the SPI bus afterward (see fpga_top_sd.vh) — no
        // second CMD59 is needed once this one lands.
        C_CMD59  = 4'd7,
        // CMD9 (SEND_CSD) — returns the 16-byte Card-Specific Data block
        // from which the card's true sector count is decoded.  Optional:
        // a card that refuses it leaves card_num_lbas at its default.
        C_CMD9   = 4'd8;

    reg [5:0]  st;
    reg [3:0]  cur_cmd;
    reg [2:0]  frame_idx;
    reg [9:0]  byte_idx;
    reg [19:0] poll_cnt;
    reg [19:0] token_cnt;
    reg [15:0] acmd41_try;
    reg [15:0] sector;
    // CMD0 retry budget (cold-boot flakiness mitigation).
    //
    // SD-SPI bring-up reality (audited 2026-05-07): individual cards take
    // a variable number of attempts before they recognise CMD0 cleanly.
    // Some return 0xFF persistently for a few CMD0 frames after power-on
    // (R1 timeout); others return a non-0x01 R1 (e.g. 0x05 illegal-cmd
    // if internal state hasn't fully settled).  The previous code went
    // straight to ERROR on either failure, which produced the observed
    // "1-in-2 cold boot fails" behaviour: the card occasionally needed
    // one or two more CMD0 frames to land in idle state.
    //
    // Fix: retry CMD0 up to CMD0_RETRY_MAX times.  Each retry replays the
    // 10-byte dummy prelude with CS deasserted (giving the card a fresh
    // recognition window per the canonical SD-SPI init recipe used by
    // every open-source SD driver), then re-asserts CS and re-frames CMD0.
    // Both R1-timeout and R1!=0x01 paths feed the same retry counter so
    // either silent-card or wrong-R1 flakes are absorbed.  ACMD41 already
    // had its own large retry budget; CMD0 didn't, and it should.
    localparam [3:0] CMD0_RETRY_MAX = 4'd8;
    reg  [3:0] cmd0_try;
    // Whole-transfer CMD18 retry budget.  CMD18 cannot resume mid-stream
    // (see sd_ctrl.v's ERR_CRC_BAD abort-via-CMD12 comment), so a CRC
    // failure anywhere in the one-shot ROM bulk-load reruns the ENTIRE
    // read from sector 0 rather than failing boot outright over a single
    // bad block — idempotent, since it just overwrites the same DDR
    // range with a fresh attempt.  Matches CMD0_RETRY_MAX's philosophy:
    // real SD/SPI links occasionally flake, don't treat one bad frame as
    // fatal on the highest-stakes path in the design.
    localparam [3:0] CTRL_RETRY_MAX = 4'd5;
    reg  [3:0] ctrl_retry;
    // Per-attempt outcome log — see dbg_attempt_log0..5 port comment above.
    reg [19:0] attempt_log [0:5];
    // ── RAM pre-zero pass state ──────────────────────────────────────
    // zero_addr is the byte address of the burst currently in flight;
    // zero_beat counts 32-bit W beats within it.  Done when the NEXT burst
    // would run past ZERO_BYTES (the elaboration guard above makes that an
    // exact fit).
    //
    // History, because the comment here has been wrong twice:
    //   * The original loop issued ONE beat per AW (ZERO_AWLEN = 0) and the
    //     comment blamed axi_narrow_to_wide for "not handling multi-beat
    //     bursts".  The adapter is not the reason: ST_ZERO_W hardcoded
    //     WLAST on its first beat, so raising ZERO_AWLEN advertised AWLEN=N
    //     and then sent one beat with WLAST -- an AXI violation that
    //     strands the write.  That is the likeliest cause of the 2026-05-03
    //     KU5P hang the adapter was blamed for.
    //   * 2026-08-15 fixed the beat counter and raised this to 15.
    //   * 2026-08-19 (this change) raised it to the AXI4 maximum and made
    //     ST_ZERO_W stream beats BACK TO BACK.  It did not: the state
    //     re-armed WVALID only from `if (!m_axi_wvalid)`, which cannot be
    //     true on the handshake cycle (the deassert above is non-blocking),
    //     so every beat cost TWO cycles.  A 16-beat burst therefore ran at
    //     2.19 cycles/word, not the ~1.06 the burst shape implies.
    //
    // Why this matters at all: ZERO_BYTES is the full 256 MiB RAM window
    // (fpga_top_boot_master.vh) so a platform reset presents as a genuine
    // cold boot.  Measured on HW 2026-08-19, the pass was 4.06 s of a
    // 4.06 s boot -- the SD ROM copy is only ~0.13-0.34 s of it.
    //
    // But DO NOT expect burst length to fix that, and do not re-derive the
    // conclusion the hard way.  Measured 2026-08-19 on the real downstream
    // chain (axi_narrow_to_wide -> l2c -> async bridge -> MIG bridge ->
    // sim MIG, DDR read latency 40), narrow-side cycles per 32-bit word:
    //
    //     AWLEN         15      63     255      | L2C bypassed
    //     cycles/word  5.19    5.05    5.01     | 1.25 (the n2w ceiling)
    //
    // With L2C_ENABLE=1 (the shipping default) every write miss in
    // l2c_ctrl is a READ-ALLOCATE, so zeroing RAM drags each 64 B line in
    // from DDR before overwriting all of it, and that fill -- not the AW/B
    // round trip -- is the cost.  It is per LINE, so it does not amortise
    // over a longer burst.  The lever that would actually move this is a
    // full-line-write no-fetch path in l2c_ctrl / l2c_mshr.  Everything
    // this module can do is worth ~3% until that exists; it is done here
    // because it is free and because it is what makes that fix pay off.
    //
    // Burst geometry: (ZERO_AWLEN+1) x 4 bytes per burst, and zero_addr
    // advances by exactly that, so every burst is naturally aligned to its
    // own size.  At the 255 default that is a 1 KiB-aligned 1 KiB burst,
    // which can never cross the AXI 4 KiB boundary.  zero_beat is 9 bits,
    // enough for 0..255.
    // Bytes retired per zero burst -- always in step with ZERO_AWLEN.
    localparam [31:0] ZERO_BURST_BYTES = (32'd1 + {24'd0, ZERO_AWLEN}) * 32'd4;
    reg [31:0] zero_addr;
    reg [8:0]  zero_beat;
    // 0 = zeroing the main ZERO_BYTES range (or that pass is done and
    // there's no VRAM pass to run); 1 = zeroing VRAM_ZERO_BASE range.
    // ST_ZERO_B is the only place this ever changes.
    reg        zero_pass_vram_q;
    wire [31:0] zero_pass_end_c =
        zero_pass_vram_q ? (VRAM_ZERO_BASE + VRAM_ZERO_BYTES) : ZERO_BYTES;
    // Beat index of the W beat being armed THIS cycle: the current one when
    // no handshake is retiring, the next one when the state is streaming
    // back to back.  Drives WLAST so the last beat is flagged exactly once.
    wire       zero_w_hs    = m_axi_wvalid && m_axi_wready;
    wire [8:0] zero_beat_nx = zero_w_hs ? (zero_beat + 9'd1) : zero_beat;
    reg        is_sdhc;
    reg [7:0]  ocr0;

    // ── CSD (CMD9) capture + capacity decode ─────────────────────────────
    // csd[0] holds CSD bits 127:120, csd[n] holds bits (127-8n):(120-8n).
    reg [7:0]  csd [0:15];
    reg [4:0]  csd_idx;

    // CSD_STRUCTURE (bits 127:126) selects the layout:
    //   0 = v1.0 (SDSC): capacity = (C_SIZE+1) * 2^(C_SIZE_MULT+2) * 2^READ_BL_LEN
    //   1 = v2.0 (SDHC/SDXC): capacity = (C_SIZE+1) * 512 KB
    // Everything below is expressed in 512-byte sectors.
    wire [1:0]  csd_structure = csd[0][7:6];

    // v2.0: C_SIZE = bits 69:48 (22 bits) -> sectors = (C_SIZE+1) * 1024
    wire [21:0] csd_c_size_v2 = {csd[7][5:0], csd[8], csd[9]};
    wire [31:0] lbas_v2       = ({10'd0, csd_c_size_v2} + 32'd1) << 10;

    // v1.0: C_SIZE = bits 73:62 (12 bits), C_SIZE_MULT = bits 49:47 (3),
    //       READ_BL_LEN = bits 83:80 (4).  sectors =
    //       (C_SIZE+1) << (C_SIZE_MULT + 2 + READ_BL_LEN - 9)
    wire [11:0] csd_c_size_v1 = {csd[6][1:0], csd[7], csd[8][7:6]};
    wire [2:0]  csd_c_mult_v1 = {csd[9][1:0], csd[10][7]};
    wire [3:0]  csd_rbl_v1    = csd[5][3:0];
    wire [5:0]  v1_shift      = {3'd0, csd_c_mult_v1} + {2'd0, csd_rbl_v1} - 6'd7;
    wire [31:0] lbas_v1       = ({20'd0, csd_c_size_v1} + 32'd1) << v1_shift;

    wire [31:0] csd_num_lbas  = (csd_structure == 2'd1) ? lbas_v2 :
                                (csd_structure == 2'd0) ? lbas_v1 :
                                                          CARD_LBAS_DEFAULT;

    // Sanity gate: a decode that lands below the reserved ROM area (or at
    // zero) is not believable — fall back rather than reporting a disk so
    // small that the SCSI target would reject every OS read.  Silently
    // trusting a garbage CSD would reproduce exactly the failure mode this
    // whole change exists to prevent.
    wire        csd_plausible = (csd_num_lbas > CARD_LBAS_MIN);

    // AXI burst accumulator: pack 4 bytes big-endian, fire W beat.
    reg [7:0]  pack_b0, pack_b1, pack_b2;
    reg [1:0]  pack_phase;
    reg [9:0]  sec_byte_idx;     // 0..511 within current sector being streamed

    // sd_ctrl handshake
    reg         ctrl_go;
    wire        ctrl_busy;
    wire        ctrl_done;
    wire        ctrl_error;
    wire [3:0]  ctrl_err_cause;
    wire        ctrl_rd_valid;
    wire [7:0]  ctrl_rd_data;
    wire [15:0] ctrl_dbg_rd_crc_calc;
    wire [15:0] ctrl_dbg_rd_crc_recv;
    wire [7:0]  ctrl_dbg_last_real_r1;
    wire [3:0]  ctrl_dbg_cur_cmd;
    wire [31:0] ctrl_dbg_lba_lat;
    wire [7:0]  ctrl_dbg_last_crc7_sent;
    wire [7:0]  ctrl_dbg_last_poll_cnt;

    // sd_ctrl → SPI path
    wire        ctrl_spi_cmd_valid;
    wire [7:0]  ctrl_spi_cmd_data;

    // Debug
    reg [7:0]  last_r1;
    reg [7:0]  last_rx;
    reg [23:0] rsp_count;
    reg [2:0]  err_cause;

    // Internal "loading" state — set/cleared by the main FSM.  Broadcast
    // to the rest of the SoC through rom_loading_q (one extra cycle of
    // latency, irrelevant at ms-scale boot transitions).
    reg        rom_loading_int;
    // KEEP + MAX_FANOUT force a real FDRE at the broadcast point and let
    // the placer replicate the FF instead of fanning a single net out to
    // ~1.4k sinks.  Pre-fix synth had collapsed the FDRE into a LUT5 that
    // recomputed the gate combinationally each cycle — see fmax autopsy
    // 2026-04-25 §"rom_loading_reg_0[0]".
    (* keep = "true", max_fanout = 100 *) reg rom_loading_q;
    assign rom_loading = rom_loading_q;

    always @(posedge clk) begin
        if (rst) rom_loading_q <= 1'b0;
        else     rom_loading_q <= rom_loading_int;
    end

    assign dbg_st         = st;
    assign dbg_cur_cmd    = cur_cmd;
    assign dbg_last_r1    = last_r1;
    assign dbg_last_rx    = last_rx;
    assign dbg_acmd41_try = acmd41_try;
    assign dbg_rsp_count  = rsp_count;
    assign dbg_sector     = sector;
    // Expose OCR-decoded SDHC class so the bring-up VIO probe / cold-boot
    // tb can read back the card class without re-running CMD58.  Also
    // gives synthesis a real consumer for is_sdhc / ocr0 so they don't
    // get dead-code-stripped (Vivado [Synth 8-6014]).  See task #244.
    assign dbg_is_sdhc    = is_sdhc;
    assign dbg_ocr0       = ocr0;
    assign dbg_err_cause  = err_cause;
    assign dbg_ctrl_retry = ctrl_retry;
    assign dbg_rd_crc_calc = ctrl_dbg_rd_crc_calc;
    assign dbg_rd_crc_recv = ctrl_dbg_rd_crc_recv;
    assign dbg_ctrl_err_cause = ctrl_err_cause;
    assign dbg_ctrl_last_real_r1 = ctrl_dbg_last_real_r1;
    assign dbg_sdctrl_cur_cmd = ctrl_dbg_cur_cmd;
    assign dbg_sdctrl_lba_lat = ctrl_dbg_lba_lat;
    assign dbg_sdctrl_last_crc7_sent = ctrl_dbg_last_crc7_sent;
    assign dbg_sdctrl_last_poll_cnt = ctrl_dbg_last_poll_cnt;
    assign dbg_attempt_log0 = attempt_log[0];
    assign dbg_attempt_log1 = attempt_log[1];
    assign dbg_attempt_log2 = attempt_log[2];
    assign dbg_attempt_log3 = attempt_log[3];
    assign dbg_attempt_log4 = attempt_log[4];
    assign dbg_attempt_log5 = attempt_log[5];

    // 2026-07-24: real-HW investigation companion to l2c_ctrl.v's
    // dbg_l2c_write_snap — bundle boot_fsm's OWN AXI write-master
    // handshake signals so a debug_ila (ENABLE_ILA=1) capture shows both
    // ends of the very first AW/W handshake into the L2C-fronted path.
    (* MARK_DEBUG = "true" *) wire [4:0] dbg_bootfsm_axi_snap = {
        m_axi_awvalid, // [4]
        m_axi_awready, // [3]
        m_axi_wvalid,  // [2]
        m_axi_wready,  // [1]
        m_axi_bvalid   // [0]
    };

    // Whether we're in the data-transport phase (sd_ctrl owns the SPI
    // byte interface).
    wire data_phase = (st == ST_CTRL_KICK) || (st == ST_CTRL_RUN);

    // Mux SPI outputs: init FSM vs sd_ctrl.
    always @* begin
        if (data_phase) begin
            spi_cmd_valid = ctrl_spi_cmd_valid;
            spi_cmd_data  = ctrl_spi_cmd_data;
        end else begin
            spi_cmd_valid = init_spi_cmd_valid;
            spi_cmd_data  = init_spi_cmd_data;
        end
    end

    // sd_ctrl instance — assumes init has been done.  The caller drives
    // spi_cs_n externally; we also ensure it's asserted during the run.
    //
    // SDHC/SDXC cards take the lba arg as a BLOCK address (1 = bytes
    // 512..1023), SDSC cards take it as a BYTE address (1 = byte 1).  We
    // decoded the card class via CMD58/OCR earlier; scale START_SECTOR
    // accordingly so a non-zero base sector resolves to the right
    // physical bytes on either card class.  CMD18 then auto-increments
    // sector by sector internally for `block_count` blocks.
    wire [31:0] ctrl_lba_arg = is_sdhc ? START_SECTOR
                                       : (START_SECTOR << 9);
    sd_ctrl u_ctrl (
        .clk           (clk),
        .rst           (rst),

        .crc_check_en  (sd_crc_enabled),
        .cmd_type      (3'd2 /* CT_CMD18 */),
        .lba           (ctrl_lba_arg),
        .block_count   (NUM_SECTORS[15:0]),
        .go            (ctrl_go),

        .rd_valid      (ctrl_rd_valid),
        .rd_data       (ctrl_rd_data),
        .rd_ready      (1'b1),

        .wr_ready      (),
        .wr_valid      (),
        .wr_data       (8'h00),
        // wr_avail: this caller always has the byte staged before the
        // engine asks for it, so it takes the legacy unpaced write
        // stream unchanged.  See rtl/vhdd.vh / sd_ctrl.v header.
        .wr_avail      (1'b1),

        .spi_cmd_valid (ctrl_spi_cmd_valid),
        .spi_cmd_ready (spi_cmd_ready),
        .spi_cmd_data  (ctrl_spi_cmd_data),
        .spi_rsp_valid (spi_rsp_valid),
        .spi_rsp_data  (spi_rsp_data),

        .busy          (ctrl_busy),
        .done          (ctrl_done),
        .error         (ctrl_error),
        .err_cause     (ctrl_err_cause),

        .dbg_rd_crc_calc(ctrl_dbg_rd_crc_calc),
        .dbg_rd_crc_recv(ctrl_dbg_rd_crc_recv),
        .dbg_last_real_r1(ctrl_dbg_last_real_r1),
        .dbg_cur_cmd(ctrl_dbg_cur_cmd),
        .dbg_lba_lat(ctrl_dbg_lba_lat),
        .dbg_last_crc7_sent(ctrl_dbg_last_crc7_sent),
        .dbg_last_write_resp(),
        .dbg_block_idx(),
        .dbg_last_poll_cnt(ctrl_dbg_last_poll_cnt)
    );

    // AXI word writer.  sd_ctrl cannot be backpressured, so completed
    // 32-bit words sit in a small FIFO while the write channel retires one
    // single-beat transaction at a time through the first-light DDR path.
    // WORD_FIFO_LOG2P is a real parameter (not a localparam) so
    // MIRROR_LOW_RAM's instantiation can double it -- pushing two entries
    // per assembled word instead of one halves the margin against
    // word_fifo_full on an un-backpressurable source, so it gets double
    // the buffer to compensate. v1 (MIRROR_LOW_RAM=0, default depth)
    // keeps the exact resource usage and margin it always had.
    localparam WORD_FIFO_LOG2 = WORD_FIFO_LOG2P;
    localparam WORD_FIFO_DEPTH = (1 << WORD_FIFO_LOG2);
    localparam [WORD_FIFO_LOG2:0] WORD_FIFO_DEPTH_COUNT =
        (1 << WORD_FIFO_LOG2);

    wire [31:0] word_fifo_head_addr;
    wire [31:0] word_fifo_head_data;
    reg [WORD_FIFO_LOG2-1:0] word_fifo_wr_ptr;
    reg [WORD_FIFO_LOG2-1:0] word_fifo_rd_ptr;
    reg [WORD_FIFO_LOG2:0]   word_fifo_count;

    wire word_fifo_full  = (word_fifo_count == WORD_FIFO_DEPTH_COUNT);
    wire word_fifo_empty = (word_fifo_count == {(WORD_FIFO_LOG2+1){1'b0}});
    // Room for TWO more entries (MIRROR_LOW_RAM's dual-push case).
    wire word_fifo_room2 = (word_fifo_count <= (WORD_FIFO_DEPTH_COUNT - {{(WORD_FIFO_LOG2-1){1'b0}}, 2'd2}));
    wire word_fifo_push = !rst && st == ST_CTRL_RUN && ctrl_rd_valid &&
                          pack_phase == 2'd3 &&
                          (MIRROR_LOW_RAM ? word_fifo_room2 : !word_fifo_full);
    wire [31:0] word_fifo_offset = {7'd0, sector, 9'd0} +
                                   {22'd0, sec_byte_idx[9:2], 2'b00};
    // Mirrored writes always enqueue an even/odd pair with identical data.
    // Store that payload once, retaining the same number of queued AXI words.
    // The read pointer's low bit chooses ROM versus low-RAM destination.
    generate if (MIRROR_LOW_RAM) begin : g_mirror_fifo
        (* ram_style = "distributed" *) reg [63:0] words [0:WORD_FIFO_DEPTH/2-1];
        wire [63:0] head = words[word_fifo_rd_ptr[WORD_FIFO_LOG2-1:1]];
        always @(posedge clk) begin
            if (word_fifo_push)
                words[word_fifo_wr_ptr[WORD_FIFO_LOG2-1:1]] <=
                    {word_fifo_offset, pack_b0, pack_b1, pack_b2, ctrl_rd_data};
        end
        assign word_fifo_head_addr = head[63:32] +
                                    (word_fifo_rd_ptr[0] ? 32'd0 : ROM_BASE_ADDR);
        assign word_fifo_head_data = head[31:0];
    end else begin : g_plain_fifo
        (* ram_style = "distributed" *) reg [63:0] words [0:WORD_FIFO_DEPTH-1];
        wire [63:0] head = words[word_fifo_rd_ptr];
        always @(posedge clk) begin
            if (word_fifo_push)
                words[word_fifo_wr_ptr] <=
                    {word_fifo_offset, pack_b0, pack_b1, pack_b2, ctrl_rd_data};
        end
        assign word_fifo_head_addr = head[63:32] + ROM_BASE_ADDR;
        assign word_fifo_head_data = head[31:0];
    end endgenerate

    reg        burst_active;         // 1 between single-beat launch and B ack
    reg        burst_aw_fired;       // 1 once AW has handshaken
    reg [6:0]  burst_beats_done;     // bit0 means W has handshaken

    // err_cause code for "a write returned a non-OKAY BRESP".
    localparam [2:0] ERR_CAUSE_BRESP = 3'd7;

    // A write-response beat is being accepted THIS cycle and it carries
    // SLVERR/DECERR.  Used by the post-`case` override at the bottom of the
    // main FSM so the ST_ERROR transition wins over whatever normal-progress
    // next state the case body picked — see the comment there.
    wire axi_b_err = m_axi_bready && m_axi_bvalid && (m_axi_bresp != 2'b00);

    wire writer_can_launch = (st == ST_CTRL_RUN || st == ST_AXI_B_WAIT ||
                               st == ST_CTRL_RETRY_DRAIN) &&
                             !burst_active &&
                             !m_axi_awvalid &&
                             !m_axi_wvalid &&
                             !word_fifo_empty &&
                             !(st == ST_CTRL_RUN && ctrl_rd_valid && pack_phase == 2'd3) &&
                             !error;

    // Frame-byte LUT for init commands.
    reg [7:0] frame_byte;
    always @(*) begin
        case (cur_cmd)
            C_CMD0: case (frame_idx)
                3'd0:    frame_byte = 8'h40;
                3'd5:    frame_byte = 8'h95;
                default: frame_byte = 8'h00;
            endcase
            C_CMD8: case (frame_idx)
                3'd0:    frame_byte = 8'h48;
                3'd1:    frame_byte = 8'h00;
                3'd2:    frame_byte = 8'h00;
                3'd3:    frame_byte = 8'h01;
                3'd4:    frame_byte = 8'hAA;
                default: frame_byte = 8'h87;
            endcase
            C_CMD55: case (frame_idx)
                3'd0:    frame_byte = 8'h77;
                3'd5:    frame_byte = 8'h65;
                default: frame_byte = 8'h00;
            endcase
            C_ACMD41: case (frame_idx)
                3'd0:    frame_byte = 8'h69;
                3'd1:    frame_byte = 8'h40;
                3'd5:    frame_byte = 8'h77;
                default: frame_byte = 8'h00;
            endcase
            C_CMD58: case (frame_idx)
                3'd0:    frame_byte = 8'h7A;
                3'd5:    frame_byte = 8'hFD;
                default: frame_byte = 8'h00;
            endcase
            C_CMD9: case (frame_idx)
                3'd0:    frame_byte = 8'h49;   // 0x40 | 9, arg = 0
                // This card enforces command CRC7 unconditionally (see the
                // CMD59 note below), so send a real one: CRC7 of
                // (0x49,0x00,0x00,0x00,0x00) with the same poly
                // (x^7+x^3+1) sd_ctrl.v's crc7_frame() uses, <<1 | 1.
                // Cross-checked against the CMD0/CMD8/CMD59 constants
                // already in this table.
                3'd5:    frame_byte = 8'hAF;
                default: frame_byte = 8'h00;
            endcase
            C_CMD59: case (frame_idx)
                3'd0:    frame_byte = 8'h7B;   // 0x40 | 59
                3'd4:    frame_byte = 8'h01;   // arg = 0x00000001 (CRC ON)
                // Real-HW finding 2026-07-23: this card enforces command
                // CRC7 UNCONDITIONALLY, not only while CRC-mode is off —
                // the "CRC7 required only on CMD0/CMD8" relaxation this
                // dummy-01 placeholder assumed does not hold. Computed
                // for (0x7B,0x00,0x00,0x00,0x01) with the same CRC7
                // (poly x^7+x^3+1) sd_ctrl.v's crc7_frame() uses.
                3'd5:    frame_byte = 8'h83;
                default: frame_byte = 8'h00;   // arg[31:8] = 0
            endcase
            C_CMD6: case (frame_idx)
                3'd0:    frame_byte = 8'h46;
                // Real-HW finding 2026-07-24: byte1 bit7 is CMD6's own
                // MODE bit (0=check function only, 1=actually SWITCH).
                // This was 0x80 (MODE=1, real switch) — meaning even
                // after 9521f93 disabled OUR OWN clock from ever
                // engaging HS_HALF, we were still telling the CARD to
                // actually switch its internal state into HS function
                // group 1. That leaves the card believing it's in HS
                // mode while we keep clocking it at the old rate — a
                // genuine host/card state mismatch, and a strong
                // candidate for the intermittent-looking CMD18 R1
                // corruption seen after this point in the sequence
                // (0x08/0x09/0x04/0x0D — no coherent pattern, consistent
                // with the card's own logic being confused about what
                // timing to expect). Changed to 0x00 (MODE=0, check
                // only) so the card's state matches what we actually do
                // — never switch — regardless of what it reports it
                // supports.
                3'd1:    frame_byte = 8'h00;
                3'd2:    frame_byte = 8'hFF;
                3'd3:    frame_byte = 8'hFF;
                3'd4:    frame_byte = 8'hF1;
                // CRC7 recomputed for (0x46,0x00,0xFF,0xFF,0xF1).
                default: frame_byte = 8'h1F;
            endcase
            default: frame_byte = 8'hFF;
        endcase
    end

    // POLL_TIMEOUT counts SPI bytes received while polling for R1.  At the
    // 400 kHz init clock (SLOW_HALF=250 / core_clk=200 MHz) one SPI byte is
    // ~20 us, so POLL_TIMEOUT=4096 caps R1 poll latency at ~80 ms — within
    // SD-spec maximums and tolerant of slow cards.  Previously 1024 (~20
    // ms), which was tight for the slowest sd-spi cards on the bench.
    localparam [19:0] POLL_TIMEOUT  = 20'd4096;
    localparam [19:0] TOKEN_TIMEOUT = 20'd500000;
    localparam [15:0] ACMD41_MAX    = 16'd1000;

    always @(posedge clk) begin
        if (rst) begin
            st             <= ST_RESET;
            cur_cmd        <= C_NONE;
            frame_idx      <= 3'd0;
            byte_idx       <= 10'd0;
            poll_cnt       <= 20'd0;
            token_cnt      <= 20'd0;
            acmd41_try     <= 16'd0;
            cmd0_try       <= 4'd0;
            ctrl_retry     <= 4'd0;
            attempt_log[0] <= 20'd0;
            attempt_log[1] <= 20'd0;
            attempt_log[2] <= 20'd0;
            attempt_log[3] <= 20'd0;
            attempt_log[4] <= 20'd0;
            attempt_log[5] <= 20'd0;
            sector         <= 16'd0;
            sd_crc_enabled <= 1'b0;
            is_sdhc        <= 1'b0;
            ocr0           <= 8'h00;
            csd_idx        <= 5'd0;
            card_num_lbas  <= CARD_LBAS_DEFAULT;
            last_r1        <= 8'hFF;
            last_rx        <= 8'hFF;
            rsp_count      <= 24'd0;
            err_cause      <= 3'd0;

            bgo            <= 1'b0;
            bt             <= 8'hFF;

            spi_cs_n       <= 1'b1;
            spi_fast_mode  <= 1'b0;
            spi_hs_mode    <= 1'b0;

            pack_b0        <= 8'h00;
            pack_b1        <= 8'h00;
            pack_b2        <= 8'h00;
            pack_phase     <= 2'd0;
            sec_byte_idx   <= 10'd0;

            ctrl_go        <= 1'b0;

            m_axi_awid     <= AXI_ID;
            m_axi_awaddr   <= 32'd0;
            m_axi_awlen    <= 8'd0;
            m_axi_awsize   <= 3'd2;
            m_axi_awburst  <= 2'b01;
            m_axi_awvalid  <= 1'b0;
            m_axi_wdata    <= 32'd0;
            m_axi_wstrb    <= 4'b0000;
            m_axi_wlast    <= 1'b0;
            m_axi_wvalid   <= 1'b0;
            m_axi_bready   <= 1'b0;

            burst_active      <= 1'b0;
            burst_aw_fired    <= 1'b0;
            burst_beats_done  <= 7'd0;
            word_fifo_wr_ptr  <= {WORD_FIFO_LOG2{1'b0}};
            word_fifo_rd_ptr  <= {WORD_FIFO_LOG2{1'b0}};
            word_fifo_count   <= {1'b0, {WORD_FIFO_LOG2{1'b0}}};

            rom_loading_int <= 1'b0;
            rom_loaded     <= 1'b0;
            error          <= 1'b0;

            // RAM pre-zero pass — start at byte 0, beat 0.  The real
            // starting address (MIRROR_IMAGE_BYTES instead of 0, under
            // MIRROR_LOW_RAM) is set again at the ST_AXI_B_WAIT ->
            // ST_ZERO_AW transition below; this reset value only matters
            // if that transition is never reached (e.g. ZERO_BYTES=0).
            zero_addr      <= 32'd0;
            zero_beat      <= 9'd0;
            zero_pass_vram_q <= 1'b0;
        end else begin
            // Default pulses
            bgo     <= 1'b0;
            ctrl_go <= 1'b0;

            // Debug: every init-phase SPI byte received
            if (bdone) begin
                last_rx   <= spi_rsp_data;
                rsp_count <= rsp_count + 24'd1;
            end

            // AXI handshake deassertions
            if (m_axi_awvalid && m_axi_awready) begin
                m_axi_awvalid  <= 1'b0;
                burst_aw_fired <= 1'b1;
            end
            if (m_axi_wvalid  && m_axi_wready) begin
                m_axi_wvalid <= 1'b0;
                m_axi_wlast  <= 1'b0;
                m_axi_wstrb  <= 4'b0000;
                burst_beats_done <= 7'd1;
            end
            // Write-response bookkeeping only.  The non-OKAY BRESP -> ST_ERROR
            // transition deliberately does NOT live here: this block runs
            // BEFORE the `case (st)` below, so a state that assigns `st` on
            // the very same B beat would silently overwrite it
            // (last-assignment-wins).  See the override after `endcase`.
            if (m_axi_bready  && m_axi_bvalid) begin
                m_axi_bready     <= 1'b0;
                burst_active     <= 1'b0;
                burst_aw_fired   <= 1'b0;
                burst_beats_done <= 7'd0;
            end

            if (writer_can_launch) begin
                m_axi_awid    <= AXI_ID;
                m_axi_awaddr  <= word_fifo_head_addr;
                m_axi_awlen   <= 8'd0;
                m_axi_awsize  <= 3'd2;
                m_axi_awburst <= 2'b01;
                m_axi_awvalid <= 1'b1;
                m_axi_wdata   <= word_fifo_head_data;
                m_axi_wstrb   <= 4'b1111;
                m_axi_wlast   <= 1'b1;
                m_axi_wvalid  <= 1'b1;
                burst_active     <= 1'b1;
                burst_aw_fired   <= 1'b0;
                burst_beats_done <= 7'd0;
                word_fifo_rd_ptr <= word_fifo_rd_ptr +
                    {{(WORD_FIFO_LOG2-1){1'b0}}, 1'b1};
                word_fifo_count <= word_fifo_count -
                    {{WORD_FIFO_LOG2{1'b0}}, 1'b1};
            end

            case (st)
                // ────────── Power-up dummy clocks ──────────
                ST_RESET: begin
                    spi_cs_n     <= 1'b1;
                    cur_cmd      <= C_NONE;
                    byte_idx     <= 10'd0;
                    frame_idx    <= 3'd0;
                    rom_loading_int <= 1'b1;
                    st           <= ST_DUMMY_SEND;
                end

                ST_DUMMY_SEND: begin
                    if (bst == BS_IDLE) begin
                        bt  <= 8'hFF;
                        bgo <= 1'b1;
                        st  <= ST_DUMMY_WAIT;
                    end
                end
                ST_DUMMY_WAIT: begin
                    if (bdone) begin
                        if (byte_idx == 10'd9) begin
                            byte_idx  <= 10'd0;
                            spi_cs_n  <= 1'b0;
                            cur_cmd   <= C_CMD0;
                            frame_idx <= 3'd0;
                            st        <= ST_FRAME_SEND;
                        end else begin
                            byte_idx <= byte_idx + 10'd1;
                            st       <= ST_DUMMY_SEND;
                        end
                    end
                end

                // ────────── 6-byte command frame ──────────
                ST_FRAME_SEND: begin
                    if (bst == BS_IDLE) begin
                        bt  <= frame_byte;
                        bgo <= 1'b1;
                        st  <= ST_FRAME_WAIT;
                    end
                end
                ST_FRAME_WAIT: begin
                    if (bdone) begin
                        if (frame_idx == 3'd5) begin
                            frame_idx <= 3'd0;
                            poll_cnt  <= 20'd0;
                            st        <= ST_R1_SEND;
                        end else begin
                            frame_idx <= frame_idx + 3'd1;
                            st        <= ST_FRAME_SEND;
                        end
                    end
                end

                // ────────── R1 poll ──────────
                ST_R1_SEND: begin
                    if (bst == BS_IDLE) begin
                        bt  <= 8'hFF;
                        bgo <= 1'b1;
                        st  <= ST_R1_WAIT;
                    end
                end
                ST_R1_WAIT: begin
                    if (bdone) begin
                        if (br[7] == 1'b0) last_r1 <= br;
                        if (br[7] == 1'b0) begin
                            case (cur_cmd)
                                C_CMD0: begin
                                    if (br == 8'h01) begin
                                        cur_cmd   <= C_CMD8;
                                        frame_idx <= 3'd0;
                                        st        <= ST_FRAME_SEND;
                                    end else if (cmd0_try < CMD0_RETRY_MAX) begin
                                        // Wrong R1 (not 0x01).  Retry CMD0
                                        // with a fresh CS-deasserted dummy-
                                        // clock prelude — same recovery
                                        // recipe as the R1-timeout path.
                                        cmd0_try  <= cmd0_try + 4'd1;
                                        spi_cs_n  <= 1'b1;
                                        byte_idx  <= 10'd0;
                                        cur_cmd   <= C_NONE;
                                        frame_idx <= 3'd0;
                                        st        <= ST_DUMMY_SEND;
                                    end else begin
                                        error     <= 1'b1;
                                        err_cause <= 3'd1;
                                        st        <= ST_ERROR;
                                    end
                                end
                                C_CMD8: begin
                                    byte_idx <= 10'd0;
                                    st       <= ST_ECHO_SEND;
                                end
                                C_CMD55: begin
                                    cur_cmd   <= C_ACMD41;
                                    frame_idx <= 3'd0;
                                    st        <= ST_FRAME_SEND;
                                end
                                C_ACMD41: begin
                                    if (br == 8'h00) begin
                                        cur_cmd   <= C_CMD58;
                                        frame_idx <= 3'd0;
                                        st        <= ST_FRAME_SEND;
                                    end else if (acmd41_try >= ACMD41_MAX) begin
                                        error     <= 1'b1;
                                        err_cause <= 3'd2;
                                        st        <= ST_ERROR;
                                    end else begin
                                        acmd41_try <= acmd41_try + 16'd1;
                                        cur_cmd    <= C_CMD55;
                                        frame_idx  <= 3'd0;
                                        st         <= ST_FRAME_SEND;
                                    end
                                end
                                C_CMD58: begin
                                    byte_idx <= 10'd0;
                                    st       <= ST_OCR_SEND;
                                end
                                C_CMD9: begin
                                    if (br == 8'h00) begin
                                        // Accepted — read the 16-byte CSD
                                        // block (token + 16 data + 2 CRC).
                                        token_cnt <= 20'd0;
                                        csd_idx   <= 5'd0;
                                        byte_idx  <= 10'd0;
                                        st        <= ST_CSD_TOK_SEND;
                                    end else begin
                                        // Card refused CMD9 — keep the
                                        // default capacity and carry on.
                                        // Capacity discovery is an
                                        // optimisation, not boot-critical.
                                        if (cfg_skip_cmd59) begin
                                            sd_crc_enabled <= 1'b1;
                                            cur_cmd        <= C_CMD6;
                                        end else begin
                                            cur_cmd        <= C_CMD59;
                                        end
                                        frame_idx <= 3'd0;
                                        st        <= ST_FRAME_SEND;
                                    end
                                end
                                C_CMD59: begin
                                    // Accept whatever R1 the card gives us
                                    // (some older/odd cards don't support
                                    // CMD59 at all) — degrade gracefully:
                                    // only claim CRC is enabled on a clean
                                    // R1=0x00, and either way move on to
                                    // CMD6/HS-switch rather than failing
                                    // the whole boot over an optional
                                    // integrity feature.
                                    sd_crc_enabled <= (br == 8'h00);
                                    cur_cmd        <= C_CMD6;
                                    frame_idx      <= 3'd0;
                                    st             <= ST_FRAME_SEND;
                                end
                                C_CMD6: begin
                                    if (br == 8'h00) begin
                                        // Accepted — read 64 status bytes
                                        // (token + 64 data + 2 CRC) and
                                        // then go high-speed.
                                        token_cnt <= 20'd0;
                                        byte_idx  <= 10'd0;
                                        st        <= ST_CMD6_TOK_SEND;
                                    end else begin
                                        // Card rejected HS switch — stay
                                        // at 25 MHz and proceed.
                                        st <= ST_CTRL_KICK;
                                    end
                                end
                                default: begin
                                    error     <= 1'b1;
                                    err_cause <= 3'd4;
                                    st        <= ST_ERROR;
                                end
                            endcase
                        end else begin
                            if (poll_cnt >= POLL_TIMEOUT) begin
                                if (cur_cmd == C_CMD0 &&
                                    cmd0_try < CMD0_RETRY_MAX) begin
                                    // CMD0 silent-card retry: replay the
                                    // 10-byte dummy prelude with CS high,
                                    // then re-frame CMD0.  Some cards take
                                    // 2-3 attempts before they recognise
                                    // the first CMD0 after power-on.
                                    cmd0_try  <= cmd0_try + 4'd1;
                                    spi_cs_n  <= 1'b1;
                                    byte_idx  <= 10'd0;
                                    poll_cnt  <= 20'd0;
                                    cur_cmd   <= C_NONE;
                                    frame_idx <= 3'd0;
                                    st        <= ST_DUMMY_SEND;
                                end else if (cur_cmd == C_CMD9) begin
                                    // Silent card on CMD9: same graceful
                                    // degradation as CMD59 below — keep the
                                    // default capacity rather than failing
                                    // the boot over capacity discovery.
                                    poll_cnt  <= 20'd0;
                                    if (cfg_skip_cmd59) begin
                                        sd_crc_enabled <= 1'b1;
                                        cur_cmd        <= C_CMD6;
                                    end else begin
                                        cur_cmd        <= C_CMD59;
                                    end
                                    frame_idx <= 3'd0;
                                    st        <= ST_FRAME_SEND;
                                end else if (cur_cmd == C_CMD59) begin
                                    // CMD59 is optional-feature plumbing,
                                    // not a boot-critical command — a
                                    // silent card here (non-compliant or
                                    // unsupported CMD59) degrades to "CRC
                                    // checking stays off" rather than
                                    // failing the whole boot.
                                    sd_crc_enabled <= 1'b0;
                                    poll_cnt       <= 20'd0;
                                    cur_cmd        <= C_CMD6;
                                    frame_idx      <= 3'd0;
                                    st             <= ST_FRAME_SEND;
                                end else begin
                                    error     <= 1'b1;
                                    err_cause <= 3'd5;
                                    st        <= ST_ERROR;
                                end
                            end else begin
                                poll_cnt <= poll_cnt + 20'd1;
                                st       <= ST_R1_SEND;
                            end
                        end
                    end
                end

                // ────────── CMD8 4-byte echo ──────────
                ST_ECHO_SEND: begin
                    if (bst == BS_IDLE) begin
                        bt  <= 8'hFF;
                        bgo <= 1'b1;
                        st  <= ST_ECHO_WAIT;
                    end
                end
                ST_ECHO_WAIT: begin
                    if (bdone) begin
                        if (byte_idx == 10'd3) begin
                            byte_idx   <= 10'd0;
                            acmd41_try <= 16'd0;
                            cur_cmd    <= C_CMD55;
                            frame_idx  <= 3'd0;
                            st         <= ST_FRAME_SEND;
                        end else begin
                            byte_idx <= byte_idx + 10'd1;
                            st       <= ST_ECHO_SEND;
                        end
                    end
                end

                // ────────── CMD58 OCR ──────────
                ST_OCR_SEND: begin
                    if (bst == BS_IDLE) begin
                        bt  <= 8'hFF;
                        bgo <= 1'b1;
                        st  <= ST_OCR_WAIT;
                    end
                end
                ST_OCR_WAIT: begin
                    if (bdone) begin
                        if (byte_idx == 10'd0) ocr0 <= br;
                        if (byte_idx == 10'd3) begin
                            is_sdhc       <= ocr0[6];
                            spi_fast_mode <= 1'b1;
                            byte_idx      <= 10'd0;
                            // Real-HW experiment 2026-07-23: skip CMD59
                            // (CRC_ON_OFF) entirely and unconditionally
                            // validate CRC anyway. Reference: ZipCPU's
                            // sdspi bench/cpp/sdspisim.cpp always
                            // computes+sends a genuine data-block CRC16
                            // on every read with NO CRC-on/off gating at
                            // all (doesn't even implement CMD59) — real
                            // cards very plausibly always compute CRC on
                            // reads regardless of this toggle, and CMD59
                            // may only govern whether the CARD validates
                            // OUR write data. If the card sends genuine
                            // CRC regardless, we get the same validation
                            // benefit without whatever CMD59 might be
                            // doing to this specific card's behavior.
                            // This directly tests whether CMD59 itself
                            // (not the CRC math, already verified
                            // algorithm-correct against the same
                            // reference) is implicated in the deterministic
                            // sector-0 failure.
                            // Read the CSD (CMD9) here, before the delicate
                            // CMD59/CMD6 ordering below: the card is fully
                            // initialised at this point and CMD9 is a plain
                            // data-block read that does not change any card
                            // state.  The CSD tells us the card's real
                            // sector count, which the SCSI target's reported
                            // capacity is derived from — see card_num_lbas.
                            cur_cmd       <= C_CMD9;
                            frame_idx     <= 3'd0;
                            st            <= ST_FRAME_SEND;
                        end else begin
                            byte_idx <= byte_idx + 10'd1;
                            st       <= ST_OCR_SEND;
                        end
                    end
                end

                // ────────── CMD6 status block (0xFE + 64 bytes + CRC) ──
                // We discard the status and just move on to HS mode.
                // ────────── CMD9 CSD block (0xFE + 16 bytes + CRC) ──
                // Same shape as the CMD6 status read below.  Unlike CMD6
                // we KEEP the payload: it is the only place the card's
                // real sector count is available.
                ST_CSD_TOK_SEND: begin
                    if (bst == BS_IDLE) begin
                        bt  <= 8'hFF;
                        bgo <= 1'b1;
                        st  <= ST_CSD_TOK_WAIT;
                    end
                end
                ST_CSD_TOK_WAIT: begin
                    if (bdone) begin
                        if (br == 8'hFE) begin
                            csd_idx <= 5'd0;
                            st      <= ST_CSD_DATA_SEND;
                        end else if (token_cnt >= TOKEN_TIMEOUT) begin
                            // Non-fatal: keep the default capacity.
                            if (cfg_skip_cmd59) begin
                                sd_crc_enabled <= 1'b1;
                                cur_cmd        <= C_CMD6;
                            end else begin
                                cur_cmd        <= C_CMD59;
                            end
                            frame_idx <= 3'd0;
                            st        <= ST_FRAME_SEND;
                        end else begin
                            token_cnt <= token_cnt + 20'd1;
                            st        <= ST_CSD_TOK_SEND;
                        end
                    end
                end
                ST_CSD_DATA_SEND: begin
                    if (bst == BS_IDLE) begin
                        bt  <= 8'hFF;
                        bgo <= 1'b1;
                        st  <= ST_CSD_DATA_WAIT;
                    end
                end
                ST_CSD_DATA_WAIT: begin
                    if (bdone) begin
                        csd[csd_idx[3:0]] <= br;
                        if (csd_idx == 5'd15) begin
                            byte_idx <= 10'd0;
                            st       <= ST_CSD_CRC_SEND;
                        end else begin
                            csd_idx <= csd_idx + 5'd1;
                            st      <= ST_CSD_DATA_SEND;
                        end
                    end
                end
                ST_CSD_CRC_SEND: begin
                    if (bst == BS_IDLE) begin
                        bt  <= 8'hFF;
                        bgo <= 1'b1;
                        st  <= ST_CSD_CRC_WAIT;
                    end
                end
                ST_CSD_CRC_WAIT: begin
                    if (bdone) begin
                        if (byte_idx == 10'd1) begin
                            byte_idx <= 10'd0;
                            // Latch the decoded capacity, but only if it
                            // looks sane — a garbage CSD must not be
                            // trusted into the SCSI capacity report.
                            if (csd_plausible) card_num_lbas <= csd_num_lbas;
                            // Resume the normal init ordering.
                            if (cfg_skip_cmd59) begin
                                sd_crc_enabled <= 1'b1;
                                cur_cmd        <= C_CMD6;
                            end else begin
                                cur_cmd        <= C_CMD59;
                            end
                            frame_idx <= 3'd0;
                            st        <= ST_FRAME_SEND;
                        end else begin
                            byte_idx <= 10'd1;
                            st       <= ST_CSD_CRC_SEND;
                        end
                    end
                end

                ST_CMD6_TOK_SEND: begin
                    if (bst == BS_IDLE) begin
                        bt  <= 8'hFF;
                        bgo <= 1'b1;
                        st  <= ST_CMD6_TOK_WAIT;
                    end
                end
                ST_CMD6_TOK_WAIT: begin
                    if (bdone) begin
                        if (br == 8'hFE) begin
                            byte_idx <= 10'd0;
                            st       <= ST_CMD6_DATA_SEND;
                        end else if (token_cnt >= TOKEN_TIMEOUT) begin
                            error     <= 1'b1;
                            err_cause <= 3'd6;
                            st        <= ST_ERROR;
                        end else begin
                            token_cnt <= token_cnt + 20'd1;
                            st        <= ST_CMD6_TOK_SEND;
                        end
                    end
                end
                ST_CMD6_DATA_SEND: begin
                    if (bst == BS_IDLE) begin
                        bt  <= 8'hFF;
                        bgo <= 1'b1;
                        st  <= ST_CMD6_DATA_WAIT;
                    end
                end
                ST_CMD6_DATA_WAIT: begin
                    if (bdone) begin
                        if (byte_idx == 10'd63) begin
                            byte_idx <= 10'd0;
                            st       <= ST_CMD6_CRC_SEND;
                        end else begin
                            byte_idx <= byte_idx + 10'd1;
                            st       <= ST_CMD6_DATA_SEND;
                        end
                    end
                end
                ST_CMD6_CRC_SEND: begin
                    if (bst == BS_IDLE) begin
                        bt  <= 8'hFF;
                        bgo <= 1'b1;
                        st  <= ST_CMD6_CRC_WAIT;
                    end
                end
                ST_CMD6_CRC_WAIT: begin
                    if (bdone) begin
                        if (byte_idx == 10'd1) begin
                            byte_idx    <= 10'd0;
                            // Real-HW finding 2026-07-23: with CRC16
                            // checking finally able to observe data
                            // correctness at HS_HALF (50 MHz), the CMD18
                            // ROM bulk-load deterministically failed its
                            // CRC on the very first block, every single
                            // attempt, across a full CTRL_RETRY_MAX
                            // exhaustion (dbg_ctrl_retry=5, dbg_sector=0
                            // every time) — a signature inconsistent with
                            // random signal noise (which would scatter
                            // across the 2048-sector run) and consistent
                            // with 50 MHz simply exceeding this board's
                            // real SD trace signal integrity. Nothing
                            // before this session ever checked data
                            // CORRECTNESS at HS speed (only that the SPI
                            // waveform itself looked right), so this may
                            // have been silently corrupting boot data on
                            // real HW all along. Deliberately never
                            // engage HS mode until/unless a board
                            // revision or slower HS divider is proven
                            // clean against CRC — stay at FAST_HALF
                            // (25 MHz) even though the card accepted the
                            // switch. The 64+2 status bytes are still
                            // drained above; only the speed selection
                            // changes (unless cfg_force_hs opts back in
                            // for A/B testing on the sdmin harness).
                            if (cfg_force_hs) spi_hs_mode <= 1'b1;
                            ctrl_retry  <= 4'd0;
                            st          <= ST_CTRL_KICK;
                        end else begin
                            byte_idx <= 10'd1;
                            st       <= ST_CMD6_CRC_SEND;
                        end
                    end
                end

                // ────────── Kick sd_ctrl for CMD18 multi-block read ──
                ST_CTRL_KICK: begin
                    // Reset per-sector/stream state and fire go.
                    sector           <= 16'd0;
                    sec_byte_idx     <= 10'd0;
                    pack_phase       <= 2'd0;
                    ctrl_go          <= 1'b1;
                    st               <= ST_CTRL_RUN;
                end

                // ────────── Stream bytes from ctrl → AXI writes ─────
                ST_CTRL_RUN: begin
                    // Every byte coming out of sd_ctrl lands in the 4-byte
                    // big-endian packer.  Every 4th byte queues one
                    // single-beat AXI word write.

                    if (ctrl_rd_valid) begin
                        case (pack_phase)
                            2'd0: pack_b0 <= ctrl_rd_data;
                            2'd1: pack_b1 <= ctrl_rd_data;
                            2'd2: pack_b2 <= ctrl_rd_data;
                            2'd3: begin
                                // 4 bytes assembled — queue one word write
                                // (two, under MIRROR_LOW_RAM: the same
                                // word a second time at the identical
                                // relative offset off address 0 -- see the
                                // MIRROR_LOW_RAM parameter comment).
                                if (MIRROR_LOW_RAM) begin
                                    if (!word_fifo_room2) begin
                                        error     <= 1'b1;
                                        err_cause <= 3'd4;
                                        st        <= ST_ERROR;
                                    end else begin
                                        word_fifo_wr_ptr <= word_fifo_wr_ptr +
                                            {{(WORD_FIFO_LOG2-2){1'b0}}, 2'd2};
                                        word_fifo_count <= word_fifo_count +
                                            {{(WORD_FIFO_LOG2-1){1'b0}}, 2'd2};
                                    end
                                end else begin
                                    if (word_fifo_full) begin
                                        error     <= 1'b1;
                                        err_cause <= 3'd4;
                                        st        <= ST_ERROR;
                                    end else begin
                                        word_fifo_wr_ptr <= word_fifo_wr_ptr +
                                            {{(WORD_FIFO_LOG2-1){1'b0}}, 1'b1};
                                        word_fifo_count <= word_fifo_count +
                                            {{WORD_FIFO_LOG2{1'b0}}, 1'b1};
                                    end
                                end
                            end
                        endcase
                        pack_phase <= pack_phase + 2'd1;

                        if (sec_byte_idx == 10'd511) begin
                            sec_byte_idx <= 10'd0;
                            sector       <= sector + 16'd1;
                            pack_phase   <= 2'd0;
                        end else begin
                            sec_byte_idx <= sec_byte_idx + 10'd1;
                        end
                    end

                    if (burst_active && burst_aw_fired &&
                        burst_beats_done != 7'd0 &&
                        !m_axi_bready && m_axi_bvalid) begin
                        m_axi_bready <= 1'b1;
                    end

                    // Termination: sd_ctrl pulses done after CMD12.  We
                    // may still have queued writes in flight — go wait
                    // for them.
                    if (ctrl_done) begin
                        // Log this attempt's outcome BEFORE ctrl_retry is
                        // incremented below, so entry index == this
                        // attempt's own ordinal (0 = first attempt).
                        attempt_log[ctrl_retry] <=
                            ctrl_error ? {ctrl_err_cause, sector}
                                       : {4'hF, sector};
                        if (ctrl_error) begin
                            if (ctrl_retry < CTRL_RETRY_MAX) begin
                                // Whole-run retry: sd_ctrl returns to its
                                // own S_IDLE after any S_DONE (error or
                                // not), so a fresh `go` starts a brand-new
                                // CMD18 from sector 0 once the aborted
                                // attempt's in-flight AXI writes are fully
                                // drained (ST_CTRL_RETRY_DRAIN).
                                ctrl_retry <= ctrl_retry + 4'd1;
                                st         <= ST_CTRL_RETRY_DRAIN;
                            end else begin
                                error     <= 1'b1;
                                err_cause <= 3'd3;
                                st        <= ST_ERROR;
                            end
                        end else begin
                            st <= ST_AXI_B_WAIT;
                        end
                    end
                end

                // ────────── Drain final AXI writes ──────────
                ST_AXI_B_WAIT: begin
                    if (burst_active && burst_aw_fired &&
                        burst_beats_done != 7'd0 && !m_axi_bready) begin
                        m_axi_bready <= 1'b1;
                    end
                    if (!burst_active && word_fifo_empty &&
                        !m_axi_awvalid && !m_axi_wvalid) begin
                        // Hand off to RAM pre-zero pass (skip if disabled
                        // at compile time via ZERO_BYTES, or for this
                        // particular boot via zero_en -- see the port).
                        // A VRAM_ZERO_BYTES pass, if any, always runs
                        // AFTER this one (ST_ZERO_B), never standalone --
                        // real usage always wants both together.
                        if (ZERO_BYTES == 32'd0 || !zero_en) begin
                            st <= ST_DONE;
                        end else begin
                            // MIRROR_LOW_RAM: start past the mirrored ROM
                            // image so this pass doesn't immediately
                            // overwrite it with zeros.
                            zero_addr <= MIRROR_LOW_RAM ? MIRROR_IMAGE_BYTES : 32'd0;
                            zero_beat <= 9'd0;
                            zero_pass_vram_q <= 1'b0;
                            st        <= ST_ZERO_AW;
                        end
                    end
                end

                // ────────── Drain before whole-run CMD18 retry ──────────
                ST_CTRL_RETRY_DRAIN: begin
                    if (burst_active && burst_aw_fired &&
                        burst_beats_done != 7'd0 && !m_axi_bready) begin
                        m_axi_bready <= 1'b1;
                    end
                    if (!burst_active && word_fifo_empty &&
                        !m_axi_awvalid && !m_axi_wvalid) begin
                        st <= ST_CTRL_KICK;
                    end
                end

                // ────────── RAM pre-zero pass ──────────
                // Walk zero_addr over 0..ZERO_BYTES-1 in (ZERO_AWLEN+1)-beat
                // 32-bit INCR write bursts of zeros, one burst in flight at
                // a time, advancing zero_addr by ZERO_BURST_BYTES per B.
                //
                // ONE OUTSTANDING BURST IS NOT A CHOICE HERE: the adapter
                // this master feeds (axi_narrow_to_wide, instantiated as
                // u_boot_n2w in fpga_top_boot_master.vh) gates its
                // n_awready on `!aw_valid_q`, and aw_valid_q only clears on
                // the B beat.  A second AW would simply sit unaccepted.
                // Long bursts, not multiple outstanding ones, are what
                // amortises the round-trip latency on this path.
                //
                // Reuses the same m_axi_aw*/w*/b* port as the ROM-copy
                // path; the ROM-copy burst is already drained at this
                // point (ST_AXI_B_WAIT exit gate) so there's no contention.
                ST_ZERO_AW: begin
                    if (!m_axi_awvalid) begin
                        m_axi_awid    <= AXI_ID;
                        m_axi_awaddr  <= zero_addr;
                        m_axi_awlen   <= ZERO_AWLEN;
                        m_axi_awsize  <= 3'd2;               // 4 bytes
                        m_axi_awburst <= 2'b01;              // INCR
                        m_axi_awvalid <= 1'b1;
                        zero_beat     <= 9'd0;
                    end
                    if (m_axi_awvalid && m_axi_awready) begin
                        // Default deassertion above the case clears
                        // m_axi_awvalid.  Move on to streaming W beats.
                        st <= ST_ZERO_W;
                    end
                end
                // Streams ZERO_AWLEN+1 W beats BACK TO BACK, WLAST only on
                // the final one.
                //
                // The re-arm condition is `!m_axi_wvalid || m_axi_wready`,
                // not `!m_axi_wvalid`.  The latter is what this state used
                // to say, and it cannot be true on a handshake cycle: the
                // deassert above the case is non-blocking, so m_axi_wvalid
                // still reads 1 while the handshake is completing.  The
                // state therefore dropped WVALID for a cycle after every
                // beat and the whole pass ran at 2 cycles/beat.  Measured
                // in tb-sd-boot-zero: 2.187 cycles/word at a 16-beat burst
                // where the shape implies ~1.06.
                //
                // The assignments below are the LAST writes to
                // m_axi_wvalid/wlast/wstrb in this always block, so they
                // win over that deassert -- which is exactly what holds
                // WVALID high across the handshake.
                ST_ZERO_W: begin
                    if (!m_axi_wvalid || m_axi_wready) begin
                        if (zero_w_hs && m_axi_wlast) begin
                            // Final beat retired: collect the response.
                            m_axi_bready <= 1'b1;
                            st           <= ST_ZERO_B;
                        end else begin
                            zero_beat     <= zero_beat_nx;
                            m_axi_wdata   <= 32'd0;
                            m_axi_wstrb   <= 4'b1111;
                            m_axi_wlast   <= (zero_beat_nx == {1'b0, ZERO_AWLEN});
                            m_axi_wvalid  <= 1'b1;
                        end
                    end
                end
                ST_ZERO_B: begin
                    if (m_axi_bready && m_axi_bvalid) begin
                        // Default handshake (above the case) clears
                        // m_axi_bready.  Decide whether to launch the next
                        // write or finish; a non-OKAY BRESP on this same beat
                        // is caught by the axi_b_err override after `endcase`,
                        // which runs later in this block and therefore wins
                        // over the ST_ZERO_AW / ST_DONE assignment below.
                        // The compare is statically constant when
                        // ZERO_BYTES=0 (which is also the bypass case at
                        // ST_AXI_B_WAIT), so suppress UNSIGNED here.
                        // zero_pass_end_c tracks whichever pass is
                        // current (main ZERO_BYTES range or, once that
                        // one finishes, the VRAM_ZERO_BYTES range) -- see
                        // its declaration and VRAM_ZERO_BYTES's parameter
                        // comment.
                        /* verilator lint_off UNSIGNED */
                        if (zero_addr + ZERO_BURST_BYTES >= zero_pass_end_c) begin
                            if (!zero_pass_vram_q && VRAM_ZERO_BYTES != 32'd0) begin
                                // Main pass just finished -- hand off to
                                // the VRAM pass instead of ST_DONE.
                                zero_pass_vram_q <= 1'b1;
                                zero_addr        <= VRAM_ZERO_BASE;
                                st               <= ST_ZERO_AW;
                            end else begin
                                st <= ST_DONE;
                            end
                        end else begin
                            zero_addr <= zero_addr + ZERO_BURST_BYTES;
                            st        <= ST_ZERO_AW;
                        end
                        /* verilator lint_on UNSIGNED */
                    end
                end

                // ────────── Terminal states ──────────
                ST_DONE: begin
                    rom_loading_int <= 1'b0;
                    rom_loaded  <= 1'b1;
                    spi_cs_n    <= 1'b1;
                end

                ST_ERROR: begin
                    rom_loading_int <= 1'b0;
                    spi_cs_n    <= 1'b1;
                end

                default: st <= ST_RESET;
            endcase

            // ── AXI write-error override — MUST remain the last statement
            //    in this block ────────────────────────────────────────────
            //
            // A non-OKAY BRESP is fatal: the DDR write did not land.  This
            // used to be handled up in the B-channel bookkeeping block, i.e.
            // BEFORE `case (st)`, which meant any state that also assigned
            // `st` on the same B beat silently overwrote the ST_ERROR
            // transition.  ST_ZERO_B did exactly that unconditionally (its
            // guard is literally the same `m_axi_bready && m_axi_bvalid`), so
            // a failed RAM pre-zero write left `error` asserted but marched
            // the FSM on to ST_DONE -> rom_loaded=1 -> boot_rom_ready
            // releases the CPU (fpga_top_clocks.vh) onto RAM that was never
            // fully zeroed.  That is indistinguishable from the
            // already-fixed "stale-RAM Sad Mac", so the bug could impersonate
            // a bug we believe is closed.  ST_CTRL_RUN's `if (ctrl_done)`
            // arm had the same (rarer, timing-dependent) exposure.
            //
            // Placing the check here makes it win over every case arm at
            // once.  ST_ERROR is sticky: its case arm never assigns `st`, and
            // this override only ever assigns ST_ERROR, so the FSM cannot be
            // walked back out by a later beat.
            if (axi_b_err) begin
                error     <= 1'b1;
                err_cause <= ERR_CAUSE_BRESP;
                st        <= ST_ERROR;
            end
        end
    end

    // verilator lint_off UNUSED
    wire _unused = &{1'b0, ctrl_busy, ctrl_err_cause, m_axi_bid, 1'b0};
    // verilator lint_on UNUSED

endmodule
