// vram.v — URAM-backed Mac framebuffer with AXI4 slave + streaming read port
//
// Topology
// ────────
//     ┌── AXI4 slave  (writes from xbar S3 — CPU + XDMA, 128-bit DATA)
//     │            + reads (debug only — not the scan-out fast path)
//     │            on `clk` (system, ~200 MHz)
//     │
//   [ URAM array (true dual-port, xpm_memory_tdpram, MEMORY_PRIMITIVE=ultra) ]
//     │
//     └── Streaming read port  (pixel-addressed, internally pipelined)
//                  on `rd_clk` (pixel clock, ~148.5 MHz for 1080p60)
//
// This module wraps `FB_WIDTH_PX * FB_HEIGHT_PX * BPP/8` bytes of on-chip
// UltraRAM.  The AXI slave accepts writes from the CPU LSU and XDMA via
// the system crossbar; the streaming read port feeds the HDMI scan-out
// scaler directly (no AXI, no DDR — the whole point of VRAM-on-URAM per
// docs/memhier.md §"VRAM placement — URAM, not DDR").
//
// Port independence (single-clock reality)
// ─────────────────────────────────────────
// Port A (`clk`) serves AXI writes + AXI debug reads.  Port B is the
// streaming scan-out read port.  Despite the `rd_clk`/`rd_rst` port
// names below, port B is NOT on an independently-clocked domain: the
// synthesis xpm_memory_tdpram instance further down is
// CLOCKING_MODE="common_clock" (Vivado's xpm_memory wrapper refuses
// CLOCKING_MODE=independent_clock when MEMORY_PRIMITIVE="ultra") and
// hard-wires `.clkb(clk)`, ignoring `rd_clk` entirely at synthesis.
// `rd_clk`/`rd_rst` are kept as ports for API stability and MUST be
// tied to `clk`/`rst` by every instantiator (fpga_top_video.vh does
// this: `.rd_clk(core_clk)` alongside `.clk(core_clk)`).  The
// behavioural VERILATOR model mirrors this by clocking its port-B
// read pipeline on `clk`, not `rd_clk`, so sim and synthesis agree.
// Any pclk-domain crossing for the HDMI scan-out chain happens
// DOWNSTREAM of this module, inside fb_reader's CDC FIFOs — never
// inside the URAM itself.
//
// Parameters
// ──────────
//   FB_WIDTH_PX  : pixel columns (default 1024)
//   FB_HEIGHT_PX : pixel rows    (default 768)
//   BPP          : bits per pixel — legal values: 8, 16
//                  (BPP=24 / BPP=32 are NOT supported on the KU5P URAM
//                  budget — see "URAM capacity math" below.  The elab
//                  guard `generate if (BPP > 16) $error(...)` kills any
//                  attempt to instantiate the module with BPP>16.  For
//                  true-colour modes route the framebuffer through DDR
//                  instead.  See docs/vram_uram.md.)
//   RD_DATA_W    : streaming-read-port output width (32 — a 4-byte group;
//                  see "Streaming read port" below).  This is the PORT
//                  width only; it does NOT change the pixel packing, the
//                  URAM word width, or BPP-derived sizing.  Must be 32.
//   DATA_WIDTH   : AXI data width (128; matches axi_xbar S3 contract)
//   ID_WIDTH     : AXI ID width (4; matches axi_xbar default)
//
// URAM capacity math (KU5P = 64 URAM288 blocks)
// ─────────────────────────────────────────────
// One URAM288 is 4096 deep × 72 bit.  For DATA_WIDTH=128 we need two
// URAMs wide (2 × 72 ≥ 128) to serve one word per cycle.  Depth
// cascades to reach N_WORDS.  Worst case URAM count for the Mac
// resolutions we care about (1024 × 768):
//
//   BPP=8  : N_WORDS = 768 KB / 16 B =  49,152 → 12 deep × 2 wide = 24 URAMs  ≈ 38% KU5P
//   BPP=16 : N_WORDS = 1536 KB / 16 B = 98,304 → 24 deep × 2 wide = 48 URAMs  ≈ 75% KU5P
//   BPP=24 : N_WORDS = packs 5 px/word unevenly; ~31 deep × 2 = 62 URAMs     ≈ 97% (NO)
//   BPP=32 : N_WORDS = 3072 KB / 16 B = 196,608 → 48 deep × 2 wide = 96 URAMs ≈ 150% (NO)
//
// The first synth that tried BPP=24 inferred ~221k RAM256X1D distributed-
// LUT RAMs because a plain 2-port `reg [...] mem [0:N-1]` array with a
// `(* ram_style = "ultra" *)` hint isn't in XST's URAM inference rule
// set for true dual-port access.  The fix is to instantiate
// `xpm_memory_tdpram` directly with `MEMORY_PRIMITIVE = "ultra"`.  Note
// this does NOT buy independently-clocked ports: Vivado's xpm_memory
// wrapper refuses `CLOCKING_MODE = "independent_clock"` when
// `MEMORY_PRIMITIVE = "ultra"`, so both ports run on `clk`
// (`CLOCKING_MODE = "common_clock"` — see the single-clock header note
// at the top of this file).
//
// For BPP > 16 users: the elab guard halts instantiation.  Route Mac
// 24-bit modes through DDR-backed framebuffer instead (outside this
// module's scope — see docs/vram_uram.md).
//
// Pixel packing / endianness
// ──────────────────────────
// Linear layout, Y-major: pixel (x, y) → byte_offset
//                              = (y * FB_WIDTH_PX + x) * (BPP / 8)
// Reads via the streaming port use `rd_addr` = pixel index = (y * W + x),
// where the scan-out walks addresses in raster order (x varies fastest).
//
// Within an AXI data word, lane 0 (bits [BPP-1:0]) holds the LOWEST-
// address pixel in that word, lane 1 holds the next, and so on.  This is
// "little-endian lanes within a word" — it matches the Mac OS convention
// for packing consecutive pixels into a 32-bit word at increasing bit
// positions (QuickDraw row pointers advance by bytes, and within a byte
// BPP=1/2/4/8 go left-to-right, which is pixel 0 in the MSBs of that
// byte; however Mac 8bpp == one pixel per byte, so lane ordering is what
// matters and we pick little-endian-lanes for hardware simplicity — it
// means `pixel_i_within_word = wdata[i*BPP +: BPP]` with no bit-reverse).
// The scaler / HDMI renderer must be told this convention; a byte-swap
// shim in the xbar integrator can flip it if Mac OS software turns out
// to expect the opposite.
//
// WSTRB per-byte masking is respected: any byte lane whose WSTRB bit is
// low leaves that byte of the stored word unchanged (standard AXI4
// semantics, directly supported by xpm_memory_tdpram's `wea[DATA/8-1:0]`
// byte-write-enable pin).
//
// Port independence (single clock, dual port)
// ────────────────────────────────────────────
// Both ports run on the SAME clock (`clk`, aliased as `rd_clk` at the
// port list for API stability — see the header note above).  Port A
// (writes + AXI debug reads) and port B (streaming scan-out reads) are
// still independent RAM ports — no memory coherency issue inside this
// module, each side sees a consistent view of any word whose write has
// committed — but there is no cross-clock-domain hazard to reason about
// because there is only one clock domain.  A word being updated on the
// AXI side the SAME cycle it is being read on the scan-out side returns
// either the old OR the new value for that cycle (standard dual-port
// "no-change" behaviour), never a glitch/metastable byte.
//
// The AXI write-response protocol guarantees that, by the time `BVALID`
// is asserted, the data has committed to URAM.  Software wishing to do
// a tear-free scan-out should sequence its writes such that all writes
// to a given frame complete before the corresponding vsync interval —
// i.e. double-buffer in software, which QuickDraw already does.
//
// Latency
// ───────
//   AXI write:  1 cycle memory write  (AW+W → BVALID on next cycle)
//   AXI read:   XPM_READ_LATENCY+1 cycles minimum
//                                      (AR → pipelined URAM read → R)
//   Read port:  XPM_READ_LATENCY cycles (rd_en → rd_valid & rd_data)
//
// Sim vs synthesis
// ────────────────
// The xpm_memory_tdpram IP is SystemVerilog and cannot be compiled by
// the sim tool-chain (verilator).  Under the VERILATOR define the
// module falls back to a behavioural 2D register array that models the
// same read/write protocol one-for-one, so the unit tbs (tb_vram,
// tb_video_smoke) see identical behaviour.  The only path that
// exercises URAM inference is the real Vivado synth/impl flow — where
// the xpm primitive is compiled in.
//
// Verilog-2005 (outside the xpm wrapper itself), synchronous active-high
// rst (per port), 4-space indent.

module vram #(
    parameter FB_WIDTH_PX  = 1024,
    parameter FB_HEIGHT_PX = 768,
    parameter BPP          = 8,        // legal: 8, 16 — see header
    // Streaming-read-port output width.  The port returns FOUR consecutive
    // VRAM bytes per request (see the streaming-read-port block below), so
    // this is 32 and is NOT the pixel width — BPP still governs
    // BYTES_PER_PX / PX_PER_WORD / N_PIXELS / FB_BYTES.  An elaboration
    // guard below rejects any other value.
    // Defaults to 4*BPP (=32 at the production BPP=8) so a 4-lane group is
    // always self-consistent without every instantiator having to say so.
    parameter RD_DATA_W    = 4 * BPP,
    parameter DATA_WIDTH   = 128,
    parameter ID_WIDTH     = 4,
    // VRAM_BYTES — total addressable VRAM aperture, INDEPENDENT of the
    // active framebuffer area (FB_WIDTH_PX × FB_HEIGHT_PX × BPP/8).  The
    // framebuffer occupies a contiguous prefix of VRAM_BYTES; the
    // remaining bytes are still real RAM (not open-bus) and follow
    // normal R/W semantics.  Default = 2 MiB to match Q700 silicon
    // (`map(0xf9000000, 0xf91fffff)` per MAME `quadra700_map`).  Keeping
    // this fixed at 2 MiB regardless of resolution lets the host swap
    // FB_WIDTH/FB_HEIGHT/BPP without re-sizing the URAM bank or the
    // xbar's `AXI_VRAM_SIZE` decode window.
    //
    // URAM count for 2 MiB at DATA_WIDTH=128:
    //   2 MiB / 16 B = 131,072 words → 32 deep × 2 wide = 64 URAMs
    //   (50% of KU5P's 128-URAM budget).
    // The earlier 1024×768×8 = 768 KB → 24 URAMs (38%) configuration is
    // still selectable by passing VRAM_BYTES = FB_WIDTH_PX*FB_HEIGHT_PX*BPP/8
    // explicitly.
    parameter VRAM_BYTES   = 32'h0020_0000,
    // Derived widths exposed as parameters so they can be used in the
    // port declarations below.  Do NOT override at instantiation — they
    // are computed from the sizing parameters above.
    //
    // PX_ADDR_SPAN — the streaming read port (`rd_addr`) must be able to
    // address every pixel in the FULL VRAM aperture (VRAM_BYTES /
    // bytes-per-pixel), not just the active FB_WIDTH_PX×FB_HEIGHT_PX
    // window.  Sizing PX_ADDR_W from FB_WIDTH_PX*FB_HEIGHT_PX alone (the
    // pre-fix behaviour) under-widens rd_addr whenever VRAM_BYTES covers
    // more pixels than the active mode — e.g. the default 2 MiB Q700
    // aperture at BPP=8 holds 2,097,152 pixels (21 bits) while a single
    // 1024×768 mode only needs 786,432 (20 bits).  A DAFB base register
    // >= 0x100000 would then alias/truncate onto the low half of the
    // URAM.  Take the max of both spans so PX_ADDR_W always covers
    // whichever is larger.
    parameter PX_ADDR_SPAN = ((VRAM_BYTES / (BPP / 8)) > (FB_WIDTH_PX * FB_HEIGHT_PX)) ?
                             (VRAM_BYTES / (BPP / 8)) : (FB_WIDTH_PX * FB_HEIGHT_PX),
    parameter PX_ADDR_W    = (PX_ADDR_SPAN <= 2) ? 1 :
                             (PX_ADDR_SPAN <= 4) ? 2 :
                             (PX_ADDR_SPAN <= 8) ? 3 :
                             (PX_ADDR_SPAN <= 16) ? 4 :
                             (PX_ADDR_SPAN <= 32) ? 5 :
                             (PX_ADDR_SPAN <= 64) ? 6 :
                             (PX_ADDR_SPAN <= 128) ? 7 :
                             (PX_ADDR_SPAN <= 256) ? 8 :
                             (PX_ADDR_SPAN <= 512) ? 9 :
                             (PX_ADDR_SPAN <= 1024) ? 10 :
                             (PX_ADDR_SPAN <= 2048) ? 11 :
                             (PX_ADDR_SPAN <= 4096) ? 12 :
                             (PX_ADDR_SPAN <= 8192) ? 13 :
                             (PX_ADDR_SPAN <= 16384) ? 14 :
                             (PX_ADDR_SPAN <= 32768) ? 15 :
                             (PX_ADDR_SPAN <= 65536) ? 16 :
                             (PX_ADDR_SPAN <= 131072) ? 17 :
                             (PX_ADDR_SPAN <= 262144) ? 18 :
                             (PX_ADDR_SPAN <= 524288) ? 19 :
                             (PX_ADDR_SPAN <= 1048576) ? 20 :
                             (PX_ADDR_SPAN <= 2097152) ? 21 :
                             (PX_ADDR_SPAN <= 4194304) ? 22 :
                             (PX_ADDR_SPAN <= 8388608) ? 23 :
                             (PX_ADDR_SPAN <= 16777216) ? 24 : 25
) (
    input  wire                       clk,
    input  wire                       rst,

    // ── Platform-reset wipe trigger ─────────────────────────────────
    // When asserted (typically driven from soc_full_rst at the top), the
    // internal clear FSM walks all VRAM words writing zero through port A.
    // While the wipe runs the AXI slave refuses new transactions
    // (AWREADY/WREADY/ARREADY all held low).  At 100 MHz on the default
    // 2 MiB / 128-bit URAM the wipe completes in ~131k cycles ≈ 1.3 ms —
    // invisible to the host during a debug-driven warm reset, but it
    // guarantees the framebuffer is clean instead of showing stale pixels
    // from the previous run.  The wipe also runs once on cold rst.
    input  wire                       clear_req,

    // ═══════════════════════════════════════════════════════════════════
    // AXI4 slave — CPU + XDMA writes (AW/W/B); debug reads via AR/R.
    // Integrator: connect to the xbar's S2 master side when axi-xbar-vram
    // adds the VRAM slave port.
    // ═══════════════════════════════════════════════════════════════════
    // AW
    input  wire [ID_WIDTH-1:0]        s_awid,
    input  wire [31:0]                s_awaddr,
    input  wire [7:0]                 s_awlen,
    input  wire [2:0]                 s_awsize,   // accepted, not enforced
    input  wire [1:0]                 s_awburst,  // INCR only; FIXED/WRAP treated as INCR
    input  wire                       s_awvalid,
    output wire                       s_awready,
    // W
    input  wire [DATA_WIDTH-1:0]      s_wdata,
    input  wire [DATA_WIDTH/8-1:0]    s_wstrb,
    input  wire                       s_wlast,
    input  wire                       s_wvalid,
    output reg                        s_wready,
    // B
    output reg  [ID_WIDTH-1:0]        s_bid,
    output reg  [1:0]                 s_bresp,
    output reg                        s_bvalid,
    input  wire                       s_bready,
    // AR
    input  wire [ID_WIDTH-1:0]        s_arid,
    input  wire [31:0]                s_araddr,
    input  wire [7:0]                 s_arlen,
    input  wire [2:0]                 s_arsize,   // accepted, not enforced
    input  wire [1:0]                 s_arburst,  // INCR only; FIXED/WRAP treated as INCR
    input  wire                       s_arvalid,
    output wire                       s_arready,
    // R
    output reg  [ID_WIDTH-1:0]        s_rid,
    output reg  [DATA_WIDTH-1:0]      s_rdata,
    output reg  [1:0]                 s_rresp,
    output reg                        s_rlast,
    output reg                        s_rvalid,
    input  wire                       s_rready,

    // ═══════════════════════════════════════════════════════════════════
    // Streaming read port — HDMI scan-out path.
    // Byte-addressed at BPP=8; `rd_data` returns a FOUR-BYTE GROUP starting
    // at `rd_addr`, in increasing-address-toward-the-LSB (big-endian) order:
    //
    //   rd_data[31:24] = byte at rd_addr      — ALWAYS valid, any alignment
    //   rd_data[23:16] = byte at rd_addr + 1  — valid ONLY if rd_addr[1:0]==0
    //   rd_data[15: 8] = byte at rd_addr + 2  — valid ONLY if rd_addr[1:0]==0
    //   rd_data[ 7: 0] = byte at rd_addr + 3  — valid ONLY if rd_addr[1:0]==0
    //
    // Why this is free: a full DATA_WIDTH(=128)-bit URAM word was ALREADY
    // being read out of port B for every request, and 15 of its 16 bytes
    // were discarded by the old single-lane mux.  Returning 4 bytes costs
    // NO extra URAM read and no second port — only a wider output mux.
    //
    // Why aligned needs no straddle handling: a 4-byte group starting at a
    // 4-byte-ALIGNED address can never cross a 16-byte URAM word boundary
    // (the aligned in-word lane is one of 0/4/8/12, and lane 12 still
    // yields in-word lanes 12,13,14,15).  Unaligned requests remain fully
    // supported for rd_data[31:24] — all the indexed (1/2/4/8bpp) scan-out
    // path ever consumes.  The 24bpp direct-colour consumer issues only
    // 4-byte-aligned requests and takes [23:16]/[15:8]/[7:0] as R/G/B of an
    // xRGB word.  For an unaligned request the upper three bytes are
    // architecturally DON'T-CARE (they wrap within the same URAM word).
    //
    // `rd_valid` is `rd_en` delayed by the XPM read latency (matches
    // `rd_data`) — unchanged by the widening.
    // `rd_clk` MUST equal `clk` (see the single-clock header note at the
    // top of this file) — it is not an independent/asynchronous domain.
    //
    // These are wires driven combinationally from the URAM output +
    // byte-gather mux — XPM's internal read pipeline provides the latency;
    // muxing happens AFTER the registered URAM output path.
    // ═══════════════════════════════════════════════════════════════════
    input  wire                       rd_clk,
    input  wire                       rd_rst,
    input  wire [PX_ADDR_W-1:0]       rd_addr,   // pixel index (0..W*H-1)
    input  wire                       rd_en,
    output wire [RD_DATA_W-1:0]       rd_data,
    output wire                       rd_valid
);

    // ── Derived sizing ─────────────────────────────────────────────────
    // N_PIXELS / FB_BYTES describe the ACTIVE framebuffer region used by
    // the streaming scan-out port.  N_BYTES describes the FULL addressable
    // VRAM aperture (≥ FB_BYTES).  AXI accesses anywhere inside [0, N_BYTES)
    // are honoured as normal R/W; only the streaming scanner is bounded
    // by N_PIXELS.
    localparam integer N_PIXELS   = FB_WIDTH_PX * FB_HEIGHT_PX;
    localparam integer BYTES_PER_PX = BPP / 8;
    localparam integer FB_BYTES   = N_PIXELS * BYTES_PER_PX;
    localparam integer N_BYTES    = (VRAM_BYTES > FB_BYTES) ? VRAM_BYTES : FB_BYTES;
    // Words in the DATA_WIDTH-wide memory array.
    localparam integer BYTES_PER_WORD = DATA_WIDTH / 8;
    localparam integer N_WORDS    = (N_BYTES + BYTES_PER_WORD - 1) / BYTES_PER_WORD;

    // Pixels per AXI word = DATA_WIDTH / BPP  (e.g. 16 at 128/8).
    localparam integer PX_PER_WORD = DATA_WIDTH / BPP;
    // XPM UltraRAM only has the primitive output register at READ_LATENCY=1.
    // A latency above 1 lets XPM emit pipeline stages on the URAM matrix path.
    // Latency 6 gives Vivado enough stages to absorb the 8-deep cascade and
    // the byte-narrow write path's column register chain.  Bumped from 5
    // after vivado.log [Synth 8-6013] flagged
    //   "u_vram/u_uram/.../gen_byte_narrow.for_mem_cols[1].mem_reg" as
    //   under-pipelined (found=3, recommended=4) — that path was the
    //   gating one for performance margin under MEMORY_OPTIMIZATION.
    localparam integer XPM_READ_LATENCY = 6;
    localparam [3:0] XPM_READ_WAIT = XPM_READ_LATENCY[3:0];

    // Address bit widths.
    // WORD_ADDR_W indexes the internal RAM.
    // PX_ADDR_W  (declared in the port list above as a parameter) indexes
    //            pixels on the streaming read port.
    // PX_IN_WORD_W is the low-bits-of-rd_addr width selecting the lane.
    // clog2 via simple function.
    function integer clog2;
        input integer x;
        integer i;
        begin
            clog2 = 0;
            for (i = x - 1; i > 0; i = i >> 1) clog2 = clog2 + 1;
            if (clog2 == 0) clog2 = 1;
        end
    endfunction

    localparam integer WORD_ADDR_W  = clog2(N_WORDS);
    localparam integer PX_IN_WORD_W = clog2(PX_PER_WORD);
    localparam [31:0] N_BYTES_U32 = N_BYTES;
    localparam [1:0] AXI_RESP_OKAY   = 2'b00;
    localparam [1:0] AXI_RESP_SLVERR = 2'b10;

    function [31:0] beat_bytes;
        input [2:0] size;
        begin
            if (size > 3'd4) beat_bytes = 32'd0;
            else             beat_bytes = (32'd1 << size);
        end
    endfunction

    function beat_unaligned;
        input [31:0] byte_addr;
        input [2:0]  size;
        reg   [31:0] mask;
        begin
            if (size > 3'd4) begin
                beat_unaligned = 1'b1;
            end else begin
                mask = beat_bytes(size) - 32'd1;
                beat_unaligned = |(byte_addr & mask);
            end
        end
    endfunction

    function beat_oob;
        input [31:0] byte_addr;
        input [2:0]  size;
        reg   [31:0] last_byte;
        begin
            if (size > 3'd4) begin
                beat_oob = 1'b1;
            end else begin
                last_byte = byte_addr + beat_bytes(size) - 32'd1;
                beat_oob = (byte_addr >= N_BYTES_U32) ||
                           (last_byte >= N_BYTES_U32) ||
                           (last_byte < byte_addr);
            end
        end
    endfunction

    // ── Synth-time capacity guard ──────────────────────────────────────
    // Halt elaboration if the caller tries to instantiate with BPP > 16.
    // 1024×768×24bpp = 62 URAMs ≈ 97% of KU5P's 64-URAM budget (with no
    // slack for L2 or future growth).  1024×768×32bpp exceeds the budget
    // entirely.  Mac true-colour modes route through DDR, not URAM.
    generate
        if (BPP > 16) begin : gen_bpp_oversize
            initial begin
                $error("[vram] VRAM URAM capacity exceeded at BPP=%0d — URAM budget on KU5P fits BPP<=16 only at 1024x768.  Use a DDR-backed framebuffer instead (see docs/vram_uram.md).", BPP);
            end
        end
    endgenerate

    // ── Streaming-read-port shape guard (elaboration-time) ─────────────
    // The real invariant is FOUR LANES, not BPP==8: the gather below emits
    // RD_GROUP_LANES = RD_DATA_W/BPP consecutive lanes, and every lane must be
    // driven, so RD_DATA_W must be exactly 4*BPP.  RD_DATA_W's default is
    // 4*BPP, so an instantiation that does not care about the streaming port
    // (e.g. a BPP=16 instance exercising only the AXI write path, as
    // tb_vram_cpu_write does) stays legal and needs no override.
    //
    // An earlier draft of this guard demanded BPP==8 outright, which broke
    // every legal BPP=16 instantiation even though none of them touch the
    // streaming read port -- hence the narrower condition here.  Same
    // generate+$error idiom as the capacity guard above, so it fires at
    // synthesis elaboration too, not just in sim.
    generate
        if (RD_DATA_W != 4 * BPP) begin : gen_rd_data_w_bad
            initial begin
                $error("[vram] streaming read port requires RD_DATA_W == 4*BPP (got RD_DATA_W=%0d, BPP=%0d) — rd_data is a 4-LANE group {l0,l1,l2,l3} and every lane must be driven.", RD_DATA_W, BPP);
            end
        end
    endgenerate

    // ═══════════════════════════════════════════════════════════════════
    // Port-A muxing — writes (AXI W) vs reads (AXI AR)
    // ═══════════════════════════════════════════════════════════════════
    // The RAM's port A serves both the AXI write path and the AXI read
    // (debug) path:
    //   - WS_DATA + W handshake drives a write     → ena=1, wea=s_wstrb
    //   - RS_IDLE + AR handshake drives a 1st read  → ena=1, wea=0
    //   - RS_RESP/RS_ARB + next-beat fetch drives   → ena=1, wea=0
    //     an Nth read
    // These are NOT structurally mutually exclusive: AW acceptance
    // (s_awready) has no dependency on read-FSM (`rs`) state, so a
    // write can be accepted and start streaming W beats while a
    // multi-beat AXI read is still in flight, and their port-A
    // requests can land on the same cycle.  wr_commit always wins that
    // arbitration (pa_ena_axi/pa_addr_axi below); a colliding read
    // fetch is never dropped — it STALLS in RS_ARB and replays on the
    // first cycle port A is free (see the read-FSM comment block for
    // the full contract).  The one true structural exclusion is at
    // AR-accept time: s_arready is gated on `ws == WS_IDLE`, so the
    // FIRST beat of a read is never accepted while a write is mid-burst.
    // Addressing: s_awaddr / s_araddr are BYTE addresses.  The RAM word
    // index comes from the current byte address, and INCR bursts advance
    // by 1 << AxSIZE bytes per beat.  That preserves full-width 128-bit
    // bursts and narrow bursts on the 128-bit port, such as 64-bit DMA
    // beats alternating low/high half-lanes.  The FB base (0x6000_0000)
    // is peeled off BY THE XBAR — so s_awaddr / s_araddr entering this
    // module is already the offset-into-VRAM.

    localparam integer WORD_BYTE_W = clog2(BYTES_PER_WORD);

    // ── AXI write FSM ──────────────────────────────────────────────────
    localparam [1:0] WS_IDLE = 2'd0,
                     WS_DATA = 2'd1,
                     WS_RESP = 2'd2;
    reg  [1:0]               ws;
    reg  [ID_WIDTH-1:0]      ws_id;
    reg  [31:0]              ws_byte_addr;
    reg  [2:0]               ws_size;
    reg  [1:0]               ws_burst;
    reg  [7:0]               ws_beats_left;
    reg                      ws_err;
    // Single-cycle "drive a write to port A" pulse — combinational from
    // the FSM state + W handshake.  Used both to advance the RAM write
    // and to mux port-A {addra, wea, dina, ena}.
    wire                     wr_fire = (ws == WS_DATA) && s_wvalid && s_wready;
    wire                     wr_fire_bad =
        beat_unaligned(ws_byte_addr, ws_size) ||
        beat_oob(ws_byte_addr, ws_size);
    wire                     wr_commit = wr_fire && !wr_fire_bad;

    // s_awready is COMBINATIONAL (not registered) so write acceptance
    // reacts to the SAME cycle's AWVALID, not a value latched one cycle
    // earlier.  This matters for the AR arbitration below: s_arready
    // must see this cycle's s_awvalid to correctly give the write
    // priority when both AWVALID and ARVALID rise together.  See the
    // long comment at the read FSM for the full race this closes.
    assign s_awready = (ws == WS_IDLE) && !clearing;

    always @(posedge clk) begin
        if (rst) begin
            ws            <= WS_IDLE;
            s_wready      <= 1'b0;
            s_bvalid      <= 1'b0;
            s_bid         <= {ID_WIDTH{1'b0}};
            s_bresp       <= 2'b00;
            ws_id         <= {ID_WIDTH{1'b0}};
            ws_byte_addr  <= 32'b0;
            ws_size       <= 3'b0;
            ws_burst      <= 2'b0;
            ws_beats_left <= 8'd0;
            ws_err        <= 1'b0;
        end else begin
            case (ws)
                WS_IDLE: begin
                    s_wready  <= 1'b0;
                    // s_awready == !clearing here (we're in WS_IDLE), so
                    // this is exactly "AW handshake this cycle".
                    if (s_awvalid && !clearing) begin
                        ws_id         <= s_awid;
                        ws_byte_addr  <= s_awaddr;
                        ws_size       <= s_awsize;
                        ws_burst      <= s_awburst;
                        ws_beats_left <= s_awlen; // beats remaining AFTER the first
                        ws_err        <= beat_unaligned(s_awaddr, s_awsize) ||
                                         beat_oob(s_awaddr, s_awsize);
                        s_wready      <= 1'b1;
                        ws            <= WS_DATA;
                    end
                end
                WS_DATA: begin
                    s_wready <= 1'b1;
                    if (wr_fire) begin
                        if (wr_fire_bad)
                            ws_err <= 1'b1;

                        // The actual RAM write happens via the port-A mux
                        // below; this FSM only advances pointers and
                        // bookkeeping here.
                        //
                        // Advance by the AXI beat size for INCR bursts.  This
                        // supports narrow bursts on the 128-bit VRAM port, such
                        // as 64-bit DMA beats alternating low/high half-lanes.
                        if (ws_burst == 2'b01)
                            ws_byte_addr <= ws_byte_addr + (32'd1 << ws_size);

                        if (ws_beats_left == 8'd0 || s_wlast) begin
                            // Final beat.  Latch B response.
                            s_wready      <= 1'b0;
                            s_bvalid      <= 1'b1;
                            s_bid         <= ws_id;
                            s_bresp       <= (ws_err || wr_fire_bad) ?
                                             AXI_RESP_SLVERR : AXI_RESP_OKAY;
                            ws            <= WS_RESP;
                        end else begin
                            ws_beats_left <= ws_beats_left - 8'd1;
                        end
                    end
                end
                WS_RESP: begin
                    if (s_bvalid && s_bready) begin
                        s_bvalid  <= 1'b0;
                        ws        <= WS_IDLE;
                    end
                end
                default: ws <= WS_IDLE;
            endcase
        end
    end

    // ═══════════════════════════════════════════════════════════════════
    // AXI slave — read FSM
    // ═══════════════════════════════════════════════════════════════════
    // AR is captured then RLEN+1 beats returned one at a time.  One
    // outstanding RAM fetch at a time.  URAM output is internally pipelined
    // by XPM_READ_LATENCY cycles; this debug path waits for that output
    // before asserting RVALID.
    //
    // Path: AR accept @ edge N → port-A addra latched, URAM reads
    //       douta after XPM_READ_LATENCY cycles → we drive s_rdata on
    //       the following clk edge.

    // RS_ARB: a subsequent-beat fetch wanted port A the same cycle a
    // concurrent AXI write's data beat (wr_commit) also wanted it.  The
    // fetch STALLS here (replayed every cycle) rather than being
    // dropped — see the long comment below.
    localparam [1:0] RS_IDLE = 2'd0,
                     RS_WAIT = 2'd1,
                     RS_RESP = 2'd2,
                     RS_ARB  = 2'd3;
    reg  [1:0]               rs;
    reg  [ID_WIDTH-1:0]      rs_id;
    reg  [31:0]              rs_byte_addr;
    reg  [2:0]               rs_size;
    reg  [1:0]               rs_burst;
    reg  [7:0]               rs_beats_left;
    reg  [3:0]               rs_wait_left;
    reg                      rs_wait_err;

    // s_arready is COMBINATIONAL and gated on `!s_awvalid` (this cycle's
    // value, not a registered one-cycle-stale copy) so a write always
    // wins a same-cycle AW/AR collision instead of both handshaking at
    // once.  See "AW/AR same-cycle race" below.
    assign s_arready = (rs == RS_IDLE) && (ws == WS_IDLE) && !s_awvalid && !clearing;

    // Port-A read-enable pulse: high the cycle we START a mem fetch for
    // an AR beat.  For the first beat this is the AR handshake cycle; for
    // subsequent beats it is either the R handshake cycle for the
    // previous beat (if port A is free that cycle) or, when a collision
    // stalls it into RS_ARB, the first later cycle port A is free again.
    wire rd_fire_first = (rs == RS_IDLE) && s_arvalid && s_arready;
    // "Want the next-beat fetch" — does NOT yet factor in whether port A
    // is actually free this cycle; that gating lives in rd_mem_fire_next
    // below, and the RS_RESP/RS_ARB state transitions (further down)
    // decide whether the request stalls.
    wire rd_fire_next  = ((rs == RS_RESP) && s_rvalid && s_rready && (rs_beats_left != 8'd0)) ||
                         (rs == RS_ARB);
    wire rd_fire_first_bad = beat_unaligned(s_araddr, s_arsize) ||
                             beat_oob(s_araddr, s_arsize);

    // ── Port-A multiplexer (wires fed into the RAM instance below) ─────
    // AW/AR same-cycle race (fixed): s_arready above is combinational
    // and reads THIS cycle's s_awvalid, so if AWVALID and ARVALID both
    // rise while ws==WS_IDLE (idle), only the write handshakes this
    // cycle — ARREADY drops immediately (same cycle) rather than a
    // stale registered value staying high for one extra cycle.  The AR
    // simply waits until the write (and any of its queued successors)
    // vacates WS_IDLE.
    //
    // Later-beat collision (fixed): once a multi-beat AR has been
    // accepted, subsequent per-beat fetches (rd_fire_next) can still
    // land on the same cycle as an unrelated, LATER write's wr_commit
    // (AW acceptance has no dependency on `rs` state, only on `ws`).
    // Port A always gives wr_commit priority (pa_addr_axi below), so
    // when that collision happens the read fetch must not be silently
    // dropped: rd_mem_fire_next requires `!wr_commit`, and the RS state
    // machine (further down) routes the miss through RS_ARB, which
    // retries the SAME pending fetch every cycle until wr_commit is
    // low.  The XPM_READ_LATENCY countdown (rs_wait_left) only starts
    // once the fetch actually issues (in RS_WAIT), so it always tracks
    // the real issue cycle, never the original handshake cycle.
    wire [31:0]                  rs_next_byte_addr =
        (rs_burst == 2'b01) ? (rs_byte_addr + (32'd1 << rs_size)) : rs_byte_addr;
    wire rd_fire_next_bad = beat_unaligned(rs_next_byte_addr, rs_size) ||
                            beat_oob(rs_next_byte_addr, rs_size);
    wire rd_mem_fire_first = rd_fire_first && !rd_fire_first_bad;
    wire rd_mem_fire_next  = rd_fire_next  && !wr_commit && !rd_fire_next_bad;
    wire rd_mem_fire       = rd_mem_fire_first || rd_mem_fire_next;
    // ── Wipe-on-reset clear FSM ─────────────────────────────────────
    // Walks `clear_addr` from 0..N_WORDS-1, writing all-zero through
    // port A.  Triggered on the FALLING edge of `clear_req` (typically
    // wired = soc_full_rst at the top, so the wipe fires when the
    // platform reset releases).  While `clearing` is high, the AXI
    // slave's ready signals are held low (gates added below at WS/RS
    // FSM accept sites).
    //
    // NOT triggered by `rst` alone — unit tbs use rst freely and don't
    // want the (~131k-cycle) wipe blocking their AXI traffic.  Bitstream
    // URAM is zero-initialised at load anyway; only later WARM resets
    // need explicit wiping.  Tie `clear_req=0` in unit tbs.
    reg                          clearing;
    reg  [WORD_ADDR_W-1:0]       clear_addr;
    reg                          clear_req_q;
    wire                         clear_done = (clear_addr == {WORD_ADDR_W{1'b1}});

    always @(posedge clk) begin
        clear_req_q <= clear_req;
        if (clearing) begin
            clear_addr <= clear_addr + 1'b1;
            if (clear_done)
                clearing <= 1'b0;
        end else if (clear_req_q && !clear_req) begin
            // Falling edge of clear_req kicks off a fresh wipe.  This
            // pattern works even when clear_req is the same signal as
            // rst — by the time rst (and clear_req) drop, this register
            // has already sampled the high value into clear_req_q.
            clearing   <= 1'b1;
            clear_addr <= {WORD_ADDR_W{1'b0}};
        end
    end

    wire                        pa_ena_axi   = wr_commit || rd_mem_fire;
    wire [DATA_WIDTH/8-1:0]     pa_wea_axi   = wr_commit ? s_wstrb : {(DATA_WIDTH/8){1'b0}};
    wire [WORD_ADDR_W-1:0]      pa_addr_axi  = wr_commit       ? ws_byte_addr[WORD_BYTE_W +: WORD_ADDR_W] :
                                                rd_mem_fire_first ? s_araddr[WORD_BYTE_W +: WORD_ADDR_W] :
                                                rd_mem_fire_next  ? rs_next_byte_addr[WORD_BYTE_W +: WORD_ADDR_W] :
                                                                {WORD_ADDR_W{1'b0}};
    wire [DATA_WIDTH-1:0]       pa_din_axi   = s_wdata;

    // Port-A mux: clear FSM wins while `clearing`; otherwise the AXI
    // path drives.  Note ena rises for the AXI-read miss case too —
    // we never see an AXI transaction during clearing because the
    // ready signals below are gated, so pa_ena_axi is always 0 here.
    wire                        pa_ena   = clearing ? 1'b1                            : pa_ena_axi;
    wire [DATA_WIDTH/8-1:0]     pa_wea   = clearing ? {(DATA_WIDTH/8){1'b1}}          : pa_wea_axi;
    wire [WORD_ADDR_W-1:0]      pa_addr  = clearing ? clear_addr                       : pa_addr_axi;
    wire [DATA_WIDTH-1:0]       pa_din   = clearing ? {DATA_WIDTH{1'b0}}              : pa_din_axi;
    wire [DATA_WIDTH-1:0]       pa_dout;   // port-A data after XPM_READ_LATENCY

    // Track the full byte address of the last AR beat issued to port A so
    // rd_fire_next can compute the next AxSIZE-based INCR address.
    always @(posedge clk) begin
        if (rst) begin
            rs            <= RS_IDLE;
            s_rvalid      <= 1'b0;
            s_rlast       <= 1'b0;
            s_rid         <= {ID_WIDTH{1'b0}};
            s_rresp       <= 2'b00;
            s_rdata       <= {DATA_WIDTH{1'b0}};
            rs_id         <= {ID_WIDTH{1'b0}};
            rs_byte_addr  <= 32'b0;
            rs_size       <= 3'b0;
            rs_burst      <= 2'b0;
            rs_beats_left <= 8'd0;
            rs_wait_left  <= 4'd0;
            rs_wait_err   <= 1'b0;
        end else begin
            case (rs)
                RS_IDLE: begin
                    // s_arready (combinational, above) already gates
                    // acceptance on ws==WS_IDLE && !s_awvalid && !clearing.
                    s_rvalid  <= 1'b0;
                    s_rlast   <= 1'b0;
                    if (rd_fire_first) begin
                        rs_id         <= s_arid;
                        rs_byte_addr  <= s_araddr;
                        rs_size       <= s_arsize;
                        rs_burst      <= s_arburst;
                        rs_beats_left <= s_arlen;
                        rs_wait_err   <= rd_fire_first_bad;
                        rs_wait_left  <= XPM_READ_WAIT;
                        rs            <= RS_WAIT;
                    end
                end
                RS_WAIT: begin
                    s_rvalid <= 1'b0;
                    s_rlast  <= 1'b0;
                    if (rs_wait_left == 4'd1) begin
                        // pa_dout now carries the RAM read issued
                        // XPM_READ_LATENCY cycles earlier.
                        s_rvalid <= 1'b1;
                        s_rid    <= rs_id;
                        s_rresp  <= rs_wait_err ? AXI_RESP_SLVERR
                                                : AXI_RESP_OKAY;
                        s_rdata  <= rs_wait_err ? {DATA_WIDTH{1'b0}}
                                                : pa_dout;
                        s_rlast  <= (rs_beats_left == 8'd0);
                        rs_wait_left <= 4'd0;
                        rs       <= RS_RESP;
                    end else begin
                        rs_wait_left <= rs_wait_left - 4'd1;
                    end
                end
                RS_RESP: begin
                    if (s_rvalid && s_rready) begin
                        if (rs_beats_left == 8'd0) begin
                            // Last beat accepted.
                            s_rvalid <= 1'b0;
                            s_rlast  <= 1'b0;
                            rs       <= RS_IDLE;
                        end else begin
                            rs_beats_left <= rs_beats_left - 8'd1;
                            rs_wait_err   <= rd_fire_next_bad;
                            s_rvalid      <= 1'b0;
                            s_rlast       <= 1'b0;
                            if (wr_commit && !rd_fire_next_bad) begin
                                // Port A stolen by a concurrent write's
                                // data beat this cycle.  STALL — do not
                                // drop the fetch.  rs_byte_addr is left
                                // unchanged so rs_next_byte_addr keeps
                                // pointing at the same pending beat;
                                // RS_ARB retries it every cycle.
                                rs <= RS_ARB;
                            end else begin
                                rs_byte_addr <= rs_next_byte_addr;
                                rs_wait_left <= XPM_READ_WAIT;
                                rs           <= RS_WAIT;
                            end
                        end
                    end
                end
                RS_ARB: begin
                    // Retry the deferred next-beat fetch every cycle
                    // until port A is free (wr_commit low).  The
                    // XPM_READ_LATENCY countdown starts HERE — the
                    // actual issue cycle — not back at the original
                    // R-handshake cycle that lost the arbitration.
                    if (!wr_commit) begin
                        rs_byte_addr <= rs_next_byte_addr;
                        rs_wait_left <= XPM_READ_WAIT;
                        rs           <= RS_WAIT;
                    end
                end
                default: rs <= RS_IDLE;
            endcase
        end
    end

    // ═══════════════════════════════════════════════════════════════════
    // Streaming read port — `rd_clk` domain (xpm port B)
    // ═══════════════════════════════════════════════════════════════════
    // Pixel-to-word/lane math:
    //   word_idx = rd_addr / PX_PER_WORD  (high bits of rd_addr)
    //   lane_idx = rd_addr % PX_PER_WORD  (low  bits of rd_addr)
    // Since PX_PER_WORD is a power of two at all supported BPP values
    // (8 and 16 divide evenly into 128), this is a simple bit split.
    //
    // Pipeline (XPM_READ_LATENCY rd_clk cycles end-to-end):
    //   cycle N-1: rd_en + rd_addr presented on port B of URAM.
    //   cycle N+L: XPM port-B pipelined output (doutb) has the FULL
    //              DATA_WIDTH(=128)-bit word; a combinational RD_DATA_W-wide
    //              gather picks FOUR consecutive lanes starting at the
    //              equally-delayed `rd_lane_sel_pipe` value.
    //
    // The 128-bit word was always being read here — the old mux threw away
    // 15 of its 16 bytes.  Widening the output to a 4-byte group therefore
    // adds NO URAM read, NO second port, and NO extra latency: it is purely
    // a wider output mux (4 × PX_PER_WORD-input byte muxes instead of 1).
    //
    // Alignment contract (see the port-list block near the top of the file
    // for the authoritative wording): rd_data[31:24] (the byte at rd_addr)
    // is valid at ANY alignment and is bit-identical to the pre-widening
    // single-lane result; the remaining three bytes are valid only when
    // rd_addr[1:0]==2'b00, where the 4-byte group provably cannot cross the
    // 16-byte URAM word boundary and so needs no straddle read.  For an
    // unaligned rd_addr the upper bytes wrap inside the same word and are
    // don't-care — the wrap is deliberate so no out-of-range part-select
    // can ever be formed.
    //
    // That is: rd_data is valid XPM_READ_LATENCY rd_clk edges after rd_en
    // was sampled high.  fb_reader.v tolerates this inside its larger
    // fixed response window.

    wire [WORD_ADDR_W-1:0]   rd_word_idx;
    wire [PX_IN_WORD_W-1:0]  rd_lane_sel;
    reg  [PX_IN_WORD_W-1:0]  rd_lane_sel_pipe [0:XPM_READ_LATENCY-1];
    reg  [XPM_READ_LATENCY-1:0] rd_en_pipe;
    integer                  rd_pipe_i;

    // Split rd_addr into word index (high bits) and lane selector (low).
    // For power-of-two PX_PER_WORD (the always-true case here), this is
    // pure wire slicing, no divider.  When VRAM_BYTES > FB_BYTES (e.g. the
    // 2 MiB Q700 aperture with a 768 KB framebuffer), WORD_ADDR_W exceeds
    // (PX_ADDR_W - PX_IN_WORD_W); zero-extend the FB-side slice so it
    // addresses the framebuffer prefix of the larger VRAM bank.
    localparam integer FB_WORD_IDX_W = (PX_ADDR_W > PX_IN_WORD_W) ?
                                       (PX_ADDR_W - PX_IN_WORD_W) : 1;
    generate
        if (PX_IN_WORD_W >= PX_ADDR_W) begin : gen_addr_all_lane
            // Pathological: the whole pixel address fits inside one word.
            // Only happens for tiny frames; collapse word_idx to 0.
            assign rd_word_idx = {WORD_ADDR_W{1'b0}};
            assign rd_lane_sel = rd_addr[PX_IN_WORD_W-1:0];
        end else if (FB_WORD_IDX_W >= WORD_ADDR_W) begin : gen_addr_full
            assign rd_word_idx = rd_addr[PX_ADDR_W-1 : PX_IN_WORD_W];
            assign rd_lane_sel = rd_addr[PX_IN_WORD_W-1:0];
        end else begin : gen_addr_split
            assign rd_word_idx = {{(WORD_ADDR_W-FB_WORD_IDX_W){1'b0}},
                                  rd_addr[PX_ADDR_W-1 : PX_IN_WORD_W]};
            assign rd_lane_sel = rd_addr[PX_IN_WORD_W-1:0];
        end
    endgenerate

    // Delay rd_en and the lane selector so they align with XPM's pipelined
    // URAM doutb.
    always @(posedge rd_clk) begin
        if (rd_rst) begin
            rd_en_pipe <= {XPM_READ_LATENCY{1'b0}};
            for (rd_pipe_i = 0; rd_pipe_i < XPM_READ_LATENCY; rd_pipe_i = rd_pipe_i + 1)
                rd_lane_sel_pipe[rd_pipe_i] <= {PX_IN_WORD_W{1'b0}};
        end else begin
            rd_en_pipe[0]       <= rd_en;
            rd_lane_sel_pipe[0] <= rd_lane_sel;
            for (rd_pipe_i = 1; rd_pipe_i < XPM_READ_LATENCY; rd_pipe_i = rd_pipe_i + 1) begin
                rd_en_pipe[rd_pipe_i]       <= rd_en_pipe[rd_pipe_i-1];
                rd_lane_sel_pipe[rd_pipe_i] <= rd_lane_sel_pipe[rd_pipe_i-1];
            end
        end
    end

    wire [DATA_WIDTH-1:0] pb_dout;  // URAM port-B output word after XPM_READ_LATENCY

    // ── 4-byte gather on the registered URAM output ────────────────────
    // Purely combinational on doutb: RD_GROUP_LANES independent BPP-wide
    // muxes, each with PX_PER_WORD inputs, all sharing the same 128-bit
    // word that was already read.  Vivado collapses each into LUT6 after
    // place, so the cost over the old single-lane mux is the extra three
    // muxes and nothing else.
    //
    // Lane k (k = 0..RD_GROUP_LANES-1) is the byte at rd_addr + k, WRAPPED
    // inside the URAM word by LANE_WRAP_MASK.  PX_PER_WORD is a power of
    // two, so the modulo is a bit-mask (never a `%` on a variable — that
    // would not be synthesisable/lint-clean).  The wrap also guarantees no
    // part-select can run off the end of pb_dout for an unaligned lane,
    // where those upper bytes are architecturally don't-care anyway.
    //
    // Byte order: k=0 lands in rd_data[31:24] and k=RD_GROUP_LANES-1 in
    // rd_data[BPP-1:0] — increasing address toward the LSB, matching how
    // the 24bpp consumer expects {pad,R,G,B} out of an aligned xRGB word.
    localparam integer            RD_GROUP_LANES = RD_DATA_W / BPP;   // 4 at 32/8
    // All-ones over the lane index, i.e. PX_PER_WORD-1 -- written as a
    // replication rather than `PX_PER_WORD - 1` so the initialiser is
    // natively PX_IN_WORD_W bits wide (the subtraction is 32-bit and
    // truncating it trips -Wall WIDTHTRUNC).  PX_PER_WORD is 2**PX_IN_WORD_W
    // by construction, so the two are identical.
    localparam [PX_IN_WORD_W-1:0] LANE_WRAP_MASK = {PX_IN_WORD_W{1'b1}};

    wire [PX_IN_WORD_W-1:0] rd_lane_out = rd_lane_sel_pipe[XPM_READ_LATENCY-1];

    genvar rd_gk;
    generate
        for (rd_gk = 0; rd_gk < RD_GROUP_LANES; rd_gk = rd_gk + 1) begin : gen_rd_byte_gather
            // Sized so the add wraps mod PX_PER_WORD; the explicit mask
            // documents (and enforces) that wrap.
            localparam [PX_IN_WORD_W-1:0] LANE_OFF = rd_gk;
            wire [PX_IN_WORD_W-1:0] lane_k = (rd_lane_out + LANE_OFF) & LANE_WRAP_MASK;
            assign rd_data[(RD_DATA_W - BPP) - (rd_gk * BPP) +: BPP] =
                pb_dout[lane_k * BPP +: BPP];
        end
    endgenerate

    assign rd_valid = rd_en_pipe[XPM_READ_LATENCY-1];

    // ═══════════════════════════════════════════════════════════════════
    // Storage primitive
    // ═══════════════════════════════════════════════════════════════════
    // Under Verilator: a plain 2D reg array with the same protocol the
    // xpm primitive would have provided (synchronous-read port A, port B
    // with XPM_READ_LATENCY-cycle output pipelines).
    //
    // Under synthesis: xpm_memory_tdpram with MEMORY_PRIMITIVE="ultra"
    // and CLOCKING_MODE="common_clock" (Vivado's xpm_memory wrapper
    // refuses CLOCKING_MODE="independent_clock" when MEMORY_PRIMITIVE=
    // "ultra"), with both `.clka`/`.clkb` tied to `clk`.  See the
    // single-clock header note at the top of this file.

`ifdef VERILATOR
    // ── Behavioural sim model (pipelined registered read on each port) ─
    reg [DATA_WIDTH-1:0] mem [0:N_WORDS-1];
    reg [DATA_WIDTH-1:0] pa_dout_pipe [0:XPM_READ_LATENCY-1];
    reg [DATA_WIDTH-1:0] pb_dout_pipe [0:XPM_READ_LATENCY-1];
    integer pa_pipe_i;
    integer pb_pipe_i;

    // Port-A: byte-enable write + registered read.
    always @(posedge clk) begin
        if (pa_ena) begin
            // Byte-enable merge: lanes with wea[b]=1 take pa_din, others
            // preserve the existing mem content.  This is the exact
            // behaviour of an xpm tdpram with BYTE_WRITE_WIDTH=8.
            begin : do_pa_write
                integer b;
                reg [DATA_WIDTH-1:0] new_w;
                new_w = mem[pa_addr];
                for (b = 0; b < BYTES_PER_WORD; b = b + 1) begin
                    if (pa_wea[b])
                        new_w[b*8 +: 8] = pa_din[b*8 +: 8];
                end
                mem[pa_addr] <= new_w;
            end
        end
        // WRITE_MODE_A = "no_change" semantics: on a write cycle, the read
        // pipeline input retains its previous value.  On a read cycle
        // (pa_ena && pa_wea==0), stage 0 gets the newly-addressed word.
        if (pa_ena && (pa_wea == {(DATA_WIDTH/8){1'b0}})) begin
            pa_dout_pipe[0] <= mem[pa_addr];
        end
        for (pa_pipe_i = 1; pa_pipe_i < XPM_READ_LATENCY; pa_pipe_i = pa_pipe_i + 1)
            pa_dout_pipe[pa_pipe_i] <= pa_dout_pipe[pa_pipe_i-1];
    end
    assign pa_dout = pa_dout_pipe[XPM_READ_LATENCY-1];

    // Port-B: read-only registered.  Clocked on `clk`, NOT `rd_clk` — the
    // real xpm_memory_tdpram instance below is CLOCKING_MODE=
    // "common_clock" and hard-wires `.clkb(clk)`, so this behavioural
    // model must match that or sim would diverge from synthesis whenever
    // an instantiator's `rd_clk` net happened to glitch relative to
    // `clk` (it never legitimately does — rd_clk MUST equal clk — but a
    // sim model keyed on the wrong port name previously masked bugs that
    // only a genuinely-common clock reproduces).  `rd_clk`/`rd_rst`
    // remain module ports for API stability; they are otherwise unused
    // by this behavioural array.
    always @(posedge clk) begin
        if (rd_en)
            pb_dout_pipe[0] <= mem[rd_word_idx];
        for (pb_pipe_i = 1; pb_pipe_i < XPM_READ_LATENCY; pb_pipe_i = pb_pipe_i + 1)
            pb_dout_pipe[pb_pipe_i] <= pb_dout_pipe[pb_pipe_i-1];
    end
    assign pb_dout = pb_dout_pipe[XPM_READ_LATENCY-1];

    // Suppress unused-signal lint on rd_rst/rd_clk (both kept as ports
    // for API stability; the rd_en_pipe/rd_lane_sel_pipe pipeline above
    // still clocks on rd_clk, which every instantiator ties = clk).
`else
    // ── Synthesis: xpm_memory_tdpram ───────────────────────────────────
    // URAM288 backing.  Port A serves AXI writes + AXI reads.  Port B
    // is read-only streaming.  Byte-write-enable on port A gives us
    // per-AXI-lane wstrb masking natively, so no read-modify-write is
    // needed on the RAM side (the old inferred-path approach forced a
    // LUT-merge that Vivado ALSO used to fall back to distributed RAM
    // whenever the two ports landed on different clocks).
    //
    // READ_LATENCY_A / _B = XPM_READ_LATENCY lets XPM/Vivado put pipeline
    // registers on the URAM matrix path instead of leaving the cascaded
    // UltraRAM array with only the primitive output register.

    xpm_memory_tdpram #(
        .MEMORY_SIZE        (N_WORDS * DATA_WIDTH),     // total bits
        // URAM288 hardware supports independent CLKA/CLKB, but Vivado's
        // xpm_memory wrapper refuses CLOCKING_MODE=independent_clock when
        // MEMORY_PRIMITIVE=ultra.  Keep URAM (saves ~50 BRAMs) and run
        // both ports on core_clk; the pclk crossing moves downstream to
        // a small async FIFO at the scaler input.  The `rd_clk` port is
        // retained for API stability but must be wired = clk by the
        // instantiator (fpga_top ties rd_clk to core_clk).
        .MEMORY_PRIMITIVE   ("ultra"),
        .CLOCKING_MODE      ("common_clock"),
        .ECC_MODE           ("no_ecc"),
        .MEMORY_INIT_FILE   ("none"),
        .MEMORY_INIT_PARAM  ("0"),
        .USE_MEM_INIT       (1),
        .WAKEUP_TIME        ("disable_sleep"),
        .AUTO_SLEEP_TIME    (0),
        .MESSAGE_CONTROL    (0),
        .USE_EMBEDDED_CONSTRAINT(0),
        .MEMORY_OPTIMIZATION("true"),
        .RST_MODE_A         ("SYNC"),
        .RST_MODE_B         ("SYNC"),

        // Port A — writes + AXI reads (system clk domain)
        .WRITE_DATA_WIDTH_A (DATA_WIDTH),
        .READ_DATA_WIDTH_A  (DATA_WIDTH),
        .BYTE_WRITE_WIDTH_A (8),             // per-byte wstrb
        .ADDR_WIDTH_A       (WORD_ADDR_W),
        .READ_RESET_VALUE_A ("0"),
        .READ_LATENCY_A     (XPM_READ_LATENCY),
        .WRITE_MODE_A       ("no_change"),

        // Port B — streaming reads (pclk domain)
        .WRITE_DATA_WIDTH_B (DATA_WIDTH),
        .READ_DATA_WIDTH_B  (DATA_WIDTH),
        .BYTE_WRITE_WIDTH_B (DATA_WIDTH),
        .ADDR_WIDTH_B       (WORD_ADDR_W),
        .READ_RESET_VALUE_B ("0"),
        .READ_LATENCY_B     (XPM_READ_LATENCY),
        .WRITE_MODE_B       ("no_change")
    ) u_uram (
        // Common
        .sleep          (1'b0),

        // Port A
        .clka           (clk),
        .rsta           (rst),
        .ena            (pa_ena),
        .regcea         (1'b1),
        .wea            (pa_wea),
        .addra          (pa_addr),
        .dina           (pa_din),
        .injectsbiterra (1'b0),
        .injectdbiterra (1'b0),
        .douta          (pa_dout),
        .sbiterra       (),
        .dbiterra       (),

        // Port B (read-only).  CLOCKING_MODE=common_clock (URAM constraint)
        // forces clkb = clka.  The module's `rd_clk` port is kept for API
        // stability but MUST be wired = clk by the instantiator.  CDC to
        // pclk is handled DOWNSTREAM via an async FIFO at the scaler
        // input — not inside this module.
        .clkb           (clk),
        .rstb           (rst),
        .enb            (rd_en),
        .regceb         (1'b1),
        .web            ({DATA_WIDTH/DATA_WIDTH{1'b0}}),  // 1 bit, tied-off
        .addrb          (rd_word_idx),
        .dinb           ({DATA_WIDTH{1'b0}}),
        .injectsbiterrb (1'b0),
        .injectdbiterrb (1'b0),
        .doutb          (pb_dout),
        .sbiterrb       (),
        .dbiterrb       ()
    );
`endif

    // ── Parameter sanity checks (elaboration-time) ─────────────────────
    // Simulation-only guards; synthesis tools ignore `initial` blocks
    // outside of memory initialisation.  The BPP > 16 guard above is a
    // generate-block `$error` that DOES fire at synth elab.
    initial begin
        if (BPP != 8 && BPP != 16) begin
            $display("[vram] FATAL: BPP=%0d not supported (legal: 8, 16).  BPP>16 exceeds KU5P URAM budget — use a DDR-backed framebuffer.", BPP);
            $finish;
        end
        if ((DATA_WIDTH % BPP) != 0) begin
            $display("[vram] FATAL: DATA_WIDTH (%0d) must be an integer multiple of BPP (%0d).", DATA_WIDTH, BPP);
            $finish;
        end
        if (PX_PER_WORD < 1) begin
            $display("[vram] FATAL: PX_PER_WORD < 1");
            $finish;
        end
    end

endmodule
