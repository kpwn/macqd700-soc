// MAME reference/excerpt/adaptation attribution: Copyright R. Belmont.
// MAME-derived portions retain BSD-3-Clause; original contributions are MIT.
// See THIRD_PARTY_NOTICES.md and LICENSES/MAME-BSD-3-Clause.txt.
// asc.v — Apple Sound Chip — Quadra 700 audio block
//
// CHIP-VARIANT NOTE (memory: project_q700_easc_not_sonora.md):
// The Q700 ships an **EASC** (Enhanced Apple Sound Chip, 343S1036),
// NOT Sonora (343S1065).  MAME models them as separate devices —
// `asc_easc_device` (asc.cpp:1419-1771) for Q700 (machine_config at
// macquadra700.cpp:805) and `asc_sonora_device` (asc.cpp:910-1117) for
// the LC III generation.  This RTL implements a hybrid that supports
// both the Sonora-style FIFO playback path used by the Sound Manager
// AND the EASC/first-gen ASC wavetable mode (R_MODE = 2) used by the
// Q700 boot ROM's startup-chime synthesis loop.  The wavetable path
// follows the canonical first-gen `asc_base_device::sound_stream_update`
// case 2 (asc.cpp:248-282): 4 voices, each reading from a 512-byte
// slot in the FIFO ram at offsets 0 / 0x200 / 0x400 / 0x600, indexed
// by `(phase[v] >> 15) & 0x1ff` with a 24-bit phase counter advanced
// by a 24-bit increment register per sample tick.
//
// R_VERSION (0x800) returns **0xB0** per MAME's
// `asc_easc_device::get_version()` (asc.cpp:1768-1771: `u8
// asc_easc_device::get_version() { return 0xb0; }`) — directly returned
// by `asc_base_device::read` at asc.cpp:312-313 (`case R_VERSION: return
// get_version();`).  Earlier doc claims that "MAME returns 0x00 due to
// a `umask32` quirk" were INCORRECT — MAME's bus dispatcher passes the
// 8-bit handler's byte through unmodified; the prior diagnostic captured
// 0x00 because of a separate ROM-trace tooling issue, not a MAME quirk.
// The ROM's `tstb (R_VERSION); BEQ no_asc_path` therefore takes the
// non-zero branch (BEQ NOT taken), falls through to the chime
// synthesis path at 0x408070CE+, and writes MODE=2 (wavetable) to
// activate the engine.  See `docs/asc_q700_rom_trace.md` for the
// updated capture; the "no-ASC simple-init path that boots cleanly"
// claim is now retired — the ROM takes the chime path on both MAME
// and our RTL.
//
// FIFO storage write semantics in wave mode (R_MODE = 2):
// We follow the first-gen ASC's `asc_device::write` (asc.cpp:669-742)
// which performs **direct-addressed** writes when not in FIFO mode —
// `m_fifo[0][offset] = data;` (asc.cpp:700) for offsets 0..0x3FF, and
// `m_fifo[1][offset - 0x400] = data;` (asc.cpp:736) for 0x400..0x7FF.
// In FIFO mode (R_MODE = 1) writes still push at the write-pointer
// (`m_fifo_wrptr[ch]++`) per the inherited `asc_base_device::write`
// (asc.cpp:380-431), so the OS Sound Manager FIFO playback path is
// unaffected.  This dual mode is what lets the Q700 ROM's chime
// synthesis loop (PC 0x40807106-0x7112) build a 4-voice chord by
// stamping bytes at FIFO offsets 0, 0x200, 0x400, 0x600 — those bytes
// become the wavetable samples the engine walks via phase/incr.
//
// 1:1 register-level fidelity with MAME's `asc_sonora_device` +
// `asc_base_device` (thirdparty/mame/src/devices/sound/asc.cpp:910-1117)
// for the Sonora-shape (FIFO) parts, plus the four-voice wavetable
// from `asc_base_device::sound_stream_update` case 2 (asc.cpp:248-282)
// for the EASC chime path.  See MAME's STAT_* constants (asc.cpp:94-97)
// for the canonical R_FIFOSTAT bit positions.  The audio output path
// is local-faithful (raw signed 8-bit samples promoted to 16-bit for
// the downstream PWM modulator), matching MAME's `(s8)byte ^ 0x80`
// pre-scaling convention.
//
// Target chipset: Quadra 700 EASC.  ROM / Sound Manager accesses are
// byte-wide against the 4 KB window at 0x5001_4000.
//
// Register map (pb_addr is the byte offset within the 4 KB window):
//   +0x000 .. +0x3FF   FIFO A data   (write fills in FIFO mode at wrptr;
//                                       direct-addressed in wave mode)
//   +0x400 .. +0x7FF   FIFO B data   (same dual-mode write semantics)
//   +0x800             Version               (RO; current readback is 0x00
//                                              for boot-path stability —
//                                              upstream MAME returns 0xB0)
//   +0x801             Mode                  (writeable: 0=off, 1=FIFO,
//                                              2=wavetable; reads back the
//                                              programmed value)
//   +0x802             Channel control       (bit 1 = stereo)
//   +0x803             FIFO control          (bit 7 = clear both)
//   +0x804             R_FIFOSTAT (per MAME asc_sonora_device — RW;
//                      read clears IRQ line iff bit 2 is clear, but
//                      does NOT clear status bits.  Sonora bit map:
//                      bit 0 STAT_HALF_FULL_A      — write-side only on
//                            Sonora; cleared by FIFO A pushes when
//                            cap >= 0x200 (in playback mode).  Stream
//                            loop never sets it.  Effectively dormant
//                            unless software pokes 0x804 directly.
//                      bit 1 STAT_EMPTY_OR_FULL_A  — set every stream
//                            tick in playback mode (PLAYRECA bit 0=0);
//                            cap >= 0x3FF on push (playback mode);
//                            re-forced by Sonora write hook after each
//                            FIFO A push in playback mode.  Cleared by
//                            push when 0 < cap < 0x200 (playback mode).
//                      bit 2 STAT_HALF_FULL_B      — Sonora-overloaded
//                            "either A or B half" — set by stream when
//                            cap_a < 0x200 || cap_b < 0x200; cleared
//                            otherwise.
//                      bit 3 STAT_EMPTY_OR_FULL_B  — Sonora-overloaded
//                            "either A or B empty" — set when cap_a==0
//                            || cap_b==0.  Also forced on FIFO clear.
//                      bits 4..7  general-purpose scratch (writable
//                            via 0x804 store; not used by Q700 ROM).
//   +0x805             Wavetable control     (latched for readback)
//   +0x806             Volume alias          (MAME/ASC register layout)
//   +0x807             Clock selector        (0/2=22 kHz, 3=44 kHz)
//   +0x808             Rate                  (local sample-tick divider seed)
//   +0x809             reserved
//   +0x80A             Volume                (0..0xFF master attenuation)
//   +0x80B .. +0x80F   reserved
//   +0x810 .. +0x82F   Wavetable phase/incr  (8 × 4 bytes; per MAME
//                      asc.cpp:33-40 + write hooks asc.cpp:473-591:
//                        +0x810 ignored / +0x811-3 phase[0] [23:0] BE
//                        +0x814 ignored / +0x815-7 incr[0]  [23:0] BE
//                        +0x818 ignored / +0x819-B phase[1] [23:0] BE
//                        +0x81C ignored / +0x81D-F incr[1]  [23:0] BE
//                        +0x820..0x823 phase[2], +0x824..0x827 incr[2],
//                        +0x828..0x82B phase[3], +0x82C..0x82F incr[3])
//   +0xF00 .. +0xF01   FIFO A write-pointer   (RO live, hi/lo split)
//   +0xF02 .. +0xF03   FIFO A read-pointer    (RO live, hi/lo split)
//   +0xF04 .. +0xF05   FIFO A SRC step        (R/W; latched, SRC engine TBD)
//   +0xF06 .. +0xF07   FIFO A volume L/R      (R/W; latched, per-ch split TBD)
//   +0xF08             FIFO A control         (R/W; latched, CD-XA mode bits)
//   +0xF09             FIFO A IRQ control    (bit 0=disable)
//   +0xF10             FIFO A CD-XA decoder   (R/W; latched-stub)
//   +0xF20 .. +0xF21   FIFO B write-pointer   (RO live, hi/lo split)
//   +0xF22 .. +0xF23   FIFO B read-pointer    (RO live, hi/lo split)
//   +0xF24 .. +0xF25   FIFO B SRC step        (R/W; latched, SRC engine TBD)
//   +0xF26 .. +0xF27   FIFO B volume L/R      (R/W; latched, per-ch split TBD)
//   +0xF28             FIFO B control         (R/W; latched, CD-XA mode bits)
//   +0xF29             FIFO B IRQ control    (bit 0=disable)
//   +0xF30             FIFO B CD-XA decoder   (R/W; latched-stub)
//
// Sample-rate clocking:
//   A small counter decrements on `phi2_tick` pulses (nominally 1 MHz).
//   The reload seed is RATE_DEFAULT (45 → 22.222 kHz at 1 MHz phi2,
//   ≈ Sonora's hardwired 22.257 kHz).  Real Sonora has no programmable
//   rate — writes to 0x808 are ignored — but the divider is
//   reload-driven and runs at the default forever.  Each tick advances
//   the FIFO read-pointer and latches the current output sample.
//
// Sonora-specific quirks (vs first-generation ASC / EASC):
//   - Mode register (0x801) is hardwired to FIFO; writes ignored,
//     reads always return 0x01.  No silent or wavetable mode.
//   - Wavetable control (0x805) is gone; writes ignored, reads 0.
//   - Clock select (0x807) is gone; writes ignored, reads 0.
//   - 0x808 (was Rate on EASC) is R_BATMANCONTROL on Sonora;
//     writes ignored.
//   - 0x830..0x83F (EASC wavetable phase/increment) reads return 0,
//     writes ignored.
//   See MAME asc_sonora_device::{read,write} (sound/asc.cpp) for the
//   reference implementation.
//
// Mono vs stereo:
//   stereo (channel_ctl bit 1 = 1): sample N is FIFO A read-pointer N,
//       sample N+1 is FIFO B read-pointer N (they advance in lock-step
//       on successive sample_ticks; we surface {A, B} as a 16-bit sample).
//   mono: FIFO A only feeds both halves of the output sample.
//
// Volume:
//   MAME's stream emits raw `(s8)byte ^ 0x80` to the host audio sink —
//   physical attenuation is the off-chip DAC's job.  We mirror that:
//   `audio_pcm_l/r` carry the centred raw signed 8-bit sample promoted
//   to 16-bit (left-shift 8).  `volume_reg` (R_VOLUME at 0x806) is
//   programmer-visible (writable + readable) but NOT applied here —
//   the downstream `audio_pwm` modulator handles the full 16-bit range.
//
// IRQ:
//   Aggregated active-high.  Sources (per MAME asc_sonora_device):
//     - Each stream tick: assert if !PLAYRECA[0] && !fa_irqen[0]
//       (playback-mode level fire — asc.cpp:970-978).
//     - Each stream tick: assert if combined-half-full-B set &&
//       !fb_irqen[0] (asc.cpp:1004-1012).
//     - 0xE00 backdoor write: assert (asc.cpp:374-378).
//     - 0xF09 write with disable→enable edge AND STAT_HALF_FULL_A
//       set: assert (asc.cpp:1082-1093).
//     - 0xF29 write with disable→enable edge AND STAT_HALF_FULL_B
//       set: assert (asc.cpp:1094-1104).
//   Clears:
//     - 0x804 read iff !STAT_HALF_FULL_B (asc.cpp:1049-1055).
//     - 0xF09/0xF29 write with enable→disable edge.
//   Q700 board glue routes this active-high internal line to VIA2 CB1
//   as an active-low external pin input.  IRQ slot in irq_agg is L5.
//
// FIFO storage:
//   2 × 1 KB simple-DP BRAM-inferrable arrays (`reg [7:0] fifo_a[0:1023]`
//   + `reg [7:0] fifo_b[0:1023]`).  Write port clocked by pb_wr, read
//   port clocked by sample_tick.  Verilator / Vivado infer BRAM when the
//   addressing + clocking satisfies the simple-DP template.
//
// Peripheral-bus contract:
//   Match via1/via2/scsi shape.  pb_wr / pb_rd are one-cycle pulses;
//   pb_ack and pb_rdata are registered on the following edge.
//   NB: the current peripheral_bus.v passes `wr_addr[13:2]` as `pb_addr`
//   (word-granular within a 16 KB window).  The task brief specifies a
//   byte-granular `pb_addr[11:0]` — we implement the byte-granular form
//   here; peripheral_bus is expected to be updated to pass the low bits
//   (or inject a byte-select equivalent).  See followups in the landing
//   report.
//
// Reset (per MAME asc_sonora_device::device_reset):
//   m_regs zeroed by base, then m_regs[R_MODE] = 1; m_regs[R_FIFOSTAT]
//   = 0x02 (STAT_EMPTY_OR_FULL_A); m_fifo_irqen[0/1] = 0.  We mirror:
//   fifostat = 0x02, fa_irqen = fb_irqen = 0, irq_q = 0.  Local: rate
//   to the Mac default (35 ≈ 22.4 kHz at Q700 phi2 783_360 Hz), volume to
//   0xFF (full scale), IRQ line low.  FIFO contents are NOT zeroed on
//   reset (BRAM power-on state is undefined on real hardware; sim
//   initialises to 0 via `--x-initial fast`).

`default_nettype none

module asc (
    input  wire        clk,
    input  wire        rst,

    // 1 MHz VIA-style timebase (one-cycle pulse every ~1 µs).  Same
    // source that drives via1.v; the top-level clk_rst module exposes
    // it as `phi2_tick`.  The sample-rate divider is expressed in
    // phi2 units so the Rate register acts as "microseconds per sample".
    input  wire        phi2_tick,

    // Peripheral-bus slave
    input  wire [11:0] pb_addr,
    input  wire [7:0]  pb_wdata,
    input  wire        pb_wr,
    input  wire        pb_rd,
    output wire [7:0]  pb_rdata,
    output reg         pb_ack,

    // Audio sample output.  audio_sample_out is the legacy packed 8-bit
    // stereo contract: {left[7:0], right[7:0]}.  audio_pcm_l/r are signed
    // 16-bit PCM samples at the native ASC sample tick, suitable for a
    // later 48 kHz resampler / I2S bridge.
    output reg  [15:0] audio_sample_out,
    output reg  [15:0] audio_pcm_l,
    output reg  [15:0] audio_pcm_r,
    output reg         audio_sample_valid,

    // Aggregated IRQ line — wired to irq_agg L5 (snd_irq).
    output wire        irq
);

    // ── MAME R_FIFOSTAT bit constants (asc.cpp:94-97) ─────────────────
    localparam integer STAT_BIT_HALF_FULL_A     = 0;   // 0x01
    localparam integer STAT_BIT_EMPTY_OR_FULL_A = 1;   // 0x02
    localparam integer STAT_BIT_HALF_FULL_B     = 2;   // 0x04
    localparam integer STAT_BIT_EMPTY_OR_FULL_B = 3;   // 0x08

    localparam [7:0] STAT_HALF_FULL_A     = 8'h01;
    localparam [7:0] STAT_EMPTY_OR_FULL_A = 8'h02;
    localparam [7:0] STAT_HALF_FULL_B     = 8'h04;
    localparam [7:0] STAT_EMPTY_OR_FULL_B = 8'h08;

    // ── Register constants ────────────────────────────────────────────
    // R_VERSION readback.  Real EASC silicon (asc.cpp:1768-1771):
    //   u8 asc_easc_device::get_version() { return 0xb0; }
    // and `asc_base_device::read` (asc.cpp:312-313) returns the byte
    // directly.
    //
    // Iter 4: bookkeeping changes A/B/C plus structural EASC pop_fifo
    // semantics are now gated on `IS_EASC` (== `VERSION_USE == 8'hB0`).
    // With VERSION_USE = 0x00 the chip behaves as the iter-1 hybrid
    // (ASC base + Sonora-shape FIFO + first-gen wavetable) so the
    // existing tb-asc test catalogue stays green and the boot-path
    // wavetable chime continues to sound.  With VERSION_USE = 0xB0 the
    // chip flips to MAME-faithful asc_easc_device behavior:
    //   - R_MODE writes mask data & 1 (asc.cpp:1683-1690), with the
    //     bit-0-toggle FIFO state reset side-effect.
    //   - R_CLOCK reads return 3 (asc.cpp:1662-1664).
    //   - FIFO pops update STAT_HALF_FULL_x AND STAT_EMPTY_OR_FULL_x
    //     PER-CHANNEL based on cap[ch] <= 0x1ff / cap[ch] == 0
    //     (asc.cpp:1555-1576) — NOT the Sonora-overloaded combined
    //     `cap_a||cap_b` model in `asc_sonora_device::sound_stream_update`.
    //   - The Sonora "top-of-stream PLAYRECA force" (asc.cpp:970-978)
    //     and "FIFO A push post-hook re-force STAT_EMPTY_OR_FULL_A"
    //     (asc.cpp:1109-1116) are SUPPRESSED — EASC has neither.
    //   - R_FIFOA/B_IRQCTRL edge-IRQ semantics (asc.cpp:1697-1719)
    //     match Sonora bit-for-bit; existing implementation is correct
    //     for both variants and unchanged for iter 4.
    //
    // The flip is bus-safe under VERSION=0xB0 only because
    // peripheral_bus.v's wr_asc_* multi-byte FSM (landed in iter 3)
    // serializes the 16-bit MOVE.W stores to FIFOA/B_VOLUME_LR
    // (0xF06/0xF26) into back-to-back single-byte pb_wr pulses with
    // HIGH→LOW strobe walk — see tb_peripheral_bus
    // test_asc_multi_byte_word_off2 / off0 / long_off0.
    localparam [7:0] VERSION_EASC      = 8'h00;
    localparam [7:0] VERSION_EASC_REAL = 8'hB0;
    localparam [7:0] VERSION_SONORA    = 8'hBC;

    // Active version readback.  Iter-4 attempted a 0x00 → 0xB0 flip
    // with bookkeeping divergences A/B/C/D landed (R_MODE bit-0-mask,
    // R_CLOCK=3, per-channel STAT_HALF_FULL_x, suppress Sonora top-of-
    // stream / FIFO A push post-hooks, EASC reset values).  Boot
    // smoke reproduced iter-3's failure mode: q700_descriptor_selected=1
    // but MacsBug entry at retired=50000 PC=0x408be2c0 (macsbug_scc_tx_
    // poll) before any audio frame emits — audio_dump TSV stays
    // empty.  The bookkeeping fixes alone (in this commit) do NOT
    // suffice to cross the MacsBug-entry frontier under VERSION=0xB0.
    // Per task-brief escape valve: ROLL BACK VERSION ONLY, keep
    // bookkeeping fixes for iter 5.
    //
    // The plumbing (IS_EASC localparam + gated R_MODE / R_CLOCK /
    // STAT pop semantics / Sonora-only post-hook suppression / EASC
    // reset values) is in place and exercised by tb-asc scenarios
    // 40-43.  Iter 5 should diagnose the post-MacsBug frontier and
    // re-attempt the flip when the additional gating is identified.
    localparam [7:0] VERSION_USE = VERSION_EASC;
    localparam       IS_EASC     = (VERSION_USE == VERSION_EASC_REAL);

    // Mode register encodings (per MAME asc.cpp:23-24 R_MODE comment):
    //   0 = chip off (silent / disabled)
    //   1 = FIFO mode (Sonora-shape OS Sound Manager playback)
    //   2 = wavetable mode (4-voice mixer driven by phase/incr regs;
    //       used by the Q700 boot ROM startup-chime synthesis loop)
    // Per first-gen ASC `asc_base_device::write` (asc.cpp:439-440)
    // `data &= 3` so all 2 bits are stored.  EASC's override
    // (asc.cpp:1683-1690) restricts to bit 0 only, dropping wavetable —
    // we DO support wavetable here because the Q700 ROM relies on it
    // (write at PC 0x408070B2: `moveb #2, %a3@(2049)`).
    localparam [7:0] MODE_SILENT    = 8'h00;
    localparam [7:0] MODE_FIFO      = 8'h01;
    localparam [7:0] MODE_WAVE      = 8'h02;

    // Default Mac sample-rate divider for the Q700 VIA timebase.
    //   VIA_PHI2_HZ = 783_360 (Q700-faithful, NOT 1 MHz).
    //   Sonora hardwired stream rate ≈ 22_257 Hz.
    //   783_360 / 22_257 ≈ 35.197  → RATE_DEFAULT = 35  → 22_381 Hz
    //   (0.56% sharp vs Sonora silicon — well below audible).
    // The earlier value of 45 was based on the wrong assumption that
    // phi2 was 1 MHz; that produced ~17 kHz playback, ~22% slow.
    localparam [7:0] RATE_DEFAULT   = 8'd35;
    localparam [7:0] RATE_44KHZ     = 8'd18;   // 783360/44100 ≈ 17.76

    // ── FIFO storage (BRAM-inferrable) ────────────────────────────────
    // ROOT CAUSE of the long-standing LUTRAM fallback (fixed 2026-09-12):
    // the arrays were not BRAM-shaped.  A single flat `fifo_a[0:1023]`
    // carried FOUR concurrent read ports plus a write port —
    //     R1  sample_a_raw <= fifo_a[fa_rp]              (FIFO-mode pop)
    //     R2  wt_byte0_r   <= fifo_a[{1'b0, phase0}]     (wavetable v0)
    //     R3  wt_byte1_r   <= fifo_a[{1'b1, phase1}]     (wavetable v1)
    //     R4  pb_rdata     <= fifo_a[pb_addr[9:0]]       (CPU readback)
    // R2 and R3 are unconditionally simultaneous (both voices sample on
    // the same sample_tick), and R4 lives in a different always block.
    // A Xilinx BRAM has at most TWO ports, so no attribute can map this;
    // Vivado said so explicitly every build:
    //     [Synth 8-6849] Infeasible attribute ram_style = "block" set
    //     for RAM "u_asc/fifo_a_reg", trying to implement using LUTRAM
    // — 10 such warnings per array.  Adding the attribute (2026-05-22)
    // and registering the reads (task T8 fix 3) were both necessary but
    // neither addressed the port count, so 1152 LUTs of distributed RAM
    // survived in the routed design.
    //
    // Fix: give every read its own physical port by splitting the write
    // fan-out across three arrays per channel.  Storage doubles (16 Kbit
    // -> 32 Kbit per channel) but BRAM is the abundant resource here.
    //
    //   fifo_X       [0:1023]  TRUE DUAL PORT
    //                          port A = CPU write / CPU readback
    //                                   (pb_wr and pb_rd are mutually
    //                                    exclusive, so they share a port)
    //                          port B = FIFO-mode sample read at f_rp
    //   fifo_X_wt0   [0:511]   write-mirror of the LOW  half; one read
    //                          port for the even wavetable voice
    //   fifo_X_wt1   [0:511]   write-mirror of the HIGH half; one read
    //                          port for the odd  wavetable voice
    //
    // The wavetable voices only ever read their own 512-byte slot
    // (voice 0 -> fifo_a[0x000..0x1FF], voice 1 -> fifo_a[0x200..0x3FF],
    // likewise 2/3 on fifo_b), so the half-split is exact — no address
    // is reachable from more than one mirror.  Every read keeps the SAME
    // cycle and the SAME enable it had before, so this is a pure
    // structural change: audio timing is bit-identical, not merely
    // "inaudibly different".  Expect ~6 RAMB18 (~3 RAMB36 equivalent),
    // which is what the 2026-05-22 note budgeted for in the first place.
    (* ram_style = "block" *) reg [7:0] fifo_a     [0:1023];
    (* ram_style = "block" *) reg [7:0] fifo_a_wt0 [0:511];
    (* ram_style = "block" *) reg [7:0] fifo_a_wt1 [0:511];
    (* ram_style = "block" *) reg [7:0] fifo_b     [0:1023];
    (* ram_style = "block" *) reg [7:0] fifo_b_wt0 [0:511];
    (* ram_style = "block" *) reg [7:0] fifo_b_wt1 [0:511];

    // BRAM port-A output registers for the CPU readback path.  These are
    // dedicated regs (a BRAM output register may not have any other
    // driver) and are muxed onto pb_rdata by the read block below.
    reg [7:0] fa_cpu_dout, fb_cpu_dout;

    // Write / read pointers — 10 bits wide (1024 entries).  Depth is
    // tracked as (wp - rp) modulo 1024; half-empty = (depth < 512).
    reg [9:0] fa_wp, fa_rp;
    reg [9:0] fb_wp, fb_rp;

    // Fullness counters: explicit so half-empty compare is trivial.  10
    // bits covers 0..1023; 1024 = "full" is tracked as wp==rp + wp_wrap.
    // We use a slightly simpler model: count how many bytes have been
    // written but not yet read since reset.  Clamped to [0, 1024].
    reg [10:0] fa_count;   // 0..1024
    reg [10:0] fb_count;

    // R_FIFOSTAT — full 8-bit register, mirrors MAME's m_regs[R_FIFOSTAT].
    // Low 4 bits have hardware semantics (STAT_*); high 4 are scratch
    // (writable via 0x804 store per MAME's catch-all m_regs assignment).
    reg [7:0] fifostat;

    // IRQ output flop.  MAME models the IRQ line via set_irq_line(state)
    // — a single bit, set/cleared by various event sites.  We mirror it
    // with a registered flop.
    reg irq_q;

    // Sticky "PLAYRECA has been programmed at least once" flag.  Gates
    // the playback-mode level-fire IRQ so the line stays quiescent
    // during the Q700 boot ROM (which never writes 0x80A).  Once the
    // OS Sound Manager configures PLAYRECA, MAME-equivalent stream-
    // update IRQ semantics take effect.  Set sticky on any write to
    // 0x80A, regardless of value.  See docs/asc_q700_rom_trace.md
    // findings #1 and #2 for the empirical observation that drove
    // this gate.
    reg playreca_written;

    // Half-empty edge-latched flags (local bring-up convention; not
    // MAME).  Surfaced via 0x810/0x811 status registers and OR'd into
    // the IRQ line as a level-fire on downward 512 cross.  Cleared on
    // 0x804 read or per-FIFO status read.
    reg fa_half_empty_irq;
    reg fb_half_empty_irq;

    // Sticky fault state (local bring-up; not MAME).  Surfaced via
    // 0x810/0x811; do NOT raise IRQ on their own.
    reg fa_overflow_sticky, fb_overflow_sticky;
    reg fa_underflow_sticky, fb_underflow_sticky;

    // Edge detect: we need the previous cycle's half-empty boolean to
    // detect the downward crossing.  `fa_was_above` latches whether the
    // count was ≥ 512 last cycle.
    reg fa_was_above, fb_was_above;

    // ── Control registers ─────────────────────────────────────────────
    // mode_reg is hardwired to MODE_FIFO post-reset (Sonora ignores
    // mode writes), but kept as a reg to preserve the existing
    // fifo_active wire / readback decode.
    reg [7:0] mode_reg;
    reg [7:0] chan_ctl;      // R_CONTROL — Sonora reads 0; we keep this
                             //   writable as a tb backdoor for the
                             //   stereo presentation path.  Reads via
                             //   0x802 still return 0 to match MAME.
    reg [7:0] fifo_ctl;      // bit 7 latches "clear FIFOs" on write
    reg [7:0] fa_irqen;      // R_FIFOA_IRQCTRL (0xF09).  Bit 0 = 1 means
                             //   "IRQ disabled" per MAME convention.
    reg [7:0] fb_irqen;      // R_FIFOB_IRQCTRL (0xF29)
    reg [7:0] test_reg;      // R_TEST (0x80F) — writable scratch
    // rate_reg is hardwired to RATE_DEFAULT post-reset on Sonora;
    // 0x808 writes are ignored (R_BATMANCONTROL on real silicon).
    reg [7:0] rate_reg;
    reg [7:0] volume_reg;    // R_VOLUME — writable + readable, but NOT
                             //   applied to audio output (MAME emits
                             //   raw signed sample; off-chip codec
                             //   handles attenuation).
    // R_PLAYRECA at +0x80A.  Bit 0 = 0  → "playback mode" — Sonora
    // forces STAT_EMPTY_OR_FULL_A high on every stream tick so the CPU
    // sees a constantly-empty FIFO A and keeps refilling, and asserts
    // the FIFO A IRQ at the same cadence so an IRQ-driven chime feeder
    // gets serviced.  Bit 0 = 1  → record mode (FIFO A passes through
    // status normally).  Reset value = 0x00 → playback enabled, which
    // matches MAME asc_base_device::device_reset (memset(m_regs, 0)).
    reg [7:0] playreca_reg;

    // ── Wavetable phase / increment registers (0x810..0x82F) ─────────
    // Per MAME asc.cpp:33-40 + write hooks asc.cpp:473-591, each voice
    // has a 24-bit phase + 24-bit increment, packed big-endian into 4
    // consecutive bytes (the high byte of the 4-byte slot is ignored on
    // both read and write).  Phase is treated as 9.15 fixed-point —
    // `(phase >> 15) & 0x1ff` produces a 9-bit lookup index into the
    // 512-byte voice slot.  Increments accumulate into phase on every
    // sample tick while in MODE_WAVE.  Reset value is 0 per the base
    // device's `memset(m_phase, 0, ...)` at asc.cpp:152-153.
    reg [23:0] wt_phase [0:3];
    reg [23:0] wt_incr  [0:3];
    // Registered wavetable sample bytes — see the synchronous read at
    // the sample_tick site below (task T8 fix 3: async fifo_a/fifo_b
    // reads defeated BRAM inference on the `ram_style="block"` arrays).
    reg [7:0]  wt_byte0_r, wt_byte1_r, wt_byte2_r, wt_byte3_r;
    // Post-increment phase, computed one tick early (combinational) so
    // the fifo_a/fifo_b read address is a plain wire, not a bit-select
    // on an inline expression (`{concat}[bit]` / expr[bit] selects are
    // avoided project-wide — some synth front-ends reject them).
    wire [23:0] wt_phase_next0 = wt_phase[0] + wt_incr[0];
    wire [23:0] wt_phase_next1 = wt_phase[1] + wt_incr[1];
    wire [23:0] wt_phase_next2 = wt_phase[2] + wt_incr[2];
    wire [23:0] wt_phase_next3 = wt_phase[3] + wt_incr[3];

    // ── EASC extended-register block (0xF00..0xF3F) ───────────────────
    // MAME's asc_base_device::write latches every byte landing in the
    // 0x800..0xFFF window into m_regs[offset-0x800] (with R_FIFOA/B_
    // IRQCTRL handled specially in asc_sonora_device::write).  Reads
    // return the same shadowed bytes — except WRPTRA/RDPTRA/WRPTRB/
    // RDPTRB, which we surface as live FIFO pointers per the task brief
    // (this is an enhancement on top of MAME, which only shadows them).
    //
    // R_SRCA/B   — sample-rate-converter step value (16-bit big-endian
    //              fixed-point per asc.h, "0x7fff = step 0x8000 ≈ 22 kHz").
    //              We don't have an SRC engine; latched-only.
    // R_VOLA/B_L/R — per-channel L/R volume on Sonora's stereo path.  We
    //              currently apply a single master volume_reg (0x806) to
    //              both halves; the per-channel split is deferred but
    //              the registers are software-visible.
    // R_FIFOA/B_CTRL — bits[1:0] select CD-XA decode mode (0=PCM, 1=8:1,
    //              2=4:1, 3=reserved); bit 7 enables CD-XA decode.  We
    //              don't decode CD-XA, so this is latched-only too.
    // R_CDXA_A/B — real Sonora performs CD-XA ADPCM decode through this
    //              register; we stub as a single latched byte (interface-
    //              compatible, decode side-effect deferred).
    reg [7:0] r_srca_h, r_srca_l;
    reg [7:0] r_vola_l, r_vola_r;
    reg [7:0] r_fifoa_ctrl;
    reg [7:0] r_cdxa_a;
    reg [7:0] r_srcb_h, r_srcb_l;
    reg [7:0] r_volb_l, r_volb_r;
    reg [7:0] r_fifob_ctrl;
    reg [7:0] r_cdxa_b;
    // Shadow latches for WRPTRA/B + RDPTRA/B writes.  Real silicon
    // accepts the byte (MAME's base latches it into m_regs[]), but the
    // live pointer surfaces on read — so the shadow only matters if a
    // future PR ever lets software seed the pointer directly.  Kept for
    // strict MAME-shape parity with the m_regs[] window even though
    // they're not consulted by any read path today.
    reg [7:0] r_wrptra_h_shadow, r_wrptra_l_shadow;
    reg [7:0] r_rdptra_h_shadow, r_rdptra_l_shadow;
    reg [7:0] r_wrptrb_h_shadow, r_wrptrb_l_shadow;
    reg [7:0] r_rdptrb_h_shadow, r_rdptrb_l_shadow;

    wire stereo_mode = chan_ctl[1];
    // Mode-active wires.  fifo_active gates FIFO push/pop and the OS
    // Sound Manager playback path.  wave_active gates the 4-voice
    // wavetable engine.  sample_active is true in either of the two
    // audio-emitting modes (FIFO or WAVE) so the sample-rate divider
    // continues to clock the audio output stream regardless of which
    // engine is driving the samples.  MODE_SILENT (0) suppresses the
    // sample tick entirely.
    wire fifo_active   = (mode_reg == MODE_FIFO);
    wire wave_active   = (mode_reg == MODE_WAVE);
    wire sample_active = fifo_active | wave_active;
    wire fa_fault_sticky = fa_overflow_sticky | fa_underflow_sticky;
    wire fb_fault_sticky = fb_overflow_sticky | fb_underflow_sticky;

    // ── Sample-rate tick generator ────────────────────────────────────
    // Decrement `rate_cnt` on every phi2_tick; when it reaches zero
    // emit a combinational `sample_tick` on the SAME cycle as the phi2
    // pulse and reload.  When `rate_reg` is 0 we treat it as 1 (avoid
    // divide-by-zero stall).  Combinational firing lets the FIFO-pointer
    // update and the audio-sample latch land on the same edge as the
    // phi2_tick observation — important for the unit-tb which checks
    // audio_sample_valid immediately after a one-cycle phi2 pulse.
    reg [7:0] rate_cnt;

    wire [7:0] rate_seed = (rate_reg == 8'd0) ? RATE_DEFAULT : rate_reg;
    wire       sample_tick = phi2_tick && sample_active && (rate_cnt <= 8'd1);

    // EASC's sample rate is hardwired (~22.257 kHz on real silicon).
    // We model it with a phi2-clocked divider seeded from RATE_DEFAULT
    // (35 → Q700 phi2 783_360 / 35 ≈ 22_381 Hz).  The Q700 ROM never
    // writes 0x808, but the unit-tb does — writes to 0x808 reload
    // rate_cnt immediately so the next sample fires after the chosen
    // count.  Both FIFO and WAVE mode share the same divider.
    always @(posedge clk) begin
        if (rst) begin
            rate_cnt <= RATE_DEFAULT;
        end else if (pb_wr && pb_addr == 12'h808) begin
            rate_cnt <= (pb_wdata == 8'd0) ? RATE_DEFAULT : pb_wdata;
        end else if (phi2_tick && sample_active) begin
            if (rate_cnt <= 8'd1)
                rate_cnt <= rate_seed;
            else
                rate_cnt <= rate_cnt - 8'd1;
        end
    end

    // ── Sample read ports (BRAM-inferrable) ───────────────────────────
    // Combinational-read model: we latch the output sample on sample_tick
    // by reading fifo_a[fa_rp] / fifo_b[fb_rp] synchronously.  Verilator
    // infers ROM/RAM from the indexed read-in-always pattern.
    reg [7:0] sample_a_raw;
    reg [7:0] sample_b_raw;

    // EASC-only: a R_MODE (0x801) bit-0 toggle clears fa_count/fb_count
    // (see the 8'h01 write-case below) same as the R_FIFOMODE (0x803)
    // clear-bit write.  Both must gate the fa_count_next/fb_count_next
    // catch-all below (task T8 fix 4) — without this, the catch-all
    // (which lands LAST in program order, so wins under Verilog's
    // last-nonblocking-assignment-wins rule) silently overwrites the
    // mode-toggle's fa_count<=0/fb_count<=0 with the pre-clear
    // fa_count_next/fb_count_next on the very same cycle.  Dead today
    // (IS_EASC=0 in the Sonora build) but load-bearing once the
    // version register flips to 0xB0.
    wire easc_mode_toggle_write = IS_EASC && pb_wr && pb_addr == 12'h801
                                && (pb_wdata[0] != mode_reg[0]);
    wire fifo_clear_write = (pb_wr && pb_addr == 12'h803 && pb_wdata[7])
                          || easc_mode_toggle_write;
    // Gate pops on `fifo_active` so wave mode does not drain FIFO data.
    wire fa_sample_pop = sample_tick && fifo_active && (fa_count > 11'd0);
    // EASC/Sonora FIFO mode consumes both FIFO channels.  The channel
    // control stereo bit affects presentation, not whether FIFO B drains.
    wire fb_sample_pop = sample_tick && fifo_active && (fb_count > 11'd0);
    // FIFO mode pushes at the write-pointer; cap saturates at 1024.
    wire fa_cpu_push = fifo_active && pb_wr && pb_addr[11:10] == 2'b00
                     && ((fa_count < 11'd1024) || fa_sample_pop);
    wire fb_cpu_push = fifo_active && pb_wr && pb_addr[11:10] == 2'b01
                     && ((fb_count < 11'd1024) || fb_sample_pop);
    // Wave mode: direct-addressed FIFO ram writes (per first-gen ASC
    // `asc_device::write` asc.cpp:700 / 736 — `m_fifo[ch][offset] =
    // data;`).  No wrptr bump, no count tracking — the wavetable engine
    // treats FIFO ram as a static lookup table.
    wire fa_wave_write = wave_active && pb_wr && pb_addr[11:10] == 2'b00;
    wire fb_wave_write = wave_active && pb_wr && pb_addr[11:10] == 2'b01;

    // ── FIFO RAM ports ────────────────────────────────────────────────
    // One write stream per channel (FIFO push at the write pointer, or a
    // wave-mode direct-addressed stamp), fanned out to the main array and
    // to the matching 512-byte wavetable mirror.  fa_cpu_push and
    // fa_wave_write are mutually exclusive (fifo_active vs wave_active),
    // so a plain OR reproduces the old if/else-if priority exactly.
    wire        fa_mem_we    = fa_cpu_push | fa_wave_write;
    wire [9:0]  fa_mem_waddr = fa_cpu_push ? fa_wp : pb_addr[9:0];
    wire        fb_mem_we    = fb_cpu_push | fb_wave_write;
    wire [9:0]  fb_mem_waddr = fb_cpu_push ? fb_wp : pb_addr[9:0];

    // CPU readback selects (port A read).  pb_wr wins the port on the
    // rare cycle both are asserted; the peripheral bus never does that.
    wire        fa_cpu_rd    = pb_rd && pb_addr[11:10] == 2'b00;
    wire        fb_cpu_rd    = pb_rd && pb_addr[11:10] == 2'b01;

    // ── Port A / port B write-read collision ──────────────────────────
    // On a block RAM the port-B read data is UNDEFINED when port A
    // writes the same address on the same edge (Xilinx guarantees the
    // stored data, not the read data).  Distributed RAM has no such
    // hazard, so this is the one behaviour the LUTRAM implementation got
    // for free and a naive BRAM port would lose.  It is reachable here:
    // fa_cpu_push permits a push into a brim-full FIFO exactly on the
    // tick that pops (its `|| fa_sample_pop` term), and at
    // fa_count == 1024 the pointers are equal, so fa_wp == fa_rp.  With
    // the Sonora playback hook forcing STAT_EMPTY_OR_FULL_A high the
    // driver is actively encouraged to overrun, so this is a routine
    // case, not a curiosity.
    //
    // The cure is to make port B FREE-RUNNING: it prefetches the byte at
    // the read pointer on every edge, and the pop consumes the value
    // captured on the PREVIOUS edge, which no same-cycle write can
    // reach.  The one case that breaks is push-into-empty immediately
    // followed by the pop of that same byte (fa_wp == fa_rp at
    // fa_count == 0), where the prefetch is one edge too old — the
    // fa_pf_fwd_q forwarding mux covers exactly that.  The pair is
    // bit-for-bit equivalent to the old `sample_a_raw <= fifo_a[fa_rp]`:
    //   sample_a_raw(N) = fa_prefetch(N)
    //                   = fwd(N-1) ? wdata(N-1) : mem_before_{N-1}[rp]
    // and fa_rp is constant across N-1..N (pops are ~4400 clocks apart),
    // so the mux picks wdata exactly when a write landed on rp at N-1
    // and the plain prefetch otherwise — in both cases mem_before_N[rp],
    // which is what the old read-first expression evaluated to.
    reg [7:0] fa_pf_dout;        // port-B output register (free-running)
    reg [7:0] fa_pf_fwd_data;    // write data held for the forward mux
    reg       fa_pf_fwd_q;       // previous edge wrote the byte at fa_rp
    wire [7:0] fa_prefetch = fa_pf_fwd_q ? fa_pf_fwd_data : fa_pf_dout;

    // ── FIFO A main array — true dual port ────────────────────────────
    // Port A: CPU write, else CPU readback.  Port B: free-running
    // prefetch of fifo_a[fa_rp].
    always @(posedge clk) begin
        if (fa_mem_we)
            fifo_a[fa_mem_waddr] <= pb_wdata;
        else if (fa_cpu_rd)
            fa_cpu_dout <= fifo_a[pb_addr[9:0]];

        fa_pf_dout     <= fifo_a[fa_rp];
        fa_pf_fwd_data <= pb_wdata;
        fa_pf_fwd_q    <= fa_mem_we && (fa_mem_waddr == fa_rp);
    end

    // Sample register: loads on a pop, holds on an underflow tick — the
    // "hold previous sample" semantics MAME's stale-rdptr read produces.
    always @(posedge clk) begin
        if (rst)
            sample_a_raw <= 8'h80;      // silent centre until first sample
        else if (fa_sample_pop)
            sample_a_raw <= fa_prefetch;
    end

    // FIFO A wavetable mirrors — voice 0 (low half) / voice 1 (high half).
    // The wavetable mirrors carry the same port-A/port-B hazard: in wave
    // mode the CPU stamps table bytes while the voices sample them.  The
    // read address changes every tick, so the free-running prefetch used
    // for the FIFO path does not apply; instead the colliding read is
    // simply SUPPRESSED, leaving the voice's sample register holding its
    // previous byte.  A data-side bypass mux was tried first and is NOT
    // usable: putting a mux on the `dout <= mem[addr]` expression breaks
    // Vivado's RAM template match and sends all four mirrors straight
    // back to LUTRAM (measured: 16 more "Infeasible attribute" warnings,
    // 384 LUTRAM cells, 1096 LUTs).  An enable-side guard keeps the
    // template intact.
    //
    // The cost of suppressing is one repeated sample byte on one voice,
    // and the collision needs a CPU write to land on the exact sample-
    // tick clock AND at the exact 9-bit index that voice is sampling:
    // ~1 in 2.3 million writes, inside the brief window where the ROM is
    // stamping the chime table.  Unlike the FIFO-full collision above --
    // which the Sonora playback hook makes routine -- this one is a
    // formality; it is guarded only so no BRAM read is ever undefined.
    wire fifo_a_wt0_fwd = fa_mem_we && (fa_mem_waddr == {1'b0, wt_phase_next0[23:15]});
    always @(posedge clk) begin
        if (fa_mem_we && !fa_mem_waddr[9])
            fifo_a_wt0[fa_mem_waddr[8:0]] <= pb_wdata;
        if (rst)
            wt_byte0_r <= 8'h00;
        else if (sample_tick && wave_active && !fifo_a_wt0_fwd)
            wt_byte0_r <= fifo_a_wt0[wt_phase_next0[23:15]];
    end

    wire fifo_a_wt1_fwd = fa_mem_we && (fa_mem_waddr == {1'b1, wt_phase_next1[23:15]});
    always @(posedge clk) begin
        if (fa_mem_we && fa_mem_waddr[9])
            fifo_a_wt1[fa_mem_waddr[8:0]] <= pb_wdata;
        if (rst)
            wt_byte1_r <= 8'h00;
        else if (sample_tick && wave_active && !fifo_a_wt1_fwd)
            wt_byte1_r <= fifo_a_wt1[wt_phase_next1[23:15]];
    end

    // ── FIFO B main array — true dual port ────────────────────────────
    // Same free-running-prefetch + forward structure as FIFO A above.
    reg [7:0] fb_pf_dout;
    reg [7:0] fb_pf_fwd_data;
    reg       fb_pf_fwd_q;
    wire [7:0] fb_prefetch = fb_pf_fwd_q ? fb_pf_fwd_data : fb_pf_dout;

    always @(posedge clk) begin
        if (fb_mem_we)
            fifo_b[fb_mem_waddr] <= pb_wdata;
        else if (fb_cpu_rd)
            fb_cpu_dout <= fifo_b[pb_addr[9:0]];

        fb_pf_dout     <= fifo_b[fb_rp];
        fb_pf_fwd_data <= pb_wdata;
        fb_pf_fwd_q    <= fb_mem_we && (fb_mem_waddr == fb_rp);
    end

    always @(posedge clk) begin
        if (rst)
            sample_b_raw <= 8'h80;
        else if (fb_sample_pop)
            sample_b_raw <= fb_prefetch;
    end

    // FIFO B wavetable mirrors — voice 2 (low half) / voice 3 (high half).
    wire fifo_b_wt0_fwd = fb_mem_we && (fb_mem_waddr == {1'b0, wt_phase_next2[23:15]});
    always @(posedge clk) begin
        if (fb_mem_we && !fb_mem_waddr[9])
            fifo_b_wt0[fb_mem_waddr[8:0]] <= pb_wdata;
        if (rst)
            wt_byte2_r <= 8'h00;
        else if (sample_tick && wave_active && !fifo_b_wt0_fwd)
            wt_byte2_r <= fifo_b_wt0[wt_phase_next2[23:15]];
    end

    wire fifo_b_wt1_fwd = fb_mem_we && (fb_mem_waddr == {1'b1, wt_phase_next3[23:15]});
    always @(posedge clk) begin
        if (fb_mem_we && fb_mem_waddr[9])
            fifo_b_wt1[fb_mem_waddr[8:0]] <= pb_wdata;
        if (rst)
            wt_byte3_r <= 8'h00;
        else if (sample_tick && wave_active && !fifo_b_wt1_fwd)
            wt_byte3_r <= fifo_b_wt1[wt_phase_next3[23:15]];
    end

    wire [11:0] fa_count_delta = {1'b0, fa_count}
                               + (fa_cpu_push ? 12'd1 : 12'd0)
                               - (fa_sample_pop ? 12'd1 : 12'd0);
    wire [11:0] fb_count_delta = {1'b0, fb_count}
                               + (fb_cpu_push ? 12'd1 : 12'd0)
                               - (fb_sample_pop ? 12'd1 : 12'd0);
    wire [10:0] fa_count_next = (fa_count_delta > 12'd1024)
                              ? 11'd1024 : fa_count_delta[10:0];
    wire [10:0] fb_count_next = (fb_count_delta > 12'd1024)
                              ? 11'd1024 : fb_count_delta[10:0];
    wire fa_next_above = (fa_count_next >= 11'd512);
    wire fb_next_above = (fb_count_next >= 11'd512);

    // ── CPU write / read path + pointer + count updates ───────────────
    always @(posedge clk) begin
        if (rst) begin
            // Sonora vs EASC device_reset:
            //   Sonora (asc_sonora_device::device_reset, asc.cpp:954-965):
            //     m_regs zeroed, R_MODE=1, R_FIFOSTAT=0x02,
            //     m_fifo_irqen[0/1] = 0  (IRQ enabled)
            //   EASC (asc_easc_device::device_reset, asc.cpp:1753-1766):
            //     base reset (zeroed + R_FIFOSTAT=0x02 by base), then
            //     m_fifo_irqen[0/1] = 1  (IRQ DISABLED at reset).
            //   Note R_MODE=0 on EASC base reset (asc_base_device::
            //     device_reset memset).
            mode_reg     <= IS_EASC ? MODE_SILENT : MODE_FIFO;
            chan_ctl     <= 8'h00;
            fifo_ctl     <= 8'h00;
            fa_irqen     <= IS_EASC ? 8'h01 : 8'h00;
            fb_irqen     <= IS_EASC ? 8'h01 : 8'h00;
            test_reg     <= 8'h00;
            // Local: rate_reg + volume_reg defaults preserved so the
            // chime plays at 22 kHz / full scale without ROM programming.
            rate_reg     <= RATE_DEFAULT;
            volume_reg   <= 8'hFF;
            playreca_reg <= 8'h00;          // playback mode enabled
            playreca_written <= 1'b0;       // sticky: set on first 0x80A write
            // EASC ext-block resets — all zero per MAME asc_base_device
            // ::device_reset (memset(m_regs, 0, sizeof(m_regs))).
            r_srca_h          <= 8'h00;
            r_srca_l          <= 8'h00;
            r_vola_l          <= 8'h00;
            r_vola_r          <= 8'h00;
            r_fifoa_ctrl      <= 8'h00;
            r_cdxa_a          <= 8'h00;
            r_srcb_h          <= 8'h00;
            r_srcb_l          <= 8'h00;
            r_volb_l          <= 8'h00;
            r_volb_r          <= 8'h00;
            r_fifob_ctrl      <= 8'h00;
            r_cdxa_b          <= 8'h00;
            r_wrptra_h_shadow <= 8'h00;
            r_wrptra_l_shadow <= 8'h00;
            r_rdptra_h_shadow <= 8'h00;
            r_rdptra_l_shadow <= 8'h00;
            r_wrptrb_h_shadow <= 8'h00;
            r_wrptrb_l_shadow <= 8'h00;
            r_rdptrb_h_shadow <= 8'h00;
            r_rdptrb_l_shadow <= 8'h00;
            fa_wp        <= 10'd0;
            fa_rp        <= 10'd0;
            fb_wp        <= 10'd0;
            fb_rp        <= 10'd0;
            fa_count     <= 11'd0;
            fb_count     <= 11'd0;
            fa_half_empty_irq   <= 1'b0;
            fb_half_empty_irq   <= 1'b0;
            // R_FIFOSTAT reset:
            //   Sonora (asc.cpp:964): 0x02  (STAT_EMPTY_OR_FULL_A)
            //   EASC base reset zeroes m_regs; no override → 0x00.
            fifostat            <= IS_EASC ? 8'h00 : STAT_EMPTY_OR_FULL_A;
            fa_overflow_sticky  <= 1'b0;
            fb_overflow_sticky  <= 1'b0;
            fa_underflow_sticky <= 1'b0;
            fb_underflow_sticky <= 1'b0;
            fa_was_above <= 1'b0;
            fb_was_above <= 1'b0;
            wt_phase[0]  <= 24'h0;
            wt_phase[1]  <= 24'h0;
            wt_phase[2]  <= 24'h0;
            wt_phase[3]  <= 24'h0;
            wt_incr[0]   <= 24'h0;
            wt_incr[1]   <= 24'h0;
            wt_incr[2]   <= 24'h0;
            wt_incr[3]   <= 24'h0;
            irq_q        <= 1'b0;
        end else begin
            // ─── FIFO write (CPU push) ────────────────────────────────
            // FIFO A: pb_addr 0x000..0x3FF
            // FIFO B: pb_addr 0x400..0x7FF
            // FIFO mode: push at wrptr (MAME asc_base_device::write
            // asc.cpp:380-431).  WAVE mode: direct-addressed ram write
            // at pb_addr[9:0] (MAME first-gen asc_device::write
            // asc.cpp:700 / 736 — required for the Q700 boot ROM's
            // chime synthesis loop to stamp wavetable bytes at FIFO
            // offsets 0 / 0x200 / 0x400 / 0x600).
            if (pb_wr && pb_addr[11:10] == 2'b00) begin
                if (fa_cpu_push) begin
                    // RAM write itself lives in the FIFO A port block.
                    fa_wp         <= fa_wp + 10'd1;
                end else if (fa_wave_write) begin
                    // RAM write itself lives in the FIFO A port block.
                end else if (fifo_active) begin
                    // FIFO mode but full → overflow sticky.
                    fa_overflow_sticky <= 1'b1;
                end
            end
            else if (pb_wr && pb_addr[11:10] == 2'b01) begin
                if (fb_cpu_push) begin
                    // RAM write itself lives in the FIFO B port block.
                    fb_wp         <= fb_wp + 10'd1;
                end else if (fb_wave_write) begin
                    // RAM write itself lives in the FIFO B port block.
                end else if (fifo_active) begin
                    fb_overflow_sticky <= 1'b1;
                end
            end

            // ─── R_FIFOSTAT base-write hook (FIFO A push, playback mode) ──
            // Per MAME asc_base_device::write lines 386-404: in playback
            // mode (R_PLAYRECA bit 0 = 0), every successful FIFO A push
            // updates the STAT_HALF_FULL_A / STAT_EMPTY_OR_FULL_A bits
            // based on the post-push capacity (cap_next here).  Then the
            // Sonora write hook (asc_sonora_device::write lines 1109-
            // 1116) re-forces STAT_EMPTY_OR_FULL_A high after every FIFO
            // A push in playback mode (regardless of cap).
            if (fa_cpu_push && !playreca_reg[0]) begin
                if (fa_count_next >= 11'd512) begin
                    fifostat[STAT_BIT_HALF_FULL_A] <= 1'b0;
                    if (fa_count_next >= 11'd1023)
                        fifostat[STAT_BIT_EMPTY_OR_FULL_A] <= 1'b1;
                end else if (fa_count_next > 11'd0) begin
                    fifostat[STAT_BIT_EMPTY_OR_FULL_A] <= 1'b0;
                end
            end
            // Sonora write post-hook for FIFO A in playback mode: re-
            // force STAT_EMPTY_OR_FULL_A high (overrides the same-cycle
            // base-hook clear above when applicable).  This lands LAST
            // in the always block (Verilog last-write-wins for nb-assigns
            // to the same bit) — we ensure that by ordering this hook
            // after the base hook in source.
            //
            // EASC has NO equivalent post-hook (asc_easc_device::write
            // at asc.cpp:1679-1735 falls through to asc_base_device::
            // write without a "in playback mode FIFO A always empty"
            // override).  So under IS_EASC this re-force is suppressed.
            if (!IS_EASC
                && pb_wr && pb_addr[11:10] == 2'b00 && !playreca_reg[0]) begin
                fifostat[STAT_BIT_EMPTY_OR_FULL_A] <= 1'b1;
            end

            // ─── R_FIFOSTAT base-write hook (FIFO B push) ─────────────
            // Per MAME asc_base_device::write lines 410-428.  Note the
            // base-write hook for FIFO B has NO PLAYRECA gate (unlike
            // FIFO A), so it fires unconditionally on every successful
            // push.  But there's no Sonora-specific re-force hook for B.
            if (fb_cpu_push) begin
                if (fb_count_next >= 11'd512) begin
                    fifostat[STAT_BIT_HALF_FULL_B] <= 1'b0;
                    if (fb_count_next >= 11'd1023)
                        fifostat[STAT_BIT_EMPTY_OR_FULL_B] <= 1'b1;
                end else if (fb_count_next > 11'd0) begin
                    fifostat[STAT_BIT_EMPTY_OR_FULL_B] <= 1'b0;
                end
            end

            // ─── Control-register writes (pb_addr 0x800..0x8FF) ──────
            // MAME asc_sonora_device::write ignores most ASC-era
            // registers (R_MODE, R_CONTROL, R_WTCONTROL, R_CLOCK,
            // R_BATMANCONTROL).  But asc_base_device::write's catch-all
            // bottom (line 593-597: `m_regs[offset-0x800] = data`) lands
            // every byte in 0x800-0xFFF into m_regs[].  The ones that
            // matter for the programmer-visible interface are tracked
            // explicitly here; "ignored" registers are not stored,
            // matching the MAME read paths that override them with
            // hardcoded values (R_CONTROL→0, R_FIFOMODE→0, etc.).
            if (pb_wr && pb_addr[11:8] == 4'h8) begin
                case (pb_addr[7:0])
                    // R_VERSION (0x00) — read-only, writes ignored.
                    8'h00: ;
                    // R_MODE (0x01).  Two semantics depending on
                    // IS_EASC:
                    //   Non-EASC (VERSION 0x00 hybrid path): per MAME
                    //     asc_base_device::write asc.cpp:439-440
                    //     `data &= 3`, all 2 bits land.  Lets the Q700
                    //     ROM select wavetable mode (data = 2) on the
                    //     iter-1 chime path.
                    //   EASC (VERSION 0xB0): per MAME
                    //     asc_easc_device::write (asc.cpp:1683-1690):
                    //       `m_regs[R_MODE] = data & 1`
                    //     plus on a (data&1) != m_regs[R_MODE] transition,
                    //     reset rdptr/wrptr/cap for both channels and
                    //     OR STAT_EMPTY_OR_FULL_B (0x8) into FIFOSTAT.
                    //     The bit-0 mask strictly disables wavetable
                    //     mode selection on EASC silicon.
                    8'h01: begin
                        if (IS_EASC) begin
                            // EASC: bit-0 mask + on toggle reset
                            // FIFO state.  asc.cpp:1684-1689.
                            if (pb_wdata[0] != mode_reg[0]) begin
                                fa_rp     <= 10'd0;
                                fa_wp     <= 10'd0;
                                fb_rp     <= 10'd0;
                                fb_wp     <= 10'd0;
                                fa_count  <= 11'd0;
                                fb_count  <= 11'd0;
                            end
                            mode_reg <= {7'b0, pb_wdata[0]};
                            // asc.cpp:1691: m_regs[R_FIFOSTAT] |= 0x8
                            //   ("signal playback FIFO empty").
                            fifostat[STAT_BIT_EMPTY_OR_FULL_B] <= 1'b1;
                        end else begin
                            mode_reg <= {6'b0, pb_wdata[1:0]};
                        end
                    end
                    // R_CONTROL (0x02) — Sonora reads 0; we accept the
                    // write into chan_ctl as a tb backdoor for stereo
                    // presentation.  Reads still return 0 (MAME-faithful).
                    8'h02: chan_ctl   <= pb_wdata;
                    // R_FIFOMODE (0x03) — bit 7 clears FIFOs.
                    8'h03: begin
                        fifo_ctl <= pb_wdata;
                        if (pb_wdata[7]) begin
                            fa_wp        <= 10'd0;
                            fa_rp        <= 10'd0;
                            fb_wp        <= 10'd0;
                            fb_rp        <= 10'd0;
                            fa_count     <= 11'd0;
                            fb_count     <= 11'd0;
                            fa_half_empty_irq   <= 1'b0;
                            fb_half_empty_irq   <= 1'b0;
                            // Per MAME asc_base_device::write line 465:
                            // R_FIFOSTAT |= 0xa (FIFO A+B "empty" bits).
                            fifostat <= fifostat | 8'h0a;
                            fa_overflow_sticky  <= 1'b0;
                            fb_overflow_sticky  <= 1'b0;
                            fa_underflow_sticky <= 1'b0;
                            fb_underflow_sticky <= 1'b0;
                            fa_was_above <= 1'b0;
                            fb_was_above <= 1'b0;
                        end
                    end
                    // R_FIFOSTAT (0x04) — full byte writable per MAME's
                    // catch-all `m_regs[offset-0x800]=data` at line 596.
                    // This lands AFTER the base-write hooks above (which
                    // touch individual bits); a same-cycle 0x804 write
                    // wins (last-write-wins for nb-assign to the whole
                    // register).  Q700 ROM doesn't actually use this
                    // path, but ASCTester does as part of its idle-IRQ
                    // probe.
                    8'h04: fifostat <= pb_wdata;
                    // R_WTCONTROL (0x05) — Sonora ignores.
                    8'h05: ;
                    // R_VOLUME (0x06).
                    8'h06: volume_reg <= pb_wdata;
                    // R_CLOCK (0x07) — Sonora ignores.
                    8'h07: ;
                    // R_BATMANCONTROL (0x08) — Sonora ignores; tb backdoor
                    // accepts the write into rate_reg (Q700 ROM doesn't
                    // touch 0x808).
                    8'h08: rate_reg <= pb_wdata;
                    // R_PLAYRECA (0x0A).
                    8'h0A: begin
                        playreca_reg     <= pb_wdata;
                        playreca_written <= 1'b1;   // sticky-set
                    end
                    // R_TEST (0x0F) — writable scratch (MAME catch-all
                    // m_regs[]; we expose readback as test_reg).
                    8'h0F: test_reg <= pb_wdata;
                    // 0x810 / 0x811 are LOCAL per-FIFO status bytes,
                    // not in MAME — they expose sticky overflow/
                    // underflow flags for hardware bring-up.  Writing
                    // these clears the local sticky bits.  These bytes
                    // also overlap MAME's wavetable phase[0] BE slot;
                    // the wavetable handler below picks them up too,
                    // and the sticky-clear here is benign.
                    8'h10: begin
                        fa_half_empty_irq   <= 1'b0;
                        fa_overflow_sticky  <= 1'b0;
                        fa_underflow_sticky <= 1'b0;
                    end
                    8'h11: begin
                        fb_half_empty_irq   <= 1'b0;
                        fb_overflow_sticky  <= 1'b0;
                        fb_underflow_sticky <= 1'b0;
                    end
                    default: ;
                endcase
            end

            // ─── Wavetable phase / increment writes (0x811..0x82F) ──
            // Per MAME asc_base_device::write asc.cpp:473-591.  Each
            // voice has a 4-byte big-endian slot (high byte ignored,
            // low 3 bytes form a 24-bit value).  These writes are
            // honoured in any mode (MAME stores them into m_regs[]
            // regardless of R_MODE), but they only have audible effect
            // while in MODE_WAVE.  Decode the full 12-bit address so
            // the case constants align byte-for-byte with the MAME
            // switch (asc.cpp:473-591).
            if (pb_wr) begin
                case (pb_addr)
                    // Voice 0 phase: 0x811-0x813 → bits 23:16, 15:8, 7:0
                    12'h811: wt_phase[0][23:16] <= pb_wdata;
                    12'h812: wt_phase[0][15:8]  <= pb_wdata;
                    12'h813: wt_phase[0][7:0]   <= pb_wdata;
                    // Voice 0 incr:  0x815-0x817
                    12'h815: wt_incr[0][23:16]  <= pb_wdata;
                    12'h816: wt_incr[0][15:8]   <= pb_wdata;
                    12'h817: wt_incr[0][7:0]    <= pb_wdata;
                    // Voice 1 phase: 0x819-0x81B
                    12'h819: wt_phase[1][23:16] <= pb_wdata;
                    12'h81A: wt_phase[1][15:8]  <= pb_wdata;
                    12'h81B: wt_phase[1][7:0]   <= pb_wdata;
                    // Voice 1 incr:  0x81D-0x81F
                    12'h81D: wt_incr[1][23:16]  <= pb_wdata;
                    12'h81E: wt_incr[1][15:8]   <= pb_wdata;
                    12'h81F: wt_incr[1][7:0]    <= pb_wdata;
                    // Voice 2 phase: 0x821-0x823
                    12'h821: wt_phase[2][23:16] <= pb_wdata;
                    12'h822: wt_phase[2][15:8]  <= pb_wdata;
                    12'h823: wt_phase[2][7:0]   <= pb_wdata;
                    // Voice 2 incr:  0x825-0x827
                    12'h825: wt_incr[2][23:16]  <= pb_wdata;
                    12'h826: wt_incr[2][15:8]   <= pb_wdata;
                    12'h827: wt_incr[2][7:0]    <= pb_wdata;
                    // Voice 3 phase: 0x829-0x82B
                    12'h829: wt_phase[3][23:16] <= pb_wdata;
                    12'h82A: wt_phase[3][15:8]  <= pb_wdata;
                    12'h82B: wt_phase[3][7:0]   <= pb_wdata;
                    // Voice 3 incr:  0x82D-0x82F
                    12'h82D: wt_incr[3][23:16]  <= pb_wdata;
                    12'h82E: wt_incr[3][15:8]   <= pb_wdata;
                    12'h82F: wt_incr[3][7:0]    <= pb_wdata;
                    default: ;
                endcase
            end

            // ─── 0xE00 backdoor (asc_base_device::write line 374-378) ──
            // Sets fifostat |= 0x0F + asserts IRQ.  Programmer-visible;
            // ASCTester uses it to verify the IRQ pin wiring.
            if (pb_wr && pb_addr == 12'hE00) begin
                fifostat <= fifostat | 8'h0F;
                irq_q    <= 1'b1;
            end

            // ─── 0xF09 / 0xF29 (Sonora IRQ enable, edge-detect) ────────
            // Per MAME asc_sonora_device::write (lines 1082-1104):
            //   If clearing the disable (data&1==0) AND was-disabled
            //   (m_fifo_irqen[X]&1==1) AND STAT_HALF_FULL_x bit set →
            //   ASSERT IRQ.
            //   If setting the disable (data&1==1) → CLEAR IRQ.
            //   Then update m_fifo_irqen[X] = data & 1.
            if (pb_wr && pb_addr == 12'hF09) begin
                if (!pb_wdata[0] && fa_irqen[0]
                    && fifostat[STAT_BIT_HALF_FULL_A]) begin
                    irq_q <= 1'b1;
                end else if (pb_wdata[0]) begin
                    irq_q <= 1'b0;
                end
                fa_irqen <= {7'b0000000, pb_wdata[0]};
            end
            if (pb_wr && pb_addr == 12'hF29) begin
                if (!pb_wdata[0] && fb_irqen[0]
                    && fifostat[STAT_BIT_HALF_FULL_B]) begin
                    irq_q <= 1'b1;
                end else if (pb_wdata[0]) begin
                    irq_q <= 1'b0;
                end
                fb_irqen <= {7'b0000000, pb_wdata[0]};
            end

            // ─── EASC extended-register writes (0xF00..0xF3F) ────────
            // MAME's asc_base_device::write latches every byte landing
            // in this window into m_regs[offset-0x800].  We mirror that
            // shadowing behavior for the registers we model; CD-XA and
            // shadowed pointer writes are accepted but functionally
            // inert (no live pointer or ADPCM decoder is moved).  F09
            // and F29 are handled by the dedicated lines above and
            // intentionally skipped here.
            if (pb_wr && pb_addr[11:8] == 4'hF) begin
                case (pb_addr[7:0])
                    // FIFO A side
                    8'h00: r_wrptra_h_shadow <= pb_wdata;
                    8'h01: r_wrptra_l_shadow <= pb_wdata;
                    8'h02: r_rdptra_h_shadow <= pb_wdata;
                    8'h03: r_rdptra_l_shadow <= pb_wdata;
                    8'h04: r_srca_h     <= pb_wdata;
                    8'h05: r_srca_l     <= pb_wdata;
                    8'h06: r_vola_l     <= pb_wdata;
                    8'h07: r_vola_r     <= pb_wdata;
                    8'h08: r_fifoa_ctrl <= pb_wdata;
                    // 0x09 = R_FIFOA_IRQCTRL — handled above, do not touch.
                    8'h10: r_cdxa_a     <= pb_wdata;
                    // FIFO B side
                    8'h20: r_wrptrb_h_shadow <= pb_wdata;
                    8'h21: r_wrptrb_l_shadow <= pb_wdata;
                    8'h22: r_rdptrb_h_shadow <= pb_wdata;
                    8'h23: r_rdptrb_l_shadow <= pb_wdata;
                    8'h24: r_srcb_h     <= pb_wdata;
                    8'h25: r_srcb_l     <= pb_wdata;
                    8'h26: r_volb_l     <= pb_wdata;
                    8'h27: r_volb_r     <= pb_wdata;
                    8'h28: r_fifob_ctrl <= pb_wdata;
                    // 0x29 = R_FIFOB_IRQCTRL — handled above, do not touch.
                    8'h30: r_cdxa_b     <= pb_wdata;
                    default: ;
                endcase
            end

            // ─── CPU read side-effects ─────────────────────────────
            // Per MAME asc_sonora_device::read line 1049-1055:
            //   Reading R_FIFOSTAT clears the IRQ line iff
            //   STAT_HALF_FULL_B is NOT set.  Status BITS are NOT cleared.
            // Local convention (not MAME): per-FIFO 0x810/0x811 read
            //   clears the local sticky/edge latches.
            if (pb_rd && !pb_wr) begin
                case (pb_addr)
                    12'h804: begin
                        if (!fifostat[STAT_BIT_HALF_FULL_B])
                            irq_q <= 1'b0;
                        // No status-bit clear (MAME-faithful).  We also
                        // clear the local edge latches here to keep the
                        // per-FIFO status registers usable on bring-up.
                        fa_half_empty_irq <= 1'b0;
                        fb_half_empty_irq <= 1'b0;
                    end
                    12'h810: begin
                        fa_half_empty_irq   <= 1'b0;
                        fa_overflow_sticky  <= 1'b0;
                        fa_underflow_sticky <= 1'b0;
                    end
                    12'h811: begin
                        fb_half_empty_irq   <= 1'b0;
                        fb_overflow_sticky  <= 1'b0;
                        fb_underflow_sticky <= 1'b0;
                    end
                    default: ;
                endcase
            end

            // ─── Sample tick: advance read-pointers + latch output ───
            // Per MAME asc_sonora_device::sound_stream_update lines 967-
            // 1029.  Key bits:
            //   - Top of routine: if !PLAYRECA[0], R_FIFOSTAT |=
            //     STAT_EMPTY_OR_FULL_A and (if A IRQ enabled) assert.
            //   - Per sample: smpll = (s8)fifo[0][rdptr0]^0x80; capture
            //     BEFORE cap decrement.  m_last_left = smpll; on
            //     underflow (cap==0), rdptr doesn't advance, so smpll
            //     reflects the stale-but-last-good byte.
            //   - After decrement: combined-half-B = (cap_a<0x200) ||
            //     (cap_b<0x200) → set/clear bit 2; if bit 2 newly set
            //     and B IRQ enabled, assert.  Combined-empty-B = (cap_a
            //     ==0) || (cap_b==0) → set/clear bit 3.
            //
            // Wavetable branch: no FIFO pop, no PLAYRECA force.  Per
            // MAME asc_base_device::sound_stream_update case 2
            // (asc.cpp:248-282): each voice advances phase by incr,
            // then samples fifo[0..1KB] / fifo[1..1KB] at index
            // (phase>>15)&0x1ff, sums, and emits.  Voices 0/1 read from
            // fifo_a (offsets 0 and 0x200 within the 1 KB array);
            // voices 2/3 read from fifo_b (offsets 0 and 0x200).  This
            // matches the brief's simplified flat layout: voice v
            // wavetable at FIFO offset v*0x200 of the unified 2 KB
            // array (FIFO A then FIFO B).
            if (sample_tick) begin
                // ─── Sonora-only top-of-stream PLAYRECA force ────────
                // Per MAME asc_sonora_device::sound_stream_update
                // (asc.cpp:970-978): in playback mode FIFO A always
                // reads as empty.  This is a Sonora override on the
                // base stream loop; EASC does NOT have it (asc.cpp:
                // 1457-1535 has no equivalent block at the top of the
                // EASC stream update).  Suppressed under IS_EASC.
                if (!IS_EASC && fifo_active && !playreca_reg[0])
                    fifostat[STAT_BIT_EMPTY_OR_FULL_A] <= 1'b1;

                // FIFO A pop or hold-last-sample-on-underflow (FIFO mode).
                if (fa_sample_pop) begin
                    // sample_a_raw is loaded by the FIFO A port block.
                    fa_rp        <= fa_rp + 10'd1;
                end else if (fifo_active) begin
                    // Underflow: hold previous sample_a_raw (matches
                    // MAME's m_last_left = smpll where smpll = stale
                    // fifo[rdptr] = previous good byte).  Local sticky.
                    fa_underflow_sticky <= 1'b1;
                end

                // FIFO B pop or hold-last-sample-on-underflow.  Sonora's
                // stream loop drains B unconditionally — no chan_ctl gate.
                if (fb_sample_pop) begin
                    // sample_b_raw is loaded by the FIFO B port block.
                    fb_rp        <= fb_rp + 10'd1;
                end else if (fifo_active) begin
                    fb_underflow_sticky <= 1'b1;
                end

                // Wave mode: advance phase counters and, in the same
                // cycle, register the post-increment wavetable byte read.
                // `wt_phase[v] + wt_incr[v]` is exactly the value that
                // lands in wt_phase[v] one cycle from now (matches the
                // prior combinational-read semantics: "post-update
                // phase"), computed one tick early here so the fifo_a/
                // fifo_b lookup itself becomes a plain synchronous
                // `reg <= mem[addr]` read (BRAM-eligible) instead of a
                // fully combinational one.  Voice 0/1 read fifo_a, 2/3
                // read fifo_b; odd voices add the 0x200 sub-array
                // offset.  Sample cadence is 22 kHz, so folding the read
                // into this same edge costs no audible latency.
                if (wave_active) begin
                    wt_phase[0] <= wt_phase_next0;
                    wt_phase[1] <= wt_phase_next1;
                    wt_phase[2] <= wt_phase_next2;
                    wt_phase[3] <= wt_phase_next3;
                end

                if (IS_EASC) begin
                    // ─── EASC pop_fifo() per-channel STAT updates ────
                    // Per MAME asc_easc_device::pop_fifo (asc.cpp:1542-
                    // 1586): each consumed sample updates HALF_FULL_x
                    // and EMPTY_OR_FULL_x INDEPENDENTLY for the channel
                    // it belongs to, using the "cap before decrement".
                    //   if (cap_pre_decr <= 0x1ff)
                    //     R_FIFOSTAT |= stat_half[ch];
                    //     if (!fifo_irqen[ch] & 1) ASSERT_IRQ;
                    //   else
                    //     R_FIFOSTAT &= ~stat_half[ch];
                    //   if (cap_pre_decr == 0)
                    //     R_FIFOSTAT |= stat_empty[ch];
                    //   else
                    //     R_FIFOSTAT &= ~stat_empty[ch];
                    //
                    // We compare against fa_count / fb_count (cap
                    // BEFORE the post-decrement that fa_count_next
                    // reflects) — fa_sample_pop/fb_sample_pop drives
                    // the decrement in the same cycle.
                    //
                    // Both channels update each sample tick under EASC
                    // because the stream pops both in lockstep
                    // (asc.cpp:1490-1523 calls pop_fifo for each).
                    if (fifo_active) begin
                        // Channel A
                        if (fa_count <= 11'h1ff) begin
                            fifostat[STAT_BIT_HALF_FULL_A] <= 1'b1;
                            if (!fa_irqen[0])
                                irq_q <= 1'b1;
                        end else begin
                            fifostat[STAT_BIT_HALF_FULL_A] <= 1'b0;
                        end
                        if (fa_count == 11'h0)
                            fifostat[STAT_BIT_EMPTY_OR_FULL_A] <= 1'b1;
                        else
                            fifostat[STAT_BIT_EMPTY_OR_FULL_A] <= 1'b0;

                        // Channel B
                        if (fb_count <= 11'h1ff) begin
                            fifostat[STAT_BIT_HALF_FULL_B] <= 1'b1;
                            if (!fb_irqen[0])
                                irq_q <= 1'b1;
                        end else begin
                            fifostat[STAT_BIT_HALF_FULL_B] <= 1'b0;
                        end
                        if (fb_count == 11'h0)
                            fifostat[STAT_BIT_EMPTY_OR_FULL_B] <= 1'b1;
                        else
                            fifostat[STAT_BIT_EMPTY_OR_FULL_B] <= 1'b0;
                    end
                    // EASC has NO Sonora-style "playback-mode level
                    // fire on FIFO A IRQ enable" — IRQ assertion is
                    // strictly per-channel via the half-full check
                    // above.
                end else begin
                    // Sonora-overloaded combined HALF_FULL_B / EMPTY_B.
                    // Bit 2 = "either A or B half"; bit 3 = "either
                    // empty".  Per MAME asc_sonora_device::sound_stream_
                    // update (asc.cpp:1004-1025).
                    if ((fa_count_next < 11'd512)
                        || (fb_count_next < 11'd512))
                        fifostat[STAT_BIT_HALF_FULL_B] <= 1'b1;
                    else
                        fifostat[STAT_BIT_HALF_FULL_B] <= 1'b0;

                    if ((fa_count_next == 11'd0)
                        || (fb_count_next == 11'd0))
                        fifostat[STAT_BIT_EMPTY_OR_FULL_B] <= 1'b1;
                    else
                        fifostat[STAT_BIT_EMPTY_OR_FULL_B] <= 1'b0;

                    // ─── Sonora stream-side IRQ asserts ──────────
                    //   - Playback-mode level fire: !PLAYRECA[0] && A
                    //     IRQ enabled (asc.cpp:970-978).  Gated on
                    //     playreca_written so the Q700 boot ROM
                    //     (which never touches 0x80A) doesn't trigger
                    //     a continuous IRQ storm.
                    //   - Combined-half-B fire (asc.cpp:1004-1012).
                    if (playreca_written && !playreca_reg[0] && !fa_irqen[0])
                        irq_q <= 1'b1;
                    if (playreca_written
                        && ((fa_count_next < 11'd512)
                            || (fb_count_next < 11'd512))
                        && !fb_irqen[0])
                        irq_q <= 1'b1;
                end
            end

            if (!fifo_clear_write) begin
                fa_count <= fa_count_next;
                fb_count <= fb_count_next;
            end

            // ─── Half-empty edge detection (LOCAL bring-up only) ─────
            // The per-FIFO 0x810/0x811 status registers expose a one-shot
            // half-empty flag latched on a downward 512-cross.  Used by
            // hardware bring-up.  NOT routed to the IRQ line — the MAME
            // stream loop handles all IRQ asserts (combined-half-B in
            // sound_stream_update + playback-mode level fire).
            if (fifo_clear_write) begin
                fa_was_above <= 1'b0;
                fb_was_above <= 1'b0;
            end else begin
                fa_was_above <= fa_next_above;
                fb_was_above <= fb_next_above;
                if (fa_was_above && !fa_next_above)
                    fa_half_empty_irq <= 1'b1;
                if (fb_was_above && !fb_next_above)
                    fb_half_empty_irq <= 1'b1;
            end
        end
    end

    // ── Audio output path ─────────────────────────────────────────────
    // MAME emits raw `(s8)byte ^ 0x80` to the host audio sink — physical
    // attenuation is the off-chip DAC's job.  We mirror that: signed
    // 16-bit PCM = raw sample x 256 (left-shift 8 of centred sample).
    // No fabric volume scaling; the downstream `audio_pwm` modulator
    // handles the full 16-bit range.
    wire [15:0] a_centred = {{8{1'b0}}, sample_a_raw} - 16'h0080;
    wire [15:0] b_centred = {{8{1'b0}}, sample_b_raw} - 16'h0080;
    wire signed [15:0] a_pcm16 = $signed(a_centred) <<< 8;
    wire signed [15:0] b_pcm16 = $signed(b_centred) <<< 8;

    // Legacy packed contract: low byte of centred sample.  Kept for the
    // legacy 8-bit-stereo bridge in the HDMI/I2S path.
    wire [7:0] a_pack = a_centred[7:0];
    wire [7:0] b_pack = b_centred[7:0];

    // ── Wavetable mixer ────────────────────────────────────────────────
    // Per MAME asc_base_device::sound_stream_update case 2
    // (asc.cpp:248-282).  Each voice samples the FIFO ram at index
    // `(phase[v] >> 15) & 0x1ff`, XORs with 0x80 to centre, sign-extends
    // to s16, multiplies by 256 (×= shift-left-8 in MAME).  Sum of the
    // four voices forms the mixed PCM sample, clamped to s16 range.
    // The fifo_a/fifo_b lookup itself is a REGISTERED synchronous read
    // (wt_byte{0..3}_r, updated at the sample_tick site above using the
    // post-update phase, matching MAME's ordering:
    // `m_phase[ch] += m_incr[ch]; smpl = (s8)fifo[...];`).  Only the
    // centre/sign-extend/sum below remains combinational.
    // Centre (XOR 0x80) and place in high byte of s16 — equivalent to
    // MAME's `(s8)smpl = byte ^ 0x80; mixL += smpl*256`.  Multiply by
    // 256 = shift-left-8, so the centred 8-bit signed value fills the
    // high byte of the s16 with the low byte zeroed.
    wire [7:0] wt_centred0 = wt_byte0_r ^ 8'h80;
    wire [7:0] wt_centred1 = wt_byte1_r ^ 8'h80;
    wire [7:0] wt_centred2 = wt_byte2_r ^ 8'h80;
    wire [7:0] wt_centred3 = wt_byte3_r ^ 8'h80;
    wire signed [15:0] wt_s16_0 = $signed({wt_centred0, 8'h00});
    wire signed [15:0] wt_s16_1 = $signed({wt_centred1, 8'h00});
    wire signed [15:0] wt_s16_2 = $signed({wt_centred2, 8'h00});
    wire signed [15:0] wt_s16_3 = $signed({wt_centred3, 8'h00});
    // Sum the four voices (s18 covers 4×s16 max range without saturation),
    // then divide by 4 (>>2) to match MAME's `stream.put_int(..., 32768*4)`
    // normalization (asc.cpp:278-279).  This places one full-amplitude
    // voice at quarter-scale s16 so the chord at full mix doesn't clip.
    wire signed [17:0] wt_sum = $signed({{2{wt_s16_0[15]}}, wt_s16_0})
                              + $signed({{2{wt_s16_1[15]}}, wt_s16_1})
                              + $signed({{2{wt_s16_2[15]}}, wt_s16_2})
                              + $signed({{2{wt_s16_3[15]}}, wt_s16_3});
    wire signed [15:0] wt_mix_pcm = wt_sum[17:2];

    // One-cycle delay: sample_tick drives sample_X_raw at edge N; the
    // audio output latch happens at edge N+1 so the audio sees the
    // freshly-loaded byte.  Existing tb expectation; the downstream
    // modulator doesn't care.
    reg sample_tick_q;
    reg wave_active_q;
    reg stereo_q;
    always @(posedge clk) begin
        if (rst) begin
            sample_tick_q <= 1'b0;
            wave_active_q <= 1'b0;
            stereo_q      <= 1'b0;
        end else begin
            sample_tick_q <= sample_tick;
            // Latch mode-class + stereo flag together with the sample
            // tick so the audio output picks the correct path even if
            // the CPU writes 0x801 / 0x802 between the tick and the
            // settle cycle.
            wave_active_q <= wave_active;
            stereo_q      <= stereo_mode;
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            audio_sample_out   <= 16'h0000;
            audio_pcm_l        <= 16'h0000;
            audio_pcm_r        <= 16'h0000;
            audio_sample_valid <= 1'b0;
        end else begin
            audio_sample_valid <= sample_tick_q;
            if (sample_tick_q) begin
                if (wave_active_q) begin
                    // Wave mode: 4-voice mix → both channels (mono).
                    // The wavetable engine doesn't carry stereo info per
                    // MAME case 2; mixL == mixR.  Pack low byte of the
                    // mixed PCM into the legacy 8-bit-stereo bridge.
                    audio_sample_out <= {wt_mix_pcm[15:8], wt_mix_pcm[15:8]};
                    audio_pcm_l      <= wt_mix_pcm;
                    audio_pcm_r      <= wt_mix_pcm;
                end else if (stereo_q) begin
                    audio_sample_out <= {a_pack, b_pack};
                    audio_pcm_l      <= a_pcm16;
                    audio_pcm_r      <= b_pcm16;
                end else begin
                    audio_sample_out <= {a_pack, a_pack};
                    audio_pcm_l      <= a_pcm16;
                    audio_pcm_r      <= a_pcm16;
                end
            end
        end
    end

    // Register-file readback register plus the FIFO-window mux selects.
    // pb_rdata is a wire so the FIFO bytes can come straight out of the
    // BRAM port-A output registers without a second pipeline stage.
    reg [7:0] pb_rdata_reg;
    reg       pb_fifo_sel_q;    // last read targeted the FIFO window
    reg       pb_fifo_chan_q;   // 0 = FIFO A, 1 = FIFO B

    assign pb_rdata = pb_fifo_sel_q
                    ? (pb_fifo_chan_q ? fb_cpu_dout : fa_cpu_dout)
                    : pb_rdata_reg;

    // ── Read path ─────────────────────────────────────────────────────
    // Registered pb_rdata / pb_ack.  Decode the byte-offset addr into
    // the FIFO window (upper 1 KB returns 0 — FIFOs are write-only from
    // the CPU side) or the control-register file.
    always @(posedge clk) begin
        if (rst) begin
            pb_rdata_reg   <= 8'h00;
            pb_fifo_sel_q  <= 1'b0;
            pb_fifo_chan_q <= 1'b0;
            pb_ack   <= 1'b0;
        end else begin
            pb_ack <= pb_wr | pb_rd;
            pb_fifo_sel_q <= 1'b0;
            if (pb_rd) begin
                if (pb_addr[11:10] == 2'b00 || pb_addr[11:10] == 2'b01) begin
                    // FIFO-window readback.  The byte itself arrives in
                    // fa_cpu_dout / fb_cpu_dout (the main arrays' port-A
                    // output registers, loaded on this same edge); these
                    // flags steer the pb_rdata mux below.  A BRAM output
                    // register cannot be shared with the register-file
                    // readback, which is why the mux exists at all.
                    pb_fifo_sel_q  <= 1'b1;
                    pb_fifo_chan_q <= pb_addr[10];
                    pb_rdata_reg   <= 8'h00;
                end else if (pb_addr[11:8] == 4'h8) begin
                    case (pb_addr[7:0])
                        // R_VERSION — readback is the active version
                        // localparam (VERSION_USE), which selects between
                        // the iter-1 hybrid 0x00 path and the MAME-
                        // faithful EASC 0xB0 path.  See VERSION_USE /
                        // IS_EASC localparams above.
                        8'h00: pb_rdata_reg <= VERSION_USE;
                        // R_MODE — return the programmed mode.  MAME
                        // base device returns m_regs[R_MODE] directly
                        // (asc.cpp:315-316: `case R_MODE: break;` falls
                        // to the default `return m_regs[offset-0x800]`).
                        8'h01: pb_rdata_reg <= mode_reg;
                        // R_CONTROL — Sonora reads 0 (MAME asc.cpp:1042).
                        8'h02: pb_rdata_reg <= 8'h00;
                        // R_FIFOMODE — Sonora reads 0 (MAME asc.cpp:1043).
                        8'h03: pb_rdata_reg <= 8'h00;
                        // R_FIFOSTAT — return the latched byte directly.
                        // Read clears the IRQ line (handled in side-effect
                        // path above); does NOT clear the status bits.
                        8'h04: pb_rdata_reg <= fifostat;
                        // R_WTCONTROL — Sonora reads 0.
                        8'h05: pb_rdata_reg <= 8'h00;
                        // R_VOLUME — writable + readable.
                        8'h06: pb_rdata_reg <= volume_reg;
                        // R_CLOCK — depends on IS_EASC:
                        //   EASC (VERSION 0xB0): asc.cpp:1662-1664
                        //     `case R_CLOCK: return 3;` (read-only,
                        //     "this register is read-only on EASC and
                        //      shows what would be 44.1 kHz on the
                        //      original ASC").
                        //   Non-EASC (Sonora hybrid path): returns 0
                        //     per asc.cpp:1045 (`case R_CLOCK: return 0;`).
                        8'h07: pb_rdata_reg <= IS_EASC ? 8'h03 : 8'h00;
                        // R_BATMANCONTROL — Sonora reads 0; we expose
                        // rate_reg as the tb backdoor (Q700 ROM doesn't
                        // touch 0x808).
                        8'h08: pb_rdata_reg <= rate_reg;
                        // R_PLAYRECA.
                        8'h0A: pb_rdata_reg <= playreca_reg;
                        // R_TEST — readable scratch.
                        8'h0F: pb_rdata_reg <= test_reg;
                        // Local 0x810 / 0x811 per-FIFO bring-up status.
                        8'h10: pb_rdata_reg <= {fa_half_empty_irq,
                                            fa_fault_sticky,
                                            fa_overflow_sticky,
                                            fa_underflow_sticky,
                                            4'b0000};
                        8'h11: pb_rdata_reg <= {fb_half_empty_irq,
                                            fb_fault_sticky,
                                            fb_overflow_sticky,
                                            fb_underflow_sticky,
                                            4'b0000};
                        // 0x830..0x83F (EASC wavetable phase/increment) +
                        // any other unhandled register: Sonora reads 0.
                        default: pb_rdata_reg <= 8'h00;
                    endcase
                end else begin
                    // ─── EASC extended-register reads (0xF00..0xF3F) ─
                    // WRPTRA/RDPTRA/WRPTRB/RDPTRB return the LIVE FIFO
                    // pointers split into hi/lo bytes (per task brief —
                    // MAME shadows them in m_regs[] only; we surface
                    // actual state).  All other ext-block registers
                    // return their last-written latch.  CD-XA stub
                    // returns the latched byte; real silicon would
                    // return ADPCM filter coefficients here.
                    case (pb_addr)
                        // FIFO A side
                        12'hF00: pb_rdata_reg <= {6'b000000, fa_wp[9:8]};
                        12'hF01: pb_rdata_reg <= fa_wp[7:0];
                        12'hF02: pb_rdata_reg <= {6'b000000, fa_rp[9:8]};
                        12'hF03: pb_rdata_reg <= fa_rp[7:0];
                        12'hF04: pb_rdata_reg <= r_srca_h;
                        12'hF05: pb_rdata_reg <= r_srca_l;
                        12'hF06: pb_rdata_reg <= r_vola_l;
                        12'hF07: pb_rdata_reg <= r_vola_r;
                        12'hF08: pb_rdata_reg <= r_fifoa_ctrl;
                        12'hF09: pb_rdata_reg <= fa_irqen;
                        12'hF10: pb_rdata_reg <= r_cdxa_a;
                        // FIFO B side
                        12'hF20: pb_rdata_reg <= {6'b000000, fb_wp[9:8]};
                        12'hF21: pb_rdata_reg <= fb_wp[7:0];
                        12'hF22: pb_rdata_reg <= {6'b000000, fb_rp[9:8]};
                        12'hF23: pb_rdata_reg <= fb_rp[7:0];
                        12'hF24: pb_rdata_reg <= r_srcb_h;
                        12'hF25: pb_rdata_reg <= r_srcb_l;
                        12'hF26: pb_rdata_reg <= r_volb_l;
                        12'hF27: pb_rdata_reg <= r_volb_r;
                        12'hF28: pb_rdata_reg <= r_fifob_ctrl;
                        12'hF29: pb_rdata_reg <= fb_irqen;
                        12'hF30: pb_rdata_reg <= r_cdxa_b;
                        default: pb_rdata_reg <= 8'h00;
                    endcase
                end
            end else begin
                pb_rdata_reg <= 8'h00;
            end
        end
    end

    // ── IRQ line ──────────────────────────────────────────────────────
    // Driven by the registered irq_q flop (set/cleared by the various
    // event sites in the main always block — MAME's set_irq_line model).
    // Q700 board glue routes this active-high line to VIA2 CB1 as an
    // active-low external pin input.
    assign irq = irq_q;

    /* verilator lint_off UNUSEDSIGNAL */
    // EASC ext-block pointer shadows are latched-only (reads surface
    // live fa_wp/fa_rp/fb_wp/fb_rp).  Kept for MAME m_regs[] parity.
    wire [63:0] _unused_ptr_shadows = {r_wrptra_h_shadow, r_wrptra_l_shadow,
                                       r_rdptra_h_shadow, r_rdptra_l_shadow,
                                       r_wrptrb_h_shadow, r_wrptrb_l_shadow,
                                       r_rdptrb_h_shadow, r_rdptrb_l_shadow};
    // volume_reg is programmer-visible (writable + readable) but no
    // longer applied to audio output — MAME emits raw signed sample.
    // Keep the lint-suppressor reference so a future "remove
    // volume_reg" refactor doesn't drop the register accidentally.
    wire [7:0] _unused_volume = volume_reg;
    // Reference VERSION_*_REAL / VERSION_EASC constants so the
    // documented chip-id values aren't silently dropped by a tooling
    // pass when the active VERSION_USE select doesn't pick them all up.
    wire [23:0] _unused_version_const = {VERSION_EASC, VERSION_EASC_REAL,
                                         VERSION_SONORA};
    /* verilator lint_on UNUSEDSIGNAL */

endmodule

`default_nettype wire
