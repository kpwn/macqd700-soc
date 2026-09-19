# Endianness / byte-order audit (2026-04-28)

> Audit covering every endian-crossing boundary in the m68k-ooo data
> path, from the 68k LSU/IF stage out to DDR4, peripherals, VRAM, and
> the JTAG/PCIe debug bus.  Compare-point: MAME's `quadra700_map`
> address dispatch and per-device read/write helpers.

## Scope and definitions

The 68040 is **big-endian**: byte at lowest memory address is the most
significant byte of any 16/32-bit access.  Throughout this document:

- **BE-lane**: byte at lowest address sits at `wdata[31:24]` of a 32-bit
  AXI word; `wstrb[3]` guards that byte.  This is the 68k's natural
  on-the-bus layout — and it is what the LSU emits.
- **LE-lane**: byte at lowest address sits at `wdata[7:0]`; `wstrb[0]`
  guards it.  This is the AXI4 specification convention and what every
  Xilinx IP block (MIG UI, JTAG-AXI master, XDMA) emits/expects.
- **vram-LE-pixel**: VRAM packs pixel 0 (lowest pixel index, highest
  byte address bit ordering) in `[BPP-1:0]` of each 32-bit lane.  This
  is the convention `vram.v` expects on its slave port and that
  `fb_reader` consumes for scanout (`pb_dout[i*BPP +: BPP]`).
- **CPU-BE-pixel**: byte at lowest VRAM address arrives at
  `wdata[31:24]` of the 32-bit lane (the 68k LSU's natural BE-lane
  layout).  These two conventions disagree by a byte-reverse within
  each 32-bit lane — the P1 pixel-mirror finding.

The system runs on **AXI4 (LE-lane is the spec)**.  The CPU and the
peripherals (which mostly date to the 68k era) think BE-lane.  The
fabric therefore lives at a contradictory layering — and every
boundary either applies a transform or relies on lucky symmetry.  This
audit catalogs each one.

---

## §1 Boundaries inventory

| Boundary                            | Bus width        | Convention emitted     | Convention expected     | Code reference                                                                 | MAME parallel                                                                 | Verified by                                  |
|------------------------------------:|:-----------------|:-----------------------|:------------------------|:-------------------------------------------------------------------------------|:------------------------------------------------------------------------------|:----------------------------------------------|
| 1. LSU sub-word store assembly       | 32-bit narrow    | BE-lane (custom)        | n/a — produces           | `rtl/core/mem/lsu.v` lines 382–427 (`mk_strb`, `mk_wdata`)                      | `m68k_device::write` family — natural 68k BE-lane                              | tb-lsu, tb-mac-top                            |
| 2. LSU sub-word load extraction      | 32-bit narrow    | n/a — consumes          | BE-lane (custom)         | `rtl/core/mem/lsu.v` lines 429–462 (`mk_rdata`)                                  | `m68k_device::read` family                                                     | tb-lsu, tb-mac-top                            |
| 3. IF stage 128-bit fetch line       | 128-bit wide     | BE byte order (byte 0 at `[127:120]`) | BE byte order | `rtl/core/fetch/if_stage.v` line 13 + `rtl/core/decode/predecode.v` line 32–43 | n/a (Musashi consumes BE words straight from memory)                          | `tb_top.cpp` `load_fetch_line`                |
| 4. mac_top sim port (`daxi_*`)       | 32-bit narrow    | BE-lane                 | BE-lane                  | `tb/tb_top.cpp` `drive_daxi` lines 240–250 (`(1u << (3 - i))`)                 | n/a (sim only)                                                                 | every `make test` directed run                |
| 5. fpga_top `axi_narrow_to_wide`     | 32→128-bit       | BE-lane (passes through)| BE-lane (passes through) | `rtl/sys/axi_narrow_to_wide.v`                                                  | n/a (lane placement only — no swap)                                            | tb-axi-xbar, tb-mac-top-smc                   |
| 6. xbar S0 ↔ DDR (`ddr_ctrl`)        | 128-bit          | LE-lane storage         | LE-lane storage          | `rtl/sys/ddr_ctrl.v` lines 423–462 (`mem_b0..15` per-byte split)                | LE-lane (real MIG UI)                                                          | tb-ddr-model, tb-cold-boot, tb-rom-boot       |
| 7. xbar S1 ↔ peripheral_bus          | 128-bit          | BE-lane (lane select via `addr[3:2]`, byte select via `wstrb`) | BE-lane | `rtl/sys/peripheral_bus.v` lines 386–414 (`wr_byte` mux)                       | MAME Q700 `via_r/via_w/scc_r/scc_w` use byte replicate / `<<8` for high byte   | tb-peripheral-bus, tb-cold-boot               |
| 8. xbar S3 ↔ VRAM                    | 128-bit          | BE-lane in, LE-lane out  | LE-lane (`vram.v`)       | `rtl/sys/axi_xbar.v` lines 748–780, 933, 1526 (`vram_swap_words/strb`)         | n/a — Q700 silicon has no equivalent (CPU writes the VRAM aperture directly)  | tb-vram-xbar-e2e, tb-framebuffer-pixel        |
| 9. xbar S4 ↔ DAFB regs               | 128-bit→32-bit   | BE-lane (no swap)        | BE-lane                  | `rtl/sys/axi_xbar.v` lines 731–739 (`dafb_flatten`, `// no byte swap applied`) | MAME `dafb_device::map` 16- or 32-bit reads, BE                                | tb-peripheral-bus DAFB scenarios              |
| 10. boot_fsm SD pack → DDR           | 8→32→128-bit     | BE-pack (`{b0,b1,b2,b3}`) | LE-lane DDR storage     | `rtl/sys/boot_fsm.v` lines 803–820                                              | n/a (real Q700 has no SD bootstrap; ROM is raw mask)                          | tb-cold-boot scenario `test_endian`           |
| 11. JTAG-to-AXI debug master         | 32-bit narrow    | LE-lane (Xilinx IP)      | system bus is BE-lane    | `rtl/fpga_top_debug_host.vh` lines 315–394 (`debug_jtag_axi`)                  | n/a                                                                            | partial — `tools/jtag_repl.tcl` only does     |
|                                      |                  |                         |                          |                                                                                |                                                                                | full-word reads/writes.                       |
| 12. XDMA / PCIe debug master         | 128-bit wide     | LE-lane (Xilinx IP)      | system bus is BE-lane    | `rtl/fpga_top_debug_host.vh` lines 1–304 (PCIE_XDMA_ENABLE branch)             | n/a                                                                            | unverified end-to-end                         |
| 13. fb_reader → scaler → HDMI        | URAM read port   | LE-pixel (lane 0 = pixel 0) | LE-pixel                 | `rtl/sys/vram.v` lines 60–78 + `rtl/mac/video/fb_reader.v`                     | matches MAME `dafb_device::vram_r` per-byte pixel walk                         | tb-framebuffer-pixel, tb-video                |
| 14. ASC sample → audio bridge        | 16-bit packed     | BE pair `{L,R}` 8-bit  | BE pair                  | `rtl/sys/audio_hdmi_bridge.v` line 55, `rtl/sys/audio_i2s.v` line 61           | matches MAME `asc_base_device` stereo packing                                  | tb-audio                                      |
| 15. SCC `dc_ab_*` byte placement     | 16-bit MAME / 8 BE in our bus | BE high-byte of 16b | BE high-byte (or replicated) | `rtl/sys/peripheral_bus.v` line 860 (lane→addr derivation), `rtl/mac/scc.v` lines 19–26 | `MAME quadrax00_state::scc_r/w` uses `<<8` / `>>8` (high byte = data) | tb-scc (drives `pb_*` directly — does NOT cover AXI byte-lane derivation) |

Notes:

- The "BE-lane (custom)" emitted by the LSU is **not standard AXI4
  semantics**.  AXI4 says "wstrb[N] guards wdata[8N+7:8N]"; our LSU
  emits `wstrb=4'b1000, wdata[31:24]=byte` for a byte store at
  address-offset 0.  Standard AXI would interpret that as a byte
  store at address-offset 3.  The system survives because every
  consumer (DDR per-byte split, peripheral_bus `wr_byte` mux,
  narrow_to_wide replicate-and-place) is byte-symmetric: it picks
  whichever strb bit is hot regardless of which one "should" be hot.

- Boundary 8 is special.  After the b4623c6 commit (Codex tb-cold-
  boot deepening), `axi_xbar.v` itself applies `vram_swap_words` /
  `vram_swap_strb` on the S3 path.  But `rtl/fpga_top_video.vh`
  ALSO inlined the same swap on S3↔vram in production (commit
  098491b, P1 pixel-mirror fix from 2026-04-23).  The two cancel
  each other out — see §5 finding #1.

---

## §2 Brittle spots

### B1 — DOUBLE-SWAP on production xbar S3 ↔ VRAM (FIXED in this audit)

**Problem.**  `axi_xbar.v` applies a within-lane byte swap on its S3
master ports (`vram_swap_words` for `s3_wdata` and `s3_rdata`, and
`vram_swap_strb` for `s3_wstrb` — see lines 748–780, 933, 1526).
`rtl/fpga_top_video.vh` then applied THE SAME swap a second time
between `s3_wdata`/`s3_rdata` and the actual `vram` slave.  Net
effect: the swap cancels, the production VRAM path round-trips
unswapped, and CPU writes through the xbar re-create the original
**P1 pixel-mirror bug** (audit a8af9eb9590ea517a, 2026-04-23) that
the original `vram_cpu_byteswap.v` shim was supposed to fix.

**Example.**  CPU `MOVE.L #$0A0B0C0D, $F9000000`:

| Stage                                | wdata[31:0] for lane 0   |
|--------------------------------------|---------------------------|
| LSU (BE-lane)                        | `0x0A0B0C0D`              |
| narrow_to_wide (replicate)           | `0x0A0B0C0D` in lane 0    |
| xbar inputs `mw_wdata[0]`            | `0x0A0B0C0D`              |
| xbar `s3_wdata = vram_swap_words`    | `0x0D0C0B0A`              |
| `fpga_top_video.vh` `s3_wdata_swap`  | `0x0A0B0C0D` ← **mirror restored** |
| `vram` slave `pb_dout[7:0]` (pixel 0) | `0x0D` (should be `0x0A`)  |

**Why no test caught it:** `tb_vram_xbar_e2e` instantiates only the
xbar→vram leg and gets a single swap (correct).  `tb_framebuffer_pixel`
uses the standalone `vram_cpu_byteswap.v` module (single swap).
**No test stitches xbar AND fpga_top_video.vh together**, so the
double-swap only appears in the real bitstream.

**Fix shape:** remove the inline swap in `fpga_top_video.vh`; let the
xbar's swap stay as the single canonical one.  **Implemented in this
audit** — see §5 / commit accompanying this doc.

---

### B2 — Three independent BE↔LE swap conventions, no central spec

**Problem.**  Three separate code paths apply the *same* within-lane
byte-reverse with copy-pasted bit assignments:

1. `rtl/sys/vram_cpu_byteswap.v` — module form, used by tb only.
2. `rtl/sys/axi_xbar.v` `vram_swap_word32` — function form, used on
   the production xbar S3 boundary.
3. `rtl/fpga_top_video.vh` — inline `assign` form (now deleted by B1).

Each is hand-written.  A future agent that adds a new VRAM-class
slave (e.g. a DAFB indirect-FB aperture or an HDMI capture path) is
likely to re-implement the swap a fourth time, incompatibly.

**Recommendation:** make `vram_cpu_byteswap.v` the canonical form,
have `axi_xbar.v` instantiate it on its S3 boundary instead of an
inline function, and delete the function.  Same RTL, single source
of truth.  Track as follow-up.

---

### B3 — JTAG-AXI master uses LE-lane, system fabric uses BE-lane

**Problem.**  Real Xilinx IP (the `debug_jtag_axi` master, pcie_xdma)
emits standard AXI4 LE-lane: `wstrb[0]` guards `wdata[7:0]` =
byte at lowest address.  Our LSU emits BE-lane: `wstrb[3]` guards
`wdata[31:24]` = byte at lowest address.

For full-word writes (`wstrb=0xF`) this disagreement is invisible —
any byte ordering of the strobe bits doesn't matter when they're all
hot.  `tools/jtag_repl.tcl` only ever issues full-word writes/reads,
so the human-driven flow is safe.

For sub-word writes from JTAG (e.g. someone runs `hw_axi` with a
single-byte poke), the resulting CPU-visible memory state is the
**transpose of what the operator intended** — a byte poke at addr A
lands at addr A^3 in CPU view.

**Example.**  Operator's intent: `w 0x40000000 0xAB` as a byte store.
JTAG-AXI sends `awaddr=0x40000000, awsize=0, wdata=0x000000AB,
wstrb=4'b0001`.  DDR stores `mem_b0=0xAB`.  CPU reads byte at addr 0
= `rdata[31:24]` = `mem_b3 = 0`.  CPU reads byte at addr 3 =
`rdata[7:0]` = `mem_b0 = 0xAB`.

**Recommendation:** Document the convention in `tools/jtag_repl.tcl`
header AND restrict the REPL helpers to full-word ops only.  Provide
a `wb addr byte` helper that emits a 4-byte-aligned full word with
the byte placed at `[31:24]` (BE-lane) regardless of `addr[1:0]`,
mirroring the LSU contract.  Track as follow-up.

---

### B4 — Sub-word peripheral writes lose data on word-strided stores

> **STATUS 2026-08-18 — half fixed, and this entry's own example is wrong.**
> The file is `rtl/soc/peripheral_bus.v` (not `rtl/sys/`).  The
> byte-granular half of this finding is **fixed**: multi-hot writes to
> ASC, ORWELL and SONIC are now serialized into one `pb_wr` pulse per hot
> strobe bit at ascending byte addresses — see
> `docs/periph_multihot_write_serializer.md`.  The strided half (VIA1,
> VIA2, IWM, SCSI register page, SCC) is **not** fixed and must not be
> "fixed" the same way; on those slots `AWADDR[1:0]` is not in the decode,
> so serializing would pulse the *same* stateful register N times.  Their
> real defect is *which* byte survives, and it is deliberately still open.
> **The worked example below is arithmetically wrong**: with
> `wstrb=4'b1100` the priority chain reaches `wr_lane_strb[2]` first and
> selects `wr_lane_data[23:16]` = `0x34`, not `0x12`.  The direction of
> the error matters — the mux keeps the *last* byte of a big-endian
> store, not the first.  Also, "this is fine for the Mac ROM, which only
> does byte stores to peripherals" was never sourced and is false in
> general (`docs/mame_periph_multihot_reachability.md`); it happens to
> hold for VIA1/VIA2/IWM/SCSI-regs/SCC, which is why those stayed latent.

**Problem.**  `rtl/sys/peripheral_bus.v` `wr_byte` mux picks the
lowest hot strb bit (`wr_lane_strb[0] ? ... : ... : wr_lane_strb[3]`).
For a CPU `MOVE.W` with `wstrb=4'b1100` (bytes 0+1 of the operand),
the mux picks `wr_lane_data[15:8]` (byte 1) and discards byte 0.

This is fine for the Mac ROM, which only does byte stores to
peripherals.  It's a latent bug if any future peripheral is word-
addressable.  MAME handles this with `ACCESSING_BITS_8_15` /
`ACCESSING_BITS_0_7` masks — both halves of the word can be valid.

**Example.**  CPU `MOVE.W #$1234, $5000_0000` (VIA1 reg 0, longword
intent).  LSU emits `wstrb=4'b1100`, `wdata[31:16]=0x1234`.  In
peripheral_bus, `wr_byte = wr_lane_data[15:8] = 0x12`.  The 0x34
half is dropped.  VIA1 sees a single byte write of 0x12, but ROM
expected the 16-bit value 0x1234.

**Recommendation:** widen the pb_* face from 8-bit to a (byte0, byte1,
strobe-mask) tuple so peripherals can latch wider words.  Today's
peripherals (VIA, SCC, ASC, IWM) are inherently 8-bit so they do not
need this — but DAFB regs (32-bit) and a future SCSI DMA (16-bit)
might.  Track as follow-up.

---

### B5 — boot_fsm BE-pack convention is silently coupled to DDR LE-lane

**Problem.**  `rtl/sys/boot_fsm.v` line 819 packs four SD bytes as
`{pack_b0, pack_b1, pack_b2, ctrl_rd_data}`, i.e. SD-byte-0 at
`wdata[31:24]`.  Then `wstrb=4'b1111` and the narrow→wide adapter
splats the longword across all four 128-bit lanes by `addr[3:2]`.
DDR_ctrl stores per-byte: `mem_b0 ← wdata[7:0]`, etc.  Net: SD byte
N lands at the same DDR byte the CPU sees as "byte at addr N" given
the LSU's BE-lane contract.

**This works today**, but the chain is fragile.  Any of:
- changing the LSU's `mk_rdata` sub-word convention,
- changing DDR's per-byte split to use `mem_b15` for `wdata[7:0]`,
- changing narrow→wide's lane placement,
- changing boot_fsm's pack from `{b0,b1,b2,b3}` to `{b3,b2,b1,b0}`,

would break the round trip.  The memory note
`project_boot_fsm_byte_order_20260423.md` raised this hypothesis on
2026-04-23 and tagged it for `tb-cold-boot`.

**Status — REFUTED.**  `tb/tb_cold_boot.cpp` `test_endian` (registered
in `g_scenarios[]` at line 936) writes SD bytes `0..15` and asserts
`line_byte(if_rdata, 0/4/8/12) == 0x0C/0x08/0x04/0x00`.  The check
exactly probes the `if_to_axi` 128-bit reversal AND the BE/LE pack
agreement.  The scenario passes today.  The hypothesis from the
memory note is **incorrect** — the path is consistent end-to-end —
but the hypothesis writer was right that the consistency is
non-obvious and depends on FOUR separate conventions agreeing.

**Recommendation:** retain the `test_endian` scenario as the gate;
add a comment to `boot_fsm.v` line 819 cross-referencing the LSU
BE-lane contract and the DDR per-byte split.  Track as follow-up.

---

### B6 — `tb_peripheral_bus.cpp` axi_write helper uses LE-lane convention

**Problem.**  `tb/tb_peripheral_bus.cpp` `axi_write` line 350:
```c
strb = (1u << (lane * 4 + byte_in_lane));
```
`byte_in_lane=0` sets `strb` bit 0 = standard AXI LE-lane, byte at
lowest address.  But the LSU produces `byte_in_lane=3` (bit 3 hot)
for the same byte at lowest address.

The tb papers over this with the comment at line 706:
```
// byte_in_lane here is the *AXI-level* little-endian byte position
// within the 32-bit lane.  For awaddr=0x2800+k (k∈{0..3}) the CPU
// drives wstrb bit (3 - k) → byte_in_lane = (3 - k).
```
But ALL existing `axi_write(... 0, ...)` callers (e.g. VIA1, VIA2,
SCC, ASC, IWM tests) use `byte_in_lane=0` — that's bit 0 hot, which
peripheral_bus `wr_byte` interprets as byte at offset 3 (LE-lane).
The TB and the DUT's `wr_byte` happen to agree because the DUT
picks the lowest hot strb bit; what **gets tested** is the
peripheral seeing whichever byte the test put at `wdata[7:0]`.
That happens to match `byte_in_lane=0` if the test also wrote
the value to `wdata[7:0]` (which `axi_write` does via
`w[lane] = lane_data;`).  But this is **not** the layout the LSU
emits.

**Example.**  Real CPU `MOVE.B #$AB, $5000_0000`:
- LSU emits `wstrb=4'b1000`, `wdata[31:24]=0xAB`.
- peripheral_bus `wr_byte` = `wr_lane_data[31:24] = 0xAB`.

`tb_peripheral_bus`'s `axi_write(0x5000_0000, 0xAB, 0)`:
- emits `wstrb=4'b0001`, `wdata[7:0]=0xAB`.
- peripheral_bus `wr_byte` = `wr_lane_data[7:0] = 0xAB`.

Both give the same `wr_byte` value — but via different strb bits.
The two tests below — the inline `test_asc_byte_select` and the more
common `test_via1_byte_rw` — exercise opposite ends of the byte-
position space.  A bug in the BE-lane path of peripheral_bus would
slip through `test_via1_byte_rw` because that test uses the LE-lane
convention.

**Recommendation:** add a parallel `axi_write_be(addr, byte)` helper
that emits the BE-lane convention (`wstrb = 1 << (3 - addr[1:0])`,
`wdata[(3-(addr&3))*8 +: 8] = byte`), and re-run the VIA1/VIA2/
SCC/ASC tests through it.  This is the directed-test offering listed
in §5 below — implemented as `tb_endianness_byte_lane.cpp`.  Track
as follow-up to extend the peripheral tb.

---

### B7 — JTAG-display ambiguity for BE-storage interpretation

**Problem.**  When the operator types `r 0x40000000`, jtag_repl.tcl
displays the 32-bit AXI rdata as a hex word.  Standard AXI rdata is
LE-lane.  Real DDR (post-MIG) stores LE-lane natively, so `r addr`
and `r addr+1` and so on report the four bytes as if they were a
little-endian-packed longword.  But the CPU consumes that same DDR
word as a BE longword.

**Example.**  CPU `MOVE.L #$AABBCCDD, $40000000`:
- LSU writes BE-lane: `wstrb=0xF, wdata=0xAABBCCDD`.
- DDR stores: `mem_b0=0xDD, mem_b1=0xCC, mem_b2=0xBB, mem_b3=0xAA`.
- Operator types `r 0x40000000` from JTAG:
  AXI returns LE-lane `rdata = {mem_b3, mem_b2, mem_b1, mem_b0} =
  0xAABBCCDD`.
- jtag_repl displays `0xAABBCCDD`.  Looks correct.

But for byte-level interpretation:
- Operator wants "byte at addr 0x40000000": expects `0xAA`.
- AXI rdata[7:0] = `mem_b0 = 0xDD` (the standard LE convention).
- The display "rdata = 0xAABBCCDD" requires the operator to read
  the high byte to find the byte at the lowest address.

This is consistent with how MacsBug displays memory (BE-MSB
left-to-right) but **not** with how Vivado's `hw_axi` typically
shows byte-level reads.  Fine for word-level debugging, surprising
for sub-word.

**Recommendation:** add a `dump-mem-be` command alias to
`jtag_repl.tcl` that renders memory as a BE byte stream so MacsBug
veterans can read it without mental gymnastics.  Track as follow-up.

---

## §3 Recommended canonical conventions

Apply per-boundary type, top-down:

1. **CPU LSU emit and consume**: BE-lane (status quo, locked in by
   `mk_strb`/`mk_wdata`/`mk_rdata`).  Document at the top of
   `lsu.v` that the LSU emits the **inverse of standard AXI4
   wstrb-byte mapping**, and every downstream consumer of CPU
   sub-word writes must respect that.
2. **CPU IF stage 128-bit fetch line**: BE byte order, byte 0 at
   `[127:120]`.  The 4-word reverse in `if_to_axi.v` is the
   reconciliation against DDR's LE-lane storage.  Status quo.
3. **Sim-side mac_top `daxi_*` port**: BE-lane (matches LSU).
   `tb_top.cpp` `drive_daxi`'s `1u << (3-i)` is correct.
4. **Real-bitstream narrow→wide adapter (`axi_narrow_to_wide`)**:
   pass-through; no swap.  Status quo.
5. **DDR `mem_bN` storage**: LE-lane (matches Xilinx MIG).  Status
   quo.  The CPU's BE-lane survives the round trip because LSU's
   `mk_*` produces a layout that maps byte-N-of-CPU-operand to
   byte-N-of-DDR-RAM-address through a hidden inverse-permutation
   in `wstrb`.
6. **Peripheral bus byte select**: pick whichever strb bit is hot,
   take the corresponding wdata byte.  Status quo.  Effectively
   byte-symmetric — works for both BE-lane and LE-lane callers.
7. **VRAM xbar S3 boundary**: ONE within-lane byte-reverse, applied
   inside `axi_xbar.v` (using `vram_swap_words` / `vram_swap_strb`).
   `fpga_top_video.vh` MUST NOT inline a duplicate.  After the fix
   in this audit (§5 #1), this is now invariantly true.
8. **JTAG / PCIe debug masters**: standard AXI4 LE-lane.  Document
   the inversion vs the CPU's BE-lane for sub-word ops.  Provide a
   `wb` (write-byte) helper in `jtag_repl.tcl` that synthesises a
   full-word write with the byte placed BE-lane-wise.

---

## §4 Test gaps

What unit tbs are missing or under-coverage:

1. **`tb_endianness_byte_lane.cpp`** (new, see §5 directed test).
   Asserts that a CPU-style BE-lane byte store at addr `A` lands in
   `peripheral_bus`'s `wr_byte` correctly for ALL four
   `addr[1:0] ∈ {0,1,2,3}` cases AND for ALL four
   `addr[3:2] ∈ {0,1,2,3}` lane positions.  Today only the
   `byte_in_lane=0` (LE-lane) cases are exercised end-to-end.

2. **`tb-vram-xbar-cpu-end-to-end.cpp`** (new — recommended).
   Stitch `axi_narrow_to_wide` + `axi_xbar` + `fpga_top_video.vh`
   inline mux + `vram` together (i.e. the exact production stack)
   and drive a CPU-style BE-lane longword write at an aperture
   address.  Read back via the scanner port.  This is the test
   that would have caught the B1 double-swap regression on commit
   day.

3. **`tb-jtag-axi-byte-poke.cpp`** (new — recommended).  Drive the
   `axi_xbar` with a JTAG-AXI-style byte store (LE-lane) and verify
   the CPU sees the byte at the **CPU-lane** position.  Catches the
   B3 inversion if the JTAG path ever accepts sub-word writes.

4. **`tb-cold-boot` `test_endian` already exists** and gates the
   B5 boot_fsm convention.  No action needed; cite from the audit.

5. **Fuzz integration** — the existing `tools/fuzz` runs against
   Musashi using `tb/tb_top.cpp`'s `daxi_*` port (BE-lane).  Fuzz
   does NOT exercise the xbar, MIG, peripheral_bus, or VRAM byte
   paths.  All of §1 boundaries 6–15 are out-of-fuzz-scope.
   Recommend a separate "system fuzz" stream that drives random
   sub-word AXI traffic through the full fabric.

---

## §5 Concrete bugs found

### Bug 1 — Production VRAM CPU-write path is double-swapped (FIXED)

Severity: **A** (user-visible: any CPU write to the VRAM aperture
produces mirrored pixels in HW first-light).

Root cause: see §2 B1.  Diff-able fix: remove the inline byte-swap
in `rtl/fpga_top_video.vh` lines 184–221 (the `s3_wdata_swap` /
`s3_wstrb_swap` / `s3_rdata_swap` generate block) and replace the
mux assignments to feed `s3_wdata` / `s3_wstrb` / `vram_s_rdata`
through directly.  The xbar's `vram_swap_words` already does the
necessary BE→LE permutation.

**Fixed in this audit's accompanying RTL commit.**  The
`tb-vram-xbar-e2e` and `tb-framebuffer-pixel` tbs both PASS the
byte-order scenarios with the fix in place (post-fix: 11/13 PASS
on tb-vram-xbar-e2e — 2 pre-existing OOB-SLVERR failures unrelated
to byte order, see commit 6549d2a's open-bus policy switch).

The fix touches only `fpga_top_video.vh`; `axi_xbar.v` is
unchanged.

### Bug 2 — None other found by this audit

No other end-to-end correctness bug was detected.  The convention
disagreements documented in §2 B2–B7 are real — they are landmines
for future agents — but no current code path miscomputes a CPU-
visible value as a result.

In particular:

- The boot_fsm BE-pack ↔ DDR LE-lane chain works (refutes the
  P1#7 hypothesis from `project_boot_fsm_byte_order_20260423`).
- The peripheral_bus `wr_byte` mux is byte-symmetric and produces
  the right byte regardless of which strb bit is hot.
- The JTAG-AXI LE-lane vs CPU BE-lane disagreement is invisible
  for the only code path that uses it today (full-word ops in
  `jtag_repl.tcl`).

---

## Appendix A — Trace tables for the audit

### A.1 — CPU byte store `MOVE.B #$AB, $40000000` round-trip through DDR

| Stage                            | Wire / register state                                           |
|----------------------------------|------------------------------------------------------------------|
| LSU `mk_strb(BYTE, 0)`            | `wstrb = 4'b1000` (bit 3 hot)                                   |
| LSU `mk_wdata(BYTE, 0, 0xAB)`     | `wdata = 0xAB000000` (BE-lane)                                  |
| narrow→wide replicate, lane=0    | `w_wstrb = 16'h0008`, `w_wdata` = `0xAB000000` × 4 lanes        |
| DDR per-byte write                | `mem_b3[0] ← wdata[31:24] = 0xAB`                               |
| CPU `MOVE.B $40000000, D0` load   | DDR returns `rdata[31:24] = mem_b3 = 0xAB`                      |
| LSU `mk_rdata(BYTE, 0)`           | `mk_rdata = sign_extend(rdata[31:24]) = 0xFFFFFFAB`             |

### A.2 — JTAG-AXI longword write `w 0x40000000 0xDEADBEEF`

| Stage                            | Wire / register state                                           |
|----------------------------------|------------------------------------------------------------------|
| `debug_jtag_axi` master           | `wstrb = 4'b1111`, `wdata = 0xDEADBEEF` (LE-lane)               |
| narrow→wide replicate, lane=0    | `w_wstrb = 16'h000F`, `w_wdata` = `0xDEADBEEF` × 4 lanes        |
| DDR per-byte writes               | `mem_b0=0xEF, mem_b1=0xBE, mem_b2=0xAD, mem_b3=0xDE`             |
| CPU `MOVE.L $40000000, D0`        | DDR `rdata[31:0] = {mem_b3, mem_b2, mem_b1, mem_b0} = 0xDEADBEEF` |
| LSU `mk_rdata(LONG, 0)`           | `mk_rdata = 0xDEADBEEF` ✓                                        |

The longword round trip works.  Sub-word writes from the JTAG side
have the inversion described in §2 B3.

### A.3 — boot_fsm SD bytes `[0x4E, 0x71, 0x4E, 0x71]` → ROM image @ addr 0

| Stage                            | Wire / register state                                           |
|----------------------------------|------------------------------------------------------------------|
| boot_fsm pack                    | `wdata = {0x4E, 0x71, 0x4E, 0x71} = 0x4E714E71`, `wstrb=0xF`     |
| narrow→wide                       | `w_wdata` = `0x4E714E71` × 4 lanes                              |
| DDR per-byte writes               | `mem_b0=0x71, mem_b1=0x4E, mem_b2=0x71, mem_b3=0x4E`             |
| if_stage `if_addr = 0`            | issues 16-byte fetch line; expects byte 0 at `if_rdata[127:120]` |
| if_to_axi 4-word reverse          | `if_rdata[127:96] = m_rdata[31:0] = 0x4E714E71`                  |
| `if_rdata[127:120]`               | `0x4E` = NOP byte 0 ✓                                            |

The cold-boot `test_endian` scenario in `tb_cold_boot.cpp` exercises
exactly this path with bytes 0..15 and asserts byte 0 lands at
`if_rdata[127:120]`.  Refutes the P1#7 hypothesis.
