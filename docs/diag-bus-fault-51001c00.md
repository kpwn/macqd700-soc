# Diagnostic — vec=2 bus error reported on HW at PC 0x408046AA / EA 0x51001C00

Date: 2026-05-06.  Investigator: agent.  Branch: main @ HEAD.

## TL;DR — SPURIOUS in RTL? **NO sim repro.  RTL is consistent with MAME-canonical "OKAY+0 on unmapped".**

The reported HW vec=2 at the SIMM-detect probe address is *not*
reproduced by the current sim binary running the real Q700 ROM.  The
xbar correctly returns OKAY+0 for 0x51001C00, the dcache propagates
that response unchanged, the MMU is disabled (TC.E=0 at this PC), and
the LSU's bus-error gate (`dc_rresp != 2'b00`) never fires.  The
SIMM-detect routine completes and execution proceeds into the
ROM-checksum loop (final PC 0x40847516 at 80 k retired insns), exactly
as documented in `docs/rom_boot_bringup.md` §4.2b.

## Address-decode audit — xbar (rtl/sys/axi_xbar.v)

EA = 0x51001C00.  Decoded by `decode_slv()` (line 744):

| Region                            | Match?           | Note                              |
|-----------------------------------|------------------|-----------------------------------|
| RAM (`is_visible_low_ram_addr`)   | no               | not in 0x00..install_size         |
| RAM alias (0x58000000..0x5BFFFFFF)| no               | top byte 0x51, not 0x58           |
| ROM mirror (0x40000000..0x4FFFFFFF)| no              | top nibble 5, not 4               |
| FB (0x60000000..0x607FFFFF)       | no               |                                   |
| DMA (0x50100000..0x501FFFFF)      | no               | `0x51001C00 & ~0xFFFFF = 0x51000000` ≠ 0x50100000 |
| IO (0x50000000..0x50FFFFFF)       | **no**           | `0x51001C00 & ~0xFFFFFF = 0x51000000` ≠ 0x50000000 |
| VRAM (0xF9000000..0xF91FFFFF)     | no               |                                   |
| DAFB (0xF9800000..0xF98003FF)     | no               |                                   |
| **fall-through**                  | **XBAR_SLV_NONE**| local-response: OKAY + 0x00000000 |

The local-response path at lines 1839-1875 unconditionally drives
`rs_local_rsp = AXI_RESP_OKAY` and `rs_open_bus_zero = 1` (rdata = 0).
There is **no DECERR / SLVERR path** for unmapped CPU reads — verified
in source.  This matches MAME's `set_unmap_value=0` default for the
Q700 driver, per the comment block at line 1830-1838 (fixed
2026-05-03 after a halt-bisect found 0xFFFFFFFF was corrupting VBR).

## LSU bus-error gate (rtl/core/mem/lsu.v)

`cmpl_exc <= 1'b1` (with `cmpl_exc_vec <= 8'd2`) only fires from:

1. `mmu_fault_in` (line 638-653) — MMU translation fault.  Requires
   `mmu_enable = TC.E = 1`.  At PC 0x408046AA the ROM has not yet
   programmed TC, so `need_walk = mmu_enable & ... = 0` → fast-path,
   `mmu_fault = 0`.  Path inert.
2. `dc_rresp != 2'b00` after `S_LD_WAIT` (line 810).  With OKAY from
   the xbar, dcache propagates `r_resp = 2'b00` unchanged through
   `S_BY_LD_R` (line 1166) → `cmpl_exc = 0`.  Path inert.
3. Split-load second beat (line 883) — `cur_split` is 0 for a byte
   access at a longword-aligned address.  Path inert.

## Sim verification

```
$ make tb-fpga-top-rom FPGA_TOP_ROM_MAX_INSTS=80000 \
    FPGA_TOP_ROM_TIMEOUT=8000000 \
    FPGA_TOP_ROM_EXTRA="+probe +pc_trace_at_exc"
[io-discover] PC=0x408046aa A0=0x4080360c A1=0x408031b0 A2=0x50f01c00
              A3=0x00000000 A4=0x408000a6 A5=0x40802600 A6=0x40803178
              A7=0x40802600
[io-discover] D0=... D2=0x00100000 ...
... (ROM proceeds normally) ...
[fpga-rom] stop t=845894 retired=80000 pc=0x40847516 ...
```

ROM enters the SIMM-detect routine at PC 0x408046AA with the exact
register state the user described (D2=0x100000, A2=0x50F01C00, EA →
0x51001C00) and **proceeds through it without taking any exception**.
80 k retired instructions later it is happily looping in the ROM
checksum verification at 0x40847516 (per §4.2b).  No `[probe-exc]`,
no `[fault]`, no `[busfault]` printf.

`make test` confirms 612 PASS / 0 FAIL on this RTL — no regression.

## Hypothesis on HW divergence

If the user's bitstream actually shows vec=2 at this PC (and not
upstream / downstream code reading EXC_PC stale), candidate causes
that the agent could not rule out without HW access:

- **Stale bitstream**: user's bitstream predates `c3ef3fac` (the
  flush_resp sticky-OR fix) or some other today-landed RTL.  The same
  user-memory note records that commit fixed a similar but distinct
  vec=2 at 0x4080010E (PT-init).  A close-but-not-identical front-end
  bug (e.g. icache mis-tagging on a different redirect cascade) could
  yield a vec=2 pinned to a different fetch PC.
- **EXC_PC interpretation**: JTAG `r 0x50900028` reads the **last
  latched** EXC_PC (`debug_ctrl.v` OFF_EXC_PC = 0x00028).  If the CPU
  hit a vec=2 at some EARLIER PC and is now happily executing past
  0x408046AA, the JTAG poll sees a stale latched fault PC.
  `exc_count_r` (OFF_EXC_COUNT = 0x00064) would distinguish — non-zero
  & growing means a real recurring fault; static means latched-stale.
- **AXI handshake quirk**: real-HW MIG / xbar handshake skew not
  modelled in the Verilator unit-cycle sim.  Less likely given xbar
  has no SLVERR path for the NONE decode.

## Recommendation

1. **Do not modify RTL** — the sim conclusively shows the canonical
   path is correct.  The §4.2b logic is intact.
2. **HW-side check**: re-poll EXC_COUNT (`r 0x50900064`) repeatedly.
   If it stays static, the JTAG-visible EXC_PC is a stale latched
   value from an earlier (genuine) fault.  Then re-bisect that
   earlier fault with the m68k-fpga-halt-bisect skill.
3. **If EXC_COUNT IS growing on HW**: rebuild & flash the bitstream
   from current main HEAD (commits up to `8201320b` plus the
   uncommitted fetch-MMU work), since simulation says this exact
   RTL does not produce the fault.

## Key source pins

- `rtl/sys/axi_xbar.v:744-764` — `decode_slv` function; verifies
  0x51001C00 → XBAR_SLV_NONE.
- `rtl/sys/axi_xbar.v:1620-1645` — local-response data drive (OKAY
  + 0x00000000).
- `rtl/sys/axi_xbar.v:1829-1875` — unmapped-read OKAY policy with
  the historical comment block.
- `rtl/core/mem/lsu.v:638-653` — MMU fault → cmpl_exc path
  (gated by `mmu_fault_in`, which requires TC.E=1).
- `rtl/core/mem/lsu.v:810` — dcache rresp → cmpl_exc gate.
- `rtl/core/mem/dcache.v:1163-1170` (`S_BY_LD_R`) — non-cacheable
  bypass propagating r_resp transparently.
- `rtl/core/mem/mmu.v:368` — `need_walk = mmu_enable & ...`,
  proves walker stays idle when TC.E=0.
- `docs/rom_boot_bringup.md` §4.2a-b — canonical policy & history
  of the 0x51001C00 SIMM-detect probe.
