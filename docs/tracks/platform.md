# Platform / SoC track

> Read `docs/agent_policy.md` first. This brief adds platform-track scope.

## Scope

SoC plumbing that surrounds the CPU + peripherals: AXI crossbar, DDR,
HDMI, SD, clocking, synth/impl flow, KU5P constraints. Everything that
makes the design a buildable bitstream.

## Files you own

```
rtl/sys/{axi_xbar.v, clk_rst.v, ddr_ctrl.v, peripheral_bus.v,
         async_fifo.v, axi_async_bridge.v, dma_ctrl.v}
rtl/sys/axi_narrow_to_wide.v
rtl/soc/fpga_top.v         (SoC top — real FPGA instance wiring)
synth/{vivado.tcl, ku5p.xdc, impl_from_dcp.tcl, route_from_place.tcl}
tb/{tb_axi_xbar, tb_async_fifo, tb_axi_async_bridge, tb_dma_ctrl,
    tb_peripheral_bus, tb_ddr_model, tb_sd_boot, tb_sd_ctrl,
    tb_video, tb_vram, tb_mac_top_smc}.cpp
docs/{clocking.md, hardware_roadmap.md, hardware_feasibility.md,
      legacy_phy_refs.md, dma_ctrl.md}
```

Do NOT touch `rtl/core/*` (core), `rtl/mac/*` (peripheral), `tb/tb_top.cpp`
(coordination hotspot).

## KU5P budget

Part: `xcku5p-ffvb676-2-i` (or `-1` slower speed grade).

```
LUTs     216,960  (today: ~30% after dcache-bram + l1i-real)
FFs      432,960  (today: ~16%)
RAMB36       480  (today: ~8 used — 4 dcache, 4 icache tag/data)
RAMB18       960
URAM288       80  (VRAM lives here)
DSP58E2    1,824  (today: 4 for mul_div)
MMCM / PLL  12 / 24 in UltraScale+ (plenty)
```

Current post-route (100 MHz target, last synth): WNS +0.305 ns,
hold −0.026 ns (1 endpoint), pulse-width −1.266 ns (4 endpoints, async
paths), power 1.66 W.

200 MHz target (the real goal) is still a phase-boundary PM concern, not
agent default work.  Phase-C C1 / F1 registered PRF read has landed; the
remaining risk is width-related fanout, residual control distribution, and
ALU/divide path cleanup.  The canonical hardware signoff path is now the
100 MHz real-MIG debug build; generic `make impl` remains available for local
smoke work but is not the board bring-up default.

## AXI crossbar

Today (2026-07-16 master-count reduction): **3M × 6S** (`rtl/soc/axi_xbar.v`
— note the file moved from `rtl/sys/` to `rtl/soc/` in an earlier split;
update any remaining `rtl/sys/` references you see in older docs):

```
Masters:                          Slaves:
  M0 = CPU LSU + boot FSM          S0 = DDR (RAM/ROM/legacy FB)
       (merged — see below)        S1 = Peripheral bus (pb_clk island)
  M1 = Host debug                  S2 = DMA config (AXI-Lite; master
  M2 = CPU instruction fetch            port to dma_ctrl STUBBED, below)
       (read-only)                S3 = VRAM pixel aperture
                                   S4 = DAFB register shim
                                   S5 = SD JTAG writer
```

Was 5M (CPU LSU, host debug, boot FSM, CPU IF, DMA) before the reduction.
Two changes:
- **Boot FSM merged onto M0** via a `cpu_held_in_reset`-selected 2:1 mux
  inside `axi_xbar.v` (see its header "M0/boot merge") — boot FSM only
  ever drives AXI while the CPU is held in reset, and CPU LSU only drives
  it after release, so the two windows are provably disjoint and a plain
  mux (not an arbiter) is correct.  CPU LSU and CPU instruction fetch
  stay independent masters — never merge those two.
- **DMA's AXI4 master port (was M4) is stubbed out** — `dma_ctrl.v` had
  zero live consumers, so its `m_axi_*` port is tied to a permanently-
  idle slave in `fpga_top_dma.vh` rather than kept wired to a crossbar
  seat it never used.  `dma_ctrl.v` itself and its AXI-Lite config slave
  (S2, still reachable from any master) are untouched; re-add the M4 port
  and the 64→128 widening bridge when a real DMA consumer lands.

Reset overlay and DAFB decode are xbar-level policy.  The CPU core presents
raw addresses; no production peripheral is directly hooked to core data
paths except IRQ levels.

Data width: 128 bit on xbar; CPU's 64-bit master goes through
`axi_narrow_to_wide`.

## SD image split

`rtl/sys/sd_ctrl.v` now provides `sd_image_lba_map`, a zero-latency helper
that reserves the first 4 MiB of the SD image (8192 512-byte sectors) for
the ROM image and biases raw-disk LBAs above that window when `raw_sel=1`.
The future SCSI-disk path should reuse the helper with `raw_sel=1` rather
than open-coding the offset.  (`sd_provision.v` was REMOVED; SD
provisioning happens host-side, before power-up.)

Host-side first-light image checks live in `tools/m68kctl/sd_image.py`.
Run `PYTHONPATH=tools python3 -m m68kctl sd check-layout --rom files/420dbff3.rom --hdd <raw.hdv>`
before writing a card image; it rejects ROM/HDD layouts that would overlap
the 4 MiB ROM window.

## BRAM inference patterns (CRITICAL)

**XST max = 2 ports per BRAM.** Multi-port reads (e.g. 4-way parallel
cache read) FORCE demotion to distributed LUTRAM and explode LUT count.
Caught dcache at 46,500 LUTs before the per-way split landed.

**Canonical "simple dual-port read-first" template** (from UG901 §Block
RAM):

```verilog
(* ram_style = "block" *) reg [31:0] mem [0:255];
reg [31:0] dout;
always @(posedge clk) begin
    if (we) mem[waddr] <= din;
    dout <= mem[raddr];   // registered read
end
```

- One write, one registered read, same clock.
- Byte-lane writes: 4 `if (we[i]) mem[waddr][...] <= ...` guarded
  assignments, each 8 bits.
- 4-way-associative caches → 4 **independent per-way** simple-DP
  BRAMs, not one 2D array.
- For true dual-port (2 writes/cycle or concurrent 2R+1W), RAMB36
  native mode supports 1R+1W per port.

## DDR

`rtl/sys/ddr_ctrl.v` has two paths:

- **SIM_MODEL** (default): BRAM-backed simulation mock. 64 KB
  (`BRAM_LOG2_BEATS=12`) — enough for ROM + early RAM. Byte-split
  inference avoids the old 524K-LUT explosion.
- **Real-hardware MIG path** (`ifndef SIM_MODEL`): exposes the AN9134 DDR4
  physical pins, instantiates the pcie_test DDR4 MIG black box, bridges the
  repo 128-bit DDR AXI contract to the pcie_test 256-bit UI contract, and
  crosses from core clock to MIG UI clock through `axi_async_bridge`.
  Real FPGA bitstream targets set `REAL_FPGA_BUILD=1 USE_REAL_MIG=1`; the
  Vivado flow refuses `REAL_FPGA_BUILD=1` if SIM_MODEL would be defined.

The current first-light checks are `make ddr-pcie-test-check` for the
contract facts, `make tb-axi-ddr4-mig-bridge` for the AXI shim, and
`make ddr-pincheck` for the real-pin Vivado smoke path.  The successful
100 MHz bitstream flow is the hardware baseline, but the MIG artifact still
needs to become repo-owned through Tcl/XCI generation instead of relying on an
out-of-project pcie_test DCP.  The remaining open items are board calibration
evidence, timing closure after each hardware-facing change, and burst policy
beyond first-light single-beat traffic.

## Vivado flow

**Mutex** at `/var/tmp/m68k-ooo-vivado.lock`. The Makefile uses
`flock -n` and exits 75 on contention. Two concurrent KU5P synths OOM
a 62 GiB machine (each 12-15 GiB RSS + 4 workers at 3-5 GiB). Don't
remove `-n`.

Commands:

```bash
make fpga-first-hw-offline-preflight # DDR/MIG contract + Verilator checks, no Vivado
make fpga-100mhz-preflight      # canonical 100 MHz Vivado elab/pin preflight
make fpga-100mhz-jtag-bitstream-dram # canonical 100 MHz real DDR4 + VIO/JTAG-AXI bitstream
make synth                       # generic synth_only — local timing/util work
make impl                        # generic full place+route; REAL_FPGA_BUILD=1 now fails unless debug-capable or explicitly overridden
TARGET_FREQ_MHZ=100 make impl # override target (default 200)
make timing                   # post-route timing report
make gui STEP=route           # open in Vivado GUI
```

Full Vivado synth/impl/bitstream for the canonical hardware target:

```bash
make fpga-100mhz-jtag-bitstream-dram
```

That expands to:

```bash
USE_REAL_MIG=1 REAL_FPGA_BUILD=1 ENABLE_VIO=1 ENABLE_JTAG_AXI=1 \
TARGET_FREQ_MHZ=100 CORE_CLK_DIVIDE=1 CORE_CLK_HZ=100_000_000 \
VIDEO_SMOKE=1 HDMI_TEST_PATTERN=1 BOOT_ROM_SECTORS=2048 \
make impl
```

The explicit board targets also verify that implementation emitted the
bitstream, matching `.ltx`, and build manifest needed for hardware debug.
The generic `make synth` / `make impl` targets remain available for local smoke
work, but real board bitstreams should use the explicit real-MIG target.

## Common pitfalls

- **`make sim` is retired in this split repo.** rtl/core/* moved to the
  cpu/ submodule, so the old flat `mac_top.v` Verilator sim top no longer
  builds here (`rtl/soc/mac_top.v` was a stale pre-split duplicate and
  has been deleted — build-truth-hygiene cleanup).  Use `make
  tb-fpga-top-rom` (full CPU+SoC sim) or `cd cpu && make sim`
  (CPU-only monorepo build) instead.
- **Stale /tmp/m68k-ooo-build.** When in doubt, `rm -rf build/sim` and
  rebuild. Also when you add/delete .s files, rsync with `--delete`.
- **`fpga_top.v` is the real SoC top** (MIG, clocking, pin mapping),
  at `rtl/soc/fpga_top.v` — not `rtl/fpga_top.v`.
- **Speed grade.** `-2` is the cheaper common part. `-1` buys ~5%
  Fmax for a price bump. All constraints target `-2`.
- **XDC false-path / async CDC.** Clock-domain crossings need
  `set_false_path` or `set_max_delay` constraints in `synth/ku5p.xdc`.
  New async-fifo instances need matching constraints.

## Testing rigour

Every platform upgrade needs:

1. Unit tb for the new module (existing conventions: tb-axi-xbar,
   tb-async-fifo, tb-axi-async-bridge).
2. Integration check — the design still builds (`make sim`) and tests
   pass at or above the authoritative baseline in `docs/agent_policy.md`.
3. For xbar / interconnect changes, an address-decode stress test.
4. PM queues a phase-boundary `make synth` to confirm util + timing
   impact is as expected. Don't run it yourself.

## References

- `docs/hardware_roadmap.md` — H0 → H1 → H2 phases.
- `docs/hardware_feasibility.md` — KU5P BOM + util analysis.
- `docs/clocking.md` — 11 clock domains + CDC patterns A/B/C.
- `docs/dma_ctrl.md` — DMA integration spec (#110).
