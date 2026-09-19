# PCIe + XDMA debug interface

> **Status (2026-05-02): inactive.**  The canonical build flow runs with
> `ENABLE_PCIE_XDMA=0` under Vivado 2025.2 and the only host debug path is
> JTAG-AXI.  The wiring notes below were last validated against Vivado
> 2023.1 / xdma_v4_1 and remain accurate as a recipe; re-enabling PCIe
> requires regenerating the IP under 2025.2 (xdma:4.2).

Architecture and register catalogue for host-side introspection of the
m68k-ooo CPU and platform during FPGA bring-up.

Companion docs:
- [gameplan.md](gameplan.md) — phase 2 references this for bring-up
- [peripheral_arch.md](peripheral_arch.md) — system bus integration
- [pcie_xdma_generation.md](pcie_xdma_generation.md) — repo-owned XDMA IP flow

---

## Why this exists

Iterating on a CPU bring-up with JTAG-only is painful: every new probe
requires a 30–60 minute bitstream rebuild. With PCIe + XDMA the workstation
becomes a first-class peer on the AXI bus and any debug visibility is just
an `mmap` away — no rebuilds.

Three concrete capabilities this unlocks:

1. **Host-driven boot**: DMA the ROM image into DDR4 from Python in
   ~50 ms instead of waiting on SD/SCSI to come up.
2. **Live introspection**: PC trace, ROB dump, performance counters,
   cache state — read at any time without halting the CPU.
3. **Active debug**: halt, single-step, force-redirect, inject interrupts,
   patch memory, override `init_done`. The host can shape every aspect
   of execution.

---

## Architecture

```
┌──────────────────────────┐
│  PCIe Gen3 x4 edge       │
└────────────┬─────────────┘
             │
             ▼
┌──────────────────────────┐
│  PCIE40 hard block       │  (KU5P built-in, "free")
│  + XDMA IP (Xilinx)      │  (~5K LUTs, 4–6 BRAMs)
└─────┬────────────────────┘
      │
      ├──── AXI4-MM master ────┐
      │     (high bandwidth)   │
      │                        ▼
      │              ┌─────────────────┐
      │              │ system AXI      │
      │              │ crossbar        │ ◄──── CPU dual-LSU
      │              └─────────────────┘
      │                        │
      │             ┌──────────┼──────────┐
      │             ▼          ▼          ▼
      │           DDR4        ROM        I/O
      │
      └──── AXI4-Lite master ──────────────► debug_ctrl.v
                                              ├─► ctl/status regs
                                              ├─► perf counters
                                              ├─► trace ring BRAMs
                                              └─► observation taps
                                                  back into the core
```

Current RTL status:
- `ENABLE_PCIE_XDMA=1` selects a real-hardware-only skeleton in
  `fpga_top.v`.
- The skeleton instantiates the repo-generated XDMA IP, exposes the PCIe
  x4 pins, and routes XDMA `M_AXI_BYPASS` through
  `axi_async_bridge` into xbar M1.
- `ENABLE_PCIE_XDMA=1` is mutually exclusive with `ENABLE_JTAG_AXI=1`
  because both are host masters for xbar M1.
- Descriptor DMA `M_AXI` is intentionally parked. Do not use the Linux
  H2C/C2H streaming devices as a production path until the DDR burst bridge
  and end-to-end burst tests land.

Host-visible map through the bypass BAR:

```
BAR bypass offset      SoC AXI target
0x0000_0000            RAM aperture, DDR-flattened by xbar S0
0x4000_0000            ROM aperture, DDR-flattened by xbar S0
0x5000_0000            peripheral/debug bus, xbar S1
0x5010_0000            DMA controller config, xbar S2
0x6000_0000            legacy framebuffer DDR alias, xbar S0
0xF900_0000            URAM VRAM pixel aperture, xbar S3
```

The current XDMA reference has BAR0 as a 128 KiB XDMA control BAR and a
4 MiB AXI bypass aperture with `pciebar2axibar_axist_bypass=0`.  The bypass
window is therefore suitable for low-address DDR and debug/peripheral smoke
access first.  High SoC addresses such as `0xF900_0000` require either a
larger bypass BAR or BAR-to-AXI translation retuning before Linux can reach
them directly.

Linux access pattern:
- Use the Xilinx XDMA driver from the host.
- For direct host debug reads/writes, prefer the bypass character device
  exposed by the driver for the AXI bypass BAR, then `pread`/`pwrite` or
  `mmap` offsets in the table above.
- Keep accesses aligned to 32-bit or 128-bit boundaries during bring-up.
  Single-beat accesses are the safe contract today.
- Treat `/dev/xdma*_h2c_*` and `/dev/xdma*_c2h_*` as future high-throughput
  DDR paths. Those exercise XDMA descriptor DMA `M_AXI`, which this skeleton
  does not connect.

Clock-domain contract:
- The PCIe reference clock is the AB7/AB6 100 MHz MGTREFCLK pair already
  named `fabric_clk_p/n` in `fpga_top`.
- XDMA generates `axi_aclk` at 250 MHz.
- The SoC xbar remains in `core_clk`; the skeleton crosses from XDMA
  `axi_aclk` into `core_clk` with `axi_async_bridge`.
- No same-clock assumption is made between PCIe and the SoC.

M_AXI vs M_AXI_BYPASS recommendation:
- Use `M_AXI_BYPASS` first. It gives direct host-initiated AXI-MM reads and
  writes with the smallest integration surface and no descriptor engine.
- Leave XDMA `M_AXI` for later bulk copies after the DDR bridge carries
  bursts correctly and a proper arbitration/CDC plan exists for concurrent
  bypass, descriptor DMA, JTAG, CPU, boot, and DMA-controller traffic.

Burst status relevant to host access:
- `axi_xbar` preserves `awlen/arlen` and can forward INCR bursts to S3 VRAM.
- `vram.v` claims INCR support, but a separate worker owns Verilator burst
  tests and RTL fixes for the repeated-first-beat failure mode.
- `axi_ddr4_mig_bridge` still clamps real-MIG `m_awlen/m_arlen` to zero, so
  DDR should be treated as single-beat from the host until that bridge is
  intentionally upgraded and covered.

---

## Build Flow

Generate and validate the repo-owned XDMA IP before any PCIe/XDMA top-level
build:

```bash
make pcie-xdma-validate
make pcie-xdma
```

The generated XCI, DCP, stub, and manifest live under `build/pcie_xdma/`.
`synth/vivado.tcl` consumes those artifacts by default when
`ENABLE_PCIE_XDMA=1`; it no longer reads `/offlinenas/share/FPGA/pcie_test`
as the normal load-bearing IP path.

Parse-only checks:

```bash
make pcie-xdma-dry-run
make pcie-xdma-pincheck
```

First full implementation path:

```bash
make ddr4-mig
make pcie-xdma
make USE_REAL_MIG=1 ENABLE_PCIE_XDMA=1 REAL_FPGA_BUILD=1 impl
```

The convenience target for the current 50 MHz real-MIG preset is:

```bash
make pcie-xdma-bitstream
```

Keep `ENABLE_JTAG_AXI=1` off in PCIe/XDMA builds unless host-master
arbitration is redesigned.

## debug_ctrl.v module spec

```verilog
module debug_ctrl (
    input  wire        clk,         // user clock (200 MHz)
    input  wire        rst,

    // ── AXI4-Lite slave (from XDMA BAR 1) ─────────────────────────────
    input  wire [19:0] s_axi_awaddr,
    input  wire        s_axi_awvalid,
    output wire        s_axi_awready,
    input  wire [31:0] s_axi_wdata,
    input  wire [ 3:0] s_axi_wstrb,
    input  wire        s_axi_wvalid,
    output wire        s_axi_wready,
    output wire [ 1:0] s_axi_bresp,
    output wire        s_axi_bvalid,
    input  wire        s_axi_bready,
    input  wire [19:0] s_axi_araddr,
    input  wire        s_axi_arvalid,
    output wire        s_axi_arready,
    output wire [31:0] s_axi_rdata,
    output wire [ 1:0] s_axi_rresp,
    output wire        s_axi_rvalid,
    input  wire        s_axi_rready,

    // ── Control outputs (drive into core) ─────────────────────────────
    output wire        dbg_halt_req,           // assert high to halt CPU
    output wire        dbg_step_req,           // pulse high for single-step
    output wire        dbg_soft_rst,           // pulse high for soft reset
    output wire        dbg_init_done_override, // override init_done=1
    output wire [31:0] dbg_redirect_pc,        // when valid, force PC
    output wire        dbg_redirect_valid,
    output wire [ 2:0] dbg_irq_inject_lvl,     // 0=none, 1-7=inject
    output wire        dbg_irq_inject_pulse,
    output wire        dbg_halt_after_enable,
    output wire [63:0] dbg_halt_after_inst,
    output wire        dbg_break_pc_enable,
    output wire [31:0] dbg_break_pc,
    output wire        dbg_halt_exc_enable,
    output wire [ 7:0] dbg_halt_exc_vec,

    // ── Auto-halt event input (from top-level precise breakpoint logic)
    input  wire        dbg_auto_halt_event,
    input  wire [ 2:0] dbg_auto_halt_reason,   // bit0=count, bit1=PC, bit2=exception
    input  wire [31:0] dbg_auto_halt_pc,
    input  wire [63:0] dbg_auto_halt_inst,

    // ── Status inputs (from core observability ports) ─────────────────
    input  wire        dbg_halted,
    input  wire        dbg_exc_pending,
    input  wire [ 7:0] dbg_exc_vec,
    input  wire [31:0] dbg_pc,
    input  wire [31:0] dbg_committed,
    input  wire [63:0] dbg_cycle_count,
    input  wire [63:0] dbg_inst_count,
    input  wire [31:0] dbg_mispred_count,
    input  wire [31:0] dbg_icache_hits,
    input  wire [31:0] dbg_icache_misses,
    input  wire [31:0] dbg_dcache_hits,
    input  wire [31:0] dbg_dcache_misses,
    // ... more (see register catalogue below)

    // ── Trace inputs (event-driven captures) ──────────────────────────
    input  wire        commit_event_valid,
    input  wire [31:0] commit_event_pc,
    input  wire [31:0] commit_event_data,
    input  wire [ 4:0] commit_event_arch_dst,
    input  wire        commit_event_has_dst,
    input  wire        mispred_event_valid,
    input  wire [31:0] mispred_event_pc,
    input  wire [31:0] mispred_event_predicted,
    input  wire [31:0] mispred_event_actual,
    // ... more

    // ── Snapshot read ports (read core internal state) ────────────────
    output wire [ 4:0] snap_rob_idx,        // host-driven snapshot index
    input  wire [127:0] snap_rob_entry,     // ROB entry contents
    output wire [ 4:0] snap_rat_arch,
    input  wire [ 5:0] snap_rat_phys,
    output wire [ 5:0] snap_prf_idx,
    input  wire [31:0] snap_prf_value
    // ... etc
);
```

Sizing:
- ~150–250 LUTs for the AXI-Lite shell + register file
- ~6 BRAMs for ring buffers (PC trace, commit log, mispredict trace,
  memory access trace, branch trace, IRQ trace)
- Negligible impact on core Fmax — all observability ports are reads
  with one register stage of isolation between core and debug_ctrl.

---

## Address map (BAR 1, 1 MB window)

```
0x00000–0x00FFF   System control & status (256 registers, 16 reserved)
0x01000–0x01FFF   Performance counters (1024 × 32-bit slots)
0x02000–0x02FFF   Architectural state snapshot (regs, SR, VBR, etc.)
0x03000–0x03FFF   ROB / RAT / IQ direct read window
0x04000–0x04FFF   Cache state snapshot (I + D + TLB)
0x05000–0x05FFF   Branch predictor state
0x06000–0x06FFF   Watchpoint / breakpoint registers
0x07000–0x07FFF   Peripheral debug taps (VIA, SCSI, SCC observation)

0x10000–0x13FFF   PC trace ring (4 KB = 1024 × 32-bit PCs)
0x14000–0x1BFFF   Commit log ring (8 KB = 256 × 32-byte records)
0x1C000–0x1DFFF   Mispredict trace ring (2 KB = 128 × 16-byte records)
0x1E000–0x1FFFF   Memory access trace ring (2 KB)
0x20000–0x21FFF   Exception trace ring (2 KB)
0x22000–0x23FFF   IRQ trace ring (2 KB)

0x80000–0xFFFFF   Reserved for future expansion
```

All registers are 32-bit aligned. Reads and writes are single-cycle from
the AXI-Lite shell; ring buffers are read by walking the address window.

---

## Register catalogue

Each entry: name | offset | R/W | bits | purpose. Tiers reflect implementation
priority — start with **critical**, add **important** as needed for phase 2/3,
keep **nice-to-have** in mind for phase 4+.

---

### TIER 1 — CRITICAL (without these, debug is painful)

System control & status (0x00000-block):

| Offset | Name | R/W | Purpose |
|---|---|---|---|
| 0x000 | DBG_VERSION | R | Magic + version word (0xDEB6_0004) |
| 0x004 | DBG_BUILD_ID | R | Git short SHA, packed |
| 0x008 | DBG_CONTROL | RW | bit0=halt_req, bit1=step_pulse, bit2=soft_rst, bit3=init_done_override |
| 0x00C | DBG_STATUS | R | bit0=halted, bit1=exc_pending, bit2=init_done_seen, bit3=cpu_running, bit4=auto_halt_latched |
| 0x010 | DBG_PC | R | Current committed PC (last retired) |
| 0x014 | DBG_LAST_PC | R | PC of most recent retire |
| 0x018 | DBG_REDIRECT_PC | W | Write to force CPU redirect |
| 0x01C | DBG_REDIRECT_TRIGGER | W | Pulse to commit the redirect |
| 0x020 | DBG_IRQ_INJECT | W | bits[2:0] = IRQ level (0=none, 1–7=fire) |
| 0x024 | DBG_EXC_VEC | R | Last exception vector taken |
| 0x028 | DBG_EXC_PC | R | PC at which last exception occurred |
| 0x02C | DBG_RESET_CAUSE | R | Last reset reason (0=power, 1=soft, 2=watchdog) |
| 0x030 | DBG_HALT_AFTER_LO | RW | Low 32 bits of retired-instruction count that triggers auto-halt |
| 0x034 | DBG_HALT_AFTER_HI | RW | High 32 bits of retired-instruction count that triggers auto-halt |
| 0x038 | DBG_BREAK_PC | RW | Retired PC value that triggers auto-halt when enabled |
| 0x03C | DBG_HALT_CTL | RW | bit0=halt_after_enable, bit1=break_pc_enable, bit2=clear_auto_halt W1P, bit3=auto_latched, bit4=halt_after_latched, bit5=break_pc_latched, bit6=halt_exc_enable, bit7=halt_exc_latched |
| 0x040 | DBG_HALT_REASON | R | bit0=manual_halt, bit1=halt_after_latched, bit2=break_pc_latched, bit3=effective_halt, bit4=halt_after_enable, bit5=break_pc_enable, bit6=halt_exc_latched, bit7=halt_exc_enable |
| 0x044 | DBG_HALT_HIT_PC | R | Retired PC captured when the latest auto-halt latched; cleared by HALT_CTL.clear_auto_halt |
| 0x048 | DBG_HALT_HIT_INST_LO | R | Low 32 bits of captured retired-instruction count; cleared by HALT_CTL.clear_auto_halt |
| 0x04C | DBG_HALT_HIT_INST_HI | R | High 32 bits of captured retired-instruction count; cleared by HALT_CTL.clear_auto_halt |
| 0x050 | DBG_HALT_EXC_VEC | RW | Exception vector that triggers auto-halt when halt_exc_enable is set; reset default is 4 (illegal instruction) |

Performance counters (0x01000-block, all 32-bit unless paired):

| Offset | Name | R/W | Purpose |
|---|---|---|---|
| 0x1000 | DBG_CYCLE_LO | R | Cycle counter low 32 bits |
| 0x1004 | DBG_CYCLE_HI | R | Cycle counter high 32 bits |
| 0x1008 | DBG_INST_LO | R | Retired instructions low 32 bits |
| 0x100C | DBG_INST_HI | R | Retired instructions high 32 bits |
| 0x1010 | DBG_MISPRED_COUNT | R | Branch mispredicts |
| 0x1014 | DBG_FLUSH_COUNT | R | Pipeline flushes (mispred + exception) |
| 0x1018 | DBG_EXC_COUNT | R | Exceptions taken |

Trace rings (TIER 1 essentials):

| Offset range | Name | Element size | Capacity | Purpose |
|---|---|---|---|---|
| 0x10000–0x13FFF | PC_TRACE | 4 B | 1024 | Last 1024 retired PCs |
| 0x11000 | PC_TRACE_HEAD | 4 B | — | Write index into PC_TRACE |

The PC trace ring write protocol is: `commit_event_valid` pulses each
retire, head advances modulo 1024. Host reads `PC_TRACE_HEAD` first, then
walks backward through the ring.

---

### TIER 2 — IMPORTANT (greatly improves debug productivity)

Architectural register shadow (0x02000-block). On debug_ctrl v4 these values
are writable only for halt-time architectural state injection. Host tooling
should halt first, write the desired shadow state, then write
`DBG_ARCH_APPLY.start` to flush/clear speculative state, reload the
architectural map, set PC, and resume.

| Offset | Name | R/W | Purpose |
|---|---|---|---|
| 0x2000 | DBG_D0 | RW | Data register 0 shadow |
| 0x2004 | DBG_D1 | RW | Data register 1 shadow |
| ...    | ... | ... | ... |
| 0x201C | DBG_D7 | RW | Data register 7 shadow |
| 0x2020 | DBG_A0 | RW | Address register 0 shadow |
| ...    | ... | ... | ... |
| 0x203C | DBG_A7 | RW | Active stack pointer shadow |
| 0x2040 | DBG_USP | RW | User stack pointer shadow |
| 0x2044 | DBG_SSP | RW | Supervisor stack pointer shadow |
| 0x2048 | DBG_ISP | RW | Interrupt stack pointer shadow |
| 0x204C | DBG_SR | RW | Status register shadow (T, S, M, I[2:0], CCR) |
| 0x2050 | DBG_VBR | RW | Vector base register shadow |
| 0x2054 | DBG_CACR | RW | Cache control register shadow |
| 0x2058 | DBG_TC | RW | Translation control shadow |
| 0x205C | DBG_ITT0 | RW | Instruction transparent translation 0 shadow |
| 0x2060 | DBG_ITT1 | RW | Instruction transparent translation 1 shadow |
| 0x2064 | DBG_DTT0 | RW | Data transparent translation 0 shadow |
| 0x2068 | DBG_DTT1 | RW | Data transparent translation 1 shadow |
| 0x206C | DBG_URP | RW | User root pointer shadow |
| 0x2070 | DBG_SRP | RW | Supervisor root pointer shadow |
| 0x2074 | DBG_ARCH_PC | RW | Restart PC shadow |
| 0x2078 | DBG_ARCH_APPLY | RW | bit0=start apply/resume, bit1=clear done/rejected status |
| 0x207C | DBG_ARCH_STATUS | R | bit0=busy, bit1=done, bit2=rejected-not-halted |
| 0x2080 | DBG_SFC | RW | Source function code shadow |
| 0x2084 | DBG_DFC | RW | Destination function code shadow |

Live arch readback (0x02100 block, read-only).  Bypasses the host
shadow apply path: each read walks the snap chain through the core's
committed RAT and PRF, returning the actual CPU register state.
Stable only when the CPU is halted; racy under free-run.

| Offset | Name | R | Purpose |
|---|---|---|---|
| 0x2100 | DBG_LIVE_VBR | R | Live VBR (from commit.arch_vbr) |
| 0x2104 | DBG_LIVE_SR | R | Live SR (from commit.arch_sr; bits[15:0]) |
| 0x2108 | DBG_LIVE_A7 | R | Live A7 (from commit.arch_a7) |
| 0x2110 | DBG_LIVE_D0 | R | Live D0 = prf[crat[0]] |
| 0x2114..0x212C | DBG_LIVE_D1..D7 | R | +0x4 per reg |
| 0x2130 | DBG_LIVE_A0 | R | Live A0 = prf[crat[8]] |
| 0x2134..0x214C | DBG_LIVE_A1..A7 | R | +0x4 per reg |

Halt-on-exception bitmask (0x00060 block, RW).  Replaces the legacy
single-vec match.  Bit `i*32+j` of lane `i` halts when the next
exception event has `vec == i*32+j`.  Default 0 (no halt) — host
must opt in.  Master gate `dbg_halt_exc_enable` (HALT_CTL bit 6)
remains required.

| Offset | Name | RW | Purpose |
|---|---|---|---|
| 0x0060 | DBG_HALT_EXC_MASK_0 | RW | Vectors  0..31  (bit j = vec j) |
| 0x0064 | DBG_HALT_EXC_MASK_1 | RW | Vectors 32..63 |
| 0x0068 | DBG_HALT_EXC_MASK_2 | RW | Vectors 64..95 |
| 0x006C | DBG_HALT_EXC_MASK_3 | RW | Vectors 96..127 |
| 0x0070 | DBG_HALT_EXC_MASK_4 | RW | Vectors 128..159 |
| 0x0074 | DBG_HALT_EXC_MASK_5 | RW | Vectors 160..191 |
| 0x0078 | DBG_HALT_EXC_MASK_6 | RW | Vectors 192..223 |
| 0x007C | DBG_HALT_EXC_MASK_7 | RW | Vectors 224..255 |

The legacy `DBG_HALT_EXC_VEC` (0x050) register is kept for register-
read backward compatibility but does NOT drive the halt; only the
mask matters.

Stall reason counters (0x01100-block):

| Offset | Name | R/W | Purpose |
|---|---|---|---|
| 0x1100 | DBG_RENAME_STALL | R | Cycles rename couldn't dispatch |
| 0x1104 | DBG_ROB_FULL_STALL | R | Cycles ROB was full at dispatch |
| 0x1108 | DBG_IQ_INT_FULL | R | Cycles iq_int was full |
| 0x110C | DBG_IQ_MEM_FULL | R | Cycles iq_mem was full |
| 0x1110 | DBG_IQ_FP_FULL | R | Cycles iq_fp was full |
| 0x1114 | DBG_FREELIST_EMPTY | R | Cycles free-list empty (rename starvation) |
| 0x1118 | DBG_CCR_STALL | R | Cycles uops blocked on CCR-rd |
| 0x111C | DBG_LSU_BUSY_STALL | R | Cycles LSU couldn't accept new memop |

Cache performance (0x01200-block):

| Offset | Name | R/W | Purpose |
|---|---|---|---|
| 0x1200 | DBG_ICACHE_HITS | R | I-cache hits |
| 0x1204 | DBG_ICACHE_MISSES | R | I-cache misses |
| 0x1208 | DBG_DCACHE_HITS | R | D-cache hits |
| 0x120C | DBG_DCACHE_MISSES | R | D-cache misses |
| 0x1210 | DBG_DCACHE_WB | R | D-cache writebacks |
| 0x1214 | DBG_TLB_HITS | R | TLB / ATC hits |
| 0x1218 | DBG_TLB_MISSES | R | TLB / ATC misses |
| 0x121C | DBG_TLB_WALKS | R | Page table walks performed |

Branch prediction stats (0x01300-block):

| Offset | Name | R/W | Purpose |
|---|---|---|---|
| 0x1300 | DBG_BTB_LOOKUPS | R | Total BTB queries |
| 0x1304 | DBG_BTB_HITS | R | BTB hits (matched tag) |
| 0x1308 | DBG_BTB_PRED_TAKEN | R | BTB predicted taken |
| 0x130C | DBG_BR_TAKEN | R | Resolved taken |
| 0x1310 | DBG_BR_NOT_TAKEN | R | Resolved not-taken |
| 0x1314 | DBG_DIR_MISPRED | R | Direction mispredicts |
| 0x1318 | DBG_TGT_MISPRED | R | Target mispredicts (correct dir, wrong tgt) |

LSU & memory stats (0x01400-block):

| Offset | Name | R/W | Purpose |
|---|---|---|---|
| 0x1400 | DBG_LD_ISSUED | R | Loads issued |
| 0x1404 | DBG_ST_ISSUED | R | Stores issued |
| 0x1408 | DBG_LD_FORWARDED | R | Loads forwarded from store buffer |
| 0x140C | DBG_LD_OUTSTANDING | R | Current loads in flight |
| 0x1410 | DBG_ST_BUFFERED | R | Current stores in store buffer |
| 0x1414 | DBG_AXI_RD_TXN | R | AXI read transactions completed |
| 0x1418 | DBG_AXI_WR_TXN | R | AXI write transactions completed |
| 0x141C | DBG_AXI_BERR | R | AXI BRESP errors |
| 0x1420 | DBG_AXI_RERR | R | AXI RRESP errors |

ROB direct read (0x03000-block):

| Offset | Name | R/W | Purpose |
|---|---|---|---|
| 0x3000 | DBG_ROB_HEAD | R | Current head index |
| 0x3004 | DBG_ROB_TAIL | R | Current tail index |
| 0x3008 | DBG_ROB_COUNT | R | Entries currently in flight |
| 0x300C | DBG_ROB_SNAP_IDX | RW | Index to snapshot (host writes 0..31) |
| 0x3010 | DBG_ROB_SNAP_W0 | R | Snapped entry word 0 (valid, complete, type, has_dst, etc.) |
| 0x3014 | DBG_ROB_SNAP_W1 | R | Snapped entry word 1 (phys_dst, phys_old, flags_wr, flags_val) |
| 0x3018 | DBG_ROB_SNAP_W2 | R | Snapped entry word 2 (PC) |
| 0x301C | DBG_ROB_SNAP_W3 | R | Snapped entry word 3 (npc_pred or br_target) |

Trace rings (TIER 2 additions):

| Offset range | Name | Element | Cap | Purpose |
|---|---|---|---|---|
| 0x14000–0x1BFFF | COMMIT_LOG | 32 B | 256 | {pc, dst, value, flags, cycle} per retire |
| 0x14F00 | COMMIT_LOG_HEAD | 4 B | — | Write index |
| 0x1C000–0x1DFFF | MISPRED_TRACE | 16 B | 128 | {pc, predicted_tgt, actual_tgt, cycle} |
| 0x1CF00 | MISPRED_TRACE_HEAD | 4 B | — | Write index |
| 0x1E000–0x1FFFF | MEM_TRACE | 16 B | 128 | {pc, addr, data, rd_wr_size, cycle} |
| 0x1EF00 | MEM_TRACE_HEAD | 4 B | — | Write index |
| 0x20000–0x21FFF | EXC_TRACE | 16 B | 128 | {pc, vec, sr, cycle} on exception entry |
| 0x20F00 | EXC_TRACE_HEAD | 4 B | — | Write index |
| 0x22000–0x23FFF | IRQ_TRACE | 16 B | 128 | {level, vec, sr_at_entry, cycle} |
| 0x22F00 | IRQ_TRACE_HEAD | 4 B | — | Write index |

Watchpoints / breakpoints (0x06000-block, 4 slots):

| Offset | Name | R/W | Purpose |
|---|---|---|---|
| 0x6000 | DBG_WP0_PC | RW | Halt when PC == this value (0 = disabled) |
| 0x6004 | DBG_WP0_PC_MASK | RW | AND mask for PC compare (0 = exact match) |
| 0x6008 | DBG_WP0_ADDR | RW | Halt on memory access to this address |
| 0x600C | DBG_WP0_TYPE | RW | bits: 0=load, 1=store, 2=fetch, 3=enable |
| 0x6010..0x603F | WP1..WP3 | RW | Same layout, 3 more watchpoints |

---

### TIER 3 — NICE TO HAVE (phase 4+ or specific debug scenarios)

Per-instruction-class counters (0x01500-block):

| Offset | Name | R/W | Purpose |
|---|---|---|---|
| 0x1500 | DBG_CNT_INT | R | UOP_INT retired |
| 0x1504 | DBG_CNT_LOAD | R | UOP_LOAD retired |
| 0x1508 | DBG_CNT_STORE | R | UOP_STORE retired |
| 0x150C | DBG_CNT_BRANCH | R | UOP_BRANCH retired |
| 0x1510 | DBG_CNT_SYS | R | UOP_SYS retired |
| 0x1514 | DBG_CNT_FP | R | UOP_FP retired |
| 0x1518 | DBG_CNT_MUL | R | Multiplies executed |
| 0x151C | DBG_CNT_DIV | R | Divides executed |
| 0x1520 | DBG_CNT_AL_TRAP | R | A-line traps (Mac OS Toolbox calls) |
| 0x1524 | DBG_CNT_F_TRAP | R | F-line traps (FPU) |
| 0x1528 | DBG_CNT_TRAP_N | R | TRAP #n executed |
| 0x152C | DBG_CNT_RTE | R | RTE executed |
| 0x1530 | DBG_CNT_BSR | R | BSR executed |
| 0x1534 | DBG_CNT_RTS | R | RTS executed |
| 0x1538 | DBG_CNT_MOVEM | R | MOVEM executed |
| 0x153C | DBG_CNT_CPUSH | R | CPUSH executed |
| 0x1540 | DBG_CNT_CINV | R | CINV executed |
| 0x1544 | DBG_CNT_PFLUSH | R | PFLUSH (TLB invalidate) executed |

RAT direct read (0x03100-block):

| Offset | Name | R/W | Purpose |
|---|---|---|---|
| 0x3100 | DBG_RAT_SNAP_IDX | RW | Arch reg index 0..15 |
| 0x3104 | DBG_RAT_SNAP_PHYS | R | Current phys mapping |
| 0x3108 | DBG_CRAT_SNAP_IDX | RW | Committed RAT index |
| 0x310C | DBG_CRAT_SNAP_PHYS | R | Committed phys mapping |
| 0x3110 | DBG_FREELIST_HEAD | R | Free-list head pointer |
| 0x3114 | DBG_FREELIST_TAIL | R | Free-list tail pointer |
| 0x3118 | DBG_FREELIST_COUNT | R | Free regs available |
| 0x311C | DBG_PRF_SNAP_IDX | RW | Phys reg index 0..47 |
| 0x3120 | DBG_PRF_SNAP_VAL | R | Phys reg value |
| 0x3124 | DBG_PRF_SNAP_BUSY | R | Phys reg busy bit |

Issue queue direct read (0x03200-block):

| Offset | Name | R/W | Purpose |
|---|---|---|---|
| 0x3200 | DBG_IQINT_COUNT | R | Entries in iq_int |
| 0x3204 | DBG_IQINT_SNAP_IDX | RW | Slot 0..7 |
| 0x3208 | DBG_IQINT_SNAP_W0 | R | Slot contents word 0 |
| 0x320C | DBG_IQINT_SNAP_W1 | R | Slot contents word 1 |
| 0x3210 | DBG_IQMEM_COUNT | R | Entries in iq_mem |
| 0x3214 | DBG_IQMEM_SNAP_IDX | RW | Slot 0..7 |
| 0x3218 | DBG_IQMEM_SNAP_W0 | R | Slot contents word 0 |
| 0x321C | DBG_IQMEM_SNAP_W1 | R | Slot contents word 1 |
| 0x3220 | DBG_IQFP_COUNT | R | Entries in iq_fp |

Cache state dump (0x04000-block):

| Offset | Name | R/W | Purpose |
|---|---|---|---|
| 0x4000 | DBG_ICACHE_SET_IDX | RW | Set index (0..N-1) |
| 0x4004 | DBG_ICACHE_WAY_IDX | RW | Way index (0..3) |
| 0x4008 | DBG_ICACHE_TAG | R | Tag at selected (set, way) |
| 0x400C | DBG_ICACHE_VALID | R | Valid bit |
| 0x4010 | DBG_ICACHE_LRU | R | PLRU bits for the set |
| 0x4014..0x4053 | DBG_ICACHE_DATA[0..15] | R | 32 bytes of cache line data |
| 0x4100 | DBG_DCACHE_SET_IDX | RW | (same shape as ICACHE) |
| ... | ... | ... | ... |
| 0x4200 | DBG_TLB_SNAP_IDX | RW | TLB entry 0..63 |
| 0x4204 | DBG_TLB_SNAP_VPN | R | Virtual page number |
| 0x4208 | DBG_TLB_SNAP_PPN | R | Physical page number |
| 0x420C | DBG_TLB_SNAP_FLAGS | R | Valid/dirty/W/U/S bits |

Branch predictor state (0x05000-block):

| Offset | Name | R/W | Purpose |
|---|---|---|---|
| 0x5000 | DBG_BPU_SNAP_IDX | RW | BTB entry 0..N-1 |
| 0x5004 | DBG_BPU_TAG | R | Tag stored at index |
| 0x5008 | DBG_BPU_TARGET | R | Predicted target |
| 0x500C | DBG_BPU_COUNTER | R | 2-bit bimodal state |
| 0x5010 | DBG_BPU_VALID | R | Valid bit |
| 0x5100 | DBG_GHR | R | Global history register (when gshare lands) |
| 0x5200 | DBG_RAS_HEAD | R | RAS top-of-stack index |
| 0x5204 | DBG_RAS_SNAP_IDX | RW | RAS slot 0..7 |
| 0x5208 | DBG_RAS_SNAP_TGT | R | Slot contents |

Peripheral debug taps (0x07000-block):

| Offset | Name | R/W | Purpose |
|---|---|---|---|
| 0x7000 | DBG_VIA1_REGS[0..15] | R | VIA1 register file mirror |
| 0x7100 | DBG_VIA2_REGS[0..15] | R | VIA2 register file mirror |
| 0x7200 | DBG_SCSI_REGS[0..7] | R | NCR 5380 register file mirror |
| 0x7204 | DBG_SCSI_PHASE | R | Current SCSI phase |
| 0x7208 | DBG_SCSI_SD_SECTOR | R | Last SD sector accessed |
| 0x7300 | DBG_SCC_REGS[0..15] | R | SCC register mirror |
| 0x7400 | DBG_GLUE_LAST_ADDR | R | Last AXI address to peripheral bus |
| 0x7404 | DBG_GLUE_LAST_DATA | R | Last data |
| 0x7408 | DBG_GLUE_LAST_RW | R | Last R/W direction |

Performance event capture (0x01600-block):

| Offset | Name | R/W | Purpose |
|---|---|---|---|
| 0x1600 | DBG_PERF_GATE_PC_LO | RW | Start counters when PC enters [LO, HI] |
| 0x1604 | DBG_PERF_GATE_PC_HI | RW | |
| 0x1608 | DBG_PERF_GATE_ENABLE | RW | bit0=enable |
| 0x160C | DBG_PERF_GATE_CYCLES | R | Cycles counted while gated |
| 0x1610 | DBG_PERF_GATE_INSTS | R | Instructions retired while gated |

Useful for measuring "how many cycles does Mac OS spend in QuickDraw"
without instrumenting the OS.

Memory pattern check (0x06100-block):

| Offset | Name | R/W | Purpose |
|---|---|---|---|
| 0x6100 | DBG_MEMCHECK_BASE | RW | Start address of pattern check |
| 0x6104 | DBG_MEMCHECK_SIZE | RW | Bytes to check |
| 0x6108 | DBG_MEMCHECK_PATTERN | RW | 32-bit pattern (0xAA, 0x55, walking 1, etc.) |
| 0x610C | DBG_MEMCHECK_CTRL | RW | bit0=trigger, bit1=mode (write-then-read or read-only) |
| 0x6110 | DBG_MEMCHECK_STATUS | R | bit0=done, bit1=mismatch |
| 0x6114 | DBG_MEMCHECK_FAULT_ADDR | R | First mismatch address |
| 0x6118 | DBG_MEMCHECK_FAULT_EXP | R | Expected value |
| 0x611C | DBG_MEMCHECK_FAULT_GOT | R | Actual value |

System health (0x00100-block):

| Offset | Name | R/W | Purpose |
|---|---|---|---|
| 0x100 | DBG_TEMP | R | Die temperature (from XADC, °C × 256) |
| 0x104 | DBG_VCCINT | R | Internal voltage (mV) |
| 0x108 | DBG_VCCAUX | R | Aux voltage |
| 0x10C | DBG_USR_CLK_HZ | R | Measured user clock frequency |
| 0x110 | DBG_DDR_CLK_HZ | R | DDR4 clock frequency |
| 0x114 | DBG_PCIE_LINK_STATUS | R | PCIe link width & speed |

---

## Trace ring formats

### PC_TRACE (4 bytes/entry × 1024 = 4 KB)

```
+0: PC[31:0]
```

Simplest possible — just the retired PC. Combined with `DBG_CYCLE` and the
host's wall-clock, you can correlate phases of execution.

### COMMIT_LOG (32 bytes/entry × 256 = 8 KB)

```
+0:  PC[31:0]
+4:  cycle[31:0]      (low 32 bits of cycle counter at retire)
+8:  arch_dst[4:0] | has_dst | flags_wr[4:0] | flags_val[4:0] | uop_type[3:0]
+12: result_value[31:0]      (PRF read at commit)
+16: prev_phys_dst[5:0] | new_phys_dst[5:0]
+20: src_a_value[31:0]       (optional — adds rd port to PRF)
+24: src_b_value[31:0]       (optional)
+28: reserved
```

Reconstruct full instruction execution from this. ~8KB lets you see the
last ~8000 instructions in fine detail.

### MISPRED_TRACE (16 bytes/entry × 128 = 2 KB)

```
+0:  pc[31:0]                 (mispredicted branch PC)
+4:  predicted_target[31:0]
+8:  actual_target[31:0]
+12: cycle[31:0]
```

The "post-mortem on every mispredict" log. Combined with `DBG_FLUSH_COUNT`
and the BPU snapshot, identifies branches the predictor consistently fails on.

### EXC_TRACE (16 bytes/entry × 128 = 2 KB)

```
+0:  pc[31:0]                 (PC of faulting instruction)
+4:  vec[7:0] | sr[15:0] | reserved[7:0]
+8:  cycle[31:0]
+12: stack_pointer[31:0]      (A7 at entry)
```

For Mac OS this is mostly A-line traps. Histogram by `vec` to see what
Toolbox calls dominate.

### IRQ_TRACE (16 bytes/entry × 128 = 2 KB)

```
+0:  pc[31:0]                 (PC where IRQ was taken)
+4:  level[2:0] | vec[7:0] | reserved
+8:  cycle[31:0]
+12: cycles_in_handler        (filled when RTE retires for this handler)
```

### MEM_TRACE (16 bytes/entry × 128 = 2 KB) — gated, optional

```
+0:  pc[31:0]                 (PC of load/store)
+4:  addr[31:0]
+8:  data[31:0]
+12: cycle[31:0] | size[1:0] | rd_wr | reserved
```

Off by default (would fill in microseconds). Gated by a PC range or
address range setting in the watchpoint registers.

---

## Implementation phasing

### Phase A — bare-bones (1 session, do before phase 2 of gameplan)

- Generate the repo-owned XDMA IP with `make pcie-xdma`
- Stand up `pcie-bringup/` standalone project: PCIe → 4KB BRAM, host loopback
- Verify host can read/write the BRAM via the Xilinx XDMA driver
- Confirm no Fmax impact on the user-clock domain

### Phase B — TIER 1 registers (1 session)

- Implement `debug_ctrl.v` with:
  - System control / status registers
  - PC, cycle, inst, mispred, exception counters
  - PC trace ring (4 KB)
- Plumb the observability ports through `m68k_core.v`
- Python helper `tools/fpga_debug.py` — open XDMA device, read/write API

### Phase C — TIER 2 registers (1–2 sessions)

- Architectural register snapshot (D0–D7, A0–A7, USP/SSP/ISP, SR, VBR, MMU regs)
- Stall-reason counters
- Cache & branch performance counters
- ROB direct read window
- Watchpoint registers
- Commit log + mispredict trace + exception trace rings

### Phase D — TIER 3 registers (when needed)

- Per-instruction-class counters
- RAT / IQ / cache / TLB direct read
- Peripheral debug taps
- Performance gating
- Memory pattern checker

Phase A+B is enough to make phase-2 bring-up dramatically less painful.
Phase C+D is what you add when phase-3 surprises start happening.

---

## Python helper sketch

```python
# tools/fpga_debug.py
import os, mmap, struct

class Fpga:
    BAR1_SIZE = 1 << 20

    def __init__(self, dev='/dev/xdma0_user'):
        self.fd = os.open(dev, os.O_RDWR | os.O_SYNC)
        self.bar1 = mmap.mmap(self.fd, self.BAR1_SIZE)
        assert self.read32(0x000) & 0xFFFF0000 == 0xDEB60000, "bad version magic"

    def read32(self, off):
        return struct.unpack_from('<I', self.bar1, off)[0]

    def write32(self, off, val):
        struct.pack_into('<I', self.bar1, off, val)

    # ── Convenience ────────────────────────────────────────────────────
    @property
    def pc(self):       return self.read32(0x010)
    @property
    def halted(self):   return bool(self.read32(0x00C) & 1)
    @property
    def cycles(self):   return (self.read32(0x1004) << 32) | self.read32(0x1000)
    @property
    def insts(self):    return (self.read32(0x100C) << 32) | self.read32(0x1008)
    @property
    def ipc(self):      return self.insts / max(self.cycles, 1)

    def halt(self):     self.write32(0x008, 1)
    def resume(self):   self.write32(0x008, 0)
    def step(self):     self.write32(0x008, 0b10)  # pulse step bit
    def soft_rst(self): self.write32(0x008, 0b100)
    def init_done(self, v=True):
        self.write32(0x008, 0b1000 if v else 0)

    def redirect(self, pc):
        self.write32(0x018, pc)
        self.write32(0x01C, 1)

    def inject_irq(self, level):
        assert 1 <= level <= 7
        self.write32(0x020, level)

    def pc_trace(self):
        head = self.read32(0x11000)
        raw = bytes(self.bar1[0x10000:0x10000 + 1024 * 4])
        # Reorder so most-recent is last
        return struct.unpack('<1024I', raw[head*4:] + raw[:head*4])

    def commit_log(self):
        head = self.read32(0x14F00)
        raw = bytes(self.bar1[0x14000:0x14000 + 256 * 32])
        records = []
        for i in range(256):
            offset = ((head + i) % 256) * 32
            pc, cycle, packed, value = struct.unpack_from('<IIII', raw, offset)
            records.append((pc, cycle, packed, value))
        return records

    def regs(self):
        return {
            f'D{i}': self.read32(0x2000 + i*4) for i in range(8)
        } | {
            f'A{i}': self.read32(0x2020 + i*4) for i in range(8)
        } | {
            'SR':  self.read32(0x204C),
            'VBR': self.read32(0x2050),
            'PC':  self.pc,
        }

    def dma_load(self, addr, data):
        # Use XDMA's DMA channel for bulk transfer to BAR 0 (system AXI)
        with open('/dev/xdma0_h2c_0', 'wb') as f:
            f.seek(addr)
            f.write(data)

    def dma_dump(self, addr, length):
        with open('/dev/xdma0_c2h_0', 'rb') as f:
            f.seek(addr)
            return f.read(length)


# Typical bring-up session
if __name__ == '__main__':
    f = Fpga()
    print(f"FPGA build {f.read32(0x004):08x}")

    # Load ROM image directly to DRAM (50 ms vs ~2 s SD boot)
    rom = open('/data/quadra_rom.bin', 'rb').read()
    f.dma_load(0x40000000, rom)

    # Release the CPU
    f.init_done(True)

    # Watch
    while not f.halted:
        if f.read32(0x00C) & 0b10:  # exc_pending
            print(f"exception {f.read32(0x024)} at {f.read32(0x028):08x}")
            break
    print(f"final state: PC={f.pc:08x}, IPC={f.ipc:.2f}")
    print("regs:", f.regs())
    print("last 16 PCs:", [hex(p) for p in f.pc_trace()[-16:]])
```

---

## Reference: prince integration pattern

The user has a working PCIe + XDMA project at `~/prince/fpga/` (KU5P, same
board). The wiring pattern is verified for Vivado 2023.1; key files:

- `~/prince/pcie.xdc` — pin assignments for **both** PCIe and DDR4 in one
  file. Reuse directly; just add HDMI pins from `hdmi-bringup/synth/`
  and SD pins on top.
- `~/prince/fpga/rtl/prince_top.sv` — IBUFDS_GTE4 reference clock buffering
  + xdma_0 instantiation. Copy the IBUFDS_GTE4 block verbatim. The xdma_0
  port wiring is exact for Vivado 2023.1 / xdma_v4_1.
- `~/prince/fpga/vivado/create_project.tcl` — XDMA IP `set_property -dict`
  block. Reuse with these modifications:
  - Enable AXI-MM master + DMA channels (prince has them disabled):
    ```tcl
    CONFIG.c_h2c_num_chnl   {1}     # was 0
    CONFIG.c_c2h_num_chnl   {1}     # was 0
    ```
  - Bump BAR2 size from 64 KB to 1 MB to fit the full debug register map.
  - Change `STEPS.SYNTH_DESIGN.ARGS.FLATTEN_HIERARCHY` from `none`
    (prince's setting, optimised for parallel-pipeline designs) to
    `rebuilt` (default — better Fmax for tightly-coupled designs like
    our CPU pipeline).
- `~/prince/fpga/vivado/build.tcl` — reuse verbatim. `Performance_Explore`
  -`WithRemap` impl strategy + `Flow_PerfOptimized_high` synth strategy are
  already a good fit for our Fmax push.
- `~/prince/fpga/scripts/program_jtag.tcl` — reuse verbatim, pointed at
  our bitstream path.

PCIe link from prince: **Gen2 x4** (5.0 GT/s × 4 lanes ≈ 2 GB/s effective).
Sufficient for debug and DMA — the full 4 MB ROM transfers in 10–20 ms
with driver overhead.

JTAG cable (FT2232H) needs `ftdi_sio` unbound on the host side before
`hw_server` can claim it via libusb — see prince's `program_jtag.tcl`
header for the one-time setup commands.

## Open questions for the user

1. ~~**Does the KU5P board have a PCIe edge connector?**~~ Confirmed yes,
   Gen2 x4 GTY (lanes on AF7/AE9/AD7/AC5; 100 MHz refclk on AB7).
2. ~~**Linux on the host?**~~ Confirmed yes.
3. ~~**Want a `pcie-bringup/` standalone project?**~~ No — the prince
   project already validated the PCIe + XDMA wiring pattern on this
   exact board. We can integrate directly into `mac_top.v` when phase 2
   begins, copying the prince integration pattern.

---

## Host-side tooling

The Python package `tools/m68kctl/` wraps the PCIe/XDMA interface with a
library + CLI for day-to-day bring-up work.  All behaviour is testable
without an FPGA via a built-in mock backend (`--mock`).

### Overview

| Module                | Role                                                   |
|-----------------------|--------------------------------------------------------|
| `m68kctl.device`      | `XdmaDevice` (real) / `MockDevice` (test)              |
| `m68kctl.regs`        | Register offsets + bit masks                           |
| `m68kctl.bus`         | `SystemBus` over BAR 0 DMA (RAM/ROM/FB/I-O dispatch)   |
| `m68kctl.cpu`         | `CpuDebug` over BAR 1 debug_ctrl                       |
| `m68kctl.sd`          | (DEAD — sd_provision REMOVED; kept until pruned)       |
| `m68kctl.provision`   | Upload/verify/dump/load high-level flows               |
| `m68kctl.cli`         | argparse CLI — `python -m m68kctl`                     |

### CLI summary

```
m68kctl info                                  — identity + link status
m68kctl cpu {halt,resume,step,soft-rst,reset-halt,redirect,regs,trace}
m68kctl sd  {read,write,upload,verify}
m68kctl bus {read,write,dump,load}
m68kctl --mock <any>                          — run against MockDevice
```

See `tools/m68kctl/README.md` for full argument help and the library API.

### BAR layout assumed by the tooling

* **BAR 0** — XDMA AXI-MM master onto the CPU system bus; DMA via
  `/dev/xdma0_h2c_0` and `/dev/xdma0_c2h_0`.
* **BAR 1**, 1 MB, via `/dev/xdma0_user`:
    * `0x00000 – 0x7FFFF` — `debug_ctrl` (all tiers)
    * `0x80000 – 0x8FFFF` — REMOVED (was `sd_provision`; now AXI DECERR.
      Boot is via JTAG-AXI now and SD provisioning happens host-side,
      before power-up).

### Mock-mode workflow

The `MockDevice` is a dict-backed memory model with a tiny SD-card
simulator.  It lets the CLI, library, and unit tests run end-to-end
without any FPGA — handy for CI and developer-laptop iteration on the
tooling itself.

```
$ m68kctl --mock info
DBG_VERSION  : 0xdeb60001
DBG_BUILD_ID : 0x600df00d
SDP_VERSION  : 0x5d500001
STATUS       : 0x0000000c  (halted=False running=True init_done=True)
SD card_ready: True
PC           : 0x00000000
...
(mock backend — no real FPGA accessed)
```

MockDevice state is per-process — a CLI `sd upload` followed by a
separate `sd verify` invocation do not share a backing store.  For
end-to-end round-trips, use the library directly (`tb/tests/host/` does
exactly this) or a single driver script that keeps the mock alive.

### Worked examples

**1. Upload a 4 MB ROM image to the SD card on a real FPGA:**

```bash
sudo modprobe xdma                     # ensure driver is loaded
m68kctl info                           # confirms bitstream is present
m68kctl sd upload /data/quadra_rom.bin
m68kctl sd verify /data/quadra_rom.bin
```

**2. Dump 4 MB of DDR4 ROM region to a file for offline inspection:**

```bash
m68kctl bus dump 0x40000000 0x400000 -o /tmp/rom_shadow.bin
```

This uses `/dev/xdma0_c2h_0` for DMA so it completes in ~10–20 ms over
Gen2 x4.

For architectural checkpoint capture, use the bundle form so RAM and ROM
can be streamed to separate files in one pass:

```bash
m68kctl checkpoint dump --output-dir /tmp/checkpoint \
  --region ram 0x00001000 0x2000 \
  --region rom 0x40002000 0x1000
```

This path keeps the capture chunked on the host side, which avoids building
the whole dump in memory before the file write.  The smoke target
`make pcie-checkpoint-dump-check` validates the same flow against the mock
backend.

**3. Inspect CPU during boot:**

```bash
m68kctl cpu halt
m68kctl cpu regs                       # TIER 2 architectural snapshot
m68kctl cpu trace --tail 64            # last 64 retired PCs
m68kctl cpu reset-halt                 # pulse CPU reset and hold core halted
m68kctl cpu halt-after 1000000         # latch halt when retired-inst count reaches N
m68kctl cpu break-pc 0x40801234        # latch halt after this PC retires
m68kctl cpu halt-exc                   # latch halt on illegal-instruction vector 4
m68kctl cpu halt-status
m68kctl cpu clear-halt                 # clear latched auto-halt, preserving enables
m68kctl cpu step                       # single-step one retire
m68kctl cpu regs                       # observe updated state
m68kctl cpu resume
```

`step` is defined only while halted. It temporarily opens the core halt gate,
then reasserts halt and issues a precise-stop flush on the next retired
instruction boundary. Exception-entry and RTE boundaries do not complete a
step and do not increment the halt-after retired-instruction counter.

When the current bitstream only wires TIER 1 registers, `cpu regs`
prints zeros and logs a one-shot warning naming the observed tier —
the tooling does not pretend values it can't see.

### Unit tests

Run against the mock backend only:

```bash
make tb-host            # python -m unittest discover tb/tests/host
```

Host tests cover the full matrix (CPU halt/resume/step, programmable
halt/breakpoint controls, PC-trace ring wraparound, SD round-trip +
corruption detection, bus region routing, checkpoint bundle capture, and
CLI help/execution for every leaf subcommand).
