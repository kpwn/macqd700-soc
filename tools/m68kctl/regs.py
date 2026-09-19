"""regs.py — register-offset and bit-mask constants for m68kctl.

Centralised so both the live FPGA backend and the MockDevice can speak the
same protocol.  Values track ``docs/debug_pcie.md``.

BAR layout:

* BAR 0  — XDMA AXI-MM master on the system bus (RAM/ROM/FB/I-O).  Accessed
           via ``/dev/xdma0_h2c_0`` / ``/dev/xdma0_c2h_0`` for bulk DMA.
* BAR 1  — AXI-Lite peripheral window, 1 MB.
             0x00000 – 0x7FFFF : debug_ctrl (all tiers)
             0x80000 – 0x8FFFF : (REMOVED — was sd_provision; now AXI DECERR.
                                  Boot is via JTAG-AXI now and SD provisioning
                                  happens host-side, before power-up.)

The host-side ``tools/m68kctl/sd.py`` SDP_* register usage is dead code on
top of these placeholder constants; it will be pruned in a follow-up.
"""

from __future__ import annotations


# ═════════════════════════════════════════════════════════════════════════
# BAR layout
# ═════════════════════════════════════════════════════════════════════════
BAR1_SIZE       = 1 << 20          # 1 MB debug_ctrl
DEBUG_BASE      = 0x00000


# ═════════════════════════════════════════════════════════════════════════
# System-bus (BAR 0) address regions — from docs/peripheral_arch.md
# ═════════════════════════════════════════════════════════════════════════
SYS_RAM_BASE    = 0x0000_0000
SYS_RAM_END     = 0x1000_0000      # up to 256 MB reserved, 128 MB real
SYS_ROM_BASE    = 0x4000_0000
SYS_ROM_END     = 0x4100_0000      # 16 MB window, 4 MB real
SYS_IO_BASE     = 0x5000_0000
SYS_IO_END     = 0x5100_0000
SYS_FB_BASE     = 0x6000_0000
SYS_FB_END      = 0x6100_0000      # 16 MB framebuffer window


def region_of(addr: int) -> str:
    """Classify a system-bus address into a region name."""
    a = addr & 0xFFFF_FFFF
    if SYS_RAM_BASE <= a < SYS_RAM_END:
        return 'ram'
    if SYS_ROM_BASE <= a < SYS_ROM_END:
        return 'rom'
    if SYS_IO_BASE <= a < SYS_IO_END:
        return 'io'
    if SYS_FB_BASE <= a < SYS_FB_END:
        return 'fb'
    return 'unmapped'


# ═════════════════════════════════════════════════════════════════════════
# debug_ctrl register offsets (BAR 1, relative to DEBUG_BASE)
# ═════════════════════════════════════════════════════════════════════════

# TIER 1 — System control & status
OFF_DBG_VERSION          = DEBUG_BASE + 0x000
OFF_DBG_BUILD_ID         = DEBUG_BASE + 0x004
OFF_DBG_CONTROL          = DEBUG_BASE + 0x008
OFF_DBG_STATUS           = DEBUG_BASE + 0x00C
OFF_DBG_PC               = DEBUG_BASE + 0x010
OFF_DBG_LAST_PC          = DEBUG_BASE + 0x014
OFF_DBG_REDIRECT_PC      = DEBUG_BASE + 0x018
OFF_DBG_REDIRECT_TRIGGER = DEBUG_BASE + 0x01C
OFF_DBG_IRQ_INJECT       = DEBUG_BASE + 0x020
OFF_DBG_EXC_VEC          = DEBUG_BASE + 0x024
OFF_DBG_EXC_PC           = DEBUG_BASE + 0x028
OFF_DBG_RESET_CAUSE      = DEBUG_BASE + 0x02C
OFF_DBG_HALT_AFTER_LO    = DEBUG_BASE + 0x030
OFF_DBG_HALT_AFTER_HI    = DEBUG_BASE + 0x034
OFF_DBG_BREAK_PC         = DEBUG_BASE + 0x038
OFF_DBG_HALT_CTL         = DEBUG_BASE + 0x03C
OFF_DBG_HALT_REASON      = DEBUG_BASE + 0x040
OFF_DBG_HALT_HIT_PC      = DEBUG_BASE + 0x044
OFF_DBG_HALT_HIT_INST_LO = DEBUG_BASE + 0x048
OFF_DBG_HALT_HIT_INST_HI = DEBUG_BASE + 0x04C
OFF_DBG_HALT_EXC_VEC     = DEBUG_BASE + 0x050
OFF_DBG_EXC_FAULT_ADDR   = DEBUG_BASE + 0x054
OFF_DBG_HALT_EXC_MASK0   = DEBUG_BASE + 0x060  # 8 x 32-bit vector-mask lanes
OFF_DBG_BP_SKIP_ONCE     = DEBUG_BASE + 0x080
OFF_DBG_BREAK_PC1        = DEBUG_BASE + 0x084
OFF_DBG_BREAK_PC2        = DEBUG_BASE + 0x088
OFF_DBG_BREAK_PC3        = DEBUG_BASE + 0x08C
OFF_DBG_BREAK_PC_CTRL    = DEBUG_BASE + 0x090
OFF_DBG_FEATURES         = DEBUG_BASE + 0x0A0

# TIER 1 — Performance counters
OFF_DBG_CYCLE_LO         = DEBUG_BASE + 0x1000
OFF_DBG_CYCLE_HI         = DEBUG_BASE + 0x1004
OFF_DBG_INST_LO          = DEBUG_BASE + 0x1008
OFF_DBG_INST_HI          = DEBUG_BASE + 0x100C
OFF_DBG_MISPRED_COUNT    = DEBUG_BASE + 0x1010
OFF_DBG_FLUSH_COUNT      = DEBUG_BASE + 0x1014
OFF_DBG_EXC_COUNT        = DEBUG_BASE + 0x1018

# TIER 1 — PC trace ring
OFF_DBG_PC_TRACE_BASE    = DEBUG_BASE + 0x10000
OFF_DBG_PC_TRACE_HEAD    = DEBUG_BASE + 0x11000
DBG_PC_TRACE_DEPTH       = 1024

# TIER 2 — Architectural register snapshot
OFF_DBG_D0               = DEBUG_BASE + 0x2000   # +0x04 per reg, up to D7
OFF_DBG_A0               = DEBUG_BASE + 0x2020   # +0x04 per reg, up to A7
OFF_DBG_USP              = DEBUG_BASE + 0x2040
OFF_DBG_SSP              = DEBUG_BASE + 0x2044
OFF_DBG_ISP              = DEBUG_BASE + 0x2048
OFF_DBG_SR               = DEBUG_BASE + 0x204C
OFF_DBG_VBR              = DEBUG_BASE + 0x2050
OFF_DBG_CACR             = DEBUG_BASE + 0x2054
OFF_DBG_TC               = DEBUG_BASE + 0x2058
OFF_DBG_ITT0             = DEBUG_BASE + 0x205C
OFF_DBG_ITT1             = DEBUG_BASE + 0x2060
OFF_DBG_DTT0             = DEBUG_BASE + 0x2064
OFF_DBG_DTT1             = DEBUG_BASE + 0x2068
OFF_DBG_URP              = DEBUG_BASE + 0x206C
OFF_DBG_SRP              = DEBUG_BASE + 0x2070
OFF_DBG_ARCH_PC          = DEBUG_BASE + 0x2074
OFF_DBG_ARCH_APPLY       = DEBUG_BASE + 0x2078
OFF_DBG_ARCH_STATUS      = DEBUG_BASE + 0x207C
OFF_DBG_SFC              = DEBUG_BASE + 0x2080
OFF_DBG_DFC              = DEBUG_BASE + 0x2084

# Live arch readback (read-only) — bypasses the host shadow apply path.
# Returns the actual CPU register state via the snap_arch chain through
# m68k_core (cRAT → PRF).  Stable only while the core is halted.
OFF_DBG_LIVE_VBR         = DEBUG_BASE + 0x2100
OFF_DBG_LIVE_SR          = DEBUG_BASE + 0x2104
OFF_DBG_LIVE_A7          = DEBUG_BASE + 0x2108
OFF_DBG_LIVE_D0          = DEBUG_BASE + 0x2110   # +0x04 per reg, up to D7
OFF_DBG_LIVE_A0          = DEBUG_BASE + 0x2130   # +0x04 per reg, up to A7

# TIER 2 — Commit-log ring (8 KB = 256 × 32 B)
OFF_DBG_COMMIT_LOG_BASE  = DEBUG_BASE + 0x14000
OFF_DBG_COMMIT_LOG_HEAD  = DEBUG_BASE + 0x14F00
DBG_COMMIT_LOG_DEPTH     = 256
DBG_COMMIT_LOG_REC_SIZE  = 32

# DBG_VERSION magic word (hi-half bit check == 0xDEB6)
DBG_VERSION_MAGIC        = 0xDEB6_0004

# Debug arch apply bits/status
ARCH_APPLY_START         = 1 << 0
ARCH_APPLY_CLEAR_STATUS  = 1 << 1
ARCH_STATUS_BUSY         = 1 << 0
ARCH_STATUS_DONE         = 1 << 1
ARCH_STATUS_REJECTED     = 1 << 2

# DBG_CONTROL bits
# bit 0: halt_req (level), bit 1: step (pulse),
# bit 2: legacy soft_rst (DEPRECATED — alias of cold_reset_pulse for one
#        release; routes through the unified-reset path),
# bit 3: init_done_override (level),
# bit 4: cold_reset_hold (level — sticky CPU hold across unified reset),
# bit 5: cold_reset_pulse (write-1-to-pulse — canonical unified-reset
#        trigger; replaces VIO-bit-3 / btn[2] / dbg_soft_rst as the
#        host-tool reset surface).
CTL_HALT_REQ             = 1 << 0
CTL_STEP_PULSE           = 1 << 1
CTL_SOFT_RST             = 1 << 2  # DEPRECATED — see CTL_COLD_RESET_PULSE
CTL_INIT_DONE_OVR        = 1 << 3
CTL_COLD_RESET_HOLD      = 1 << 4
CTL_COLD_RESET_PULSE     = 1 << 5

# DBG_HALT_CTL bits
HALT_AFTER_ENABLE        = 1 << 0
HALT_BREAK_PC_ENABLE     = 1 << 1
HALT_CLEAR_LATCH         = 1 << 2
HALT_AUTO_LATCHED        = 1 << 3
HALT_AFTER_LATCHED       = 1 << 4
HALT_BREAK_PC_LATCHED    = 1 << 5
HALT_EXC_ENABLE          = 1 << 6
HALT_EXC_LATCHED         = 1 << 7

# DBG_STATUS bits
STS_HALTED               = 1 << 0
STS_EXC_PENDING          = 1 << 1
STS_INIT_DONE_SEEN       = 1 << 2
STS_CPU_RUNNING          = 1 << 3
STS_AUTO_HALT            = 1 << 4


# ═════════════════════════════════════════════════════════════════════════
# sd_provision register block REMOVED FROM RTL.  These constants remain only
# for MockDevice and its host tests; real BAR1 accesses here receive DECERR.
# ═════════════════════════════════════════════════════════════════════════
SDP_BASE                 = 0x80000
OFF_SDP_VERSION          = SDP_BASE + 0x00000
OFF_SDP_STATUS           = SDP_BASE + 0x00004
OFF_SDP_CMD              = SDP_BASE + 0x00008
OFF_SDP_LBA              = SDP_BASE + 0x0000C
OFF_SDP_CMD_COUNT        = SDP_BASE + 0x00010
OFF_SDP_ERR_CAUSE        = SDP_BASE + 0x00014
OFF_SDP_OWN_REQ          = SDP_BASE + 0x00018
OFF_SDP_BUF_BASE         = SDP_BASE + 0x00100
OFF_SDP_BUF_END          = SDP_BASE + 0x00300

SDP_STATUS_BUSY          = 1 << 0
SDP_STATUS_DONE          = 1 << 1
SDP_STATUS_ERROR         = 1 << 2
SDP_STATUS_CARD_READY    = 1 << 3
SDP_STATUS_OWN_STATE     = 1 << 4
SDP_CMD_NOP              = 0x00
SDP_CMD_READ             = 0x01
SDP_CMD_WRITE            = 0x02
SDP_CMD_READ_MULTI       = 0x03
SDP_CMD_WRITE_MULTI      = 0x04
SDP_VERSION_MAGIC        = 0x5D50_0002
SDP_BLOCK_SIZE           = 512
# ═════════════════════════════════════════════════════════════════════════
