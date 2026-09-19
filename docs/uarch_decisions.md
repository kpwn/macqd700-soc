# Key SoC/Platform RTL Design Decisions (DO NOT regress these)

> Architectural decisions and invariants for the macqd700-soc platform
> (peripherals, AXI fabric, DDR4/MIG, video scanout, boot FSM, synth/impl
> flow).  Each entry documents the *why*, the *what*, and pointers to the
> relevant RTL.  Treat these as load-bearing.  For CPU-core (`cpu/`
> submodule) decisions, see `cpu/docs/uarch_decisions.md`.

---

## 1. DAFB `regs[]` (256x32) and RAMDAC CLUT arrays are intentionally NOT bulk-reset (task T8)

`rtl/mac/video.v`'s raw register-file array `regs[0:255]` and the AC842
RAMDAC CLUT arrays `ramdac_clut_r/g/b[0:255]` are **not** cleared by the
module's synchronous `rst`.  A synchronous reset loop over a
dynamically-indexed array forces per-entry reset muxing, which defeats
LUTRAM inference (Vivado falls back to ~14K discrete FFs + a 256:1 decode
instead of compact LUTRAM).  See the reset-block comment in `video.v`
(`if (rst) begin ... end`) for the RTL-level rationale.

**Consequence beyond stale CLUT readback:** several *live, cross-module*
signals are derived directly from this un-reset `regs[]` array, not from
a separately-reset scalar register, so a **debug-only full reset**
(JTAG/VIO-triggered, no power-cycle) leaves them at their pre-reset
value instead of clearing:

- `scsi0_ctrl_out = regs[REG_FIRST_HIT][8:0]` (video.v) — the TurboSCSI
  bus-1 control word consumed by `scsi.v`'s pseudo-DMA/DTACK-hold gating.
  A debug-full-reset does not force this back to 0; the last value the
  CPU wrote to DAFB +0x24 before the reset stays live on the SCSI side.
- `irq_enable_reg = regs[REG_IRQ_ENABLE]` (video.v, DAFB +0x1C low bit)
  — gates `irq_observable` (the DAFB vblank IRQ line into VIA1). If the
  CPU had armed this before a debug-full-reset, vblank IRQs can start
  firing again as soon as `frame_tick` resumes, with no software
  rewrite — i.e. IRQ-enable can appear "silently still armed" across a
  debug reset.
- Swatch auto-arm gates `regs[REG_SWATCH_CTRL][0]` (Swatch VBL arm,
  DAFB +0x104 bit 0) and `regs[REG_SWATCH_CTRL][2]` (Swatch cursor
  auto-arm) — same story: these read straight out of `regs[]`, so a
  debug-full-reset does not disarm them either.

This was an explicit, brief-authorized trade-off (LUTRAM inference over
reset determinism for a *debug-only* reset path), not an oversight — but
it means **debug-full-reset is not equivalent to a ROM cold boot** for
DAFB register state.  Cold boot (power-cycle / bitstream load) zeroes
these arrays via BRAM/LUTRAM INIT values (and Verilator's sim zero-init),
and the Q700 ROM always reprograms all of DAFB's registers — including
+0x24, +0x1C, and +0x104 — before relying on them, so this is invisible
to normal boot.  It only matters for HW/JTAG debug flows that issue a
debug-full-reset mid-session and then assume a clean DAFB register slate
without re-running the ROM's DAFB init sequence.  If a future debug
workflow needs a guaranteed-clean DAFB state without a full power-cycle,
re-program +0x24/+0x1C/+0x104 explicitly (or any other `regs[]`-backed
offset) after the reset rather than relying on it to self-clear.
