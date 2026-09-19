# RESET WEDGE — measured diagnosis, and a correction to two commits

## The two commits below do NOT fix "a reset kills the board"

  * `fix(storage): a JTAG/debug cold reset must toggle the SD-bridge epoch too`
  * `fix(boot): bound the ROM re-copy — a reset must not be able to brick the board`

Both were reasoned from the RTL and committed WITHOUT reproducing the symptom. Tested
afterwards on a bitstream carrying both: firing the reset they target still wedges the
machine. Their commit messages assert a cure that is not demonstrated — do not trust them.

The changes are retained as defensible hygiene (a reset source that re-arms the boot FSM
plausibly should also reset the storage bridge; a bounded watchdog on a boot-critical wait
is reasonable). They are NOT the fix.

## What is actually happening (measured, same bitstream, back to back)

| reset path | REPL command  | result |
|---|---|---|
| JTAG cold-reset pulse | `reset`          | WEDGES: PC static in ROM, exc_count frozen, dafb_live=0; only `load-bit` recovers |
| VIO hard reset        | `vio-hard-reset` | RECOVERS: PC moving, exc_count climbing, video up |

During the wedge `rom_loading_q = 0` and `boot_error = 0`, so the boot FSM is NOT stuck
copying the ROM — the SD bridge was never the blocker. The CPU is simply HELD, which is
also why exc_count is frozen.

The mechanism was already documented in `tools/jtag_repl.tcl` (`vio_reset_and_halt_after`):

  "The current loaded bitstream cannot reliably clear cold_reset_hold while the JTAG pulse
   path keeps soc_full_rst asserted; VIO reset does not have that failure mode."

Because that path holds `soc_full_rst`, it also clears `rom_ever_loaded` — so the ROM
re-copy watchdog was STRUCTURALLY INCAPABLE of firing on the very path it was written for.

## Workaround available now

Use the VIO reset path. If the web UI's reset drives the JTAG cold-reset pulse, point it at
the VIO reset instead.

## Real fix, still open

Make the JTAG pulse path release `soc_full_rst` / `cold_reset_hold` reliably. There is now a
reliable reproduction to test against: `reset` wedges, `load-bit` recovers. Reproduce first.

---

## 2026-09-12 — measured on hardware; the diagnosis above is REFUTED

Reproduced on demand against build_id `0x3A6510BD` (integrate-eth, 2026-09-11 20:16),
twice, with live JTAG measurement rather than code reading.

### What the earlier entry claimed, and why it is wrong

It claimed the JTAG pulse path "keeps `soc_full_rst` asserted", holding the CPU.
Measured `vio_rst_bundle` = `0x2e` = `{platform_resetn=1, core_rst=0, ddr_cal_done=1,
boot_rom_ready=1, hdmi_i2c_done=1, fb_underflow_sticky=0}`.

**`core_rst = 0` and `boot_rom_ready = 1`.** The CPU is NOT held. Neither is
`dbg_cold_reset_hold` (`reset hold-status` → `cold_reset_hold=0 control=0x00000000`).

### What IS true after a `reset` that kills the machine

| Observation | Value | Meaning |
|---|---|---|
| `vio_rst_bundle` | `0x2e` | CPU out of reset, boot ROM ready |
| `reset hold-status` | `cold_reset_hold=0` | not held by the hold bit |
| `vio_boot_diag` | `0x00000800` | sector=2048 (full 1 MB), err_cause=0, retry=0 — boot_fsm re-ran and COMPLETED |
| `boot_error` | 0 | no SD/boot error |
| `dump-mem 0x0` | `420d8602 / 0000002a` | valid reset vectors present, identical to the `0x40000000` ROM mirror |
| DAFB mode | `hres=0 vres=0` (was 640x480) | peripherals DID reset |
| video source | `dafb_live=0`, boot splash | the 68k never reprogrammed the DAFB — it never got that far |

So: peripherals reset correctly, the ROM was re-copied correctly, the vectors are
readable, and the CPU is released — and it still does not execute.

### Things now ruled OUT

* CPU held in reset (`core_rst=0`).
* `dbg_cold_reset_hold` stuck (measured 0).
* boot_fsm stuck / SD re-copy failing (sector=2048, err_cause=0, boot_error=0).
* Missing or corrupt reset vectors (read back correct).
* A CPU-release *timing race*: `reset hold` + a 3 s wait + `reset release` fails
  identically, so releasing the CPU later does not help.
* The debug halt lanes: `effective=0`, all enables and latches 0, and
  `halt-clear` + `halt-release` do not revive it.

### ⚠ Two probes that LIE on this build — do not reason from them

`pc_live` and `exc_count` are the dead-probe pair already recorded in memory.
After the wedge `pc_live` reads `0x00000000` and `exc_count` freezes at a stale
non-zero value (`0x05ccddec`) — the *same* value across many minutes and across a
reset. "PC=0" is NOT evidence the CPU is at address 0. Use `video-status`
(`video mode` / `video source`) as the liveness indicator; it is reliable.

### ⚠ `vio-hard-reset` is NOT a dependable workaround

Previously recorded as the recovery path. On 2026-09-12 it failed to recover the
machine across a 40 s observation window. Only a full `load-bit` reprogram
restored it. Treat reprogramming as the recovery path.

### The remaining suspect, and the next step

Everything downstream of the CPU checks out, which leaves the one piece of state
with no probe: the **ROM overlay**. `effective_cpu_overlay_active =
cpu_overlay_active && !cpu_overlay_disabled_q` (axi_xbar.v), where
`cpu_overlay_active` is VIA1's bit and `cpu_overlay_disabled_q` is the sticky
latch cleared by `cpu_overlay_reset = soc_full_rst_bank[4] | warm_peripheral_reset`.
If the overlay stays disabled across a debug reset, the CPU's vector fetch at
0x0/0x4 goes to plain DRAM instead of the ROM alias and it dies immediately —
which fits every observation above, including "peripherals reset but the CPU
never runs".

This could not be confirmed because neither `cpu_overlay_disabled_q` nor
`via1_overlay_bit` is brought out to a VIO probe (they only drive LEDs).
**Next step: add both to a VIO probe bundle and rebuild.** That is a few lines in
fpga_top_debug_vio.vh and makes the next occurrence decidable in one command,
instead of another round of inference.
