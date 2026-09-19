# RTC has no persistence: time resets to 1904 and PRAM wipes on every reset

Reported symptom (2026-07-28): the menu-bar clock does not show a time, and
date/time *storage* does not survive either.

## Root cause — confirmed by RTL read, not inference

`rtl/mac/rtc.v:153`:

    always @(posedge clk) begin
        if (rst) begin
            seconds     <= init_seconds;   // init_seconds = 0 in synthesis
            ...

`init_seconds` is set to 0 by the `initial` block; the `+rtc_init_seconds=`
override is **simulation-only** (`$value$plusargs`), so a bitstream always
starts the counter at 0 = **Apple epoch 1904-01-01 00:00:00**.

`rst` is `pb_full_rst_bank[1]` (`rtl/soc/fpga_top_peripherals.vh:1288`), so
every full reset rewinds it.  The instantiation comment already states the
situation plainly:

    // The 32-bit seconds counter naturally restarts at 0 too; this
    // matches the cold-boot semantic (PRAM-tracked time on a real
    // Quadra 700 is held in the DS1287 across reset, but our
    // simulated PRAM is volatile — see task #145).

PRAM is likewise volatile and, since the 'SCBI' fix, **zero-filled by
default** (`pram_reset_byte` is opt-in behind `+rtc_populated_pram`).

## What is NOT wrong

The tick path is fine — do not go looking there:

* `u_rtc` IS instantiated (`fpga_top_peripherals.vh:1282`).  It is easy to
  conclude otherwise: the SoC's instantiations live in `.vh` includes, so
  `grep --include=*.v` for the instance finds nothing.  **Grep `.vh` too.**
* `.phi2_tick(phi2_tick)` is connected.
* `SEC_DIV(VIA_PHI2_HZ)` parameterises the divisor off the real phi2 rate.
* The half-second CKO toggle and `seconds + 1` on the falling half are present
  and correct.

So seconds DO advance while the machine runs.  They just always start at zero
and are lost on reset.

## Why that produces "no time in the menu bar"

1. All-zero PRAM has no valid signature, so Mac OS treats the clock as never
   set rather than showing 1904.
2. The menu-bar clock's own "show the time" preference is itself stored in
   PRAM — zeroed means off.

Both point the same way, so the symptom is the EXPECTED consequence of a
machine with no battery-backed clock, not a separate failure.

## Relationship to the ioResult stall — mostly separate, one real link

These are different mechanisms and should not be conflated: the ioResult stall
is a stuck SCSI DRQ holding VIA2 PA6 low and trapping the ROM slot dispatcher
(see `docs/ioresult_stall_investigation.md` §6y in the CPU repo).

The one genuine link is that **PRAM contents demonstrably change the boot
path**.  Precedent in this repo: the populated `pram_reset_byte` image
(bytes 0xF8-0xFB = 'SCBI') made the ROM assemble a non-zero D4 at 0x40846CE6
and branch off the path MAME takes with zero PRAM — fixed in `f57350c` by
zero-filling.  So PRAM state is not inert with respect to boot behaviour, and a
different PRAM image can select different defaults (including boot-device
choice).  That is worth keeping in mind, but it is not evidence that the clock
and the ioResult stall share a cause.

## Fix options, cheapest first

1. **Don't rewind `seconds` on `pb_full_rst`** — reset it only on power-on /
   configuration load.  One-line change; makes time survive the resets that
   JTAG debugging issues constantly.
2. **Seed `seconds` with a build-time constant** so a cold boot starts at a
   plausible date instead of 1904.  Requires threading a parameter from the
   build (the existing `+rtc_init_seconds` plusarg is the sim-side analogue).
3. **True persistence**: back `seconds` + the 256 PRAM bytes with SD or SPI
   flash, written on change and reloaded at boot.  This is what actually
   matches DS1287 behaviour, and would also let Mac OS keep its settings.

(1)+(2) together would give a sane clock immediately; (3) is the correct
long-term fix.
