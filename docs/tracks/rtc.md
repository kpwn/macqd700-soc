# RTC track

> Read `docs/agent_policy.md` first. This brief adds RTC-track scope.

## Scope

`rtl/mac/rtc.v`, the ROM-boot VIA1 side-channel model in
`tb/tb_rom_boot.cpp`, and their focused tests. Keep behavior local to
RTC / PRAM and the VIA PB0/PB1/PB2 connection.

## Current semantics

- Reset starts the seconds counter at `0`.
- Idle `rtcData` reads as pull-up high when the CPU is not driving it.
- Command, address, and write-data bits sample the physical host data line.
  If VIA1 leaves PB0 as an input for a one bit, the pull-up is accepted as
  the shifted `1`; zero bits are driven low.
- Read commands return the 32-bit seconds counter low byte first:
  `0x81`, `0x85`, `0x89`, `0x8D` map to bits `[7:0]`, `[15:8]`,
  `[23:16]`, and `[31:24]`. Register aliases `4..7` map back to the
  same four seconds bytes, matching MAME `macrtc`.
- PRAM is 256 bytes. Normal commands expose register windows `8..11`
  and `16..31`; extended commands `(cmd & 0x78) == 0x38` address all
  256 bytes with the sector bits in the command byte and byte bits in
  the following address byte.
- Reset uses deterministic PRAM defaults for the ROM-visible Q700 bytes
  and zeroes the rest. The model does not persist host NVRAM state.
- Writes commit when the transaction ends with `rtcEnb` rising.
- The test register (`reg 12`) and write-protect register (`reg 13`) have
  deterministic readback. The test bit is a latch only; it does not reset
  or gate the clock. Write-protect accepts the 343-0042 register-13 command
  variants (`0x34..0x37`) and blocks normal/extended writes except writes to
  the write-protect register itself.
- Simulation can pass `+rtc_trace` for RTC/PRAM command, data, and
  protected-write logging without changing bus-visible behavior.
- The ROM-boot harness now drives PB0 during VIA1 RTC read transactions
  rather than only logging pin traffic. Its seconds counter is deterministic
  and advances every 50,000 host cycles so ROM polling can observe time.

## Verification

`make tb-rtc`
`make tb-rom-boot-rtc-smoke`

The unit test should cover reset defaults, seconds reads and aliases,
PRAM byte addressing, extended PRAM, pull-up one bits, control/write-protect
behavior, transaction sequencing, a truncated-write abort case, read latching
during ROM-style polling, trace logging, and the fast-timebase path.
