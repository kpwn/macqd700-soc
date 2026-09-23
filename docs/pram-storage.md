# PRAM storage contract

The RTC's 256-byte parameter RAM uses synchronous, common-clock dual-port
block RAM. Configuration initializes the existing default image. Ordinary
`rst` never clears PRAM; external restores remain possible while `rst` is high.

Explicit `pram_clear` starts a 256-byte sweep, one byte per peripheral clock.
`pram_busy` includes the asserted clear input and the unfinished sweep. Holding
clear high does not restart the sweep; another rising edge does. Reset does not
interrupt a sweep. At 50 MHz, the sweep takes 5.12 microseconds.

Clear is a destructive maintenance operation: the serial transaction in progress
is aborted, and serial commands must start after busy falls. No partial clear
image is presented as a valid serial transaction. The seconds counter is not
cleared by this operation. The SD persistence CDC waits for PRAM readiness before
accepting a byte, so a restore request during clearing is retained, not dropped.
An explicit later clear still intentionally erases previously accepted writes.

Serial and external accesses use separate RAM ports. Both writes can complete
on the same edge when their addresses differ; external restore wins a same-byte
write collision. Both ports use read-before-write behavior on a common clock.
The serial read is captured at command/address completion and transferred to the
shift register before the next falling serial-clock edge. The external read has
one peripheral-clock latency. Its CDC waits an extra ready cycle after a sweep
so that the read output is fresh before acknowledgement.

Verification must cover every byte, serial/external cross-path reads, normal and
extended commands, both default images, repeated clear, warm reset during clear,
restore during clear, and write/read collisions. Synthesis must demonstrate BRAM
inference and report matched RTC FF/LUT counts; simulation alone cannot prove it.

Focused gates:

```
make tb-rtc tb-pram-bram-cdc tb-pram-sd tb-pram-sd-populated tb-pram-sd-autoload
make lint-fpga-top CPU=stub
```

The standalone RTC test covers the minimum two-system-clock serial-bit period,
same-edge port collisions, deterministic pseudorandom data at every address,
reset during clearing, and restore while reset is asserted. The CDC test queues
reads/writes throughout the sweep, including a read of the final cleared byte.
It runs at both 100:50 and phase-offset 200:50 MHz core/peripheral clock ratios.
The persistence tests compare the real SD sector and serial-visible PRAM across
both default images. The defaults-fallback hold must exceed 256 peripheral clocks
plus synchronization; shipping 2048 core clocks satisfy this at both 100 and
200 MHz core with 50 MHz peripherals.

## Validation status

The implementation's focused simulation gates pass: 21 RTC scenarios, both CDC
clock configurations, 14 SD persistence scenarios with each default image, and
two boot-autoload scenarios. Top-level `CPU=stub` lint passes. These checks do not
claim CPU/SoC timing closure or physical-board verification.

The matched RTC-only synthesis comparison completed successfully: 4,058 to
205 LUTs and 2,208 to 146 FFs, with exactly one RAMB18 and READ_FIRST on both
ports. This is out-of-context synthesis with identical options, not a whole-SoC
area or timing result. Reports are in the original `pram-bram` worktree under
`build/pram_area/{baseline,candidate}/`; log `/tmp/pram-bram-area.log`.
The compared RTC source is unchanged from the baseline used by the current
200 MHz cleanup builds. Their integrated synthesized RTC is 1,705 LUTs and
2,210 FFs; do not subtract out-of-context savings from those integrated counts.

The change is now integrated with the IQ/L2/CSR cleanup candidate. Fresh checks
pass all 21 RTC scenarios, both CDC clock configurations, 14 SD persistence
scenarios per default image, and two boot-autoload scenarios. Integration log:
`/tmp/ipc-cleanup-pram-integration-tests.log`. Full-SoC lint, BRAM inference and
placement/routing must still be checked for this integrated candidate.

No CPU pipeline or memory-interface latency changes are made here, so there is
no new simulated CPU IPC claim. Whole-SoC routing and matched board IPC still
need checking after integration; no board reset or reload was performed.
