# SCC track

> Read `docs/agent_policy.md` first. This brief narrows the peripheral-track SCC work.

## Scope

Zilog Z85C30 SCC behavior for the Quadra 700 bring-up path. The owned
surface is:

```
rtl/mac/scc.v
tb/tb_scc.cpp
docs/tracks/scc.md
```

Keep the work bounded to SCC-only register semantics and unit coverage.
Do not expand into unrelated peripherals or the system core.

## Current focus

- Channel A/B decode isolation.
- WR0 pointer selection and reset-to-zero behavior.
- WR0 Universal-bus command decode: reset external/status (`0x10`)
  and reset TX interrupt pending (`0x28`) must not alias.
- WR2 as the shared interrupt vector.
- WR9 reset commands and control-bit preservation.
- RR0 / RR1 / RR2 readback behavior for ROM probes, including stable
  idle status and RR1 overrun at the documented bit (`0x20`).
- RX/TX empty status when no external serial device is attached.
- IRQ deassertion after clear/reset paths.
- Interrupt-safe idle defaults when ROM enables SCC interrupts before
  attaching any serial source.
- Direct data-port access to RR8/WR8 must remain deterministic for
  idle LocalTalk/modem/printer probes and must not disturb the pending
  control-port pointer.
- Optional simulation logging for ROM SCC probe correlation.

## Reference notes

- Local MAME availability: `/usr/games/mame -version` reports `0.264`.
  No local MAME source checkout was used for this hardening pass.
- Command/status constants were checked against the repo-documented MAME
  source reference (`src/devices/machine/z80scc.cpp`, mame0287) and the
  Z85C30 user-manual command table.

## Test target

Run:

```bash
MAKEFLAGS='-j1' make tb-scc
```

This target is the regression gate for SCC register-file changes.

For ROM-harness visibility, run:

```bash
MAKEFLAGS='-j1' make tb-rom-boot-scc-asc-smoke
```

That target exercises the harness SCC shadow and ASC register aliases without
a full ROM run.  It checks that SCC register/data reads and writes appear in
the peripheral event summary, that RR0 reports idle-ready status instead of a
flat zero, and that ASC FIFO/control touches carry register names in the log.

For ROM probe correlation, build the SCC model with `-GLOG_PROBES=1`.
The trace is off by default and prints one compact line per SCC
peripheral-bus read/write, including channel, port, pointer state, RR0
status, and IRQ state.

## Remaining limitations

- Only the async subset used by the Mac bring-up path is modeled.
- External serial pins are still represented by deterministic local
  status behavior rather than a full physical-layer model.
- The interrupt vector path is intentionally lightweight and does not
  model every SCC daisy-chain detail.
