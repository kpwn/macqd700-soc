# Board debugging

Use the bitstream's matching `.ltx` and build information. Debug register
availability depends on the CPU revision and enabled hardware debug blocks;
do not assume an old capture script matches a new build.

## Attach without resetting

The persistent Vivado REPL provides JTAG access through `/tmp/jtag_in` and
`/tmp/jtag_out`. To attach to an already running, matching image:

```sh
JTAG_REPL_NO_PROGRAM=1 tools/p141_start_repl.sh /path/to/fpga_top.local.bit /path/to/fpga_top.ltx
tools/jt.sh "build-id"
tools/jt.sh "halt-status"
tools/jt.sh "help"
```

Without `JTAG_REPL_NO_PROGRAM=1`, the launcher programs the FPGA and resets
the machine. Do not use that form just to recover a lost debug connection.
Check the reported build ID before interpreting registers or captures.

Use `tools/jtag_lease.sh` to reserve board access and `tools/jt.sh` to send
commands. Never write directly to the FIFO: a second writer can interrupt
a long operation. SD provisioning needs exclusive access for the whole write
and verification sequence.

## Observe before changing state

Record halt/exception state and the faulting instruction before continuing,
resetting or replacing the bitstream. Reset and CPU halt are different
operations; a halt should preserve the state being investigated.

Use `perf live` for running performance counters. A halt-snapshot instruction
count is not a live counter and may legitimately read zero while the CPU is
running. Compare counter deltas over a defined interval and report the
denominator for rates such as branch misses.

Host memory access is not automatically coherent with CPU L1 caches. Check
cache state and the tooling's maintenance behavior before diagnosing a stale
read or changing code/data under a running CPU. MMIO reads may have side
effects; they are not a harmless replacement for a captured register state.

## Optional GDB interface

The host tools can provide symbols and a GDB remote interface over the REPL:

```sh
tools/macsym.py build -o build/macos.elf
tools/gdbstub.py --port 1234
# In another terminal:
gdb-multiarch build/macos.elf
# At the GDB prompt: target remote :1234
```

The symbol file contains names, not a copy of the running Mac's code.
Memory and debug operations still reach the board. Use the current tool's
help for supported commands and hardware breakpoint/watchpoint limits.
`tools/gdbstub.py --fake --port 1234` exercises the host interface without
a board; it does not validate CPU execution or hardware debug behavior.
