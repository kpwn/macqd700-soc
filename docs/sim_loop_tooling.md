# Simulation Loop Helper

Use `tools/sim_loop.py` from any worktree when iterating on Verilator
simulation targets:

```bash
python3 tools/sim_loop.py sim
python3 tools/sim_loop.py test TEST=moveq STOP_ON_FAIL=1
python3 tools/sim_loop.py tb-alu
python3 tools/sim_loop.py fuzz N=50
```

The helper still delegates to the repo Makefile. It only standardizes the
outer loop:

- sets `VERILATOR_THREADS=4`, `VERILATOR_JOBS=4`, and `MAKEFLAGS=-j1`;
- prints and records the exact command, including the worktree directory;
- streams output while saving a private log under `$(git rev-parse --git-path sim-loop)/logs`;
- reports PASS/FAIL, exit code, runtime, log path, and any Makefile summary line it finds;
- records the current git `HEAD` and short worktree status in `last-run.json`
  / `last-fail.json` so a failing command carries its source context;
- records `last-run.sh` and `last-fail.sh` in the same git-private state directory.

To repeat the last failing command:

```bash
python3 tools/sim_loop.py --show-last-fail
python3 tools/sim_loop.py --repeat-fail
```

For RAM-backed scratch builds and ROM boot traces:

```bash
python3 tools/sim_loop.py --shm tb-rom-boot
```

`--shm` creates `/dev/shm/m68k/<worktree>/build`,
`/dev/shm/m68k/<worktree>/romboot`, and `/dev/shm/m68k/<worktree>/tmp`,
then passes `BUILD_DIR=...` and `ROMBOOT_OUTPUT_ROOT=...` to `make` and sets
`TMPDIR=...`. Remove that worktree directory under `/dev/shm/m68k` whenever
you want to reclaim memory.

This wrapper is for simulation loop targets such as `sim`, `test`, `fuzz`,
and `tb-*`. Targets that look like Vivado, JTAG, FPGA, implementation,
synthesis, or timing flows are refused by default so a quick sim rerun cannot
accidentally start a hardware path.

## ROM Frontier Debug Loop

For frontier stops, keep the raw trace bounded and let the summary tool do
the first-pass triage:

```bash
python3 tools/sim_loop.py --shm rom-boot-stop-summary \
    ROMBOOT_STOP_SUMMARY_STOP='+stop_on_rom_faults' \
    ROMBOOT_STOP_SUMMARY_LASTN_CYCLES=2048 \
    ROMBOOT_STOP_SUMMARY_EXTRA='+periph_event_summary +no_waves'
```

The report now includes a compact diagnosis token derived from the last-N
sampled core, D-cache/MMU, AXI, and ifetch state:

```text
[rom-boot-stop-summary] summary: reason=no-progress cycles=500000 threshold=500000 diagnosis=bus-read-address region=q700-io evidence=arvalid=1,arready=0,addr=0x50f04000 ...
```

Use the diagnosis as the first branch:

- `core-exception`, `core-dmmu-walk`, `core-dmmu-fault`,
  `core-dcache-request`, `core-lsu`, or `core-idle-or-deadlock`: start in
  core-side state in the last-N trace before enabling broad logs.
- `bus-read-*`, `bus-write-*`, or `ifetch-wait`: inspect the address region
  and handshake evidence first.
- `region=q700-io`, `region=dafb`, `region=vram`, or `region=io-high`:
  rerun with a narrow `+periph_event_filter=...` or `+data_watch=...`;
  keep event/detail limits capped.
- `rom-loop-or-retire-stall`: compare the committed PC against the trace and
  decide whether this is a deliberate ROM poll loop or a retire-side stall.

When a plain `tb-rom-boot` run writes a `+lastn_trace_path`, `sim_loop.py`
also scans its captured log for the path and prints the same one-line ROM
summary after the make result. That avoids the old grep loop of finding the
artifact and running `tools/rom_boot_stop_summary.py` manually.

Before:

```bash
make tb-rom-boot ROMBOOT_EXTRA="+lastn_trace=4096 +lastn_trace_path=/tmp/lastn.log +no_waves"
tail -80 build/sim/rom_boot_trace.log
python3 tools/rom_boot_stop_summary.py /tmp/lastn.log --compact
grep -E "daxi_|if=|exc_" /tmp/lastn.log | tail
```

After:

```bash
python3 tools/sim_loop.py --shm tb-rom-boot \
    ROMBOOT_EXTRA="+lastn_trace=4096 +lastn_trace_path=/tmp/lastn.log +no_waves"
python3 tools/sim_loop.py --show-last-fail
python3 tools/sim_loop.py --repeat-fail
```
