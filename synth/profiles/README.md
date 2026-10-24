# Throughput-v2 build profiles

Both profiles use the same CPU execution options, Ethernet, 50 MHz peripheral
bus, storage/reset policy, and detailed performance counters.

| Feature | 100 MHz full debug | 200 MHz reduced debug |
| --- | --- | --- |
| Environment file | `throughput-v2-100mhz-debug.env` | `throughput-v2-200mhz-lean.env` |
| CPU PC-range breakpoint | Included | Hardware omitted |
| IPC and legacy ILA | Included | Omitted |
| SCSI trace | Included | Omitted |
| Counters, JTAG AXI, VIO | Included | Included |
| Exact-PC breakpoint, manual/exception halt | Included | Included |

To select a profile, export its settings before invoking the usual build:

```sh
set -a
. synth/profiles/throughput-v2-200mhz-lean.env
set +a
make impl
```

Use separate build directories/worktrees for the two profiles. `CPU_DEBUG_PROFILE`
is forwarded through both `make cpu040-gen` and Vivado's CPU regeneration step;
it defaults to `full` outside these profiles. Reduced builds return zero for
PC-range registers (`0x124`–`0x13c`) and ignore writes. Software must not claim
a range breakpoint is armed unless its enable bit reads back set.

Full debug means the supported CPU/IPC/storage diagnosis facilities; it does
not enable PCIe XDMA or Ethernet traffic generators. Neither profile relaxes
debug timing constraints, and neither programs or resets a board automatically.
The profile name is a frequency target, not a claim of timing closure.

## Validated P4 split-load image

`p4-split-200mhz-validated.env` records the exact CPU options used for the
P4-enabled 200 MHz image validated on 2026-10-24. It retains the SCSI trace
ring and uses the `AltSpreadLogic_high` placement and `AggressiveExplore`
route directives. The corresponding SoC RTL is commit `8284a403`; later SoC
`main` commits only changed the `cpu040` submodule pointer. The CPU hardware
is commit `00df756f`; later CPU `master` commits changed the replay harness
and documentation, not RTL. The source pins on the branches are intended to
make the successful image reproducible, although a fresh implementation must
still pass its own timing report.

In a clean shell with the pinned CPU submodule initialized, run:

```sh
set -a
. synth/profiles/p4-split-200mhz-validated.env
set +a
make impl-split TARGET_FREQ_MHZ=200 DDR4_MIG_DCP=/path/to/design_1_ddr4_0_1.dcp
```

The validated route had setup WNS +0.031 ns and hold WHS +0.010 ns. Four board
Benchmark Mix runs averaged 12.139 at 199.60-199.62 MHz and mean IPC
0.563651. The matched P4-disabled image used the same SoC RTL and CPU
environment, with only `CPU_LS_SPLIT_ONE_XLATE=0` and `CPU_LS_SPLIT_FWD=0`;
it averaged 11.9085 and IPC 0.554358. The CPU repo's
`docs/P4_200MHZ_VALIDATION_2026-10-23.md` records the full simulation, timing,
benchmark, and Dhrystones tradeoff. The blank bitstream needs the local ADB
firmware patch before loading on the board; that firmware is not in this
repository.

After the matched board runs, a read-only card check used the provisioning
image's `sd-read` command on all 2048 ROM sectors and four 256-sector disk
samples at LBAs 8192, 54784, 58112, and 1000000. Each batch passed the
staging CRC check and a second independent SD read with no warnings or
retries. The known-good main image was restored afterward. Logs and hashes
are under `/var/tmp/codex-p4-storage-readcheck/`. This establishes stable
reads for those regions; it does not assert that the mutable filesystem
matches a host image or that every card sector was checked.

The promoted SoC source also passed `make tb-scsi-sd-e2e-c96` on 2026-10-24:
27 passed, zero failed, including C96 write ordering under SD back pressure
and the System 7.5.3 word-burst shape. Its positive control,
`make tb-scsi-sd-e2e-c96-noorder`, confirmed that c26 fails when
`SCSI_PDMA_OUT_FIFO_ORDER` is disabled. Logs are
`/var/tmp/codex-p4-200/scsi-sd-e2e-c96{,-noorder}.log`. The P4 integration
does not alter SoC RTL relative to the tested `8284a403` source; the existing
FIFO-order fix and 200 MHz SD divider remain in that source, and
`SD_SAFE_CMD25=0` matches the established 200 MHz profile. These checks cover
the simulated SCSI-to-SD write path and sampled physical-card reads; they do
not claim a new physical-card write test of the P4 image.
