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
