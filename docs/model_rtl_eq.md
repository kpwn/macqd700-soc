# Model-vs-RTL Equality Checker

`make tb-model-rtl-eq` is the parity gate for the ROM host model surface
against RTL-observable decode contracts.

It runs two layers:

1. `tools/check_model_rtl_consistency.py` (static contract checks)
2. `tb/tb_model_rtl_eq.cpp` (dynamic `RomBootBus` vs `rtl/mac/glue.v`)

## Run

```bash
make tb-model-rtl-eq
```

## What It Guarantees

- Memory-map/decode parity for high-impact canonical addresses:
  low RAM/overlay, ROM window, IO windows, VRAM aperture, unmapped gaps.
- Overlay transition parity on the shared contract:
  reset overlay-visible low ROM, VIA-driven overlay release, low RAM visibility.
- Peripheral classification parity for:
  VIA1, VIA2, SCC, SCSI, ASC, and DAFB register window.
- Host-model baseline register behavior smoke for VIA1/VIA2/SCC/SCSI/ASC.

## What It Does Not Guarantee

- Full cycle-accurate equivalence of peripheral internal behavior.
- AXI fabric timing/arbitration behavior (`axi_xbar`, `peripheral_bus` FSMs).
- End-to-end ROM-boot architectural parity (covered by ROM frontier/parity tools).
