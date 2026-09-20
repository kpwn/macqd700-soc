# Documentation

## Architecture

- [SoC architecture](architecture.md): CPU boundary, memory, peripherals,
  clock domains and reset responsibilities.
- [L2 system cache](l2c_spec.md): geometry, ports, coherency and reset rules.
- [Fabric concurrency contract](fabric_concurrency_contract.md): interface
  requirements and ordering guarantees.

CPU microarchitecture belongs in the pinned [cpu040 repository](../cpu040),
not in a second, potentially outdated description here.

## Using the hardware

- [ADB firmware setup](adb_firmware_bitstream.md)
- [SPI flash programming](spi_flash.md)
- [SD provisioning over JTAG](sd_jtag_writer.md)
- [Board debugging](debugging_the_mac_on_fpga.md)

The [main README](../README.md) covers building, SD layout and testing.
Release-specific timing and utilization reports accompany the bitstream.
Historical plans, investigation logs and superseded core documents remain
available in Git history; they are not current design specifications.
