| idle_loop.s -- task #272 synthetic boot stub (SOC-3 behavioral smoke).
|
| NOT firmware.  This is the minimum a real 68k reset-vector boot needs:
| a valid initial SSP at address 0, a valid initial PC at address 4 (the
| standard 68k cold-reset vector table -- see M68kSocketTop's
| ResetVectorPlugin, which reads these two long-words off the CPU's own
| axi_d-side merge arbiter exactly like real hardware), followed by a
| few NOPs and a tight branch-to-self ("idle") loop at the initial PC.
|
| Purpose: confirm the SOCKET integration -- reset deassert, first
| instruction fetch (axi_i_arvalid), and sustained fetch/decode/execute
| without wedging -- with NO dependency on real OS/firmware behavior.
|
| Build (mirrors tb/tests/cold_boot_periph's M68K_AS/M68K_LD/M68K_OBJCOPY
| pattern in the top-level Makefile):
|   m68k-linux-gnu-as -m68040 -o idle_loop.o idle_loop.s
|   m68k-linux-gnu-ld -Ttext 0x40000000 -o idle_loop.elf idle_loop.o
|   m68k-linux-gnu-objcopy -O binary idle_loop.elf idle_loop.bin
| (-Ttext 0x40000000 matches `AXI_ROM_BASE` / rtl/soc/axi_defs.vh --
| the reset-time CPU overlay in rtl/soc/axi_xbar.v aliases any raw CPU
| address below `AXI_ROM_SIZE` [0x0040_0000] onto this ROM window, so
| the vectors at offset 0/4 below are what the CPU actually reads for
| its address-0/address-4 reset fetch.)
|
| Run: make tb-fpga-top-rom CPU=m68k040 ROM=<path to idle_loop.bin>

    .text
    .org 0

_vectors:
    .long   0x00500000     | initial SSP: arbitrary point inside real
                            | backing DDR RAM (tb harness RAM_BYTES =
                            | 0x0400_0000), chosen ABOVE AXI_ROM_SIZE
                            | (0x0040_0000) so the CPU reset overlay
                            | does not alias it back onto this ROM
                            | window -- plausible, but never actually
                            | used by this program (no CALL/exception
                            | path exercises the stack).
    .long   _start          | initial PC (right after the 8-byte vector
                            | table, i.e. ROM_BASE + 8)

_start:
    nop
    nop
    nop
_idle:
    bra.s   _idle
