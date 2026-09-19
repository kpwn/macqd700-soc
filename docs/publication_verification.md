# Publication verification — 2026-09-19

The 200 MHz release is an experimental, board-tested image, not a claim that
every regression test passes. Publication changed documentation and test/build
infrastructure, not the image's product RTL.

## Source and release checks

- Fresh GitHub CPU checkout at `8dccf93d`: CPU generation passed.
- Final publication snapshot: `make lint` passed, including real MIG and
  Ethernet configurations.
- CPU fast gate on `049d2abb`: 372 passed, 2 ignored, no failures. Relative
  to the pinned CPU revision, that commit changes tests and docs, not RTL.
- Firmware converter: 3 tests passed; generated PIC hex matches the original.
- Publication checker: 3 tests passed; indexed-tree scan passed.
- README SD recipe: 2 tests passed, including a wrong-offset negative control.
  These use synthetic regular files, not a physical SD card.
- Published bitstream, probes and build metadata were downloaded again and
  verified against `SHA256SUMS`. SPI programming/read-back had also passed.

## Full SoC run and follow-up

The fresh initial export (development revision `28f762ff`) completed
`make test` with **105 PASS, 3 XFAIL, 8 FAIL, 116 targets**. It exited
nonzero. The failures were investigated, not hidden by expanding the
expected-failure list.

Five failing targets now pass after focused test/build repairs:

| Target | Repair and rerun |
|---|---|
| `tb-framebuffer-pixel` | Removed a deleted byteswap source from the recipe and restored its current dependencies; pixel scenarios passed. |
| `tb-q700-eth-sonic-engine` | Used a sibling build directory so Verilator's parent-directory VPATH cannot reuse the stub-mode C++ object; all 8 engine tests passed, and the 4 stub tests passed separately. |
| `tb-scsi-trace-pb` | Explicitly tied the wrapper's unused independent peripheral reset inactive; integration checks and capture-disabled negative control passed. |
| `tb-pram-sd-autoload` | Isolated its build from the manual-mode object; both autoload tests passed. Both 14-test manual PRAM runs also passed. |
| `tb-n2w-vram-byte` | Explicitly left four unused crossbar observation outputs unconnected; this test and the shared VRAM end-to-end harness passed. |

The obsolete `tb-dma-integration` entry was removed from the aggregate list:
its target, test sources and retired `axi_n64_to_wide` wrapper were already
absent. The current `tb-dma-engine` and `tb-dma-l2c` tests passed.

The complete suite was not rerun after these repairs; the changed targets
and their shared-wrapper/mode counterparts were rerun individually.

## Unresolved results

- `tb-vram-ddr-chain`: 16 passed, 1 failed. The partial S3 write does not
  retire in `s3_flush_abort_no_wedge` within the test's bound.
- `tb-vram-ddr-chain-nol2c`: 13 passed, 4 failed. It has the same flush
  failure and also invokes three L2-dependent checks with L2 disabled.
  The test's mode selection needs correction; the flush failure remains
  unresolved. Neither target was reclassified as an expected failure.
- Existing XFAILs: `tb-dafb-via-irq`, `tb-video-smoke`, and
  `tb-mem-model`. The last lacks its memory-model source.
- Hardware: 832×624 at millions of colors fails, while 256 colors works.
  Its pixel-exact mode-matrix and DDR scanout simulations passed. Those
  simulations do not explain or invalidate the board observation.

Consequently, **the full suite is not green**. Keep these failures visible
when reproducing the release; resolving them is separate from source
publication and must not silently replace the released bitstream.

