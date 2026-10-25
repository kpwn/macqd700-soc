# TQTT 100 MHz SD read-only experiment (Samsung EVO, 2026-10-25)

This branch keeps the validated 50 MHz native-SD implementation on its own
branch. The TQTT socket, pull-ups, and FPGA bank are fixed at 3.3 V. This
experiment selects the card's High Speed function with CMD6, then clocks it
at 100 MHz at 3.3 V. It is an overclock, not a negotiated UHS mode. The
probe cannot issue a card write.

The standalone probe derives a 200 MHz controller clock from the board's
100 MHz differential clock and uses LiteSDCard dividers 500 (400 kHz card
identification), 8 (25 MHz CMD6), and 2 (100 MHz data). The routed
read-only timing-counter bitstream is
`/var/tmp/tqtt_sd_oc100_edges_probe/tqtt_sd_native_engine_probe.bit` with
matching `.ltx`. Final 200 MHz setup/hold/pulse-width slacks were
+1.371/+0.010/+1.958 ns with zero failing endpoints.

The automatic read-only matrix at
`/var/tmp/tqtt_sd_oc100_matrix/summary.tsv` passed repeated CRC32 reads at
LBA 0, 8192, 90000, 95410, 250000, 500000, 1000000, and 2000000. Every
32-sector CRC matched the corresponding 50 MHz matrix. Fast 16 KiB reads
measured 12-15 MB/s at 100 MHz versus 8-11 MB/s at 50 MHz; slow regions
remained near 2 MB/s at either clock.

The diagnostic probe times CMD18/data, DMA, CMD12, and RAM streaming and
counts SD clock edges. At fast LBA 8192, CMD18/data ended after 262,225
200 MHz cycles and the total request after 287,069 cycles, with 131,228 SD
clock edges. At slow LBA 500000, CMD18/data ended after 1,544,105 cycles
and the total after 1,568,949 cycles, with 772,168 SD clock edges. The
clock kept running near 100 MHz; the location-dependent delay is inside the
card-facing command/data interval. DMA completion cost only a few cycles,
the CMD12 safety gap about 42 us, and RAM streaming about 82 us. Logs are
`/var/tmp/tqtt_sd_oc100_edges_8192.log` and
`/var/tmp/tqtt_sd_oc100_edges_500000.log`.

This sparse read-only matrix does not qualify the overclock for writes or
normal Mac storage. Use the validated 50 MHz mode as the integration default
until a longer read stress test, write/readback test on backed-up media, and
full-Mac test pass. Larger multi-block requests are the next throughput
target: they amortize the observed per-command card latency.
