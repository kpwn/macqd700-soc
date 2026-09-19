# Ethernet source provenance

`third_party/taxi` is a source subset of
https://github.com/fpganinja/taxi at upstream commit
`c71926499b47aa7e3dd963b994bd040319d596f5`.
Keep its `LICENSE`, `AUTHORS`, upstream README and per-file notices.

The subset retains `axis`, `eth`, `io`, `lfsr`, `prim`, `stats` and `sync`
source areas and their relative file-list links. Upstream testbenches and
unrelated subsystems were omitted when originally vendored. The unused
`src/eth/example` board projects were removed during publication cleanup;
some referenced upstream test libraries absent from this subset.

Four files contain local modifications: `taxi_axis_gmii_rx.sv`,
`taxi_eth_mac_1g.sv`, `taxi_eth_mac_1g_rgmii.sv`, and
`taxi_eth_mac_1g_rgmii_fifo.sv` under `src/eth/rtl`. They add and thread the
default-off `KEEP_FCS` option: SONIC receives the actual four FCS bytes,
with the original CRC-error verdict preserved, rather than regenerating
an FCS downstream. Other retained ordinary files were checked byte-for-byte
against the upstream commit above during the 2026-09-19 publication audit.

`rtl/icmp_echo_responder.sv` is the project's fixed-address board bring-up
endpoint, not a Taxi upstream file.

The SoC's actual source closure is expanded by `tools/taxi_filelist.py` and
by the corresponding reader in `synth/vivado.tcl`. Removing the board examples
did not change any path or content in that expanded list. Re-run the expander
and `make lint-eth-link` whenever changing this subset; do not flatten its
relative `lib/taxi` symlinks or delete a file merely because the top-level
RTL does not name it directly.
