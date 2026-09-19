# Fetch guard response-arbitration correction

The local DECERR response must not replace an already-presented downstream
response, nor split a downstream burst. `m_burst_open` tracks accepted beats;
it does not cover a first beat which is stalled or accepted on the same edge
as a faulting AR. The old G_IDLE and G_PEND transitions could therefore grant
the local response while a downstream first beat was already presented.

Both transitions now require that no downstream beat is presented before
granting the local response (and no burst is open). Normal in-window AR/R
traffic remains transparent. Faulting requests can wait an extra cycle for
the downstream channel to become idle. The change implements the existing
ordering contract; it does not disable speculation, narrow fetch bursts, or
change normal fetch throughput.

Validation, 2026-09-18:

- Updated `make tb-ifetch-window-guard`: **45 checks pass, 0 fail**.
  `/tmp/codex-ifetch-guard-boundaries.log`.
- The same test with original d02c3f11 RTL: **37 pass, 8 fail**.
  `/tmp/codex-ifetch-guard-boundaries-original.log`.
- New boundary matrix covers a first beat stalled at fault AR acceptance,
  a first beat accepted simultaneously with fault AR, and the first beat
  of another previously accepted burst arriving while G_PEND is active.
  It checks complete response order/count and all 256 payload bits under
  backpressure, sampling handshakes before the active edge and advancing
  the downstream model only when its own RREADY is asserted.

## Relevance to the boot investigation

This is a real guard defect, **not yet a demonstrated cause of the hardware
boot failure**. Current IcachePlugin.scala sets RREADY for every recognized
fetch ID: `demandRspMatch || pfRspMatch || staleDrain` simplifies to the legal
ID ranges. Thus normal legal responses are not subject to CPU backpressure;
the stalled-response failure is not presently shown reachable on the board.
The simultaneous-handshake case does interleave different-ID responses in
the old guard, violating the guard's documented stronger burst policy, but
the CPU routes by RID, so that alone does not establish corruption.

The guard's `ifg_fault_sticky/addr/count` outputs terminate in unconsumed
top-level wires, despite the module's observability comment. They cannot
currently be read through JTAG; no hardware attribution was inferred from
that missing observation. Read-only halt status confirmed that the saved
board breakpoint remains 4080e2a6, reason 5, exception count 0086a0a5.

This correction was developed in the isolated `codex-fetch-guard` worktree.
The ongoing full 200 MHz build in `codex-200-timing/build/vivado200_sonic_pipe`
uses d02c3f11 and DOES NOT include it. Keep these artifact identities distinct.
