# DDR padding control-path simplification

The September 18 full-SoC 200 MHz route has a separate 333 MHz MIG UI
violation: `aw_stage_data_reg[20]` to `wr_pad_remaining_reg[*]/CE`,
WNS -0.125 ns. The reported path traverses `wr_real_need_push` and the
padding comparison. The later fanout pass does not eliminate this domain's
failures. This is timing work, not an attributed cause of the boot fault.

`wr_pad_trigger` requires `!wr_wlast_ok`; `wr_real_need_push` requires
`wr_wlast_ok`. Consequently, whenever padding can trigger:

```
wr_real_need_push = 0
wr_pushed_total_after = wr_mig_pushed_count
```

The candidate uses the existing pushed-count register directly in the
padding comparator and subtraction. The old expression remains unchanged
for real-data pushes and final-beat tagging. The padding count is loaded
only under the padding trigger. No latency, handshake, queue capacity,
throughput, reset behavior, or malformed-burst policy changes.

Simulation-only assertions compare the new trigger with the original
expression on every non-reset clock and compare the count on trigger.

Validation so far:

- Exhaustive algebra check: 512 x 512 count pairs x 32 combinations of
  fire/terminate/lastOk/pushOther/committed = 8,388,608 cases; all equivalent.
  Addition and subtraction modeled modulo 512. This is NOT RTL simulation.
- `make tb-axi-ddr4-mig-bridge` with assertions: all 46 scenarios passed,
  including malformed-burst handling, same-cycle AW/W bypass and 10,000 mixed
  concurrent operations (4,753 writes, 5,247 reads). No equivalence assertion
  fired. Log: `/tmp/codex-ddr-padding-regression.log` (September 18).
- Linked CPU required fast gate: 372 passed, zero failed, two ignored.
  Log: `/tmp/codex-ddr-padding-fast-gate.log`.
- Required next: integrated routed setup/hold/skew checks. No timing gain
  is claimed until measured, and no bitstream contains this candidate yet.
