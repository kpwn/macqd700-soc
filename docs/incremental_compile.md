# Vivado incremental compile

`make impl` and `make synth` consume an optional reference routed-DCP.
When present, opt_design / place_design / route_design reuse the prior
placement and routing for hierarchy that hasn't changed.  Typical
savings on peripheral-only edits (e.g. `rtl/mac/iwm_stub.v` patches):
**40–60 % impl wall-time**.  After every successful route, the new
`route.dcp` is copied back to the reference path so the next iteration
keeps benefiting.

## Knobs

| Variable | Default | Effect |
|---|---|---|
| `INCREMENTAL_REF_DCP` | `$(BUILD_DIR)/vivado_incremental_ref/route.dcp` | Where the rolling reference DCP lives.  First run finds nothing here, runs full, stashes the result.  Subsequent runs consume it. |
| `NO_INCREMENTAL` | (unset) | Set `=1` to force a full impl that *neither* consumes *nor* refreshes the stash.  Use after a major top-level reshape or when bisecting. |

## Workflow

```bash
# First impl — full run, stashes the routed DCP.
make impl

# Edit a peripheral (e.g. rtl/mac/iwm_stub.v).  Validate in sim first.

# Second impl — opt_design reads the stash, reuses ~95% of placement.
make impl

# See how much was reused last time.
make incremental-status

# Force a clean run (e.g. after rename/IO/clocking change).
make clean-incremental impl     # or: NO_INCREMENTAL=1 make impl
```

## When to drop the reference

- Top-level port list changes (clocking, MIG, PCIe).
- Pblock or floorplan constraint changes.
- WNS regresses sharply with no obvious RTL cause — Vivado is forcing
  a stale placement that no longer routes well.
- After a Vivado version bump.

In any of those, run `make clean-incremental impl` once and let the
stash repopulate.

## What gets reused

Vivado matches by RTL hierarchy + cell/net fingerprint.  Unchanged
modules keep their placement; changed modules + their fanin/fanout
re-place.  `report_incremental_reuse` (written to
`$(VIVADO_IMPL_DIR)/reports/incremental_reuse.rpt` on every routed
build with a reference) shows the per-cell-type reuse breakdown.

## Caveats

- Reference DCPs are large (hundreds of MB).  Don't commit them.
- The stash lives under `BUILD_DIR` and gets wiped by anything that
  removes that directory (e.g. a manual `rm -rf`).  That's fine — it
  rebuilds itself on the next full run.
- The Vivado mutex (`/var/tmp/m68k-ooo-vivado.lock`) still applies —
  incremental compile doesn't change concurrency rules.
