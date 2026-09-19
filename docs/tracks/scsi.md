# SCSI Track

> Read `docs/agent_policy.md` first. This brief narrows the SCSI-only
> work in `rtl/mac/scsi.v` and `tb/tb_scsi.cpp`.

## Scope

NCR 5380 register semantics, selection/REQ-ACK phase behavior, IRQ/DRQ
visibility, reset behavior, and the SD-backed block-device hook used by
the Mac ROM bring-up path.

## Files you own

```
rtl/mac/scsi.v
rtl/sys/sd_scsi_lba_mapper.v
tb/tb_scsi.cpp
tb/tb_sd_scsi_lba_mapper.cpp
docs/tracks/scsi.md
```

## Current behavior

- Q700/MAME comparison point:
  - MAME maps the Quadra 700 boot path through the DAFB TurboSCSI
    NCR53C96 register window plus Apple's pseudo-DMA handshake.
  - `rtl/mac/scsi.v` remains a single-target NCR-5380-like target shim,
    so it does not yet model NCR53C96 arbitration, FIFO, sequencer, or
    pseudo-DMA timing.
  - The minimal ROM-polling readiness gap is observability first:
    selection command writes, command/FIFO CDB capture, status polling,
    interrupt polling, and raw-block read/write intent must be visible in
    the ROM harness before replacing the stub with a production 53C96 path.
- Register readback is live, not probe-only:
  - reg 4 reflects reset, BSY, REQ, MSG, C/D, I/O, SEL, DBP.
  - reg 5 reflects IRQ, DRQ, the completion latch, ATN, ACK, and the
    busy-error latch.
- Selection is deterministic:
  - ID 0 selects the emulated disk.
  - any other ID stays BUS_FREE.
- REQ/ACK is phase-driven through command, data, status, and message-in.
- DRQ is live only while a data-in/data-out byte is ready under REQ; it
  drops before status/message phases and the `drq` output matches reg5[6].
- Pseudo-DMA data-in polling may read through reg 6 (`Input Data`) while
  watching reg5[6]; the shim still uses the explicit REQ/ACK phase
  handshake and does not model a NCR53C96 FIFO or DMA residue counter.
- Reg7 Reset Interrupt and bus RST clear pending IRQ/completion/busy-error
  state and any observed DMA trigger writes, so later ROM polls cannot see
  stale pseudo-DMA residue.
- IRQ is level-sensitive and clears on reg 7 read.
- STATUS and MSG_IN assert the IRQ bit/wire in the same visible phase as
  REQ, so ROM phase polling cannot observe a ready status/message byte
  before the interrupt source is visible.
- STATUS and MSG_IN also make reg5[7] END_DMA visible immediately,
  including zero-length READ(10)/WRITE(10), CHECK CONDITION, and
  pseudo-DMA completion tails.
- CHECK CONDITION sense details persist across reg7 interrupt clears and
  unrelated successful commands until REQUEST SENSE consumes them.
- A missing block-device back-end now times out into CHECK CONDITION
  instead of hanging in the SD wait state.
- Verilator builds print selection/ignored-selection events, full CDB
  bytes, decoded READ/WRITE LBA/block counts, biased SD LBA, SD request,
  reg 4 / reg 5 / reg 7 poll activity, IRQ transitions, and
  out-of-range / timeout / error events so ROM bring-up can see what the
  disk side is doing and prove the ROM is polling phases.
- Verilator builds also print one line per visible SCSI phase transition:
  selection, command, data-in/out, status, message-in, and disconnect.
- The ROM boot harness `+periph_event_filter=SCSI` path records low-noise
  TurboSCSI events for:
  - `selection_phase`
  - `command_phase`
  - `cdb_byte` / `cdb`
  - `raw_block_read` / `raw_block_write` with SCSI LBA, block count,
    biased SD LBA, byte offset, DRQ, and IRQ state
  - `status_phase`
  - `interrupt_phase`
  - `sequence_phase`
  These are observation hooks only; they do not claim the harness stub is a
  complete NCR53C96 implementation, but the harness now keeps a small
  phase-aware register stub so the ROM can observe deterministic status,
  interrupt, and DMA-poll readback instead of a flat zero response.

## Raw SD backing contract

- The SD card is treated as a raw block device, not a filesystem.
- The SCSI target presents a single raw direct-access disk at target ID 0;
  there is no partition parsing, filesystem awareness, removable-media
  model, or separate media-present input.
- The first 4 MiB are reserved for ROM, boot, and provisioning assets:
  - 4 MiB / 512 B = 8192 SD sectors.
  - SCSI disk LBA 0 maps to SD LBA 8192.
  - SCSI disk LBA N maps to SD LBA `8192 + N`.
- `sd_scsi_lba_mapper.v` is the standalone combinational contract point:
  it rejects zero-block requests, exposed-disk capacity overflow, start
  mapped-LBA overflow, and physical end-of-transfer overflow past the
  32-bit SD LBA space.
- `scsi.v` fails closed if mapping is invalid or if the SD back-end is
  absent:
  - invalid LBA range returns CHECK CONDITION with ILLEGAL REQUEST /
    LBA out of range sense.
  - no SD response returns CHECK CONDITION with NOT READY / medium not
    present sense, sets the busy-error latch, and makes later TEST UNIT
    READY / READ CAPACITY probes report not-ready until a successful raw
    block access or reset.
  - `sd_error` returns CHECK CONDITION with medium/read/write sense.
  - zero-length READ(10) / WRITE(10) complete as GOOD no-ops without
    touching the SD back-end, with STATUS/IRQ/END_DMA visible on the first
    completion poll.

## Tests

`tb/tb_scsi.cpp` covers:

- reset defaults and register decode
- ATN/ACK reflection in Bus-and-Status
- selection of ID 0 and ignored non-zero IDs
- INQUIRY
- READ(6) at the raw-disk frontier
- READ(6) / WRITE(6) transfer length 0 as the legacy 256-block form
- READ(10) at a non-zero LBA
- READ(6) with no backing store present
- TEST UNIT READY after a known no-media timeout
- same-phase STATUS IRQ visibility for ROM reg4/reg5 polling
- CHECK CONDITION sense persistence across reg7 clear and an unrelated
  successful INQUIRY, plus REQUEST SENSE clear-on-read
- READ(10) past the exposed capacity
- READ(10) with zero transfer length
- WRITE(10) past the exposed capacity
- WRITE(10) with zero transfer length
- WRITE(6)
- WRITE(10) at a non-zero LBA
- READ(10) medium error path
- pseudo-DMA-style data-in through reg 6 with reg5[DRQ] polling
- reset during selection
- IRQ persistence and clear on reg 7 read
- Bus-and-Status completion-latch visibility and clear on reg 7 read
- Bus-and-Status completion-latch visibility for CHECK CONDITION exits
- immediate STATUS/IRQ/END_DMA visibility for zero-length READ(10)
- multi-block READ(10) request sequencing, including consecutive biased
  raw SD LBAs
- WRITE(6) missing-media timeout, busy-error latch, and NOT READY sense
- raw SCSI-to-SD LBA mapper offset, last valid LBA, overrun invalidation,
  and zero-block invalidation
- ROM harness SCSI event logging through `tb-rom-boot-scsi-smoke`, including
  selection/command/status/interrupt event names in the summary

## Remaining integration gap

The top-level still needs a real raw-block-device hookup for shipping
hardware. The SCSI side now fails closed when that back-end is absent,
which keeps ROM bring-up deterministic instead of hanging.
