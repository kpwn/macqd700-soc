# Multi-hot-strobe peripheral writes — the byte-granular write serializer

Date: 2026-08-18
RTL: `rtl/soc/peripheral_bus.v`
TB: `tb/tb_peripheral_bus.cpp` (`make tb-peripheral-bus`)

Fixes the byte-granular half of a long-standing defect: a write beat whose
active-lane `WSTRB` has more than one bit hot delivered **one** byte to the
peripheral and silently dropped the rest.

Evidence that this is live, not latent:
`docs/mame_periph_multihot_reachability.md` — a MAME `macquadra700` capture
under System 7.0.1 **and** 7.5.3, two independent instruments (memory taps +
CPU-side debugger watchpoints), 1,695,109 classified peripheral writes, three
positive controls. Design context: `2026-08-18-v1-shared-infra-fixes-design.md`
§2.3–§2.4 in the sibling `m68k-core-040-ooo` repo ("Fix A").

---

## 1. The defect

`peripheral_bus.v` presents an 8-bit face to every Mac peripheral. For a write
beat it picks one byte out of the active 32-bit lane:

```verilog
wire [7:0] wr_strb_byte =
    wr_lane_strb[0] ? wr_lane_data[ 7: 0] :
    wr_lane_strb[1] ? wr_lane_data[15: 8] :
    wr_lane_strb[2] ? wr_lane_data[23:16] :
                      wr_lane_data[31:24];
wire [7:0] wr_byte =
    wr_lane_strb_onehot ? (wr_addr_byte | wr_strb_byte) : wr_strb_byte;
```

and `pb_wr_active` pulses `pb_wr` **once** per AXI beat. With more than one
strobe hot the priority chain keeps exactly one byte — and because the LSU
places the first big-endian byte at the *highest* hot strobe bit, the one it
keeps is the **last** byte of the store.

Three slots already had their own serializer and were unaffected: ASC
(`wr_asc_*`), the SCSI pseudo-DMA shim (`wr_scsi_*`), and SONIC's 16-bit word
path (`wr_sonic_*`). Everything else took the single pulse.

## 2. What the capture actually found

| slot | multi-hot writes / boot | producer |
|---|---:|---|
| `ORWELL` | 182 LONG (91 per OS) | ROM init loop `move.l D4,(A2)+` @ `0x40804872`, `0x4084BB2E`, `0x4084BD48` |
| `SONIC` | 8 LONG (4 per OS) | System 7 Ethernet driver reset path, RAM PC `0x000054EE` (`moveq #4,D0 / move.l D0,(A2)`) |
| `ASC` | 3,984 LONG | 7.5.3 Sound Manager → FIFO A/B (already serialized) |
| `SCSI` DMA shim | 41,939 WORD | ROM + OS blind-transfer loops (already serialized) |
| `VIA1`, `VIA2`, `IWM`, `SCSI` regs, `SCC` | **0** of 494,785 writes | — |
| `ENET`, `ADBINJ` | no traffic at all | — |

So the two slots that needed fixing were **ORWELL** and **SONIC**, and neither
is hypothetical.

**SONIC is the one that was actually corrupting data.** Its existing word
serializer is entered only on `wr_size_q == 3'd1 && !wr_addr_q[0]`. The
driver's beat is a **LONG**, so it missed that predicate entirely, fell through
to the generic single pulse, and delivered 1 of 4 bytes to a live 16-bit part's
register page.  The Q700 connects the SONIC to the low 16 bits of each 32-bit
bus word: `q700_eth_sonic.v` decodes `sonic_addr[7:2]` as the register index,
`sonic_addr[1]` as the connected-lane select, and `sonic_addr[0]` as the byte
half.  The serializer is still required for LONG stores: it emits all four
bytes, of which the SONIC acknowledges and ignores the upper two before
accepting the low two.

ORWELL's downstream is `orwell_stub.v`, which acknowledges and discards every
write, so the behavioural delta there is nil *today* — but it becomes
load-bearing the moment the stub grows state.

## 3. The fix

> Update (2026-08-21): SONIC no longer participates in this serializer.
> Its peripheral face is now native 16-bit (`reg`, `wdata`, `wstrb`), so a
> byte, word, or long CPU access produces one SONIC register transaction.
> The serializer described below remains applicable to ASC and ORWELL; the
> SONIC details are retained as historical context for the bug it replaced.

Generalize the proven `wr_asc_*` FSM into a slot-parametric `wr_ser_*` FSM.
No new mechanism was invented; the ASC machine was already the right one, it
was just hard-wired to one slot.

* `slot_serializes(slot)` — the membership predicate, currently
  `{SLOT_ASC, SLOT_ORWELL, SLOT_SONIC}`.
* `wr_sonic_word_path` — SONIC's pre-existing `size==WORD && !addr[0]`
  predicate, hoisted to a named wire and used both to select the word path and
  to keep `wr_ser_slot` from shadowing it. The generic walk therefore catches
  exactly the residual (any other multi-hot SONIC beat) and nothing else.
* `wr_ser_ack` / `wr_ser_wr` — per-slot muxes over `{asc,orwell,sonic}_{ack,wr}`,
  the same shape the `wr_b_done` case already builds.
* Each participating slot's `*_addr` mux gains a
  `wr_ser_on_<slot> ? {<upper addr bits>, wr_ser_addr_lsb}` term, its `*_wdata`
  mux a `wr_ser_byte` term, and its `*_wr` an extra `wr_ser_pulse_<slot>` term.
* The HIGH→LOW strobe walk is preserved verbatim — it is derived at
  `peripheral_bus.v:568-577` from the LSU's `m68k_mem_strb` / `m68k_mem_wdata`
  layout, so walking high→low emits `AW, AW+1, AW+2, …` in memory order.

### 3.1 The one place the ASC pattern did NOT generalize unchanged

ASC **registers** its `pb_ack`; ORWELL and SONIC ack **combinationally**
(`assign ack = cs && (rd || wr)`). The historical FSM pulses in phase 0 and
waits for the ack in phase 1 — by which time a combinational slave's ack is
already gone. Wired up as-is, an ORWELL/SONIC burst delivered its first byte
and then sat until the `PB_ACK_TIMEOUT` watchdog returned SLVERR. This was
caught by the new tests, not by inspection.

Fixed with a one-bit latch, `wr_ser_ack_seen_q`: phase 0 captures
`wr_ser_ack`, phase 1 retires the byte on `wr_ser_ack || wr_ser_ack_seen_q`.
Both device styles consume the byte at the pulse cycle's posedge, so latching
the ack — rather than *holding* `pb_wr` until it arrives — is also what keeps
`pb_wr` a strict one-cycle pulse with a low cycle between bytes. Holding it
would double-write any device that samples `cs && wr` every posedge, which is
all of them.

For ASC the latch is provably a no-op: a registered ack cannot be high in the
same cycle as its own pulse, so `wr_ser_ack_seen_q` is always loaded with 0 and
phase 1 behaves exactly as before.

### 3.2 Cost

Two cycles per byte, unchanged from the ASC path. An ORWELL LONG is 8 cycles
instead of ~2; at 91 LONG writes per boot that is under a thousand extra cycles
across an entire ROM init. SONIC takes 4 such beats per boot.

## 4. What was NOT changed, and why that is deliberate

### 4.1 The strided/aliased family — VIA1, VIA2, IWM, SCSI register page

Their device-local address is `wr_addr_q[12:9]` (0x200 stride) or
`{5'b0, addr[7:4]}` (0x10 stride). `AWADDR[1:0]` is a **don't-care**, so
"increment the byte address and pulse again" resolves to pulsing the *same*
stateful register N times — VIA `IFR`/`IER`/`SR` or a timer latch written
twice, the SWIM phase-line latches driven twice. That is strictly worse than
today's single pulse.

Their real defect is different: not a dropped byte but the *wrong* byte (the
priority chain keeps the last byte of the store, not the byte `AWADDR` names).
The capture found **zero** multi-hot beats on any of them across 436,279 writes
and 13/16, 10/16, 7/16 and 11/16 registers respectively, so the defect is
latent. It is left open on purpose rather than fixed speculatively.

### 4.2 SCC

`scc_addr = {addr[5:4], addr[1], addr[2]}` — `addr[0]` is absent, `addr[1:2]`
are present, so it is in neither family. Serializing a WORD write would pulse
the *same* SCC register twice, and the Z85C30's register pointer is shared
across both channels; this repository already has that corruption mechanism
characterised as a known-red finding (`tb_peripheral_bus.cpp`'s
`test_KNOWNRED_scc_altbase_steals_swim_reg0`, `scc.v:438,732-745`). 58,506 SCC
writes were observed, all single-byte, in both OS versions including one with
AppleTalk active. Untouched.

### 4.3 ADBINJ and ENET — byte-granular, still deliberately excluded

This is the non-obvious one, and it is why "serialize every byte-granular slot"
would have been a live regression.

A multi-hot beat only *means* "N bytes" under the m68k LSU's convention.
`peripheral_bus.v:614-623` documents a **second** live convention: host/debug
masters put one meaningful byte in the strobed lane with a **word-aligned**
`AWADDR` and all four strobes hot. `ADBINJ` is written exclusively by the
JTAG-AXI master using exactly that shape — `axi_write(0x0011010, 0x0000001C,
strb=0xF)` injects keycode `0x1C` into `KBD_ENQUEUE` at `+0x10`. Serializing it
would put `0x00` at `+0x10`, `+0x11`, `+0x12` and the keycode at `+0x13`,
breaking ADB injection on hardware. `test_adbinj_decode` failed the moment
ADBINJ was added to the family; that is how this was found.

`ENET` is the Ethernet address PROM window: zero writes of any width in any
run, so there is nothing to serve and the same ambiguity would apply.

Both are locked by dedicated tests
(`test_adbinj_full_strobe_word_not_serialized`,
`test_enet_multi_byte_not_serialized`) so a future widening of
`slot_serializes()` has to argue with a failing test rather than a comment.

### 4.4 The read path

Untouched. The read side's one-`pb_rd`-pulse-plus-byte-broadcast behaviour is a
separate, already-characterised issue.

## 5. Blast radius

`wr_lane_strb_onehot` is untouched and the serializer only engages when
`wr_lane_strb_count > 1`, so **every single-byte write produces a
bit-identical, cycle-identical result**. The single-byte termination condition
inside the new branch was deliberately written to be the *same expression* as
the generic arm's — `(ack && wr) || (wr_w_done && ack)` — so ORWELL/SONIC
single-byte writes still complete in the W-capture cycle (combinational ack)
and ASC still completes a cycle later (registered ack), exactly as before.

Since every access that boots System 7 today is either single-byte or already
handled by one of the three existing serializers, this change **cannot alter an
access that currently works**. It can only alter accesses that were already
wrong.

Known limitation, inherited verbatim from the ASC serializer and not introduced
here: `wr_ser_addr_lsb` is a 2-bit add, so a malformed beat whose
`AWADDR[1:0] + hot-strobe-count` exceeds 4 wraps within the 32-bit word instead
of carrying. Well-formed accesses (2-byte at an even offset, 4-byte at a
4-aligned offset) never reach that case.

## 6. Verification

`make tb-peripheral-bus` — **55/55 scenarios pass**.

### 6.1 Fails-then-passes pair

The six new positive scenarios were run against the **unmodified**
`peripheral_bus.v` from HEAD `15e4650` with the new tb in place:

```
FAIL orwell long write count:           got 0x1, expected 0x4
FAIL orwell word write count:           got 0x1, expected 0x2
FAIL orwell burst write count:          got 0x1, expected 0x4
FAIL sonic long write count:            got 0x1, expected 0x4
FAIL sonic long distinct write count:   got 0x1, expected 0x4
FAIL sonic burst write count:           got 0x1, expected 0x4
49/55 scenarios passed.
```

`got 0x1, expected 0x4` **is** the bug: one byte delivered, three dropped.
With the fix: `55/55 scenarios passed.`

Note which scenarios were **already green** before the RTL change —
`test_sonic_word_path_not_shadowed_by_serializer`,
`test_adbinj_full_strobe_word_not_serialized`,
`test_enet_multi_byte_not_serialized`,
`test_strided_slots_multi_hot_stay_single_pulse`, and all 45 pre-existing
scenarios. They are non-regression locks, and they are green on both sides.

### 6.2 New scenarios

| scenario | what it pins |
|---|---|
| `test_orwell_multi_byte_long_serializes` | the ROM's real beat (`0x124F0810`, LONG, strb `1111`) → 4 writes at `+0..+3`, big-endian order |
| `test_orwell_multi_byte_word_off2_serializes` | WORD at an odd-word offset → 2 writes |
| `test_orwell_multi_byte_followup_single_byte` | FSM state-leak lock |
| `test_sonic_driver_reset_long_write_serializes` | byte-for-byte the captured driver beat (`0x50F0A000`, LONG, data `0x00000004`) |
| `test_sonic_multi_byte_long_distinct_bytes` | four distinguishable bytes, so a byte at the wrong address cannot pass by coincidence |
| `test_sonic_word_path_not_shadowed_by_serializer` | the one non-mechanical part: `AWSIZE=WORD && !addr[0]` still takes the *word* path. Told apart by construction — the word path reads `lane_data[31:16]` and ignores WSTRB, the generic walk would follow the strobes into `lane_data[15:0]` |
| `test_sonic_multi_byte_followup_single_byte` | FSM state-leak lock |
| `test_adbinj_full_strobe_word_not_serialized` | §4.3 — the host-convention exclusion |
| `test_enet_multi_byte_not_serialized` | §4.3 |
| `test_strided_slots_multi_hot_stay_single_pulse` | §4.1/§4.2 — VIA1, VIA2, IWM, SCSI regs, SCC each take exactly one pulse |

### 6.3 Adjacent testbenches

`make tb-pb-scsi` 6/6, `make tb-adb-inject` 11/11 (both instantiate a real
`peripheral_bus.v`).

Full `make tb-all`: **PASS=76 XFAIL=5 FAIL=1 (of 82)**. Every testbench that
instantiates `peripheral_bus.v` is green — `tb-peripheral-bus` 55/55,
`tb-pb-scsi`, `tb-adb-inject`, `tb-turboscsi`, all six `tb-scsi-c96-*`,
`tb-scsi-dual`, `tb-scsi-trace-ring` — as are the peripheral-family unit tbs
`tb-via1`, `tb-via2`, `tb-scc`, `tb-asc`, `tb-iwm`, `tb-q700-eth-sonic`,
`tb-adb`, `tb-glue`, `tb-irq-agg`, `tb-rtc`. The 5 XFAILs are the documented
`TB_KNOWN_BROKEN` entries.

The one FAIL is **`tb-vram-ddr-chain-nol2c` (14 PASS / 3 FAIL)** and is
**structurally unrelated**: its `VRAM_DDR_CHAIN_RTL` source list
(`Makefile:5889`) does not contain `peripheral_bus.v` at all, and its three
failing scenarios are L2C/scanout ones (`scanout_bounded_under_l2_saturation`,
`cold_fill_scoreboard_under_scanout`, `reset_mid_l2_victim_writeback`). It is
pre-existing and not yet listed in `TB_KNOWN_BROKEN`.

### 6.4 Two tb-model changes, both narrow

* `axi_write_multi()` gained an optional `awsize` argument, defaulting to `4` —
  every historical call site is byte-identical. It is needed because SONIC's
  word serializer is the only thing in the file that keys on `AWSIZE`, and the
  new tests must drive both a real LONG (`awsize=2`) and a real WORD
  (`awsize=1`).
* The SONIC pb-write sampler now logs on a rising edge **or** an address change
  while `pb_wr` is held. SONIC's word path is the one place `pb_wr` stays high
  across two cycles at two different addresses; `q700_eth_sonic.v` samples
  `cs && wr` every posedge, so that really is two device writes and a plain
  rising-edge detector under-counted it. Slots that hold `pb_wr` at a constant
  address across a stall still log exactly once.

## 7. Still required before this can be called fully verified

1. **On-hardware boot to Finder.** This is RTL on the live peripheral write
   path of a machine that boots System 7.0.1 on real hardware today. The design
   spec's pre-committed consequence table makes a hardware gate
   **mandatory-blocking** for exactly this outcome (a hit on a byte-granular
   slot). Not done here.
2. **A synthesis / implementation gate.** Not run: the Vivado mutex
   (`/var/tmp/m68k-ooo-vivado.lock`) was held by a live JTAG session at the time
   of the change, and `CLAUDE.md`'s sim-first policy forbids block-waiting on it
   or running a second Vivado machine-wide. The change is small combinational
   logic (three 3-way muxes plus one flop) but that is an argument, not a
   measurement.
3. **The full-RTL ROM-boot canary (M2).** The generalized `$display` should now
   fire on ORWELL during a full-RTL ROM boot — the reachability measurement
   predicts ORWELL only, since ASC's and SONIC's multi-hot producers are
   OS-resident code the RTL ROM boot never reaches. Running it would be an
   independent confirmation that the RTL and MAME agree about which slot takes
   multi-hot traffic.
4. **The strided-family fix ("Fix B").** Still open by design (§4.1). It changes
   *which byte a stateful device receives*, is latent on this evidence, and
   deserves its own hardware gate.

## 8. Where this departs from the design spec, and why

`2026-08-18-v1-shared-infra-fixes-design.md` **SI4** defines Fix A's family as
`{ASC, ENET, ORWELL, ADBINJ, SONIC}`. This implementation lands
`{ASC, ORWELL, SONIC}` and excludes ENET and ADBINJ (§4.3).

The spec's own §2.5 identified the host/debug byte convention and built a
fallback arm around it — but only for Fix B, on the assumption that the
byte-granular family had no host/debug producer. ADBINJ does, exclusively, and
serializing it breaks ADB injection on hardware. The spec should be amended:
the discriminator is not "is `AWADDR[1:0]` in the decode?" alone, it is that
**plus** "does this slot have a producer whose multi-hot beats mean N bytes?".

Two of the spec's other locks were honoured as written: **SI9** (the SONIC-word
and SCSI-DMA-shim serializers are untouched and unfolded) and **SI7** (the
`wr_lane_strb_onehot` arm of `wr_byte` is not modified, so no currently-working
access changes).

## 9. Files touched

```
rtl/soc/peripheral_bus.v        wr_asc_* → slot-parametric wr_ser_*, + wr_ser_ack_seen_q
tb/tb_peripheral_bus.cpp        10 new scenarios, awsize arg, SONIC sampler, comment correction
docs/endianness_audit.md        B4 status note (half fixed; its worked example was wrong)
docs/periph_multihot_write_serializer.md   this file
```
