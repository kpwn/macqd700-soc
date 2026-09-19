# Diag: vec-2 bus-error frame, A6/A7 corruption (live FPGA)

## Live-state recap

```
break-PC  = 0xEBD20004           (after garbage JMP)
exc_pc    = 0x408046AA           (= TST.B at 0x4084_46AA in ROM)
exc_vec   = 2                    (bus error)
A6        = 0x40803178           (currently — was 0xEBD20000 at moment of JMP)
A7        = 0x001FDD6B           (ODD)
A5        = 0x001FDDA7  (= VBR)  (also ODD)
SR        = 0x2700, VBR = 0x001FDDA7
D7        = 0x08000000           (Sad Mac fatal-bit set)
```

Stack near A7 (long-aligned reads):

```
0x001FDD60: 40804080
0x001FDD64: 9A0A001F
0x001FDD68: DE424027
0x001FDD6C: 10408046
0x001FDD70: AA700851   ← bytes A7+6..A7+9 = 70 08 51 00 ...
0x001FDD74: 001C0005
```

## Frame-format analysis (this is the KEY finding)

Decoded byte-by-byte starting at A7=0x001FDD6B:

| stk-addr | bytes | meaning                          |
|----------|-------|----------------------------------|
| DD6B,DD6C | 27 10 | **SR = 0x2710** (S=1, IPL=7)    |
| DD6D..DD70 | 40 80 46 AA | **PC = 0x408046AA** (matches exc_pc) |
| DD71,DD72 | 70 08 | **format/vec = 0x7008 → fmt=7, vec=2**  |
| DD73..DD76 | 51 00 1C 00 | fault_addr = 0x51001C00 (unmapped I/O) |
| DD77,DD78 | 05 ?? | SSW (low half visible)       |

**The frame format itself is CORRECT.** Format-7 (60-byte access-error
frame) with the right SR / PC / format-vec / EA layout for vec-2 per
68040 PRM §8.4.1. Big-endian byte order is correct. Frame-size bookkeeping
and post-push A7 (= pre-A7 − 60) are consistent with what `exception.v`
and `exception_uop_gen.vh` produce.

What we push:
- `rtl/core/exception.v:537-595` selects fmt-7 for vec 2/3; `frame_sz=60`,
  `push_word_count=30`.
- `rtl/core/exception_uop_gen.vh:60-86` generates push EA = `i_a7_new + idx*2`.
- `frame_word_data` (exception.v:371-404 + exception_uop_gen.vh:97-147) puts
  SR@0, PC@2-5, fmt/vec@6-7, fault_addr@8-11, SSW@12-13.

That all matches the bytes observed. **The frame push is not the bug.**

## Root cause: A7 was already odd BEFORE the fault

The frame top is **A7-pre = 0x001FDD6B + 60 = 0x001FDDA7** — i.e. A7 was
**already odd** when the bus error fired. Since `0x001FDDA7 == VBR == A5`,
the odd-A7 trace is:

1. Earlier (handler-stage) code did `MOVE.L A5,A7` at 0x40846A86 with
   A5 = 0x001FDDA7 (= VBR), making A7 odd.
2. `JMP (A6)` jumps to 0xEBD20000 (corrupt A6).
3. Instruction fetch at 0xEBD20000 takes a (second) bus error → vec=2.
4. Sequencer pushes the 60-byte fmt-7 frame on the now-odd A7 →
   A7-post = 0x001FDD6B (still odd, but the frame bytes themselves are
   laid out big-endian-correctly).

So A7 oddness propagates from VBR/A5 oddness via the redirector's
`MOVE.L A5,A7`. The frame layout itself is fine.

## Why VBR is 0x001FDDA7 (odd)

68040 architecturally **does not mask VBR low bits** on `MOVEC Rn,VBR`
— Musashi confirms (`tb/models/musashi/m68k_in.c:6874`,
`m68kcpu.c:723`: `REG_VBR = MASK_OUT_ABOVE_32(value)` — full 32 bits).
Software must align VBR. We have a regression test
(`tb/tests/asm/unaligned_long_vector_dispatch.s`) that explicitly
exercises an UNALIGNED VBR (= 0x0001FECE) — this is intentional Q700
behaviour.

`rtl/core/commit.v:3177` (`arch_vbr <= rob_src_a_val;`) is correct as-is.
DO NOT mask the low byte — that would regress the unaligned-VBR test
and diverge from MAME/Musashi.

So **VBR=0x001FDDA7 is whatever the ROM wrote**. The Q700 ROM normally
relocates VBR to 0x001FECE (per our test). 0x001FDDA7 is different —
either a different boot path or **ROM wrote VBR with a corrupted
source register**. The source-side corruption (= an `An` with a junk
low byte) is upstream of the MOVEC.

## A6 corruption (= 0xEBD20000 at moment of JMP)

A6 was loaded earlier in the handler (likely a MOVEM.L pop or a
RAM-resident vector-dispatch table near 0x40846A60..7E which contains
ROM addresses 0x40846BDC, 0x40846BE6, 0x40846BF0, 0x40846AC6×5). None
of those ROM addresses match 0xEBD20000, so A6 is being loaded from
**RAM** that the ROM expected to contain a valid handler pointer.

0xEBD20000 has the bit pattern of a misaligned-LONG read across an
RAM/peripheral boundary, OR a PRF readback for an A-reg that was
renamed but written by a junk physical reg. Cannot be discriminated
from the JTAG snapshot alone.

## Fix proposal — none surgical, this needs sim repro

The frame-format hypothesis is **disproven** by the byte-level decode:
SR/PC/format-vec/EA are at the right offsets, big-endian, fmt=7 nibble
correct. So no `commit.v` / `exception.v` / `exception_uop_gen.vh`
change is justified.

The remaining possibilities are upstream of the bus-error frame push:

1. **Source register feeding `MOVEC Rn,VBR` had a corrupted low byte.**
   Find which MOVEC site (out of the 21 sites in ROM) ran, what register
   was its source, and what wrote the low byte.

2. **A5/A6 corruption from an earlier load.** Track where the handler
   loads A5/A6 (likely `MOVEM.L (sp)+,...` or `MOVE.L (An),A6`); check
   whether the load address was misaligned and whether our split-LONG
   load reconstructs the right bytes.

3. **Renamer/PRF drop**: an A-reg destination wrote junk (= stale PRF
   value) because the producer didn't actually retire the right value.

## Recommended next step

Use `m68k-mame-state-replay` to snap MAME at PC=0x408046AA during the
Sad Mac path, load that snapshot into Verilator, run forward, and
compare the post-vec-2 sequence step-by-step with MAME. The hardware
JTAG snapshot is too late — A6 has already been re-loaded post-JMP.
A sim run lets us see the *first* moment A6 differs from MAME, which
will pin down whether the bug is:

- (a) MOVEC source-data corruption (VBR low byte set wrongly),
- (b) misaligned-LONG load reconstruction in LSU, or
- (c) renamer / PRF allocation bug for An destinations.

Specifically: dump A5/A6/VBR at every retired uop in both sim and MAME,
diff at first divergence. The bisect skill `m68k-mame-rtl-bisect` is
designed for exactly this.

## Hard "do-not-fix" list

- Do **not** mask VBR low bits in `commit.v:3177` — that would break
  `tb/tests/asm/unaligned_long_vector_dispatch.s` and diverge from
  Musashi/MAME. The Q700 ROM intentionally uses unaligned VBR.
- Do **not** change the fmt-7 push order in `exception_uop_gen.vh` —
  the byte-level decode confirms it matches the architectural layout.
- Do **not** alter `frame_sz=60` / `push_word_count=30` for vec 2/3 —
  matches PRM §8.4.1.
