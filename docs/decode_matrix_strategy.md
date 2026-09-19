# Decode Matrix Strategy

Goal: stop widening the 68040 decoder one ROM-frontier instruction at a
time.  The target is every legal size and addressing-mode combination for the
families in `romcodes`, plus explicit vec-4 coverage for illegal combinations.

## Structural Direction

`uop_size` is already the right common field for most byte/word/long cracking.
The decoder should use one family template where the ISA encoding only changes
size, and derive `uop_size` from the opcode bits:

- group 0 immediate ops: `op[7:6]` maps B/W/L, with `11` reserved for sibling
  encodings such as bit ops and CAS/CMP2.
- group 4 unary ops: `op[7:6]` maps B/W/L for CLR/TST/NEG/NEGX/NOT.
- group 5 quick ops: `op[7:6]` maps B/W/L for ADDQ/SUBQ; An direct is word/long
  address arithmetic and does not write CCR.
- groups 8/9/B/C/D binary ops: size is usually `op[7:6]`, while adjacent bits
  select direction and special register/address-register forms.
- MOVE is special: the top nibble selects byte/long/word, and destination EA
  fields are encoded differently from the other ALU families.

`uop_size` alone is not enough.  A shared family crack also needs a shared EA
contract because size affects immediate length, memory access width,
postincrement/predecrement stride, A7 byte stride, register merge semantics, and
CCR updates.

## EA Contract

The useful abstraction is an EA classifier that returns the facts every family
currently hand-writes:

- legal for source, destination, memory-alterable, control, or data-alterable
  use.
- base register, absolute marker, displacement/immediate payload, and extension
  word count.
- postincrement/predecrement update register and byte stride, including the A7
  byte stride of 2.
- immediate payload width for `#imm` forms.
- PC-relative legality and extension length.
- brief/full extension length for indexed and memory-indirect forms.

`decode.v` already has `decode_ea_ext_words`; widen from that toward a small
set of helpers rather than adding unrelated local length expressions in every
include file.

### Reusable Helper Shape

Keep the helper as a textual include or helper block in `decode.v`, not as a
new module boundary.  The family include files already depend on decode-local
wires, so a flat helper preserves the current style and avoids new timing or
hierarchy questions.

The first useful helper set is:

- `decode_ea_ext_words(mode, reg, size, ext)` for instruction-length accounting.
- `decode_ea_stride_bytes(size, reg)` for postincrement/predecrement writeback,
  including the A7 byte stride of 2.
- `decode_ea_is_data_alterable(mode, reg)` for normal ALU destination EAs.
- `decode_ea_is_memory_alterable(mode, reg)` for read-modify-write memory
  destinations.
- `decode_ea_is_control(mode, reg)` and `decode_ea_is_control_alterable(mode,
  reg)` for `JMP`, `JSR`, `LEA`, `PEA`, and branch-target-like users.
- `decode_ea_is_immediate(mode, reg)` and `decode_ea_is_pc_relative(mode, reg)`
  so family code can explicitly reject forms that are legal only as sources.

Do not add a packed EA descriptor bus yet.  The decode outputs already describe
the uop contract well enough: `has_src_a`, `has_src_b`, `has_dst`, `imm_valid`,
`is_load`, `is_store`, `is_abs`, `flags_rd`, and `flags_wr`.  The reusable layer
should provide legality, length, stride, and address-class facts; family code
should still choose the ALU op, flag policy, source/destination registers, and
multi-uop ordering.

The current orphaned decode worktrees show why this split matters: broad
hand-expanded patches for MOVE, 0000 bit/immediate forms, 0101 quick/Scc forms,
and 0100 unary/CHK forms compile or partly compile, but fail their own focused
tests or have structural decode damage.  Those failures are consistent with
duplicated EA sequencing rather than one isolated opcode bug.

## Family Templates

Good candidates for one parameterized template:

- `ADD/SUB/AND/OR` register-destination and memory-destination forms.
- `CMP` register-destination forms, with no destination writeback.
- `EOR` data-register source to EA.
- `ADDI/SUBI/ANDI/ORI/EORI/CMPI` immediate to data-alterable EA.
- `CLR/TST/NEG/NEGX/NOT` unary EA forms.
- `ADDQ/SUBQ`, with separate An-direct address arithmetic handling.
- data-register shifts/rotates, where B/W/L are the same crack and only
  `uop_size` changes.

Keep these explicit:

- `MOVEA.W` sign-extends and does not write CCR.
- `MOVE` memory-to-memory cracks need source and destination EA sequencing.
- bit ops use long width for Dn destinations and byte width for memory
  destinations despite having no size suffix.
- memory shifts/rotates are word-only.
- `MOVEM`, `MOVEC`, `MOVES`, `CAS`, `CHK/CMP2`, long multiply/divide, bitfield
  ops, and exception/system ops have multi-uop or privilege details that should
  stay in family-specific code until the common EA helpers are stable.

## Migration Order

Convert families in the order that maximizes reuse while minimizing semantic
risk:

1. `ADD`, `SUB`, and `CMP` register-destination source-EA forms in groups
   `1101`, `1001`, and `1011`.  These already share the cleanest EA prelude
   shape and have directed address-register source coverage.
2. `OR` and `AND` in groups `1000` and `1100`.  They reuse the same source-EA
   contract but add immediate-register and byte/word merge coverage.
3. `ADDI`, `SUBI`, `ANDI`, `ORI`, `EORI`, and `CMPI` immediate-to-EA forms.
   These should use the same destination legality and extension-length helper
   while keeping immediate placement family-local.
4. `ADDQ/SUBQ` and unary memory forms.  These are tempting to bulk-expand, but
   they need postincrement/predecrement ordering, A7 byte stride, and
   read-modify-write flag tests.
5. `MOVE` and `MOVEA` last.  They need source and destination EA sequencing,
   MOVEA sign-extension and CCR suppression, and memory-to-memory ordering.

## Test Gate

The gate should be matrix-shaped:

1. Expand `romcodes` to size siblings using `make rom-decode-gaps
   ROM_DECODE_GAP_FAMILIES=romcodes`.
2. For each family, generate legal synthetic assembly rows across size and EA
   classes, using the GNU assembler as the syntax/encoding source.
3. Probe each generated instruction with `tb_decode_probe`.
4. Require legal rows to avoid vec-4 and illegal rows to produce vec-4.
5. Add directed execution tests only for semantic risks: flags, byte/word
   register merge, postinc/predec writeback, A7 byte stride, sign extension,
   and multi-uop ordering.

This separates decode coverage from full semantic execution.  The decode probe
should catch missing cracks cheaply; Musashi parity and directed tests should
be reserved for behavior that decode acceptance cannot prove.
