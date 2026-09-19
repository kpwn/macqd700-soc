| fsave_frestore_basic.s — Test FSAVE/FRESTORE short-frame semantics
|
| Verifies, for every supported EA mode, that:
|   1. FSAVE writes a 68040 IDLE frame (0x41000000) to memory.
|   2. FRESTORE reads 4 bytes from memory (we sanity-check by
|      pre-poisoning the slot with 0xDEADBEEF and confirming the
|      load actually happened — no side-effect we can observe other
|      than An post-update, but we verify that).
|   3. Address-register side effects (predec / postinc) are correct.
|
| EA modes covered (sub-tests 1..5):
|   1. FSAVE -(An)        + FRESTORE (An)+    ← stack idiom (most common)
|   2. FSAVE (An)         + FRESTORE (An)
|   3. FSAVE (d16,An)     + FRESTORE (d16,An)
|   4. FSAVE (xxx).W      + FRESTORE (xxx).W
|   5. FSAVE (xxx).L      + FRESTORE (xxx).L

    .text
    .org 0

_start:
    lea     0x00010000, %a7         | stack high in scratch RAM

    | Pre-poison a 16-byte scratch buffer with non-zero so a NOP
    | FSAVE would be detectable.
    lea     0x00020000, %a0
    move.l  #0xDEADBEEF, (%a0)
    move.l  #0xDEADBEEF, 4(%a0)
    move.l  #0xDEADBEEF, 8(%a0)
    move.l  #0xDEADBEEF, 12(%a0)

    | ── Sub-test 1: FSAVE -(A7) / FRESTORE (A7)+ ───────────────
    move.l  %a7, %d2                | save A7 baseline
    fsave   -(%a7)                  | A7 -= 4, write 4-byte IDLE frame
    | A7 must now be d2 - 4
    move.l  %d2, %d3
    sub.l   #4, %d3
    cmp.l   %d3, %a7
    bne     _fail
    | Memory at new A7 must be the 68040 IDLE frame
    move.l  (%a7), %d4
    cmp.l   #0x41000000, %d4
    bne     _fail
    | Pop it back: A7 += 4
    frestore (%a7)+
    cmp.l   %d2, %a7                | A7 restored
    bne     _fail

    | ── Sub-test 2: FSAVE (A0) / FRESTORE (A0) ────────────────
    | Re-poison slot 0
    move.l  #0xDEADBEEF, (%a0)
    fsave   (%a0)                   | write 4-byte IDLE frame at (a0)
    move.l  (%a0), %d4
    cmp.l   #0x41000000, %d4
    bne     _fail
    | A0 must NOT change
    lea     0x00020000, %a1
    cmp.l   %a1, %a0
    bne     _fail
    | FRESTORE (A0) — observable: A0 still unchanged after
    frestore (%a0)
    cmp.l   %a1, %a0
    bne     _fail

    | ── Sub-test 3: FSAVE/FRESTORE (d16,An) ───────────────────
    | Use offset +8 from A0
    move.l  #0xDEADBEEF, 8(%a0)
    fsave   8(%a0)
    move.l  8(%a0), %d4
    cmp.l   #0x41000000, %d4
    bne     _fail
    cmp.l   %a1, %a0                | A0 unchanged
    bne     _fail
    frestore 8(%a0)
    cmp.l   %a1, %a0                | still unchanged
    bne     _fail

    | ── Sub-test 4: FSAVE/FRESTORE (xxx).W ────────────────────
    | (xxx).W is sign-extended 16-bit. Use a low address that
    | survives sign-extension: 0x1F00 (inside scratch RAM at low
    | end, well below 0x00010000 stack and far from 0x00020000).
    | Reset the 4-byte slot to 0xDEADBEEF first.
    move.l  #0xDEADBEEF, 0x1F00.w
    fsave   0x1F00.w
    move.l  0x1F00.w, %d4
    cmp.l   #0x41000000, %d4
    bne     _fail
    frestore 0x1F00.w               | exercise the LOAD path

    | ── Sub-test 5: FSAVE/FRESTORE (xxx).L ────────────────────
    move.l  #0xDEADBEEF, 0x00020010
    fsave   0x00020010
    move.l  0x00020010, %d4
    cmp.l   #0x41000000, %d4
    bne     _fail
    frestore 0x00020010

    | ── PASS ───────────────────────────────────────────────────
_pass:
    lea     0xFFFF0000, %a0
    move.l  #0xC0FFEE00, %d0
    move.l  %d0, (%a0)
_halt:
    stop    #0x2700
    bra     _halt

_fail:
    lea     0xFFFF0000, %a0
    move.l  #0xDEADBEEF, %d0
    move.l  %d0, (%a0)
_halt_fail:
    stop    #0x2700
    bra     _halt_fail
