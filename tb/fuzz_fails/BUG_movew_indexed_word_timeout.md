# BUG: MOVE.W (d8,An,Xn) indexed load times out in RTL

The word-index brief-extension form hangs.  A proposed long-index directed
suite test also timed out during integration, so keep both shapes out of live
fuzz/regstate coverage until the decode path is fixed.  This repro keeps the
two candidate addresses deterministic:

- base `A0 = 0x00100000`
- index `D0 = 0x00010002`
- word target `0x00100014`
- long target `0x00110014`

Observed behavior:

- Musashi completes and loads the seeded word from `0x00100014`.
- RTL does not reach the PASS sentinel and times out.

Hypothesis:

The brief-extension decode or the indexed load writeback path is incomplete
for MOVE.W `(d8,An,Xn)` forms. The word-sized index case additionally stresses
sign-extension when the upper 16 bits of the index register are non-zero.
