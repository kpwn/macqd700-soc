# Handoff — `DIV.L` SZ=1 (64÷32, dual-register dividend)

Single-bug handoff. Reproduced on hardware 2026-08-20.

## Symptom

Zero-divide trap (vector 5), caught in MacsBug, at ROM `0x40891988`.

## The instruction

```
40891984:  4c44 1403    DIVU.L D4, D3:D1      <-- the divide
40891988:  691a         BVS.S  0x408919a4     <-- the reported PC
```

The reported PC is the instruction *after* the divide, which is normal for a
trap taken on it.

`0x4C44` = `DIVU/DIVS.L`, EA = mode 0 / reg 4 = `D4` (the divisor).
Extension `0x1403` decodes as:

| field | bits | value | meaning |
|---|---|---|---|
| Dq | 14:12 | `001` | quotient → `D1` |
| signed | 11 | `0` | **DIVU** (unsigned) |
| **SZ** | **10** | **`1`** | **64-bit dividend in `Dr:Dq` = `D3:D1`** |
| Dr | 2:0 | `011` | remainder → `D3` |

So: **`DIVU.L D4, D3:D1`** — the 64÷32 dual-register form.

## Root cause: this form is explicitly deferred, not implemented

`cpu/rtl/core/decode/decode.v:1120-1125`:

```verilog
wire muldiv_sz0_ok    = muldiv_is_w_f || (muldiv_is_l_f && !ext1[10]);
// Dual-destination MUL.L SZ=1 (task #79): ext1[10]=1 on MUL.L only.
// Emitted as a SINGLE μop with has_dst_b=1 (arch_dst_b = Dh = ext1[2:0]).
// DIV.L SZ=1 (64÷32 with 64-bit dividend in Dh:Dl) remains deferred —
// it needs a 64-bit src_a plumb too, which this landing does not do.
wire muldiv_mul_l_sz1 = muldiv_is_mul_l && ext1[10];
```

`MUL.L` SZ=1 landed; **`DIV.L` SZ=1 did not**, because it additionally needs a
64-bit source-A path. `muldiv_sz0_ok` gates on `ext1[10] == 0`, so this
encoding does not take the implemented divide path.

`CLAUDE.md` already lists `MUL.L/DIV.L SZ=1 dual-dst` among the pending items,
**with fuzz seed `183896784` recorded against it** — there is a repro waiting.

## Why it surfaced now

Same shape as the last three frontiers this session: **a documented deferral
that became reachable because the things upstream of it were fixed.**

The FPSP work (`2649626e` FCMP-in-extended + `FMOVE.D`, `1711879` dyadic
`.X`/`.D` memory sources) let MacBench and Speedometer run *past* the FPSP
compare/convert code they previously died in. FPSP's decimal↔binary conversion
is exactly where 64-bit divides live. Note the address neighbourhood: the Sad
Mac signature that `fpsp_dec2bin_sp_integrity` pins is at `0x4088E4C6`, a few
KB below this site.

So this is **progress**, not a regression. Do not bisect it against today's
FPU commits.

## First thing to check — it is cheap and it changes the fix

**Read `D4` at the trap in MacsBug.**

- **`D4 == 0`** → the divide-by-zero is *architecturally correct*; the real bug
  is upstream, in whatever computed `D4`. Chase the producer, not the divide.
- **`D4 != 0`** → we are mis-decoding or mis-executing the SZ=1 form and
  raising a spurious vector 5. That is the bug described here.

The second is much more likely given the RTL comment, but the check costs one
MacsBug command and decides which problem you are solving. Also worth
capturing `D1` and `D3` (the 64-bit dividend halves) at the same time — they
tell you whether the operands arrived intact.

## The fix, if it is the decode path

1. **Plumb a 64-bit `src_a`.** This is the piece `MUL.L` SZ=1 did not need and
   is the reason the deferral exists. `MUL.L` SZ=1's approach — a single µop
   with `has_dst_b=1`, `arch_dst_b = Dh = ext1[2:0]` — is the template for the
   *destination* half; the source half is new work.
2. **Both `DIVU.L` and `DIVS.L`** (`ext1[11]` selects). Do not do only the
   unsigned form because that is what the ROM hit.
3. **Overflow semantics matter here.** The ROM does `BVS.S` immediately after,
   so it *expects* a quotient overflow to be reportable. `590ea5fb` already
   landed DIV.L overflow N/Z/C preservation per Musashi — check that work
   covers the SZ=1 form or extend it. Getting V wrong turns a handled case
   into a wrong branch.
4. **Divide-by-zero must still trap** (vector 5) with the correct stacked
   frame, and must not be raised when the divisor is non-zero.

## Verification

- **Musashi is the referee.** `tools/fuzz/fuzz.py --replay` with the recorded
  seed **`183896784`** is the starting repro.
- **Widen `tools/fuzz/gen_program.py`** to emit `DIVU.L`/`DIVS.L` SZ=1 with
  randomised 64-bit dividends — including divisors that produce a quotient
  overflow, a zero divisor, and dividends whose high half is non-zero (the
  case a 32-bit-only implementation silently gets right for the wrong reason).
  Show it **RED on the pre-fix tree**; recent precedent is `115/85 → 200/0`
  (`99b9ea26`) and `40/60 → 60/60` (`1711879`).
- **Directed test using the exact ROM encoding** `4c44 1403`, plus the signed
  form and the overflow/zero-divisor corners.
- Every new scenario **RED-verified against a named mutant**. Nine times this
  session an agent's first test version silently passed the mutant it was
  written to catch — twice because a new file was auto-found by Verilator's
  `-I`/`-y` and so was **not a make prerequisite; nothing rebuilt**.

### Gates (current baseline)

`make lint` 0 · `make test` **992 PASS / 0 FAIL / 1 DEFER**
(`ipl_lower_andi_precise` is the expected DEFER) · `make fuzz N=200` 200/200 ·
`tb-fpu` 70/70 · `tb-fp-rat` 10/10 · `tb-iq-fp` 14/14.

## Follow-ups to close with it

- `CLAUDE.md`'s pending list still names `MUL.L/DIV.L SZ=1 dual-dst`; `MUL.L`
  is done, so that line is already half stale.
- `docs/decode_coverage_matrix.md` and `docs/isa_status.md` should record the
  SZ=1 divide once it lands.
- The RTL comment at `decode.v:1123` is the deferral notice — delete it rather
  than leaving it to contradict the code.
