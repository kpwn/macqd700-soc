# Final-register Musashi comparison

`regstate_compare.py` is a deterministic companion to `tools/fuzz/fuzz.py`.
It generates many small assembly programs focused on byte/word writes to data
registers plus a small set of address-register and postincrement smoke cases,
runs each program on both the RTL simulator and Musashi, then diffs the final
CPU register dump produced by the existing `+dump_final_state=...` protocol.

Run a bounded sweep through the top-level Makefile:

```bash
make regstate-compare
make regstate-compare REGSTATE_ARGS="--family move_w_imm --dst 0 --limit 12"
make musashi-parity
make musashi-adversarial
```

Or run it directly:

```bash
python3 tools/regstate/regstate_compare.py \
  --sim build/sim/Vmac_top \
  --musashi tb/models/musashi_run \
  --work build/regstate-sweep \
  --limit 32
```

Useful filters:

```bash
# See generated case names without running them.
python3 tools/regstate/regstate_compare.py --list --limit 20

# Run one operation family across the selected matrix.
python3 tools/regstate/regstate_compare.py --family move_w_imm --limit 64

# Focus on one destination register.
python3 tools/regstate/regstate_compare.py --dst 0 --limit 64

# Focused slices for recent decode/regstate corners.
python3 tools/regstate/regstate_compare.py --family byte_unary --limit 16
python3 tools/regstate/regstate_compare.py --family movea_postinc --limit 16
python3 tools/regstate/regstate_compare.py --family move_w_postinc --limit 16
python3 tools/regstate/regstate_compare.py --family move_reg_partial
python3 tools/regstate/regstate_compare.py --family alu_reg_bw_supported
python3 tools/regstate/regstate_compare.py --family quick_reg_supported
python3 tools/regstate/regstate_compare.py --family quick_mem_supported --timeout 50000 --host-timeout 10
```

The generated matrix is deliberately wider than today's known-good decode
surface.  Treat timeouts from unsupported families as triage data; land focused
family slices once the matching RTL behavior is fixed.  Do not add broad
families to the default smoke until the full selected slice is known to pass.

The default `make regstate-compare` smoke now also includes:

- `movea.w #imm,An` sign-extension.
- `movea.l (An)+,An` writeback.
- `move.w (An)+,Dn` partial-register preservation.
- `move.b (An)+,Dn` upper-24-bit preservation.
- `swap` and `extb.l` final-state checks.
- `move.b/w Dn,Dn` partial-register preservation with nonzero upper bits.
- Supported `and`, `or`, and `add` byte/word register-source forms.
- Supported `addq/subq` byte/word/long data-register forms, including the
  encoded-0-as-8 immediate, plus `addq.l/subq.l` address-register forms.
- Supported `addq/subq` byte/word/long memory-destination forms for the
  currently implemented `(An)`/small-displacement probes.
- A bounded Q700 ROM-frontier slice covering exact opwords for `MOVE.W A7,D0`,
  `MOVE.L A0,(A7)+`, `MOVEM.L (d16,PC),D0-D5`, PC-indexed/scaled `LEA`,
  `SUBA.L (An),An`, indexed `Scc`, indexed `CMP.B`, indexed `NOT.B`,
  PC-indexed `JMP`, and indexed byte stores.

Excluded from the default smoke for now:

- `eor_b_reg` and `eor_w_reg`: current final-state probes report Dn
  mismatches for register-source byte/word forms.
- `sub_b_reg` and `sub_w_reg`: current final-state probes time out in the RTL
  harness.
- Broad memory-destination, shift/rotate, CCR, and `a7` comparison surfaces.
  Keep these behind explicit filters until their supported subsets are pinned.

The default comparison keys are `pass`, `d0` through `d7`, and `a0` through
`a6`.  `a7` is skipped for the same reason as the fuzzer: Musashi and RTL stack
initialization are not normalized.  CCR is skipped by default because the
existing PASS sentinel is a `MOVE.L` store after the instruction under test,
which clobbers flags before final-state capture.

Memory-write comparison is intentionally opt-in with `--include-memory`.
The current write log has a known D-cache stale-byte writeback artifact
documented in `tb/fuzz_fails/BUG_dcache_writeback_stale_bytes.md`; default
regstate cases fold memory effects back into final registers instead.
When enabled, memory write-log mismatches are summarized by line and class
(`stale-byte-line`, `presence-only`, or `mixed-values`) before the individual
byte diffs.

Generated programs and state dumps live under the `--work` directory.  Passing
case assembly and binaries are removed unless `--keep-asm` is passed; failing
cases are copied to `--work/fails`.

`make musashi-parity` runs a bounded stop-PC slice that avoids the PASS
sentinel store so final CCR/SR are preserved.  It compares D0-D7, A0-A7, PC,
SR, and CCR for a small trap/RTE + ROM-frontier slice:

- trap/RTE CCR restore;
- user-mode trap round-trip with supervisor-stack switching;
- format-0 vector-frame word extraction plus Dn partial-register drift;
- the ROM memory-probe frontier shape around `MOVEA.L (A7)+`, indexed
  `CMP.B`, and indexed memory `NOT.B`.

`make musashi-adversarial` runs the same comparator in sentinel-completion
mode with memory writes enabled.  It keeps a small write-log-aware slice
focused on unaligned WORD access, same-address store/load repetition, and a
known-current MOVEM A7 xfail.  The xfail entries are annotated in
`tools/regstate/regstate_compare.py` with explicit reason strings.

Pass `MUSASHI_ADVERSARIAL_ARGS="... --compare-a7"` or
`MUSASHI_PARITY_ARGS="... --include-memory"` for local investigations when you
want to widen a single slice temporarily.  The default parity target leaves
memory out because the top-level write-back cache can report clean-line
writeback bytes that Musashi never records as CPU writes.
