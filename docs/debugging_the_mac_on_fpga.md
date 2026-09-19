# Debugging the Mac on the FPGA

How to attach real GDB to the m68k-ooo Quadra 700 and actually find things.
You should not need to read any source to follow this.

---

## 0. TL;DR

```bash
# once per checkout
tools/macsym.py build -o build/macos.elf

# terminal 1 — the JTAG bridge (needs Vivado + the board)
mkfifo /tmp/jtag_in
nohup bash -c 'exec 7>/tmp/jtag_in; sleep 99999' >/dev/null 2>&1 & disown
nohup bash -c "vivado -mode tcl -nojournal -nolog -source tools/jtag_repl.tcl \
  -tclargs <bitstream.bit> <probes.ltx> < /tmp/jtag_in > /tmp/jtag_out 2>&1" \
  >/dev/null 2>&1 & disown
# wait until /tmp/jtag_out ends with "> READY"

# terminal 2 — the GDB stub
tools/gdbstub.py --port 1234

# terminal 3 — GDB
gdb-multiarch build/macos.elf
(gdb) target remote :1234
```

**No board?** `tools/gdbstub.py --fake --port 1234` runs the whole stack
against a simulated CPU. Everything below works, and nothing you see is real.

---

## 1. What you get

| GDB command | Works | Notes |
|---|---|---|
| `info registers` | yes | D0-D7, A0-A7, PC, SR, and USP/SSP/ISP/VBR/TC/ITT/DTT/SRP/URP |
| `x/`, `set var`, `print` | yes | memory read/write, D-cache handled (§5) |
| `break` / `hbreak` | yes | **4 maximum** — hardware slots, see §3 |
| `watch` / `rwatch` / `awatch` | yes | **2 maximum**, physical addresses, see §4 |
| `stepi` / `nexti` | yes | injection-based; works across traps |
| `continue`, Ctrl-C | yes | |
| `bt` | see §7 | GDB's own unwinder needs to analyse prologues; **`monitor bt` is the one that always works** |
| `disassemble` | yes | reads live memory through the stub |
| `info symbol`, `break _FSDispatch` | yes | from the symbol file, §2 |
| `step` / `next` (source level) | no | there is no source or line info |
| FPU registers | no | not plumbed |

The stub deliberately has **no software breakpoints**. Patching an illegal
opcode into memory would take the exception, push a frame, and leave the
machine somewhere it cannot be cleanly resumed from — it would appear to work
right up to the point where it lied about where you were. You get four real,
precise hardware breakpoints and an honest error when they run out.

---

## 2. Symbols

There is no Mac OS symbol file. There is a hand-built map of addresses this
project has identified, in `tools/macsyms/*.syms`:

```
0002E938  _FSDispatch   0040 ; File System trap dispatcher
08CE      CrsrNew       0001 :data ; cursor-changed flag
```

Build it into something GDB can load:

```bash
tools/macsym.py build -o build/macos.elf
gdb-multiarch build/macos.elf
```

Now `bt`, `disassemble`, `info symbol 0x2e94a` and `break _FSDispatch` all
work. The ELF carries **no code** — its sections are `SHT_NOBITS` — so GDB
disassembles by reading the live board, never a stale copy.

**Found a new address?** Add a line to `tools/macsyms/macos_q700.syms` and
rebuild. That is how the file grows; it is the accumulated output of every
debugging session. Rules the tool enforces: names must be unique, every
symbol must sit inside a declared `region`, and a malformed line is a hard
error rather than a skipped line — a half-loaded symbol file puts confident
wrong names on right addresses.

Symbolization is also used everywhere in `monitor` output, and
`macsym.py lookup 0x2e94a` works standalone. Addresses far from any known
symbol print as bare hex rather than as `something+0x4b000`.

---

## 3. Breakpoints

`break *0x2e938` or `break _FSDispatch`. Four slots. They are **precise and
pre-effect**: the CPU stops *before* the breakpointed instruction has any
side effects.

```
(gdb) monitor bp status
slot 0: ARMED pc=0x0002e938 <_FSDispatch>   (GDB breakpoint at 0x0002e938 <_FSDispatch>)
slot 1: off   pc=0x00000000
...
0/4 slots used by GDB. This target has no software breakpoints.
```

Every arm is written and then **read back and compared**. If a slot does not
take the value, you get an error, not a breakpoint that silently never fires.

Which breakpoint was hit is determined from the halted PC, not from the
hardware's hit-slot register — multi-slot hit attribution has been wrong on
this design before, and the PC never is.

---

## 4. Watchpoints

`watch *(short*)0x8ce` arms one of **2** hardware data watchpoints.

Three things to know, all of them hardware properties:

1. **They compare physical addresses**, after MMU translation. With
   translation on, a virtual address from GDB matches only where the mapping
   is identity. The stub warns once per session if `TC` says translation is
   enabled.
2. **The compare is longword-granular.** Watching one byte also stops on
   accesses to the other three bytes of that longword.
3. **Read-modify-write byte stores are missed.** `bset`/`bclr` on memory do
   not trip a watchpoint on this bitstream. This is a known hardware defect,
   not something the host can work around.

`monitor watch status` shows what is actually armed, read back from hardware,
plus the last latched hit.

---

## 5. Memory, and the zero that isn't

**JTAG reads bypass the CPU's write-back D-cache.** A location the CPU wrote
a moment ago can read back as whatever RAM last held — very often zero. A zero
looks exactly like "never written", and this has cost real debugging time.

The stub pushes the D-cache to RAM once per halt before serving any memory
read, so by default `x/` shows you the truth. Controls:

```
(gdb) monitor cache          # show current setting
(gdb) monitor cache flush    # push now
(gdb) monitor cache off      # faster, but you are back on your own
```

`monitor bt` calls this out explicitly when it hits a zeroed frame, because
"the chain ended" and "the chain hasn't been written back" look identical.

Memory *writes* push, then write, then invalidate both caches, in that order —
invalidating first would discard the CPU's own dirty lines.

---

## 6. A-trap breakpoints — the Mac-specific superpower

Every Mac OS toolbox call is an A-line instruction. Stopping on one needs no
boot disk, no OS cooperation and no single-stepping, and it stops *before* the
trap takes effect, with **A0 and D0 captured at the trap** — which is exactly
what identifies the request, since Mac OS passes the param-block pointer in A0
and a selector in D0.

```
(gdb) monitor atrap arm _SCSIDispatch
A-trap slot 0 armed: value=0xa815 mask=0xffff  _SCSIDispatch
(config survives CPU reset, so you can arm it before 'monitor reset' to catch
an early-boot call)

(gdb) continue
[A-trap slot 0: opword 0xa815 at 0x00009c74 <_SCSIGet> A0=0x00383d10 D0=0x00000001]
[  trap _SCSIDispatch]
```

Useful forms:

```
monitor atrap arm _Read                    # one specific trap
monitor atrap arm 0xA800 mask 0xFF00       # the entire 0xA8xx family
monitor atrap arm _Control d0 0x00000005   # only when D0 == 5
monitor atrap list                         # known trap names
monitor atrap status                       # armed slots + last hit
monitor atrap off 0
```

Two slots. Mask polarity here is **1 = compare this bit** (the opposite of the
watchpoint address mask, where 1 = ignore — both conventions come from the
RTL). The hardware also requires the opcode to be `0xAxxx`, so an
all-don't-care mask can never degenerate into "stop on every instruction".

**Arm before reset.** The debug configuration survives a CPU reset, and
JTAG-AXI is dead for 40-90 s after a platform reset while the DDR controller
calibrates. Arming first is the only way to catch a one-shot early-boot call.

One subtlety: with a D0 qualifier armed, a *non-matching* call halts the CPU
for a few cycles and then resumes itself. The stub knows this and will not
report it as a stop.

---

## 7. Mac-aware monitor commands

```
(gdb) monitor help
```

### Registers and stack

```
monitor regs            # everything, with SR decoded into flags/S/M/IPL
monitor bt [n]          # explicit A6 frame-chain walk, symbolized
monitor status          # halt state, stop reason, build id, staged writes
```

`monitor bt` stops and says why the moment the chain stops looking like a
chain (odd pointer, cycle, non-growing stack, unreadable frame) rather than
printing plausible frames.

**Use `monitor bt`, not `bt`, when you care about the answer.** GDB's own
unwinder works by disassembling each function's prologue to work out the frame
layout. That is the right approach when it succeeds, but there is no debug
info here, Mac OS is full of hand-written assembly, and it gives up quietly —
typically showing `#0` and then `?? ()`. `monitor bt` walks the A6 link chain
directly, symbolizes each return address, and tells you exactly where and why
it stopped. Neither has been exercised against a real booted Mac OS stack
yet; `monitor bt`'s walk is verified against a synthetic chain only.

### Low-memory globals by name

```
(gdb) monitor lomem DSErrCode CrsrNew
DSErrCode      $0AF0 (2B) = 0x000c   ; System Error code
CrsrNew        $08CE (1B) = 0xff     ; cursor-changed flag

(gdb) monitor lomem            # everything in the map, with values
```

Widths come from the symbol file, so a byte global is read as a byte. A name
that isn't in the map is reported as missing — the command will not guess an
address.

### OS queues

```
(gdb) monitor queue fs
File System queue  (FSQHdr @ $0360)
  qFlags=0x0000  qHead=0x00383d10  qTail=0x00383d10
  pb 0x00383d10  qType=2  ioTrap=0xa002(_Read)  ioResult=1  ioRefNum=-5  ioCompletion=0x00000000
```

`monitor queue fs | dt | vbl | drive` walks the File System, Deferred Task,
VBL and Drive queues and decodes each element's real structure. It stops on a
cycle, an odd pointer or an unreadable node and says so.

### Device Manager

```
(gdb) monitor dce           # walk the unit table
(gdb) monitor dce -5        # just that refNum
```

### Everything else

```
monitor features            # what this bitstream actually supports
monitor halt-on-exc add 2   # stop on bus error
monitor halt-on-exc enable
monitor exc-ring 32         # recent exceptions
monitor pc-trace 64         # recent PC redirects
monitor reset [hold|release]
monitor repl <anything>     # raw jtag_repl.tcl passthrough
```

`monitor features` is worth running first on any unfamiliar bitstream. A good
share of historical confusion in this project came from exercising a feature
the loaded bitstream did not have.

---

## 8. Things that will bite you

**Register writes are deferred.** `set $d0 = 5` does not take effect
immediately — it is staged and applied on the next `continue` or `stepi`.
This is not laziness: applying architectural registers on this hardware
*auto-resumes the CPU*, and a debugger that let the machine run for a few
million cycles every time you typed `set $pc` would be worse than useless.
`monitor status` lists anything staged. Reads reflect staged values, so
`print $d0` after `set $d0 = 5` shows 5.

**4 breakpoints, 2 watchpoints, 2 A-traps.** Hard hardware limits. GDB is
told, and you get an error rather than a breakpoint that silently isn't there.

**A double fault is not a stop.** If `monitor status` reports a double fault,
the CPU is wedged and cannot be resumed — only `monitor reset` recovers it.

**`monitor repl` is a loaded gun.** It bypasses every verification in the
stub. In particular `break-pc` in the raw REPL *releases the halt and runs for
200 ms* as part of arming; that is fine for interactive bring-up and wrong for
GDB, which is why the stub programs the breakpoint registers itself.

**Nothing here can read registers on a running CPU.** The register snap chain
is only valid while halted. The stub refuses rather than returning the
coherent-looking nonsense you get otherwise — that read has produced a
confident wrong conclusion in this project before.

---

## 9. When something looks wrong

The stub is built so that a failure is loud. If you see an error, it is
telling you something true:

* `REPL echoed address 0x… but we asked for 0x…` — operand parsing or bus
  aliasing. The value was discarded, not returned.
* `AXI read failed — REPL returned the BADA0BAD sentinel` — a real bus
  failure. Not a zero.
* `dump-mem … returned N identical words but an independent single read
  disagrees` — the burst path is fabricating data; the read was refused.
* `wrote 0x… but read back 0x…` — a breakpoint/watchpoint arm did not land.
  It is *not* reported as armed.
* `refusing to read architectural registers while the CPU is running` — halt
  first.

If you would rather have the raw value anyway, `monitor repl r <addr>` gives
it to you unchecked. Be aware that is exactly the path that has produced wrong
answers historically.

---

## 10. Developing the tooling itself

```bash
make tb-gdbstub-host      # host-side protocol tests, no FPGA, no Vivado
make tb-jtag-repl-host    # jtag_repl.tcl helper tests
```

`tb/tests/host/fake_jtag_repl.py` is a software model of the debug register
file that deliberately reproduces the hardware's awkward semantics — the
auto-resume on register apply, the read-bit-15/write-bit-14 asymmetry on the
breakpoint hit latch, poisoned register reads while running, the D-cache
bypass. Tests passing against it prove the *host* is right. They say nothing
about the hardware.
