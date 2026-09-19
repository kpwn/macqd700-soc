#!/usr/bin/env python3
"""cpu040_mame_divergence_diff.py -- find the FIRST control-flow divergence
between a cpu040 full-SoC boot sim and MAME running the same ROM.

    usage: cpu040_mame_divergence_diff.py <mame_trace> <cpu040_pc_dump> <window>

<mame_trace>  a FULL, uncondensed MAME CPU trace.  Capture it with
              `trace <file>,0,noloop` in a -debugscript.  NOTE THE GOTCHA:
              in MAME, `noloop` means "do NOT condense loops", i.e. it is
              what produces the complete instruction-by-instruction trace.
              Omitting it condenses loops and is useless here.
<cpu040_pc_dump>  tb_fpga_top_rom.cpp's `+pc_dump_path=` output, run with
              `+pc_dump_every=1`.  Format: `<retire_idx> <sim_time> 0xPC`.
              Snapshot it with `head -n -1` -- reading a live file yields a
              truncated final line that looks exactly like a divergence.
<window>      lookahead in MAME instructions (64 is a good default).

Why a subsequence match rather than a straight diff: the cpu040 pc-dump is
NOT the raw instruction stream.  It reports only what the two RobPlugin
traceVec ports expose, so some executed instructions never appear (a
missing PC is NOT a skipped instruction), and ~33% of its lines are
duplicate consecutive reports of the same PC.  So we greedily match the
cpu040 PC stream as a subsequence of MAME's, absorbing duplicates.

Known blind spot: because duplicates are absorbed, this cannot see cpu040
executing MORE iterations of a SINGLE-INSTRUCTION self-loop than MAME.  It
does detect cpu040 leaving a loop early, and every multi-instruction
loop-count difference in either direction.

See docs/BUG_calibration_word_misplaced_0d00.md Part 112.
"""
import sys
from array import array
mame_path, cpu_path, W = sys.argv[1], sys.argv[2], int(sys.argv[3])
print("loading MAME trace...", flush=True)
pcs=[]
with open(mame_path,'r',errors='replace') as f:
    for line in f:
        if len(line)>8 and line[8]==':':
            try: pcs.append(int(line[:8],16))
            except ValueError: pass
M=array("I",pcs); del pcs
print("MAME instrs:",len(M), flush=True)
i=0; n=0; dup=0; maxgap=0; maxgap_at=None; last=-1
prev=[]
with open(cpu_path,'r',errors='replace') as f:
    for line in f:
        p=line.split()
        if len(p)!=3: continue
        try: pc=int(p[2],16)
        except ValueError: continue
        n+=1
        if pc==last:            # duplicate trace-port report; absorb
            dup+=1; continue
        hi=min(i+W,len(M)); j=-1
        for k in range(i,hi):
            if M[k]==pc: j=k; break
        if j<0:
            print("\n*** DIVERGENCE at cpu040 retire #%d ***"%n)
            print("cpu040 pc=0x%08x time=%s  (MAME pos i=%d)"%(pc,p[1],i))
            print("--- MAME expected next (i-4 .. i+30) ---")
            for k in range(max(0,i-4), min(i+30,len(M))):
                print("   [%d] 0x%08x %s"%(k,M[k],"<<< expected here" if k==i else ""))
            print("--- cpu040 last 16 distinct retired PCs ---")
            for x in prev[-16:]: print("   0x%08x"%x)
            sys.exit(0)
        g=j-i
        if g>maxgap: maxgap=g; maxgap_at=(n,hex(pc),i,j)
        i=j+1; last=pc
        prev.append(pc)
        if len(prev)>40: prev.pop(0)
        if n%1000000==0: print("  %d cpu040 retires -> MAME pos %d (dups %d, maxgap %d)"%(n,i,dup,maxgap), flush=True)
print("\nNO DIVERGENCE in compared window.")
print("cpu040 dump lines: %d (dups absorbed %d); MAME instrs consumed: %d"%(n,dup,i))
print("max skip-gap: %d at %r"%(maxgap,maxgap_at))
