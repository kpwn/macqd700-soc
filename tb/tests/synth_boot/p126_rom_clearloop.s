| p126_rom_clearloop.s -- Part 126 reproduction asset (docs/BUG_calibration_word_misplaced_0d00.md).
| Build+run: make tb-fpga-top-rom CPU=m68k040 SIM_L2C_ENABLE=1 ROM=<this, as .bin>
| Result on the real SoC RTL: the CLR loop runs exactly 8 times and EXITS --
| i.e. it does NOT reproduce the hardware wedge.  Kept as the negative control
| for anyone tempted to reproduce that wedge from the instruction stream alone.
| Part 126 stub 3: the ROM's 0x4084BEA8..0x4084BED2 routine copied VERBATIM,
| including its `lea %pc@(ret),%fp / jmp %pc@(sub)` call and `jmp %fp@` indirect
| return, entered with the board's own measured register state for a single
| 64 MB bank.  Same posture: CACR=0x00008000 (D-cache OFF), MMU off.
    .text
    .org 0
_vectors:
    .long   0x00500000
    .long   _start
_start:
    move.l  #0x00008000,%d1
    movec   %d1,%cacr
    nop
    lea     _bankbase,%a1       | [a1] = 0, the bank base, exactly like the ROM
    move.l  #0x04000000,%d3     | total RAM
    move.l  #0x0000002C,%d1     | the 44-byte reservation
    move.l  #0x00400000,%d2     | size unit for the shift subroutine
    move.l  #0x00000005,%d5     | one bank, nibble 5 -> 0x00400000 << 4 = 0x04000000
    bra.s   _bea8
_be9a:
    moveq   #0,%d0
    tst.w   %d1
    beq.s   _bea6
    move.l  %d2,%d0
    subq.w  #1,%d1
    lsl.l   %d1,%d0
_bea6:
    jmp     (%fp)               | 4ed6 -- indirect return through A6
_bea8:
    movea.l (%a1),%a5
    movea.l %a5,%a0
    adda.l  %d3,%a5
    suba.l  %d1,%a5
    movea.l %a5,%sp             | 2e4d
_beb2:
    moveq   #15,%d1
    and.b   %d5,%d1
    beq.s   _bec6
    lea     _bec0,%fp           | 4dfa
    jmp     _be9a               | 4efa
_bec0:
    move.l  %a0,(%sp)+          | 2ec8
    move.l  %d0,(%sp)+          | 2ec0
    adda.l  %d0,%a0
_bec6:
    lsr.l   #4,%d5
    bne.s   _beb2
    subq.l  #1,%d5
    move.l  %d5,(%sp)+          | 2ec5
_bece:
    clr.l   (%sp)+              | 429f  <-- THE WEDGE
    move.w  %sp,%d0             | 300f
    bne.s   _bece               | 66fa
    move.l  #0x5A932BC7,%d7     | reached only if the loop EXITS
    move.l  #0x00010DB0,%a2
    move.l  %d7,(%a2)
_done:
    bra.s   _done
    .align 4
_bankbase:
    .long   0x00000000
