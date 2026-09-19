| bchg_ccr_step.s — Step-by-step BCHG CCR test
|
| Goal: isolate which step of the BCHG memory form breaks CCR preservation.
|
| PASS: 0xC0FFEE00 at 0xFFFF0000.
| FAIL sentinels distinguish where:
|   0xDEAD0001  — MOVE.W #0x000a, CCR didn't stick (initial CCR wrong)
|   0xDEAD0002  — BCHG Dn,Dm cleared V (register form bug?)
|   0xDEAD0003  — BCHG #0,mem cleared V (static memory form)
|   0xDEAD0004  — BCHG Dn,mem cleared V (dynamic memory form)

    .text
    .org 0

_start:
    lea     0x00010000, %a7

    | === Step 1: confirm MOVE.W #imm, CCR works ===
    move.w  #0x000a, %ccr
    move.w  %sr, %d3
    and.l   #0x1f, %d3
    cmp.l   #0x0a, %d3
    beq     _step2
    move.l  #0xDEAD0001, %d7
    bra     _fail

_step2:
    | === Step 2: BCHG register form preserves V ===
    move.l  #0x0, %d0
    move.l  #0x0, %d4                     | bit 0 of D4=0, BCHG sets Z=1, toggles to 1
    move.w  #0x000a, %ccr                 | CCR=NV again
    bchg    %d0, %d4
    move.w  %sr, %d3
    and.l   #0x1f, %d3
    cmp.l   #0x0e, %d3                    | expect NVZ
    beq     _step3
    move.l  #0xDEAD0002, %d7
    bra     _fail

_step3:
    | === Step 3: BCHG #0, mem preserves V ===
    move.b  #0x00, 0x00020000
    move.w  #0x000a, %ccr                 | CCR=NV again
    bchg    #0, 0x00020000
    move.w  %sr, %d3
    and.l   #0x1f, %d3
    cmp.l   #0x0e, %d3                    | expect NVZ
    beq     _step4
    move.l  #0xDEAD0003, %d7
    bra     _fail

_step4:
    | === Step 4: BCHG Dn, mem preserves V ===
    move.b  #0x00, 0x00020001
    move.w  #0x000a, %ccr                 | CCR=NV again
    moveq   #0, %d0
    lea     0x00020001, %a0
    bchg    %d0, (%a0)
    move.w  %sr, %d3
    and.l   #0x1f, %d3
    cmp.l   #0x0e, %d3                    | expect NVZ
    beq     _pass
    move.l  #0xDEAD0004, %d7
    bra     _fail

_pass:
    lea     0xFFFF0000, %a0
    move.l  #0xC0FFEE00, %d0
    move.l  %d0, (%a0)
_halt:
    bra     _halt

_fail:
    lea     0xFFFF0000, %a0
    move.l  %d7, (%a0)
_halt_fail:
    bra     _halt_fail
