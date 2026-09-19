| dafb_vram_pattern.s -- JTAG-loadable first-light ROM smoke.
|
| fpga_top resets the CPU to 0x4000002A so this flat binary carries a small
| vector/header pad and places the first instruction at offset 0x2A.  Load it
| at 0x40000000 through synth/jtag_bringup.tcl rom-load.
|
| The program:
|   1. Programs the live DAFB scanout latches for VRAM base 0, stride 1024,
|      and Q700-ROM-style 8bpp selector 0x30.
|   2. Re-seeds CLUT entries 0..15 with the shim's simple colour codes.
|   3. Writes a full 1024x768 8bpp 32x32 black/white checkerboard to VRAM.
|   4. Parks with D0 = 0x600DDAFB for debug visibility.

    .ifndef FB_BLOCKS_X
    .set FB_BLOCKS_X, 32      | 1024 px / 32 px per tile
    .endif
    .ifndef FB_ROWS
    .set FB_ROWS, 768
    .endif
    .ifndef TILE_ROWS
    .set TILE_ROWS, 32
    .endif
    .ifndef TILE_WORDS
    .set TILE_WORDS, 8        | 32 bytes per tile / 4 bytes per longword
    .endif

    .text
    .org 0
    .long   0x00010000        | conventional initial SP if a reset-vector path reads it
    .long   0x4000002a        | absolute entry for future reset-vector based runs

    .org 0x2a
    .globl _start
_start:
    | DAFB scanout placement: framebuffer base, stride, and 8bpp selector.
    lea     0xF9800008, %a1
    moveq   #0, %d0
    move.l  %d0, (%a1)
    lea     0xF980000C, %a1
    move.l  #0x00000400, %d0
    move.l  %d0, (%a1)
    lea     0xF9800010, %a1
    moveq   #0x30, %d0
    move.l  %d0, (%a1)

    | CLUT entries.  The DAFB shim decodes the low nibble as a fixed RGB code.
    lea     0xF9800300, %a1
    moveq   #0x0, %d0
    move.l  %d0, (%a1)
    lea     0xF9800310, %a1
    moveq   #0x1, %d0
    move.l  %d0, (%a1)
    lea     0xF9800320, %a1
    moveq   #0x2, %d0
    move.l  %d0, (%a1)
    lea     0xF9800330, %a1
    moveq   #0x3, %d0
    move.l  %d0, (%a1)
    lea     0xF9800340, %a1
    moveq   #0x4, %d0
    move.l  %d0, (%a1)
    lea     0xF9800350, %a1
    moveq   #0x5, %d0
    move.l  %d0, (%a1)
    lea     0xF9800360, %a1
    moveq   #0x6, %d0
    move.l  %d0, (%a1)
    lea     0xF9800370, %a1
    moveq   #0x7, %d0
    move.l  %d0, (%a1)
    lea     0xF9800380, %a1
    moveq   #0x8, %d0
    move.l  %d0, (%a1)
    lea     0xF9800390, %a1
    moveq   #0x9, %d0
    move.l  %d0, (%a1)
    lea     0xF98003A0, %a1
    moveq   #0xA, %d0
    move.l  %d0, (%a1)
    lea     0xF98003B0, %a1
    moveq   #0xB, %d0
    move.l  %d0, (%a1)
    lea     0xF98003C0, %a1
    moveq   #0xC, %d0
    move.l  %d0, (%a1)
    lea     0xF98003D0, %a1
    moveq   #0xD, %d0
    move.l  %d0, (%a1)
    lea     0xF98003E0, %a1
    moveq   #0xE, %d0
    move.l  %d0, (%a1)
    lea     0xF98003F0, %a1
    moveq   #0xF, %d0
    move.l  %d0, (%a1)

    | Full-frame 32x32 checkerboard in 8bpp VRAM.
    lea     0xF9000000, %a0
    moveq   #0, %d6
    move.w  #(TILE_ROWS - 1), %d5
    move.w  #(FB_ROWS - 1), %d0

row_loop:
    move.l  %d6, %d3
    move.w  #(FB_BLOCKS_X - 1), %d1

block_loop:
    move.w  #(TILE_WORDS - 1), %d2

word_loop:
    move.l  %d3, (%a0)+
    dbra    %d2, word_loop

    not.l   %d3
    dbra    %d1, block_loop

    dbra    %d5, same_row_tile
    not.l   %d6
    move.w  #(TILE_ROWS - 1), %d5

same_row_tile:
    dbra    %d0, row_loop

done:
    move.l  #0x600DDAFB, %d0

park:
    bra     park
