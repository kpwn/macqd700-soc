| checkerboard.s -- first-hardware VRAM checkerboard smoke program
|
| Link for hardware at 0x40000000 and place the flat binary wherever the
| current first-board smoke flow loads short reset programs.
|
| The program writes 1024x768 8bpp VRAM at 0xF9000000 as 32x32 black/white
| tiles, then parks in a tight loop with D0 = 0x600D600D for VIO/debug reads.
| Override FB_ROWS in sim to shorten the run while preserving row geometry.

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

    .globl _start
_start:
    lea     0xF9000000, %a0   | A0 walks the VRAM pixel aperture.
    moveq   #0, %d6           | Row-start colour: 0x00000000 first.
    move.w  #(TILE_ROWS - 1), %d5
    move.w  #(FB_ROWS - 1), %d0

row_loop:
    move.l  %d6, %d3          | Current 32-pixel tile colour.
    move.w  #(FB_BLOCKS_X - 1), %d1

block_loop:
    move.w  #(TILE_WORDS - 1), %d2

word_loop:
    move.l  %d3, (%a0)+       | Four 8bpp pixels, all black or all white.
    dbra    %d2, word_loop

    not.l   %d3               | Next 32-pixel tile flips colour.
    dbra    %d1, block_loop

    dbra    %d5, same_row_tile
    not.l   %d6               | Every 32 rows, flip row-start colour.
    move.w  #(TILE_ROWS - 1), %d5

same_row_tile:
    dbra    %d0, row_loop

done:
    move.l  #0x600D600D, %d0

park:
    bra     park
