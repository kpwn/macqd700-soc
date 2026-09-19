// l2c_defs.vh — shared geometry/encoding constants for the l2c_* module
// family (rtl/soc/l2c.v + l2c_tags.v/l2c_data.v/l2c_mshr.v/l2c_victim.v/
// l2c_bypass.v/l2c_reset.v).  See docs/l2c_spec.md for the full picture.
//
// Geometry is fixed for v1: 2 MB, 8-way, 64 B line -> 4096 sets.  Not
// parameterized further because the brief pins this exact geometry; a
// later task can lift these to module parameters if a different L2 size
// is ever needed.
//
// NOTE: only `define constants live here (safe to include-guard -- a
// `define is global preprocessor state, one definition is correct no
// matter how many modules include this file).  The l2c_plru_* pure
// FUNCTIONS live in the separate, deliberately UNGUARDED
// `l2c_plru_funcs.vh` -- Verilog functions are module-scoped, so each
// module that calls them needs its own textual copy; an include guard
// on that file would mean only the *first* module to `include it in the
// whole compilation actually gets the function bodies, silently leaving
// every other includer with an undefined-function compile error.

`ifndef L2C_DEFS_VH
`define L2C_DEFS_VH

`define L2C_WAYS        8
`define L2C_WAY_BITS    3
`define L2C_SETS        4096
`define L2C_SET_BITS    12
`define L2C_LINE_BYTES  64
`define L2C_OFF_BITS    6
`define L2C_TAG_BITS    14
`define L2C_LINE_BITS   512
`define L2C_QUAD_BITS   2      // addr[5:4] -- which 128b quadrant of a line

`define L2C_MSHR_N      8
`define L2C_MSHR_BITS   3
`define L2C_REPLAY_N    4
`define L2C_REPLAY_BITS 2

`define L2C_VICTIM_N    2

// Fixed AXI-master-port ID tagging so top-level response routing can tell
// which sub-engine issued a given outstanding transaction without a CAM:
// bit [ID_WIDTH-1] of the ID driven on the shared physical master port is
// 1 for bypass-engine traffic, 0 for core (mshr-fill / victim-writeback)
// traffic.  See docs/l2c_spec.md S5/S6.  Each module computes this bit
// inline as {1'b1, {(ID_WIDTH-1){1'b0}}} rather than via a macro, to keep
// plain Verilog-2005 tooling happy.

`endif // L2C_DEFS_VH
