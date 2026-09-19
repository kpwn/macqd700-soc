// exception_uop_gen_wrap.v — thin wrapper module that exposes the
// pure helper functions in `rtl/core/exception_uop_gen.vh` as
// combinational outputs.  Used by `tb/tb_exception_uop_gen.cpp` to
// validate the µop-bundle field generators in isolation, without
// having to instantiate the full exception sequencer or the rest of
// the back-end.
//
// All outputs are pure functions of the inputs — no clocked state.
// The inputs mirror the relevant fields of `exception.v`'s sequencer
// state at the moment a µop would be injected.

`include "uop_pkg.v"

module exception_uop_gen_wrap (
    input  wire [4:0]  push_word_idx,
    input  wire        is_fmt7,
    input  wire        is_fmt2,
    input  wire [15:0] cur_sr,
    input  wire [31:0] cur_fault_pc,
    input  wire [31:0] cur_fault_addr,
    input  wire [15:0] format_vec_word,
    input  wire [15:0] fmt7_ssw,
    input  wire [31:0] cur_a7_new,
    input  wire [31:0] cur_vbr,
    input  wire [7:0]  cur_vec,
    input  wire        cur_is_m_irq,
    input  wire [31:0] cur_isp_new,
    input  wire [31:0] cur_msp_new,

    output wire [4:0]  o_push_count,
    output wire [2:0]  o_uop_type,
    output wire [1:0]  o_uop_size,
    output wire [31:0] o_uop_addr,
    output wire [31:0] o_uop_data,
    output wire        o_imm_is_data,
    output wire [2:0]  o_fc_override,
    output wire        o_is_finalize,
    output wire        o_is_store,
    output wire        o_is_load,
    output wire [31:0] o_inject_a7_new
);

    `include "exception_uop_gen.vh"

    wire [4:0] push_count_w = exc_uop_push_count(is_fmt7, is_fmt2, cur_is_m_irq);

    assign o_push_count   = push_count_w;
    assign o_uop_type     = exc_uop_type(push_word_idx, push_count_w);
    assign o_uop_size     = exc_uop_size(push_word_idx, push_count_w);
    assign o_uop_addr     = exc_uop_addr(push_word_idx, push_count_w,
                                         cur_a7_new, cur_vbr, cur_vec,
                                         cur_is_m_irq, cur_isp_new, cur_msp_new);
    assign o_uop_data     = exc_uop_data(push_word_idx, push_count_w,
                                         cur_sr, cur_fault_pc, cur_fault_addr,
                                         cur_fault_pc,
                                         format_vec_word, fmt7_ssw,
                                         is_fmt2, is_fmt7, cur_is_m_irq);
    assign o_imm_is_data  = exc_uop_imm_is_data(push_word_idx, push_count_w);
    assign o_fc_override  = exc_uop_fc_override(1'b0);
    assign o_is_finalize  = exc_uop_is_finalize(push_word_idx, push_count_w);
    assign o_is_store     = (push_word_idx <  push_count_w);
    assign o_is_load      = (push_word_idx >= push_count_w);
    assign o_inject_a7_new = cur_a7_new;

endmodule
