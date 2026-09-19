module tb_decode_ea_helpers (
    input  wire [2:0] ea_mode,
    input  wire [2:0] ea_reg,
    input  wire [1:0] ea_size,
    input  wire [15:0] ea_ext,
    output wire [3:0] stride,
    output wire [3:0] ext_words,
    output wire       data_alterable,
    output wire       memory_alterable,
    output wire       control,
    output wire       control_alterable,
    output wire       an_ind_or_d16,
    output wire       immediate,
    output wire       pc_relative
);
`include "decode_ea.vh"

    assign stride = decode_ea_stride_bytes(ea_size, ea_reg);
    assign ext_words = decode_ea_ext_words(ea_mode, ea_reg, ea_size, ea_ext);
    assign data_alterable = decode_ea_is_data_alterable(ea_mode, ea_reg);
    assign memory_alterable = decode_ea_is_memory_alterable(ea_mode, ea_reg);
    assign control = decode_ea_is_control(ea_mode, ea_reg);
    assign control_alterable = decode_ea_is_control_alterable(ea_mode, ea_reg);
    assign an_ind_or_d16 = decode_ea_is_an_ind_or_d16(ea_mode, ea_reg);
    assign immediate = decode_ea_is_immediate(ea_mode, ea_reg);
    assign pc_relative = decode_ea_is_pc_relative(ea_mode, ea_reg);
endmodule
