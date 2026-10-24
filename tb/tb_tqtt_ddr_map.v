`include "axi_defs.vh"

module tb_tqtt_ddr_map;
    localparam [32:0] DDR_BYTES = 33'h0_4000_0000;
    localparam [32:0] RAM_END = {1'b0, `AXI_DDR_RAM_OFFSET} + `AXI_RAM_SIZE;
    localparam [32:0] ROM_START = {1'b0, `AXI_DDR_ROM_OFFSET};
    localparam [32:0] ROM_END = ROM_START + `AXI_ROM_SIZE;
    localparam [32:0] FB_START = {1'b0, `AXI_DDR_FB_OFFSET};
    localparam [32:0] FB_END = FB_START + `AXI_FB_SIZE;

    initial begin
        if ((`AXI_RAM_SIZE & (`AXI_RAM_SIZE - 1)) != 0)
            $fatal(1, "RAM_SIZE must remain a power of two for axi_xbar decode");
        if (!(RAM_END <= ROM_START && ROM_END <= FB_START && FB_END <= DDR_BYTES))
            $fatal(1, "TQTT DDR RAM, ROM and FB ranges overlap or exceed 1 GiB");
        $display("TQTT DDR map: RAM end=%h ROM=%h..%h FB=%h..%h", RAM_END,
                 ROM_START, ROM_END, FB_START, FB_END);
        $finish;
    end
endmodule
