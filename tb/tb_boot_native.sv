`timescale 1ns/1ps
// Exercise the native boot stream across a 32-sector DMA boundary while the
// AXI writer is slowed enough to require genuine byte-stream back-pressure.
module tb_boot_native;
    reg clk = 0;
    always #5 clk = ~clk;
    reg rst = 1;
    integer cycles = 0;
    always @(posedge clk) cycles <= cycles + 1;

    wire native_go;
    wire [31:0] native_lba;
    wire [15:0] native_block_count;
    wire native_rd_ready;
    reg native_busy = 0;
    reg native_done = 0;
    reg native_error = 0;
    reg native_ready = 1;
    reg native_init_error = 0;
    reg [31:0] native_card_lbas = 32'd8388608;
    reg native_active = 0;
    reg [31:0] native_base = 0;
    integer native_pos = 0;
    integer native_total = 0;
    integer requests = 0;
    integer stalled_bytes = 0;
    wire native_rd_valid = native_active && native_rd_ready;
    wire [7:0] native_rd_data = ((native_base + (native_pos / 512)) * 37 +
                                 (native_pos % 512)) & 8'hff;

    always @(posedge clk) begin
        if (rst) begin
            native_busy <= 0;
            native_done <= 0;
            native_active <= 0;
            native_pos <= 0;
            native_total <= 0;
            requests <= 0;
            stalled_bytes <= 0;
        end else begin
            native_done <= 0;
            if (native_go) begin
                if (native_active || native_busy) $fatal(1, "overlapping native request");
                if (requests == 0 && (native_lba != 0 || native_block_count != 32))
                    $fatal(1, "first native request LBA=%0d count=%0d", native_lba, native_block_count);
                if (requests == 1 && (native_lba != 32 || native_block_count != 8))
                    $fatal(1, "second native request LBA=%0d count=%0d", native_lba, native_block_count);
                if (requests > 1) $fatal(1, "extra native request");
                native_base <= native_lba;
                native_total <= native_block_count * 512;
                native_pos <= 0;
                native_active <= 1;
                native_busy <= 1;
                requests <= requests + 1;
            end else if (native_active) begin
                if (!native_rd_ready) stalled_bytes <= stalled_bytes + 1;
                if (native_rd_valid) begin
                    native_pos <= native_pos + 1;
                    if (native_pos + 1 == native_total) begin
                        native_active <= 0;
                        native_busy <= 0;
                        native_done <= 1;
                    end
                end
            end
        end
    end

    wire [3:0] awid;
    wire [31:0] awaddr;
    wire [7:0] awlen;
    wire [2:0] awsize;
    wire [1:0] awburst;
    wire awvalid;
    wire [31:0] wdata;
    wire [3:0] wstrb;
    wire wlast, wvalid, bready;
    reg bvalid = 0;
    integer bdelay = 0;
    integer aw_count = 0;
    integer w_count = 0;
    wire rom_loaded, boot_error, crc_enabled;
    wire [31:0] card_lbas;
    wire [5:0] boot_state;
    function automatic [7:0] expected_byte(input integer word_index, input integer lane);
        integer sector_index;
        integer sector_byte;
        begin
            sector_index = word_index / 128;
            sector_byte = (word_index % 128) * 4 + lane;
            expected_byte = (sector_index * 37 + sector_byte) & 8'hff;
        end
    endfunction
    wire [31:0] expected_word = {
        expected_byte(w_count, 0), expected_byte(w_count, 1),
        expected_byte(w_count, 2), expected_byte(w_count, 3)
    };

    always @(posedge clk) begin
        if (rst) begin
            bvalid <= 0;
            bdelay <= 0;
            aw_count <= 0;
            w_count <= 0;
        end else begin
            if (awvalid) begin
                if (awlen != 0 || awsize != 2 || awburst != 1 ||
                    awaddr != 32'h4000_0000 + aw_count * 4)
                    $fatal(1, "bad AXI address/shape at word %0d", aw_count);
                aw_count <= aw_count + 1;
            end
            if (wvalid) begin
                if (wdata != expected_word || wstrb != 4'hf || !wlast)
                    $fatal(1, "bad AXI data word %0d got=%08x expected=%08x", w_count, wdata, expected_word);
                w_count <= w_count + 1;
                bdelay <= 12;
            end else if (bdelay != 0) begin
                bdelay <= bdelay - 1;
                if (bdelay == 1) bvalid <= 1;
            end
            if (bvalid && bready) bvalid <= 0;
        end
    end

    boot_fsm #(.NUM_SECTORS(32'd40), .ZERO_BYTES(32'd0)) dut (
        .clk(clk), .rst(rst), .zero_en(1'b0),
        .cfg_skip_cmd59(1'b0), .cfg_force_hs(1'b0),
        .native_ready(native_ready), .native_init_error(native_init_error),
        .native_card_lbas(native_card_lbas),
        .native_go(native_go), .native_lba(native_lba),
        .native_block_count(native_block_count),
        .native_busy(native_busy), .native_done(native_done),
        .native_error(native_error), .native_rd_valid(native_rd_valid),
        .native_rd_data(native_rd_data), .native_rd_ready(native_rd_ready),
        .spi_cmd_ready(1'b0), .spi_rsp_valid(1'b0), .spi_rsp_data(8'hff),
        .m_axi_awid(awid), .m_axi_awaddr(awaddr), .m_axi_awlen(awlen),
        .m_axi_awsize(awsize), .m_axi_awburst(awburst),
        .m_axi_awvalid(awvalid), .m_axi_awready(1'b1),
        .m_axi_wdata(wdata), .m_axi_wstrb(wstrb), .m_axi_wlast(wlast),
        .m_axi_wvalid(wvalid), .m_axi_wready(1'b1),
        .m_axi_bid(4'd0), .m_axi_bresp(2'd0),
        .m_axi_bvalid(bvalid), .m_axi_bready(bready),
        .rom_loaded(rom_loaded), .error(boot_error),
        .sd_crc_enabled(crc_enabled), .card_num_lbas(card_lbas),
        .dbg_st(boot_state)
    );

    initial begin
        repeat (5) @(negedge clk);
        rst = 0;
        wait (rom_loaded || boot_error || cycles > 500000);
        if (boot_error || !rom_loaded) $fatal(1, "native boot failed state=%0d", boot_state);
        if (requests != 2 || aw_count != 5120 || w_count != 5120 ||
            stalled_bytes == 0 || card_lbas != 8388608 || !crc_enabled)
            $fatal(1, "native boot mismatch req=%0d AW=%0d W=%0d stalls=%0d capacity=%0d",
                   requests, aw_count, w_count, stalled_bytes, card_lbas);
        $display("PASS native boot: 40 sectors, 2 chunks, %0d AXI words, %0d back-pressure cycles",
                 w_count, stalled_bytes);

        @(negedge clk);
        rst = 1;
        native_ready = 0;
        native_init_error = 1;
        repeat (5) @(negedge clk);
        rst = 0;
        repeat (10) @(negedge clk);
        if (!boot_error || rom_loaded || requests != 0)
            $fatal(1, "native init error did not fail boot closed");

        rst = 1;
        native_ready = 1;
        native_init_error = 0;
        native_card_lbas = 39;
        repeat (5) @(negedge clk);
        rst = 0;
        repeat (10) @(negedge clk);
        if (!boot_error || rom_loaded || requests != 0)
            $fatal(1, "undersized native card did not fail boot closed");
        $display("PASS native boot init-error and capacity guards");
        $finish;
    end
endmodule
