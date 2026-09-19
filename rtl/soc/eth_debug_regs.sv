// eth_debug_regs.sv -- optional low-cost SONIC/Taxi JTAG telemetry page.
//
// This block is instantiated only with ETH_DEBUG_ENABLE.  It is deliberately
// counter/snapshot based: no wide VIO fanout and no ILA BRAM tax.  The host
// reaches it through the upper half of the 0x5090_0000 debug service BAR;
// the canonical address is 0x5098_0000.
`default_nettype none

module eth_debug_regs (
    input  wire        clk,
    input  wire        rst,

    input  wire [19:0] awaddr,
    input  wire        awvalid,
    output wire        awready,
    input  wire [31:0] wdata,
    input  wire [3:0]  wstrb,
    input  wire        wvalid,
    output wire        wready,
    output wire [1:0]  bresp,
    output reg         bvalid,
    input  wire        bready,
    input  wire [19:0] araddr,
    input  wire        arvalid,
    output wire        arready,
    output reg  [31:0] rdata,
    output wire [1:0]  rresp,
    output reg         rvalid,
    input  wire        rready,

    // Runtime SONIC receive filter override.  This is intentionally exposed
    // through the debug page rather than the architectural SONIC register
    // block so bring-up can accept broadcasts/all destinations without
    // changing the guest-visible device model.
    output reg         promisc_enable,

    // ── SONIC register-access trace-ring readout ─────────────────────────
    // (rtl/soc/sonic_trace_ring.v).  Parked in this page rather than in a
    // slave of its own for the same reason the 53C96 ring lives in
    // vhdd_ctrl: this block is already an AXI-Lite terminator on core_clk
    // with a whole BAR half and <0x80 of it used, so adding an xbar slave
    // for a debug ring would be a far larger change for no benefit.
    //
    // TRACE_CTRL (0x80) bit0 = freeze (level; sticky inside the ring until
    // cleared), bit1 = clear + re-arm (a "do it" bit: writing 1 flips the
    // toggle the ring edge-detects).  Reads give {frozen, wrapped, wr_ptr}.
    // TRACE_ADDR (0x84) sets the RAM index; TRACE_DATA (0x88) returns
    // ring[TRACE_ADDR].  TRACE_DATA does NOT auto-increment: the RAM read is
    // registered, so an auto-increment would return the PREVIOUS index's
    // word and every dump would be off by one -- a silent, plausible-looking
    // corruption.  The host writes the address then reads the data, one pair
    // per entry (tools/jtag_repl.tcl `sonic-trace`).
    // TRACE_FILT (0x8c) is the live suppressed-poll total; it is meaningful
    // WITHOUT freezing and is what distinguishes "driver still spinning on
    // ISR" from "driver stopped touching the SONIC".
    output reg  [11:0] trace_rd_addr,
    input  wire [31:0] trace_rd_data,
    input  wire [11:0] trace_wrptr,
    input  wire [15:0] trace_filtered,
    input  wire        trace_wrapped,
    input  wire        trace_frozen,
    output reg         trace_freeze,
    output reg         trace_clear,     // TOGGLE, not a pulse

    // Slow/status CDC inputs.  Synchronizers live here so producers remain
    // free of debug-only timing loads when this module is absent.
    input  wire [1:0]  link_speed_async,
    input  wire [7:0]  mac_event_toggle_async,
    input  wire        sonic_irq_async,
    input  wire        sonic_rx_enable_async,
    input  wire [15:0] sonic_cr_async,
    input  wire [15:0] sonic_dcr_async,
    input  wire [15:0] sonic_imr_async,
    input  wire [15:0] sonic_isr_async,

    input  wire [3:0]  tx_state,
    input  wire [4:0]  rx_state,
    input  wire [31:0] tx_descriptor_addr,
    input  wire [31:0] rx_descriptor_addr,
    input  wire [15:0] rx_frame_len,
    input  wire [15:0] rx_rcr,
    input  wire [15:0] rx_cam_enable,
    input  wire [47:0] rx_cam_entry,
    output reg  [3:0]  cam_index,
    input  wire        tx_cmd_fire,
    input  wire        tx_done_fire,
    input  wire        tx_done_error,
    input  wire        rx_done_fire,
    input  wire        rx_done_error,
    input  wire        tx_axis_fire,
    input  wire        tx_axis_last,
    input  wire        rx_axis_fire,
    input  wire        rx_axis_last,
    input  wire        rx_axis_user,

    input  wire [1:0]  dma_req_fire,
    input  wire [63:0] dma_req_addr,
    input  wire [13:0] dma_req_len,
    input  wire [15:0] dma_req_tag,
    input  wire [1:0]  dma_req_write,
    input  wire [1:0]  dma_rsp_fire,
    input  wire [3:0]  dma_rsp_status,
    input  wire [15:0] dma_rsp_tag,
    input  wire [1:0]  dma_rsp_write
);
    localparam [31:0] IDENT = 32'h4554_4801; // "ETH", ABI v1

    (* ASYNC_REG="TRUE" *) reg [1:0] link_meta, link_sync;
    (* ASYNC_REG="TRUE" *) reg [7:0] mac_tog_meta, mac_tog_sync;
    reg [7:0] mac_tog_last;
    (* ASYNC_REG="TRUE" *) reg irq_meta, irq_sync, rxen_meta, rxen_sync;
    (* ASYNC_REG="TRUE" *) reg [15:0] cr_meta, cr_sync;
    (* ASYNC_REG="TRUE" *) reg [15:0] dcr_meta, dcr_sync;
    (* ASYNC_REG="TRUE" *) reg [15:0] imr_meta, imr_sync;
    (* ASYNC_REG="TRUE" *) reg [15:0] isr_meta, isr_sync;
    wire [7:0] mac_event = mac_tog_sync ^ mac_tog_last;

    reg [19:0] awaddr_q;
    reg aw_hold;
    reg [31:0] wdata_q;
    reg [3:0] wstrb_q;
    reg w_hold;
    wire aw_fire = awvalid && awready;
    wire w_fire = wvalid && wready;
    wire write_commit = !bvalid && (aw_hold || aw_fire) &&
                        (w_hold || w_fire);
    wire [19:0] write_addr = aw_hold ? awaddr_q : awaddr;
    wire [31:0] write_data = w_hold ? wdata_q : wdata;
    wire [3:0] write_strb = w_hold ? wstrb_q : wstrb;
    wire clear_stats = write_commit && (write_addr[7:0] == 8'h08) &&
                       write_strb[0] && write_data[0];

    assign awready = !aw_hold && !bvalid;
    assign wready  = !w_hold && !bvalid;
    assign bresp = 2'b00;
    assign arready = !rvalid;
    assign rresp = 2'b00;

    function automatic [15:0] sat_inc16;
        input [15:0] value;
        begin sat_inc16 = (&value) ? value : value + 1'b1; end
    endfunction

    reg [15:0] tx_cmd_count, tx_frame_count, tx_done_count, tx_error_count;
    reg [15:0] rx_frame_count, rx_axis_error_count, rx_done_count, rx_error_count;
    reg [15:0] dma_tx_req_count, dma_rx_req_count;
    reg [15:0] dma_tx_rsp_count, dma_rx_rsp_count;
    reg [15:0] dma_tx_error_count, dma_rx_error_count;
    reg [15:0] mac_count [0:7];
    reg [15:0] tx_byte_count, rx_byte_count;
    reg [15:0] last_tx_frame_len, last_rx_frame_len;
    reg [31:0] last_dma_addr;
    reg [31:0] last_dma_meta;
    reg first_error_valid;
    reg [31:0] first_error_info;
    reg [31:0] first_error_addr;
    integer i;

    wire tx_dma_error = dma_rsp_fire[0] && (dma_rsp_status[1:0] != 0);
    wire rx_dma_error = dma_rsp_fire[1] && (dma_rsp_status[3:2] != 0);

    always @(posedge clk) begin
        if (rst) begin
            link_meta <= 0; link_sync <= 0;
            mac_tog_meta <= 0; mac_tog_sync <= 0; mac_tog_last <= 0;
            irq_meta <= 0; irq_sync <= 0; rxen_meta <= 0; rxen_sync <= 0;
            cr_meta <= 0; cr_sync <= 0; imr_meta <= 0; imr_sync <= 0;
            dcr_meta <= 0; dcr_sync <= 0;
            isr_meta <= 0; isr_sync <= 0;
        end else begin
            link_meta <= link_speed_async; link_sync <= link_meta;
            mac_tog_meta <= mac_event_toggle_async; mac_tog_sync <= mac_tog_meta;
            mac_tog_last <= mac_tog_sync;
            irq_meta <= sonic_irq_async; irq_sync <= irq_meta;
            rxen_meta <= sonic_rx_enable_async; rxen_sync <= rxen_meta;
            cr_meta <= sonic_cr_async; cr_sync <= cr_meta;
            dcr_meta <= sonic_dcr_async; dcr_sync <= dcr_meta;
            imr_meta <= sonic_imr_async; imr_sync <= imr_meta;
            isr_meta <= sonic_isr_async; isr_sync <= isr_meta;
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            awaddr_q <= 0; aw_hold <= 0; wdata_q <= 0; wstrb_q <= 0;
            w_hold <= 0; bvalid <= 0; promisc_enable <= 1'b0; cam_index <= 4'd0;
            trace_rd_addr <= 12'd0; trace_freeze <= 1'b0; trace_clear <= 1'b0;
        end else begin
            if (aw_fire) begin awaddr_q <= awaddr; aw_hold <= 1'b1; end
            if (w_fire) begin wdata_q <= wdata; wstrb_q <= wstrb; w_hold <= 1'b1; end
            if (write_commit) begin
                aw_hold <= 1'b0; w_hold <= 1'b0; bvalid <= 1'b1;
                if ((write_addr[7:0] == 8'h70) && write_strb[0])
                    promisc_enable <= write_data[0];
                // Which CAM entry 0x78/0x7c report.  Its own offset rather
                // than spare bits in 0x70, so selecting an entry cannot
                // disturb the promiscuous setting by read-modify-write.
                if ((write_addr[7:0] == 8'h5c) && write_strb[0])
                    cam_index <= write_data[3:0];
                if ((write_addr[7:0] == 8'h80) && write_strb[0]) begin
                    trace_freeze <= write_data[0];
                    // bit1 is a "do it" bit, not stored state: flip the
                    // toggle the ring edge-detects.
                    if (write_data[1]) trace_clear <= ~trace_clear;
                end
                if ((write_addr[7:0] == 8'h84) && write_strb[0])
                    trace_rd_addr <= write_data[11:0];
            end
            if (bvalid && bready) bvalid <= 1'b0;
        end
    end

    always @(posedge clk) begin
        if (rst || clear_stats) begin
            tx_cmd_count<=0; tx_frame_count<=0; tx_done_count<=0; tx_error_count<=0;
            rx_frame_count<=0; rx_axis_error_count<=0; rx_done_count<=0; rx_error_count<=0;
            dma_tx_req_count<=0; dma_rx_req_count<=0; dma_tx_rsp_count<=0; dma_rx_rsp_count<=0;
            dma_tx_error_count<=0; dma_rx_error_count<=0;
            tx_byte_count<=0; rx_byte_count<=0;
            last_tx_frame_len<=0; last_rx_frame_len<=0;
            last_dma_addr<=0; last_dma_meta<=0;
            first_error_valid<=0; first_error_info<=0; first_error_addr<=0;
            for (i=0; i<8; i=i+1) mac_count[i]<=0;
        end else begin
            if (tx_cmd_fire) tx_cmd_count <= sat_inc16(tx_cmd_count);
            if (tx_done_fire) tx_done_count <= sat_inc16(tx_done_count);
            if (tx_done_fire && tx_done_error) tx_error_count <= sat_inc16(tx_error_count);
            if (rx_done_fire) rx_done_count <= sat_inc16(rx_done_count);
            if (rx_done_fire && rx_done_error) rx_error_count <= sat_inc16(rx_error_count);
            if (tx_axis_fire) begin
                tx_byte_count <= tx_byte_count + 1'b1;
                if (tx_axis_last) begin
                    tx_frame_count <= sat_inc16(tx_frame_count);
                    last_tx_frame_len <= tx_byte_count + 1'b1;
                    tx_byte_count <= 0;
                end
            end
            if (rx_axis_fire) begin
                rx_byte_count <= rx_byte_count + 1'b1;
                if (rx_axis_user) rx_axis_error_count <= sat_inc16(rx_axis_error_count);
                if (rx_axis_last) begin
                    rx_frame_count <= sat_inc16(rx_frame_count);
                    last_rx_frame_len <= rx_byte_count + 1'b1;
                    rx_byte_count <= 0;
                end
            end
            if (dma_req_fire[0]) begin
                dma_tx_req_count <= sat_inc16(dma_tx_req_count);
                last_dma_addr <= dma_req_addr[31:0];
                last_dma_meta <= {1'b0,dma_req_write[0],7'd0,dma_req_tag[7:0],
                                  8'd0,dma_req_len[6:0]};
            end
            if (dma_req_fire[1]) begin
                dma_rx_req_count <= sat_inc16(dma_rx_req_count);
                last_dma_addr <= dma_req_addr[63:32];
                last_dma_meta <= {1'b1,dma_req_write[1],7'd0,dma_req_tag[15:8],
                                  8'd0,dma_req_len[13:7]};
            end
            if (dma_rsp_fire[0]) dma_tx_rsp_count <= sat_inc16(dma_tx_rsp_count);
            if (dma_rsp_fire[1]) dma_rx_rsp_count <= sat_inc16(dma_rx_rsp_count);
            if (tx_dma_error) dma_tx_error_count <= sat_inc16(dma_tx_error_count);
            if (rx_dma_error) dma_rx_error_count <= sat_inc16(dma_rx_error_count);
            for (i=0; i<8; i=i+1)
                if (mac_event[i]) mac_count[i] <= sat_inc16(mac_count[i]);

            if (!first_error_valid) begin
                if (tx_dma_error) begin
                    first_error_valid<=1; first_error_info<={8'h03,6'd0,
                        dma_rsp_write[0],1'b0,dma_rsp_tag[7:0],6'd0,dma_rsp_status[1:0]};
                    first_error_addr<=last_dma_addr;
                end else if (rx_dma_error) begin
                    first_error_valid<=1; first_error_info<={8'h04,6'd0,
                        dma_rsp_write[1],1'b1,dma_rsp_tag[15:8],6'd0,dma_rsp_status[3:2]};
                    first_error_addr<=last_dma_addr;
                end else if (tx_done_fire && tx_done_error) begin
                    first_error_valid<=1; first_error_info<=32'h0100_0000;
                    first_error_addr<=tx_descriptor_addr;
                end else if (rx_done_fire && rx_done_error) begin
                    first_error_valid<=1; first_error_info<=32'h0200_0000;
                    first_error_addr<=rx_descriptor_addr;
                end else if (rx_axis_fire && rx_axis_user) begin
                    first_error_valid<=1; first_error_info<=32'h0500_0000;
                    first_error_addr<=rx_descriptor_addr;
                // Taxi event bits: 0 TX underflow, 1 TX FIFO overflow,
                // 2 TX bad frame, 3 TX GOOD frame, 4 RX bad frame, 5 RX bad
                // FCS, 6 RX FIFO overflow, 7 RX GOOD frame.  The mask used to
                // be 8'b0110_1111, which flagged every GOOD TX frame (bit 3)
                // as the first error and ignored RX bad frames (bit 4) -- so a
                // healthy link reported first_error=1 with info 0x10000008.
                end else if (|(mac_event & 8'b0111_0111)) begin
                    first_error_valid<=1; first_error_info<={8'h10,16'd0,mac_event};
                    first_error_addr<=0;
                end
            end
        end
    end

    function automatic [31:0] read_value;
        input [7:0] addr;
        begin
            case (addr)
                8'h00: read_value=IDENT;
                // caps bit 7 (0x...ff) = the SONIC register-access trace
                // ring at 0x80..0x8c is present.  tools/jtag_repl.tcl
                // `sonic-trace` gates on it so an older debug page cannot
                // be decoded as an empty-but-plausible ring.
                // caps bit 8 (0x...1ff) = CAM entry select at 0x5c, so
                // 0x78/0x7c report ANY of the 16 entries rather than only
                // entry 0.  The driver uses entry 15, which entry 0 could
                // never show.
                8'h04: read_value=32'h0001_01ff; // + RX filter state + trace ring + CAM select
                8'h08: read_value={16'd0,first_error_valid,2'd0,tx_state,
                                   rx_state,rxen_sync,irq_sync,link_sync};
                8'h0c: read_value={cr_sync,imr_sync};
                8'h10: read_value={isr_sync,13'd0,rxen_sync,irq_sync,first_error_valid};
                8'h14: read_value={12'd0,last_tx_frame_len,tx_state};
                8'h18: read_value={11'd0,rx_frame_len,rx_state};
                8'h1c: read_value=tx_descriptor_addr;
                8'h20: read_value=rx_descriptor_addr;
                8'h24: read_value={last_tx_frame_len,last_rx_frame_len};
                8'h28: read_value=last_dma_addr;
                8'h2c: read_value=last_dma_meta;
                8'h30: read_value=first_error_info;
                8'h34: read_value=first_error_addr;
                8'h38: read_value={16'd0,mac_event,mac_tog_sync};
                // Descriptor width lives in DCR bit 5 (DW); the TX/RX
                // engines take it from here, so a 16-bit read of a
                // 32-bit descriptor ring is visible without an ILA.
                8'h3c: read_value={16'd0,dcr_sync};
                8'h40: read_value={tx_cmd_count,tx_frame_count};
                8'h44: read_value={tx_done_count,tx_error_count};
                8'h48: read_value={rx_frame_count,rx_axis_error_count};
                8'h4c: read_value={rx_done_count,rx_error_count};
                8'h50: read_value={dma_tx_req_count,dma_rx_req_count};
                8'h54: read_value={dma_tx_rsp_count,dma_rx_rsp_count};
                8'h58: read_value={dma_tx_error_count,dma_rx_error_count};
                8'h60: read_value={mac_count[0],mac_count[1]};
                8'h64: read_value={mac_count[2],mac_count[3]};
                8'h68: read_value={mac_count[4],mac_count[5]};
                8'h6c: read_value={mac_count[6],mac_count[7]};
                // Receive-admission inputs.  A frame is dropped silently
                // unless one of these admits it, which presents as "RX does
                // not work" with zero DMA activity to explain it.
                8'h74: read_value={rx_rcr,rx_cam_enable};
                8'h5c: read_value={28'd0,cam_index};
                8'h78: read_value=rx_cam_entry[31:0];
                8'h7c: read_value={16'd0,rx_cam_entry[47:32]};
                8'h70: read_value={31'd0,promisc_enable};
                // SONIC register-access trace ring.  See the port block.
                8'h80: read_value={14'd0,trace_frozen,trace_wrapped,
                                   4'd0,trace_wrptr};
                8'h84: read_value={20'd0,trace_rd_addr};
                8'h88: read_value=trace_rd_data;
                8'h8c: read_value={16'd0,trace_filtered};
                default: read_value=32'd0;
            endcase
        end
    endfunction

    always @(posedge clk) begin
        if (rst) begin
            rvalid <= 1'b0;
            rdata <= 32'd0;
        end else begin
            if (arvalid && arready) begin
                rdata <= read_value(araddr[7:0]);
                rvalid <= 1'b1;
            end else if (rvalid && rready) begin
                rvalid <= 1'b0;
            end
        end
    end

endmodule

`default_nettype wire
