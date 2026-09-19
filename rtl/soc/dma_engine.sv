// dma_engine.sv -- shared, pipelined, bidirectional byte-granular DMA engine.
//
// One request from any client may be accepted each cycle.  Read and write
// queues are separate so an active AXI W burst cannot block read dispatch.
// Eight shared AXI IDs cover both directions and are never reused while live.
//
// Byte order: clients speak GUEST byte order -- byte i of a request payload or
// a response is the byte at guest_addr+i.  This SoC's memory does not use
// byte-invariant AXI lanes: the 68k LSU places the lowest-address byte of a
// 32-bit datum in bits [31:24] of its lane, so a raw AXI beat holds each
// 32-bit group byte-reversed relative to guest order.  rtl/soc/
// vram_cpu_byteswap.v documents and corrects the same convention on the VRAM
// hop.  Set BYTE_SWAP32 for a memory port that follows it; the engine then
// reverses W/R payload and W strobes within every 32-bit lane so client-facing
// buffers stay in guest order.  Byte-granular (narrow) beats replicate the data
// byte across all lanes, so for those only the strobe permutation matters.
//
// The AXI W channel leaves through an axi_w_skid register slice: WDATA/WSTRB/
// WLAST/WVALID are REGISTERED outputs, one cycle behind the queue state that
// produced them.  AW/AR/B/R are unchanged.  See the "W-channel output register
// slice" note below for the measurement that forced it and the cost.
`default_nettype none
module dma_engine #(
    parameter integer N_CLIENTS = 4,
    parameter integer ADDR_WIDTH = 32,
    parameter integer TAG_WIDTH = 8,
    parameter integer DATA_WIDTH = 128,
    parameter integer ID_WIDTH = 6,
    parameter integer QUEUE_DEPTH = 16,
    parameter integer BYTE_SWAP32 = 0
) (
    input wire clk, input wire rst,
    input wire [N_CLIENTS*ADDR_WIDTH-1:0] req_addr,
    input wire [N_CLIENTS*7-1:0] req_len,
    input wire [N_CLIENTS*512-1:0] req_wdata,
    input wire [N_CLIENTS*TAG_WIDTH-1:0] req_tag,
    input wire [N_CLIENTS-1:0] req_write,
    input wire [N_CLIENTS-1:0] req_valid,
    output wire [N_CLIENTS-1:0] req_ready,
    output wire [N_CLIENTS*512-1:0] rsp_rdata,
    output wire [N_CLIENTS*TAG_WIDTH-1:0] rsp_tag,
    output wire [N_CLIENTS*7-1:0] rsp_len,
    output wire [N_CLIENTS*2-1:0] rsp_status,
    output wire [N_CLIENTS-1:0] rsp_write,
    output wire [N_CLIENTS-1:0] rsp_valid,
    input wire [N_CLIENTS-1:0] rsp_ready,

    output wire [ID_WIDTH-1:0] m_awid, output wire [ADDR_WIDTH-1:0] m_awaddr,
    output wire [7:0] m_awlen, output wire [2:0] m_awsize, output wire [1:0] m_awburst,
    output wire m_awvalid, input wire m_awready,
    output wire [DATA_WIDTH-1:0] m_wdata, output wire [DATA_WIDTH/8-1:0] m_wstrb,
    output wire m_wlast, output wire m_wvalid, input wire m_wready,
    input wire [ID_WIDTH-1:0] m_bid, input wire [1:0] m_bresp,
    input wire m_bvalid, output wire m_bready,
    output wire [ID_WIDTH-1:0] m_arid, output wire [ADDR_WIDTH-1:0] m_araddr,
    output wire [7:0] m_arlen, output wire [2:0] m_arsize, output wire [1:0] m_arburst,
    output wire m_arvalid, input wire m_arready,
    input wire [ID_WIDTH-1:0] m_rid, input wire [DATA_WIDTH-1:0] m_rdata,
    input wire [1:0] m_rresp, input wire m_rlast, input wire m_rvalid, output wire m_rready
);
    localparam integer IDS=8, IW=3, CW=(N_CLIENTS<=1)?1:$clog2(N_CLIENTS);
    localparam integer QW=(QUEUE_DEPTH<=1)?1:$clog2(QUEUE_DEPTH);
    localparam integer BUS_BYTES=DATA_WIDTH/8, OFF_W=$clog2(BUS_BYTES);
    // Unaligned transfers use byte-sized AXI beats; aligned transfers use the
    // full bus width.  Seven bits cover either a 64-byte narrow burst or the
    // width-dependent full-beat count.
    localparam integer BW=7;
    localparam integer WSEL_W=(BUS_BYTES>=64)?1:$clog2(64/BUS_BYTES);
    localparam integer QCW=$clog2(QUEUE_DEPTH+1);
    localparam [2:0] AXI_SIZE=(DATA_WIDTH==128)?3'd4:(DATA_WIDTH==256)?3'd5:3'd6;
    localparam [QCW-1:0] QUEUE_DEPTH_COUNT=QCW'(QUEUE_DEPTH);
    reg [ADDR_WIDTH-1:0] rq_addr[0:QUEUE_DEPTH-1], wq_addr[0:QUEUE_DEPTH-1];
    (* ram_style="block" *) reg [511:0] wq_data[0:QUEUE_DEPTH-1];
    reg [511:0] wq_data_read;
    reg [TAG_WIDTH-1:0] rq_tag[0:QUEUE_DEPTH-1], wq_tag[0:QUEUE_DEPTH-1];
    reg [6:0] rq_len[0:QUEUE_DEPTH-1],wq_len[0:QUEUE_DEPTH-1];
    reg [CW-1:0] rq_client[0:QUEUE_DEPTH-1], wq_client[0:QUEUE_DEPTH-1];
    reg [QW-1:0] rq_wp,rq_rp,wq_wp,wq_aw_rp,wq_w_rp;
    reg [QCW-1:0] rq_count,wq_count,wq_aw_count,wq_issued_count;
    reg [IDS-1:0] id_live,id_write,id_narrow;
    reg [CW-1:0] id_client[0:IDS-1]; reg [TAG_WIDTH-1:0] id_tag[0:IDS-1];
    reg [6:0] id_len[0:IDS-1]; reg [OFF_W-1:0] id_offset[0:IDS-1];
    reg [BW-1:0] id_rbeat[0:IDS-1]; reg [1:0] id_rstatus[0:IDS-1];
    // Beat zero replaces stale contents, so this array needs only the single
    // R-channel write port and maps to distributed RAM.
    (* ram_style="distributed" *) reg [511:0] id_rdata[0:IDS-1];
    reg [BW-1:0] w_beat;
    reg prefer_read;
    reg [CW-1:0] rr_client;
    reg [511:0] rsp_rdata_q;
    reg [TAG_WIDTH-1:0] rsp_tag_q;
    reg [6:0] rsp_len_q;
    reg [1:0] rsp_status_q;
    reg rsp_write_q,rsp_valid_q;
    reg [CW-1:0] rsp_client_q;
    integer i,pick,candidate; reg pick_valid;
    always @* begin
        pick=0; pick_valid=0;
        for(i=0;i<N_CLIENTS;i=i+1) begin
            candidate=int'(rr_client)+i;
            if(candidate>=N_CLIENTS) candidate=candidate-N_CLIENTS;
            if(!pick_valid&&req_valid[candidate]) begin pick=candidate; pick_valid=1; end
        end
    end
    wire picked_write=req_write[pick];
    wire have_id=(id_live!={IDS{1'b1}});
    wire [IW-1:0] free_id=!id_live[0]?0:!id_live[1]?1:!id_live[2]?2:!id_live[3]?3:!id_live[4]?4:!id_live[5]?5:!id_live[6]?6:7;
    wire can_r=(rq_count!=0)&&have_id;
    wire can_w=(wq_aw_count!=0)&&have_id;
    wire select_r=can_r&&(!can_w||prefer_read);
    wire select_w=can_w&&!select_r;
    wire [OFF_W-1:0] rq_off=rq_addr[rq_rp][OFF_W-1:0];
    wire [OFF_W-1:0] wq_aw_off=wq_addr[wq_aw_rp][OFF_W-1:0];
    wire [OFF_W-1:0] wq_w_off=wq_addr[wq_w_rp][OFF_W-1:0];
    wire rq_narrow=(rq_off!=0);
    wire wq_aw_narrow=(wq_aw_off!=0);
    wire wq_w_narrow=(wq_w_off!=0);
    wire [BW-1:0] rq_wide_beats=(rq_len[rq_rp]+BW'(BUS_BYTES-1))>>OFF_W;
    wire [BW-1:0] wq_aw_wide_beats=(wq_len[wq_aw_rp]+BW'(BUS_BYTES-1))>>OFF_W;
    wire [BW-1:0] wq_w_wide_beats=(wq_len[wq_w_rp]+BW'(BUS_BYTES-1))>>OFF_W;
    wire [BW-1:0] rq_beats=rq_narrow ? rq_len[rq_rp] : rq_wide_beats;
    wire [BW-1:0] wq_aw_beats=wq_aw_narrow ? wq_len[wq_aw_rp] : wq_aw_wide_beats;
    wire [BW-1:0] wq_w_beats=wq_w_narrow ? wq_len[wq_w_rp] : wq_w_wide_beats;
    wire [DATA_WIDTH-1:0] w_wide_data=
        wq_data_read[w_beat[WSEL_W-1:0]*DATA_WIDTH +: DATA_WIDTH];
    wire [7:0] w_narrow_byte=wq_data_read[w_beat[5:0]*8 +: 8];
`ifdef DMA_ENGINE_MUTANT_DROP_LAST_BYTE
    wire [6:0] w_effective_len=wq_len[wq_w_rp]-1'b1;
`else
    wire [6:0] w_effective_len=wq_len[wq_w_rp];
`endif
    wire [BW-1:0] w_byte_base=w_beat<<OFF_W;
    wire [OFF_W-1:0] w_narrow_lane=wq_w_off+w_beat[OFF_W-1:0];
    wire [BUS_BYTES-1:0] w_wide_strb;
    genvar strobe_lane;
    generate for(strobe_lane=0;strobe_lane<BUS_BYTES;strobe_lane=strobe_lane+1) begin : g_w_wide_strb
        assign w_wide_strb[strobe_lane]=(w_byte_base+strobe_lane)<w_effective_len;
    end endgenerate
    wire [BUS_BYTES-1:0] w_narrow_strb=(w_beat<w_effective_len) ?
        ({{(BUS_BYTES-1){1'b0}},1'b1}<<w_narrow_lane) : '0;
    // Guest-order <-> memory-lane permutation (see the header note).
    function automatic [DATA_WIDTH-1:0] swap32_data(input [DATA_WIDTH-1:0] d);
        integer w;
        begin
            swap32_data = d;
            if (BYTE_SWAP32 != 0)
                for (w = 0; w < DATA_WIDTH/32; w = w + 1)
                    swap32_data[w*32 +: 32] = {d[w*32 +: 8], d[w*32+8 +: 8],
                                               d[w*32+16 +: 8], d[w*32+24 +: 8]};
        end
    endfunction
    function automatic [BUS_BYTES-1:0] swap32_strb(input [BUS_BYTES-1:0] b);
        integer w;
        begin
            swap32_strb = b;
            if (BYTE_SWAP32 != 0)
                for (w = 0; w < BUS_BYTES/4; w = w + 1)
                    swap32_strb[w*4 +: 4] = {b[w*4], b[w*4+1], b[w*4+2], b[w*4+3]};
        end
    endfunction
    wire [DATA_WIDTH-1:0] m_rdata_g = swap32_data(m_rdata);

    assign m_arid={{(ID_WIDTH-IW){1'b0}},free_id};
    assign m_araddr=rq_addr[rq_rp];
    assign m_arlen={1'b0,rq_beats}-1'b1; assign m_arsize=rq_narrow?3'd0:AXI_SIZE; assign m_arburst=2'b01;
    assign m_arvalid=select_r;
    assign m_awid={{(ID_WIDTH-IW){1'b0}},free_id};
    assign m_awaddr=wq_addr[wq_aw_rp];
    assign m_awlen={1'b0,wq_aw_beats}-1'b1; assign m_awsize=wq_aw_narrow?3'd0:AXI_SIZE; assign m_awburst=2'b01;
    assign m_awvalid=select_w;
    // ── W-channel output register slice (2026-09-16, FMax) ────────────
    //
    // WHY.  Everything below this point is computed live from the write
    // queue: `wq_data` is a BRAM whose CLKARDCLK->DOUT alone costs ~1.1 ns,
    // and the strobe/last cone walks wq_w_rp through the wq_len/wq_addr
    // distributed RAMs and the byte-count comparators -- 6 to 11 logic
    // levels.  In the Quadra 700 SoC that cone then had to CROSS THE DIE:
    // the engine places around SLICE_X55..X68 Y30..Y38, while its three
    // W-channel consumers sit at opposite corners -- the MIG write FIFO
    // near X20Y55, the peripheral-bus CDC bridge near X37Y40 and the vhdd
    // controller near X77Y86.  `axi_xbar.v` broadcasts every master's
    // WDATA/WSTRB to a 4:1 mux at EACH slave port, so one unregistered
    // source drove all three.
    //
    // Measured on build/vivado200_fmax2 post_route_holdclean.dcp at
    // 200 MHz: 41 of the design's 1753 failing setup endpoints started
    // here, including the design's WNS (-0.138 ns, 9 levels, logic
    // 0.785 ns / route 4.129 ns = 84% wire).  Of that worst path 2.18 ns
    // was spent inside this module and 2.73 ns getting out of it, the last
    // hop alone being a 1.245 ns fanout-1 jump from a crossbar mux LUT at
    // SLICE_X80Y61 into the CDC bridge's FIFO at SLICE_X37Y40.
    //
    // Replication cannot fix this and was not tried again: the pointer was
    // ALREADY replicated (`wq_w_rp_reg_rep[1]__0`) and still carried
    // 4.129 ns of route.  See rtl/soc/l2c_ctrl.v:916 -- a wire cannot be
    // shortened by copying the gate on the far end of it.  A register can,
    // because it gives each half its own clock period.
    //
    // COST: one cycle of W-channel LATENCY per burst.  Throughput is
    // unchanged in steady state (the slice accepts a beat every cycle when
    // the downstream is ready); a backpressure episode costs one bubble as
    // the skid drains.  AW is NOT delayed, so W still trails AW and no AXI
    // ordering rule is touched.  The queue accounting is safe across the
    // slice because it copies the payload -- freeing and re-filling
    // wq_data[wq_w_rp] cannot corrupt a beat already handed over.
    wire [DATA_WIDTH-1:0] w_wdata_c=swap32_data(wq_w_narrow ? {BUS_BYTES{w_narrow_byte}}
                                                            : w_wide_data);
    wire [BUS_BYTES-1:0] w_wstrb_c=swap32_strb(wq_w_narrow ? w_narrow_strb : w_wide_strb);
    wire w_wlast_c=(w_beat==wq_w_beats-1'b1);
    wire w_wvalid_c=(wq_issued_count!=0);
    wire w_wready_c;
    axi_w_skid #(
        .DATA_WIDTH(DATA_WIDTH),
        .STRB_WIDTH(BUS_BYTES)
    ) u_w_slice (
        .clk(clk), .rst(rst),
        .s_wdata (w_wdata_c ), .s_wstrb (w_wstrb_c ),
        .s_wlast (w_wlast_c ), .s_wvalid(w_wvalid_c),
        .s_wready(w_wready_c),
        .m_wdata (m_wdata   ), .m_wstrb (m_wstrb   ),
        .m_wlast (m_wlast   ), .m_wvalid(m_wvalid  ),
        .m_wready(m_wready  )
    );
    wire [CW-1:0] b_client=id_client[m_bid[IW-1:0]], r_client=id_client[m_rid[IW-1:0]];
    wire rsp_slot_ready=!rsp_valid_q||rsp_ready[rsp_client_q];
    assign m_bready=rsp_slot_ready;
    assign m_rready=!m_rvalid||!m_rlast||
        (rsp_slot_ready&&!(m_bvalid&&m_bready));
    wire [511:0] r_prior=(id_rbeat[m_rid[IW-1:0]]==0) ? '0 : id_rdata[m_rid[IW-1:0]];
    wire [OFF_W-1:0] r_narrow_lane=
        id_offset[m_rid[IW-1:0]]+id_rbeat[m_rid[IW-1:0]][OFF_W-1:0];
    wire [7:0] r_narrow_byte=m_rdata_g[r_narrow_lane*8 +: 8];
    wire [511:0] r_wide_chunk={{(512-DATA_WIDTH){1'b0}},m_rdata_g} <<
        (id_rbeat[m_rid[IW-1:0]][WSEL_W-1:0]*DATA_WIDTH);
    wire [511:0] r_narrow_chunk={{504{1'b0}},r_narrow_byte} <<
        (id_rbeat[m_rid[IW-1:0]]*8);
    wire [511:0] r_current=r_prior |
        (id_narrow[m_rid[IW-1:0]] ? r_narrow_chunk : r_wide_chunk);
    wire [511:0] r_payload;
    genvar payload_byte;
    generate for(payload_byte=0;payload_byte<64;payload_byte=payload_byte+1) begin : g_r_payload
        assign r_payload[payload_byte*8 +: 8]=
            (payload_byte<id_len[m_rid[IW-1:0]]) ?
                r_current[payload_byte*8 +: 8] : 8'b0;
    end endgenerate
    assign rsp_rdata={N_CLIENTS{rsp_rdata_q}};
    assign rsp_tag={N_CLIENTS{rsp_tag_q}};
    assign rsp_len={N_CLIENTS{rsp_len_q}};
    assign rsp_status={N_CLIENTS{rsp_status_q}};
    assign rsp_write=rsp_valid_q&&rsp_write_q ?
        ({{(N_CLIENTS-1){1'b0}},1'b1}<<rsp_client_q) : '0;
    assign rsp_valid=rsp_valid_q ?
        ({{(N_CLIENTS-1){1'b0}},1'b1}<<rsp_client_q) : '0;
    wire r_issue=m_arvalid&&m_arready, aw_issue=m_awvalid&&m_awready;
    // The engine's own view of the W channel is the slice's INPUT side.
    wire w_done=w_wvalid_c&&w_wready_c&&w_wlast_c;
    wire picked_space=picked_write ? ((wq_count!=QUEUE_DEPTH_COUNT)||w_done) :
                                     ((rq_count!=QUEUE_DEPTH_COUNT)||r_issue);
    wire req_fire=pick_valid&&picked_space;
`ifndef SYNTHESIS
    initial begin
        if(DATA_WIDTH!=128&&DATA_WIDTH!=256&&DATA_WIDTH!=512) $error("dma_engine DATA_WIDTH must be 128/256/512");
        if(ID_WIDTH<IW) $error("dma_engine ID_WIDTH must be at least 3");
        if(QUEUE_DEPTH<2||(QUEUE_DEPTH&(QUEUE_DEPTH-1))!=0) $error("dma_engine QUEUE_DEPTH must be a power of two >= 2");
    end
    always @(posedge clk) if(!rst&&req_fire&&
        (req_len[pick*7 +: 7]==0 || req_len[pick*7 +: 7]>64))
        $error("dma_engine request length must be 1..64 bytes");
    always @(posedge clk) if(!rst) begin
        if(m_bvalid&&((m_bid>>IW)!=0 || !id_live[m_bid[IW-1:0]] || !id_write[m_bid[IW-1:0]]))
            $error("dma_engine received B for an invalid/non-write ID");
        if(m_rvalid&&((m_rid>>IW)!=0 || !id_live[m_rid[IW-1:0]] || id_write[m_rid[IW-1:0]]))
            $error("dma_engine received R for an invalid/non-read ID");
    end
`endif
    // Synchronous BRAM read with last-beat look-ahead.  Sampling the next
    // queue slot on the edge that retires this burst makes its payload
    // available for the following cycle, so BRAM costs no W-channel bubble.
    wire [QW-1:0] wq_data_read_addr=w_done ? (wq_w_rp+1'b1) : wq_w_rp;
    always @(posedge clk) begin
        wq_data_read<=wq_data[wq_data_read_addr];
        if(req_fire&&picked_write)
            wq_data[wq_wp]<=req_wdata[pick*512 +: 512];
    end
    assign req_ready=req_fire ? ({{(N_CLIENTS-1){1'b0}},1'b1}<<pick) : '0;
    always @(posedge clk) begin
        if(rst) begin
            rq_wp<=0;rq_rp<=0;wq_wp<=0;wq_aw_rp<=0;wq_w_rp<=0;rq_count<=0;wq_count<=0;
            wq_aw_count<=0;wq_issued_count<=0;id_live<=0;id_write<=0;id_narrow<=0;
            w_beat<=0;prefer_read<=0;rr_client<=0;rsp_valid_q<=0;
        end else begin
            if(rsp_valid_q&&rsp_ready[rsp_client_q]) rsp_valid_q<=0;
            if(req_fire&&picked_write) begin
                wq_addr[wq_wp]<=req_addr[pick*ADDR_WIDTH +: ADDR_WIDTH];
                wq_len[wq_wp]<=req_len[pick*7 +: 7];wq_tag[wq_wp]<=req_tag[pick*TAG_WIDTH +: TAG_WIDTH]; wq_client[wq_wp]<=pick[CW-1:0]; wq_wp<=wq_wp+1'b1;
            end
            if(req_fire&&!picked_write) begin
                rq_addr[rq_wp]<=req_addr[pick*ADDR_WIDTH +: ADDR_WIDTH];rq_len[rq_wp]<=req_len[pick*7 +: 7]; rq_tag[rq_wp]<=req_tag[pick*TAG_WIDTH +: TAG_WIDTH];
                rq_client[rq_wp]<=pick[CW-1:0]; rq_wp<=rq_wp+1'b1;
            end
            if(req_fire) rr_client<=CW'((pick==N_CLIENTS-1)?0:pick+1);
            if(aw_issue) begin
                id_live[free_id]<=1;id_write[free_id]<=1;
                id_client[free_id]<=wq_client[wq_aw_rp];id_tag[free_id]<=wq_tag[wq_aw_rp];id_len[free_id]<=wq_len[wq_aw_rp];
                wq_aw_rp<=wq_aw_rp+1'b1;prefer_read<=1;
            end
            if(w_wvalid_c&&w_wready_c) begin
                if(w_wlast_c) begin w_beat<=0;wq_w_rp<=wq_w_rp+1'b1; end
                else w_beat<=w_beat+1'b1;
            end
            if(r_issue) begin
                id_live[free_id]<=1;id_write[free_id]<=0;id_client[free_id]<=rq_client[rq_rp];id_tag[free_id]<=rq_tag[rq_rp];
                id_len[free_id]<=rq_len[rq_rp];id_offset[free_id]<=rq_off;id_narrow[free_id]<=rq_narrow;id_rbeat[free_id]<=0;id_rstatus[free_id]<=0;rq_rp<=rq_rp+1'b1;prefer_read<=0;
            end
            case({req_fire&&!picked_write,r_issue}) 2'b10:rq_count<=rq_count+1'b1;2'b01:rq_count<=rq_count-1'b1;default:rq_count<=rq_count;endcase
            case({req_fire&&picked_write,w_done}) 2'b10:wq_count<=wq_count+1'b1;2'b01:wq_count<=wq_count-1'b1;default:wq_count<=wq_count;endcase
            case({req_fire&&picked_write,aw_issue}) 2'b10:wq_aw_count<=wq_aw_count+1'b1;2'b01:wq_aw_count<=wq_aw_count-1'b1;default:wq_aw_count<=wq_aw_count;endcase
            case({aw_issue,w_done}) 2'b10:wq_issued_count<=wq_issued_count+1'b1;2'b01:wq_issued_count<=wq_issued_count-1'b1;default:wq_issued_count<=wq_issued_count;endcase
            if(m_rvalid&&m_rready) begin
                id_rdata[m_rid[IW-1:0]]<=r_current;
                if(m_rresp!=0) id_rstatus[m_rid[IW-1:0]]<=m_rresp;
                if(m_rlast) begin
                    id_live[m_rid[IW-1:0]]<=0;rsp_valid_q<=1;rsp_write_q<=0;rsp_client_q<=r_client;
                    rsp_tag_q<=id_tag[m_rid[IW-1:0]];
                    rsp_len_q<=id_len[m_rid[IW-1:0]];
                    rsp_status_q<=(m_rresp!=0)?m_rresp:id_rstatus[m_rid[IW-1:0]];
                    rsp_rdata_q<=r_payload;
                end else id_rbeat[m_rid[IW-1:0]]<=id_rbeat[m_rid[IW-1:0]]+1'b1;
            end
            if(m_bvalid&&m_bready) begin
                id_live[m_bid[IW-1:0]]<=0;rsp_valid_q<=1;rsp_write_q<=1;rsp_client_q<=b_client;
                rsp_tag_q<=id_tag[m_bid[IW-1:0]];
                rsp_len_q<=id_len[m_bid[IW-1:0]];
                rsp_status_q<=m_bresp;rsp_rdata_q<=0;
            end
        end
    end
endmodule
`default_nettype wire
