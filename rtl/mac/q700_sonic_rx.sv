// MAME reference/excerpt/adaptation attribution: Copyright Patrick Mackinlay.
// MAME-derived portions retain BSD-3-Clause; original contributions are MIT.
// See THIRD_PARTY_NOTICES.md and LICENSES/MAME-BSD-3-Clause.txt.
// q700_sonic_rx.sv -- SONIC receive resource/descriptor DMA engine.
//
// Taxi supplies a verified payload-only frame: its GMII RX pipeline delays
// the stream to validate the wire FCS, then removes those four bytes.  SONIC
// buffer contents and byte counts include FCS, so this engine recreates and
// appends the Ethernet CRC32 before emitting consecutive <=64-byte DMA writes.
// It commits the RDA only
// after all payload write responses have completed.
`default_nettype none
module q700_sonic_rx #(
    parameter integer MAX_CHUNKS = 32,
    // Bring-up escape hatch: accept every frame after FCS validation,
    // bypassing CAM/broadcast/multicast admission.  Keep the architectural
    // filter as the default; the Ethernet FPGA integration enables this
    // temporarily while the SONIC CAM programming path is debugged.
    parameter bit ACCEPT_ALL = 1'b0
) (
    input wire clk, input wire rst,
    input wire promisc_enable,
    input wire rx_enable,
    input wire cfg_valid, output wire cfg_ready,
    input wire [2:0] cfg_op, // bit 0: fetch RRA, bit 1: reload RDA, bit 2: load CAM
    input wire [15:0] cfg_dcr, cfg_rcr, cfg_urda, cfg_crda, cfg_urra,
    input wire [15:0] cfg_rsa, cfg_rea, cfg_rrp, cfg_rwp, cfg_eobc,
    input wire [15:0] cfg_rsc, cfg_llfa, cfg_cdp, cfg_cdc,

    input wire [7:0] rx_axis_tdata, input wire rx_axis_tvalid,
    output wire rx_axis_tready, input wire rx_axis_tlast, input wire rx_axis_tuser,

    output reg [31:0] dma_req_addr, output reg [6:0] dma_req_len,
    output reg [511:0] dma_req_wdata, output reg [7:0] dma_req_tag,
    output reg dma_req_write, output reg dma_req_valid, input wire dma_req_ready,
    input wire [511:0] dma_rsp_rdata, input wire [7:0] dma_rsp_tag,
    input wire [6:0] dma_rsp_len, input wire [1:0] dma_rsp_status,
    input wire dma_rsp_write, input wire dma_rsp_valid, output wire dma_rsp_ready,

    output reg done_valid, input wire done_ready, output reg done_error,
    output reg [15:0] done_rcr, done_crda, done_crba0, done_crba1,
    output reg [15:0] done_rbwc0, done_rbwc1, done_rrp, done_rsc,
    output reg [15:0] done_llfa, done_trba0, done_trba1,
    output reg [15:0] done_tbwc0, done_tbwc1, done_isr_set,
    output reg [15:0] done_cdp, done_cdc, done_ce,

    output wire [4:0] dbg_state,
    output wire [31:0] dbg_descriptor_addr,
    output wire [15:0] dbg_frame_len,
    // Receive-admission observability.  filter_accept is the single gate
    // between "the MAC gave us a valid frame" and "we DMA it to the guest",
    // and every one of its terms comes from driver-programmed state that is
    // otherwise invisible from JTAG.
    output wire [15:0] dbg_rcr,
    output wire [15:0] dbg_cam_enable,
    input  wire [3:0]  dbg_cam_index,
    // The driver loads the Mac's own MAC into CAM entry 15, not entry 0, so a
    // tap hardwired to entry 0 could never show the address that unicast
    // filtering actually compares against.  The chip's own readback path
    // (write CEP, read CAP0/1/2) is not modelled here or in MAME, so this mux
    // is the only window onto cam_table.  Same clock domain as the reader.
    output wire [47:0] dbg_cam_entry
);
    /* verilator lint_off WIDTHTRUNC */
    /* verilator lint_off WIDTHEXPAND */
    // dp83932c.h: RCR_LBK is the *status* bit reported in the descriptor when
    // a frame arrived via loopback; RCR_LB is the 2-bit loopback CONTROL field.
    localparam [15:0] RCR_LBK=16'h0002, RCR_LB=16'h0600;
    localparam [15:0] RCR_PRX=16'h0001, RCR_LPKT=16'h0040,
        RCR_BC=16'h0080, RCR_MC=16'h0100, RCR_AMC=16'h0800,
        RCR_PRO=16'h1000, RCR_BRD=16'h2000, RCR_RNT=16'h4000;
    localparam [15:0] ISR_RBAE=16'h0010, ISR_RBE=16'h0020,
        ISR_RDE=16'h0040, ISR_PKTRX=16'h0400, ISR_LCD=16'h1000;
    localparam [7:0] TAG_RRA=8'hf0, TAG_RDA=8'hf1, TAG_LINK=8'hf2,
        TAG_CLEAR=8'hf3, TAG_CAM=8'hf4, TAG_CE=8'hf5;
    localparam [4:0] S_IDLE=0, S_RRA_REQ=1, S_RRA_WAIT=2, S_READY=3,
        S_CAPTURE=4, S_DMA_LOAD=5, S_FILTER=6, S_DMA_PREP=7, S_DMA_WRITE=8,
        S_DMA_WAIT=9, S_RDA_REQ=10, S_RDA_WAIT=11, S_LINK_REQ=12,
        S_LINK_WAIT=13, S_CLEAR_REQ=14, S_CLEAR_WAIT=15,
        S_RELOAD_REQ=16, S_RELOAD_WAIT=17, S_CAM_REQ=18,
        S_CAM_WAIT=19, S_CE_REQ=20, S_CE_WAIT=21, S_FCS_APPEND=22,
        S_EOL_RETRY=23, S_EOL_WAIT=24;

    reg [4:0] state;
    reg wide_desc;
    reg descriptor_blocked, resource_blocked, reload_after_rra;
    reg [2:0] word_bytes;
    reg [15:0] rcr, urda, crda, urra, rsa, rea, rrp, rwp, eobc, rsc, llfa;
    reg [31:0] crba, rbwc;
    reg [31:0] trba, tbwc;
    reg [15:0] status_rcr;
    reg [15:0] frame_len;
    reg [31:0] frame_crc, fcs_shift;
    reg [1:0] fcs_index;
    reg [47:0] dest_mac;
    reg [47:0] cam_table [0:15];
    reg [15:0] cam_enable, cam_cdp;
    reg [4:0] cam_count;
    reg frame_bad;
    (* ram_style="block" *) reg [511:0] frame_chunks[0:MAX_CHUNKS-1];
    reg [511:0] capture_chunk;
    reg capture_write_pending;
    reg [5:0] capture_write_addr_q;
    reg [511:0] capture_write_data_q;
    reg [5:0] frame_read_addr;
    reg [511:0] frame_read_data;
    reg [511:0] issue_data;
    reg [5:0] issue_chunk, total_chunks, writes_done;
    reg [15:0] issue_offset, total_len;
    reg failed;
    integer i, cam_index;

    function automatic [15:0] desc_word;
        input [511:0] data; input integer index; input bit wide;
        integer b;
        begin
            b=index*(wide?4:2)+(wide?2:0);
            desc_word={data[b*8 +: 8],data[(b+1)*8 +: 8]};
        end
    endfunction
    function automatic [511:0] pack_word;
        input [15:0] value; input integer index; input bit wide;
        reg [511:0] t; integer b;
        begin
            t=0; b=index*(wide?4:2)+(wide?2:0);
            t[b*8 +: 8]=value[15:8]; t[(b+1)*8 +: 8]=value[7:0];
            pack_word=t;
        end
    endfunction

    function automatic [31:0] crc32_byte;
        input [31:0] crc;
        input [7:0] data;
        reg [31:0] c;
        integer bit_index;
        begin
            c=crc;
            for(bit_index=0;bit_index<8;bit_index=bit_index+1)
                c=(c[0]^data[bit_index]) ?
                    ((c>>1)^32'hedb88320) : (c>>1);
            crc32_byte=c;
        end
    endfunction

    wire axis_fire=rx_axis_tvalid&&rx_axis_tready;
    wire [15:0] rda_len=frame_len;
    wire [31:0] rbwc_after=rbwc-((rda_len+1'b1)>>1);
    wire [31:0] crba_after=crba+rda_len;
    wire dest_broadcast=&dest_mac;
    wire dest_multicast=dest_mac[40];
    reg dest_cam;
    always @(*) begin
        dest_cam=1'b0;
        for(cam_index=0;cam_index<16;cam_index=cam_index+1)
            if(cam_enable[cam_index] && dest_mac==cam_table[cam_index])
                dest_cam=1'b1;
    end
    // Treat an unconnected debug input as disabled in simulation as well as
    // in hardware; this keeps standalone users/tests source-compatible.
    wire filter_promisc = (promisc_enable === 1'b1);
    wire filter_accept=filter_promisc || ACCEPT_ALL || (rcr&RCR_PRO)!=0 || dest_cam ||
        (dest_broadcast&&((rcr&(RCR_BRD|RCR_AMC))!=0)) ||
        (dest_multicast&&((rcr&RCR_AMC)!=0));
    wire [6:0] current_dma_len=((total_len-issue_offset)>64)?7'd64:
        (total_len-issue_offset);
    wire payload_rsp=dma_rsp_valid&&(dma_rsp_tag<MAX_CHUNKS);
    wire [15:0] rsp_link=desc_word(dma_rsp_rdata,0,wide_desc);
    wire [15:0] rsp_cam_pointer=desc_word(dma_rsp_rdata,0,wide_desc);
    wire [3:0] rsp_cam_index=rsp_cam_pointer[3:0];
    wire [15:0] rsp_cam_cap0=desc_word(dma_rsp_rdata,1,wide_desc);
    wire [15:0] rsp_cam_cap1=desc_word(dma_rsp_rdata,2,wide_desc);
    wire [15:0] rsp_cam_cap2=desc_word(dma_rsp_rdata,3,wide_desc);
    // SONIC CAP words use low byte first relative to the network-order MAC.
    // This matches the native driver/MAME descriptor convention.
`ifdef SONIC_RX_MUTANT_CAM_NO_SWAP
    wire [47:0] rsp_cam_mac={rsp_cam_cap0,rsp_cam_cap1,rsp_cam_cap2};
`else
    wire [47:0] rsp_cam_mac={rsp_cam_cap0[7:0],rsp_cam_cap0[15:8],
        rsp_cam_cap1[7:0],rsp_cam_cap1[15:8],
        rsp_cam_cap2[7:0],rsp_cam_cap2[15:8]};
`endif
    // LPKT is set when RBWC is "less than or EQUAL to" EOBC (ds:1873) -- this
    // compared with a strict `<`, so the last packet that exactly fills a
    // buffer was not flagged as the last one.  And in 32-bit mode the SONIC
    // "holds the LSB always low so that it properly compares with the RBWC0,1
    // registers" (ds:1019-1020), which was not modelled either.  Both are
    // one-count edge effects on the end-of-buffer boundary, which is where
    // bulk receive goes wrong.
    wire [15:0] eobc_eff = wide_desc ? (eobc & 16'hfffe) : eobc;

    wire fcs_append=(state==S_FCS_APPEND);
    wire capture_event=axis_fire||fcs_append;
    wire capture_first=axis_fire&&(state==S_READY);
    wire [7:0] capture_byte=fcs_append?fcs_shift[7:0]:rx_axis_tdata;
    wire [5:0] capture_byte_index=capture_first?6'd0:frame_len[5:0];
    reg [511:0] capture_with_byte;
    always @(*) begin
        capture_with_byte = capture_first ? 512'd0 : capture_chunk;
        capture_with_byte[capture_byte_index*8 +: 8] = capture_byte;
    end
    wire capture_commit = capture_event &&
        ((capture_byte_index==6'd63)||(fcs_append&&(fcs_index==2'd3)));
    wire [5:0] capture_write_addr = capture_first?6'd0:frame_len[10:6];

    assign cfg_ready=(state==S_IDLE)||(state==S_READY);
    // descriptor_blocked (ring at EOL) deliberately does NOT gate acceptance.
    // MAME reloads CRDA from LLFA lazily, when the next frame arrives, and only
    // rejects the frame if the reload still shows EOL -- it never latches the
    // receiver off.  Latching it off meant a driver that polls during init (and
    // so never runs the interrupt handler that issues the RDE write-clear) hung
    // permanently the moment its first ring filled: reception stopped and the
    // only thing that could restart it was the handler it had not installed.
    assign rx_axis_tready=((state==S_READY)&&rx_enable&&!cfg_valid&&
        !resource_blocked)||(state==S_CAPTURE);
    assign dma_rsp_ready=1'b1;
    assign dbg_state=state;
    assign dbg_descriptor_addr={urda,crda};
    assign dbg_frame_len=frame_len;
    assign dbg_rcr=rcr;
    assign dbg_cam_enable=cam_enable;
    assign dbg_cam_entry=cam_table[dbg_cam_index];

    // Taxi is byte-wide, but the packet RAM is chunk-wide.  One staging
    // register absorbs a byte every cycle and commits a complete (or final
    // partial) chunk in one write.  The synchronous read address is
    // prefetched so adjacent 64-byte DMA writes can issue without bubbles.
    // Freeze the read pipeline while the DMA is back-pressuring.
    //
    // frame_read_addr only advances on an accepted request, but this read
    // fired EVERY cycle -- so during a stall frame_read_data caught up to the
    // held address and advanced to the NEXT chunk.  When the accept finally
    // landed, issue_data captured that already-advanced chunk: one chunk was
    // skipped and every later one shifted down, leaving the tail of the frame
    // holding stale bytes from the previous packet.
    //
    // On hardware this bit at the 17th write -- the first to meet
    // back-pressure with QUEUE_DEPTH(16) -- giving a hard 1088-byte (17*64)
    // receive ceiling: below it the shifted tail lands only in the
    // reconstructed FCS, which the driver ignores; above it the corruption
    // reaches real payload, the IP checksum fails, and the packet is dropped
    // silently.
    wire frame_read_en = !((state == S_DMA_WRITE) && !dma_req_ready);

    always @(posedge clk) begin
        if (frame_read_en) frame_read_data <= frame_chunks[frame_read_addr];
        // Cut the measured 200 MHz state/AXIS-ready -> byte insertion ->
        // packet-BRAM data path. Pipeline the complete write transaction;
        // no receive bubble or DMA issue bubble is introduced. After the final
        // FCS byte, S_FILTER drains this write before S_DMA_PREP/S_DMA_LOAD
        // consume chunk zero. Even a one-chunk frame observes the new data.
        // Do not add a reset to the wide data register: pending is its validity.
        capture_write_data_q <= capture_with_byte;
        capture_write_addr_q <= capture_write_addr;
        capture_write_pending <= !rst && capture_commit;
        if (!rst && capture_write_pending)
            frame_chunks[capture_write_addr_q] <= capture_write_data_q;
        if (rst)
            capture_chunk <= 512'd0;
        else if (capture_event)
            capture_chunk <= capture_commit ? 512'd0 : capture_with_byte;
    end

    always @(*) begin
        dma_req_addr=0; dma_req_len=0; dma_req_wdata=0; dma_req_tag=0;
        dma_req_write=0; dma_req_valid=0;
        case(state)
            S_RRA_REQ: begin
                dma_req_addr={urra,rrp}; dma_req_len=word_bytes*4;
                dma_req_tag=TAG_RRA; dma_req_valid=1;
            end
            S_DMA_WRITE: begin
                dma_req_addr=trba+issue_offset; dma_req_len=current_dma_len;
                dma_req_wdata=issue_data; dma_req_tag=issue_chunk;
                dma_req_write=1; dma_req_valid=1;
            end
            S_RDA_REQ: begin
                dma_req_addr={urda,crda}; dma_req_len=word_bytes*5;
                dma_req_wdata=pack_word(status_rcr,0,wide_desc)|
                    pack_word(rda_len,1,wide_desc)|pack_word(trba[15:0],2,wide_desc)|
                    pack_word(trba[31:16],3,wide_desc)|pack_word(rsc,4,wide_desc);
                dma_req_tag=TAG_RDA; dma_req_write=1; dma_req_valid=1;
            end
            S_LINK_REQ: begin
                dma_req_addr={urda,crda} + 5*word_bytes;
                dma_req_len=word_bytes; dma_req_tag=TAG_LINK; dma_req_valid=1;
            end
            S_CLEAR_REQ: begin
                dma_req_addr={urda,done_llfa} + word_bytes;
                dma_req_len=word_bytes; dma_req_tag=TAG_CLEAR;
                dma_req_write=1; dma_req_valid=1;
            end
            S_RELOAD_REQ, S_EOL_RETRY: begin
                dma_req_addr={urda,llfa}; dma_req_len=word_bytes;
                dma_req_tag=TAG_LINK; dma_req_valid=1;
            end
            S_CAM_REQ: begin
                dma_req_addr={urra,cam_cdp}; dma_req_len=word_bytes*4;
                dma_req_tag=TAG_CAM; dma_req_valid=1;
            end
            S_CE_REQ: begin
                dma_req_addr={urra,cam_cdp}; dma_req_len=word_bytes;
                dma_req_tag=TAG_CE; dma_req_valid=1;
            end
            default: begin end
        endcase
    end

    always @(posedge clk) begin
        if(rst) begin
            state<=S_IDLE; done_valid<=0; done_error<=0; failed<=0;
            descriptor_blocked<=1; resource_blocked<=1; reload_after_rra<=0;
            done_rcr<=0; done_crda<=0; done_crba0<=0; done_crba1<=0;
            done_rbwc0<=0; done_rbwc1<=0; done_rrp<=0; done_rsc<=0;
            done_llfa<=0; done_trba0<=0; done_trba1<=0;
            done_tbwc0<=0; done_tbwc1<=0; done_isr_set<=0;
            done_cdp<=0; done_cdc<=0; done_ce<=0;
            cam_enable<=0; cam_cdp<=0; cam_count<=0;
            frame_crc<=32'hffffffff; fcs_shift<=0; fcs_index<=0;
            for(i=0;i<16;i=i+1) cam_table[i]<=0;
            frame_read_addr<=0;
        end else begin
            if(payload_rsp) begin
                writes_done<=writes_done+1'b1;
                if(dma_rsp_status!=0||!dma_rsp_write) failed<=1;
            end
            if(cfg_valid&&cfg_ready) begin
                // The receive FILTER input is adopted for EVERY op, including
                // the refresh -- that is the whole point of the refresh.  RCR
                // is read per frame on real silicon; this engine used to see
                // it only in the snapshot taken at a config handshake, and
                // drivers program RCR *after* their last RRA/CAM command, so
                // it held RCR=0 forever and dropped every validated frame.
                rcr<=cfg_rcr;
                done_valid<=0; done_error<=0; done_isr_set<=0;

                // Descriptor, resource and WIDTH state is adopted only by a
                // REAL command.  A refresh must not touch it: the driver
                // writes RCR at an arbitrary moment, so taking the whole block
                // there adopts whatever URDA/CRDA/RRP hold mid-setup and --
                // worse -- re-derives wide_desc from a DCR that may still read
                // 0, flipping the engine to 16-bit parsing on a 32-bit ring.
                // Wrong field offsets, wrong link addresses, DMA writes at
                // wrong addresses: a memory-corruption path, not a filter bug.
                if(cfg_op!=3'b000) begin
                    wide_desc<=cfg_dcr[5]; word_bytes<=cfg_dcr[5]?4:2;
                    urda<=cfg_urda; crda<=cfg_crda;
                    urra<=cfg_urra; rsa<=cfg_rsa; rea<=cfg_rea;
                    rrp<=cfg_rrp; rwp<=cfg_rwp; eobc<=cfg_eobc;
                    rsc<=cfg_rsc; llfa<=cfg_llfa; failed<=0;
                    cam_cdp<=cfg_cdp; cam_count<=cfg_cdc[4:0];
                    reload_after_rra<=cfg_op[0]&&cfg_op[1];
                    if(cfg_op[2]) begin
                        state<=(cfg_cdc[4:0]==0)?S_CE_REQ:S_CAM_REQ;
                    end else if(cfg_op[0]) begin
                        resource_blocked<=1; descriptor_blocked<=cfg_crda[0];
                        state<=S_RRA_REQ;
                    end else if(cfg_op[1]) begin
                        descriptor_blocked<=1; state<=S_RELOAD_REQ;
                    end
                end
                // cfg_op == 0 issues no DMA and no completion, so it cannot
                // disturb descriptor/resource state or write back stale
                // pointers.  It simply stays idle with the new filter value.
            end else case(state)
                S_IDLE: done_valid<=0;
                S_RRA_REQ: if(dma_req_ready) state<=S_RRA_WAIT;
                S_RRA_WAIT: if(dma_rsp_valid&&dma_rsp_tag==TAG_RRA) begin
                    crba<={desc_word(dma_rsp_rdata,1,wide_desc),desc_word(dma_rsp_rdata,0,wide_desc)};
                    rbwc<={desc_word(dma_rsp_rdata,3,wide_desc),desc_word(dma_rsp_rdata,2,wide_desc)};
                    rrp<=((rrp+4*word_bytes)==rea)?rsa:(rrp+4*word_bytes);
                    done_rcr<=rcr; done_crda<=crda;
                    done_crba0<=desc_word(dma_rsp_rdata,0,wide_desc);
                    done_crba1<=desc_word(dma_rsp_rdata,1,wide_desc);
                    done_rbwc0<=desc_word(dma_rsp_rdata,2,wide_desc);
                    done_rbwc1<=desc_word(dma_rsp_rdata,3,wide_desc);
                    done_rrp<=((rrp+4*word_bytes)==rea)?rsa:(rrp+4*word_bytes);
                    done_rsc<=rsc; done_llfa<=llfa;
                    resource_blocked<=((((rrp+4*word_bytes)==rea)?rsa:
                        (rrp+4*word_bytes))==rwp);
                    if((((rrp+4*word_bytes)==rea)?rsa:(rrp+4*word_bytes))==rwp)
                        done_isr_set<=done_isr_set|ISR_RBE;
                    if(dma_rsp_status!=0||dma_rsp_write) begin failed<=1; done_error<=1; end
                    if(reload_after_rra) begin reload_after_rra<=0; state<=S_RELOAD_REQ; end
                    else begin done_valid<=1; state<=S_CLEAR_WAIT; end
                end
                S_READY: if(axis_fire) begin
                    frame_len<=1; dest_mac<=0;
                    frame_crc<=crc32_byte(32'hffffffff,rx_axis_tdata);
                    dest_mac[47:40]<=rx_axis_tdata; frame_bad<=rx_axis_tuser;
                    status_rcr<=rcr&16'hfe00;
                    done_isr_set<=0; done_error<=0;
                    if(rx_axis_tlast) begin
                        fcs_shift<=~crc32_byte(32'hffffffff,rx_axis_tdata);
                        fcs_index<=0;
`ifdef SONIC_RX_MUTANT_NO_FCS
                        state<=S_FILTER;
`else
                        state<=S_FCS_APPEND;
`endif
                    end
                    else state<=S_CAPTURE;
                end
                S_CAPTURE: if(axis_fire) begin
                    if(frame_len<6) dest_mac[(5-frame_len)*8 +:8]<=rx_axis_tdata;
                    frame_len<=frame_len+1'b1;
                    frame_crc<=crc32_byte(frame_crc,rx_axis_tdata);
                    frame_bad<=frame_bad|rx_axis_tuser;
                    if(rx_axis_tlast) begin
                        fcs_shift<=~crc32_byte(frame_crc,rx_axis_tdata);
                        fcs_index<=0;
`ifdef SONIC_RX_MUTANT_NO_FCS
                        state<=S_FILTER;
`else
                        state<=S_FCS_APPEND;
`endif
                    end
                end
                S_FCS_APPEND: begin
                    frame_len<=frame_len+1'b1;
                    fcs_shift<={8'd0,fcs_shift[31:8]};
                    if(fcs_index==2'd3) state<=S_FILTER;
                    else fcs_index<=fcs_index+1'b1;
                end
                S_FILTER: begin
                    if(frame_bad || !filter_accept || ((rda_len<64)&&((rcr&RCR_RNT)==0))) begin
                        state<=S_READY;
                    end else if(descriptor_blocked) begin
                        // Ring is at EOL.  Re-read the link at LLFA once for
                        // this frame, exactly as MAME does on receive.
                        state<=S_EOL_RETRY;
                    end else if((rbwc<<1)<rda_len) begin
                        done_isr_set<=ISR_RBAE; done_error<=1; done_valid<=1; state<=S_CLEAR_WAIT;
                    end else begin
                        status_rcr<=(rcr&16'hfe00)|RCR_PRX|
                            (((rcr&RCR_LB)!=0)?RCR_LBK:16'h0000)|
                            (dest_broadcast?RCR_BC:0)|(dest_multicast&&!dest_broadcast?RCR_MC:0)|
                            ((rbwc_after<=eobc_eff)?RCR_LPKT:0);
                        trba<=crba; tbwc<=rbwc; total_len<=rda_len;
                        total_chunks<=(rda_len+63)>>6; issue_chunk<=0; issue_offset<=0;
                        writes_done<=0; failed<=0; frame_read_addr<=0; state<=S_DMA_PREP;
                    end
                end
                S_DMA_PREP: begin frame_read_addr<=1; state<=S_DMA_LOAD; end
                S_DMA_LOAD: begin
                    issue_data<=frame_read_data;
                    // frame_read_data already holds chunk 1 while chunk 0 is
                    // issued.  Point the synchronous port at chunk 2 now so
                    // every later accepted write has its successor ready.
`ifdef SONIC_RX_MUTANT_SHORT_LOOKAHEAD
                    frame_read_addr<=1;
`else
                    frame_read_addr<=2;
`endif
                    state<=S_DMA_WRITE;
                end
                S_DMA_WRITE: if(dma_req_ready) begin
                    if(issue_chunk+1'b1==total_chunks) begin
`ifdef SONIC_RX_MUTANT_RDA_EARLY
                        state<=S_RDA_REQ;
`else
                        state<=S_DMA_WAIT;
`endif
                    end
                    else begin
                        issue_data<=frame_read_data;
`ifdef SONIC_RX_MUTANT_SHORT_LOOKAHEAD
                        frame_read_addr<=issue_chunk+2'd2;
`else
                        frame_read_addr<=issue_chunk+2'd3;
`endif
                        issue_chunk<=issue_chunk+1'b1; issue_offset<=issue_offset+64;
                    end
                end
                S_DMA_WAIT: if(writes_done==total_chunks) begin
                    if(failed) begin done_error<=1; done_valid<=1; state<=S_CLEAR_WAIT; end
                    else begin crba<=crba_after; rbwc<=rbwc_after; state<=S_RDA_REQ; end
                end
                S_RDA_REQ: if(dma_req_ready) state<=S_RDA_WAIT;
                S_RDA_WAIT: if(dma_rsp_valid&&dma_rsp_tag==TAG_RDA) begin
                    if(dma_rsp_status!=0||!dma_rsp_write) failed<=1;
                    llfa<=crda+5*word_bytes; done_llfa<=crda+5*word_bytes;
                    state<=S_LINK_REQ;
                end
                S_LINK_REQ: if(dma_req_ready) state<=S_LINK_WAIT;
                S_LINK_WAIT: if(dma_rsp_valid&&dma_rsp_tag==TAG_LINK) begin
                    crda<=rsp_link;
                    if(dma_rsp_status!=0||dma_rsp_write) failed<=1;
                    if(rsp_link[0]) begin
                        descriptor_blocked<=1;
                        done_isr_set<=ISR_PKTRX|ISR_RDE; done_rcr<=status_rcr;
                        done_crda<=rsp_link; done_crba0<=crba[15:0]; done_crba1<=crba[31:16];
                        done_rbwc0<=rbwc[15:0]; done_rbwc1<=rbwc[31:16]; done_rrp<=rrp;
                        done_rsc<=rsc; done_trba0<=trba[15:0]; done_trba1<=trba[31:16];
                        done_tbwc0<=tbwc[15:0]; done_tbwc1<=tbwc[31:16];
                        state<=S_CLEAR_WAIT; done_valid<=1;
                    end
                    else begin descriptor_blocked<=0; state<=S_CLEAR_REQ; end
                end
                S_EOL_RETRY: if(dma_req_ready) state<=S_EOL_WAIT;
                S_EOL_WAIT: if(dma_rsp_valid&&dma_rsp_tag==TAG_LINK) begin
                    // One retry per received frame, which is what rate-limits
                    // this -- there is no free-running poll of the link.
                    crda<=rsp_link;
                    descriptor_blocked<=rsp_link[0];
                    if(dma_rsp_status!=0||dma_rsp_write) failed<=1;
                    // Still EOL: drop this frame (RDE is already pending from
                    // when the ring ran out).  Reopened: fall back into the
                    // normal filter path, which now sees descriptor_blocked=0
                    // and stores the frame -- so the reload is transparent.
                    state<=rsp_link[0] ? S_READY : S_FILTER;
                end
                S_RELOAD_REQ: if(dma_req_ready) state<=S_RELOAD_WAIT;
                S_RELOAD_WAIT: if(dma_rsp_valid&&dma_rsp_tag==TAG_LINK) begin
                    crda<=rsp_link; done_crda<=rsp_link;
                    descriptor_blocked<=rsp_link[0];
                    if(rsp_link[0]) done_isr_set<=done_isr_set|ISR_RDE;
                    if(dma_rsp_status!=0||dma_rsp_write) begin failed<=1; done_error<=1; end
                    done_rcr<=rcr; done_crba0<=crba[15:0]; done_crba1<=crba[31:16];
                    done_rbwc0<=rbwc[15:0]; done_rbwc1<=rbwc[31:16]; done_rrp<=rrp;
                    done_rsc<=rsc; done_llfa<=llfa; done_trba0<=trba[15:0];
                    done_trba1<=trba[31:16]; done_tbwc0<=tbwc[15:0]; done_tbwc1<=tbwc[31:16];
                    done_valid<=1; state<=S_CLEAR_WAIT;
                end
                S_CAM_REQ: if(dma_req_ready) state<=S_CAM_WAIT;
                S_CAM_WAIT: if(dma_rsp_valid&&dma_rsp_tag==TAG_CAM) begin
                    if(dma_rsp_status!=0||dma_rsp_write) begin
                        failed<=1; done_error<=1;
                    end else begin
                        cam_table[rsp_cam_index]<=rsp_cam_mac;
                    end
                    cam_cdp<=cam_cdp+4*word_bytes;
                    if(cam_count<=1) begin
                        cam_count<=0; state<=S_CE_REQ;
                    end else begin
                        cam_count<=cam_count-1'b1; state<=S_CAM_REQ;
                    end
                end
                S_CE_REQ: if(dma_req_ready) state<=S_CE_WAIT;
                S_CE_WAIT: if(dma_rsp_valid&&dma_rsp_tag==TAG_CE) begin
                    if(dma_rsp_status!=0||dma_rsp_write) begin
                        failed<=1; done_error<=1; cam_enable<=0; done_ce<=0;
                    end else begin
                        cam_enable<=desc_word(dma_rsp_rdata,0,wide_desc);
                        done_ce<=desc_word(dma_rsp_rdata,0,wide_desc);
                    end
                    done_cdp<=cam_cdp; done_cdc<=0;
                    done_isr_set<=ISR_LCD; done_valid<=1; state<=S_CLEAR_WAIT;
                end
                S_CLEAR_REQ: if(dma_req_ready) state<=S_CLEAR_WAIT;
                S_CLEAR_WAIT: begin
                    if(!done_valid && dma_rsp_valid&&dma_rsp_tag==TAG_CLEAR) begin
                        if(dma_rsp_status!=0||!dma_rsp_write) failed<=1;
                        done_isr_set<=ISR_PKTRX;
                        // Same "less than or EQUAL to" rule (ds:1873), and
                        // the more consequential of the two: this decides
                        // when the current RBA is exhausted and the next
                        // receive resource must be fetched.  Being one count
                        // late means keeping a buffer that no longer has room
                        // for the frame that follows.
                        if(rbwc<=eobc_eff) begin
                            rsc<=(rsc&16'hff00)+16'h0100;
                            state<=S_RRA_REQ;
                        end else begin
                            rsc<=(rsc&16'hff00)|((rsc+1'b1)&16'h00ff);
                            done_error<=failed|(dma_rsp_status!=0)||!dma_rsp_write;
                            done_rcr<=status_rcr; done_crda<=crda;
                            done_crba0<=crba[15:0]; done_crba1<=crba[31:16];
                            done_rbwc0<=rbwc[15:0]; done_rbwc1<=rbwc[31:16];
                            done_rrp<=rrp;
                            done_rsc<=(rsc&16'hff00)|((rsc+1'b1)&16'h00ff);
                            done_trba0<=trba[15:0]; done_trba1<=trba[31:16];
                            done_tbwc0<=tbwc[15:0]; done_tbwc1<=tbwc[31:16];
                            done_valid<=1;
                        end
                    end
                    if(done_valid) begin
                        done_error<=failed; done_rcr<=status_rcr; done_crda<=crda;
                        done_crba0<=crba[15:0]; done_crba1<=crba[31:16];
                        done_rbwc0<=rbwc[15:0]; done_rbwc1<=rbwc[31:16]; done_rrp<=rrp;
                        done_rsc<=rsc; done_trba0<=trba[15:0]; done_trba1<=trba[31:16];
                        done_tbwc0<=tbwc[15:0]; done_tbwc1<=tbwc[31:16];
                        if(done_ready) begin done_valid<=0; frame_len<=0; state<=S_READY; end
                    end
                end
                default: state<=S_IDLE;
            endcase
        end
    end
    /* verilator lint_off UNUSEDSIGNAL */
    wire _unused_rsp_len=|dma_rsp_len;
    /* verilator lint_on UNUSEDSIGNAL */
    /* verilator lint_on WIDTHEXPAND */
    /* verilator lint_on WIDTHTRUNC */
endmodule
`default_nettype wire
