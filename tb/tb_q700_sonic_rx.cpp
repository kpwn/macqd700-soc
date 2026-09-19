#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <verilated.h>
#include "Vq700_sonic_rx.h"

struct Req { uint32_t a; uint8_t n,t; bool w; int delay; };
static Vq700_sonic_rx *d; static std::vector<uint8_t> mem(65536);
static std::vector<Req> q; static uint64_t cyc; static std::vector<uint64_t> payload_cycles;
static bool rda_before_payload_done;
static bool wide_desc;
static bool stall_requests;
static void zero(WData *p){for(int i=0;i<16;i++)p[i]=0;}
static void putw(uint32_t a,uint16_t v){mem[a]=v>>8;mem[a+1]=v;}
static uint16_t getw(uint32_t a){return uint16_t(mem[a])<<8|mem[a+1];}
static uint32_t desc_addr(uint32_t base,unsigned word){return base+word*(wide_desc?4:2)+(wide_desc?2:0);}
static void putd(uint32_t base,unsigned word,uint16_t v){putw(desc_addr(base,word),v);}
static uint16_t getd(uint32_t base,unsigned word){return getw(desc_addr(base,word));}
static uint32_t crcbyte(uint32_t c,uint8_t x){for(int i=0;i<8;i++)c=(c^(x>>i))&1?(c>>1)^0xedb88320:c>>1;return c;}
static void drive_rsp(){
 d->dma_rsp_valid=0;d->dma_rsp_write=0;d->dma_rsp_status=0;d->dma_rsp_tag=0;d->dma_rsp_len=0;zero(d->dma_rsp_rdata);
 for(auto &x:q)if(x.delay)--x.delay; int pick=-1;
 for(int i=0;i<(int)q.size();i++)if(!q[i].delay&&(pick<0||q[i].t>q[pick].t))pick=i;
 if(pick<0)return; auto &x=q[pick];d->dma_rsp_valid=1;d->dma_rsp_write=x.w;d->dma_rsp_tag=x.t;d->dma_rsp_len=x.n;
 if(!x.w)for(unsigned i=0;i<x.n;i++)d->dma_rsp_rdata[i/4]|=uint32_t(mem[x.a+i])<<((i&3)*8);
}
static void accept_req(){
 if(!(d->dma_req_valid&&d->dma_req_ready))return;
 Req x{uint32_t(d->dma_req_addr),uint8_t(d->dma_req_len),uint8_t(d->dma_req_tag),bool(d->dma_req_write),2};
 if(x.t<32){x.delay=10+(x.t?0:3);payload_cycles.push_back(cyc);}
 if(x.t==0xf1)for(auto &p:q)if(p.t<32)rda_before_payload_done=true;
 if(x.w)for(unsigned i=0;i<x.n;i++)mem.at(x.a+i)=uint8_t(d->dma_req_wdata[i/4]>>((i&3)*8));
 q.push_back(x);
}
static void tick(){
 d->clk=0;d->dma_req_ready=!stall_requests||((cyc%3)!=0);drive_rsp();d->eval();accept_req();
 bool eat=d->dma_rsp_valid&&d->dma_rsp_ready;uint8_t tag=d->dma_rsp_tag;
 d->clk=1;d->eval();if(eat){auto it=std::find_if(q.begin(),q.end(),[&](Req const&x){return x.t==tag&&!x.delay;});if(it!=q.end())q.erase(it);}cyc++;
}
static bool wait_done(int limit=1000){while(!d->done_valid&&limit--)tick();return d->done_valid;}
int main(int argc,char**argv){
 Verilated::commandArgs(argc,argv);wide_desc=(argc>1 && std::strcmp(argv[1],"wide")==0);
 const unsigned first_len=argc>2?std::strtoul(argv[2],nullptr,0):125;
 if(first_len<60 || first_len>128){printf("FAIL first frame length must be 60..128\n");return 1;}
 d=new Vq700_sonic_rx;std::fill(mem.begin(),mem.end(),0);
 const unsigned wb=wide_desc?4:2;
 const uint32_t cam=0x0800,rra=0x1000,rba=0x2003,rda=0x3000;
 putd(cam,0,1);putd(cam,1,0x1102);putd(cam,2,0x3322);putd(cam,3,0x5544);
 putd(cam+4*wb,0,3);putd(cam+4*wb,1,0x3402);putd(cam+4*wb,2,0x7856);putd(cam+4*wb,3,0xbc9a);
 putd(cam+8*wb,0,0x0008); // enable only CAM entry 3
 putd(rra,0,rba);putd(rra,1,rba>>16);putd(rra,2,0x0400);putd(rra,3,0);putd(rda,5,1);
 d->rst=1;d->rx_enable=1;d->cfg_valid=0;d->cfg_op=1;d->done_ready=0;d->rx_axis_tvalid=0;d->rx_axis_tlast=0;d->rx_axis_tuser=0;
 for(int i=0;i<3;i++)tick();d->rst=0;
 d->cfg_dcr=wide_desc?0x20:0;d->cfg_rcr=0x2000;d->cfg_urda=0;d->cfg_crda=rda;d->cfg_urra=0;
 d->cfg_rsa=rra;d->cfg_rea=rra+8*wb;d->cfg_rrp=rra;d->cfg_rwp=rra+6*wb;d->cfg_eobc=32;d->cfg_rsc=7;d->cfg_llfa=0;
 d->cfg_cdp=cam;d->cfg_cdc=2;
 d->cfg_op=4;d->cfg_valid=1;while(!d->cfg_ready)tick();tick();d->cfg_valid=0;
 if(!wait_done()){printf("FAIL CAM load completion timeout\n");return 1;}
 if(d->done_isr_set!=0x1000||d->done_cdp!=cam+8*wb||d->done_cdc!=0||d->done_ce!=8){
   printf("FAIL CAM completion isr=%04x cdp=%04x cdc=%04x ce=%04x\n",d->done_isr_set,d->done_cdp,d->done_cdc,d->done_ce);return 1;
 }
 // The debug tap must show the entry the caller SELECTS, not entry 0.  The
 // driver loads the Mac's own MAC into entry 15, so a tap hardwired to 0 can
 // never show the address unicast filtering actually compares against -- and
 // the chip's own CEP/CAP readback is modelled neither here nor in MAME.
 // cap words are byte-swapped on load, so 0x1102/0x3322/0x5544 -> 02:11:22:33:44:55.
 {
   struct { int idx; uint64_t want; const char* what; } cases[] = {
     {1, 0x021122334455ULL, "entry 1 as loaded"},
     {3, 0x0234'5678'9abcULL, "entry 3 as loaded"},
     {0, 0x000000000000ULL, "entry 0 never loaded"},
     {15,0x000000000000ULL, "entry 15 never loaded"},
   };
   for (auto& c : cases) {
     d->dbg_cam_index=c.idx; d->eval();
     if(d->dbg_cam_entry!=c.want){
       printf("FAIL cam tap %s: idx=%d got %012llx want %012llx\n",
              c.what,c.idx,(unsigned long long)d->dbg_cam_entry,
              (unsigned long long)c.want); return 1;
     }
   }
   d->dbg_cam_index=0; d->eval();
 }
 d->done_ready=1;tick();d->done_ready=0;
 d->cfg_op=1;
 d->cfg_valid=1;while(!d->cfg_ready)tick();tick();d->cfg_valid=0;
 if(!wait_done()){printf("FAIL init completion timeout\n");return 1;} d->done_ready=1;tick();d->done_ready=0;
 if(!d->rx_axis_tready){printf("FAIL RX blocked after init isr=%04x rrp=%04x\n",d->done_isr_set,d->done_rrp);return 1;}

 // A cfg_op==0 REFRESH adopts the receive filter and NOTHING else.  The
 // driver writes RCR at an arbitrary moment, so if a refresh took the whole
 // config block it would adopt whatever URDA/CRDA/RRP happen to hold
 // mid-setup and re-derive the descriptor WIDTH from a DCR that may still
 // read 0 -- flipping to 16-bit parsing on a 32-bit ring, which puts RDA
 // fields and link addresses at wrong offsets and lands DMA writes at wrong
 // addresses.  Feed a refresh deliberately poisoned config and require that
 // only the filter moves.
 {
   uint32_t desc_before=d->dbg_descriptor_addr;
   uint16_t save_dcr=d->cfg_dcr,save_urda=d->cfg_urda,save_crda=d->cfg_crda;
   uint16_t save_urra=d->cfg_urra,save_rrp=d->cfg_rrp,save_rwp=d->cfg_rwp;
   d->cfg_dcr=0x0000; d->cfg_urda=0xdead; d->cfg_crda=0xbeef;
   d->cfg_urra=0xfeed; d->cfg_rrp=0x1234; d->cfg_rwp=0x5678;
   d->cfg_rcr=0x2000;                       // the one field it may adopt
   d->cfg_op=0; d->cfg_valid=1; while(!d->cfg_ready)tick(); tick(); d->cfg_valid=0;
   for(int i=0;i<8;i++)tick();
   if(d->dbg_descriptor_addr!=desc_before){
     printf("FAIL refresh moved the descriptor pointer: %08x -> %08x\n",
            desc_before,d->dbg_descriptor_addr);return 1;
   }
   if(d->dbg_rcr!=0x2000){
     printf("FAIL refresh did not adopt RCR: %04x\n",d->dbg_rcr);return 1;
   }
   d->cfg_dcr=save_dcr; d->cfg_urda=save_urda; d->cfg_crda=save_crda;
   d->cfg_urra=save_urra; d->cfg_rrp=save_rrp; d->cfg_rwp=save_rwp;
   if(!d->rx_axis_tready){printf("FAIL RX blocked after refresh\n");return 1;}
 }
 // 125 payload bytes place the four-byte FCS across a 64-byte packet-RAM
 // boundary (payload ends at offset 60 of the second chunk).
 // The length sweep also covers a single chunk and FCS on either side of
 // the first boundary, where a delayed packet-RAM write is most exposed.
 std::vector<uint8_t> frame(first_len);frame[0]=0x02;frame[1]=0x34;frame[2]=0x56;frame[3]=0x78;frame[4]=0x9a;frame[5]=0xbc;
 for(unsigned i=6;i<frame.size();i++)frame[i]=uint8_t(i*3+1);
 uint32_t crc=0xffffffff;for(auto b:frame)crc=crcbyte(crc,b);crc=~crc;
 std::vector<uint8_t> expected=frame;for(int i=0;i<4;i++)expected.push_back(crc>>(8*i));
 for(unsigned i=0;i<frame.size();i++){
   d->rx_axis_tdata=frame[i];d->rx_axis_tvalid=1;d->rx_axis_tlast=(i+1==frame.size());
   int ready_limit=1000;while(!d->rx_axis_tready&&ready_limit--)tick();
   if(!d->rx_axis_tready){printf("FAIL RX ready timeout at frame byte %u\n",i);return 1;}tick();
 } d->rx_axis_tvalid=0;d->rx_axis_tlast=0;
 if(!wait_done(3000)){printf("FAIL packet completion timeout\n");return 1;}
 bool ok=true;
 for(unsigned i=0;i<expected.size();i++)if(mem[rba+i]!=expected[i]){printf("FAIL payload byte %u got=%02x exp=%02x crc=%08x actual_fcs=%02x%02x%02x%02x\n",i,mem[rba+i],expected[i],crc,mem[rba+100],mem[rba+101],mem[rba+102],mem[rba+103]);ok=false;break;}
 if(payload_cycles.size()!=(expected.size()+63)/64){printf("FAIL RX write count\n");ok=false;}
 for(unsigned i=1;i<payload_cycles.size();i++)if(payload_cycles[i]!=payload_cycles[i-1]+1){printf("FAIL RX writes not consecutive\n");ok=false;}
 if(rda_before_payload_done){printf("FAIL RDA committed before payload responses\n");ok=false;}
 if(getd(rda,0)!=0x2001||getd(rda,1)!=expected.size()||getd(rda,2)!=uint16_t(rba)||getd(rda,3)!=0||getd(rda,4)!=7){
   printf("FAIL RDA words %04x %u %04x %04x %u\n",getd(rda,0),getd(rda,1),getd(rda,2),getd(rda,3),getd(rda,4));ok=false;
 }
 if(d->done_isr_set!=0x0440||d->done_crda!=1||d->done_crba0!=uint16_t(rba+expected.size())||d->done_rbwc0!=0x0400-(expected.size()+1)/2){
   printf("FAIL completion isr=%04x crda=%04x crba=%04x rbwc=%04x\n",d->done_isr_set,d->done_crda,d->done_crba0,d->done_rbwc0);ok=false;
 }
 // EOL must stop STORING frames -- but it must NOT latch the receiver off.
 // MAME re-reads the link at URDA:LLFA lazily, on the next received frame,
 // and only rejects that frame if the reload still shows EOL (dp83932c.cpp
 // recv_start_cb).  We used to gate rx_axis_tready on the EOL flag instead,
 // so the only thing that could restart reception was software clearing RDE
 // -- and a driver that POLLS during init never runs the interrupt handler
 // that issues that write.  Its first ring filled and the machine hung for
 // good.  So the invariant to hold here is "no frame is stored", not "the
 // receiver stops accepting".
 d->done_ready=1;tick();d->done_ready=0;
 if(!d->rx_axis_tready){printf("FAIL RX latched off at descriptor EOL\n");ok=false;}
 putd(rda,5,0x3100);
 // Link the rest of the chain NOW.  The engine latches a descriptor's link
 // when it advances, so writing it after the previous frame completes is too
 // late -- it will have read whatever was there (zero).
 putd(0x3100,5,0x3200);
 putd(0x3200,5,0);
 d->cfg_crda=d->done_crda;d->cfg_rrp=d->done_rrp;d->cfg_rsc=d->done_rsc;
 d->cfg_llfa=d->done_llfa;d->cfg_rwp=rra+6*wb;d->cfg_op=2;d->cfg_valid=1;
 int cfg_limit=1000;while(!d->cfg_ready&&cfg_limit--)tick();
 if(!d->cfg_ready){printf("FAIL RDE recovery config timeout\n");return 1;}tick();d->cfg_valid=0;
 if(!wait_done()){printf("FAIL RDE reload completion timeout\n");return 1;}
 if(d->done_crda!=0x3100){printf("FAIL RDE reload crda=%04x\n",d->done_crda);ok=false;}
 d->cfg_crda=d->done_crda;
 d->done_ready=1;tick();d->done_ready=0;
 if(!d->rx_axis_tready){printf("FAIL RX remained blocked after valid RDA reload\n");ok=false;}

 // Consuming the last advertised RRA entry raises RBE and blocks packets.
 // Once software advances RWP, its write-one-clear command fetches the next
 // resource and releases the receiver.
 putd(rra+4*wb,0,0x4000);putd(rra+4*wb,1,0);putd(rra+4*wb,2,0x0400);putd(rra+4*wb,3,0);
 d->cfg_rrp=rra+4*wb;d->cfg_rwp=rra;d->cfg_op=1;d->cfg_valid=1;
 while(!d->cfg_ready)tick();tick();d->cfg_valid=0;
 if(!wait_done()){printf("FAIL RBE-producing RRA fetch timeout\n");return 1;}
 if((d->done_isr_set&0x0020)==0){printf("FAIL last RRA entry did not raise RBE\n");ok=false;}
 d->done_ready=1;tick();d->done_ready=0;
 if(d->rx_axis_tready){printf("FAIL RX did not stop at RBE\n");ok=false;}
 d->cfg_rrp=d->done_rrp;d->cfg_rwp=rra+6*wb;d->cfg_op=1;d->cfg_valid=1;
 while(!d->cfg_ready)tick();tick();d->cfg_valid=0;
 if(!wait_done()){printf("FAIL replenished RRA fetch timeout\n");return 1;}
 d->done_ready=1;tick();d->done_ready=0;
 if(!d->rx_axis_tready){printf("FAIL RX remained blocked after RRA replenishment\n");ok=false;}

 auto rejected_frame=[&](unsigned n,bool bad,bool foreign,const char*name){
   std::vector<uint8_t> f(n,0x5a);f[0]=0x02;f[1]=foreign?0x11:0x34;
   f[2]=foreign?0x22:0x56;f[3]=foreign?0x33:0x78;f[4]=foreign?0x44:0x9a;f[5]=foreign?0x55:0xbc;
   size_t before=payload_cycles.size();
   for(unsigned j=0;j<n;j++){
     d->rx_axis_tdata=f[j];d->rx_axis_tvalid=1;d->rx_axis_tlast=(j+1==n);
     d->rx_axis_tuser=bad&&(j+1==n);int lim=1000;while(!d->rx_axis_tready&&lim--)tick();
     if(!d->rx_axis_tready){printf("FAIL %s input ready timeout\n",name);return false;}tick();
   }
   d->rx_axis_tvalid=0;d->rx_axis_tlast=0;d->rx_axis_tuser=0;
   for(int j=0;j<20;j++)tick();
   if(d->done_valid||payload_cycles.size()!=before||!d->rx_axis_tready){
     printf("FAIL rejected %s reached DMA/completion path\n",name);return false;
   }
   return true;
 };
 ok&=rejected_frame(68,false,true,"foreign unicast");
 ok&=rejected_frame(59,false,false,"runt");
 ok&=rejected_frame(68,true,false,"Taxi error");

 // A second accepted frame exercises request-side backpressure and the
 // ordinary non-EOL descriptor path (including clearing the next in-use
 // word only after payload retirement).
 std::vector<uint8_t> stalled(80);stalled[0]=0x02;stalled[1]=0x34;stalled[2]=0x56;
 stalled[3]=0x78;stalled[4]=0x9a;stalled[5]=0xbc;for(unsigned j=6;j<stalled.size();j++)stalled[j]=uint8_t(0xa0+j);
 uint32_t stalled_crc=0xffffffff;for(auto b:stalled)stalled_crc=crcbyte(stalled_crc,b);stalled_crc=~stalled_crc;
 std::vector<uint8_t> stalled_expected=stalled;for(int j=0;j<4;j++)stalled_expected.push_back(stalled_crc>>(8*j));
 size_t stalled_req_base=payload_cycles.size();stall_requests=true;rda_before_payload_done=false;
 for(unsigned j=0;j<stalled.size();j++){
   d->rx_axis_tdata=stalled[j];d->rx_axis_tvalid=1;d->rx_axis_tlast=(j+1==stalled.size());
   int lim=1000;while(!d->rx_axis_tready&&lim--)tick();if(!d->rx_axis_tready){printf("FAIL stalled frame input timeout\n");return 1;}tick();
 }
 d->rx_axis_tvalid=0;d->rx_axis_tlast=0;
 if(!wait_done(3000)){printf("FAIL stalled DMA packet completion timeout\n");return 1;}
 stall_requests=false;
 if(payload_cycles.size()!=stalled_req_base+2||rda_before_payload_done){printf("FAIL stalled DMA request/ordering behavior\n");ok=false;}
 for(unsigned j=0;j<stalled_expected.size();j++)if(mem[rba+j]!=stalled_expected[j]){printf("FAIL stalled payload/FCS byte %u\n",j);ok=false;break;}
 if(getd(0x3100,1)!=stalled_expected.size()){printf("FAIL stalled RDA length %u\n",getd(0x3100,1));ok=false;}
 // CRDA now advances to 0x3200 rather than reading back null: the ring is
 // linked one descriptor further so the full-size frame below has somewhere
 // to land.  The property under test is unchanged -- a non-EOL completion
 // reports PKTRX alone and moves CRDA on.
 if(d->done_crda!=0x3200||d->done_isr_set!=0x0400){printf("FAIL non-EOL completion crda=%04x isr=%04x\n",d->done_crda,d->done_isr_set);ok=false;}

 // ── A FULL-SIZE ETHERNET FRAME ────────────────────────────────────────
 // Every frame above this point is under 130 bytes, i.e. at most three
 // 64-byte packet-RAM chunks, so multi-chunk assembly AT SCALE has never
 // been exercised here.  1514 bytes is the largest standard Ethernet frame;
 // with the reconstructed FCS that is 1518 stored, spanning 24 of the 32
 // chunks and 12 payload DMA bursts instead of 3.
 //
 // This exists because hardware silently discards every frame over 964
 // bytes somewhere upstream of this engine -- no counter anywhere records
 // it -- which let TCP connect and then stall on the first full-size
 // segment.  If the fault turns out to be in THIS module, this case is what
 // catches it; if it passes here, the defect is upstream in the CDC FIFO or
 // the stream share, and that is worth knowing just as precisely.
 uint16_t full_base = d->done_crba0;      // where the engine will place it
 d->done_ready=1;tick();d->done_ready=0;
 // Stall the DMA while a MULTI-CHUNK frame is written.  The existing stalled
 // case above uses an 80-byte frame -- two chunks -- and losing a chunk to a
 // prefetch/stall race needs at least three, so that case could never catch
 // it.  On hardware, back-pressure at the 17th write corrupted everything
 // from that chunk on.
 stall_requests=true;
 std::vector<uint8_t> full(1514);
 full[0]=0x02;full[1]=0x34;full[2]=0x56;full[3]=0x78;full[4]=0x9a;full[5]=0xbc;
 for(unsigned j=6;j<full.size();j++) full[j]=uint8_t(j*7+3);
 uint32_t full_crc=0xffffffff;for(auto b:full)full_crc=crcbyte(full_crc,b);full_crc=~full_crc;
 std::vector<uint8_t> full_expected=full;
 for(int j=0;j<4;j++) full_expected.push_back(full_crc>>(8*j));
 for(unsigned j=0;j<full.size();j++){
   d->rx_axis_tdata=full[j];d->rx_axis_tvalid=1;d->rx_axis_tlast=(j+1==full.size());
   int lim=2000;while(!d->rx_axis_tready&&lim--)tick();
   if(!d->rx_axis_tready){printf("FAIL full-size frame input timeout at byte %u\n",j);return 1;}
   tick();
 }
 d->rx_axis_tvalid=0;d->rx_axis_tlast=0;
 if(!wait_done(20000)){printf("FAIL full-size frame completion timeout\n");return 1;}
 for(unsigned j=0;j<full_expected.size();j++){
   if(mem[full_base+j]!=full_expected[j]){
     printf("FAIL full-size byte %u (chunk %u) got=%02x exp=%02x\n",
            j,j>>6,mem[full_base+j],full_expected[j]);ok=false;break;
   }
 }
 stall_requests=false;
 if(getd(0x3200,1)!=full_expected.size()){
   printf("FAIL full-size RDA length %u want %zu\n",getd(0x3200,1),full_expected.size());ok=false;
 }
 if(d->done_isr_set!=0x0400){
   printf("FAIL full-size completion isr=%04x\n",d->done_isr_set);ok=false;
 }

 // ── BACK-TO-BACK FULL-SIZE FRAMES ─────────────────────────────────────
 // Every frame above arrives after the previous one has fully completed.
 // A TCP burst does not work that way: full-size frames arrive one after
 // another with only an inter-frame gap, so the engine must absorb frame
 // N+1 while still retiring frame N.  This is the pattern that fails on
 // hardware -- a bulk transfer stalls while interactive traffic is fine.
 //
 // The invariant is NOT "the second frame must be stored": the ring or the
 // buffer may legitimately be exhausted.  It is that the engine must SAY SO
 // -- a completion carrying RDE/RBAE, or back-pressure via rx_axis_tready.
 // Silently consuming the bytes and producing nothing is the failure that
 // makes this invisible to every counter, on hardware and here alike.
 d->done_ready=1;tick();d->done_ready=0;
 {
   size_t completions_before = payload_cycles.size();
   bool accepted_all = true;
   for(unsigned j=0;j<full.size();j++){
     d->rx_axis_tdata=full[j];d->rx_axis_tvalid=1;d->rx_axis_tlast=(j+1==full.size());
     int lim=4000;while(!d->rx_axis_tready&&lim--)tick();
     if(!d->rx_axis_tready){accepted_all=false;break;}   // back-pressure: legal
     tick();
   }
   d->rx_axis_tvalid=0;d->rx_axis_tlast=0;
   if(accepted_all){
     // Bytes were taken, so something must come back.
     bool completed = wait_done(20000);
     if(!completed){
       printf("FAIL back-to-back full-size frame was consumed with NO completion "
              "and NO back-pressure -- silently lost\n");
       ok=false;
     } else if(payload_cycles.size()==completions_before &&
               (d->done_isr_set & 0x0450)==0){
       printf("FAIL back-to-back frame produced neither payload nor a "
              "resource status (isr=%04x)\n",d->done_isr_set);
       ok=false;
     }
   }
 }
 if(ok)printf("PASS q700_sonic_rx %u-bit descriptors: DMA-loaded CAM filtering, Taxi-validated FCS reconstruction across a 64-byte boundary, consecutive payload writes, ordering and RDE/RBE recovery\n",wide_desc?32:16);
 delete d;return ok?0:1;
}
