// Acknowledged RX configuration/completion mailboxes plus enable synchronizer.
`default_nettype none
module q700_sonic_rx_cdc (
 input wire pb_clk,pb_rst,input wire pb_enable,
 input wire pb_cfg_valid,output wire pb_cfg_ready,input wire [2:0] pb_cfg_op,
 input wire [15:0] pb_cfg_dcr,pb_cfg_rcr,pb_cfg_urda,pb_cfg_crda,pb_cfg_urra,
 input wire [15:0] pb_cfg_rsa,pb_cfg_rea,pb_cfg_rrp,pb_cfg_rwp,pb_cfg_eobc,pb_cfg_rsc,pb_cfg_llfa,pb_cfg_cdp,pb_cfg_cdc,
 output wire pb_done_valid,input wire pb_done_ready,output wire pb_done_error,
 output wire [15:0] pb_done_rcr,pb_done_crda,pb_done_crba0,pb_done_crba1,
 output wire [15:0] pb_done_rbwc0,pb_done_rbwc1,pb_done_rrp,pb_done_rsc,pb_done_llfa,
 output wire [15:0] pb_done_trba0,pb_done_trba1,pb_done_tbwc0,pb_done_tbwc1,pb_done_isr_set,
 output wire [15:0] pb_done_cdp,pb_done_cdc,pb_done_ce,
 input wire core_clk,core_rst,output wire core_enable,
 output wire core_cfg_valid,input wire core_cfg_ready,output wire [2:0] core_cfg_op,
 output wire [15:0] core_cfg_dcr,core_cfg_rcr,core_cfg_urda,core_cfg_crda,core_cfg_urra,
 output wire [15:0] core_cfg_rsa,core_cfg_rea,core_cfg_rrp,core_cfg_rwp,core_cfg_eobc,core_cfg_rsc,core_cfg_llfa,core_cfg_cdp,core_cfg_cdc,
 input wire core_done_valid,output wire core_done_ready,input wire core_done_error,
 input wire [15:0] core_done_rcr,core_done_crda,core_done_crba0,core_done_crba1,
 input wire [15:0] core_done_rbwc0,core_done_rbwc1,core_done_rrp,core_done_rsc,core_done_llfa,
 input wire [15:0] core_done_trba0,core_done_trba1,core_done_tbwc0,core_done_tbwc1,core_done_isr_set,
 input wire [15:0] core_done_cdp,core_done_cdc,core_done_ce
);
 (* ASYNC_REG="TRUE" *) reg en1,en2,cv1,cv2,ca1,ca2,dv1,dv2,da1,da2;
 reg core_cfg_ack,pb_done_ack;
 assign core_enable=en2; assign core_cfg_valid=cv2&&!core_cfg_ack; assign pb_cfg_ready=ca2;
 assign core_cfg_op=pb_cfg_op;
 assign pb_done_valid=dv2&&!pb_done_ack; assign core_done_ready=da2;
 assign core_cfg_dcr=pb_cfg_dcr;assign core_cfg_rcr=pb_cfg_rcr;assign core_cfg_urda=pb_cfg_urda;
 assign core_cfg_crda=pb_cfg_crda;assign core_cfg_urra=pb_cfg_urra;assign core_cfg_rsa=pb_cfg_rsa;
 assign core_cfg_rea=pb_cfg_rea;assign core_cfg_rrp=pb_cfg_rrp;assign core_cfg_rwp=pb_cfg_rwp;
 assign core_cfg_eobc=pb_cfg_eobc;assign core_cfg_rsc=pb_cfg_rsc;assign core_cfg_llfa=pb_cfg_llfa;
 assign core_cfg_cdp=pb_cfg_cdp;assign core_cfg_cdc=pb_cfg_cdc;
 assign pb_done_error=core_done_error;assign pb_done_rcr=core_done_rcr;assign pb_done_crda=core_done_crda;
 assign pb_done_crba0=core_done_crba0;assign pb_done_crba1=core_done_crba1;
 assign pb_done_rbwc0=core_done_rbwc0;assign pb_done_rbwc1=core_done_rbwc1;
 assign pb_done_rrp=core_done_rrp;assign pb_done_rsc=core_done_rsc;assign pb_done_llfa=core_done_llfa;
 assign pb_done_trba0=core_done_trba0;assign pb_done_trba1=core_done_trba1;
 assign pb_done_tbwc0=core_done_tbwc0;assign pb_done_tbwc1=core_done_tbwc1;assign pb_done_isr_set=core_done_isr_set;
 assign pb_done_cdp=core_done_cdp;assign pb_done_cdc=core_done_cdc;assign pb_done_ce=core_done_ce;
 always @(posedge core_clk) if(core_rst)begin en1<=0;en2<=0;cv1<=0;cv2<=0;core_cfg_ack<=0;da1<=0;da2<=0;end else begin
   en1<=pb_enable;en2<=en1;cv1<=pb_cfg_valid;cv2<=cv1;
   if(core_cfg_valid&&core_cfg_ready)core_cfg_ack<=1;else if(!cv2)core_cfg_ack<=0;
   da1<=pb_done_ack;da2<=da1;
 end
 always @(posedge pb_clk) if(pb_rst)begin ca1<=0;ca2<=0;dv1<=0;dv2<=0;pb_done_ack<=0;end else begin
   ca1<=core_cfg_ack;ca2<=ca1;dv1<=core_done_valid;dv2<=dv1;
   if(pb_done_valid&&pb_done_ready)pb_done_ack<=1;else if(!dv2)pb_done_ack<=0;
 end
endmodule
`default_nettype wire
