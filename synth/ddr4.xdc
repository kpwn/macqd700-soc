################################################################################
# ddr4.xdc — DDR4 SODIMM constraints for the m68k-ooo KU5P board
#
# Part: xcku5p-ffvb676-2-i
# Board: "Mystery Chinese" KU5P module with on-board DDR4 (32-bit, x8×4)
#
# Ports live on the real-hardware `fpga_top` interface when SIM_MODEL is
# absent and are forwarded directly into the top-level MIG instance. Keep
# the port names here in lockstep with rtl/fpga_top.v.
#
# Pin locations were migrated from wip/ddr-peripheral-bringup:synth/base.xdc
# and verified against the DDR4 table CSV referenced there.  The
# The real-MIG non-project flow stitches the pcie_test DDR4 DCP after
# top-level synthesis. Recreate the generated in-context contract here so
# synth_design does not insert normal top-level IO/clock buffers around the
# MIG-owned DDR pins before the DCP is stitched.
#
# Pin summary:
#   14 control signals  (ACT_N, RESET_N, ODT, CS_N, CKE, CK_T, CK_C,
#                        BG, BA[1:0], plus ADR groupings)
#   17 address bits     (ADR[16:0] — includes WE/CAS/RAS multiplexing)
#    4 data masks       (DM_N[3:0])
#    4 DQS pairs        (DQS_T[3:0] / DQS_C[3:0])
#   32 data bits        (DQ[31:0])
#   TOTAL: 1 × 17 + 1 × 2 + 1 × 1 + 1 × 1 + 1 × 1 + 1 × 1 + 32 + 4 + 4 + 4 = 71 constrained DDR pins
#
# The known-good ~/FPGA/pcie_test DDR4 IP has C0.DDR4_EN_PARITY=false and
# exposes no parity/alert top-level ports.  Keep this shell aligned with
# that port set until a regenerated MIG IP says otherwise.
#
# `synth/vivado.tcl` reads this file only when USE_REAL_MIG=1.  SIM_MODEL
# builds intentionally omit these ports and constraints.
################################################################################

# ──────────────────────────────────────────────────────────────────────────────
# DDR4 reference clock (200 MHz differential, bank 66)
#
# This is the SAME physical pin as `sys_clk_p/n` in ku5p.xdc.  When
# real-hw integration lands, the board clock port will be promoted out of
# ku5p.xdc into this file, or aliased — but for now this constraint is
# commented out to avoid the duplicate-PACKAGE_PIN error.
# ──────────────────────────────────────────────────────────────────────────────
# set_property PACKAGE_PIN  T24   [get_ports sys_clk_p]
# set_property PACKAGE_PIN  U24   [get_ports sys_clk_n]
# set_property IOSTANDARD   DIFF_SSTL12 [get_ports sys_clk_p]
# set_property IOSTANDARD   DIFF_SSTL12 [get_ports sys_clk_n]
# create_clock -name ddr4_sys_clk -period 5.000 [get_ports sys_clk_p]

# ──────────────────────────────────────────────────────────────────────────────
# Control / command signals
# ──────────────────────────────────────────────────────────────────────────────
set _mig_phys_ports [get_ports {
    sys_clk_p sys_clk_n
    ddr4_act_n ddr4_reset_n
    ddr4_adr[*] ddr4_ba[*] ddr4_bg[*]
    ddr4_cke[*] ddr4_odt[*] ddr4_cs_n[*]
    ddr4_ck_t[*] ddr4_ck_c[*]
    ddr4_dq[*] ddr4_dqs_t[*] ddr4_dqs_c[*] ddr4_dm_dbi_n[*]
}]
set_property IO_BUFFER_TYPE NONE $_mig_phys_ports
set_property CLOCK_BUFFER_TYPE NONE $_mig_phys_ports
set_property DRIVE 8 [get_ports ddr4_reset_n]

# Match the known-good pcie_test MIG generated XDC.  The stitched OOC DCP
# still expects the top-level physical ports to carry the MIG-recommended
# electrical properties; otherwise hardware calibration can fail even when
# the pinout is correct.
set_property IOSTANDARD SSTL12_DCI [get_ports {
    ddr4_adr[*] ddr4_act_n ddr4_ba[*] ddr4_bg[*] ddr4_cke[*]
    ddr4_odt[*] ddr4_cs_n[*]
}]
set_property IOSTANDARD DIFF_SSTL12_DCI [get_ports {
    ddr4_ck_t[*] ddr4_ck_c[*]
}]
set_property IOSTANDARD POD12_DCI [get_ports {
    ddr4_dq[*] ddr4_dm_dbi_n[*]
}]
set_property IOSTANDARD DIFF_POD12_DCI [get_ports {
    ddr4_dqs_t[*] ddr4_dqs_c[*]
}]
set_property IOSTANDARD LVCMOS12 [get_ports ddr4_reset_n]

set_property OUTPUT_IMPEDANCE RDRV_40_40 [get_ports {
    ddr4_adr[*] ddr4_act_n ddr4_ba[*] ddr4_bg[*] ddr4_cke[*]
    ddr4_odt[*] ddr4_cs_n[*] ddr4_ck_t[*] ddr4_ck_c[*]
    ddr4_dq[*] ddr4_dqs_t[*] ddr4_dqs_c[*] ddr4_dm_dbi_n[*]
}]

set_property SLEW FAST [get_ports {
    ddr4_adr[*] ddr4_act_n ddr4_ba[*] ddr4_bg[*] ddr4_cke[*]
    ddr4_ck_t[*] ddr4_ck_c[*] ddr4_odt[*] ddr4_cs_n[*]
    ddr4_dq[*] ddr4_dqs_t[*] ddr4_dqs_c[*] ddr4_dm_dbi_n[*]
}]
set_property IBUF_LOW_PWR FALSE [get_ports {
    ddr4_dq[*] ddr4_dqs_t[*] ddr4_dqs_c[*] ddr4_dm_dbi_n[*]
}]
set_property ODT RTT_40 [get_ports {
    ddr4_dq[*] ddr4_dqs_t[*] ddr4_dqs_c[*] ddr4_dm_dbi_n[*]
}]
set_property EQUALIZATION EQ_LEVEL2 [get_ports {
    ddr4_dq[*] ddr4_dqs_t[*] ddr4_dqs_c[*] ddr4_dm_dbi_n[*]
}]
set_property PRE_EMPHASIS RDRV_240 [get_ports {
    ddr4_dq[*] ddr4_dqs_t[*] ddr4_dqs_c[*] ddr4_dm_dbi_n[*]
}]
set_property DATA_RATE SDR [get_ports {
    ddr4_adr[*] ddr4_act_n ddr4_ba[*] ddr4_bg[*] ddr4_cke[*]
    ddr4_odt[*] ddr4_cs_n[*]
}]
set_property DATA_RATE DDR [get_ports {
    ddr4_dq[*] ddr4_dqs_t[*] ddr4_dqs_c[*] ddr4_dm_dbi_n[*]
    ddr4_ck_t[*] ddr4_ck_c[*]
}]

set_property PACKAGE_PIN  P24   [get_ports ddr4_act_n]
set_property PACKAGE_PIN  P19   [get_ports ddr4_reset_n]
set_property PACKAGE_PIN  R23   [get_ports {ddr4_odt[0]}]
set_property PACKAGE_PIN  P25   [get_ports {ddr4_cs_n[0]}]
set_property PACKAGE_PIN  P20   [get_ports {ddr4_cke[0]}]
set_property PACKAGE_PIN  V24   [get_ports {ddr4_ck_t[0]}]
set_property PACKAGE_PIN  W24   [get_ports {ddr4_ck_c[0]}]
set_property PACKAGE_PIN  R22   [get_ports {ddr4_bg[0]}]
set_property PACKAGE_PIN  P21   [get_ports {ddr4_ba[0]}]
set_property PACKAGE_PIN  P26   [get_ports {ddr4_ba[1]}]
# ──────────────────────────────────────────────────────────────────────────────
# Address (A0–A16 — includes WE_B, CAS_B, RAS_B mux on A[14], A[15], A[16])
# ──────────────────────────────────────────────────────────────────────────────
set_property PACKAGE_PIN  Y22   [get_ports {ddr4_adr[0]}]
set_property PACKAGE_PIN  Y25   [get_ports {ddr4_adr[1]}]
set_property PACKAGE_PIN  W23   [get_ports {ddr4_adr[2]}]
set_property PACKAGE_PIN  V26   [get_ports {ddr4_adr[3]}]
set_property PACKAGE_PIN  R26   [get_ports {ddr4_adr[4]}]
set_property PACKAGE_PIN  U26   [get_ports {ddr4_adr[5]}]
set_property PACKAGE_PIN  R21   [get_ports {ddr4_adr[6]}]
set_property PACKAGE_PIN  W25   [get_ports {ddr4_adr[7]}]
set_property PACKAGE_PIN  R20   [get_ports {ddr4_adr[8]}]
set_property PACKAGE_PIN  Y26   [get_ports {ddr4_adr[9]}]
set_property PACKAGE_PIN  R25   [get_ports {ddr4_adr[10]}]
set_property PACKAGE_PIN  V23   [get_ports {ddr4_adr[11]}]
set_property PACKAGE_PIN  AA24  [get_ports {ddr4_adr[12]}]
set_property PACKAGE_PIN  W26   [get_ports {ddr4_adr[13]}]
set_property PACKAGE_PIN  P23   [get_ports {ddr4_adr[14]}]  ;# WE_B
set_property PACKAGE_PIN  AA25  [get_ports {ddr4_adr[15]}]  ;# CAS_B
set_property PACKAGE_PIN  T25   [get_ports {ddr4_adr[16]}]  ;# RAS_B

# ──────────────────────────────────────────────────────────────────────────────
# Data masks / DBI_N (one per byte lane — 4 lanes for 32-bit data)
# ──────────────────────────────────────────────────────────────────────────────
set_property PACKAGE_PIN  AE25  [get_ports {ddr4_dm_dbi_n[0]}]
set_property PACKAGE_PIN  AE22  [get_ports {ddr4_dm_dbi_n[1]}]
set_property PACKAGE_PIN  AD20  [get_ports {ddr4_dm_dbi_n[2]}]
set_property PACKAGE_PIN  Y20   [get_ports {ddr4_dm_dbi_n[3]}]

# ──────────────────────────────────────────────────────────────────────────────
# DQS strobes (4 pairs — one per byte lane)
# ──────────────────────────────────────────────────────────────────────────────
set_property PACKAGE_PIN  AC26  [get_ports {ddr4_dqs_t[0]}]
set_property PACKAGE_PIN  AD26  [get_ports {ddr4_dqs_c[0]}]
set_property PACKAGE_PIN  AA22  [get_ports {ddr4_dqs_t[1]}]
set_property PACKAGE_PIN  AB22  [get_ports {ddr4_dqs_c[1]}]
set_property PACKAGE_PIN  AC18  [get_ports {ddr4_dqs_t[2]}]
set_property PACKAGE_PIN  AD18  [get_ports {ddr4_dqs_c[2]}]
set_property PACKAGE_PIN  AB17  [get_ports {ddr4_dqs_t[3]}]
set_property PACKAGE_PIN  AC17  [get_ports {ddr4_dqs_c[3]}]

# ──────────────────────────────────────────────────────────────────────────────
# Data bus DQ[0..31] (byte lane 0..3)
# ──────────────────────────────────────────────────────────────────────────────
set_property PACKAGE_PIN  AB26  [get_ports {ddr4_dq[0]}]
set_property PACKAGE_PIN  AB25  [get_ports {ddr4_dq[1]}]
set_property PACKAGE_PIN  AF25  [get_ports {ddr4_dq[2]}]
set_property PACKAGE_PIN  AF24  [get_ports {ddr4_dq[3]}]
set_property PACKAGE_PIN  AD25  [get_ports {ddr4_dq[4]}]
set_property PACKAGE_PIN  AD24  [get_ports {ddr4_dq[5]}]
set_property PACKAGE_PIN  AC24  [get_ports {ddr4_dq[6]}]
set_property PACKAGE_PIN  AB24  [get_ports {ddr4_dq[7]}]
set_property PACKAGE_PIN  AE23  [get_ports {ddr4_dq[8]}]
set_property PACKAGE_PIN  AD23  [get_ports {ddr4_dq[9]}]
set_property PACKAGE_PIN  AC23  [get_ports {ddr4_dq[10]}]
set_property PACKAGE_PIN  AC22  [get_ports {ddr4_dq[11]}]
set_property PACKAGE_PIN  AE21  [get_ports {ddr4_dq[12]}]
set_property PACKAGE_PIN  AD21  [get_ports {ddr4_dq[13]}]
set_property PACKAGE_PIN  AC21  [get_ports {ddr4_dq[14]}]
set_property PACKAGE_PIN  AB21  [get_ports {ddr4_dq[15]}]
set_property PACKAGE_PIN  AD19  [get_ports {ddr4_dq[16]}]
set_property PACKAGE_PIN  AC19  [get_ports {ddr4_dq[17]}]
set_property PACKAGE_PIN  AF19  [get_ports {ddr4_dq[18]}]
set_property PACKAGE_PIN  AF18  [get_ports {ddr4_dq[19]}]
set_property PACKAGE_PIN  AF17  [get_ports {ddr4_dq[20]}]
set_property PACKAGE_PIN  AE17  [get_ports {ddr4_dq[21]}]
set_property PACKAGE_PIN  AE16  [get_ports {ddr4_dq[22]}]
set_property PACKAGE_PIN  AD16  [get_ports {ddr4_dq[23]}]
set_property PACKAGE_PIN  AB19  [get_ports {ddr4_dq[24]}]
set_property PACKAGE_PIN  AA19  [get_ports {ddr4_dq[25]}]
set_property PACKAGE_PIN  AB20  [get_ports {ddr4_dq[26]}]
set_property PACKAGE_PIN  AA20  [get_ports {ddr4_dq[27]}]
set_property PACKAGE_PIN  AA17  [get_ports {ddr4_dq[28]}]
set_property PACKAGE_PIN  Y17   [get_ports {ddr4_dq[29]}]
set_property PACKAGE_PIN  AA18  [get_ports {ddr4_dq[30]}]
set_property PACKAGE_PIN  Y18   [get_ports {ddr4_dq[31]}]

# ──────────────────────────────────────────────────────────────────────────────
# Timing hints
#
# The MIG IP ships its own IP-level .xdc that drives the DDR4 primitive
# timing.  This file only constrains board-level pin location.  The
# 200 MHz clock from sys_clk_p feeds only the MIG c0_sys_clk input in
# real-MIG builds.  The SoC fabric clock is the separate AB7/AB6 MGTREFCLK
# constrained in fpga_top.xdc.
# ──────────────────────────────────────────────────────────────────────────────
