# 200 MHz timing campaign — WITH ETHERNET (2026-09-13)

Every build here: `CPU=m68k040 ETH_ENABLE=1 ETH_ICMP_RESPONDER=0
ETH_RK5_DIR=/home/qwertyoruiop/rk5-eth CORE_MMCM=1`, KU5P `xcku5p-ffvb676-2-i`,
Vivado 2025.2, `L2C_ENABLE=1 VRAM_IN_DDR=1 USE_REAL_MIG=1 ENABLE_VIO=1
ENABLE_JTAG_AXI=1`.

`ETH_ICMP_RESPONDER=0` (the SONIC packet adapter) and not the default ICMP
responder: in the ICMP arm `q700_eth_link` ties `sonic_tx_tready`/`sonic_rx_*`
off and the SONIC DMA client is trimmed, so only that arm puts BOTH the taxi
MAC and the sonic datapath in the netlist.

## Ethernet is genuinely in (the previous campaign's builds had ETH_ENABLE=0)

Log line, both builds: `Ethernet: ETH_ENABLE=1 ETH_ICMP_RESPONDER=0 ETH_DEBUG_ENABLE=0`
Manifest, both builds: `eth_enable=1`, `eth_icmp_responder=0`
Routed netlist (`reports/utilization_route.rpt`, post-route hierarchy):
`u_q700_eth_link` / `q700_eth_link`, `eth_mac_inst` / `taxi_eth_mac_1g_rgmii_fifo`,
`taxi_eth_mac_1g_rgmii`, `taxi_eth_mac_1g`, `taxi_axis_gmii_rx`, `taxi_axis_gmii_tx`,
`taxi_axis_async_fifo*`, `g_sonic_dma_client.u_q700_sonic_rx`,
`g_sonic_dma_client.u_q700_sonic_tx`, `q700_eth_stream_share`.

For contrast the ETH_ENABLE=0 150 MHz build contains exactly one eth-ish
instance -- `u_q700_eth_sonic` (300 LUTs, the register block that is present in
every build) -- and ZERO taxi cells.  `u_q700_eth_sonic` is therefore NOT
evidence of ethernet; `q700_eth_link` + `taxi_*` is.

Ethernet costs 177847 - 167426 = +10421 LUTs (+6.2%), 82% of the device.

## THE RESULT: core:pb must be an INTEGER ratio.  120 MHz is the WORST choice.

CORE_MMCM puts core and pb on one 1200 MHz VCO and keeps them phase-related, so
Vivado times every core<->pb path as a SYNCHRONOUS transfer.  The setup window
is then gcd(core_period, 20 ns), which collapses when core/50 is not an integer:

| core | ratio | gcd window | pb->core WNS | core->pb WNS | failing EPs |
|------|-------|-----------|--------------|--------------|-------------|
| 100 MHz | 2:1 | 10.000 ns | clean | clean | 0 |
| 150 MHz | 3:1 |  6.667 ns | +0.255 | +0.533 | 0 |
| 200 MHz | 4:1 |  5.000 ns | -0.356 | 0.000 | 622 |
| **120 MHz** | **12:5** | **1.667 ns** | **-3.493** | **-3.133** | **19431** |

120 MHz gives every one of ~20600 core<->pb paths a 1.667 ns budget instead of
6.667.  Measured at post-place: pb->core TNS -16129.115 over 7966/8916
endpoints, core->pb TNS -14082.759 over 11465/11685.  Routing cannot recover
3.5 ns, so that build was abandoned mid-route rather than finished.

**The "120 MHz is the safe speedup, ~1.4 ns margin" advice is wrong.**  It was
an intra-core-clock number and never looked at the core<->pb transfer this
clock topology deliberately relies on.  Only 100 / 150 / 200 are legal.

## 200 MHz + ethernet: DOES NOT CLOSE

`build/vivado200eth`, build_id `0x61e3259a`.  Post-route:

| clock | setup WNS | fail EPs | hold WHS | fail EPs |
|-------|-----------|----------|----------|----------|
| core_mmcm_clkout0 (200.000 MHz) | **-0.678** | 22520 | -0.084 | 12 |
| mmcm_clkout0 (333.333 MHz MIG UI) | 0.000 | 0 | **-0.067** | **62** |
| core_mmcm_clkout1 (pb, 50.000 MHz exact) | +8.799 | 0 | +0.011 | 0 |
| pclk_unbuf (video 148.5) | +0.001 | 0 | +0.002 | 0 |
| clk_125mhz_mmcm (eth MAC) | +0.095 | 0 | +0.019 | 0 |
| phy_rx_clk (RGMII) | +4.097 | 0 | +0.016 | 0 |

Design: WNS -0.678, TNS -5655.007, 22520 failing setup; WHS -0.084, 74 failing
hold.  Implied core Fmax 1/(5.000+0.678) = **176.1 MHz**.

Top failing cones (all tied at WNS, all CPU-internal):
1. -0.678, 16 lvl, 67% route: `RobPlugin_logic_exc_fsFrameBase_reg[1]_replica` -> `DtlbPlugin_logic_missReqReg_write_reg/CE`
2. -0.678, 11 lvl, 84% route: `DcachePlugin_logic_loadShadowValid_reg` -> `DcachePlugin_logic_maint_lastSet_reg[1]/CE`
3. -0.678, 19 lvl, 67% route: `IssueQueuePlugin_logic_lines_0_ways_0_sel_reg` -> `IssueQueuePlugin_logic_selPorts_1_rData_pFpccDst_reg[2]`
4. -0.677,  6 lvl, 89% route: `RobPlugin_logic_exc_fsm_stateReg_reg[1]_rep__2` -> `RobPlugin_logic_sysValStore_61_reg[14]/CE`

The FetchAlignPlugin cone that dominates POST-SYNTH (-1.694, 30 levels: async
FTQ RAM read -> 32-bit ftqDiff carry chain -> the whole PredecodeWord length
decoder) is levelled by place/phys_opt into this 22520-endpoint plateau.  The
problem is the PLATEAU, not any one cone: 22520 endpoints all sit within
~0.7 ns, so retiming one cone buys almost nothing.  Ethernet is NOT implicated
anywhere -- every eth clock is clean at synth, place and route.

## 150 MHz + ethernet: CLOSES COMPLETELY.  "All user specified timing constraints are met."

`build/vivado150eth`, artifacts copied to
`artifacts_eth/fpga_top_150mhz_eth_61e3259a.{bit,ltx,buildinfo}`.  Post-route:

| clock | setup WNS | fail EPs | hold WHS | fail EPs |
|-------|-----------|----------|----------|----------|
| core_mmcm_clkout0 (150.000 MHz) | **+0.159** | 0 | +0.010 | 0 |
| mmcm_clkout0 (333.333 MHz MIG UI) | **+0.043** | 0 | +0.010 | 0 |
| core_mmcm_clkout1 (pb, **50.000 MHz exact**) | +9.279 | 0 | +0.014 | 0 |
| pclk_unbuf | +0.066 | 0 | +0.026 | 0 |
| clk_125mhz_mmcm (eth MAC) | +0.202 | 0 | +0.037 | 0 |
| phy_rx_clk | +3.523 | 0 | +0.014 | 0 |
| clk_312mhz_mmcm | +2.575 | 0 | +0.087 | 0 |

Design: WNS +0.043, TNS 0.000, 0 failing setup; WHS +0.010, THS 0.000, 0
failing hold; 0 failing pulse-width.  Inter-clock core<->pb +0.255 / +0.533,
0 failing.

Worst core cone is no longer in the CPU: `u_l2c/g_active.u_ctrl/req_id_reg[2]`
-> `u_l2c/.../u_data/g_way[7].mem_reg_uram_1/ADDR_A[6]`, +0.159, 12 levels,
82% route.  Design-limiting path is the MIG one:
`u_ddr/u_repo_to_pcie_mig/wq_count_reg[2]` -> `wr_pack_data_reg[72]/CE`, +0.043,
7 levels.

## Why this 150 differs from the 150 that DIED on silicon

The earlier 150 MHz build (ETH OFF, build_id 0xF1E64B31) did NOT meet timing.
Its core met by +0.041, but `mmcm_clkout0` -- the 333 MHz MIG UI domain -- was
**WNS -0.007 with 4 VIOLATED setup endpoints**, all
`u_ddr/u_core_to_mig_ui/aw_stage_data_reg[19]_replica_1` ->
`u_ddr/u_repo_to_pcie_mig/wr_pad_remaining_reg[*]/CE` (12 levels, 5x CARRY8,
60% route) -- the DDR WRITE-PADDING counter.  The 100 MHz build on the same
commit is clean everywhere.

A corrupted write-pad count corrupts what the boot FSM writes into DRAM, which
includes the ROM image.  That is a direct mechanism for "illegal instruction
(vector 4) at ROM 0x40846dc0, exc_count frozen at 1" and it does not require
the core clock to be marginal at all.  The new 150 MHz build closes that domain
at +0.043 with zero failing endpoints, so the leading candidate cause of the
silicon death is absent from this bitstream.

Stated honestly: this is a DIFFERENT PLACEMENT, not a fix.  Nothing in the RTL
changed to make the MIG domain close; the +10k LUTs of ethernet changed the
netlist and the placer landed better.  It is therefore not guaranteed to
reproduce on the next build.

## Recommendation

Bench `artifacts_eth/fpga_top_150mhz_eth_61e3259a.bit`, i.e. 150 MHz.

- It is the ONLY frequency above 100 that is structurally legal (integer
  core:pb ratio) AND fully timing-clean.
- 120 MHz must NOT be tried: -3.493 ns and 19431 failing endpoints.
- 200 MHz is out of reach: -0.678 ns core over 22520 endpoints plus 62 failing
  HOLD endpoints on the MIG domain (hold does not improve by slowing the core).
  Implied ceiling 176 MHz, and the cost is a broad plateau, not one cone.
- Margin honesty: core +0.159 ns, design +0.043 ns (MIG-limited).  For scale,
  the 100 MHz bitstream that BOOTS is core +0.753 / design +0.053 -- so this
  150 has ~4.7x less core margin, but a slightly BETTER design margin, because
  the MIG domain sits near zero at every frequency.
- If 150 still dies on the bench, the next experiment is NOT a lower frequency
  -- it is `CORE_MMCM=1 CORE_CLK_HZ=100_000_000`.  The booting 100 MHz
  bitstream uses the OLD BUFG_GT divider topology; no CORE_MMCM build has ever
  been proven on silicon.  That control separates "the MMCM clock topology is
  broken" from "the core cannot run at 150".

## Build-id caveat

Both builds in this campaign carry build_id `0x61e3259a` (the build_id is the
SoC git SHA and only one commit was made).  Tell them apart by artifact path or
by `core_clk_hz` in the manifest, NOT by build_id.
