# Applied after synth_design because these objects are internal hierarchy.
set_property DELAY_VALUE 0 [get_cells {u_q700_eth_link/phy_rx_ctl_idelay u_q700_eth_link/phy_rxd_idelay_bit[*].idelay_inst}]
set_property CLKOUT1_PHASE 90 [get_cells u_q700_eth_link/clk_mmcm_inst]

# TX completion is transferred from the 125 MHz MAC domain into the fabric
# domain with a toggle and a three-flop ASYNC_REG synchronizer.  These clocks
# have a common physical source, so Vivado otherwise times the first stage
# against their nearest nominal edge (a 2 ns requirement) even though that
# stage is deliberately allowed to go metastable.  Keep the route short with
# the same one-source-period datapath bound used by Taxi's async FIFOs; do not
# false-path the synchronizer stages after sync1.
#
# THIS CDC EXISTS ONLY IN THE SONIC CONFIGURATION.  q700_eth_link generates
# EITHER g_icmp_responder (ETH_ICMP_RESPONDER=1, the default) OR
# g_sonic_packet_adapter (=0); tx_cpl_toggle and u_tx_cpl_rx live in the latter.
# This block used to demand those cells UNCONDITIONALLY, so an ICMP-responder
# build died here with "did not resolve uniquely" -- a real configuration
# reported as corruption.  It survived a long time because the recovery flows
# resume from a placed checkpoint and never re-run post-synth constraints, so
# only a FULL build from RTL reaches it.
#
# Gate on the generic, and keep the hard error for the configuration that is
# actually supposed to have these registers -- a silently skipped CDC
# constraint is exactly the failure this guard was added to prevent.
set icmp_responder [expr {[info exists ::eth_icmp_responder] ? $::eth_icmp_responder : 1}]
set tx_cpl_toggle_reg [get_cells -quiet \
    u_q700_eth_link/g_sonic_packet_adapter.tx_cpl_toggle_reg]
# The reset-safe receiver moved the synchronizer into q700_toggle_rx. Its
# W=1, SYNC_FF=3 synthesis emits sync_q_reg[0][0] as the first stage.
# Match only that bit/stage; never constrain the entire synchronizer chain.
set tx_cpl_sync1_reg [get_cells -quiet -hierarchical -regexp \
    {u_q700_eth_link/g_sonic_packet_adapter\.u_tx_cpl_rx/sync_q_reg\[0\]\[0\]}]
if {$icmp_responder} {
    # Inverse drift check: if the SONIC adapter somehow IS generated in an
    # ICMP build, the branch selection is wrong and the constraint below would
    # be skipped silently.  Say so instead.
    if {[llength $tx_cpl_toggle_reg] != 0 || [llength $tx_cpl_sync1_reg] != 0} {
        error "ETH_ICMP_RESPONDER=1 but the SONIC packet adapter was generated"
    }
    puts "ethernet_rgmii_post: ETH_ICMP_RESPONDER=1 -- no SONIC TX completion CDC, skipping"
} else {
    if {[llength $tx_cpl_toggle_reg] != 1 || [llength $tx_cpl_sync1_reg] != 1} {
        error "SONIC TX completion CDC registers did not resolve uniquely"
    }
    puts "SONIC TX completion CDC: $tx_cpl_toggle_reg -> $tx_cpl_sync1_reg"
    set_max_delay -from $tx_cpl_toggle_reg -to $tx_cpl_sync1_reg \
        -datapath_only 8.0
}
