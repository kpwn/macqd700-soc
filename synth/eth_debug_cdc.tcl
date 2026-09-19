# The optional Ethernet telemetry samples Taxi status from clk_125mhz into
# core_clk.  mac_tog_meta/link_meta are the explicit first metastability
# stages; timing their D pins as ordinary related-clock endpoints makes the
# debug observer compete with functional 100 MHz paths and reports a bogus
# setup failure.  Only the first stages are cut.  The meta->sync paths remain
# timed normally, preserving placement of the synchronizer pairs.
set eth_debug_cdc_meta [get_cells -quiet -hier -filter \
    {NAME =~ "*u_eth_debug_regs/mac_tog_meta_reg*" || \
     NAME =~ "*u_eth_debug_regs/link_meta_reg*"}]
if {[llength $eth_debug_cdc_meta] != 10} {
    error "ETH debug CDC constraint expected 10 first-stage FFs, found [llength $eth_debug_cdc_meta]: $eth_debug_cdc_meta"
}
set eth_debug_cdc_d [get_pins -quiet -of_objects $eth_debug_cdc_meta \
    -filter {REF_PIN_NAME == D}]
if {[llength $eth_debug_cdc_d] != 10} {
    error "ETH debug CDC constraint expected 10 D pins, found [llength $eth_debug_cdc_d]: $eth_debug_cdc_d"
}
set_false_path -to $eth_debug_cdc_d
puts "=== ETH DEBUG CDC: false-pathed 10 Taxi->core first-stage synchronizer D pins ==="
