# Board-specific SD pin and timing checks after routing.
# A resumed route opens a checkpoint in a fresh Vivado process, where the
# original build environment may be absent.  The routed top-level ports are
# authoritative: native SD has sd_cmd; SPI has sd_mosi/sd_cs_n.  When the
# environment is provided, reject a mismatch rather than checking the wrong
# interface and producing a misleading timing error.
set native_sd_ports [get_ports -quiet sd_cmd]
if {[llength $native_sd_ports] > 1} {error "multiple sd_cmd ports"}
set native_sd_mode [expr {[llength $native_sd_ports] == 1}]
if {[info exists ::env(TQTT_NATIVE_SD)]} {
    if {$::env(TQTT_NATIVE_SD) ni {0 1}} {
        error "TQTT_NATIVE_SD must be 0 or 1"
    }
    if {$::env(TQTT_NATIVE_SD) != $native_sd_mode} {
        error "TQTT_NATIVE_SD=$::env(TQTT_NATIVE_SD) disagrees with routed SD ports"
    }
}
if {$native_sd_mode} {
    foreach {sd_port sd_pin} {
        sd_clk AE15
        sd_cmd AD15
        {sd_dat[0]} AF13
        {sd_dat[1]} AE13
        {sd_dat[2]} AF15
        {sd_dat[3]} AF14
    } {
        set port [get_ports $sd_port]
        if {[llength $port] != 1} {error "native SD port $sd_port is missing"}
        if {[get_property PACKAGE_PIN $port] ne $sd_pin} {
            error "native SD port $sd_port is not on $sd_pin"
        }
        if {[get_property IOSTANDARD $port] ne "LVCMOS33"} {
            error "native SD port $sd_port is not LVCMOS33"
        }
        puts "NATIVE_SD_PIN $sd_port $sd_pin LVCMOS33"
    }
} else {
    # SPI pads have explicit 4ns output and 3ns input allocations.
    foreach sd_output {sd_clk sd_mosi sd_cs_n} {
        set sd_paths [get_timing_paths -to [get_ports $sd_output] -delay_type max -max_paths 1]
        if {[llength $sd_paths] == 0} {error "SPI output $sd_output is not timed"}
        if {abs([get_property REQUIREMENT $sd_paths] - 4.0) > 0.001} {
            error "SPI output $sd_output lost its 4ns latency allocation"
        }
        puts "SPI_IO_BUDGET $sd_output slack=[get_property SLACK $sd_paths]"
    }
    set sd_input_paths [get_timing_paths -from [get_ports sd_miso] -delay_type max -max_paths 1]
    if {[llength $sd_input_paths] == 0} {error "SPI input sd_miso is not timed"}
    if {abs([get_property REQUIREMENT $sd_input_paths] - 3.0) > 0.001} {
        error "SPI input sd_miso lost its 3ns latency allocation"
    }
    puts "SPI_IO_BUDGET sd_miso slack=[get_property SLACK $sd_input_paths]"
    report_timing -from [get_ports sd_miso] -delay_type max -max_paths 4 \
        -file $output_dir/reports/sd_miso_timing.rpt
    report_timing -to [get_ports {sd_clk sd_mosi sd_cs_n}] -delay_type max -max_paths 6 \
        -file $output_dir/reports/sd_output_timing.rpt
}
