# gen_pcie_xdma.tcl -- repo-owned PCIe/XDMA generator for the KU5P board.
#
# Usage:
#   vivado -mode batch -source synth/gen_pcie_xdma.tcl -tclargs validate build/pcie_xdma
#   vivado -mode batch -source synth/gen_pcie_xdma.tcl -tclargs synth    build/pcie_xdma
#
# `validate` creates the configured XCI and generated targets, then checks the
# key pcie_test-derived parameters.  `synth` also runs OOC IP synthesis and
# writes build/pcie_xdma/design_1_xdma_0_0.dcp for synth/vivado.tcl to read.
#
# Source of truth:
#   /offlinenas/share/FPGA/pcie_test/pcie_test.srcs/sources_1/bd/design_1/ip/
#       design_1_xdma_0_0/design_1_xdma_0_0.xci
#
# Board constraints are intentionally not generated here.  The top-level flow
# owns PCIe lane/PERST placement in synth/pcie_xdma.xdc and the AB7/AB6
# MGTREFCLK pair in synth/fpga_top_real_mig.xdc.

set mode [lindex $argv 0]
set output_dir [lindex $argv 1]

if {$mode eq ""} {
    set mode "synth"
}

set script_dir [file normalize [file dirname [info script]]]
set proj_root  [file normalize [file join $script_dir ..]]
if {$output_dir eq ""} {
    set output_dir [file normalize [file join $proj_root build pcie_xdma]]
} else {
    set output_dir [file normalize $output_dir]
}

if {$mode ne "validate" && $mode ne "synth"} {
    puts stderr "ERROR: mode must be one of {validate, synth}; got: $mode"
    exit 1
}

set part        "xcku5p-ffvb676-2-i"
set ip_name     "design_1_xdma_0_0"
set ip_root     [file join $output_dir ip]
set ip_dir      [file join $ip_root $ip_name]
set root_xci    [file join $output_dir ${ip_name}.xci]
set root_dcp    [file join $output_dir ${ip_name}.dcp]
set root_stub   [file join $output_dir ${ip_name}_stub.v]
set manifest    [file join $output_dir ${ip_name}_manifest.txt]

set_param general.maxThreads 16

proc find_named_file {root filename} {
    foreach item [glob -nocomplain -directory $root *] {
        if {[file isdirectory $item]} {
            set found [find_named_file $item $filename]
            if {$found ne ""} {
                return $found
            }
        } elseif {[file tail $item] eq $filename} {
            return $item
        }
    }
    return ""
}

proc apply_required_ip_config {ip params} {
    foreach pair $params {
        set key [lindex $pair 0]
        set value [lindex $pair 1]
        set prop CONFIG.$key
        if {[catch {set_property $prop $value $ip} err]} {
            puts stderr "ERROR: failed to set $prop=$value"
            puts stderr "       $err"
            exit 1
        }
    }
}

proc require_ip_config {ip key expected} {
    set prop CONFIG.$key
    if {[catch {set actual [get_property $prop $ip]} err]} {
        puts stderr "ERROR: cannot read $prop for generated PCIe/XDMA IP"
        puts stderr "       $err"
        exit 1
    }
    if {$actual ne $expected} {
        puts stderr "ERROR: PCIe/XDMA config mismatch for $key: expected '$expected', got '$actual'"
        exit 1
    }
}

proc assert_reference_xci_if_present {xci_path} {
    if {![file exists $xci_path]} {
        puts "INFO: pcie_test XDMA reference XCI not found; skipped reference marker comparison:"
        puts "      $xci_path"
        return
    }

    set fh [open $xci_path r]
    set txt [read $fh]
    close $fh

    set needles [list]
    lappend needles {"component_reference": "xilinx.com:ip:xdma:4.1"}
    lappend needles {"ip_revision": "23"}
    lappend needles "\"pl_link_cap_max_link_width\": \[ \{ \"value\": \"X4\""
    lappend needles "\"pl_link_cap_max_link_speed\": \[ \{ \"value\": \"8.0_GT/s\""
    lappend needles "\"axi_data_width\": \[ \{ \"value\": \"128_bit\""
    lappend needles "\"axisten_freq\": \[ \{ \"value\": \"250\""
    lappend needles "\"axist_bypass_en\": \[ \{ \"value\": \"true\""
    lappend needles "\"axi_bypass_64bit_en\": \[ \{ \"value\": \"true\""
    lappend needles "\"select_quad\": \[ \{ \"value\": \"GTY_Quad_224\""
    foreach needle $needles {
        if {[string first $needle $txt] < 0} {
            puts stderr "ERROR: pcie_test XDMA reference XCI does not match the expected host-access markers."
            puts stderr "       Missing marker: $needle"
            puts stderr "       XCI: $xci_path"
            exit 1
        }
    }
}

proc write_manifest {path ip_name part output_dir root_xci root_dcp root_stub mode reference_xci} {
    set fh [open $path w]
    puts $fh "ip=$ip_name"
    puts $fh "part=$part"
    puts $fh "mode=$mode"
    puts $fh "output_dir=$output_dir"
    puts $fh "xci=$root_xci"
    puts $fh "dcp=$root_dcp"
    puts $fh "stub=$root_stub"
    puts $fh "source=pcie_test design_1_xdma_0_0.xci"
    puts $fh "source_xci=$reference_xci"
    puts $fh "source_component=xilinx.com:ip:xdma:4.2 in Vivado 2025.2 (contract from pcie_test xdma:4.1 rev 23 / Vivado 2023.1)"
    puts $fh "pcie_link=Gen3_x4"
    puts $fh "pcie_block=X0Y0"
    puts $fh "gt_quad=GTY_Quad_224"
    puts $fh "pcie_refclk=100_MHz"
    puts $fh "axi_aclk_mhz=250"
    puts $fh "axi_data_width=128"
    puts $fh "axi_id_width=4"
    puts $fh "axi_addr_width=64"
    puts $fh "bypass_enabled=true"
    puts $fh "bypass_addr_width=64"
    puts $fh "bypass_bar_size=4_Megabytes"
    puts $fh "xdma_h2c_channels=4"
    puts $fh "xdma_c2h_channels=4"
    puts $fh "user_irqs=9"
    puts $fh "msi=true"
    puts $fh "msix=true"
    puts $fh "cfg_mgmt_if=true"
    puts $fh "board_constraints=synth/pcie_xdma.xdc and synth/fpga_top_real_mig.xdc"
    close $fh
}

# These are the pcie_test XCI component_parameters values that define the XDMA
# contract used by rtl/fpga_top.v.  Descriptor DMA M_AXI is generated but parked
# in the top-level skeleton; M_AXI_BYPASS is the active host-access path.
set xdma_params {
    {Component_Name design_1_xdma_0_0}
    {functional_mode DMA}
    {mode_selection Advanced}
    {pcie_blk_locn X0Y0}
    {pl_link_cap_max_link_width X4}
    {pl_link_cap_max_link_speed 8.0_GT/s}
    {ref_clk_freq 100_MHz}
    {free_run_freq 100_MHz}
    {axi_data_width 128_bit}
    {axisten_freq 250}
    {en_axi_slave_if true}
    {en_axi_master_if true}
    {dedicate_perst true}
    {sys_reset_polarity ACTIVE_LOW}
    {pf0_device_id 9034}
    {pf0_base_class_menu Processing_accelerators}
    {pf0_class_code_base 12}
    {pf0_class_code_sub 00}
    {pf0_class_code_interface 00}
    {axist_bypass_en true}
    {axist_bypass_size 4}
    {axist_bypass_scale Megabytes}
    {pf0_msi_enabled true}
    {pf0_msi_cap_multimsgcap 1_vector}
    {Shared_Logic 1}
    {xdma_rnum_chnl 4}
    {xdma_wnum_chnl 4}
    {xdma_num_usr_irq 9}
    {en_gt_selection true}
    {select_quad GTY_Quad_224}
    {plltype QPLL1}
    {ext_sys_clk_bufg true}
    {xdma_pcie_64bit_en true}
    {pcie_extended_tag true}
    {pf0_msix_enabled true}
    {pf0_msix_cap_table_size 01F}
    {pf0_msix_cap_table_offset 00008000}
    {pf0_msix_cap_table_bir BAR_1:0}
    {pf0_msix_cap_pba_offset 00008FE0}
    {pf0_msix_cap_pba_bir BAR_1:0}
    {cfg_mgmt_if true}
    {axi_bypass_64bit_en true}
    {axi_id_width 4}
    {enable_jtag_dbg true}
    {pf0_bar0_enabled true}
    {pf0_bar0_size 128}
    {pf0_bar0_scale Kilobytes}
    {pf1_bar0_enabled true}
    {pf1_bar0_size 32}
    {pf1_bar0_scale Megabytes}
    {pf1_bar1_enabled true}
    {pf1_bar1_size 128}
    {pf1_bar1_scale Kilobytes}
    {pf1_bar2_enabled true}
    {pf1_bar2_size 128}
    {pf1_bar2_scale Kilobytes}
    {pf1_bar2_64bit true}
    {pciebar2axibar_axist_bypass 0x0000000000000000}
    {bar_indicator BAR_0}
    {bar0_indicator 1}
}

set required_checks {
    {Component_Name design_1_xdma_0_0}
    {functional_mode DMA}
    {mode_selection Advanced}
    {pcie_blk_locn X0Y0}
    {pl_link_cap_max_link_width X4}
    {pl_link_cap_max_link_speed 8.0_GT/s}
    {ref_clk_freq 100_MHz}
    {axi_data_width 128_bit}
    {axisten_freq 250}
    {en_axi_master_if true}
    {axist_bypass_en true}
    {axist_bypass_size 4}
    {axist_bypass_scale Megabytes}
    {xdma_rnum_chnl 4}
    {xdma_wnum_chnl 4}
    {xdma_num_usr_irq 9}
    {select_quad GTY_Quad_224}
    {plltype QPLL1}
    {ext_sys_clk_bufg true}
    {xdma_pcie_64bit_en true}
    {axi_bypass_64bit_en true}
    {axi_id_width 4}
    {pf0_bar0_size 128}
    {pf0_bar0_scale Kilobytes}
    {pciebar2axibar_axist_bypass 0x0000000000000000}
}

if {[info exists ::env(PCIE_TEST_DIR)]} {
    set pcie_test_dir $::env(PCIE_TEST_DIR)
} elseif {[file exists /offlinenas/share/FPGA/pcie_test]} {
    set pcie_test_dir /offlinenas/share/FPGA/pcie_test
} else {
    set pcie_test_dir [file join $::env(HOME) FPGA pcie_test]
}
set reference_xci [file normalize [file join $pcie_test_dir \
    pcie_test.srcs sources_1 bd design_1 ip design_1_xdma_0_0 design_1_xdma_0_0.xci]]

puts "=== PCIe/XDMA GENERATOR ==="
puts "Mode: $mode"
puts "Part: $part"
puts "IP: xilinx.com:ip:xdma:4.2 / $ip_name"
puts "Output: $output_dir"
puts "Reference XCI: $reference_xci"
puts "Board constraints: synth/pcie_xdma.xdc, synth/fpga_top_real_mig.xdc"

assert_reference_xci_if_present $reference_xci

file mkdir $output_dir
file delete -force $ip_dir
file delete -force $root_xci $root_dcp $root_stub $manifest
file mkdir $ip_root

create_project -in_memory -part $part -force
set_property target_language Verilog [current_project]
set_property default_lib xil_defaultlib [current_project]
set_property XPM_LIBRARIES {XPM_CDC XPM_FIFO XPM_MEMORY} [current_project]

create_ip -name xdma -vendor xilinx.com -library ip -version 4.2 \
    -module_name $ip_name -dir $ip_root
set ip [get_ips $ip_name]
apply_required_ip_config $ip $xdma_params

generate_target {synthesis instantiation_template} $ip
foreach pair $required_checks {
    require_ip_config $ip [lindex $pair 0] [lindex $pair 1]
}

set xci_path [find_named_file $ip_dir ${ip_name}.xci]
if {$xci_path eq ""} {
    puts stderr "ERROR: generated XCI not found under $ip_dir"
    exit 1
}
file copy -force $xci_path $root_xci

if {$mode eq "synth"} {
    puts "=== Synthesising PCIe/XDMA IP out-of-context ==="
    synth_ip $ip
    set dcp_path [find_named_file $ip_dir ${ip_name}.dcp]
    if {$dcp_path eq ""} {
        puts stderr "ERROR: generated DCP not found under $ip_dir"
        exit 1
    }
    file copy -force $dcp_path $root_dcp

    set stub_path [find_named_file $ip_dir ${ip_name}_stub.v]
    if {$stub_path ne ""} {
        file copy -force $stub_path $root_stub
    } else {
        puts "INFO: generated Verilog stub not found; IP XCI/DCP remain the load-bearing artifacts."
    }
}

write_manifest $manifest $ip_name $part $output_dir $root_xci $root_dcp $root_stub $mode $reference_xci

puts "=== PCIe/XDMA generator complete ==="
puts "XCI: $root_xci"
if {$mode eq "synth"} {
    puts "DCP: $root_dcp"
} else {
    puts "DCP: not produced in validate mode"
}
puts "Manifest: $manifest"

close_project
