# gen_ddr4_mig.tcl -- repo-owned DDR4 MIG generator for the KU5P board.
#
# Usage:
#   vivado -mode batch -source synth/gen_ddr4_mig.tcl -tclargs validate build/ddr4_mig
#   vivado -mode batch -source synth/gen_ddr4_mig.tcl -tclargs synth    build/ddr4_mig
#   tclsh  synth/gen_ddr4_mig.tcl cache-check build/ddr4_mig
#
# `validate` creates the configured XCI and generated targets, then checks the
# key pcie_test-derived parameters.  `synth` also runs OOC IP synthesis and
# writes build/ddr4_mig/design_1_ddr4_0_1.dcp for synth/vivado.tcl to stitch.
# `cache-check` is a fast filesystem+manifest validity check intended for
# Makefile cache-hit gating before launching Vivado.
#
# Source of truth:
#   /offlinenas/share/FPGA/pcie_test/pcie_test.srcs/sources_1/bd/design_1/ip/
#       design_1_ddr4_0_1/design_1_ddr4_0_1.xci
#
# Board constraints are intentionally not generated here.  The top-level flow
# owns board pin placement in synth/ddr4.xdc and the separate AB7/AB6 fabric
# clock in synth/fpga_top_real_mig.xdc.

set mode [lindex $argv 0]
set output_dir [lindex $argv 1]

if {$mode eq ""} {
    set mode "synth"
}

set script_dir [file normalize [file dirname [info script]]]
set proj_root  [file normalize [file join $script_dir ..]]
if {$output_dir eq ""} {
    set output_dir [file normalize [file join $proj_root build ddr4_mig]]
} else {
    set output_dir [file normalize $output_dir]
}

if {$mode ne "validate" && $mode ne "synth" && $mode ne "cache-check"} {
    puts stderr "ERROR: mode must be one of {validate, synth, cache-check}; got: $mode"
    exit 1
}

set part        "xcku5p-ffvb676-2-i"
set ip_name     "design_1_ddr4_0_1"
set ip_root     [file join $output_dir ip]
set ip_dir      [file join $ip_root $ip_name]
set root_xci    [file join $output_dir ${ip_name}.xci]
set root_dcp    [file join $output_dir ${ip_name}.dcp]
set root_stub   [file join $output_dir ${ip_name}_stub.v]
set manifest    [file join $output_dir ${ip_name}_manifest.txt]

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
    # Apply atomically via set_property -dict so the IP customize hook can
    # reconcile dependent params in one pass.  Sequential set_property calls
    # are NOT equivalent: under Vivado 2025.2 the DDR4 IP silently resets
    # `C0.DDR4_AxiSelection` back to false when `No_Controller=1` is later
    # applied via individual set_property calls — yielding a native-PHY IP
    # whose stub does not match the AXI port set the design wires up.
    set dict_args [list]
    foreach pair $params {
        lappend dict_args CONFIG.[lindex $pair 0] [lindex $pair 1]
    }
    if {[catch {set_property -dict $dict_args $ip} err]} {
        puts stderr "ERROR: failed to apply DDR4 IP config dict"
        puts stderr "       $err"
        exit 1
    }
}

proc require_ip_config {ip key expected} {
    set prop CONFIG.$key
    if {[catch {set actual [get_property $prop $ip]} err]} {
        puts stderr "ERROR: cannot read $prop for generated DDR4 MIG IP"
        puts stderr "       $err"
        exit 1
    }
    if {$actual ne $expected} {
        puts stderr "ERROR: DDR4 MIG config mismatch for $key: expected '$expected', got '$actual'"
        exit 1
    }
}

proc hash32_hex {txt} {
    # FNV-1a 32-bit hash in pure Tcl so Vivado Tcl and tclsh produce identical keys.
    set hash 2166136261
    binary scan $txt c* bytes
    foreach byte $bytes {
        set hash [expr {($hash ^ ($byte & 0xff)) & 0xffffffff}]
        set hash [expr {($hash * 16777619) & 0xffffffff}]
    }
    return [format "%08x" $hash]
}

proc file_hash32_hex {path} {
    if {![file exists $path]} {
        return ""
    }
    set fh [open $path r]
    fconfigure $fh -translation binary
    set txt [read $fh]
    close $fh
    return [hash32_hex $txt]
}

proc calc_cache_key {part ip_name ddr4_params required_checks script_hash} {
    set payload [list]
    lappend payload "part=$part"
    lappend payload "ip=$ip_name"
    lappend payload "script_hash=$script_hash"
    foreach pair $ddr4_params {
        lappend payload "[lindex $pair 0]=[lindex $pair 1]"
    }
    lappend payload "__required_checks__"
    foreach pair $required_checks {
        lappend payload "[lindex $pair 0]=[lindex $pair 1]"
    }
    return [hash32_hex [join $payload "\n"]]
}

proc manifest_pairs {ip_name part output_dir root_xci root_dcp mode cache_key script_hash} {
    set pairs [list]
    lappend pairs [list ip $ip_name]
    lappend pairs [list part $part]
    lappend pairs [list mode $mode]
    lappend pairs [list output_dir $output_dir]
    lappend pairs [list xci $root_xci]
    lappend pairs [list dcp $root_dcp]
    lappend pairs [list source "pcie_test design_1_ddr4_0_1.xci"]
    lappend pairs [list source_component "xilinx.com:ip:ddr4:2.2 revision 28 in Vivado 2025.2"]
    lappend pairs [list memory_part "MT40A512M16LY-075"]
    lappend pairs [list memory_period_ps "750"]
    lappend pairs [list input_clock_period_ps "5000"]
    lappend pairs [list phy_clock_ratio "4:1"]
    lappend pairs [list ddr4_data_width "32"]
    lappend pairs [list ddr4_data_mask "DM_NO_DBI"]
    lappend pairs [list ddr4_parity "false"]
    lappend pairs [list axi_data_width "256"]
    lappend pairs [list axi_addr_width "31"]
    lappend pairs [list axi_id_width "1"]
    lappend pairs [list ui_clock_hz "333250000"]
    lappend pairs [list board_constraints "synth/ddr4.xdc and synth/fpga_top_real_mig.xdc"]
    lappend pairs [list cache_key $cache_key]
    lappend pairs [list script_hash32 $script_hash]
    return $pairs
}

proc write_manifest {path pairs} {
    set fh [open $path w]
    foreach pair $pairs {
        puts $fh "[lindex $pair 0]=[lindex $pair 1]"
    }
    close $fh
}

proc read_manifest_dict {path} {
    set dict_out [dict create]
    if {![file exists $path]} {
        return $dict_out
    }
    set fh [open $path r]
    while {[gets $fh line] >= 0} {
        set split_idx [string first "=" $line]
        if {$split_idx <= 0} {
            continue
        }
        set key [string range $line 0 [expr {$split_idx - 1}]]
        set value [string range $line [expr {$split_idx + 1}] end]
        dict set dict_out $key $value
    }
    close $fh
    return $dict_out
}

proc cache_check {manifest root_xci root_dcp expected_pairs} {
    if {![file exists $manifest]} {
        return [list 0 "missing manifest ($manifest)"]
    }
    if {![file exists $root_xci]} {
        return [list 0 "missing XCI ($root_xci)"]
    }
    if {[file size $root_xci] <= 0} {
        return [list 0 "empty XCI ($root_xci)"]
    }
    if {![file exists $root_dcp]} {
        return [list 0 "missing DCP ($root_dcp)"]
    }
    if {[file size $root_dcp] <= 0} {
        return [list 0 "empty DCP ($root_dcp)"]
    }

    set manifest_dict [read_manifest_dict $manifest]
    foreach pair $expected_pairs {
        set key [lindex $pair 0]
        set expected [lindex $pair 1]
        if {![dict exists $manifest_dict $key]} {
            return [list 0 "manifest key '$key' missing"]
        }
        set actual [dict get $manifest_dict $key]
        if {$actual ne $expected} {
            return [list 0 "manifest mismatch for '$key': expected '$expected', got '$actual'"]
        }
    }

    return [list 1 "manifest and MIG artifacts match expected contract"]
}

# These are the pcie_test XCI component_parameters values that define the MIG
# contract.  Generated/model-only values are deliberately omitted; Vivado
# derives them from this set for the installed DDR4 IP revision.
set ddr4_params {
    {C0.ControllerType DDR4_SDRAM}
    {IOPowerReduction OFF}
    {Enable_SysPorts true}
    {Phy_Only Complete_Memory_Controller}
    {RESET_BOARD_INTERFACE Custom}
    {C0_CLOCK_BOARD_INTERFACE Custom}
    {C0_DDR4_MEMORY_MAP_BASEADDR 0x00000000}
    {C0_DDR4_MEMORY_MAP_HIGHADDR 0xFFFFFFFF}
    {C0_DDR4_MEMORY_MAP_CTRL_BASEADDR 0x00000000}
    {C0_DDR4_MEMORY_MAP_CTRL_HIGHADDR 0xFFFFFFFF}
    {IS_FROM_PHY 1}
    {RECONFIG_XSDB_SAVE_RESTORE false}
    {AL_SEL 0}
    {Example_TG SIMPLE_TG}
    {C0.DDR4_Clamshell false}
    {C0.MIGRATION false}
    {TIMING_OP1 false}
    {TIMING_OP2 false}
    {TIMING_3DS false}
    {SET_DW_TO_40 false}
    {DIFF_TERM_SYSCLK false}
    {C0_DDR4_BOARD_INTERFACE Custom}
    {C0.DDR4_TimePeriod 750}
    {C0.DDR4_InputClockPeriod 5000}
    {C0.DDR4_Specify_MandD false}
    {C0.DDR4_CLKFBOUT_MULT 5}
    {C0.DDR4_DIVCLK_DIVIDE 1}
    {C0.DDR4_CLKOUT0_DIVIDE 3}
    {C0.DDR4_PhyClockRatio 4:1}
    {C0.DDR4_MemoryType Components}
    {C0.DDR4_MemoryPart MT40A512M16LY-075}
    {C0.DDR4_Slot Single}
    {C0.DDR4_MemoryVoltage 1.2V}
    {C0.DDR4_DataWidth 32}
    {C0.DDR4_DataMask DM_NO_DBI}
    {C0.DDR4_Ecc false}
    {C0.DDR4_MCS_ECC false}
    {C0.DDR4_AxiSelection true}
    {C0.DDR4_AUTO_AP_COL_A3 false}
    {C0.DDR4_Ordering Normal}
    {C0.DDR4_BurstLength 8}
    {C0.DDR4_BurstType Sequential}
    {C0.DDR4_OutputDriverImpedenceControl RZQ/7}
    {C0.DDR4_OnDieTermination RZQ/6}
    {C0.DDR4_CasLatency 19}
    {C0.DDR4_CasWriteLatency 14}
    {C0.DDR4_ChipSelect true}
    {C0.DDR4_isCKEShared false}
    {C0.DDR4_AxiDataWidth 256}
    {C0.DDR4_AxiArbitrationScheme RD_PRI_REG}
    {C0.DDR4_AxiNarrowBurst false}
    {C0.DDR4_AxiAddressWidth 31}
    {C0.DDR4_AxiIDWidth 1}
    {C0.DDR4_Capacity 512}
    {C0.DDR4_Mem_Add_Map ROW_COLUMN_BANK}
    {C0.DDR4_MemoryName MainMemory}
    {C0.DDR4_AutoPrecharge false}
    {C0.DDR4_UserRefresh_ZQCS false}
    {C0.DDR4_CustomParts no_file_loaded}
    {C0.DDR4_isCustom false}
    {C0.DDR4_SELF_REFRESH false}
    {C0.DDR4_SAVE_RESTORE false}
    {C0.DDR4_RESTORE_CRC false}
    {ADDN_UI_CLKOUT1_FREQ_HZ None}
    {ADDN_UI_CLKOUT2_FREQ_HZ None}
    {ADDN_UI_CLKOUT3_FREQ_HZ None}
    {ADDN_UI_CLKOUT4_FREQ_HZ None}
    {CLKOUT6 false}
    {Component_Name design_1_ddr4_0_1}
    {No_Controller 1}
    {System_Clock Differential}
    {Reference_Clock Differential}
    {Debug_Signal Disable}
    {IO_Power_Reduction false}
    {DCI_Cascade false}
    {Default_Bank_Selections false}
    {Simulation_Mode BFM}
    {PARTIAL_RECONFIG_FLOW_MIG false}
    {MCS_DBG_EN false}
    {C0.DDR4_CK_SKEW_0 0}
    {C0.DDR4_CK_SKEW_1 0}
    {C0.DDR4_CK_SKEW_2 0}
    {C0.DDR4_CK_SKEW_3 0}
    {C0.DDR4_ADDR_SKEW_0 0}
    {C0.DDR4_ADDR_SKEW_1 0}
    {C0.DDR4_ADDR_SKEW_2 0}
    {C0.DDR4_ADDR_SKEW_3 0}
    {C0.DDR4_ADDR_SKEW_4 0}
    {C0.DDR4_ADDR_SKEW_5 0}
    {C0.DDR4_ADDR_SKEW_6 0}
    {C0.DDR4_ADDR_SKEW_7 0}
    {C0.DDR4_ADDR_SKEW_8 0}
    {C0.DDR4_ADDR_SKEW_9 0}
    {C0.DDR4_ADDR_SKEW_10 0}
    {C0.DDR4_ADDR_SKEW_11 0}
    {C0.DDR4_ADDR_SKEW_12 0}
    {C0.DDR4_ADDR_SKEW_13 0}
    {C0.DDR4_ADDR_SKEW_14 0}
    {C0.DDR4_ADDR_SKEW_15 0}
    {C0.DDR4_ADDR_SKEW_16 0}
    {C0.DDR4_ADDR_SKEW_17 0}
    {C0.DDR4_BA_SKEW_0 0}
    {C0.DDR4_BA_SKEW_1 0}
    {C0.DDR4_BG_SKEW_0 0}
    {C0.DDR4_BG_SKEW_1 0}
    {C0.DDR4_CS_SKEW_0 0}
    {C0.DDR4_CS_SKEW_1 0}
    {C0.DDR4_CS_SKEW_2 0}
    {C0.DDR4_CS_SKEW_3 0}
    {C0.DDR4_CKE_SKEW_0 0}
    {C0.DDR4_CKE_SKEW_1 0}
    {C0.DDR4_CKE_SKEW_2 0}
    {C0.DDR4_CKE_SKEW_3 0}
    {C0.DDR4_ACT_SKEW 0}
    {C0.DDR4_PAR_SKEW 0}
    {C0.DDR4_ODT_SKEW_0 0}
    {C0.DDR4_ODT_SKEW_1 0}
    {C0.DDR4_ODT_SKEW_2 0}
    {C0.DDR4_ODT_SKEW_3 0}
    {C0.DDR4_LR_SKEW_0 0}
    {C0.DDR4_LR_SKEW_1 0}
    {C0.DDR4_TREFI 0}
    {C0.DDR4_TRFC 0}
    {C0.DDR4_TRFC_DLR 0}
    {C0.DDR4_TXPR 0}
    {C0.DDR4_nCK_TREFI 0}
    {C0.DDR4_nCK_TRFC 0}
    {C0.DDR4_nCK_TRFC_DLR 0}
    {C0.DDR4_nCK_TXPR 5}
    {C0.ADDR_WIDTH 17}
    {C0.BANK_GROUP_WIDTH 1}
    {C0.LR_WIDTH 1}
    {C0.CK_WIDTH 1}
    {C0.CKE_WIDTH 1}
    {C0.CS_WIDTH 1}
    {C0.ODT_WIDTH 1}
    {C0.StackHeight 1}
    {PING_PONG_PHY 1}
    {C0.DDR4_Enable_LVAUX false}
    {C0.DDR4_EN_PARITY false}
    {EN_PP_4R_MIR false}
    {MCS_WO_DSP false}
}

set required_checks {
    {Component_Name design_1_ddr4_0_1}
    {C0.DDR4_MemoryPart MT40A512M16LY-075}
    {C0.DDR4_DataWidth 32}
    {C0.DDR4_DataMask DM_NO_DBI}
    {C0.DDR4_EN_PARITY false}
    {C0.DDR4_TimePeriod 750}
    {C0.DDR4_InputClockPeriod 5000}
    {C0.DDR4_PhyClockRatio 4:1}
    {C0.DDR4_AxiSelection true}
    {C0.DDR4_AxiDataWidth 256}
    {C0.DDR4_AxiAddressWidth 31}
    {C0.DDR4_AxiIDWidth 1}
    {C0.ADDR_WIDTH 17}
    {C0.BANK_GROUP_WIDTH 1}
    {C0.CK_WIDTH 1}
    {C0.CKE_WIDTH 1}
    {C0.CS_WIDTH 1}
    {C0.ODT_WIDTH 1}
}

set script_hash [file_hash32_hex [info script]]
if {$script_hash eq ""} {
    puts stderr "ERROR: failed to compute script hash for [info script]"
    exit 1
}
set cache_key [calc_cache_key $part $ip_name $ddr4_params $required_checks $script_hash]
set expected_synth_manifest [manifest_pairs $ip_name $part $output_dir $root_xci $root_dcp "synth" $cache_key $script_hash]

if {$mode eq "cache-check"} {
    lassign [cache_check $manifest $root_xci $root_dcp $expected_synth_manifest] cache_ok cache_msg
    if {$cache_ok} {
        puts "CACHE_HIT: $cache_msg"
        exit 0
    }
    puts "CACHE_MISS: $cache_msg"
    exit 2
}

puts "=== DDR4 MIG GENERATOR ==="
puts "Mode: $mode"
puts "Part: $part"
puts "IP: xilinx.com:ip:ddr4:2.2 / $ip_name"
puts "Output: $output_dir"
puts "Board constraints: synth/ddr4.xdc, synth/fpga_top_real_mig.xdc"

set_param general.maxThreads 16

file mkdir $output_dir
file delete -force $ip_dir
file delete -force $root_xci $root_dcp $root_stub $manifest
file mkdir $ip_root

create_project -in_memory -part $part -force
set_property target_language Verilog [current_project]
set_property default_lib xil_defaultlib [current_project]
set_property XPM_LIBRARIES {XPM_CDC XPM_FIFO XPM_MEMORY} [current_project]

create_ip -name ddr4 -vendor xilinx.com -library ip -version 2.2 \
    -module_name $ip_name -dir $ip_root
set ip [get_ips $ip_name]
apply_required_ip_config $ip $ddr4_params

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
    puts "=== Synthesising DDR4 MIG IP out-of-context ==="
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
        puts "INFO: generated Verilog stub not found; rtl/vendor/design_1_ddr4_0_1_stub.v remains the checked-in stub."
    }
}

write_manifest $manifest [manifest_pairs $ip_name $part $output_dir $root_xci $root_dcp $mode $cache_key $script_hash]

puts "=== DDR4 MIG generator complete ==="
puts "XCI: $root_xci"
if {$mode eq "synth"} {
    puts "DCP: $root_dcp"
} else {
    puts "DCP: not produced in validate mode"
}
puts "Manifest: $manifest"

close_project
