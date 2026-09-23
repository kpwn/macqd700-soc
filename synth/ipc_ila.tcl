# Compact retirement/dispatch capture, independent of legacy fault-debug ILA.
proc gen_ipc_ila_ip {output_dir part} {
    set ip_dir $output_dir/ip
    file mkdir $ip_dir
    create_project -in_memory -part $part -force
    create_ip -name ila -vendor xilinx.com -library ip -version 6.2 \
        -module_name ipc_ila -dir $ip_dir
    set_property -dict [list CONFIG.C_NUM_OF_PROBES {4} \
        CONFIG.C_DATA_DEPTH {1024} CONFIG.C_INPUT_PIPE_STAGES {1} \
        CONFIG.C_TRIGOUT_EN {false} CONFIG.C_TRIGIN_EN {false} \
        CONFIG.C_ADV_TRIGGER {false} CONFIG.C_EN_STRG_QUAL {0} \
        CONFIG.ALL_PROBE_SAME_MU {true} CONFIG.ALL_PROBE_SAME_MU_CNT {1} \
        CONFIG.C_PROBE0_WIDTH {33} CONFIG.C_PROBE1_WIDTH {33} \
        CONFIG.C_PROBE2_WIDTH {23} CONFIG.C_PROBE3_WIDTH {6}] [get_ips ipc_ila]
    generate_target {synthesis} [get_ips ipc_ila]
    synth_ip [get_ips ipc_ila]
    close_project
    return $ip_dir/ipc_ila/ipc_ila.xci
}
