# Vitis HLS flow for VoxLIO v0.1: C simulation + C synthesis (phases 6 and 7).
#
#   vitis_hls -f hls/run_hls.tcl                      (Vitis HLS up to 2023.x)
#   vitis-run --mode hls --tcl hls/run_hls.tcl        (Vitis 2024.x and later)
#
# Run from the repository root, after `make vectors`. Environment overrides:
#   VOXLIO_PART     FPGA part            (default xc7z020clg400-1, Zynq-7020)
#   VOXLIO_CLOCK    clock period in ns   (default 10)
#   VOXLIO_NUMERIC  float | fixed        (default float)
#   VOXLIO_COSIM    1 to also run C/RTL co-simulation (slow)
#
# Baseline settings only: no optimisation directives beyond the interface
# pragmas in voxlio_core.cpp. Every setting used is echoed into
# results/synthesis/<solution>/config.txt next to the reports (the tool
# version is recorded inside the reports themselves).
#
# NOTE: written without access to the Xilinx tools; not yet run.

proc env_or {name default} {
    if {[info exists ::env($name)]} { return $::env($name) }
    return $default
}

set root    [pwd]
set part    [env_or VOXLIO_PART xc7z020clg400-1]
set clock   [env_or VOXLIO_CLOCK 10]
set numeric [env_or VOXLIO_NUMERIC float]
set cosim   [env_or VOXLIO_COSIM 0]

set cases {room room_face_aligned room_seed_b room_seed_c}
set solution "${numeric}_baseline"
set out_dir  "$root/results/synthesis/$solution"
set sim_dir  "$out_dir/csim"

set cflags "-std=c++14 -I$root/hls"
if {$numeric eq "fixed"} {
    append cflags " -DVOXLIO_FIXED_POINT"
}
set tb_cflags "$cflags -I$root/reference/cpp -I$root/testbench -Wno-unknown-pragmas"

open_project -reset build/hls_$numeric
set_top voxlio_core

foreach src {transform voxel_index voxel_lookup geometry accumulator voxlio_core} {
    add_files hls/$src.cpp -cflags $cflags
}
add_files -tb testbench/tb_voxlio.cpp -cflags $tb_cflags
add_files -tb reference/cpp/voxlio_ref.cpp -cflags $tb_cflags

open_solution -reset $solution -flow_target vivado
set_part $part
create_clock -period $clock -name default

# The testbench writes one result file per case and reads the test vectors.
set argv_list [list $sim_dir]
foreach case $cases {
    file mkdir $sim_dir/$case
    lappend argv_list $root/data/synthetic/$case
}

file mkdir $out_dir
set cfg [open $out_dir/config.txt w]
puts $cfg "date        [clock format [clock seconds] -format {%Y-%m-%d %H:%M:%S}]"
puts $cfg "top         voxlio_core"
puts $cfg "part        $part"
puts $cfg "clock_ns    $clock"
puts $cfg "numeric     $numeric"
puts $cfg "cflags      $cflags"
puts $cfg "flow_target vivado"
puts $cfg "directives  none (interface pragmas in hls/voxlio_core.cpp only)"
close $cfg

csim_design -argv [join $argv_list " "]

csynth_design

if {$cosim} {
    cosim_design -argv [join $argv_list " "]
}

# Keep the reports with the results.
set report_dir build/hls_$numeric/$solution/syn/report
foreach report [glob -nocomplain $report_dir/*csynth.rpt $report_dir/*csynth.xml] {
    file copy -force $report $out_dir/
}

exit
