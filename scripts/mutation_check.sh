#!/bin/bash
# Mutation check: plant one deliberate bug at a time in a scratch copy of the
# sources and confirm that a test fails. A mutation that survives marks a gap
# in the tests. Run from anywhere; needs the test vectors (`make vectors`) and,
# for the fixed-point mutation, the ap_fixed headers (`make fixed` once).
#
# Exit status: 0 if the unmutated control passes and every mutation is caught.

REPO=$(cd "$(dirname "$0")/.." && pwd)
WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

SRC="hls/transform.cpp hls/voxel_index.cpp hls/voxel_lookup.cpp hls/geometry.cpp \
     hls/accumulator.cpp hls/voxlio_core.cpp reference/cpp/voxlio_ref.cpp"
FLAGS="-std=c++14 -O1 -w -ffp-contract=off -fsanitize=address -Ihls -Ireference/cpp -Itestbench"
FIXED="-DVOXLIO_FIXED_POINT -I${AP_TYPES_INC:-$REPO/third_party/ap_types/include}"
survivors=0

fresh_copy() {
    rm -rf "$WORK/src"
    mkdir -p "$WORK/src"
    cp -r "$REPO/hls" "$REPO/reference" "$REPO/testbench" "$REPO/tests" "$REPO/conftest.py" "$WORK/src/"
    ln -s "$REPO/data" "$WORK/src/data"
}

# run_test MODE TEST -> exit status of the test in the scratch copy
run_test() {
    local mode="$1" test="$2" flags="$FLAGS"
    [ "$mode" = fixed ] && flags="$flags $FIXED"
    cd "$WORK/src" || return 99
    if [ "$test" = pytest ]; then
        python3 -m pytest tests -q -x >/dev/null 2>&1
    else
        g++ $flags testbench/$test.cpp $SRC -o $test 2>/dev/null || return 98
        mkdir -p out/room
        ./$test out data/synthetic/room >/dev/null 2>&1
    fi
}

control() {
    fresh_copy
    if run_test "$1" "$2"; then
        echo "control ok : $2 ($1)"
    else
        echo "CONTROL FAILED: $2 ($1); results below are meaningless"
        survivors=$((survivors + 1))
    fi
}

# mutate NAME FILE SED_EXPRESSION MODE TEST
mutate() {
    local name="$1" file="$2" expr="$3" mode="$4" test="$5"
    fresh_copy
    sed -i "$expr" "$WORK/src/$file"
    if cmp -s "$WORK/src/$file" "$REPO/$file"; then
        echo "NOT APPLIED: $name (pattern no longer matches $file)"
        survivors=$((survivors + 1))
        return
    fi
    run_test "$mode" "$test"
    local rc=$?
    if [ $rc -eq 98 ]; then
        echo "NO BUILD   : $name (mutant does not compile)"
        survivors=$((survivors + 1))
    elif [ $rc -ne 0 ]; then
        echo "caught     : $name ($test, $mode)"
    else
        echo "SURVIVED   : $name ($test, $mode)"
        survivors=$((survivors + 1))
    fi
}

for t in tb_transform tb_voxel_index tb_geometry tb_accumulator tb_voxlio pytest; do
    control float $t
done
control fixed tb_voxlio

mutate "neighbour bounds check removed"      hls/voxel_lookup.cpp 's/if (!in_grid) {/if (false) {/' float tb_voxlio
mutate "tie-break < changed to <="           hls/voxel_lookup.cpp 's/if (abs_r < best_abs) {/if (abs_r <= best_abs) {/' float tb_voxlio
mutate "Jacobian sign (unit)"                hls/geometry.cpp 's/J\[0\] = p.y \* v.nz - p.z \* v.ny;/J[0] = p.y * v.nz + p.z * v.ny;/' float tb_geometry
mutate "Jacobian sign (top level)"           hls/geometry.cpp 's/J\[1\] = p.z \* v.nx - p.x \* v.nz;/J[1] = p.x * v.nz - p.z * v.nx;/' float tb_voxlio
mutate "threshold < changed to <="           hls/voxlio_core.cpp 's/if (!(vox_abs(best_r) < threshold)) {/if (!(vox_abs(best_r) <= threshold)) {/' float tb_voxlio
mutate "upper grid bound inclusive"          hls/voxel_index.cpp 's/f >= 0 \&\& f < n/f >= 0 \&\& f <= n/' float tb_voxel_index
mutate "lower grid bound dropped"            hls/voxel_index.cpp 's/f >= 0 \&\& f < n/f < n/' float tb_voxel_index
mutate "H packing table entry"               hls/accumulator.cpp 's/TRI_ROW\[21\] = {0, 0, 1, 0, 1, 2,/TRI_ROW[21] = {0, 0, 1, 0, 2, 2,/' float tb_accumulator
mutate "zero-normal check removed"           hls/geometry.cpp 's/return v.valid != 0 \&\& !zero_normal;/return v.valid != 0;/' float tb_voxlio
mutate "valid flag ignored"                  hls/geometry.cpp 's/return v.valid != 0 \&\& !zero_normal;/return !zero_normal;/' float tb_voxlio
mutate "point-count overflow not checked"    hls/voxlio_core.cpp 's/if (num_points > c_max_points) {/if (false) {/' float tb_voxlio
mutate "pose finiteness not checked"         hls/voxlio_core.cpp 's/if (!pose_is_finite(pose)) {/if (false) {/' float tb_voxlio
mutate "translation dropped from transform"  hls/transform.cpp 's/ + pose.t\[1\];/;/' float tb_transform
mutate "g accumulated with wrong sign"       hls/accumulator.cpp 's/result.g\[i\] += J\[i\] \* residual;/result.g[i] -= J[i] * residual;/' float tb_voxlio
mutate "coord_t wraps instead of saturating" hls/voxlio_types.hpp 's/VOXLIO_COORD_I, VOXLIO_QUANT, AP_SAT> coord_t;/VOXLIO_COORD_I, VOXLIO_QUANT, AP_WRAP> coord_t;/' fixed tb_voxlio
mutate "C++ reference: g sign"               reference/cpp/voxlio_ref.cpp 's/res.g\[i\] += J\[i\] \* best_r;/res.g[i] -= J[i] * best_r;/' float tb_voxlio
mutate "python: neighbour bounds removed"    reference/voxlio_reference.py 's/if not (0 <= cx < cfg.NX and 0 <= cy < cfg.NY and 0 <= cz < cfg.NZ):/if False:/' float pytest
mutate "python: tie-break"                   reference/voxlio_reference.py 's/if abs(r) < best_abs:/if abs(r) <= best_abs:/' float pytest
mutate "python: threshold"                   reference/voxlio_reference.py 's/if not abs(best_r) < threshold:/if not abs(best_r) <= threshold:/' float pytest
mutate "python: Jacobian"                    reference/geometry.py 's/p\[2\] \* n\[0\] - p\[0\] \* n\[2\],/p[0] * n[2] - p[2] * n[0],/' float pytest
mutate "python: map index axes swapped"      reference/geometry.py 's/return (iz \* cfg.NY + iy) \* cfg.NX + ix/return (ix * cfg.NY + iy) * cfg.NX + iz/' float pytest

# ---- RTL mutations: the Verilator testbench must notice each one --------------
RTL_FILES="voxlio_pkg.sv voxlio_round_sat.sv voxlio_ram.sv voxlio_transform.sv voxlio_grid.sv \
           voxlio_search.sv voxlio_jacobian.sv voxlio_accumulate.sv voxlio_core.sv"

# mutate_rtl NAME FILE SED_EXPRESSION
mutate_rtl() {
    local name="$1" file="$2" expr="$3"
    local d="$WORK/rtl"
    rm -rf "$d"
    mkdir -p "$d/out/room"
    cp -r "$REPO/rtl" "$d/src"
    sed -i "$expr" "$d/src/$file"
    if cmp -s "$d/src/$file" "$REPO/rtl/$file"; then
        echo "NOT APPLIED: $name (pattern no longer matches rtl/$file)"
        survivors=$((survivors + 1))
        return
    fi
    local srcs=""
    for f in $RTL_FILES; do srcs="$srcs $d/src/$f"; done
    if ! verilator --cc --exe --build -j 4 -O1 --assert --top-module voxlio_core -Mdir "$d/build" \
         -CFLAGS "-std=c++17 -O1 -w -DVOXLIO_FIXED_POINT -I$REPO/hls -I$REPO/reference/cpp -I$REPO/testbench -isystem ${AP_TYPES_INC:-$REPO/third_party/ap_types/include}" \
         $srcs "$REPO/testbench/tb_rtl.cpp" "$REPO"/hls/*.cpp "$REPO/reference/cpp/voxlio_ref.cpp" >/dev/null 2>&1; then
        echo "NO BUILD   : $name (mutant does not build)"
        survivors=$((survivors + 1))
        return
    fi
    if (cd "$REPO" && "$d/build/Vvoxlio_core" "$d/out" data/synthetic/room >/dev/null 2>&1); then
        echo "SURVIVED   : $name (tb_rtl)"
        survivors=$((survivors + 1))
    else
        echo "caught     : $name (tb_rtl)"
    fi
}

if command -v verilator >/dev/null; then
    mutate_rtl "rtl: transform truncates instead of rounding" voxlio_transform.sv 's/\.ROUND(ROUND_TO_NEAREST)/.ROUND(0)/'
    mutate_rtl "rtl: tie-break <= in search"                  voxlio_search.sv 's/(f_abs < best_abs)/(f_abs <= best_abs)/'
    mutate_rtl "rtl: search ignores grid bounds"              voxlio_search.sv 's/    in_grid = (cx >= 0)/    in_grid = 1; if (0) in_grid = (cx >= 0)/'
    mutate_rtl "rtl: grid upper bound inclusive"              voxlio_grid.sv 's/(f\[a\] < LIMIT_RAW\[a\])/(f[a] <= LIMIT_RAW[a])/'
    mutate_rtl "rtl: Jacobian sign"                           voxlio_jacobian.sv "s/diff\[i\] <= DIFF_W'(prod\[2\*i\]) - DIFF_W'(prod\[2\*i+1\]);/diff[i] <= DIFF_W'(prod[2*i+1]) - DIFF_W'(prod[2*i]);/"
    mutate_rtl "rtl: accumulator packing"                     voxlio_accumulate.sv 's/pjj\[row + col \* (col + 1) \/ 2\] <= j\[row\] \* j\[col\];/pjj[row + col * (col + 1) \/ 2] <= j[col] * j[col];/'
    mutate_rtl "rtl: threshold not strict"                    voxlio_core.sv 's/(sat_abs(best_r) < THRESHOLD_RAW)/(sat_abs(best_r) <= THRESHOLD_RAW)/'
    mutate_rtl "rtl: zero-normal check dropped"               voxlio_search.sv 's/f_usable <= e_in_grid \&\& e_flag \&\& (e_n\[0\] != 0 || e_n\[1\] != 0 || e_n\[2\] != 0);/f_usable <= e_in_grid \&\& e_flag;/'
    mutate_rtl "rtl: saturation disabled"                     voxlio_round_sat.sv 's/if (!overflow)          out = shifted\[OUT_W-1:0\];/if (1)          out = shifted[OUT_W-1:0];/'
else
    echo "verilator not found: RTL mutations skipped"
fi

echo
if [ $survivors -eq 0 ]; then
    echo "all mutations caught"
else
    echo "$survivors problem(s): see lines above"
fi
exit $((survivors > 0))
