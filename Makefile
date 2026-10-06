# VoxLIO v0.1 verification flow. `make test` runs every level that does not
# need the Xilinx tools; `make hls` runs C simulation and synthesis in Vitis HLS.

CXX      ?= g++
PYTHON   ?= python3
BUILD    := build
VERIF    := results/verification

# -ffp-contract=off: no fused multiply-add, so float results are the same
# sequence of IEEE single operations as the Python golden model.
CXXFLAGS := -std=c++14 -O2 -g -Wall -Wextra -Wno-unknown-pragmas -Wno-unused-label -ffp-contract=off \
            -fsanitize=address,undefined -fno-sanitize-recover=undefined
INCLUDES := -Ihls -Ireference/cpp -Itestbench

HLS_SRC  := hls/transform.cpp hls/voxel_index.cpp hls/voxel_lookup.cpp \
            hls/geometry.cpp hls/accumulator.cpp hls/voxlio_core.cpp
REF_SRC  := reference/cpp/voxlio_ref.cpp
HEADERS  := $(wildcard hls/*.hpp reference/cpp/*.hpp testbench/*.hpp) Makefile
TESTS    := tb_transform tb_voxel_index tb_geometry tb_accumulator tb_voxlio

CASES     := room room_face_aligned room_seed_b room_seed_c
CASE_DIRS := $(addprefix data/synthetic/,$(CASES))
VECTORS   := $(addsuffix /expected_f32.txt,$(CASE_DIRS))

# ap_fixed headers for the fixed-point build. With Vitis HLS installed, point
# this at its include directory instead (e.g. $(XILINX_HLS)/include).
AP_TYPES_DIR := third_party/ap_types
AP_TYPES_INC ?= $(AP_TYPES_DIR)/include
AP_TYPES_REV := 200a9aecaadf471592558540dc5a88256cbf880f
# The ap_fixed simulation headers shift negative values and read their own
# uninitialised storage in default constructors; both are harmless there.
FIXED_FLAGS  := -DVOXLIO_FIXED_POINT -isystem $(AP_TYPES_INC) -fno-sanitize=shift \
                -Wno-maybe-uninitialized -Wno-uninitialized $(FIXED_DEFS)

.PHONY: all check test pytest vectors float fixed rtl rtl-lint quartus sweep bench mutation hls clean

all: test

# Verify every tool `make test` needs is present and new enough.
check:
	@ok=1; \
	for t in $(CXX) make git $(PYTHON); do \
	  if command -v $$t >/dev/null; then echo "ok       $$t ($$($$t --version 2>&1 | head -1))"; \
	  else echo "MISSING  $$t"; ok=0; fi; done; \
	if $(PYTHON) -c "import numpy, pytest" 2>/dev/null; then echo "ok       python numpy + pytest"; \
	else echo "MISSING  python packages: pip install -r requirements.txt"; ok=0; fi; \
	if command -v verilator >/dev/null; then v=$$(verilator --version | awk '{print $$2}'); \
	  case $$v in 5.*) echo "ok       verilator $$v";; *) echo "TOO OLD  verilator $$v (need 5.x; Ubuntu 24.04 or oss-cad-suite)"; ok=0;; esac; \
	else echo "MISSING  verilator (only needed for make rtl)"; fi; \
	if command -v iverilog >/dev/null; then echo "ok       iverilog (optional)"; else echo "absent   iverilog (optional)"; fi; \
	if [ -f $(AP_TYPES_INC)/ap_fixed.h ]; then echo "ok       ap_fixed headers"; \
	else echo "absent   ap_fixed headers (make fixed fetches them with git)"; fi; \
	if command -v quartus_sh >/dev/null; then echo "ok       quartus_sh"; else echo "absent   quartus_sh (only needed for make quartus)"; fi; \
	[ $$ok = 1 ] && echo "ready: run make test" || { echo "fix the MISSING items above"; exit 1; }

test: pytest float fixed rtl

pytest:
	$(PYTHON) -m pytest tests -q

vectors: $(VECTORS)

$(VECTORS) &: scripts/export_vectors.py $(wildcard reference/*.py) hls/voxlio_config.hpp
	$(PYTHON) scripts/export_vectors.py

# ---- Float build: unit tests, Python == C++ == HLS-compatible C++ -----------

$(BUILD)/float/%: testbench/%.cpp $(HLS_SRC) $(REF_SRC) $(HEADERS)
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) $(INCLUDES) $< $(HLS_SRC) $(REF_SRC) -o $@

float: $(addprefix $(BUILD)/float/,$(TESTS)) $(VECTORS)
	@mkdir -p $(addprefix $(VERIF)/,$(CASES))
	@for t in $(filter-out tb_voxlio,$(TESTS)); do $(BUILD)/float/$$t || exit 1; done
	$(BUILD)/float/tb_voxlio $(VERIF) $(CASE_DIRS)
	$(PYTHON) scripts/compare_outputs.py --level float --report $(VERIF)/float_report.md $(CASES)

# ---- Fixed-point build ------------------------------------------------------

$(AP_TYPES_DIR)/include/ap_fixed.h:
	git clone -q https://github.com/Xilinx/HLS_arbitrary_Precision_Types.git $(AP_TYPES_DIR)
	git -C $(AP_TYPES_DIR) checkout -q $(AP_TYPES_REV)

$(BUILD)/fixed/%: testbench/%.cpp $(HLS_SRC) $(REF_SRC) $(HEADERS) $(AP_TYPES_INC)/ap_fixed.h
	@mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) $(FIXED_FLAGS) $(INCLUDES) $< $(HLS_SRC) $(REF_SRC) -o $@

fixed: $(addprefix $(BUILD)/fixed/,$(TESTS)) $(VECTORS)
	@mkdir -p $(addprefix $(VERIF)/,$(CASES))
	@for t in $(filter-out tb_voxlio,$(TESTS)); do $(BUILD)/fixed/$$t || exit 1; done
	$(BUILD)/fixed/tb_voxlio $(VERIF) $(CASE_DIRS)
	$(PYTHON) scripts/compare_outputs.py --level fixed --report $(VERIF)/fixed_report.md $(CASES)

# ---- RTL: Verilator simulation against the fixed-point C++ core -----------

RTL_PKG  := rtl/voxlio_pkg.sv
RTL_SRC  := $(RTL_PKG) rtl/voxlio_round_sat.sv rtl/voxlio_ram.sv rtl/voxlio_transform.sv \
            rtl/voxlio_grid.sv rtl/voxlio_search.sv rtl/voxlio_jacobian.sv \
            rtl/voxlio_accumulate.sv rtl/voxlio_core.sv
RTL_TB   := testbench/tb_rtl.cpp
RTL_BIN  := $(BUILD)/rtl/Vvoxlio_core
RTL_CXX  := -std=c++17 -O2 -w -DVOXLIO_FIXED_POINT -I$(abspath hls) -I$(abspath reference/cpp) \
            -I$(abspath testbench) -isystem $(abspath $(AP_TYPES_INC))

$(RTL_PKG): scripts/gen_rtl_pkg.py hls/voxlio_config.hpp reference/config.py
	$(PYTHON) scripts/gen_rtl_pkg.py

rtl-lint: $(RTL_PKG)
	verilator --lint-only -Wall --top-module voxlio_core $(RTL_SRC)

$(RTL_BIN): $(RTL_SRC) $(RTL_TB) $(HLS_SRC) $(REF_SRC) $(HEADERS) $(AP_TYPES_INC)/ap_fixed.h
	verilator --cc --exe --build -j 4 -Wall -O2 --assert --top-module voxlio_core -Mdir $(BUILD)/rtl \
	    -CFLAGS "$(RTL_CXX)" $(RTL_SRC) $(abspath $(RTL_TB) $(HLS_SRC) $(REF_SRC))

rtl: $(RTL_BIN) $(VECTORS)
	@mkdir -p $(addprefix $(VERIF)/,$(CASES))
	$(RTL_BIN) $(VERIF) $(CASE_DIRS)
	$(PYTHON) scripts/compare_outputs.py --level rtl --report $(VERIF)/rtl_report.md $(CASES)

# Width sweep, one numeric category at a time.
sweep: $(VECTORS) $(AP_TYPES_INC)/ap_fixed.h
	$(PYTHON) scripts/fixed_point_sweep.py

# Plant deliberate bugs and confirm the tests catch them.
mutation: $(VECTORS) $(AP_TYPES_INC)/ap_fixed.h
	scripts/mutation_check.sh

# CPU baseline timing and the host-loop convergence experiment.
bench: $(VECTORS)
	$(PYTHON) scripts/bench_cpu.py $(CASES)
	$(PYTHON) scripts/pose_convergence.py

# ---- Quartus: synthesis and fit of the RTL core for the DE10-Nano ---------

quartus: $(RTL_PKG)
	cd quartus && quartus_sh --flow compile voxlio
	$(PYTHON) scripts/summarize_quartus.py

# ---- Vitis HLS: C simulation + synthesis (needs the Xilinx tools) -----------

hls: $(VECTORS)
	vitis_hls -f hls/run_hls.tcl
	$(PYTHON) scripts/summarize_synthesis.py

clean:
	rm -rf $(BUILD)
