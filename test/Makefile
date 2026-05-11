# Makefile
# See https://docs.cocotb.org/en/stable/quickstart.html for more info

# defaults
SIM    ?= verilator
# Gate-level sim must use icarus: verilator is not available in the GL test
# workflow and cannot handle sky130 primitive timing delays.
ifeq ($(GATES),yes)
override SIM := icarus
endif
WAVES  ?= 0
FST    ?= -fst # Use more efficient FST format
TOPLEVEL_LANG ?= verilog

# Verilator root — required by cocotb's Makefile.verilator for version checks
export VERILATOR_ROOT ?= /foss/tools/verilator/share/verilator

# Write results.xml at the repo-level path expected by test-summary/action
export COCOTB_RESULTS_FILE ?= $(CURDIR)/results.xml

SRC_DIR = $(PWD)/../src
PROJECT_SOURCES = \
  tt_um_neutern_pkg.sv \
  common/packages/tile_pkg.sv \
  common/interfaces/rv_if.sv \
  common/mem/coldfoot_mem_bytelane_sync.sv \
  logical_neuron/src/logical_neuron_pkg.sv \
  logical_neuron/src/logical_neuron_context_bank.sv \
  logical_neuron/src/logical_neuron_state_bank.sv \
  logical_neuron/src/logical_neuron_ucode_bank.sv \
  neuron_compute/src/neuron_exec.sv \
  neuron_compute/src/neuron_compute_core.sv \
  tile/src/tile_event_queue_bank.sv \
  tile/src/tile_fanout_bank.sv \
  tile/src/tile_fanout_pool.sv \
  tile/src/tile_ingress.sv \
  tile/src/tile_host_io.sv \
  tile/src/tile_fanout_executor.sv \
  tile/src/tile_dispatch_scheduler.sv \
  tile/src/tile_noc_egress.sv \
  tile/src/tile_top.sv \
  tile_top_tt.sv \
  project.v

ifneq ($(GATES),yes)

# RTL simulation:
SIM_BUILD				= sim_build/rtl
VERILOG_SOURCES += $(addprefix $(SRC_DIR)/,$(PROJECT_SOURCES))

else

# Gate level simulation:
SIM_BUILD				= sim_build/gl
COMPILE_ARGS    += -DGL_TEST
COMPILE_ARGS    += -DFUNCTIONAL
COMPILE_ARGS    += -DUSE_POWER_PINS
COMPILE_ARGS    += -DSIM
COMPILE_ARGS    += -DUNIT_DELAY=\#1
VERILOG_SOURCES += $(PDK_ROOT)/sky130A/libs.ref/sky130_fd_sc_hd/verilog/primitives.v
VERILOG_SOURCES += $(PDK_ROOT)/sky130A/libs.ref/sky130_fd_sc_hd/verilog/sky130_fd_sc_hd.v

# this gets copied in by the GDS action workflow
VERILOG_SOURCES += $(PWD)/gate_level_netlist.v

endif

# Design defines
COMPILE_ARGS		+= -DTILE_ASIC_LEAN

# Allow sharing configuration between design and testbench via `include`:
COMPILE_ARGS 		+= -I$(SRC_DIR)
COMPILE_ARGS 		+= -I$(SRC_DIR)/common/packages
COMPILE_ARGS 		+= -I$(SRC_DIR)/tile/src

# Verilator-specific: suppress non-fatal warnings; ignore delays in tb
ifneq ($(SIM),icarus)
EXTRA_ARGS 			+= -Wno-fatal --no-timing
endif
# Waveform tracing: enabled at compile time when WAVES=1
ifeq ($(WAVES),1)
ifneq ($(SIM),icarus)
EXTRA_ARGS += --trace-fst --trace-depth 8
endif
endif

# Waveform plusargs: passed to simulator at runtime when WAVES=1
WAVE_FILE ?= tb.fst
ifeq ($(WAVES),1)
SIM_ARGS += +trace +tracefile=$(WAVE_FILE)
endif

# Include the testbench sources:
VERILOG_SOURCES += $(PWD)/tb.v
TOPLEVEL = tb

# List test modules to run, separated by commas and without the .py suffix:
COCOTB_TEST_MODULES = test

# ── All TT-level test names (for `make waves`) ────────────────────────────────
TT_ALL_TESTS = \
  test_project \
  test_reset_clears_all_outputs \
  test_no_x_propagation_after_reset \
  test_rv_in_ready_asserted_at_idle \
  test_no_transfer_without_valid \
  test_single_flit_handshake \
  test_spike_to_all_four_neurons \
  test_all_weight_values_accepted \
  test_max_positive_weight \
  test_max_negative_weight \
  test_zero_weight \
  test_reserved_bits_ignored \
  test_full_flit_sweep_no_crash \
  test_back_to_back_flits_same_neuron \
  test_back_to_back_flits_all_neurons \
  test_enable_disable_recover \
  test_uo_out_format_zero_at_idle \
  test_uo_out_reserved_field_is_zero \
  test_ingress_backpressure_fills_and_drains \
  test_reset_during_spike_ingress \
  test_rapid_reset_recovery

# ── `make waves` — per-test FST capture ──────────────────────────────────────
# Runs every test individually with Verilator FST tracing enabled.
# Output: waves/<testname>.fst  (one file per test)
.PHONY: waves
waves:
	@mkdir -p waves
	$(foreach t,$(if $(TESTCASE),$(TESTCASE),$(TT_ALL_TESTS)), \
	  echo "=== Waveform capture: $(t) ===" && \
	  $(MAKE) SIM=verilator WAVES=1 TESTCASE=$(t) \
	          WAVE_FILE=waves/$(t).fst \
	          SIM_BUILD=sim_build/waves/$(t) \
	          COCOTB_LOG_LEVEL=WARNING 2>&1 | tail -4 && \
	  echo "  → waves/$(t).fst" ;)

# include cocotb's make rules to take care of the simulator setup
include $(shell cocotb-config --makefiles)/Makefile.sim
