# Makefile — tile_top cocotb simulation (Verilator or Icarus)
# ─────────────────────────────────────────────────────────────────────────────
# Run from this directory:
#   make                          # Verilator (default), no waveforms
#   make WAVES=1                  # Enable FST waveform dump → waves/tile_top_flat.fst
#   make TESTCASE=test_reset_idle # single test (batch, no waves)
#   make waves                    # Run every test individually, save waves/<name>.fst
# ─────────────────────────────────────────────────────────────────────────────

SIM             ?= verilator
WAVES           ?= 0
TOPLEVEL_LANG   ?= verilog
TOPLEVEL        = tile_top_flat

# ── Source roots ─────────────────────────────────────────────────────────────
SRC_DIR = $(abspath ../../)

# All RTL sources required to elaborate tile_top
VERILOG_SOURCES = \
  $(SRC_DIR)/common/packages/tile_pkg.sv \
  $(SRC_DIR)/common/interfaces/rv_if.sv \
  $(SRC_DIR)/common/interfaces/rv_flit_if.sv \
  $(SRC_DIR)/common/arb/coldfoot_rr_arbiter.sv \
  $(SRC_DIR)/common/mem/tile_mem_1r1w_sync.sv \
  $(SRC_DIR)/common/mem/coldfoot_mem_bytelane_sync.sv \
  $(SRC_DIR)/common/mem/gf180mcu_ocd_sram_lint_stubs.v \
  $(SRC_DIR)/logical_neuron/src/logical_neuron_pkg.sv \
  $(SRC_DIR)/logical_neuron/src/logical_neuron_context_bank.sv \
  $(SRC_DIR)/logical_neuron/src/logical_neuron_state_bank.sv \
  $(SRC_DIR)/logical_neuron/src/logical_neuron_ucode_bank.sv \
  $(SRC_DIR)/neuron_compute/src/neuron_exec.sv \
  $(SRC_DIR)/neuron_compute/src/neuron_compute_core.sv \
  $(SRC_DIR)/tile/src/tile_event_queue_bank.sv \
  $(SRC_DIR)/tile/src/tile_fanout_bank.sv \
  $(SRC_DIR)/tile/src/tile_fanout_pool.sv \
  $(SRC_DIR)/tile/src/tile_ingress.sv \
  $(SRC_DIR)/tile/src/tile_host_io.sv \
  $(SRC_DIR)/tile/src/tile_fanout_executor.sv \
  $(SRC_DIR)/tile/src/tile_dispatch_scheduler.sv \
  $(SRC_DIR)/tile/src/tile_noc_egress.sv \
  $(SRC_DIR)/tile/src/tile_top.sv \
  $(abspath .)/tile_top_flat.sv

# ── Include paths ─────────────────────────────────────────────────────────────
COMPILE_ARGS += -I$(SRC_DIR)
COMPILE_ARGS += -I$(SRC_DIR)/common/packages
COMPILE_ARGS += -I$(SRC_DIR)/common/struct
COMPILE_ARGS += -I$(SRC_DIR)/tile/src

# ── Design-time defines ───────────────────────────────────────────────────────
# TILE_ASIC_LEAN: strips debug counters (same as synthesis path)
COMPILE_ARGS += -DTILE_ASIC_LEAN

# ── Verilator options ─────────────────────────────────────────────────────────
ifeq ($(SIM),verilator)
EXTRA_ARGS += -Wno-fatal --no-timing
EXTRA_ARGS += -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-UNUSEDPARAM -Wno-VARHIDDEN
EXTRA_ARGS += -Wno-DECLFILENAME -Wno-MULTIDRIVEN
# Increase convergence limit for deep combinational paths in tile_top
EXTRA_ARGS += --converge-limit 500
# Waveform tracing: enabled at compile time when WAVES=1 or make waves target
ifeq ($(WAVES),1)
EXTRA_ARGS += --trace-fst --trace-depth 8
endif
endif

# ── Waveform plusargs (passed to the simulator at runtime) ────────────────────
# When WAVES=1 and no explicit TESTCASE, write a single combined FST.
# The `make waves` target overrides WAVE_FILE per test.
WAVE_FILE ?= waves/tile_top_flat.fst
ifeq ($(WAVES),1)
SIM_ARGS += +trace +tracefile=$(WAVE_FILE)
endif

# ── Build / output directories ────────────────────────────────────────────────
SIM_BUILD ?= sim_build/tile_top

# ── Test module(s) ────────────────────────────────────────────────────────────
# All test functions live in test_tile_top.py
COCOTB_TEST_MODULES ?= test_tile_top

# ── All test names (used by the `waves` phony target) ────────────────────────
ALL_TESTS = \
  test_reset_idle \
  test_double_reset \
  test_rv_in_valid_only_transfers_on_ready \
  test_rv_out_ready_backpressure \
  test_ping_pong \
  test_ping_pong_three_consecutive \
  test_csr_write_broadcast \
  test_csr_write_unicast_all_neurons \
  test_csr_write_all_addresses \
  test_ucode_write_single_word \
  test_ucode_write_multiple_words \
  test_ucode_write_broadcast \
  test_fanout_weight_write \
  test_fanout_weight_write_broadcast \
  test_spike_unicast_each_neuron \
  test_spike_with_positive_weight \
  test_spike_with_negative_weight \
  test_spike_invalid_core_id_dropped \
  test_spike_broadcast_accepted \
  test_spike_broadcast_vs_unicast_same_weight \
  test_ingress_fifo_backpressure \
  test_ingress_fifo_drains_after_stall \
  test_asic_lean_no_debug_counters \
  test_enable_gating_blocks_output \
  test_graph_state_clear \
  test_compute_core_dispatch_and_drain \
  test_single_worker_serializes_four_neurons \
  test_context_isolation_across_neurons \
  test_state_bank_csr_read_after_write \
  test_noc_egress_tied_off \
  test_fanout_executor_idle_no_output \
  test_host_output_fifo_depth \
  test_full_pipeline_spike_to_compute \
  test_rapid_send_burst_no_deadlock \
  test_interleaved_config_and_spikes \
  test_all_weight_values_sweep \
  test_all_neuron_ids_unicast \
  test_msg_write_unknown_cmd_kind \
  test_ucode_prog_boundary_word_indices \
  test_simultaneous_host_response_and_spike

# ── `make waves` — per-test FST capture ──────────────────────────────────────
# Runs every test individually with Verilator FST tracing enabled.
# Output: waves/<testname>.fst  (one file per test)
# Usage:  make waves
#         make waves TESTCASE=test_reset_idle   # single test only
.PHONY: waves
waves:
	@mkdir -p waves
	$(foreach t,$(if $(TESTCASE),$(TESTCASE),$(ALL_TESTS)), \
	  echo "=== Waveform capture: $(t) ===" && \
	  $(MAKE) SIM=verilator WAVES=1 TESTCASE=$(t) \
	          WAVE_FILE=waves/$(t).fst \
	          SIM_BUILD=sim_build/waves/$(t) \
	          COCOTB_LOG_LEVEL=WARNING 2>&1 | tail -4 && \
	  echo "  → waves/$(t).fst" ;)

# ── cocotb makefile include ───────────────────────────────────────────────────
include $(shell cocotb-config --makefiles)/Makefile.sim
