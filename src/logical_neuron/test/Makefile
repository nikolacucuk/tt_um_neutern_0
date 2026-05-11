# Makefile — cocotb testbench for logical_neuron_context_bank
# Runs with: make SIM=verilator   (default)
#            make SIM=icarus

SIM ?= verilator
TOPLEVEL_LANG ?= verilog

SRC_DIR = $(realpath $(CURDIR)/../../..)

# ── Design sources (ordered: packages first) ──────────────────────────────────
VERILOG_SOURCES += $(SRC_DIR)/src/common/packages/tile_pkg.sv
VERILOG_SOURCES += $(SRC_DIR)/src/common/packages/tile_flit_types.vh
VERILOG_SOURCES += $(SRC_DIR)/src/logical_neuron/src/logical_neuron_pkg.sv
VERILOG_SOURCES += $(SRC_DIR)/src/logical_neuron/src/logical_neuron_context_bank.sv

# ── Include paths ─────────────────────────────────────────────────────────────
COMPILE_ARGS += -I$(SRC_DIR)/src/common/packages
COMPILE_ARGS += -I$(SRC_DIR)/src/tile/src

# ── Top-level parameters ──────────────────────────────────────────────────────
# Use NEURONS_PER_TILE=4 for comprehensive multi-index testing
COMPILE_ARGS += -GNEURONS_PER_TILE=4

# ── Design defines ────────────────────────────────────────────────────────────
COMPILE_ARGS += -DTILE_ASIC_LEAN

# ── Verilator options ─────────────────────────────────────────────────────────
EXTRA_ARGS += -Wno-fatal --no-timing

# ── DUT and test module ───────────────────────────────────────────────────────
TOPLEVEL = logical_neuron_context_bank
COCOTB_TEST_MODULES = test_logical_neuron_context_bank

# ── Output directory ──────────────────────────────────────────────────────────
SIM_BUILD = $(CURDIR)/../sim_build/logical_neuron_context_bank_tb

include $(shell cocotb-config --makefiles)/Makefile.sim
