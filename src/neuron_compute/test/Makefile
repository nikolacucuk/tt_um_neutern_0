# Makefile — neuron_compute_core cocotb simulation
#
# Usage:
#   make              — run all tests with Verilator
#   make SIM=icarus   — run with Icarus Verilog
#   make WAVES=1      — emit FST waveform (Verilator only)
#
# From the neuron_compute/test directory; the source tree is three levels up.

SIM            ?= verilator
TOPLEVEL_LANG  ?= verilog

SRC_DIR         = $(abspath $(CURDIR)/../../..)
NC_SRC          = $(abspath $(CURDIR)/..)

# ── Verilog source list ───────────────────────────────────────────────────────
VERILOG_SOURCES += $(SRC_DIR)/src/common/packages/tile_pkg.sv
VERILOG_SOURCES += $(SRC_DIR)/src/common/interfaces/rv_if.sv
VERILOG_SOURCES += $(NC_SRC)/src/neuron_exec.sv
VERILOG_SOURCES += $(NC_SRC)/src/neuron_compute_core.sv
VERILOG_SOURCES += $(CURDIR)/neuron_compute_core_tb.sv

# ── Include paths ─────────────────────────────────────────────────────────────
COMPILE_ARGS   += -I$(SRC_DIR)/src/common/packages

# ── Verilator flags ───────────────────────────────────────────────────────────
EXTRA_ARGS     += -Wno-fatal --no-timing

ifeq ($(WAVES),1)
COMPILE_ARGS   += --trace-fst
endif

# ── Toplevel and test module ──────────────────────────────────────────────────
TOPLEVEL        = neuron_compute_core_tb
MODULE          = test_neuron_compute

SIM_BUILD       = $(NC_SRC)/sim_build/neuron_compute_core_tb

include $(shell cocotb-config --makefiles)/Makefile.sim
