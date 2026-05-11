# TT-Boundary Cocotb Testbench — tt_um_neutern_0 (Coldfoot SNN Tile)

This directory contains the **primary verification testbench** for the `tt_um_neutern_0` TinyTapeout submission.
It uses [cocotb](https://docs.cocotb.org/en/stable/) with Verilator to exhaustively test the DUT through the TT-standard pin interface.

## Test coverage (`test.py` — 31 tests, including header config/readback)

The testbench exercises the full TT pin interface of the SNN tile:

| Category | Tests | Coverage |
|---|---|---|
| Reset & idle | `test_project`, `test_reset_clears_all_outputs`, `test_no_x_propagation_after_reset` | Power-on, `rst_n` de-assertion, all outputs defined |
| RV-in protocol | `test_rv_in_ready_asserted_at_idle`, `test_no_transfer_without_valid`, `test_single_flit_handshake` | `uio_in[0]`=valid, `uio_out[0]`=ready handshake |
| Spike ingress | `test_spike_to_all_four_neurons`, `test_spike_neuron_00/10/01/11` | All neuron grid positions |
| Weight encoding | `test_all_weight_values_accepted`, `test_max_positive_weight`, `test_max_negative_weight`, `test_zero_weight` | Full 4-bit signed weight range |
| Reserved bits (spike mode) | `test_reserved_bits_ignored`, `test_full_flit_sweep_no_crash` | `ui_in[3:2]` ignored when `uio_in[2]=0` |
| Header config/readback | `test_weight_header_write_then_readback_all_neurons`, `test_isa_header_program_ack` | Weight write/read headers and ISA headers |
| Back-pressure | `test_rv_out_valid_held_when_ready_0`, `test_rv_out_ready_0_does_not_cause_deadlock` | `uio_in[1]`=ready flow control |
| Burst / pipeline | `test_back_to_back_flits_same_neuron`, `test_back_to_back_flits_all_neurons`, `test_burst_8_flits_mixed_neurons_weights` | Multi-flit streams |
| Enable gating | `test_enable_disable_recover`, `test_enable_0_no_output` | `ena` signal gating |
| Output format | `test_uo_out_format_zero_at_idle`, `test_uo_out_reserved_field_is_zero` | `uo_out[7:0]` payload |
| FIFO / reset | `test_ingress_backpressure_fills_and_drains`, `test_reset_during_spike_ingress`, `test_rapid_reset_recovery` | Queue fill/drain, mid-transaction reset |

### Pin interface (TT-standard)

| Signal | Description |
|---|---|
| `ui_in[7:0]` | Mode-dependent flit byte (spike / weight-header / ISA-header) |
| `uio_in[0]` | `rv_in_valid` — host asserts to present a flit |
| `uio_in[1]` | `rv_out_ready` — host asserts to accept output |
| `uio_in[2]` | `rv_in_is_header` — `0`: spike mode, `1`: header mode |
| `uio_in[3]` | `rv_in_header_is_isa` — with bit2=`1`: `0`: weight header, `1`: ISA header |
| `uio_out[0]` | `rv_in_ready` — tile asserts when ingress can accept |
| `uio_out[1]` | `rv_out_valid` — tile asserts when output is available |
| `uo_out[7:0]` | `rv_out_payload[7:0]` — output flit byte |

## Setting up

1. Edit [Makefile](Makefile) and modify `PROJECT_SOURCES` to point to your Verilog files.
2. Edit [tb.v](tb.v) and replace `tt_um_neutern_0` if your module name changes.

## How to run

To run the RTL simulation:

```sh
make -B
```

To run gatelevel simulation, first harden your project and copy `../runs/wokwi/results/final/verilog/gl/{your_module_name}.v` to `gate_level_netlist.v`.

Then run:

```sh
make -B GATES=yes
```

If you wish to save the waveform in VCD format instead of FST format, edit tb.v to use `$dumpfile("tb.vcd");` and then run:

```sh
make -B FST=
```

This will generate `tb.vcd` instead of `tb.fst`.

## How to view the waveform file

Using GTKWave

```sh
gtkwave tb.fst tb.gtkw
```

Using Surfer

```sh
surfer tb.fst
```
