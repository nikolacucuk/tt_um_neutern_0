![](../../workflows/gds/badge.svg) ![](../../workflows/docs/badge.svg) ![](../../workflows/test/badge.svg) ![](../../workflows/fpga/badge.svg)

# tt_um_neutern_0 — Coldfoot SNN Tile (TinyTapeout)

A single **Spiking Neural Network (SNN) tile** from the Coldfoot SoC, submitted to TinyTapeout.
The tile hosts a 2×2 neuron grid (4 neurons), one shared compute worker, a fanout pool,
and a host I/O path — all interfaced to TinyTapeout's 8-bit UI pins via a compact
ready/valid (RV) protocol.

- [Architecture documentation](docs/tt_um_neutern_0_architecture.md)
- [Project info](docs/info.md)

## TT pin interface

| Pin | Direction | Description |
|---|---|---|
| `ui_in[7:0]` | input | Mode-dependent flit byte (spike / weight-header / ISA-header) |
| `uio_in[0]` | input | `rv_in_valid` — host presents a flit |
| `uio_in[1]` | input | `rv_out_ready` — host ready to accept output |
| `uio_in[2]` | input | `rv_in_is_header` — `0`: spike flit, `1`: header/config flit |
| `uio_in[3]` | input | `rv_in_header_is_isa` — with `uio_in[2]=1`: `0`: weight header, `1`: ISA header |
| `uio_out[0]` | output | `rv_in_ready` — tile can accept a flit |
| `uio_out[1]` | output | `rv_out_valid` — tile has output available |
| `uo_out[7:0]` | output | `rv_out_payload[7:0]` — output flit byte |
| `ena` | input | Tile enable (active high) |
| `clk` | input | Clock |
| `rst_n` | input | Active-low reset |

## Verification

### TT-boundary tests (`test/test.py`) — includes header config/readback coverage ✅

Exercises the full DUT stack through TT pins using Verilator:

```sh
cd test && make SIM=verilator
```

Coverage: spike ingress for all 4 neurons, full 4-bit signed weight range,
header-based weight write/readback, ISA-header programming ack,
back-pressure, burst streaming, enable gating, reset-during-transfer,
and `uo_out` format verification.

### tile_top-level tests (`src/tile/test/test_tile_top.py`) — 40/40 pass ✅

Exercises `tile_top` internals directly (host I/O, CSR, ucode, fanout weight):

```sh
cd src/tile/test && make SIM=verilator
```

All 40 tile-level tests now pass, including host I/O, CSR, ucode, fanout weight,
spike ingress, compute-core dispatch/drain, back-pressure, burst streaming,
and sweep scenarios.

## Project structure

```
src/
  tile_top_tt.sv        — TT adapter (spike decode → message_packet_t)
  tile/src/tile_top.sv  — SNN tile core
  logical_neuron/       — Neuron context, state, ucode banks
  neuron_compute/       — Compute worker (neuron_exec + core)
  common/               — Shared packages, interfaces, memory primitives
test/
  test.py               — 31-test TT-boundary cocotb suite (primary)
  Makefile              — Verilator-based build for TT tests
src/tile/test/
  test_tile_top.py      — 40-test tile_top cocotb suite (supplemental)
  tile_top_flat.sv      — Flat wrapper for Verilator simulation
```