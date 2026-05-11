# tile_top Cocotb Testbench

This directory contains a cocotb testbench targeting `tile_top` directly
(below the TT adapter), using a flat-signal wrapper (`tile_top_flat.sv`)
to avoid Verilator parameterized-interface-port limitations.

## DUT topology

```
tile_top_flat (flat wrapper, test/tile_top_flat.sv)
  └─ tile_top (tile/src/tile_top.sv)
       ├─ tile_ingress
       ├─ tile_host_io
       ├─ tile_fanout_pool / tile_fanout_executor
       ├─ tile_dispatch_scheduler
       ├─ tile_event_queue_bank
       ├─ neuron_compute_core × WORKER_CORES_PER_TILE
       └─ tile_noc_egress
```

Parameters used in simulation:
`MESSAGE_W=40, NEURONS_PER_TILE=4, WORKER_CORES_PER_TILE=1, FANOUT_POOL_DEPTH=4,
LOGICAL_EVENT_QUEUE_DEPTH=4, INGRESS_QUEUE_DEPTH=4, HOST_OUTPUT_FIFO_DEPTH=4`

## How to run

```sh
cd src/tile/test
make SIM=verilator
```

## Test coverage (`test_tile_top.py` — 40 tests)

**14 tests pass** at the tile_top level (all host-IO and configuration paths).
**26 tests are blocked** by a known Verilator convergence limitation (see below).

| Section | Tests | Pass? |
|---|---|---|
| §R Reset | `test_reset_idle`, `test_double_reset` | ✅ |
| §H RV handshake | `test_rv_in_valid_only_transfers_on_ready`, `test_rv_out_ready_backpressure` | ✅ |
| §6.4 Host I/O | `test_ping_pong`, `test_ping_pong_three_consecutive` | ✅ |
| §6.4 CSR writes | `test_csr_write_broadcast`, `test_csr_write_unicast_all_neurons`, `test_csr_write_all_addresses` | ✅ |
| §6.4 Ucode prog | `test_ucode_write_single_word`, `test_ucode_write_multiple_words`, `test_ucode_write_broadcast` | ✅ |
| §6.4 Fanout | `test_fanout_weight_write`, `test_fanout_weight_write_broadcast` | ✅ |
| §6.1 Spike ingress | `test_spike_unicast_each_neuron`, … (7 tests) | ❌ (see below) |
| §6.2 Compute core | `test_compute_core_dispatch_and_drain`, … (5 tests) | ❌ (see below) |
| §7 Misc | back-pressure, ASIC-lean, burst, sweep tests (14 tests) | ❌ (see below) |

### Verilator DIDNOTCONVERGE limitation

Verilator's `--no-timing` mode evaluates combinational logic iteratively until
stable. `tile_top` contains a combinational path from `enqueue_ev_if.valid`
→ `ingress_event_enqueue_fire_c` → `enqueue_ev_payload_c.neuron_idx`
→ `enq_worker_sel_c` → `logical_event_full_c` → `ingress_spike_enqueue_ready`
→ `current_packet_consume` → `enqueue_ev_if.valid`. This can oscillate when
a targeted worker's queue is full while worker-0's queue has slack, causing
Verilator to exceed its convergence limit (`--converge-limit 500`).

This loop is only active when the compute core is processing spikes. Tests
that send spikes and wait for output trigger this condition and cannot pass
through the flat wrapper. These same paths are exercised at the **TT-boundary
level** by `test/test.py` (31/31 PASS), which uses the registered
`tile_top_tt` adapter that breaks the loop.

## tile_top_flat.sv

`tile_top_flat.sv` is a simulation-only flat wrapper that:
- Exposes all `tile_top` ports as plain logic signals (no SV interface ports)
- Instantiates `rv_if #(.WIDTH(MESSAGE_W))` internally
- Wires the internal interfaces to `tile_top`

This file must **not** be included in synthesis.
