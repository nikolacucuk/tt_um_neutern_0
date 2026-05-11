# Tile IP

`tile_top` is the maintained mesh node boundary for the packed logical-neuron plus worker-compute architecture.

## Current Responsibilities

`tile_top` owns:

- tile-scoped `PING` handling and bad-core host responses
- direct loader-to-bank programming for whole-graph deploys
- registered host egress
- registered NoC egress plus direct same-tile routed-spike enqueue
- a tile-local per-worker event queue (SRAM-backed `tile_spike_t` FIFOs)
- RAM-backed logical-neuron context, control, and ucode storage
- a per-worker fanout output queue (`tile_out_spike_t` SRAM-backed FIFOs) fed by the fanout executor
- worker scheduling across `WORKER_DIM <= Z_DIM`
- exact-core state, fanout, and ucode readback
- whole-graph programming through the direct loader path only
- host-visible packet writes and `MSG_PROG_*` programming requests now return `BAD_KIND`; use `load-graph` to program and `dump-neuron` or `read-*` to inspect state
- runtime STDP commits back into the live tile fanout weights
- same-tile and tile-to-tile routed spike delivery on the maintained packed path

The maintained packed path uses [logical_neuron](/C:/Users/justi/Projects/coldfoot_soc/hw/ip/logical_neuron) storage banks plus [neuron_compute_core.sv](/C:/Users/justi/Projects/coldfoot_soc/hw/ip/neuron_compute/src/neuron_compute_core.sv) workers. The old per-neuron wrapper path has been removed.

`tile_top` is now a single flat module. The earlier `tile_top` → `tile_top_packed` → `tile_top_packed_core` wrappers were collapsed on 2026-04-21; the host/NoC output mux lives inline, and `X_COORD` / `Y_COORD` are plumbed as parameters rather than ports.

## Current Limits

- `Z_DIM` means logical local neurons per tile.
- `WORKER_DIM` means physical worker cores per tile.
- Packet `core_id` is a 4-bit local `z`.
- `CMD_WEIGHT` is maintained as a signed 8-bit tile-bank path: writes consume `data[7:0]`, readback returns the live signed byte in `data` with `data_hi = 0`, and `weight_code[0]` extends the local synapse/weight slot index to 5 bits instead of selecting a weight precision mode.
- The maintained packed `2x2x4 / workers=4` SoC cocotb path exercises multi-neuron tiles with `WORKER_DIM < Z_DIM`.
- The maintained SoC cocotb path treats host packet programming as deprecated behavior and checks that it is rejected cleanly.
- The maintained FPGA full-core default is `2x2x172 / workers=4` at `40 MHz`.
- Future work is mainly optimization and further compaction, not another architecture split.

## Source Files

- [tile_top.sv](/C:/Users/justi/Projects/coldfoot_soc/hw/ip/tile/src/tile_top.sv)
- [tile_ingress.sv](/C:/Users/justi/Projects/coldfoot_soc/hw/ip/tile/src/tile_ingress.sv)
- [tile_dispatch_scheduler.sv](/C:/Users/justi/Projects/coldfoot_soc/hw/ip/tile/src/tile_dispatch_scheduler.sv)
- [tile_fanout_executor.sv](/C:/Users/justi/Projects/coldfoot_soc/hw/ip/tile/src/tile_fanout_executor.sv)
- [tile_host_io.sv](/C:/Users/justi/Projects/coldfoot_soc/hw/ip/tile/src/tile_host_io.sv)
- [tile_event_dispatch.sv](/C:/Users/justi/Projects/coldfoot_soc/hw/ip/tile/src/tile_event_dispatch.sv)
- [tile_event_queue_bank.sv](/C:/Users/justi/Projects/coldfoot_soc/hw/ip/tile/src/tile_event_queue_bank.sv)
- [tile_fanout_bank.sv](/C:/Users/justi/Projects/coldfoot_soc/hw/ip/tile/src/tile_fanout_bank.sv)
- [tile_noc_egress.sv](/C:/Users/justi/Projects/coldfoot_soc/hw/ip/tile/src/tile_noc_egress.sv)

## Verification

Typical maintained checks:

```sh
fusesoc run --target lint_coldfoot coldfoot:soc:coldfoot:0.1.0 --X_DIM 2 --Y_DIM 2 --Z_DIM 4 --WORKER_DIM 4
docker exec -i iic-osic-tools_xserver /bin/bash -lc "cd /foss/designs/coldfoot_soc && python3 tools/flows/tool_flow.py formal --project tile"
docker exec -i iic-osic-tools_xserver /bin/bash -lc "cd /foss/designs/coldfoot_soc && fusesoc run --target sim_coldfoot coldfoot:soc:coldfoot:0.1.0 --X_DIM 2 --Y_DIM 2 --Z_DIM 4 --WORKER_DIM 4"
```
