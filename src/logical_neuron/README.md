# Logical Neuron IP

`logical_neuron` owns the tile-side persistent state for logical neurons:

- control/CSR state
- initial register-file values
- runtime context/state snapshots
- packed ucode storage

These banks are consumed by `tile_top`, which schedules logical neurons
onto `neuron_compute` workers.

Current key files:

- `src/logical_neuron_context_bank.sv`
- `src/logical_neuron_state_bank.sv`
- `src/logical_neuron_ucode_bank.sv`

## Cocotb Unit Tests

Maintained logical_neuron cocotb targets:

- `logical_neuron_context_bank`
- `logical_neuron_state_bank`

### Run directly (in this workspace)

From `src/logical_neuron/test/`:

```sh
# context_bank — exhaustive sweep (25 tests, NEURONS_PER_TILE=4)
make -C src/logical_neuron/test SIM=verilator

# Run a single named testcase:
TESTCASE=test_soft_reset_priority_over_commit_same_cycle \
  make -C src/logical_neuron/test SIM=verilator
```

### Run in OSIC Container (recommended for full flow)

### Run in OSIC Container (for full flow)

From repo root:

```sh
docker start iic-osic-tools_xserver
docker exec -t iic-osic-tools_xserver /bin/bash -lc "cd /foss/designs/coldfoot_soc && python3 tools/dev/flow.py sim logical_neuron_context_bank --sim verilator"
docker exec -t iic-osic-tools_xserver /bin/bash -lc "cd /foss/designs/coldfoot_soc && python3 tools/dev/flow.py sim logical_neuron_state_bank --sim verilator"
```

### Run a Single Testcase (optional)

Use `TESTCASE=<name>` with the same target:

```sh
docker exec -t iic-osic-tools_xserver /bin/bash -lc "cd /foss/designs/coldfoot_soc && TESTCASE=test_reset_defaults_all_neurons_port_a python3 tools/dev/flow.py sim logical_neuron_context_bank --sim verilator"
docker exec -t iic-osic-tools_xserver /bin/bash -lc "cd /foss/designs/coldfoot_soc && TESTCASE=test_reset_defaults_and_invalid_epoch_behavior python3 tools/dev/flow.py sim logical_neuron_state_bank --sim verilator"
```

## Passing Test Coverage (Current)

### `logical_neuron_context_bank` — `TESTS=35 PASS=35 FAIL=0 SKIP=0`

Run command: `make -C src/logical_neuron/test SIM=verilator`

Test sections cross-referenced to architecture document:

| Test | Arch §  | What it verifies |
|------|---------|-----------------|
| `test_reset_defaults_all_neurons_port_a` | §2/§7 | After rst_n, all ports A return DEFAULT_RF=0x700 |
| `test_reset_defaults_all_neurons_port_b` | §2/§7 | After rst_n, all ports B return DEFAULT_RF=0x700 |
| `test_mem_stall_permanently_zero` | §2 | mem_stall is 0 across reset/commit/soft_reset/clear |
| `test_soft_reset_writes_rf_clears_all_meta` | §6 | soft_reset stores rf; zeros tag/cmp_ge/cmp_eq/spike |
| `test_soft_reset_all_neuron_indices` | §6 | soft_reset works for all 4 neuron indices |
| `test_soft_reset_overwrites_committed_meta` | §6 | soft_reset clears meta even if entry had flags set |
| `test_commit_writes_full_context` | §6 | commit stores rf+tag+cmp_ge+cmp_eq+spike_flag |
| `test_commit_all_four_neurons_independent` | §6 | All 4 neuron entries independently addressable |
| `test_commit_flags_all_combinations` | §6 | All 8 {cmp_ge,cmp_eq,spike_flag} combos stored |
| `test_commit_tag_both_values` | §6 | TAG_W=1: tag=0 and tag=1 both stored correctly |
| `test_commit_rf_boundary_values` | §6 | RF=0x000 and RF=0xFFF boundary cases |
| `test_successive_commits_overwrite` | §6 | Latest commit to same index wins |
| `test_commit_marks_entry_valid` | §6/§7 | After commit, entry is valid (not DEFAULT_RF) |
| `test_soft_reset_priority_over_commit_same_cycle` | §6 | Priority: soft_reset > commit |
| `test_graph_state_clear_priority_over_soft_reset` | §6 | Priority: graph_state_clear > soft_reset |
| `test_graph_state_clear_priority_over_commit` | §6 | Priority: graph_state_clear > commit |
| `test_graph_state_clear_invalidates_all_neurons` | §6 | 1-cycle clear invalidates all 4 neurons |
| `test_graph_state_clear_single_cycle_sufficient` | §6 | One clock pulse is sufficient |
| `test_reinitialize_after_graph_state_clear` | §6 | Post-clear re-init via soft_reset and commit |
| `test_graph_state_clear_does_not_corrupt_ctx_r` | §6 | Clear only resets valid bits, not ctx_r data |
| `test_ena_blocks_commit` | §6 | ena=0 gates commit |
| `test_ena_does_not_block_soft_reset` | §6 | ena=0 does not block soft_reset |
| `test_ena_blocks_commit_but_not_soft_reset_sequence` | §6 | Both ena behaviours in same test |
| `test_dual_port_same_index_returns_same_data` | §7 | A==B index → identical data on both ports |
| `test_dual_port_different_indices_simultaneous` | §7 | A≠B index → independent correct data |
| `test_dual_port_both_uninitialized_different_indices` | §7 | Both ports DEFAULT_RF for different uninit idx |
| `test_dual_port_a_initialized_b_uninitialized` | §7 | Port A initialized, port B uninitialized simultaneously |
| `test_dual_port_combinatorial_zero_latency` | §7 | Index change reflected combinatorially (no clock) |
| `test_dual_port_full_sweep_all_pairs` | §7/§8 | All 4×4 (A,B) index pairs verified |
| `test_no_bypass_commit_same_cycle_reads_old_value` | §10 | No write-read bypass on commit |
| `test_no_bypass_soft_reset_same_cycle_reads_old_value` | §10 | No write-read bypass on soft_reset |
| `test_validity_soft_reset_marks_entry_valid` | §7 | soft_reset sets ctx_valid_r |
| `test_validity_graph_state_clear_makes_all_entries_invalid` | §7 | clear resets all ctx_valid_r |
| `test_validity_selective_initialization` | §7 | Only written entries differ from DEFAULT_RF |
| `test_default_rf_encodes_threshold_7` | §2/§4 | DEFAULT_RF = {threshold=7, accum=0, vm=0} |

### `logical_neuron_state_bank` — `TESTS=7 PASS=7 FAIL=0 SKIP=0`

- `test_reset_defaults_and_invalid_epoch_behavior`
- `test_csr_ctrl_unicast_soft_reset_and_egress`
- `test_ucode_programming_by_direct_address`
- `test_fanout_prog_drives_all_async_read_domains`
- `test_csr_init_dump_reset_readback_alignment`
- `test_graph_state_clear_resets_visibility_and_egress`
- `test_reset_trigger_broadcast_drains_all_indices`
