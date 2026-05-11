"""
test_logical_neuron_context_bank.py — exhaustive cocotb verification

DUT: logical_neuron_context_bank  (NEURONS_PER_TILE=4, RF_FLAT_W=12, TAG_W=1)

Row layout (CTX_W=16, MSB→LSB):
  [15] spike_flag
  [14] cmp_eq
  [13] cmp_ge
  [12] last_tag  (TAG_W=1)
  [11:0] rf_state_flat (3×4-bit: bits[11:8]=rf[2]=threshold, [7:4]=rf[1]=accum, [3:0]=rf[0]=vm)

DEFAULT_RF = 0x700  ({threshold=7, accum=0, vm=0})
mem_stall  = always 0 (combinatorial reads, no stall cycles)

Verification coverage (cross-referenced to architecture doc sections):
  §2  — module overview properties (FF array, dual-port comb read, validity)
  §5  — architecture diagram (write mux priority, read port behaviour)
  §6  — write sequencer priority (graph_state_clear > soft_reset > commit)
  §7  — dual-port combinatorial read (zero latency, port A ≠ port B)
  §8  — port usage (read_idx_a = dispatch, read_idx_b = dump)
  §10 — timing model (no write-read bypass on same cycle)
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ReadWrite, RisingEdge

# ── Constants matching current DUT ────────────────────────────────────────────
# RF_FLAT_W=12 (3×4-bit), TAG_W=1, NEURONS_PER_TILE=4
DEFAULT_RF   = 0x700        # threshold=7, accum=0, vm=0
NEURONS      = 4            # NEURONS_PER_TILE used for this TB
RF_MASK      = 0xFFF        # 12-bit mask
TAG_MASK     = 0x1          # 1-bit mask
_CLOCK_TASK  = None


# ── Helper: check port A returns default context ───────────────────────────────
def _expect_default_port_a(dut, msg: str = "") -> None:
    assert int(dut.read_rf_state_flat_a.value) == DEFAULT_RF, \
        f"[A rf] expected {DEFAULT_RF:#x}, got {int(dut.read_rf_state_flat_a.value):#x} {msg}"
    assert int(dut.read_last_tag_a.value)   == 0, f"[A tag] {msg}"
    assert int(dut.read_cmp_ge_a.value)     == 0, f"[A cmp_ge] {msg}"
    assert int(dut.read_cmp_eq_a.value)     == 0, f"[A cmp_eq] {msg}"
    assert int(dut.read_spike_flag_a.value) == 0, f"[A spike] {msg}"


# ── Helper: check port B returns default context ───────────────────────────────
def _expect_default_port_b(dut, msg: str = "") -> None:
    assert int(dut.read_rf_state_flat_b.value) == DEFAULT_RF, \
        f"[B rf] expected {DEFAULT_RF:#x}, got {int(dut.read_rf_state_flat_b.value):#x} {msg}"
    assert int(dut.read_last_tag_b.value)   == 0, f"[B tag] {msg}"
    assert int(dut.read_cmp_ge_b.value)     == 0, f"[B cmp_ge] {msg}"
    assert int(dut.read_cmp_eq_b.value)     == 0, f"[B cmp_eq] {msg}"
    assert int(dut.read_spike_flag_b.value) == 0, f"[B spike] {msg}"


async def _tick(dut, cycles: int = 1) -> None:
    for _ in range(cycles):
        await RisingEdge(dut.clk)
        await ReadWrite()


async def _drive_idle(dut) -> None:
    """Drive all control inputs to idle (no write, no clear)."""
    dut.graph_state_clear.value   = 0
    dut.soft_reset_valid.value    = 0
    dut.soft_reset_idx.value      = 0
    dut.init_rf_flat.value        = 0
    dut.commit_valid.value        = 0
    dut.commit_idx.value          = 0
    dut.commit_rf_state_flat.value = 0
    dut.commit_last_tag.value     = 0
    dut.commit_cmp_ge.value       = 0
    dut.commit_cmp_eq.value       = 0
    dut.commit_spike_flag.value   = 0


async def _set_read_indices(dut, idx_a: int, idx_b: int) -> None:
    dut.read_idx_a.value = idx_a
    dut.read_idx_b.value = idx_b


async def _reset_dut(dut) -> None:
    global _CLOCK_TASK
    if _CLOCK_TASK is None or _CLOCK_TASK.done():
        _CLOCK_TASK = cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())

    dut.rst_n.value = 0
    dut.ena.value = 0
    await _drive_idle(dut)
    await _set_read_indices(dut, 0, 0)

    await _tick(dut, 3)

    dut.rst_n.value = 1
    dut.ena.value = 1
    await _tick(dut, 3)


async def _soft_reset_write(dut, *, idx: int, init_rf: int) -> None:
    """Issue one soft_reset write cycle; waits 2 settling cycles after."""
    dut.soft_reset_valid.value = 1
    dut.soft_reset_idx.value   = idx
    dut.init_rf_flat.value     = init_rf & RF_MASK
    await _tick(dut, 1)
    dut.soft_reset_valid.value = 0
    await _tick(dut, 2)


async def _commit_write(
    dut,
    *,
    idx: int,
    rf: int,
    tag: int,
    cmp_ge: int,
    cmp_eq: int,
    spike: int,
) -> None:
    """Issue one commit write cycle; waits 2 settling cycles after."""
    dut.commit_valid.value         = 1
    dut.commit_idx.value           = idx
    dut.commit_rf_state_flat.value = rf & RF_MASK
    dut.commit_last_tag.value      = tag & TAG_MASK
    dut.commit_cmp_ge.value        = cmp_ge
    dut.commit_cmp_eq.value        = cmp_eq
    dut.commit_spike_flag.value    = spike
    await _tick(dut, 1)
    dut.commit_valid.value = 0
    await _tick(dut, 2)



# ═══════════════════════════════════════════════════════════════════════════════
# §2 / §5  Module overview: reset, defaults, mem_stall
# ═══════════════════════════════════════════════════════════════════════════════

@cocotb.test()
async def test_reset_defaults_all_neurons_port_a(dut):
    """§2 After rst_n deasserted, every neuron index on port A returns DEFAULT_RF
    with all-zero meta fields. Covers arch-doc §4 (port summary), §7 (validity)."""
    await _reset_dut(dut)

    for idx in range(NEURONS):
        await _set_read_indices(dut, idx, idx)
        await _tick(dut, 1)
        _expect_default_port_a(dut, msg=f"neuron {idx}")


@cocotb.test()
async def test_reset_defaults_all_neurons_port_b(dut):
    """§2 After rst_n deasserted, every neuron index on port B returns DEFAULT_RF
    with all-zero meta fields."""
    await _reset_dut(dut)

    for idx in range(NEURONS):
        await _set_read_indices(dut, idx, idx)
        await _tick(dut, 1)
        _expect_default_port_b(dut, msg=f"neuron {idx}")


@cocotb.test()
async def test_mem_stall_permanently_zero(dut):
    """§2 mem_stall is always 0 — combinatorial reads never stall.
    Verified across reset, idle, soft_reset, commit, and graph_state_clear."""
    await _reset_dut(dut)
    assert int(dut.mem_stall.value) == 0, "mem_stall after reset"

    # During a commit write cycle
    dut.commit_valid.value = 1
    dut.commit_idx.value   = 0
    await _tick(dut, 1)
    assert int(dut.mem_stall.value) == 0, "mem_stall during commit"
    dut.commit_valid.value = 0

    # During a soft_reset cycle
    dut.soft_reset_valid.value = 1
    dut.soft_reset_idx.value   = 0
    await _tick(dut, 1)
    assert int(dut.mem_stall.value) == 0, "mem_stall during soft_reset"
    dut.soft_reset_valid.value = 0

    # During graph_state_clear
    dut.graph_state_clear.value = 1
    await _tick(dut, 1)
    assert int(dut.mem_stall.value) == 0, "mem_stall during graph_state_clear"
    dut.graph_state_clear.value = 0

    await _tick(dut, 2)
    assert int(dut.mem_stall.value) == 0, "mem_stall after all ops"


# ═══════════════════════════════════════════════════════════════════════════════
# §6  Write sequencer — soft_reset path
# ═══════════════════════════════════════════════════════════════════════════════

@cocotb.test()
async def test_soft_reset_writes_rf_clears_all_meta(dut):
    """§6 soft_reset writes init_rf_flat and zeros tag/cmp_ge/cmp_eq/spike_flag.
    Architecture doc §6: soft_reset_valid writes pack_ctx(init_rf_flat, '0, 0, 0, 0)."""
    await _reset_dut(dut)

    init_rf = 0xABC  # 12-bit test value
    await _soft_reset_write(dut, idx=2, init_rf=init_rf)

    await _set_read_indices(dut, 2, 2)
    await _tick(dut, 1)

    assert int(dut.read_rf_state_flat_a.value) == init_rf, "rf after soft_reset"
    assert int(dut.read_last_tag_a.value)       == 0,       "tag after soft_reset"
    assert int(dut.read_cmp_ge_a.value)         == 0,       "cmp_ge after soft_reset"
    assert int(dut.read_cmp_eq_a.value)         == 0,       "cmp_eq after soft_reset"
    assert int(dut.read_spike_flag_a.value)     == 0,       "spike_flag after soft_reset"


@cocotb.test()
async def test_soft_reset_all_neuron_indices(dut):
    """§6 soft_reset works for all 4 neuron indices with independent storage."""
    await _reset_dut(dut)

    rf_vals = [0x123, 0x456, 0x789, 0xABC]

    for idx in range(NEURONS):
        await _soft_reset_write(dut, idx=idx, init_rf=rf_vals[idx])

    # Verify each index independently via both ports
    for idx in range(NEURONS):
        await _set_read_indices(dut, idx, idx)
        await _tick(dut, 1)
        got = int(dut.read_rf_state_flat_a.value)
        assert got == rf_vals[idx], f"neuron {idx}: expected {rf_vals[idx]:#x}, got {got:#x}"
        assert int(dut.read_last_tag_a.value)   == 0, f"tag neuron {idx}"
        assert int(dut.read_cmp_ge_a.value)     == 0, f"cmp_ge neuron {idx}"
        assert int(dut.read_cmp_eq_a.value)     == 0, f"cmp_eq neuron {idx}"
        assert int(dut.read_spike_flag_a.value) == 0, f"spike neuron {idx}"


@cocotb.test()
async def test_soft_reset_overwrites_committed_meta(dut):
    """§6 soft_reset clears meta even when the entry had been committed with flags set."""
    await _reset_dut(dut)

    # First write a full commit with all flags set
    await _commit_write(dut, idx=1, rf=0xF0F, tag=1, cmp_ge=1, cmp_eq=1, spike=1)

    # Verify flags are stored
    await _set_read_indices(dut, 1, 1)
    await _tick(dut, 1)
    assert int(dut.read_cmp_ge_a.value)     == 1
    assert int(dut.read_cmp_eq_a.value)     == 1
    assert int(dut.read_spike_flag_a.value) == 1
    assert int(dut.read_last_tag_a.value)   == 1

    # Now soft_reset that entry — all meta must go to zero
    await _soft_reset_write(dut, idx=1, init_rf=0x321)
    await _set_read_indices(dut, 1, 1)
    await _tick(dut, 1)

    assert int(dut.read_rf_state_flat_a.value) == 0x321, "rf after soft_reset overwrite"
    assert int(dut.read_last_tag_a.value)       == 0,     "tag cleared by soft_reset"
    assert int(dut.read_cmp_ge_a.value)         == 0,     "cmp_ge cleared by soft_reset"
    assert int(dut.read_cmp_eq_a.value)         == 0,     "cmp_eq cleared by soft_reset"
    assert int(dut.read_spike_flag_a.value)     == 0,     "spike cleared by soft_reset"


# ═══════════════════════════════════════════════════════════════════════════════
# §6  Write sequencer — commit path
# ═══════════════════════════════════════════════════════════════════════════════

@cocotb.test()
async def test_commit_writes_full_context(dut):
    """§6 Commit stores rf + tag + cmp_ge + cmp_eq + spike_flag in full."""
    await _reset_dut(dut)

    await _commit_write(dut, idx=1, rf=0x511, tag=1, cmp_ge=1, cmp_eq=0, spike=1)

    await _set_read_indices(dut, 1, 1)
    await _tick(dut, 1)

    assert int(dut.read_rf_state_flat_a.value) == 0x511
    assert int(dut.read_last_tag_a.value)       == 1
    assert int(dut.read_cmp_ge_a.value)         == 1
    assert int(dut.read_cmp_eq_a.value)         == 0
    assert int(dut.read_spike_flag_a.value)     == 1


@cocotb.test()
async def test_commit_all_four_neurons_independent(dut):
    """§6 Each of the 4 neuron entries is independently addressable via commit."""
    await _reset_dut(dut)

    payloads = [
        dict(rf=0x100, tag=0, cmp_ge=0, cmp_eq=0, spike=0),
        dict(rf=0x200, tag=1, cmp_ge=1, cmp_eq=0, spike=0),
        dict(rf=0x300, tag=0, cmp_ge=0, cmp_eq=1, spike=0),
        dict(rf=0x400, tag=1, cmp_ge=1, cmp_eq=1, spike=1),
    ]

    for idx, p in enumerate(payloads):
        await _commit_write(dut, idx=idx, **p)

    for idx, p in enumerate(payloads):
        await _set_read_indices(dut, idx, idx)
        await _tick(dut, 1)
        assert int(dut.read_rf_state_flat_a.value) == p["rf"],     f"rf[{idx}]"
        assert int(dut.read_last_tag_a.value)       == p["tag"],    f"tag[{idx}]"
        assert int(dut.read_cmp_ge_a.value)         == p["cmp_ge"], f"ge[{idx}]"
        assert int(dut.read_cmp_eq_a.value)         == p["cmp_eq"], f"eq[{idx}]"
        assert int(dut.read_spike_flag_a.value)     == p["spike"],  f"spike[{idx}]"


@cocotb.test()
async def test_commit_flags_all_combinations(dut):
    """§6 All 8 flag combinations {cmp_ge, cmp_eq, spike_flag} are stored correctly."""
    await _reset_dut(dut)

    for combo in range(8):
        ge    = (combo >> 2) & 1
        eq    = (combo >> 1) & 1
        spike = (combo >> 0) & 1
        rf    = 0x100 + combo

        await _commit_write(dut, idx=0, rf=rf, tag=0, cmp_ge=ge, cmp_eq=eq, spike=spike)
        await _set_read_indices(dut, 0, 0)
        await _tick(dut, 1)

        assert int(dut.read_cmp_ge_a.value)         == ge,    f"cmp_ge combo={combo}"
        assert int(dut.read_cmp_eq_a.value)         == eq,    f"cmp_eq combo={combo}"
        assert int(dut.read_spike_flag_a.value)     == spike, f"spike combo={combo}"
        assert int(dut.read_rf_state_flat_a.value)  == rf,    f"rf combo={combo}"


@cocotb.test()
async def test_commit_tag_both_values(dut):
    """§6 TAG_W=1: both tag=0 and tag=1 are stored and returned correctly."""
    await _reset_dut(dut)

    for tag_val in [0, 1]:
        await _commit_write(dut, idx=0, rf=0xABC, tag=tag_val,
                            cmp_ge=0, cmp_eq=0, spike=0)
        await _set_read_indices(dut, 0, 0)
        await _tick(dut, 1)
        assert int(dut.read_last_tag_a.value) == tag_val, f"tag={tag_val}"


@cocotb.test()
async def test_commit_rf_boundary_values(dut):
    """§6 RF boundary values RF=0x000 (all-zero) and RF=0xFFF (all-ones) stored correctly."""
    await _reset_dut(dut)

    for rf_val in [0x000, 0xFFF]:
        await _commit_write(dut, idx=0, rf=rf_val, tag=0,
                            cmp_ge=0, cmp_eq=0, spike=0)
        await _set_read_indices(dut, 0, 0)
        await _tick(dut, 1)
        assert int(dut.read_rf_state_flat_a.value) == rf_val, f"rf={rf_val:#x}"


@cocotb.test()
async def test_successive_commits_overwrite(dut):
    """§6 Successive commits to the same index update the stored value (latest wins)."""
    await _reset_dut(dut)

    for rf_val in [0x111, 0x222, 0x333, 0x444]:
        await _commit_write(dut, idx=2, rf=rf_val, tag=0,
                            cmp_ge=0, cmp_eq=0, spike=0)

    await _set_read_indices(dut, 2, 2)
    await _tick(dut, 1)
    assert int(dut.read_rf_state_flat_a.value) == 0x444, "last commit value wins"


@cocotb.test()
async def test_commit_marks_entry_valid(dut):
    """§6 / §7 After commit, ctx_valid_r[idx]=1 so the entry no longer returns DEFAULT_RF."""
    await _reset_dut(dut)

    # Verify DEFAULT_RF before write
    await _set_read_indices(dut, 3, 3)
    await _tick(dut, 1)
    _expect_default_port_a(dut, "before commit")

    # Commit a value that differs from DEFAULT_RF
    await _commit_write(dut, idx=3, rf=0x001, tag=0, cmp_ge=0, cmp_eq=0, spike=0)

    await _set_read_indices(dut, 3, 3)
    await _tick(dut, 1)
    assert int(dut.read_rf_state_flat_a.value) != DEFAULT_RF, "entry no longer returns DEFAULT_RF"
    assert int(dut.read_rf_state_flat_a.value) == 0x001


# ═══════════════════════════════════════════════════════════════════════════════
# §6  Write sequencer — priority arbitration
# ═══════════════════════════════════════════════════════════════════════════════

@cocotb.test()
async def test_soft_reset_priority_over_commit_same_cycle(dut):
    """§6 Priority: soft_reset > commit. When both asserted same cycle, soft_reset wins.
    Result: rf=init_rf_flat, tag=0, all flags=0 (NOT the commit values)."""
    await _reset_dut(dut)

    # Pre-load with a known commit value
    await _commit_write(dut, idx=3, rf=0xDEA, tag=0, cmp_ge=0, cmp_eq=0, spike=0)

    # Assert soft_reset and commit simultaneously
    dut.soft_reset_valid.value     = 1
    dut.soft_reset_idx.value       = 3
    dut.init_rf_flat.value         = 0xBEF

    dut.commit_valid.value         = 1
    dut.commit_idx.value           = 3
    dut.commit_rf_state_flat.value = 0xF0F  # must NOT be written
    dut.commit_last_tag.value      = 1
    dut.commit_cmp_ge.value        = 1
    dut.commit_cmp_eq.value        = 1
    dut.commit_spike_flag.value    = 1

    await _tick(dut, 1)
    await _drive_idle(dut)
    await _set_read_indices(dut, 3, 3)
    await _tick(dut, 1)

    # soft_reset writes: rf=0xBEF, all meta cleared
    assert int(dut.read_rf_state_flat_a.value) == 0xBEF, "soft_reset rf wins"
    assert int(dut.read_last_tag_a.value)       == 0,     "tag cleared by soft_reset"
    assert int(dut.read_cmp_ge_a.value)         == 0,     "cmp_ge cleared"
    assert int(dut.read_cmp_eq_a.value)         == 0,     "cmp_eq cleared"
    assert int(dut.read_spike_flag_a.value)     == 0,     "spike cleared"


@cocotb.test()
async def test_graph_state_clear_priority_over_soft_reset(dut):
    """§6 Priority: graph_state_clear > soft_reset. When both asserted, validity is cleared
    and soft_reset write is blocked (graph_state_clear is in the top branch of if/else if)."""
    await _reset_dut(dut)

    # Pre-load neuron 2 via soft_reset
    await _soft_reset_write(dut, idx=2, init_rf=0x777)

    # Simultaneously assert graph_state_clear and soft_reset_valid
    dut.graph_state_clear.value = 1
    dut.soft_reset_valid.value  = 1
    dut.soft_reset_idx.value    = 2
    dut.init_rf_flat.value      = 0x999  # must NOT be written (else if blocked)

    await _tick(dut, 1)
    dut.graph_state_clear.value = 0
    dut.soft_reset_valid.value  = 0
    await _tick(dut, 2)

    # Entry 2 must now return DEFAULT_RF (ctx_valid_r[2] cleared by graph_state_clear)
    await _set_read_indices(dut, 2, 2)
    await _tick(dut, 1)
    _expect_default_port_a(dut, "neuron 2 after simultaneous gsc+soft_reset")


@cocotb.test()
async def test_graph_state_clear_priority_over_commit(dut):
    """§6 Priority: graph_state_clear > commit. When both asserted, validity is cleared
    and commit write is blocked."""
    await _reset_dut(dut)

    # Pre-load neuron 0 via commit
    await _commit_write(dut, idx=0, rf=0x555, tag=1, cmp_ge=1, cmp_eq=0, spike=1)

    # Simultaneously assert graph_state_clear and commit_valid
    dut.graph_state_clear.value    = 1
    dut.commit_valid.value         = 1
    dut.commit_idx.value           = 0
    dut.commit_rf_state_flat.value = 0xFFF  # must NOT be written
    dut.commit_cmp_ge.value        = 1
    dut.commit_cmp_eq.value        = 1
    dut.commit_spike_flag.value    = 1

    await _tick(dut, 1)
    dut.graph_state_clear.value = 0
    dut.commit_valid.value      = 0
    await _tick(dut, 2)

    await _set_read_indices(dut, 0, 0)
    await _tick(dut, 1)
    _expect_default_port_a(dut, "neuron 0 after simultaneous gsc+commit")


# ═══════════════════════════════════════════════════════════════════════════════
# §6  Write sequencer — graph_state_clear
# ═══════════════════════════════════════════════════════════════════════════════

@cocotb.test()
async def test_graph_state_clear_invalidates_all_neurons(dut):
    """§6 graph_state_clear sets ctx_valid_r=0 for ALL entries in a single cycle.
    All neurons return DEFAULT_RF after one clear pulse."""
    await _reset_dut(dut)

    # Write all neurons with distinct values
    for idx in range(NEURONS):
        await _commit_write(dut, idx=idx, rf=0x100 + idx * 0x111,
                            tag=1, cmp_ge=1, cmp_eq=1, spike=1)

    # Verify all are written
    for idx in range(NEURONS):
        await _set_read_indices(dut, idx, idx)
        await _tick(dut, 1)
        assert int(dut.read_rf_state_flat_a.value) != DEFAULT_RF, \
            f"neuron {idx} should be written before clear"

    # One pulse of graph_state_clear
    dut.graph_state_clear.value = 1
    await _tick(dut, 1)
    dut.graph_state_clear.value = 0
    await _tick(dut, 1)

    # All neurons must now return DEFAULT_RF
    for idx in range(NEURONS):
        await _set_read_indices(dut, idx, idx)
        await _tick(dut, 1)
        _expect_default_port_a(dut, f"neuron {idx} after graph_state_clear")
        _expect_default_port_b(dut, f"neuron {idx} B after graph_state_clear")


@cocotb.test()
async def test_graph_state_clear_single_cycle_sufficient(dut):
    """§6 A single cycle of graph_state_clear is sufficient to invalidate all entries.
    Second cycle is not required — tests combinatorial latching of validity."""
    await _reset_dut(dut)

    await _commit_write(dut, idx=0, rf=0xABC, tag=1, cmp_ge=0, cmp_eq=1, spike=0)
    await _commit_write(dut, idx=1, rf=0xDEF, tag=0, cmp_ge=1, cmp_eq=0, spike=1)

    # Exactly one clock with clear asserted
    dut.graph_state_clear.value = 1
    await _tick(dut, 1)
    dut.graph_state_clear.value = 0

    # Immediately check (no extra settling)
    await _tick(dut, 1)
    for idx in range(NEURONS):
        await _set_read_indices(dut, idx, idx)
        await _tick(dut, 1)
        _expect_default_port_a(dut, f"neuron {idx} one-cycle clear check")


@cocotb.test()
async def test_reinitialize_after_graph_state_clear(dut):
    """§6 After graph_state_clear, entries can be re-initialized via soft_reset or commit."""
    await _reset_dut(dut)

    # Write → clear → re-write via soft_reset
    await _commit_write(dut, idx=0, rf=0xAA0, tag=1, cmp_ge=1, cmp_eq=1, spike=1)

    dut.graph_state_clear.value = 1
    await _tick(dut, 1)
    dut.graph_state_clear.value = 0
    await _tick(dut, 1)

    # Re-initialize via soft_reset
    await _soft_reset_write(dut, idx=0, init_rf=0xBB0)
    await _set_read_indices(dut, 0, 0)
    await _tick(dut, 1)
    assert int(dut.read_rf_state_flat_a.value) == 0xBB0, "soft_reset after clear"
    assert int(dut.read_cmp_ge_a.value) == 0

    # Re-initialize a different neuron via commit
    await _commit_write(dut, idx=2, rf=0xCC0, tag=1, cmp_ge=1, cmp_eq=0, spike=1)
    await _set_read_indices(dut, 2, 2)
    await _tick(dut, 1)
    assert int(dut.read_rf_state_flat_a.value) == 0xCC0, "commit after clear"
    assert int(dut.read_spike_flag_a.value) == 1


@cocotb.test()
async def test_graph_state_clear_does_not_corrupt_ctx_r(dut):
    """§6 graph_state_clear only clears ctx_valid_r bits; ctx_r data is NOT erased.
    After clear + soft_reset to same index, the newly written value is correct."""
    await _reset_dut(dut)

    # Write, clear, then write a different RF value
    await _commit_write(dut, idx=1, rf=0x111, tag=0, cmp_ge=0, cmp_eq=0, spike=0)

    dut.graph_state_clear.value = 1
    await _tick(dut, 1)
    dut.graph_state_clear.value = 0
    await _tick(dut, 1)

    # Soft reset writes a fresh value — must come through cleanly
    await _soft_reset_write(dut, idx=1, init_rf=0x222)
    await _set_read_indices(dut, 1, 1)
    await _tick(dut, 1)
    assert int(dut.read_rf_state_flat_a.value) == 0x222
    assert int(dut.read_last_tag_a.value) == 0


# ═══════════════════════════════════════════════════════════════════════════════
# §6  Write sequencer — ena gating
# ═══════════════════════════════════════════════════════════════════════════════

@cocotb.test()
async def test_ena_blocks_commit(dut):
    """§6 When ena=0, commit_valid has no effect on ctx_r or ctx_valid_r.
    Entry remains in its prior state (DEFAULT_RF if not previously written)."""
    await _reset_dut(dut)

    dut.ena.value = 0
    await _commit_write(dut, idx=1, rf=0xCAB, tag=1, cmp_ge=1, cmp_eq=1, spike=1)

    await _set_read_indices(dut, 1, 1)
    await _tick(dut, 1)
    _expect_default_port_a(dut, "ena=0: commit must not write")
    assert int(dut.mem_stall.value) == 0


@cocotb.test()
async def test_ena_does_not_block_soft_reset(dut):
    """§6 ena gate applies only to commit. soft_reset_valid always writes regardless of ena."""
    await _reset_dut(dut)

    dut.ena.value = 0
    await _soft_reset_write(dut, idx=2, init_rf=0xFAC)

    await _set_read_indices(dut, 2, 2)
    await _tick(dut, 1)
    assert int(dut.read_rf_state_flat_a.value) == 0xFAC, "soft_reset ignores ena"
    assert int(dut.read_cmp_ge_a.value) == 0


@cocotb.test()
async def test_ena_blocks_commit_but_not_soft_reset_sequence(dut):
    """§6 ena=0 blocks commit but soft_reset still writes. Tests both in same test."""
    await _reset_dut(dut)

    dut.ena.value = 0
    # Commit should be blocked
    await _commit_write(dut, idx=0, rf=0xBAD, tag=1, cmp_ge=1, cmp_eq=1, spike=1)
    await _set_read_indices(dut, 0, 0)
    await _tick(dut, 1)
    _expect_default_port_a(dut, "commit blocked by ena=0")

    # Soft reset should still work
    await _soft_reset_write(dut, idx=0, init_rf=0xA00)
    await _set_read_indices(dut, 0, 0)
    await _tick(dut, 1)
    assert int(dut.read_rf_state_flat_a.value) == 0xA00, "soft_reset with ena=0"
    assert int(dut.read_last_tag_a.value)       == 0
    assert int(dut.read_cmp_ge_a.value)         == 0


# ═══════════════════════════════════════════════════════════════════════════════
# §7 / §8  Dual-port combinatorial read
# ═══════════════════════════════════════════════════════════════════════════════

@cocotb.test()
async def test_dual_port_same_index_returns_same_data(dut):
    """§7 When read_idx_a == read_idx_b, both ports return identical data."""
    await _reset_dut(dut)

    await _commit_write(dut, idx=2, rf=0x5A5, tag=1, cmp_ge=1, cmp_eq=0, spike=1)

    await _set_read_indices(dut, 2, 2)
    await _tick(dut, 1)

    assert int(dut.read_rf_state_flat_a.value) == int(dut.read_rf_state_flat_b.value)
    assert int(dut.read_last_tag_a.value)       == int(dut.read_last_tag_b.value)
    assert int(dut.read_cmp_ge_a.value)         == int(dut.read_cmp_ge_b.value)
    assert int(dut.read_cmp_eq_a.value)         == int(dut.read_cmp_eq_b.value)
    assert int(dut.read_spike_flag_a.value)     == int(dut.read_spike_flag_b.value)
    assert int(dut.mem_stall.value)             == 0


@cocotb.test()
async def test_dual_port_different_indices_simultaneous(dut):
    """§7 Port A and port B can address different neurons simultaneously with correct data."""
    await _reset_dut(dut)

    await _commit_write(dut, idx=0, rf=0x100, tag=0, cmp_ge=0, cmp_eq=1, spike=0)
    await _commit_write(dut, idx=3, rf=0xF0F, tag=1, cmp_ge=1, cmp_eq=0, spike=1)

    # Read neuron 0 on A and neuron 3 on B simultaneously
    await _set_read_indices(dut, 0, 3)
    await _tick(dut, 1)

    assert int(dut.read_rf_state_flat_a.value) == 0x100, "port A: neuron 0 rf"
    assert int(dut.read_cmp_eq_a.value)         == 1,     "port A: neuron 0 cmp_eq"
    assert int(dut.read_spike_flag_a.value)     == 0,     "port A: neuron 0 spike"

    assert int(dut.read_rf_state_flat_b.value) == 0xF0F, "port B: neuron 3 rf"
    assert int(dut.read_last_tag_b.value)       == 1,     "port B: neuron 3 tag"
    assert int(dut.read_cmp_ge_b.value)         == 1,     "port B: neuron 3 cmp_ge"
    assert int(dut.read_spike_flag_b.value)     == 1,     "port B: neuron 3 spike"

    assert int(dut.mem_stall.value) == 0


@cocotb.test()
async def test_dual_port_both_uninitialized_different_indices(dut):
    """§7 Both ports return DEFAULT_RF independently for different uninitialized indices."""
    await _reset_dut(dut)

    await _set_read_indices(dut, 1, 3)
    await _tick(dut, 1)

    _expect_default_port_a(dut, "port A uninitialized neuron 1")
    _expect_default_port_b(dut, "port B uninitialized neuron 3")
    assert int(dut.mem_stall.value) == 0


@cocotb.test()
async def test_dual_port_a_initialized_b_uninitialized(dut):
    """§7 Port A reads an initialized entry while port B reads an uninitialized one simultaneously."""
    await _reset_dut(dut)

    await _soft_reset_write(dut, idx=1, init_rf=0x9AB)

    # Port A → neuron 1 (initialized), port B → neuron 2 (not written)
    await _set_read_indices(dut, 1, 2)
    await _tick(dut, 1)

    assert int(dut.read_rf_state_flat_a.value) == 0x9AB, "port A: initialized"
    assert int(dut.read_last_tag_a.value)       == 0
    _expect_default_port_b(dut, "port B: uninitialized neuron 2")


@cocotb.test()
async def test_dual_port_combinatorial_zero_latency(dut):
    """§7 Reads are purely combinatorial — new index is reflected without an extra clock.
    Changing read_idx_a without a clock still updates the output combinatorially."""
    await _reset_dut(dut)

    await _commit_write(dut, idx=0, rf=0xA00, tag=0, cmp_ge=0, cmp_eq=0, spike=0)
    await _commit_write(dut, idx=1, rf=0xB00, tag=0, cmp_ge=0, cmp_eq=0, spike=0)

    # Point to neuron 0
    await _set_read_indices(dut, 0, 0)
    await _tick(dut, 1)
    assert int(dut.read_rf_state_flat_a.value) == 0xA00

    # Switch to neuron 1 and check ReadWrite phase (comb update without clock)
    dut.read_idx_a.value = 1
    await ReadWrite()
    assert int(dut.read_rf_state_flat_a.value) == 0xB00, "combinatorial update on index change"


@cocotb.test()
async def test_dual_port_full_sweep_all_pairs(dut):
    """§7 Full cross-product sweep: read all 4×4 (A, B) index pairs, verify correct data."""
    await _reset_dut(dut)

    rf_vals = [0x100, 0x200, 0x300, 0x400]
    tags    = [0, 1, 0, 1]
    for idx in range(NEURONS):
        await _commit_write(dut, idx=idx, rf=rf_vals[idx], tag=tags[idx],
                            cmp_ge=0, cmp_eq=0, spike=0)

    for ia in range(NEURONS):
        for ib in range(NEURONS):
            await _set_read_indices(dut, ia, ib)
            await _tick(dut, 1)
            assert int(dut.read_rf_state_flat_a.value) == rf_vals[ia], \
                f"A[{ia}]: rf mismatch"
            assert int(dut.read_last_tag_a.value)       == tags[ia], \
                f"A[{ia}]: tag mismatch"
            assert int(dut.read_rf_state_flat_b.value) == rf_vals[ib], \
                f"B[{ib}]: rf mismatch"
            assert int(dut.read_last_tag_b.value)       == tags[ib], \
                f"B[{ib}]: tag mismatch"
            assert int(dut.mem_stall.value) == 0


# ═══════════════════════════════════════════════════════════════════════════════
# §10  Timing model — no write-read bypass
# ═══════════════════════════════════════════════════════════════════════════════

@cocotb.test()
async def test_no_bypass_commit_same_cycle_reads_old_value(dut):
    """§10 No write-read bypass: when read_idx_a == commit_idx on the same cycle,
    the read port sees the OLD (pre-commit) value from that clock edge.
    The new value appears on the NEXT cycle after the write."""
    await _reset_dut(dut)

    # Pre-load neuron 1 with a known value
    await _soft_reset_write(dut, idx=1, init_rf=0x555)

    # Point read at neuron 1, verify the value is visible combinatorially
    dut.read_idx_a.value = 1
    dut.read_idx_b.value = 1
    await ReadWrite()
    assert int(dut.read_rf_state_flat_a.value) == 0x555, "pre-commit read sanity"

    # Now drive commit for neuron 1 with 0x999 — the write takes effect at the
    # NEXT rising edge; the combinatorial read should still show 0x555 (no bypass)
    dut.commit_valid.value         = 1
    dut.commit_idx.value           = 1
    dut.commit_rf_state_flat.value = 0x999
    dut.commit_last_tag.value      = 0
    dut.commit_cmp_ge.value        = 0
    dut.commit_cmp_eq.value        = 0
    dut.commit_spike_flag.value    = 0

    # Settle combinatorial logic BEFORE clocking — FF still holds 0x555
    await ReadWrite()

    old_rf = int(dut.read_rf_state_flat_a.value)
    assert old_rf == 0x555, \
        f"§10 bypass check: expected old value 0x555, got {old_rf:#x}"

    # Clock — FF gets written with 0x999
    await RisingEdge(dut.clk)
    dut.commit_valid.value = 0
    await ReadWrite()

    new_rf = int(dut.read_rf_state_flat_a.value)
    assert new_rf == 0x999, \
        f"§10 next-cycle check: expected 0x999, got {new_rf:#x}"


@cocotb.test()
async def test_no_bypass_soft_reset_same_cycle_reads_old_value(dut):
    """§10 No write-read bypass: when read_idx_a == soft_reset_idx on the same cycle,
    read port sees the OLD value. New value is available on the following cycle."""
    await _reset_dut(dut)

    # Pre-load neuron 0 with a known commit
    await _commit_write(dut, idx=0, rf=0xAA0, tag=1, cmp_ge=1, cmp_eq=0, spike=0)

    # Point read at neuron 0, verify current value
    dut.read_idx_a.value = 0
    dut.read_idx_b.value = 0
    await ReadWrite()
    assert int(dut.read_rf_state_flat_a.value) == 0xAA0, "pre-soft_reset read sanity"

    # Drive soft_reset for neuron 0 with 0x110 — write takes effect at NEXT posedge
    dut.soft_reset_valid.value = 1
    dut.soft_reset_idx.value   = 0
    dut.init_rf_flat.value     = 0x110

    # Settle comb BEFORE clocking — FF still holds 0xAA0
    await ReadWrite()

    old_rf = int(dut.read_rf_state_flat_a.value)
    assert old_rf == 0xAA0, \
        f"§10 soft_reset bypass check: expected old 0xAA0, got {old_rf:#x}"

    # Clock — FF gets written with 0x110
    await RisingEdge(dut.clk)
    dut.soft_reset_valid.value = 0
    await ReadWrite()

    new_rf = int(dut.read_rf_state_flat_a.value)
    assert new_rf == 0x110, \
        f"§10 soft_reset next-cycle: expected 0x110, got {new_rf:#x}"


# ═══════════════════════════════════════════════════════════════════════════════
# §7  Validity tracking — uninitialized entry returns DEFAULT_RF
# ═══════════════════════════════════════════════════════════════════════════════

@cocotb.test()
async def test_validity_soft_reset_marks_entry_valid(dut):
    """§7 After soft_reset, ctx_valid_r[idx]=1: entry no longer returns DEFAULT_RF."""
    await _reset_dut(dut)

    # Uninitialized
    await _set_read_indices(dut, 2, 2)
    await _tick(dut, 1)
    _expect_default_port_a(dut, "before soft_reset")

    # Initialize via soft_reset with rf=0 (distinct from threshold=7 in DEFAULT_RF)
    await _soft_reset_write(dut, idx=2, init_rf=0x000)
    await _set_read_indices(dut, 2, 2)
    await _tick(dut, 1)

    # rf_state_flat should now be 0x000, NOT DEFAULT_RF=0x700
    assert int(dut.read_rf_state_flat_a.value) == 0x000, \
        "soft_reset with rf=0 makes entry valid; should return 0, not DEFAULT_RF"


@cocotb.test()
async def test_validity_graph_state_clear_makes_all_entries_invalid(dut):
    """§7 graph_state_clear resets ctx_valid_r to all-zeros; all entries return DEFAULT_RF."""
    await _reset_dut(dut)

    # Initialize neurons 0 and 3 with non-default RF
    await _commit_write(dut, idx=0, rf=0x001, tag=0, cmp_ge=0, cmp_eq=0, spike=0)
    await _commit_write(dut, idx=3, rf=0x003, tag=0, cmp_ge=0, cmp_eq=0, spike=0)

    dut.graph_state_clear.value = 1
    await _tick(dut, 1)
    dut.graph_state_clear.value = 0
    await _tick(dut, 1)

    for idx in range(NEURONS):
        await _set_read_indices(dut, idx, idx)
        await _tick(dut, 1)
        _expect_default_port_a(dut, f"validity clear neuron {idx}")


@cocotb.test()
async def test_validity_selective_initialization(dut):
    """§7 Only initialized entries differ from DEFAULT_RF; others remain at DEFAULT_RF."""
    await _reset_dut(dut)

    # Initialize neurons 1 and 2 only
    await _commit_write(dut, idx=1, rf=0x111, tag=0, cmp_ge=0, cmp_eq=0, spike=0)
    await _commit_write(dut, idx=2, rf=0x222, tag=0, cmp_ge=0, cmp_eq=0, spike=0)

    for idx in range(NEURONS):
        await _set_read_indices(dut, idx, idx)
        await _tick(dut, 1)
        if idx == 1:
            assert int(dut.read_rf_state_flat_a.value) == 0x111, "neuron 1 initialized"
        elif idx == 2:
            assert int(dut.read_rf_state_flat_a.value) == 0x222, "neuron 2 initialized"
        else:
            _expect_default_port_a(dut, f"neuron {idx} not initialized")


# ═══════════════════════════════════════════════════════════════════════════════
# §2  Area / correctness properties
# ═══════════════════════════════════════════════════════════════════════════════

@cocotb.test()
async def test_default_rf_encodes_threshold_7(dut):
    """§2 DEFAULT_RF = 0x700 = {rf[2]=7=threshold, rf[1]=0=accum, rf[0]=0=vm}.
    Uninitialized neurons have threshold=7, accumulator=0, membrane=0."""
    await _reset_dut(dut)

    for idx in range(NEURONS):
        await _set_read_indices(dut, idx, idx)
        await _tick(dut, 1)
        rf = int(dut.read_rf_state_flat_a.value)
        threshold = (rf >> 8) & 0xF   # bits [11:8]
        accum     = (rf >> 4) & 0xF   # bits [7:4]
        vm        = (rf >> 0) & 0xF   # bits [3:0]
        assert threshold == 7, f"neuron {idx}: threshold={threshold}, expected 7"
        assert accum     == 0, f"neuron {idx}: accum={accum}, expected 0"
        assert vm        == 0, f"neuron {idx}: vm={vm}, expected 0"

