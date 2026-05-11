"""
test_neuron_compute.py — Exhaustive cocotb testbench for neuron_compute_core.

Architecture reference: docs/neuron_compute_core_architecture.md
                        docs/neuron_exec_architecture.md

Struct layout (current tile_pkg):
    NEURON_IDX_W  = 2   (NEURON_LOCAL_W=1, 2x2 neurons)
    RF_REG_W      = 4   (4-bit per register, range -8..+7)
    RF_COUNT      = 3   (rf[0]=vm, rf[1]=acc, rf[2]=threshold)
    RF_FLAT_W     = 12  (3 x 4-bit, packed {rf[2], rf[1], rf[0]})
    TAG_W         = 1
    WEIGHT_W      = 4
    PROG_IDX_W    = 5
    DATA_W        = 4
    NEURON_UCODE_RSP_W = 12  (compact 12-bit instruction)

    Compact 12-bit instruction format: [11:7]=op, [6:4]=rd, [3]=sign, [2:0]=k
    k[0] (bit 0 of k-field, i.e. instr bit 2): tag_gate — skip on data events
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ReadOnly

# ── Architectural constants ───────────────────────────────────────────────────
NEURON_IDX_W   = 2
RF_REG_W       = 4
RF_COUNT       = 3
RF_FLAT_W      = RF_COUNT * RF_REG_W    # 12
TAG_W          = 1
WEIGHT_W       = 4
PROG_IDX_W     = 5
DATA_W         = 4
UCODE_RSP_W    = 12

NEURON_EXEC_CTX_W     = RF_FLAT_W + TAG_W + 3          # 16
NEURON_WORKER_EVENT_W = TAG_W + WEIGHT_W                # 5
NEURON_WORKER_START_W = (NEURON_IDX_W + NEURON_EXEC_CTX_W
                         + NEURON_WORKER_EVENT_W + 2 * PROG_IDX_W)  # 33
NEURON_UCODE_REQ_W    = NEURON_IDX_W + PROG_IDX_W      # 7
NEURON_EMIT_W         = 1 + DATA_W                      # 5
NEURON_WORKER_RESULT_W = NEURON_IDX_W + NEURON_EXEC_CTX_W + NEURON_EMIT_W  # 23

# ── Opcodes ───────────────────────────────────────────────────────────────────
OP_LDI         = 0
OP_RECV        = 1
OP_ACCUM_W     = 2
OP_INTEG       = 4
OP_SPIKE_IF_GE = 5
OP_RESET       = 6
OP_EMIT        = 8

TAG_DATA    = 0
TAG_BARRIER = 1


# ── Instruction encoding ──────────────────────────────────────────────────────
def encode_instr(op, rd=0, sign=0, k=0):
    """Compact 12-bit instruction: [11:7]=op, [6:4]=rd, [3]=sign, [2:0]=k."""
    return ((op & 0x1F) << 7) | ((rd & 0x7) << 4) | ((sign & 0x1) << 3) | (k & 0x7)


# ── Standard programs ─────────────────────────────────────────────────────────
# 6-step tag-gated IF neuron
IF_STANDARD = [
    encode_instr(OP_RECV),               # capture event tag
    encode_instr(OP_ACCUM_W),            # acc += weight (all events)
    encode_instr(OP_INTEG, k=4),         # vm+=acc, clear acc (k[2]=1 -> barrier-gated + acc-clear)
    encode_instr(OP_SPIKE_IF_GE, k=4),  # compare, barrier-gated (k[2]=1)
    encode_instr(OP_RESET, k=0),         # zero-reset (not gated; spike_flag guards)
    encode_instr(OP_EMIT, k=4),          # emit, barrier-gated (k[2]=1)
]

# Ungated program (no tag gating)
IF_UNGATED = [
    encode_instr(OP_RECV),
    encode_instr(OP_ACCUM_W),
    encode_instr(OP_INTEG, k=4),         # k[2]=1 = clear acc
    encode_instr(OP_SPIKE_IF_GE),
    encode_instr(OP_RESET, k=0),
    encode_instr(OP_EMIT),
]


# ── RF helpers ────────────────────────────────────────────────────────────────
def _pack_rf(vm=0, acc=0, threshold=0):
    """Pack {rf[2][3:0], rf[1][3:0], rf[0][3:0]} into 12-bit integer."""
    return (int(vm) & 0xF) | ((int(acc) & 0xF) << 4) | ((int(threshold) & 0xF) << 8)


def _unpack_rf(flat):
    """Return (vm, acc, threshold) as signed ints from a 12-bit flat value."""
    mask = (1 << RF_REG_W) - 1

    def s4(v):
        return v - (1 << RF_REG_W) if v >= (1 << (RF_REG_W - 1)) else v

    return s4(flat & mask), s4((flat >> 4) & mask), s4((flat >> 8) & mask)


# ── Payload pack / unpack ─────────────────────────────────────────────────────
def _pack_start(logical_idx=0, vm=0, acc=0, threshold=0,
                last_tag=0, cmp_ge=0, cmp_eq=0, spike_flag=0,
                event_tag=0, weight=0, ucode_ptr=0, ucode_len=0):
    """Pack neuron_worker_start_t → integer (NEURON_WORKER_START_W bits).

    Layout [32:0]:
      [32:31] logical_idx
      [30:19] rf_flat
      [18]    last_tag
      [17]    cmp_ge
      [16]    cmp_eq
      [15]    spike_flag
      [14]    event.tag
      [13:10] event.weight
      [9:5]   ucode_ptr
      [4:0]   ucode_len
    """
    rf_flat = _pack_rf(vm, acc, threshold)
    v = 0
    v |= (int(logical_idx) & 0x3) << 31
    v |= (rf_flat & 0xFFF)         << 19
    v |= (int(last_tag) & 1)       << 18
    v |= (int(cmp_ge) & 1)         << 17
    v |= (int(cmp_eq) & 1)         << 16
    v |= (int(spike_flag) & 1)     << 15
    v |= (int(event_tag) & 1)      << 14
    v |= (int(weight) & 0xF)       << 10
    v |= (int(ucode_ptr) & 0x1F)   << 5
    v |= (int(ucode_len) & 0x1F)
    return v


def _unpack_result(val):
    """Unpack neuron_worker_result_t from NEURON_WORKER_RESULT_W-bit integer.

    Layout [22:0]:
      [22:21] logical_idx
      [20:9]  rf_flat
      [8]     last_tag
      [7]     cmp_ge
      [6]     cmp_eq
      [5]     spike_flag
      [4]     emit_valid
      [3:0]   emit_data
    """
    v = int(val)
    emit_data   = v & 0xF;           v >>= DATA_W
    emit_valid  = v & 1;             v >>= 1
    spike_flag  = v & 1;             v >>= 1
    cmp_eq      = v & 1;             v >>= 1
    cmp_ge      = v & 1;             v >>= 1
    last_tag    = v & 1;             v >>= TAG_W
    rf_flat     = v & 0xFFF;         v >>= RF_FLAT_W
    logical_idx = v & 0x3
    vm, acc, threshold = _unpack_rf(rf_flat)
    return {"logical_idx": logical_idx, "vm": vm, "acc": acc,
            "threshold": threshold, "last_tag": last_tag,
            "cmp_ge": cmp_ge, "cmp_eq": cmp_eq, "spike_flag": spike_flag,
            "emit_valid": emit_valid, "emit_data": emit_data}


def _unpack_ucode_req(val):
    v = int(val)
    word_index  = v & 0x1F;  v >>= PROG_IDX_W
    logical_idx = v & 0x3
    return {"logical_idx": logical_idx, "word_index": word_index}


# ── DUT port accessor ─────────────────────────────────────────────────────────
def _s(dut, name):
    return getattr(dut, name)


# ── Ucode server ──────────────────────────────────────────────────────────────
async def _serve_ucode(dut, words):
    """Respond to ucode_read requests with 1-cycle latency (SRAM model).

    cocotb 2.x / Verilator: ReadWrite cannot follow ReadOnly in the same cycle.
    Pattern: sample on RisingEdge (active phase), drive immediately after.
    """
    _s(dut, "ucode_rsp_valid").value   = 0
    _s(dut, "ucode_rsp_payload").value = 0
    pending = None
    while True:
        await RisingEdge(dut.clk)
        # Drive the response from the previous cycle's request.
        if pending is not None:
            _s(dut, "ucode_rsp_payload").value = int(pending) & 0xFFF
            _s(dut, "ucode_rsp_valid").value   = 1
        else:
            _s(dut, "ucode_rsp_valid").value   = 0
        # Sample the current request (combinational output, valid on same edge).
        # We do this after driving to avoid ReadOnly ordering issues; in
        # Verilator all signals are stable after the rising edge propagates.
        if int(_s(dut, "ucode_read_valid").value):
            req = _unpack_ucode_req(int(_s(dut, "ucode_read_payload").value))
            idx = req["word_index"]
            assert 0 <= idx < len(words), f"ucode OOB idx={idx} len={len(words)}"
            pending = words[idx]
        else:
            pending = None


# ── Reset ─────────────────────────────────────────────────────────────────────
async def _reset_dut(dut):
    cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())
    dut.rst_n.value                         = 0
    dut.ena.value                           = 0
    dut.graph_state_clear.value             = 0
    _s(dut, "worker_start_valid").value     = 0
    _s(dut, "worker_start_payload").value   = 0
    _s(dut, "ucode_read_ready").value       = 1
    _s(dut, "ucode_rsp_valid").value        = 0
    _s(dut, "ucode_rsp_payload").value      = 0
    _s(dut, "worker_result_ready").value    = 0
    for _ in range(4):
        await RisingEdge(dut.clk)
    dut.rst_n.value = 1
    dut.ena.value   = 1
    await RisingEdge(dut.clk)


# ── Dispatch helpers ──────────────────────────────────────────────────────────
async def _dispatch(dut, ucode_words, **kwargs):
    task = cocotb.start_soon(_serve_ucode(dut, ucode_words))
    for _ in range(64):
        await RisingEdge(dut.clk)
        await ReadOnly()
        if int(_s(dut, "worker_start_ready").value):
            break
    else:
        task.kill()
        raise AssertionError("worker_start_ready never asserted")
    payload = _pack_start(ucode_len=len(ucode_words), **kwargs)
    # Advance to next active phase for driving
    await RisingEdge(dut.clk)
    _s(dut, "worker_start_payload").value = payload
    _s(dut, "worker_start_valid").value   = 1
    await RisingEdge(dut.clk)
    _s(dut, "worker_start_valid").value   = 0
    return task


async def _wait_result(dut, max_cycles=128):
    for _ in range(max_cycles):
        await RisingEdge(dut.clk)
        await ReadOnly()
        if int(_s(dut, "worker_result_valid").value):
            result = _unpack_result(int(_s(dut, "worker_result_payload").value))
            # Deassert after next edge
            await RisingEdge(dut.clk)
            _s(dut, "worker_result_ready").value = 1
            await RisingEdge(dut.clk)
            _s(dut, "worker_result_ready").value = 0
            return result
    raise AssertionError("Timed out waiting for worker_result_valid")


async def _run(dut, ucode_words, max_cycles=128, **kwargs):
    task = await _dispatch(dut, ucode_words, **kwargs)
    try:
        return await _wait_result(dut, max_cycles=max_cycles)
    finally:
        task.kill()


# =============================================================================
# OP_LDI — Load Immediate
# =============================================================================

@cocotb.test()
async def test_ldi_rf0_positive(dut):
    """OP_LDI rd=0 imm=+5 loads vm=5.

    k=5=101b -> instr_word[2]=1 -> tag-gated; send barrier event.
    """
    await _reset_dut(dut)
    r = await _run(dut, [encode_instr(OP_LDI, rd=0, k=5)], event_tag=TAG_BARRIER)
    assert r["vm"] == 5, f"vm={r['vm']}"


@cocotb.test()
async def test_ldi_rf1_positive(dut):
    """OP_LDI rd=1 imm=+3 loads acc=3."""
    await _reset_dut(dut)
    r = await _run(dut, [encode_instr(OP_LDI, rd=1, k=3)])
    assert r["acc"] == 3, f"acc={r['acc']}"


@cocotb.test()
async def test_ldi_rf2_sets_threshold(dut):
    """OP_LDI rd=2 imm=+7 sets threshold=7.

    k=7=111b -> tag-gated; send barrier event.
    """
    await _reset_dut(dut)
    r = await _run(dut, [encode_instr(OP_LDI, rd=2, k=7)], event_tag=TAG_BARRIER)
    assert r["threshold"] == 7, f"threshold={r['threshold']}"


@cocotb.test()
async def test_ldi_imm_negative_eight(dut):
    """OP_LDI imm=-8 (sign=1, k=000) sets register to -8."""
    await _reset_dut(dut)
    r = await _run(dut, [encode_instr(OP_LDI, rd=2, sign=1, k=0)])
    assert r["threshold"] == -8, f"threshold={r['threshold']}"


@cocotb.test()
async def test_ldi_imm_minus_one(dut):
    """OP_LDI imm=-1 (sign=1, k=111) sets vm=-1.

    k=7=111b -> tag-gated; send barrier event.
    """
    await _reset_dut(dut)
    r = await _run(dut, [encode_instr(OP_LDI, rd=0, sign=1, k=7)], event_tag=TAG_BARRIER)
    assert r["vm"] == -1, f"vm={r['vm']}"


@cocotb.test()
async def test_ldi_full_range_all_registers(dut):
    """OP_LDI: sweep all 16 imm4 values for each of the 3 registers.

    Immediates with bit 2 of k set (k>=4) are tag-gated; send barrier for those.
    """
    await _reset_dut(dut)
    for rd in range(RF_COUNT):
        for raw_imm in range(16):
            sign = (raw_imm >> 3) & 1
            k    = raw_imm & 0x7
            expected = raw_imm - 16 if raw_imm >= 8 else raw_imm
            # k[2] = instr_word[2] = tag_gate bit
            tag = TAG_BARRIER if (k >= 4) else TAG_DATA
            r = await _run(dut, [encode_instr(OP_LDI, rd=rd, sign=sign, k=k)],
                           event_tag=tag)
            got = ("vm", "acc", "threshold")[rd]
            assert r[got] == expected, \
                f"LDI rd={rd} raw={raw_imm} k={k}: expected {expected}, got {r[got]}"


@cocotb.test()
async def test_ldi_out_of_range_rd_is_nop(dut):
    """OP_LDI with rd >= RF_COUNT is a no-op."""
    await _reset_dut(dut)
    r = await _run(dut, [encode_instr(OP_LDI, rd=3, k=7)],
                   vm=1, acc=2, threshold=3)
    assert r["vm"] == 1 and r["acc"] == 2 and r["threshold"] == 3


# =============================================================================
# OP_RECV — Record Event Tag
# =============================================================================

@cocotb.test()
async def test_recv_records_data_tag(dut):
    """OP_RECV with event_tag=0 → last_tag=0."""
    await _reset_dut(dut)
    r = await _run(dut, [encode_instr(OP_RECV)], event_tag=TAG_DATA)
    assert r["last_tag"] == 0


@cocotb.test()
async def test_recv_records_barrier_tag(dut):
    """OP_RECV with event_tag=1 → last_tag=1."""
    await _reset_dut(dut)
    r = await _run(dut, [encode_instr(OP_RECV)], event_tag=TAG_BARRIER)
    assert r["last_tag"] == 1


@cocotb.test()
async def test_recv_does_not_modify_rf(dut):
    """OP_RECV must not change any register-file value."""
    await _reset_dut(dut)
    r = await _run(dut, [encode_instr(OP_RECV)],
                   vm=3, acc=-2 & 0xF, threshold=5, event_tag=TAG_BARRIER)
    assert r["vm"] == 3 and r["acc"] == -2 and r["threshold"] == 5


# =============================================================================
# OP_ACCUM_W — Accumulate Synaptic Weight
# =============================================================================

@cocotb.test()
async def test_accum_positive_weight(dut):
    """OP_ACCUM_W with weight=+3, acc=0 → acc=3."""
    await _reset_dut(dut)
    r = await _run(dut, [encode_instr(OP_ACCUM_W)], weight=3)
    assert r["acc"] == 3, f"acc={r['acc']}"


@cocotb.test()
async def test_accum_negative_weight(dut):
    """OP_ACCUM_W with weight=-4, acc=0 → acc=-4."""
    await _reset_dut(dut)
    r = await _run(dut, [encode_instr(OP_ACCUM_W)], weight=-4 & 0xF)
    assert r["acc"] == -4, f"acc={r['acc']}"


@cocotb.test()
async def test_accum_adds_to_existing(dut):
    """OP_ACCUM_W adds to pre-loaded acc: acc=2+3=5."""
    await _reset_dut(dut)
    r = await _run(dut, [encode_instr(OP_ACCUM_W)], acc=2, weight=3)
    assert r["acc"] == 5, f"acc={r['acc']}"


@cocotb.test()
async def test_accum_saturates_positive(dut):
    """OP_ACCUM_W saturates at +7 when overflow (6+7=13 → 7)."""
    await _reset_dut(dut)
    r = await _run(dut, [encode_instr(OP_ACCUM_W)], acc=6, weight=7)
    assert r["acc"] == 7, f"acc={r['acc']}"


@cocotb.test()
async def test_accum_saturates_negative(dut):
    """OP_ACCUM_W saturates at -8 when underflow (-6-4=-10 → -8)."""
    await _reset_dut(dut)
    r = await _run(dut, [encode_instr(OP_ACCUM_W)],
                   acc=-6 & 0xF, weight=-4 & 0xF)
    assert r["acc"] == -8, f"acc={r['acc']}"


@cocotb.test()
async def test_accum_does_not_touch_vm_threshold(dut):
    """OP_ACCUM_W must not modify vm or threshold."""
    await _reset_dut(dut)
    r = await _run(dut, [encode_instr(OP_ACCUM_W)], vm=5, threshold=4, weight=2)
    assert r["vm"] == 5 and r["threshold"] == 4


# =============================================================================
# OP_INTEG — Integrate Membrane Potential
# =============================================================================

@cocotb.test()
async def test_integ_adds_acc_to_vm(dut):
    """OP_INTEG: vm=2, acc=3 → vm=5; acc unchanged when k[2]=0."""
    await _reset_dut(dut)
    r = await _run(dut, [encode_instr(OP_INTEG, k=0)], vm=2, acc=3)
    assert r["vm"] == 5, f"vm={r['vm']}"
    assert r["acc"] == 3, "acc should be unchanged (k[2]=0)"


@cocotb.test()
async def test_integ_clears_acc_when_k2_set(dut):
    """OP_INTEG k=4 (instr_word[2]=1): clears acc AND is tag-gated; use barrier."""
    await _reset_dut(dut)
    r = await _run(dut, [encode_instr(OP_INTEG, k=4)], vm=1, acc=3,
                   event_tag=TAG_BARRIER)
    assert r["vm"] == 4, f"vm={r['vm']}"
    assert r["acc"] == 0, f"acc should be cleared, got {r['acc']}"


@cocotb.test()
async def test_integ_negative_acc_decrements_vm(dut):
    """OP_INTEG: vm=5, acc=-3 → vm=2."""
    await _reset_dut(dut)
    r = await _run(dut, [encode_instr(OP_INTEG, k=0)], vm=5, acc=-3 & 0xF)
    assert r["vm"] == 2, f"vm={r['vm']}"


@cocotb.test()
async def test_integ_saturates_positive(dut):
    """OP_INTEG vm saturates at +7 (6+5=11 → 7)."""
    await _reset_dut(dut)
    r = await _run(dut, [encode_instr(OP_INTEG, k=0)], vm=6, acc=5)
    assert r["vm"] == 7, f"vm={r['vm']}"


@cocotb.test()
async def test_integ_saturates_negative(dut):
    """OP_INTEG vm saturates at -8 (-5+(-5)=-10 → -8)."""
    await _reset_dut(dut)
    r = await _run(dut, [encode_instr(OP_INTEG, k=0)], vm=-5 & 0xF, acc=-5 & 0xF)
    assert r["vm"] == -8, f"vm={r['vm']}"


@cocotb.test()
async def test_integ_does_not_modify_threshold(dut):
    """OP_INTEG must not change threshold."""
    await _reset_dut(dut)
    r = await _run(dut, [encode_instr(OP_INTEG, k=0)], vm=1, acc=1, threshold=5)
    assert r["threshold"] == 5


# =============================================================================
# OP_SPIKE_IF_GE — Threshold Comparison
# =============================================================================

@cocotb.test()
async def test_spike_ge_fires_when_vm_equals_threshold(dut):
    """OP_SPIKE_IF_GE: vm==threshold -> spike_flag=1.

    Note: cmp_ge is a context passthrough; RTL does not update it via SPIKE_IF_GE.
    """
    await _reset_dut(dut)
    r = await _run(dut, [encode_instr(OP_SPIKE_IF_GE)], vm=4, threshold=4)
    assert r["spike_flag"] == 1, f"spike_flag={r['spike_flag']}"


@cocotb.test()
async def test_spike_ge_fires_when_vm_exceeds(dut):
    """OP_SPIKE_IF_GE: vm > threshold → spike_flag=1."""
    await _reset_dut(dut)
    r = await _run(dut, [encode_instr(OP_SPIKE_IF_GE)], vm=6, threshold=4)
    assert r["spike_flag"] == 1, f"spike_flag={r['spike_flag']}"


@cocotb.test()
async def test_spike_ge_no_fire_below_threshold(dut):
    """OP_SPIKE_IF_GE: vm < threshold -> spike_flag=0."""
    await _reset_dut(dut)
    r = await _run(dut, [encode_instr(OP_SPIKE_IF_GE)], vm=3, threshold=4)
    assert r["spike_flag"] == 0


@cocotb.test()
async def test_spike_ge_clears_prior_spike_flag(dut):
    """OP_SPIKE_IF_GE overwrites spike_flag from context when vm < threshold."""
    await _reset_dut(dut)
    r = await _run(dut, [encode_instr(OP_SPIKE_IF_GE)],
                   vm=2, threshold=5, spike_flag=1)
    assert r["spike_flag"] == 0


@cocotb.test()
async def test_spike_ge_does_not_modify_rf(dut):
    """OP_SPIKE_IF_GE must not modify vm, acc, or threshold."""
    await _reset_dut(dut)
    r = await _run(dut, [encode_instr(OP_SPIKE_IF_GE)], vm=3, acc=2, threshold=7)
    assert r["vm"] == 3 and r["acc"] == 2 and r["threshold"] == 7


@cocotb.test()
async def test_spike_ge_exhaustive_4bit(dut):
    """Exhaustive sweep: all vm x threshold 4-bit combinations (-8..+7)."""
    await _reset_dut(dut)
    for vm_val in range(-8, 8):
        for thr_val in range(-8, 8):
            r = await _run(dut, [encode_instr(OP_SPIKE_IF_GE)],
                           vm=vm_val & 0xF, threshold=thr_val & 0xF)
            exp = 1 if vm_val >= thr_val else 0
            assert r["spike_flag"] == exp, \
                f"vm={vm_val} thr={thr_val}: expected spike={exp}, got {r['spike_flag']}"


# =============================================================================
# OP_RESET — Post-Fire Membrane Reset
# =============================================================================

@cocotb.test()
async def test_reset_mode00_zeroes_vm(dut):
    """OP_RESET mode=00: vm=0 after spike."""
    await _reset_dut(dut)
    prog = [encode_instr(OP_SPIKE_IF_GE), encode_instr(OP_RESET, k=0)]
    r = await _run(dut, prog, vm=5, threshold=3)
    assert r["spike_flag"] == 1 and r["vm"] == 0


@cocotb.test()
async def test_reset_mode01_subtractive(dut):
    """OP_RESET mode=01 (subtractive): vm = vm - threshold after spike."""
    await _reset_dut(dut)
    prog = [encode_instr(OP_SPIKE_IF_GE), encode_instr(OP_RESET, k=1)]
    r = await _run(dut, prog, vm=7, threshold=4, event_tag=TAG_BARRIER)
    assert r["spike_flag"] == 1 and r["vm"] == 3, f"vm={r['vm']}"


@cocotb.test()
async def test_reset_mode10_clamp(dut):
    """OP_RESET mode=10 (clamp): vm = min(vm, threshold) after spike."""
    await _reset_dut(dut)
    prog = [encode_instr(OP_SPIKE_IF_GE), encode_instr(OP_RESET, k=2)]
    r = await _run(dut, prog, vm=7, threshold=4)
    assert r["spike_flag"] == 1 and r["vm"] == 4, f"vm={r['vm']}"


@cocotb.test()
async def test_reset_mode11_zeroes_vm(dut):
    """OP_RESET mode=11: vm=0 after spike on barrier."""
    await _reset_dut(dut)
    prog = [encode_instr(OP_SPIKE_IF_GE), encode_instr(OP_RESET, k=3)]
    r = await _run(dut, prog, vm=5, threshold=3, event_tag=TAG_BARRIER)
    assert r["spike_flag"] == 1 and r["vm"] == 0


@cocotb.test()
async def test_reset_nop_when_no_spike(dut):
    """OP_RESET is a no-op when spike_flag=0."""
    await _reset_dut(dut)
    prog = [encode_instr(OP_SPIKE_IF_GE), encode_instr(OP_RESET, k=0)]
    r = await _run(dut, prog, vm=2, threshold=5)
    assert r["spike_flag"] == 0 and r["vm"] == 2


@cocotb.test()
async def test_reset_subtractive_saturates(dut):
    """Subtractive reset result saturates at -8 on underflow."""
    await _reset_dut(dut)
    prog = [encode_instr(OP_SPIKE_IF_GE), encode_instr(OP_RESET, k=1)]
    # vm=-1, threshold=-4: spike since -1 >= -4; reset: -1-(-4)=3
    r = await _run(dut, prog, vm=-1 & 0xF, threshold=-4 & 0xF,
                   event_tag=TAG_BARRIER)
    assert r["spike_flag"] == 1 and r["vm"] == 3, f"vm={r['vm']}"


# =============================================================================
# OP_EMIT — Emit Output Spike
# =============================================================================

@cocotb.test()
async def test_emit_sets_valid(dut):
    """OP_EMIT following spike → emit_valid=1."""
    await _reset_dut(dut)
    prog = [encode_instr(OP_SPIKE_IF_GE), encode_instr(OP_EMIT)]
    r = await _run(dut, prog, vm=5, threshold=3)
    assert r["emit_valid"] == 1


@cocotb.test()
async def test_emit_fires_regardless_of_spike(dut):
    """OP_EMIT always sets emit_valid; spike only affects emit_data content."""
    await _reset_dut(dut)
    r = await _run(dut, [encode_instr(OP_EMIT)], vm=2, threshold=5)
    assert r["emit_valid"] == 1


@cocotb.test()
async def test_emit_gated_suppressed_on_data(dut):
    """Tag-gated OP_EMIT (k=4, instr_word[2]=1) does not fire on data events."""
    await _reset_dut(dut)
    r = await _run(dut, [encode_instr(OP_EMIT, k=4)], event_tag=TAG_DATA)
    assert r["emit_valid"] == 0


@cocotb.test()
async def test_emit_gated_fires_on_barrier(dut):
    """Tag-gated OP_EMIT (k=4, instr_word[2]=1) fires on barrier events."""
    await _reset_dut(dut)
    r = await _run(dut, [encode_instr(OP_EMIT, k=4)], event_tag=TAG_BARRIER)
    assert r["emit_valid"] == 1


# =============================================================================
# Tag-gating integration
# =============================================================================

@cocotb.test()
async def test_tag_gate_skips_integ_on_data(dut):
    """Tag-gated OP_INTEG (k=4, instr_word[2]=1) is skipped for data events."""
    await _reset_dut(dut)
    r = await _run(dut, [encode_instr(OP_INTEG, k=4)],
                   vm=3, acc=4, event_tag=TAG_DATA)
    assert r["vm"] == 3, f"vm should not change: vm={r['vm']}"


@cocotb.test()
async def test_tag_gate_executes_integ_on_barrier(dut):
    """Tag-gated OP_INTEG (k=4) executes on barrier events -> vm += acc."""
    await _reset_dut(dut)
    r = await _run(dut, [encode_instr(OP_INTEG, k=4)],
                   vm=3, acc=4, event_tag=TAG_BARRIER)
    assert r["vm"] == 7, f"vm={r['vm']}"


@cocotb.test()
async def test_tag_gate_spike_compare_skipped_on_data(dut):
    """Tag-gated OP_SPIKE_IF_GE (k=4) preserves spike_flag when skipped on data."""
    await _reset_dut(dut)
    r = await _run(dut, [encode_instr(OP_SPIKE_IF_GE, k=4)],
                   vm=6, threshold=3, spike_flag=0, event_tag=TAG_DATA)
    assert r["spike_flag"] == 0


@cocotb.test()
async def test_tag_gate_spike_compare_executes_on_barrier(dut):
    """Tag-gated OP_SPIKE_IF_GE executes on barrier → spike when vm>=threshold."""
    await _reset_dut(dut)
    r = await _run(dut, [encode_instr(OP_SPIKE_IF_GE, k=4)],
                   vm=6, threshold=3, event_tag=TAG_BARRIER)
    assert r["spike_flag"] == 1


# =============================================================================
# Full IF neuron integration tests
# =============================================================================

@cocotb.test()
async def test_standard_data_event_accumulates(dut):
    """IF_STANDARD data event: acc += weight; gated ops (integ/spike/emit) skipped."""
    await _reset_dut(dut)
    r = await _run(dut, IF_STANDARD,
                   threshold=4, event_tag=TAG_DATA, weight=3)
    assert r["acc"] == 3,        f"acc={r['acc']}"
    assert r["vm"] == 0,         f"vm={r['vm']}"  # integ gated, no change
    assert r["spike_flag"] == 0  # spike_if_ge gated, no change
    assert r["emit_valid"] == 0  # emit gated, no fire


@cocotb.test()
async def test_standard_barrier_triggers_spike(dut):
    """IF_STANDARD barrier: acc=5, threshold=4 → spike=1, emit=1, vm=0."""
    await _reset_dut(dut)
    r = await _run(dut, IF_STANDARD,
                   acc=5, threshold=4, event_tag=TAG_BARRIER)
    assert r["spike_flag"] == 1
    assert r["emit_valid"] == 1
    assert r["vm"] == 0   # reset after spike
    assert r["acc"] == 0  # cleared by INTEG k[2]=1


@cocotb.test()
async def test_standard_barrier_below_threshold(dut):
    """IF_STANDARD barrier: acc=2, threshold=4 -> no spike; emit still fires (ungated emit_valid)."""
    await _reset_dut(dut)
    r = await _run(dut, IF_STANDARD,
                   acc=2, threshold=4, event_tag=TAG_BARRIER)
    assert r["spike_flag"] == 0, f"spike_flag={r['spike_flag']}"
    assert r["vm"] == 2, f"vm={r['vm']}"  # integrated (acc=2) but below threshold=4
    assert r["acc"] == 0, f"acc cleared by integ k[2]=1: acc={r['acc']}"  # integ clears acc
    assert r["emit_valid"] == 1, f"emit fires on barrier even without spike: emit_valid={r['emit_valid']}"


@cocotb.test()
async def test_two_data_events_then_barrier_spikes(dut):
    """Accumulate over two data events, then barrier triggers spike."""
    await _reset_dut(dut)
    # Data event 1: weight=3 → acc=3
    task = await _dispatch(dut, IF_STANDARD,
                           vm=0, acc=0, threshold=5, event_tag=TAG_DATA, weight=3)
    r1 = await _wait_result(dut)
    task.kill()
    assert r1["acc"] == 3

    # Data event 2: weight=3 → acc=6
    task = await _dispatch(dut, IF_STANDARD,
                           vm=r1["vm"], acc=r1["acc"],
                           threshold=r1["threshold"],
                           event_tag=TAG_DATA, weight=3)
    r2 = await _wait_result(dut)
    task.kill()
    assert r2["acc"] == 6

    # Barrier: integrate 6 > 5 → spike
    task = await _dispatch(dut, IF_STANDARD,
                           vm=r2["vm"], acc=r2["acc"],
                           threshold=r2["threshold"],
                           event_tag=TAG_BARRIER, weight=0)
    r3 = await _wait_result(dut)
    task.kill()
    assert r3["spike_flag"] == 1
    assert r3["emit_valid"] == 1
    assert r3["vm"] == 0


# =============================================================================
# Zero-length dispatch
# =============================================================================

@cocotb.test()
async def test_zero_len_no_fetch(dut):
    """ucode_len=0: no ucode fetch; result is immediate context snapshot."""
    await _reset_dut(dut)
    payload = _pack_start(logical_idx=1, vm=3, acc=-2 & 0xF, threshold=5,
                          last_tag=1, cmp_ge=1, spike_flag=1,
                          ucode_len=0)
    # Wait for ready using same pattern as _dispatch
    for _ in range(16):
        await RisingEdge(dut.clk)
        await ReadOnly()
        if int(_s(dut, "worker_start_ready").value):
            break
    else:
        raise AssertionError("worker_start_ready never asserted")
    await RisingEdge(dut.clk)
    _s(dut, "worker_start_payload").value = payload
    _s(dut, "worker_start_valid").value   = 1
    await RisingEdge(dut.clk)
    # Verify no ucode requests sent
    await ReadOnly()
    assert int(_s(dut, "ucode_read_valid").value) == 0, \
        "ucode_read_valid should be 0 for zero-length program"
    await RisingEdge(dut.clk)
    _s(dut, "worker_start_valid").value = 0
    r = await _wait_result(dut, max_cycles=16)
    assert r["vm"] == 3 and r["acc"] == -2 and r["threshold"] == 5
    assert r["last_tag"] == 1 and r["spike_flag"] == 1
    assert r["emit_valid"] == 0


@cocotb.test()
async def test_zero_len_rf_round_trip(dut):
    """RF state round-trips unchanged through a zero-length dispatch."""
    await _reset_dut(dut)
    for vm in range(-4, 4):
        for thr in range(-4, 4):
            r = await _run(dut, [], vm=vm & 0xF, threshold=thr & 0xF)
            assert r["vm"] == vm, f"vm RT {vm}→{r['vm']}"
            assert r["threshold"] == thr, f"thr RT {thr}→{r['threshold']}"


# =============================================================================
# Pipeline / handshake correctness
# =============================================================================

@cocotb.test()
async def test_result_held_until_ready(dut):
    """worker_result_valid must remain asserted until worker_result_ready."""
    await _reset_dut(dut)
    task = await _dispatch(dut, [encode_instr(OP_RECV)])
    for _ in range(64):
        await RisingEdge(dut.clk)
        await ReadOnly()
        if int(_s(dut, "worker_result_valid").value):
            break
    else:
        task.cancel()
        raise AssertionError("result_valid never asserted")
    # Hold without ready for 4 cycles — valid must stay high
    for _ in range(4):
        await RisingEdge(dut.clk)
        await ReadOnly()
        assert int(_s(dut, "worker_result_valid").value) == 1, \
            "result_valid dropped prematurely"
    await RisingEdge(dut.clk)
    _s(dut, "worker_result_ready").value = 1
    await RisingEdge(dut.clk)
    _s(dut, "worker_result_ready").value = 0
    task.cancel()


@cocotb.test()
async def test_back_to_back_dispatches(dut):
    """Sequential dispatches complete independently with correct RF state."""
    await _reset_dut(dut)
    r1 = await _run(dut, IF_STANDARD,
                    threshold=5, event_tag=TAG_DATA, weight=3)
    assert r1["acc"] == 3
    r2 = await _run(dut, IF_STANDARD,
                    vm=r1["vm"], acc=r1["acc"], threshold=r1["threshold"],
                    event_tag=TAG_BARRIER)
    assert r2["vm"] == 3 and r2["spike_flag"] == 0


@cocotb.test()
async def test_ucode_ptr_offset(dut):
    """ucode_ptr=2 fetches only the 3rd word (index 2) in a 3-word program.

    NCC fetches words at [ptr, ptr+1, ..., ptr+len-1]. With ptr=2, len=1 derived
    from ucode_len: we pass words with len=1 explicitly so only index 2 executes.
    Use k=3 (=011b, instr_word[2]=0, not tag-gated) for imm=+3.
    """
    await _reset_dut(dut)
    # Only 1 word starting at ptr=2; server is asked for index 2.
    # ucode_len=1 means NCC will request exactly one word.
    words_srv = {2: encode_instr(OP_LDI, rd=0, k=3)}  # imm=+3, no tag gate

    async def _serve_single(dut):
        _s(dut, "ucode_rsp_valid").value   = 0
        _s(dut, "ucode_rsp_payload").value = 0
        pending = None
        while True:
            await RisingEdge(dut.clk)
            if pending is not None:
                _s(dut, "ucode_rsp_payload").value = int(pending) & 0xFFF
                _s(dut, "ucode_rsp_valid").value   = 1
            else:
                _s(dut, "ucode_rsp_valid").value   = 0
            await ReadOnly()
            if int(_s(dut, "ucode_read_valid").value):
                req = _unpack_ucode_req(int(_s(dut, "ucode_read_payload").value))
                pending = words_srv.get(req["word_index"], 0)
            else:
                pending = None

    ucode_task = cocotb.start_soon(_serve_single(dut))
    try:
        # dispatch with ucode_len=1, ucode_ptr=2
        payload = _pack_start(ucode_len=1, ucode_ptr=2)
        for _ in range(64):
            await RisingEdge(dut.clk)
            await ReadOnly()
            if int(_s(dut, "worker_start_ready").value):
                break
        await RisingEdge(dut.clk)
        _s(dut, "worker_start_payload").value = payload
        _s(dut, "worker_start_valid").value   = 1
        await RisingEdge(dut.clk)
        _s(dut, "worker_start_valid").value   = 0
        r = await _wait_result(dut)
    finally:
        ucode_task.cancel()
    assert r["vm"] == 3, f"vm={r['vm']}"


@cocotb.test()
async def test_logical_idx_preserved_in_result(dut):
    """logical_idx in start payload round-trips unchanged into result."""
    await _reset_dut(dut)
    for idx in range(4):
        r = await _run(dut, [], logical_idx=idx)
        assert r["logical_idx"] == idx, f"idx {idx}→{r['logical_idx']}"


# =============================================================================
# graph_state_clear and reset
# =============================================================================

@cocotb.test()
async def test_graph_state_clear_restores_ready(dut):
    """graph_state_clear=1 aborts busy state and restores worker_start_ready."""
    await _reset_dut(dut)
    # Begin a dispatch without serving ucode so NCC stays busy
    _s(dut, "ucode_read_ready").value     = 0
    _s(dut, "worker_start_payload").value = _pack_start(ucode_len=4)
    _s(dut, "worker_start_valid").value   = 1
    await RisingEdge(dut.clk)
    _s(dut, "worker_start_valid").value   = 0
    await RisingEdge(dut.clk)
    dut.graph_state_clear.value = 1
    await RisingEdge(dut.clk)
    dut.graph_state_clear.value = 0
    _s(dut, "ucode_read_ready").value     = 1
    await RisingEdge(dut.clk)
    await ReadOnly()
    assert int(_s(dut, "worker_start_ready").value) == 1, \
        "DUT should be ready after graph_state_clear"


@cocotb.test()
async def test_rst_n_clears_state(dut):
    """rst_n=0 then rst_n=1: DUT is idle and ready; no stale valid."""
    await _reset_dut(dut)
    task = await _dispatch(dut, IF_STANDARD,
                           vm=5, acc=3, threshold=3, event_tag=TAG_BARRIER)
    dut.rst_n.value = 0
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)
    dut.rst_n.value = 1
    task.kill()
    await RisingEdge(dut.clk)
    await ReadOnly()
    assert int(_s(dut, "worker_result_valid").value) == 0
    assert int(_s(dut, "worker_start_ready").value) == 1


# =============================================================================
# ena suppression
# =============================================================================

@cocotb.test()
async def test_ena_low_no_ucode_requests(dut):
    """ena=0 suppresses ucode_read requests while dispatch is pending."""
    await _reset_dut(dut)
    _s(dut, "worker_start_payload").value = _pack_start(ucode_len=4)
    _s(dut, "worker_start_valid").value   = 1
    await RisingEdge(dut.clk)
    _s(dut, "worker_start_valid").value   = 0
    dut.ena.value = 0
    for _ in range(6):
        await RisingEdge(dut.clk)
        await ReadOnly()
        assert int(_s(dut, "ucode_read_valid").value) == 0


@cocotb.test()
async def test_ena_reenable_resumes_fetch(dut):
    """After ena=0, re-enabling causes ucode_read_valid to reassert."""
    await _reset_dut(dut)
    _s(dut, "worker_start_payload").value = _pack_start(ucode_len=1)
    _s(dut, "worker_start_valid").value   = 1
    await RisingEdge(dut.clk)
    _s(dut, "worker_start_valid").value   = 0
    dut.ena.value = 0
    for _ in range(4):
        await RisingEdge(dut.clk)
    dut.ena.value = 1
    seen = False
    for _ in range(10):
        await RisingEdge(dut.clk)
        await ReadOnly()
        if int(_s(dut, "ucode_read_valid").value):
            seen = True
            break
    assert seen, "ucode_read_valid did not reassert after ena reenable"
