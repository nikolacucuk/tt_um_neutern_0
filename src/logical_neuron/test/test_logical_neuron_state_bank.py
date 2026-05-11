import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ReadWrite, RisingEdge

_CLOCK_TASK = None

CSR_CTRL = 0x0
CSR_UCODE_PTR = 0x1
CSR_UCODE_LEN = 0x2
CSR_VEC_BASE_01 = 0x3
CSR_VEC_BASE_23 = 0x4
CSR_INIT_VI = 0x5
CSR_INIT_TR = 0x6
CSR_INIT_T01 = 0x7
CSR_INIT_WAUX = 0x8
CSR_NEURON_META = 0x9
CSR_RESET_TRIGGER = 0xA

CSR_CTRL_SOFT_RESET_BIT = 0
CSR_CTRL_HOST_EGRESS_BIT = 3
CSR_CTRL_NOC_EGRESS_BIT = 4
CSR_RESET_TRIGGER_SOFT_RESET_BIT = 0


def _mask_for(*indices: int) -> int:
    mask = 0
    for idx in indices:
        mask |= 1 << idx
    return mask


def _init_rf_flat(v_mem: int, syn: int, refr: int, aux: int) -> int:
    # Compact 32-bit layout: rf[0]=v_mem, rf[1]=syn, rf[2]=refr, rf[3]=aux.
    # Each byte occupies its natural position [byte*8 +: 8].
    value = 0
    value |= (v_mem & 0xFF) << 0
    value |= (syn   & 0xFF) << 8
    value |= (refr  & 0xFF) << 16
    value |= (aux   & 0xFF) << 24
    return value


async def _tick(dut, cycles: int = 1) -> None:
    for _ in range(cycles):
        await RisingEdge(dut.clk)
        await ReadWrite()


async def _wait_until(dut, condition, max_cycles: int = 24) -> bool:
    for _ in range(max_cycles):
        if condition():
            return True
        await _tick(dut, 1)
    return condition()


async def _drive_idle_writes(dut) -> None:
    dut.graph_state_clear.value = 0
    dut.csr_write_en.value = 0
    dut.csr_write_addr.value = 0
    dut.csr_write_broadcast.value = 0
    dut.csr_addr.value = CSR_CTRL
    dut.csr_data.value = 0

    dut.ucode_prog_en.value = 0
    dut.ucode_prog_addr.value = 0
    dut.ucode_prog_ptr.value = 0
    dut.ucode_prog_len.value = 0

    dut.fanout_prog_ptr_en.value = 0
    dut.fanout_prog_len_en.value = 0
    dut.fanout_prog_addr.value = 0
    dut.fanout_prog_ptr.value = 0
    dut.fanout_prog_len.value = 0


async def _set_all_read_indices(dut, idx: int) -> None:
    dut.target_read_idx.value = idx
    dut.dispatch_read_idx.value = idx
    dut.fanout_read_idx.value = idx
    dut.commit_read_idx.value = idx
    dut.dump_read_idx.value = idx
    dut.reset_read_idx.value = idx


async def _reset_dut(dut) -> None:
    global _CLOCK_TASK
    if _CLOCK_TASK is None or _CLOCK_TASK.done():
        _CLOCK_TASK = cocotb.start_soon(Clock(dut.clk, 10, units="ns").start())

    dut.rst_n.value = 0
    dut.ena.value = 0
    await _drive_idle_writes(dut)
    await _set_all_read_indices(dut, 0)
    await _tick(dut, 3)

    dut.rst_n.value = 1
    dut.ena.value = 1
    await _tick(dut, 4)


async def _pulse_csr_write(
    dut,
    *,
    idx: int,
    addr: int,
    data: int,
    broadcast: int = 0,
) -> None:
    dut.csr_write_en.value = 1
    dut.csr_write_addr.value = idx
    dut.csr_write_broadcast.value = broadcast
    dut.csr_addr.value = addr
    dut.csr_data.value = data
    await _tick(dut, 1)
    dut.csr_write_en.value = 0
    dut.csr_write_broadcast.value = 0
    await _tick(dut, 1)


async def _pulse_ucode_prog(dut, *, idx: int, ptr: int, length: int) -> None:
    dut.ucode_prog_en.value = 1
    dut.ucode_prog_addr.value = idx
    dut.ucode_prog_ptr.value = ptr
    dut.ucode_prog_len.value = length
    await _tick(dut, 1)
    dut.ucode_prog_en.value = 0
    await _tick(dut, 1)


async def _pulse_fanout_prog(
    dut,
    *,
    idx: int,
    ptr: int | None = None,
    length: int | None = None,
) -> None:
    dut.fanout_prog_addr.value = idx
    dut.fanout_prog_ptr_en.value = 1 if ptr is not None else 0
    dut.fanout_prog_len_en.value = 1 if length is not None else 0
    dut.fanout_prog_ptr.value = 0 if ptr is None else ptr
    dut.fanout_prog_len.value = 0 if length is None else length
    await _tick(dut, 1)
    dut.fanout_prog_ptr_en.value = 0
    dut.fanout_prog_len_en.value = 0
    await _tick(dut, 1)


async def _read_target_csr(dut, *, idx: int, csr_addr: int) -> int:
    dut.target_read_idx.value = idx
    dut.csr_addr.value = csr_addr
    await _tick(dut, 10)
    return int(dut.target_csr_read_data.value)


@cocotb.test()
async def test_reset_defaults_and_invalid_epoch_behavior(dut):
    await _reset_dut(dut)

    neurons = len(dut.host_egress_en_bus)
    all_neurons_mask = (1 << neurons) - 1

    ctrl_data = await _read_target_csr(dut, idx=0, csr_addr=CSR_CTRL)
    assert ctrl_data == 0x10

    await _set_all_read_indices(dut, 0)
    await _tick(dut, 3)

    assert int(dut.target_fanout_ptr.value) == 0
    assert int(dut.target_fanout_len.value) == 0
    assert int(dut.dispatch_ucode_ptr.value) == 0
    assert int(dut.dispatch_ucode_len.value) == 0
    assert int(dut.dispatch_fanout_len.value) == 0
    assert int(dut.fanout_read_ptr.value) == 0
    assert int(dut.fanout_read_len.value) == 0
    assert int(dut.commit_fanout_ptr.value) == 0
    assert int(dut.dump_init_rf_flat.value) == 0
    assert int(dut.dump_ucode_ptr.value) == 0
    assert int(dut.dump_ucode_len.value) == 0
    assert int(dut.dump_neuron_flags.value) == 0
    assert int(dut.dump_fanout_ptr.value) == 0
    assert int(dut.dump_fanout_len.value) == 0
    assert int(dut.reset_init_rf_flat.value) == 0
    assert int(dut.reset_last_event_time.value) == 0
    assert int(dut.soft_reset_pulse.value) == 0
    assert int(dut.soft_reset_valid.value) == 0
    assert int(dut.host_egress_en_bus.value) == 0
    assert int(dut.noc_egress_en_bus.value) == all_neurons_mask


@cocotb.test()
async def test_csr_ctrl_unicast_soft_reset_and_egress(dut):
    await _reset_dut(dut)

    ctrl_value = (
        (1 << CSR_CTRL_SOFT_RESET_BIT)
        | (1 << CSR_CTRL_HOST_EGRESS_BIT)
        | (0 << CSR_CTRL_NOC_EGRESS_BIT)
    )

    await _pulse_csr_write(dut, idx=1, addr=CSR_CTRL, data=ctrl_value)

    # Legacy pulse bus is tied off in scalar-reset refactor.
    assert int(dut.soft_reset_pulse.value) == 0
    assert int(dut.host_egress_en_bus.value) == _mask_for(1)
    assert int(dut.noc_egress_en_bus.value) == _mask_for(0, 2, 3)

    idx1_ctrl = await _read_target_csr(dut, idx=1, csr_addr=CSR_CTRL)
    idx3_ctrl = await _read_target_csr(dut, idx=3, csr_addr=CSR_CTRL)
    assert idx1_ctrl == (1 << CSR_CTRL_HOST_EGRESS_BIT)
    assert idx3_ctrl == 0x10


@cocotb.test()
async def test_ucode_programming_by_direct_address(dut):
    await _reset_dut(dut)

    await _pulse_ucode_prog(dut, idx=2, ptr=7, length=9)

    dut.dispatch_read_idx.value = 2
    await _tick(dut, 10)
    assert int(dut.dispatch_ucode_ptr.value) == 7
    assert int(dut.dispatch_ucode_len.value) == 9

    dut.dispatch_read_idx.value = 3
    await _tick(dut, 10)
    assert int(dut.dispatch_ucode_ptr.value) == 0
    assert int(dut.dispatch_ucode_len.value) == 0

    ptr_data = await _read_target_csr(dut, idx=2, csr_addr=CSR_UCODE_PTR)
    len_data = await _read_target_csr(dut, idx=2, csr_addr=CSR_UCODE_LEN)
    assert ptr_data == 7
    assert len_data == 9


@cocotb.test()
async def test_fanout_prog_drives_all_async_read_domains(dut):
    await _reset_dut(dut)

    await _pulse_fanout_prog(dut, idx=0, ptr=11, length=4)
    await _pulse_fanout_prog(dut, idx=1, ptr=29, length=7)

    await _set_all_read_indices(dut, 0)
    await _tick(dut, 10)
    assert int(dut.target_fanout_ptr.value) == 11
    assert int(dut.target_fanout_len.value) == 4
    assert int(dut.fanout_read_ptr.value) == 11
    assert int(dut.fanout_read_len.value) == 4
    assert int(dut.commit_fanout_ptr.value) == 11
    assert int(dut.dump_fanout_ptr.value) == 11
    assert int(dut.dump_fanout_len.value) == 4
    assert int(dut.dispatch_fanout_len.value) == 4

    await _set_all_read_indices(dut, 1)
    assert await _wait_until(
        dut,
        lambda: int(dut.target_fanout_ptr.value) == 29
        and int(dut.target_fanout_len.value) == 7
        and int(dut.fanout_read_ptr.value) == 29
        and int(dut.fanout_read_len.value) == 7
        and int(dut.commit_fanout_ptr.value) == 29
        and int(dut.dump_fanout_ptr.value) == 29
        and int(dut.dump_fanout_len.value) == 7
        and int(dut.dispatch_fanout_len.value) == 7,
    )


@cocotb.test()
async def test_csr_init_dump_reset_readback_alignment(dut):
    await _reset_dut(dut)

    idx = 2
    await _pulse_csr_write(dut, idx=idx, addr=CSR_INIT_VI, data=0x12)
    await _pulse_csr_write(dut, idx=idx, addr=CSR_INIT_TR, data=0x34)
    await _pulse_csr_write(dut, idx=idx, addr=CSR_INIT_T01, data=0x56)
    await _pulse_csr_write(dut, idx=idx, addr=CSR_INIT_WAUX, data=0x78)
    await _pulse_csr_write(dut, idx=idx, addr=CSR_VEC_BASE_01, data=0x9A)
    await _pulse_csr_write(dut, idx=idx, addr=CSR_NEURON_META, data=0x2D)

    await _set_all_read_indices(dut, idx)
    assert await _wait_until(
        dut,
        lambda: int(dut.dump_init_rf_flat.value) == _init_rf_flat(0x12, 0x34, 0x56, 0x78)
        and int(dut.dump_neuron_flags.value) == 0x9A
        and int(dut.reset_init_rf_flat.value) == _init_rf_flat(0x12, 0x34, 0x56, 0x78)
        and int(dut.reset_last_event_time.value) == (0x2D & 0x3F),
    )

    assert await _read_target_csr(dut, idx=idx, csr_addr=CSR_INIT_VI) == 0x12
    assert await _read_target_csr(dut, idx=idx, csr_addr=CSR_INIT_TR) == 0x34
    assert await _read_target_csr(dut, idx=idx, csr_addr=CSR_INIT_T01) == 0x56
    assert await _read_target_csr(dut, idx=idx, csr_addr=CSR_INIT_WAUX) == 0x78
    assert await _read_target_csr(dut, idx=idx, csr_addr=CSR_VEC_BASE_01) == 0x9A
    assert await _read_target_csr(dut, idx=idx, csr_addr=CSR_NEURON_META) == (0x2D & 0x3F)


@cocotb.test()
async def test_graph_state_clear_resets_visibility_and_egress(dut):
    await _reset_dut(dut)

    idx = 1
    await _pulse_csr_write(dut, idx=idx, addr=CSR_INIT_VI, data=0xAB)
    await _pulse_csr_write(
        dut,
        idx=idx,
        addr=CSR_CTRL,
        data=(1 << CSR_CTRL_HOST_EGRESS_BIT),
    )

    await _set_all_read_indices(dut, idx)
    await _tick(dut, 4)
    assert int(dut.dump_init_rf_flat.value) != 0
    assert int(dut.host_egress_en_bus.value) == _mask_for(idx)

    dut.graph_state_clear.value = 1
    await _tick(dut, 1)
    dut.graph_state_clear.value = 0
    await _tick(dut, 4)

    assert int(dut.dump_init_rf_flat.value) == 0
    assert int(dut.host_egress_en_bus.value) == 0
    assert int(dut.noc_egress_en_bus.value) == _mask_for(0, 1, 2, 3)


@cocotb.test()
async def test_reset_trigger_broadcast_drains_all_indices(dut):
    await _reset_dut(dut)

    await _pulse_csr_write(
        dut,
        idx=0,
        addr=CSR_RESET_TRIGGER,
        data=(1 << CSR_RESET_TRIGGER_SOFT_RESET_BIT),
        broadcast=1,
    )

    seen = set()
    if int(dut.soft_reset_valid.value):
        seen.add(int(dut.soft_reset_idx.value))
    for _ in range(16):
        await _tick(dut, 1)
        if int(dut.soft_reset_valid.value):
            seen.add(int(dut.soft_reset_idx.value))
        if len(seen) == 4:
            break

    assert seen == {0, 1, 2, 3}
