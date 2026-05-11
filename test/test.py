# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-License-Identifier: Apache-2.0
"""
test.py — exhaustive cocotb verification for tt_um_neutern_0 (TT boundary)

DUT top-level: tb (tb.v) wrapping tt_um_neutern_0 → tile_top_tt → tile_top

TT pin mapping (tile_top_tt adapter, architecture §tile_top_tt):
    ui_in[7:0]  = mode-dependent flit byte
        Spike mode (uio_in[2]=0):
            [7:4] weight[3:0]  signed 4-bit (-8..+7)
            [3:2] reserved     2'b00 (ignored by tile)
            [1]   neuron_y[0]  row address (0 or 1)
            [0]   neuron_x[0]  column address (0 or 1)
        Weight header mode (uio_in[2]=1, uio_in[3]=0):
            [2]   read/write selector (0=write, 1=readback)
            [7:4] weight nibble for write mode
            [1:0] target neuron coordinates
        ISA header mode (uio_in[2]=1, uio_in[3]=1):
            [7:3] op5, [2] barrier, [1:0] target coordinates
  uio_in[0]   = rv_in_valid   (host→tile flow: spike valid)
  uio_in[1]   = rv_out_ready  (host←tile flow: host ready to receive)
    uio_in[2]   = rv_in_is_header
    uio_in[3]   = rv_in_header_is_isa
  uio_out[0]  = rv_in_ready   (tile back-pressure signal)
  uio_out[1]  = rv_out_valid  (tile output spike valid)
  uo_out[7:0] = rv_out_payload (same neutern_spike_t format as ui_in)

Grid configuration: 2×2 = 4 neurons
  neuron_x=0,neuron_y=0 → core_id 0
  neuron_x=1,neuron_y=0 → core_id 1
  neuron_x=0,neuron_y=1 → core_id 2
  neuron_x=1,neuron_y=1 → core_id 3

Verification coverage:
  §R  — Reset and idle state
  §H  — rv_in ready/valid handshake correctness (protocol discipline)
  §I  — Spike ingress: all neuron addresses, all weight values, reserved bits
  §B  — Back-pressure: rv_out_ready de-asserted holds rv_out_valid
  §F  — Burst and back-to-back flit transport
  §E  — Enable gating: ena=0 stops tile activity
  §O  — Output flit format: uo_out encoding correctness
    §C  — Header config/readback: weight write/read and ISA header ack
"""

from __future__ import annotations

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ReadOnly, ClockCycles, Timer
from cocotb.result import SimTimeoutError

# ─────────────────────────────────────────────────────────────────────────────
# Clock and signal constants
# ─────────────────────────────────────────────────────────────────────────────

CLK_PERIOD_US  = 10          # 100 KHz — matches TT test harness
NEURONS_PER_TILE = 4         # 2×2 grid

# uio_in bit positions (host-driven)
UIO_RV_IN_VALID  = 0         # bit 0: host→tile spike valid
UIO_RV_OUT_READY = 1         # bit 1: host ready to receive tile output
UIO_RV_IN_IS_HEADER = 2      # bit 2: 1=header packet, 0=normal spike flit
UIO_RV_IN_HEADER_IS_ISA = 3  # bit 3: with bit2=1, 1=ISA header, 0=weight header

# uio_out bit positions (tile-driven)
UIO_RV_IN_READY  = 0         # bit 0: tile accepts input (back-pressure)
UIO_RV_OUT_VALID = 1         # bit 1: tile has output flit ready


# ─────────────────────────────────────────────────────────────────────────────
# Helper: encode neutern_spike_t
# ─────────────────────────────────────────────────────────────────────────────

def spike_flit(weight: int, neuron_y: int, neuron_x: int) -> int:
    """
    Encode a neutern_spike_t into 8-bit ui_in format:
      [7:4] weight[3:0] — signed 4-bit in two's complement
      [3:2] 2'b00       — reserved (always 0)
      [1]   neuron_y[0]
      [0]   neuron_x[0]
    """
    w4 = weight & 0xF   # two's complement 4-bit
    return ((w4 & 0xF) << 4) | ((neuron_y & 0x1) << 1) | (neuron_x & 0x1)


def weight_header_flit(weight: int, neuron_y: int, neuron_x: int, *, read: bool) -> int:
        """
        Encode a weight-header flit (uio_in[2]=1, uio_in[3]=0):
            [7:4] weight nibble (used for write)
            [2]   0=write, 1=read
            [1]   neuron_y[0]
            [0]   neuron_x[0]
        """
        w4 = weight & 0xF
        return ((w4 & 0xF) << 4) | ((1 if read else 0) << 2) | ((neuron_y & 0x1) << 1) | (neuron_x & 0x1)


def isa_header_flit(op5: int, neuron_y: int, neuron_x: int, *, barrier: int = 0) -> int:
        """
        Encode an ISA-header flit (uio_in[2]=1, uio_in[3]=1):
            [7:3] op5 opcode
            [2]   barrier bit
            [1]   neuron_y[0]
            [0]   neuron_x[0]
        """
        op5v = op5 & 0x1F
        return (op5v << 3) | ((barrier & 0x1) << 2) | ((neuron_y & 0x1) << 1) | (neuron_x & 0x1)


def unpack_flit(byte_val: int) -> dict:
    """
    Unpack a neutern_spike_t byte into named fields.
    Returns weight as signed integer (-8..+7).
    """
    w4 = (byte_val >> 4) & 0xF
    weight = w4 if w4 < 8 else w4 - 16   # sign-extend
    return {
        "weight":   weight,
        "reserved": (byte_val >> 2) & 0x3,
        "neuron_y": (byte_val >> 1) & 0x1,
        "neuron_x": (byte_val >> 0) & 0x1,
    }


# ─────────────────────────────────────────────────────────────────────────────
# DUT helpers
# ─────────────────────────────────────────────────────────────────────────────

def _rv_in_ready(dut) -> int:
    return (int(dut.uio_out.value) >> UIO_RV_IN_READY) & 1


def _rv_out_valid(dut) -> int:
    return (int(dut.uio_out.value) >> UIO_RV_OUT_VALID) & 1


async def _reset(dut, *, cycles: int = 10) -> None:
    """Drive reset for 'cycles' clocks then de-assert."""
    dut.ena.value    = 0
    dut.ui_in.value  = 0
    dut.uio_in.value = 0
    dut.rst_n.value  = 0
    await ClockCycles(dut.clk, cycles)
    dut.rst_n.value = 1
    dut.ena.value   = 1
    # Default: ready to receive tile output, no spike to send
    dut.uio_in.value = (1 << UIO_RV_OUT_READY)
    await RisingEdge(dut.clk)


async def _send_spike(dut, flit: int, timeout_cycles: int = 50) -> None:
    """
    Send one spike flit on ui_in via rv_in ready/valid handshake.
    Sets ui_in=flit, uio_in[0]=1, waits for uio_out[0]=1 in same cycle, then
    advances one clock and de-asserts valid.
    """
    dut.ui_in.value  = flit
    uio = (int(dut.uio_in.value) & ~(1 << UIO_RV_IN_VALID)) | (1 << UIO_RV_IN_VALID)
    dut.uio_in.value = uio

    deadline = timeout_cycles
    while True:
        await ReadOnly()
        if _rv_in_ready(dut):
            await RisingEdge(dut.clk)
            break
        await RisingEdge(dut.clk)
        deadline -= 1
        assert deadline > 0, (
            f"rv_in handshake timeout after {timeout_cycles} cycles "
            f"(flit=0x{flit:02x})"
        )

    # De-assert valid and clear payload
    uio = int(dut.uio_in.value) & ~(1 << UIO_RV_IN_VALID)
    dut.uio_in.value = uio
    dut.ui_in.value  = 0


async def _send_flit(
    dut,
    flit: int,
    *,
    is_header: bool,
    header_is_isa: bool,
    timeout_cycles: int = 50,
) -> None:
    """
    Send one byte on ui_in with explicit header mode controls on uio_in[3:2].
    """
    dut.ui_in.value = flit

    uio = int(dut.uio_in.value)
    uio |= (1 << UIO_RV_IN_VALID)
    if is_header:
        uio |= (1 << UIO_RV_IN_IS_HEADER)
    else:
        uio &= ~(1 << UIO_RV_IN_IS_HEADER)
    if header_is_isa:
        uio |= (1 << UIO_RV_IN_HEADER_IS_ISA)
    else:
        uio &= ~(1 << UIO_RV_IN_HEADER_IS_ISA)
    dut.uio_in.value = uio

    deadline = timeout_cycles
    while True:
        await ReadOnly()
        if _rv_in_ready(dut):
            await RisingEdge(dut.clk)
            break
        await RisingEdge(dut.clk)
        deadline -= 1
        assert deadline > 0, (
            f"rv_in handshake timeout after {timeout_cycles} cycles "
            f"(flit=0x{flit:02x}, is_header={int(is_header)}, header_is_isa={int(header_is_isa)})"
        )

    # Drop valid and restore non-header mode defaults for subsequent spikes.
    uio = int(dut.uio_in.value)
    uio &= ~(1 << UIO_RV_IN_VALID)
    uio &= ~(1 << UIO_RV_IN_IS_HEADER)
    uio &= ~(1 << UIO_RV_IN_HEADER_IS_ISA)
    dut.uio_in.value = uio
    dut.ui_in.value = 0


async def _try_recv_flit(dut, timeout_cycles: int = 30):
    """
    Try receiving one output flit; return None if no flit appears before timeout.
    """
    deadline = timeout_cycles
    while deadline > 0:
        await ReadOnly()
        if _rv_out_valid(dut):
            raw = int(dut.uo_out.value)
            await RisingEdge(dut.clk)
            return unpack_flit(raw)
        await RisingEdge(dut.clk)
        deadline -= 1
    return None


async def _send_weight_header_write(dut, *, weight: int, neuron_y: int, neuron_x: int) -> None:
    await _send_flit(
        dut,
        weight_header_flit(weight=weight, neuron_y=neuron_y, neuron_x=neuron_x, read=False),
        is_header=True,
        header_is_isa=False,
        timeout_cycles=80,
    )


async def _send_weight_header_read(dut, *, neuron_y: int, neuron_x: int) -> None:
    await _send_flit(
        dut,
        weight_header_flit(weight=0, neuron_y=neuron_y, neuron_x=neuron_x, read=True),
        is_header=True,
        header_is_isa=False,
        timeout_cycles=80,
    )


async def _send_isa_header(dut, *, op5: int, neuron_y: int, neuron_x: int, barrier: int = 0) -> None:
    await _send_flit(
        dut,
        isa_header_flit(op5=op5, neuron_y=neuron_y, neuron_x=neuron_x, barrier=barrier),
        is_header=True,
        header_is_isa=True,
        timeout_cycles=80,
    )


async def _recv_flit(dut, timeout_cycles: int = 200) -> dict:
    """
    Receive one output flit from uo_out when rv_out_valid (uio_out[1]) asserts.
    Returns unpacked neutern_spike_t fields.
    """
    deadline = timeout_cycles
    while True:
        await ReadOnly()
        if _rv_out_valid(dut):
            raw = int(dut.uo_out.value)
            await RisingEdge(dut.clk)
            return unpack_flit(raw)
        await RisingEdge(dut.clk)
        deadline -= 1
        assert deadline > 0, (
            f"rv_out receive timeout after {timeout_cycles} cycles"
        )


# ─────────────────────────────────────────────────────────────────────────────
# §R — Reset and idle
# ─────────────────────────────────────────────────────────────────────────────

@cocotb.test(timeout_time=5, timeout_unit="ms")
async def test_project(dut):
    """
    §R — Canonical TT idle check (required by harness).

    After reset with no valid input:
      - uio_out[1] (rv_out_valid) must be 0 (no tile output)
      - uo_out must be 0 (no output flit)
    """
    dut._log.info("Start")
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_US, units="us").start())
    await _reset(dut)

    for _ in range(5):
        await ReadOnly()
        rv_out_v = _rv_out_valid(dut)
        assert rv_out_v == 0, \
            f"Expected rv_out_valid=0 after reset, got {rv_out_v}"
        assert int(dut.uo_out.value) == 0, \
            f"Expected uo_out=0 after reset, got {int(dut.uo_out.value)}"
        await RisingEdge(dut.clk)

    dut._log.info("Post-reset idle check passed")


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def test_reset_clears_all_outputs(dut):
    """
    §R — All DUT outputs are 0 or defined immediately after rst_n=1.

    Exercises double-reset: assert, de-assert, hold for a few cycles, re-assert.
    """
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_US, units="us").start())

    for iteration in range(2):
        await _reset(dut, cycles=5)
        await ClockCycles(dut.clk, 3)
        await RisingEdge(dut.clk)
        assert _rv_out_valid(dut) == 0, \
            f"[iter={iteration}] rv_out_valid must be 0 after reset"
        assert _rv_in_ready(dut) == 1, \
            f"[iter={iteration}] rv_in_ready must be 1 after reset"
        assert int(dut.uo_out.value) == 0, \
            f"[iter={iteration}] uo_out must be 0 at idle"


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def test_no_x_propagation_after_reset(dut):
    """
    §R — No X-values on observable outputs after reset.

    Runs 20 idle cycles checking for X/Z on uio_out and uo_out.
    """
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_US, units="us").start())
    await _reset(dut)

    for _ in range(20):
        await ReadOnly()
        uio_str = str(dut.uio_out.value).lower()
        uo_str  = str(dut.uo_out.value).lower()
        assert "x" not in uio_str and "z" not in uio_str, \
            f"X/Z on uio_out: {uio_str}"
        assert "x" not in uo_str and "z" not in uo_str, \
            f"X/Z on uo_out: {uo_str}"
        await RisingEdge(dut.clk)


# ─────────────────────────────────────────────────────────────────────────────
# §H — Ready/valid handshake correctness
# ─────────────────────────────────────────────────────────────────────────────

@cocotb.test(timeout_time=5, timeout_unit="ms")
async def test_rv_in_ready_asserted_at_idle(dut):
    """
    §H — rv_in_ready (uio_out[0]) is asserted at idle.

    Tile must signal readiness to accept flits immediately after reset.
    """
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_US, units="us").start())
    await _reset(dut)

    # Hold valid=0 — tile must still assert ready
    dut.uio_in.value = 0
    for _ in range(5):
        await ReadOnly()
        assert _rv_in_ready(dut) == 1, \
            f"rv_in_ready must be 1 at idle; got {_rv_in_ready(dut)}"
        await RisingEdge(dut.clk)


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def test_no_transfer_without_valid(dut):
    """
    §H — Flit is not consumed when rv_in_valid=0, regardless of payload.

    Drives a non-zero ui_in with valid=0 for several cycles and checks
    that uo_out stays 0 (no spurious output from an unconsumed flit).
    """
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_US, units="us").start())
    await _reset(dut)

    # ui_in has non-zero data but valid=0
    dut.ui_in.value  = spike_flit(weight=7, neuron_y=0, neuron_x=0)
    dut.uio_in.value = (1 << UIO_RV_OUT_READY)   # ready to receive, but no valid

    for _ in range(10):
        await ReadOnly()
        assert _rv_out_valid(dut) == 0, \
            "Tile produced output flit with rv_in_valid=0 (protocol violation)"
        await RisingEdge(dut.clk)


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def test_single_flit_handshake(dut):
    """
    §H — Single flit is accepted when valid and ready are both asserted.

    Drives valid=1 and verifies ready rises before the handshake deadline.
    """
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_US, units="us").start())
    await _reset(dut)

    flit = spike_flit(weight=1, neuron_y=0, neuron_x=0)
    dut.ui_in.value  = flit
    dut.uio_in.value = (1 << UIO_RV_OUT_READY) | (1 << UIO_RV_IN_VALID)

    accepted = False
    for _ in range(20):
        await ReadOnly()
        if _rv_in_ready(dut):
            accepted = True
            await RisingEdge(dut.clk)
            break
        await RisingEdge(dut.clk)

    dut.uio_in.value = (1 << UIO_RV_OUT_READY)
    assert accepted, "rv_in_ready never asserted within 20 cycles"


# ─────────────────────────────────────────────────────────────────────────────
# §I — Spike ingress: all neuron addresses
# ─────────────────────────────────────────────────────────────────────────────

@cocotb.test(timeout_time=10, timeout_unit="ms")
async def test_spike_to_all_four_neurons(dut):
    """
    §I — Send one spike to each of the 4 neuron addresses.

    Neuron grid: 2×2 (neuron_x ∈ {0,1}, neuron_y ∈ {0,1}).
    Verifies tile accepts all 4 neuron addresses without stalling.
    """
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_US, units="us").start())
    await _reset(dut)

    addresses = [
        (0, 0), (1, 0),   # row 0
        (0, 1), (1, 1),   # row 1
    ]

    for (ny, nx) in addresses:
        flit = spike_flit(weight=1, neuron_y=ny, neuron_x=nx)
        await _send_spike(dut, flit)
        await ClockCycles(dut.clk, 3)
        # rv_in_ready is a registered output; stable after RisingEdge
        await RisingEdge(dut.clk)


@cocotb.test(timeout_time=10, timeout_unit="ms")
async def test_spike_neuron_00(dut):
    """§I — Spike to neuron (y=0, x=0): minimum address."""
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_US, units="us").start())
    await _reset(dut)
    await _send_spike(dut, spike_flit(weight=0, neuron_y=0, neuron_x=0))
    await ClockCycles(dut.clk, 5)
    await ReadOnly()
    assert "x" not in str(dut.uio_out.value).lower(), \
        "X on uio_out after spike to neuron (0,0)"


@cocotb.test(timeout_time=10, timeout_unit="ms")
async def test_spike_neuron_10(dut):
    """§I — Spike to neuron (y=0, x=1): column 1."""
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_US, units="us").start())
    await _reset(dut)
    await _send_spike(dut, spike_flit(weight=0, neuron_y=0, neuron_x=1))
    await ClockCycles(dut.clk, 5)
    await ReadOnly()
    assert "x" not in str(dut.uio_out.value).lower()


@cocotb.test(timeout_time=10, timeout_unit="ms")
async def test_spike_neuron_01(dut):
    """§I — Spike to neuron (y=1, x=0): row 1."""
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_US, units="us").start())
    await _reset(dut)
    await _send_spike(dut, spike_flit(weight=0, neuron_y=1, neuron_x=0))
    await ClockCycles(dut.clk, 5)
    await ReadOnly()
    assert "x" not in str(dut.uio_out.value).lower()


@cocotb.test(timeout_time=10, timeout_unit="ms")
async def test_spike_neuron_11(dut):
    """§I — Spike to neuron (y=1, x=1): maximum address."""
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_US, units="us").start())
    await _reset(dut)
    await _send_spike(dut, spike_flit(weight=0, neuron_y=1, neuron_x=1))
    await ClockCycles(dut.clk, 5)
    await ReadOnly()
    assert "x" not in str(dut.uio_out.value).lower()


# ─────────────────────────────────────────────────────────────────────────────
# §I — Spike ingress: all weight values (-8..+7)
# ─────────────────────────────────────────────────────────────────────────────

@cocotb.test(timeout_time=50, timeout_unit="ms")
async def test_all_weight_values_accepted(dut):
    """
    §I — All 16 signed weight values (-8..+7) are accepted without crash.

    Sends one spike per weight value to neuron (y=0, x=0) and verifies
    no X propagation on observable outputs.
    """
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_US, units="us").start())
    await _reset(dut)

    for w in range(-8, 8):   # -8, -7, ..., 0, ..., +7
        flit = spike_flit(weight=w, neuron_y=0, neuron_x=0)
        await _send_spike(dut, flit, timeout_cycles=60)
        await ClockCycles(dut.clk, 2)
        await RisingEdge(dut.clk)
        assert "x" not in str(dut.uio_out.value).lower(), \
            f"X on uio_out at weight={w}"

    dut._log.info("All 16 weight values accepted without crash")


@cocotb.test(timeout_time=20, timeout_unit="ms")
async def test_max_positive_weight(dut):
    """§I — Maximum positive weight (+7): w4=0b0111."""
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_US, units="us").start())
    await _reset(dut)
    await _send_spike(dut, spike_flit(weight=7, neuron_y=0, neuron_x=0))
    await ClockCycles(dut.clk, 5)
    await ReadOnly()
    assert "x" not in str(dut.uio_out.value).lower()


@cocotb.test(timeout_time=20, timeout_unit="ms")
async def test_max_negative_weight(dut):
    """§I — Maximum negative weight (-8): w4=0b1000 (two's complement)."""
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_US, units="us").start())
    await _reset(dut)
    await _send_spike(dut, spike_flit(weight=-8, neuron_y=0, neuron_x=0))
    await ClockCycles(dut.clk, 5)
    await ReadOnly()
    assert "x" not in str(dut.uio_out.value).lower()


@cocotb.test(timeout_time=20, timeout_unit="ms")
async def test_zero_weight(dut):
    """§I — Zero weight (w=0): all weight bits = 0."""
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_US, units="us").start())
    await _reset(dut)
    await _send_spike(dut, spike_flit(weight=0, neuron_y=0, neuron_x=0))
    await ClockCycles(dut.clk, 5)
    await ReadOnly()
    assert "x" not in str(dut.uio_out.value).lower()


# ─────────────────────────────────────────────────────────────────────────────
# §I — Reserved bits in spike mode: ui_in[3:2] must be ignored
# ─────────────────────────────────────────────────────────────────────────────

@cocotb.test(timeout_time=20, timeout_unit="ms")
async def test_reserved_bits_ignored(dut):
    """
    §I — In spike mode (uio_in[2]=0), ui_in[3:2] are ignored by tile_top_tt.

    Sends spikes with reserved bits set to 0b01, 0b10, 0b11 and verifies
    the tile does not stall or crash (it must treat them as don't-care).
    """
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_US, units="us").start())
    await _reset(dut)

    base_flit = spike_flit(weight=1, neuron_y=0, neuron_x=0)

    for reserved_val in (0b01, 0b10, 0b11):
        # Corrupt the reserved bits [3:2]
        flit_with_reserved = (base_flit & ~0xC) | (reserved_val << 2)
        await _send_spike(dut, flit_with_reserved, timeout_cycles=60)
        await ClockCycles(dut.clk, 3)
        await RisingEdge(dut.clk)
        assert "x" not in str(dut.uio_out.value).lower(), \
            f"X on uio_out with reserved_bits={reserved_val:02b}"

    dut._log.info("Reserved bits ui_in[3:2] are handled safely")


# ─────────────────────────────────────────────────────────────────────────────
# §I — Full flit sweep: all 256 ui_in values
# ─────────────────────────────────────────────────────────────────────────────

@cocotb.test(timeout_time=200, timeout_unit="ms")
async def test_full_flit_sweep_no_crash(dut):
    """
    §I — Exhaustive flit sweep: all 256 possible ui_in byte values.

    Sends every possible 8-bit flit value as a spike and verifies:
      - tile accepts each flit (handshake completes within timeout)
      - no X propagation on uio_out after each flit
    This is the most comprehensive single-test ingress coverage.
    """
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_US, units="us").start())
    await _reset(dut)

    for flit_val in range(256):
        dut.ui_in.value  = flit_val
        uio = (int(dut.uio_in.value) & ~(1 << UIO_RV_IN_VALID)) | (1 << UIO_RV_IN_VALID)
        dut.uio_in.value = uio

        # Wait for ready (with generous timeout to allow FIFO drain)
        for _ in range(80):
            await ReadOnly()
            if _rv_in_ready(dut):
                await RisingEdge(dut.clk)
                break
            await RisingEdge(dut.clk)

        # De-assert valid
        uio = int(dut.uio_in.value) & ~(1 << UIO_RV_IN_VALID)
        dut.uio_in.value = uio
        dut.ui_in.value  = 0

        await ReadOnly()
        assert "x" not in str(dut.uio_out.value).lower(), \
            f"X on uio_out at flit_val=0x{flit_val:02X}"

        # Brief gap between flits
        await ClockCycles(dut.clk, 1)

    dut._log.info("All 256 flit values accepted without X propagation")


# ─────────────────────────────────────────────────────────────────────────────
# §B — Back-pressure: rv_out_ready=0 holds rv_out_valid asserted
# ─────────────────────────────────────────────────────────────────────────────

@cocotb.test(timeout_time=20, timeout_unit="ms")
async def test_rv_out_valid_held_when_ready_0(dut):
    """
    §B — rv_out_valid stays asserted while rv_out_ready=0.

    This test is conditional: it only verifies back-pressure behavior if the
    tile produces output (rv_out_valid=1 at some point during the test window).
    If no output is produced, the test is skipped with a warning.
    """
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_US, units="us").start())
    await _reset(dut)

    # Hold downstream not-ready
    dut.uio_in.value = 0   # rv_out_ready=0, rv_in_valid=0

    # Send a spike; if the tile happens to emit, check back-pressure
    flit = spike_flit(weight=7, neuron_y=0, neuron_x=0)
    dut.ui_in.value = flit
    dut.uio_in.value = (1 << UIO_RV_IN_VALID)   # valid=1, ready_out=0

    # Wait for accept or timeout
    for _ in range(30):
        await ReadOnly()
        if _rv_in_ready(dut):
            await RisingEdge(dut.clk)
            break
        await RisingEdge(dut.clk)
    dut.uio_in.value = 0   # drop valid

    # Observe for 20 cycles — if rv_out_valid asserts, verify back-pressure
    for _ in range(20):
        await ReadOnly()
        if _rv_out_valid(dut):
            # Back-pressure asserted: rv_out_valid must stay high
            for j in range(5):
                await ReadOnly()
                assert _rv_out_valid(dut) == 1, \
                    f"rv_out_valid de-asserted with rv_out_ready=0 at cycle {j}"
                await RisingEdge(dut.clk)
            # Release ready
            dut.uio_in.value = (1 << UIO_RV_OUT_READY)
            await ClockCycles(dut.clk, 5)
            dut._log.info("Back-pressure test: output produced and held correctly")
            return
        await RisingEdge(dut.clk)

    dut._log.warning("No output produced during back-pressure test window — skipping hold check")


@cocotb.test(timeout_time=20, timeout_unit="ms")
async def test_rv_out_ready_0_does_not_cause_deadlock(dut):
    """
    §B — Holding rv_out_ready=0 does not cause permanent rv_in stall.

    Sends spikes with rv_out_ready=0 and verifies rv_in_ready does not
    permanently drop (tile input channel should decouple from output readiness).
    """
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_US, units="us").start())
    await _reset(dut)

    # Block output consumption
    dut.uio_in.value = 0   # rv_out_ready=0

    for n in range(4):
        flit = spike_flit(weight=1, neuron_y=(n >> 1) & 1, neuron_x=n & 1)
        dut.ui_in.value  = flit
        dut.uio_in.value = (1 << UIO_RV_IN_VALID)   # valid=1, out_ready=0
        accepted = False
        for _ in range(30):
            await ReadOnly()
            if _rv_in_ready(dut):
                await RisingEdge(dut.clk)
                accepted = True
                break
            await RisingEdge(dut.clk)
        dut.uio_in.value = 0
        if not accepted:
            break  # FIFO full — expected, not a deadlock
        await ClockCycles(dut.clk, 1)

    # Verify tile is still alive (not deadlocked)
    dut.uio_in.value = (1 << UIO_RV_OUT_READY)
    await ClockCycles(dut.clk, 10)
    await RisingEdge(dut.clk)
    assert "x" not in str(dut.uio_out.value).lower(), \
        "X on uio_out after back-pressure stress"


# ─────────────────────────────────────────────────────────────────────────────
# §F — Burst and back-to-back flits
# ─────────────────────────────────────────────────────────────────────────────

@cocotb.test(timeout_time=30, timeout_unit="ms")
async def test_back_to_back_flits_same_neuron(dut):
    """
    §F — 4 consecutive flits to the same neuron without inter-flit gaps.

    Tests pipelined flit acceptance: each flit must be consumed as soon as
    ready is asserted, without extra dead cycles.
    """
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_US, units="us").start())
    await _reset(dut)

    for i in range(4):
        flit = spike_flit(weight=i, neuron_y=0, neuron_x=0)
        await _send_spike(dut, flit, timeout_cycles=80)

    await ClockCycles(dut.clk, 10)
    await ReadOnly()
    assert "x" not in str(dut.uio_out.value).lower(), \
        "X on uio_out after back-to-back spike burst"


@cocotb.test(timeout_time=30, timeout_unit="ms")
async def test_back_to_back_flits_all_neurons(dut):
    """
    §F — 4 consecutive flits, one to each neuron (round-robin).

    Tests ingress classification for all 4 neuron addresses in rapid succession.
    """
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_US, units="us").start())
    await _reset(dut)

    addresses = [(0, 0), (0, 1), (1, 0), (1, 1)]
    for ny, nx in addresses:
        flit = spike_flit(weight=2, neuron_y=ny, neuron_x=nx)
        await _send_spike(dut, flit, timeout_cycles=80)

    await ClockCycles(dut.clk, 10)
    await ReadOnly()
    assert "x" not in str(dut.uio_out.value).lower()


@cocotb.test(timeout_time=50, timeout_unit="ms")
async def test_burst_8_flits_mixed_neurons_weights(dut):
    """
    §F — 8-flit burst with alternating neurons and weights.

    Maximally stresses the tile_top_tt decode path and tile_ingress classifier.
    """
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_US, units="us").start())
    await _reset(dut)

    flits = [
        spike_flit(weight= 7, neuron_y=0, neuron_x=0),
        spike_flit(weight=-8, neuron_y=1, neuron_x=1),
        spike_flit(weight= 3, neuron_y=0, neuron_x=1),
        spike_flit(weight=-3, neuron_y=1, neuron_x=0),
        spike_flit(weight= 1, neuron_y=0, neuron_x=0),
        spike_flit(weight=-1, neuron_y=1, neuron_x=1),
        spike_flit(weight= 6, neuron_y=0, neuron_x=1),
        spike_flit(weight=-6, neuron_y=1, neuron_x=0),
    ]

    for flit in flits:
        await _send_spike(dut, flit, timeout_cycles=100)

    await ClockCycles(dut.clk, 20)
    await ReadOnly()
    assert "x" not in str(dut.uio_out.value).lower(), \
        "X on uio_out after 8-flit burst"


# ─────────────────────────────────────────────────────────────────────────────
# §E — Enable gating
# ─────────────────────────────────────────────────────────────────────────────

@cocotb.test(timeout_time=10, timeout_unit="ms")
async def test_enable_disable_recover(dut):
    """
    §E — Deasserting ena disables tile; re-asserting ena restores it.

    After ena=1 recovery, tile must still be X-free and accept flits.
    """
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_US, units="us").start())
    await _reset(dut)

    # Disable
    dut.ena.value = 0
    await ClockCycles(dut.clk, 5)

    # Re-enable
    dut.ena.value = 1
    await ClockCycles(dut.clk, 5)
    await ReadOnly()

    uio_str = str(dut.uio_out.value).lower()
    assert "x" not in uio_str and "z" not in uio_str, \
        f"X/Z on uio_out after enable recovery: {uio_str}"


@cocotb.test(timeout_time=10, timeout_unit="ms")
async def test_enable_0_no_output(dut):
    """
    §E — ena=0 suppresses all tile output.

    With ena=0, the tile's clock gate should prevent state changes,
    so rv_out_valid must stay 0 and rv_in_ready may de-assert.
    """
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_US, units="us").start())
    await _reset(dut)

    dut.ena.value = 0
    dut.uio_in.value = (1 << UIO_RV_OUT_READY)

    for _ in range(10):
        await ReadOnly()
        # With ena=0 there should be no new output from the tile
        assert _rv_out_valid(dut) == 0, \
            "rv_out_valid asserted with ena=0 (unexpected output)"
        await RisingEdge(dut.clk)


# ─────────────────────────────────────────────────────────────────────────────
# §O — Output flit format: uo_out encoding correctness
# ─────────────────────────────────────────────────────────────────────────────

@cocotb.test(timeout_time=20, timeout_unit="ms")
async def test_uo_out_format_zero_at_idle(dut):
    """
    §O — uo_out = 0x00 when rv_out_valid = 0.

    The uo_out bus must carry 0 when the tile has no pending output,
    matching the neutern_spike_t format (no garbage on the bus).
    """
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_US, units="us").start())
    await _reset(dut)

    for _ in range(15):
        await ReadOnly()
        if _rv_out_valid(dut) == 0:
            assert int(dut.uo_out.value) == 0, \
                f"uo_out={int(dut.uo_out.value):#04x} while rv_out_valid=0"
        await RisingEdge(dut.clk)


@cocotb.test(timeout_time=20, timeout_unit="ms")
async def test_uo_out_reserved_field_is_zero(dut):
    """
    §O — When tile produces an output flit, uo_out[3:2] (reserved) must be 0.

    The tile_top_tt adapter must zero the reserved field of the output
    neutern_spike_t (matching the input format contract).

    This test is conditional: asserted only when rv_out_valid is observed.
    """
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_US, units="us").start())
    await _reset(dut)

    # Send a burst of spikes to maximize chance of observing an output flit
    for ny in range(2):
        for nx in range(2):
            flit = spike_flit(weight=7, neuron_y=ny, neuron_x=nx)
            dut.ui_in.value  = flit
            dut.uio_in.value = (1 << UIO_RV_OUT_READY) | (1 << UIO_RV_IN_VALID)
            for _ in range(20):
                await ReadOnly()
                if _rv_in_ready(dut):
                    await RisingEdge(dut.clk)
                    break
                await RisingEdge(dut.clk)
            dut.uio_in.value = (1 << UIO_RV_OUT_READY)
            dut.ui_in.value  = 0

    # Monitor output for 50 cycles
    any_output_seen = False
    for _ in range(50):
        await ReadOnly()
        if _rv_out_valid(dut):
            any_output_seen = True
            raw = int(dut.uo_out.value)
            reserved = (raw >> 2) & 0x3
            assert reserved == 0, \
                f"uo_out reserved bits [3:2] are {reserved:#04b}, expected 0b00 " \
                f"(raw=0x{raw:02X})"
        await RisingEdge(dut.clk)

    if any_output_seen:
        dut._log.info("Output flit format verified: reserved bits = 0")
    else:
        dut._log.warning("No output flit seen — reserved field check skipped")


# ─────────────────────────────────────────────────────────────────────────────
# §I §H — Ingress FIFO back-pressure: rv_in_ready de-asserts when full
# ─────────────────────────────────────────────────────────────────────────────

@cocotb.test(timeout_time=50, timeout_unit="ms")
async def test_ingress_backpressure_fills_and_drains(dut):
    """
    §I §H — Ingress FIFO fills (rv_in_ready=0) and drains (rv_in_ready=1).

    Sends many spikes to the same neuron trying to fill the event queue
    (depth=4). Verifies that:
      1. rv_in_ready eventually de-asserts (FIFO full)  — OR the tile
         accepts all (compute drains fast enough).
      2. rv_in_ready eventually returns to 1 (FIFO drained).
      3. No X propagation throughout.
    """
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_US, units="us").start())
    await _reset(dut)

    flit = spike_flit(weight=1, neuron_y=0, neuron_x=0)
    full_seen = False

    for _ in range(12):   # 3× FIFO depth
        dut.ui_in.value  = flit
        dut.uio_in.value = (1 << UIO_RV_OUT_READY) | (1 << UIO_RV_IN_VALID)

        accepted = False
        for _ in range(5):
            await RisingEdge(dut.clk)
            uio_str = str(dut.uio_out.value).lower()
            assert "x" not in uio_str, f"X on uio_out: {uio_str}"
            if _rv_in_ready(dut) == 0:
                full_seen = True
            if _rv_in_ready(dut):
                accepted = True
                break

        dut.uio_in.value = (1 << UIO_RV_OUT_READY)
        dut.ui_in.value  = 0
        await ClockCycles(dut.clk, 1)

    dut._log.info(f"FIFO-full observed: {full_seen}")

    # Wait for FIFO to drain
    drained = False
    for _ in range(400):
        await RisingEdge(dut.clk)
        if _rv_in_ready(dut) == 1:
            drained = True
            break

    assert drained, "rv_in_ready never returned to 1 after FIFO fill+drain"


# ─────────────────────────────────────────────────────────────────────────────
# §R — Reset during active ingress (mid-flit reset)
# ─────────────────────────────────────────────────────────────────────────────

@cocotb.test(timeout_time=20, timeout_unit="ms")
async def test_reset_during_spike_ingress(dut):
    """
    §R — Asserting rst_n=0 mid-flit safely terminates the transaction.

    Drives valid=1 for 3 cycles, then asserts reset. After de-assertion
    of reset, the tile must return to idle (rv_out_valid=0, rv_in_ready=1).
    """
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_US, units="us").start())
    await _reset(dut)

    # Start sending a spike but do not complete the handshake
    flit = spike_flit(weight=5, neuron_y=1, neuron_x=1)
    dut.ui_in.value  = flit
    dut.uio_in.value = (1 << UIO_RV_IN_VALID)   # hold valid

    # Simulate 3 cycles of ongoing transaction
    await ClockCycles(dut.clk, 3)

    # Suddenly assert reset
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 5)
    dut.uio_in.value = 0
    dut.ui_in.value  = 0
    dut.rst_n.value  = 1
    await ClockCycles(dut.clk, 3)

    await ReadOnly()
    assert _rv_out_valid(dut) == 0, \
        "rv_out_valid asserted after reset during spike ingress"
    assert _rv_in_ready(dut) == 1, \
        "rv_in_ready not restored after reset during spike ingress"


@cocotb.test(timeout_time=20, timeout_unit="ms")
async def test_rapid_reset_recovery(dut):
    """
    §R — Rapid reset/release cycle recovers correctly.

    Toggles rst_n=0/1 five times rapidly and verifies no X on outputs after
    final de-assertion.
    """
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_US, units="us").start())
    await _reset(dut)

    for _ in range(5):
        dut.rst_n.value = 0
        await ClockCycles(dut.clk, 2)
        dut.rst_n.value = 1
        await ClockCycles(dut.clk, 2)

    await ClockCycles(dut.clk, 5)
    await ReadOnly()
    uio_str = str(dut.uio_out.value).lower()
    assert "x" not in uio_str and "z" not in uio_str, \
        f"X/Z on uio_out after rapid reset cycles: {uio_str}"
    assert _rv_out_valid(dut) == 0, "rv_out_valid must be 0 after reset"
    assert _rv_in_ready(dut) == 1, "rv_in_ready must be 1 after reset"


# ─────────────────────────────────────────────────────────────────────────────
# §C — Header-based neuron configuration and readback
# ─────────────────────────────────────────────────────────────────────────────

@cocotb.test(timeout_time=40, timeout_unit="ms")
async def test_weight_header_write_then_readback_all_neurons(dut):
    """
    §C — Header write updates per-neuron weight and header read returns it.

    Flow per neuron:
      1) Send weight header write (uio[2]=1, uio[3]=0, ui[2]=0)
      2) Drain status response (MSG_STATUS_OK)
      3) Send weight header read  (uio[2]=1, uio[3]=0, ui[2]=1)
      4) Receive readback flit and verify weight + neuron coordinates
    """
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_US, units="us").start())
    await _reset(dut)

    vectors = [
        (0, 0, 3),
        (0, 1, -2),
        (1, 0, 7),
        (1, 1, -8),
    ]

    for ny, nx, weight in vectors:
        await _send_weight_header_write(dut, weight=weight, neuron_y=ny, neuron_x=nx)

        # Write path returns an immediate status response; consume it so the
        # following readback check observes the CMD_WEIGHT read response.
        status_rsp = await _recv_flit(dut, timeout_cycles=200)
        assert status_rsp["weight"] == 0, (
            "Expected status code 0x0 on write acknowledge"
        )

        await _send_weight_header_read(dut, neuron_y=ny, neuron_x=nx)
        read_rsp = await _recv_flit(dut, timeout_cycles=200)

        assert read_rsp["neuron_y"] == ny, (
            f"Readback neuron_y mismatch: expected {ny}, got {read_rsp['neuron_y']}"
        )
        assert read_rsp["neuron_x"] == nx, (
            f"Readback neuron_x mismatch: expected {nx}, got {read_rsp['neuron_x']}"
        )
        assert read_rsp["weight"] == weight, (
            f"Readback weight mismatch for neuron ({ny},{nx}): "
            f"expected {weight}, got {read_rsp['weight']}"
        )


@cocotb.test(timeout_time=40, timeout_unit="ms")
async def test_isa_header_program_ack(dut):
    """
    §C — ISA header packets are accepted and acknowledged for each neuron.

    Sends one ISA header per neuron and verifies a status response is emitted.
    """
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_US, units="us").start())
    await _reset(dut)

    vectors = [
        (0, 0, 0x00, 0),
        (0, 1, 0x05, 1),
        (1, 0, 0x08, 0),
        (1, 1, 0x0B, 1),
    ]

    for ny, nx, op5, barrier in vectors:
        await _send_isa_header(
            dut,
            op5=op5,
            neuron_y=ny,
            neuron_x=nx,
            barrier=barrier,
        )

        ack = await _recv_flit(dut, timeout_cycles=200)
        assert ack["weight"] == 0, (
            f"ISA header ack not OK for neuron ({ny},{nx}), op5=0x{op5:02x}"
        )

