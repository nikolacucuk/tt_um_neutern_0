# Neuron Compute IP

`neuron_compute` contains the lightweight execution engines used by packed
tiles. A compute worker has no permanent neuron identity; it executes a
logical-neuron context snapshot supplied by the tile scheduler and returns an
updated context plus any emit/weight-commit side effects.

## Key Files

- `src/neuron_compute_core.sv` — Top-level worker FSM; handles dispatch handshake, ucode fetch, and result commit
- `src/neuron_exec.sv` — Combinational execution unit; decodes compact 12-bit instructions and executes opcodes

## Instruction Encoding

12-bit compact format: `[11:7]=op (5b) | [6:4]=rd (3b) | [3]=sign | [2:0]=k`

**Tag gate**: `instr_word[2]` (k[2]) enables barrier-gating. When set and the event tag is DATA (0), the instruction is silently skipped. When the event tag is BARRIER (1), the instruction executes normally.

**OP_INTEG dual-use**: For OP_INTEG, k[2] also controls accumulator auto-clear after integration. Using k=4 (instr_word[2]=1) both gates on barrier AND clears acc after vm+=acc.

## Supported Opcodes

| Opcode | Value | Description |
|--------|-------|-------------|
| OP_LDI | 0 | Load 4-bit signed immediate into RF[rd] |
| OP_RECV | 1 | Capture event tag into last_tag context field |
| OP_ACCUM_W | 2 | acc += signed weight from event payload |
| OP_INTEG | 4 | vm += acc; k[2]=1 also clears acc |
| OP_SPIKE_IF_GE | 5 | spike_flag = (vm >= threshold) |
| OP_RESET | 6 | Reset vm on spike: mode[1:0] from k[1:0] |
| OP_EMIT | 8 | Emit output packet if not already pending |

## Verification

### Simulation (cocotb + Verilator)

```bash
cd src/neuron_compute/test && make SIM=verilator
```

56 tests covering all opcodes exhaustively: **TESTS=56 PASS=56 FAIL=0**

Test categories:
- OP_LDI: all 16 imm4 values × 3 registers (barrier for k[2]=1 cases)
- OP_RECV: tag capture and last_tag round-trip
- OP_ACCUM_W: all 16 weight values including saturation
- OP_INTEG: add+saturate, acc-clear, tag-gating
- OP_SPIKE_IF_GE: exhaustive 4-bit vm × threshold sweep (256 combos)
- OP_RESET: all 4 reset modes
- OP_EMIT: gated/ungated emit, back-pressure
- Tag gate: skip-on-data and execute-on-barrier for INTEG, SPIKE_IF_GE, EMIT
- IF neuron: data accumulate, barrier spike/reset/emit, multi-event sequences
- Infrastructure: zero-len dispatch, RF round-trip, result handshake hold, ucode ptr offset, back-to-back, logical_idx, graph_state_clear, rst_n, ena

### Formal

```bash
python tools/flows/tool_flow.py formal --project neuron_compute
```

