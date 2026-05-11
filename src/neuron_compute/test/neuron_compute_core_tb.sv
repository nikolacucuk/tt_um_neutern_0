// neuron_compute_core_tb.sv
// ─────────────────────────────────────────────────────────────────────────────
// Flat-port testbench wrapper for neuron_compute_core.
//
// Note: cocotb/Verilator cannot directly drive rv_if parameters from Python, so
// this wrapper instantiates rv_if channels with the correct WIDTH values
// (derived from tile_pkg constants) and exposes them as flat I/O ports.
//
// ASCII block diagram:
//
//   ┌──────────────────────────────────────────────────────────────────────┐
//   │  neuron_compute_core_tb                                              │
//   │                                                                      │
//   │  flat worker_start ──►  rv_if(START_W) ──► neuron_compute_core      │
//   │  flat ucode_read   ◄──  rv_if(REQ_W)  ◄──  │                       │
//   │  flat ucode_rsp    ──►  rv_if(RSP_W)  ──►  │                       │
//   │  flat worker_result◄──  rv_if(RESULT_W)◄── │                       │
//   └──────────────────────────────────────────────────────────────────────┘
// ─────────────────────────────────────────────────────────────────────────────

`default_nettype none

`include "tile_flit_types.vh"

module neuron_compute_core_tb #(
    parameter int unsigned LOGICAL_IDX_W = NEURON_IDX_W
) (
    input  wire clk,
    input  wire rst_n,
    input  wire ena,
    input  wire graph_state_clear,

    // ── worker_start (rx) flat ports ────────────────────────────────────────
    input  wire                               worker_start_valid,
    output wire                               worker_start_ready,
    input  wire [NEURON_WORKER_START_W-1:0]   worker_start_payload,

    // ── ucode_read (tx) flat ports ──────────────────────────────────────────
    output wire                               ucode_read_valid,
    input  wire                               ucode_read_ready,
    output wire [NEURON_UCODE_REQ_W-1:0]      ucode_read_payload,

    // ── ucode_rsp (rx) flat ports ───────────────────────────────────────────
    input  wire                               ucode_rsp_valid,
    output wire                               ucode_rsp_ready,
    input  wire [NEURON_UCODE_RSP_W-1:0]      ucode_rsp_payload,

    // ── worker_result (tx) flat ports ───────────────────────────────────────
    output wire                               worker_result_valid,
    input  wire                               worker_result_ready,
    output wire [NEURON_WORKER_RESULT_W-1:0]  worker_result_payload
);

    // ── Internal rv_if instances with correct WIDTH ─────────────────────────
    rv_if #(.WIDTH(NEURON_WORKER_START_W))  worker_start_if  ();
    rv_if #(.WIDTH(NEURON_UCODE_REQ_W))     ucode_read_if    ();
    rv_if #(.WIDTH(NEURON_UCODE_RSP_W))     ucode_rsp_if     ();
    rv_if #(.WIDTH(NEURON_WORKER_RESULT_W)) worker_result_if ();

    // ── Flat port ↔ rv_if wiring ────────────────────────────────────────────
    // worker_start: driven by testbench (tx direction), NCC is rx.
    assign worker_start_if.valid      = worker_start_valid;
    assign worker_start_ready         = worker_start_if.ready;
    assign worker_start_if.rv_payload = worker_start_payload;

    // ucode_read: driven by NCC (tx direction), read by testbench.
    assign ucode_read_valid           = ucode_read_if.valid;
    assign ucode_read_if.ready        = ucode_read_ready;
    assign ucode_read_payload         = ucode_read_if.rv_payload;

    // ucode_rsp: driven by testbench (tx direction), NCC is rx.
    assign ucode_rsp_if.valid         = ucode_rsp_valid;
    assign ucode_rsp_ready            = ucode_rsp_if.ready;
    assign ucode_rsp_if.rv_payload    = ucode_rsp_payload;

    // worker_result: driven by NCC (tx direction), read by testbench.
    assign worker_result_valid        = worker_result_if.valid;
    assign worker_result_if.ready     = worker_result_ready;
    assign worker_result_payload      = worker_result_if.rv_payload;

    // ── DUT instantiation ───────────────────────────────────────────────────
    neuron_compute_core #(
        .LOGICAL_IDX_W(LOGICAL_IDX_W)
    ) u_dut (
        .clk              (clk),
        .rst_n            (rst_n),
        .ena              (ena),
        .graph_state_clear(graph_state_clear),
        .worker_start     (worker_start_if.rx),
        .ucode_read       (ucode_read_if.tx),
        .ucode_rsp        (ucode_rsp_if.rx),
        .worker_result    (worker_result_if.tx)
    );

endmodule

`default_nettype wire
