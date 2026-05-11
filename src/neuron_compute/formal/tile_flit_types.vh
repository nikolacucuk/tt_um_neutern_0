`ifndef TILE_FLIT_TYPES_VH
`define TILE_FLIT_TYPES_VH

// Minimal formal-time type shim for neuron_compute_core proofs.
// Uses literal widths to stay compatible with yosys built-in frontend.

typedef struct packed {
    logic [31:0]                  rf_state_flat;
    logic [4:0]                   last_sid;
    logic [1:0]                   last_tag;
    logic [5:0]                   last_time;
    logic                       cmp_ge;
    logic                       cmp_eq;
    logic                       spike_flag;
} neuron_exec_ctx_t;

typedef struct packed {
    logic [4:0]                   sid;
    logic [1:0]                   tag;
    logic [5:0]                   spike_time;
    logic signed [7:0]            weight;
} neuron_worker_event_t;

typedef struct packed {
    logic                       valid;
    logic [4:0]                   sid;
    logic signed [7:0]            value;
} neuron_weight_commit_t;

typedef struct packed {
    logic                       valid;
    logic [7:0]                   data;
    logic [5:0]                   event_time;
} neuron_emit_t;

typedef struct packed {
    logic [9:0]                   logical_idx;
    neuron_exec_ctx_t           ctx;
    neuron_worker_event_t       in_event;
    logic [4:0]                   ucode_ptr;
    logic [4:0]                   ucode_len;
} neuron_worker_start_t;

typedef struct packed {
    logic [9:0]                   logical_idx;
    logic [4:0]                   word_index;
} neuron_ucode_req_t;

typedef struct packed {
    logic [15:0]                word;
} neuron_ucode_rsp_t;

typedef struct packed {
    logic [9:0]                   logical_idx;
    neuron_exec_ctx_t           ctx;
    neuron_weight_commit_t      weight_commit;
    neuron_emit_t               emit;
} neuron_worker_result_t;

`endif // TILE_FLIT_TYPES_VH
