// tile_top_formal.sv
// ──────────────────────────────────────────────────────────────────────────
// Formal harness for tile_top (neutern 4-neuron configuration).
//
// Architecture:
//   tile_top has rv_if interface ports which cause Yosys RTLIL name
//   collisions when instantiated from a module-level harness. This harness
//   uses a flat-port shadow wrapper (tile_top_flat) that converts the two
//   rv_if ports to plain logic vectors, then wraps tile_top.
//   tile_top_flat is the formal top.
//
// Properties proven (each gated by a task define):
//
//  RESET_QUIESCENCE          After reset deasserts, rv_out must be invalid
//                            for at least one cycle.
//
//  ENA_DISABLED_NO_OUTPUT    While ena=0, tile must not assert rv_out.valid.
//
//  HOST_PRIORITY_MUX         When both host and noc out are valid, rv_out
//                            carries the host payload, not the noc payload.
//
//  NOC_YIELDS_TO_HOST        rv_noc_out_ready is de-asserted whenever
//                            rv_host_out_valid is high and rv_out.ready=1.
//
//  OUTPUT_VALID_STABLE_UNTIL_READY
//                            Once rv_out.valid asserted, payload stays stable
//                            until rv_out.ready (backpressure contract).
//
//  HOST_PAYLOAD_STABLE_ON_BACKPRESSURE
//                            Host channel payload stable under backpressure.
//
//  NOC_PAYLOAD_STABLE_ON_BACKPRESSURE
//                            NoC channel payload stable under backpressure.
//
// ──────────────────────────────────────────────────────────────────────────
`default_nettype none

// ── Flat wrapper ─────────────────────────────────────────────────────────
// Converts tile_top's rv_if ports to plain logic vectors.
// This avoids the slang→Yosys interface-flattening name collision.
module tile_top_flat #(
    parameter int unsigned NEURONS_PER_TILE         = 4,
    parameter int unsigned WORKER_CORES_PER_TILE    = 1,
    parameter int unsigned FANOUT_POOL_DEPTH         = 4,
    parameter int unsigned TILE_BANK_MEM_STYLE       = 0,
    parameter int unsigned LOGICAL_EVENT_QUEUE_DEPTH = 4,
    parameter int unsigned INGRESS_QUEUE_DEPTH       = 4,
    parameter int unsigned HOST_OUTPUT_FIFO_DEPTH    = 4,
    parameter int unsigned FANOUT_QUEUE_DEPTH        = 4,
    parameter int unsigned MESSAGE_W                 = 53
) (
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  ena,

    // rv_in flat
    input  wire                  rv_in_valid,
    input  wire  [MESSAGE_W-1:0] rv_in_payload,
    output wire                  rv_in_ready,

    // rv_out flat
    output wire                  rv_out_valid,
    output wire  [MESSAGE_W-1:0] rv_out_payload,
    input  wire                  rv_out_ready,

`ifdef FORMAL
    // Observability passthrough
    output wire                  rv_in_ready_obs,
    output wire                  rv_out_valid_obs,
    output wire  [MESSAGE_W-1:0] rv_out_payload_obs,
    output wire                  rv_host_out_valid_obs,
    output wire  [MESSAGE_W-1:0] rv_host_out_payload_obs,
    output wire                  rv_noc_out_valid_obs,
    output wire  [MESSAGE_W-1:0] rv_noc_out_payload_obs,
    output wire                  rv_host_out_ready_obs,
    output wire                  rv_noc_out_ready_obs,
    output wire                  tile_graph_state_clear_r_obs,
`endif

    output wire                  _unused_dummy  // keep port list non-empty
);
    assign _unused_dummy = 1'b0;

    rv_if #(.WIDTH(MESSAGE_W)) u_rv_in_if ();
    rv_if #(.WIDTH(MESSAGE_W)) u_rv_out_if();

    assign u_rv_in_if.valid      = rv_in_valid;
    assign u_rv_in_if.rv_payload = rv_in_payload;
    assign rv_in_ready           = u_rv_in_if.ready;
    assign u_rv_out_if.ready     = rv_out_ready;
    assign rv_out_valid          = u_rv_out_if.valid;
    assign rv_out_payload        = u_rv_out_if.rv_payload;

    tile_top #(
        .NEURONS_PER_TILE        (NEURONS_PER_TILE),
        .WORKER_CORES_PER_TILE   (WORKER_CORES_PER_TILE),
        .FANOUT_POOL_DEPTH       (FANOUT_POOL_DEPTH),
        .TILE_BANK_MEM_STYLE     (TILE_BANK_MEM_STYLE),
        .LOGICAL_EVENT_QUEUE_DEPTH(LOGICAL_EVENT_QUEUE_DEPTH),
        .INGRESS_QUEUE_DEPTH     (INGRESS_QUEUE_DEPTH),
        .HOST_OUTPUT_FIFO_DEPTH  (HOST_OUTPUT_FIFO_DEPTH),
        .FANOUT_QUEUE_DEPTH      (FANOUT_QUEUE_DEPTH),
        .MESSAGE_W               (MESSAGE_W)
    ) u_dut (
        .clk     (clk),
        .rst_n   (rst_n),
        .ena     (ena),
        .rv_in   (u_rv_in_if),
        .rv_out  (u_rv_out_if)
`ifdef FORMAL
        ,
        .rv_in_ready_obs        (rv_in_ready_obs),
        .rv_out_valid_obs       (rv_out_valid_obs),
        .rv_out_payload_obs     (rv_out_payload_obs),
        .rv_host_out_valid_obs  (rv_host_out_valid_obs),
        .rv_host_out_payload_obs(rv_host_out_payload_obs),
        .rv_noc_out_valid_obs   (rv_noc_out_valid_obs),
        .rv_noc_out_payload_obs (rv_noc_out_payload_obs),
        .rv_host_out_ready_obs          (rv_host_out_ready_obs),
        .rv_noc_out_ready_obs           (rv_noc_out_ready_obs),
        .tile_graph_state_clear_r_obs   (tile_graph_state_clear_r_obs)
`endif
    );

endmodule


// ── Formal harness top ────────────────────────────────────────────────────
module tile_top_formal #(
    parameter int unsigned NEURONS_PER_TILE         = 4,
    parameter int unsigned WORKER_CORES_PER_TILE    = 1,
    parameter int unsigned FANOUT_POOL_DEPTH         = 4,
    parameter int unsigned TILE_BANK_MEM_STYLE       = 0,
    parameter int unsigned LOGICAL_EVENT_QUEUE_DEPTH = 4,
    parameter int unsigned INGRESS_QUEUE_DEPTH       = 4,
    parameter int unsigned HOST_OUTPUT_FIFO_DEPTH    = 4,
    parameter int unsigned FANOUT_QUEUE_DEPTH        = 4,
    parameter int unsigned MESSAGE_W                 = 53
) (
    input logic clk,
    input logic rst_n
);

    logic                  ena;
    logic                  rv_in_valid;
    logic [MESSAGE_W-1:0]  rv_in_payload;
    logic                  rv_in_ready;
    logic                  rv_out_valid;
    logic [MESSAGE_W-1:0]  rv_out_payload;
    logic                  rv_out_ready;

`ifdef FORMAL
    logic                  rv_in_ready_obs;
    logic                  rv_out_valid_obs;
    logic [MESSAGE_W-1:0]  rv_out_payload_obs;
    logic                  rv_host_out_valid_obs;
    logic [MESSAGE_W-1:0]  rv_host_out_payload_obs;
    logic                  rv_noc_out_valid_obs;
    logic [MESSAGE_W-1:0]  rv_noc_out_payload_obs;
    logic                  rv_host_out_ready_obs;
    logic                  rv_noc_out_ready_obs;
    logic                  tile_graph_state_clear_r_obs;
    wire                   _dummy_obs;
`endif

    tile_top_flat #(
        .NEURONS_PER_TILE        (NEURONS_PER_TILE),
        .WORKER_CORES_PER_TILE   (WORKER_CORES_PER_TILE),
        .FANOUT_POOL_DEPTH       (FANOUT_POOL_DEPTH),
        .TILE_BANK_MEM_STYLE     (TILE_BANK_MEM_STYLE),
        .LOGICAL_EVENT_QUEUE_DEPTH(LOGICAL_EVENT_QUEUE_DEPTH),
        .INGRESS_QUEUE_DEPTH     (INGRESS_QUEUE_DEPTH),
        .HOST_OUTPUT_FIFO_DEPTH  (HOST_OUTPUT_FIFO_DEPTH),
        .FANOUT_QUEUE_DEPTH      (FANOUT_QUEUE_DEPTH),
        .MESSAGE_W               (MESSAGE_W)
    ) u_wrap (
        .clk            (clk),
        .rst_n          (rst_n),
        .ena            (ena),
        .rv_in_valid    (rv_in_valid),
        .rv_in_payload  (rv_in_payload),
        .rv_in_ready    (rv_in_ready),
        .rv_out_valid   (rv_out_valid),
        .rv_out_payload (rv_out_payload),
        .rv_out_ready   (rv_out_ready),
`ifdef FORMAL
        .rv_in_ready_obs        (rv_in_ready_obs),
        .rv_out_valid_obs       (rv_out_valid_obs),
        .rv_out_payload_obs     (rv_out_payload_obs),
        .rv_host_out_valid_obs  (rv_host_out_valid_obs),
        .rv_host_out_payload_obs(rv_host_out_payload_obs),
        .rv_noc_out_valid_obs   (rv_noc_out_valid_obs),
        .rv_noc_out_payload_obs (rv_noc_out_payload_obs),
        .rv_host_out_ready_obs         (rv_host_out_ready_obs),
        .rv_noc_out_ready_obs          (rv_noc_out_ready_obs),
        .tile_graph_state_clear_r_obs  (tile_graph_state_clear_r_obs),
`endif
        ._unused_dummy  (_dummy_obs)
    );

`ifdef FORMAL

    // ── Reset / past-valid tracking ─────────────────────────────────────
    // f_past_valid uses async-reset so clk2fflogic infers \init=0 for it,
    // meaning it is constrained to 0 at step 0 without any explicit initial
    // assume.  Properties gated on f_past_valid therefore cannot fire at
    // step 0 (the unconstrained initial state).
    logic f_past_valid;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) f_past_valid <= 1'b0;
        else        f_past_valid <= 1'b1;
    end

    // ── First-cycle reset sequencer ──────────────────────────────────────
    // f_first_cycle starts at 1 (inline initialiser → INIT=1 attribute).
    // setundef -undriven -anyseq does not override driven registers with
    // INIT attributes, so f_first_cycle=1 at the initial SMT state.
    // It clears to 0 after the first posedge, so the assume below forces
    // rst_n=0 exactly at the first clock edge, guaranteeing every
    // synchronous FF sees at least one active-low reset before any
    // property is checked.
    logic f_first_cycle = 1'b1;
    always @(posedge clk) f_first_cycle <= 1'b0;
    always @(posedge clk) if (f_first_cycle) assume(!rst_n);

    // ── Environment assumptions ─────────────────────────────────────────
    always @(posedge clk) begin
        if (!rst_n) assume(!ena);
    end

    // rv_in backpressure: once valid asserted, hold until ready
    always @(posedge clk) begin
        if (f_past_valid && $past(rv_in_valid) && !$past(rv_in_ready)) begin
            assume(rv_in_valid   == $past(rv_in_valid));
            assume(rv_in_payload == $past(rv_in_payload));
        end
    end

    // ── ena and rst_n stability (once-high-stays-high) ──────────────────
    // f_ena_seen / f_rst_n_seen start at 0 (INIT=0 from inline initialiser;
    // setundef -anyseq does not override driven registers with INIT attrs).
    // They go high on the first posedge where ena/rst_n is 1, after which
    // the assumptions permanently prevent ena/rst_n from dropping — even
    // across mid-run rst_n pulses where f_past_valid temporarily goes 0.
    logic f_ena_seen  = 1'b0;
    logic f_rst_n_seen = 1'b0;
    always @(posedge clk) begin
        if (ena)   f_ena_seen  <= 1'b1;
        if (rst_n) f_rst_n_seen <= 1'b1;
    end
    always @(posedge clk) begin
        if (f_ena_seen)  assume(ena);
        if (f_rst_n_seen) assume(rst_n);
    end

    // ── RESET_QUIESCENCE ────────────────────────────────────────────────
    `ifdef RESET_QUIESCENCE
    always @(posedge clk) begin
        if (f_past_valid && !$past(rst_n) && rst_n) begin
            assert(!rv_out_valid);
        end
    end
    `endif

    // ── ENA_DISABLED_NO_OUTPUT ──────────────────────────────────────────
    `ifdef ENA_DISABLED_NO_OUTPUT
    always @(posedge clk) begin
        if (rst_n && !ena) begin
            assert(!rv_out_valid_obs);
        end
    end
    `endif

    // ── HOST_PRIORITY_MUX ───────────────────────────────────────────────
    `ifdef HOST_PRIORITY_MUX
    always @(posedge clk) begin
        if (rv_host_out_valid_obs && rv_noc_out_valid_obs) begin
            assert(rv_out_payload_obs == rv_host_out_payload_obs);
        end
        if (rv_host_out_valid_obs || rv_noc_out_valid_obs) begin
            assert(rv_out_valid_obs);
        end
    end
    `endif

    // ── NOC_YIELDS_TO_HOST ──────────────────────────────────────────────
    `ifdef NOC_YIELDS_TO_HOST
    always @(posedge clk) begin
        if (rv_host_out_valid_obs && rv_out_ready) begin
            assert(!rv_noc_out_ready_obs);
        end
    end
    `endif

    // ── OUTPUT_VALID_STABLE_UNTIL_READY ─────────────────────────────────
    `ifdef OUTPUT_VALID_STABLE_UNTIL_READY
    always @(posedge clk) begin
        if (f_past_valid && !$past(tile_graph_state_clear_r_obs) &&
                $past(rv_out_valid) && !$past(rv_out_ready)) begin
            assert(rv_out_valid);
            assert(rv_out_payload == $past(rv_out_payload));
        end
    end
    `endif

    // ── HOST_PAYLOAD_STABLE_ON_BACKPRESSURE ─────────────────────────────
    `ifdef HOST_PAYLOAD_STABLE_ON_BACKPRESSURE
    always @(posedge clk) begin
        if (f_past_valid && !$past(tile_graph_state_clear_r_obs) &&
                $past(rv_host_out_valid_obs) && !$past(rv_host_out_ready_obs)) begin
            assert(rv_host_out_valid_obs);
            assert(rv_host_out_payload_obs == $past(rv_host_out_payload_obs));
        end
    end
    `endif

    // ── NOC_PAYLOAD_STABLE_ON_BACKPRESSURE ──────────────────────────────
    `ifdef NOC_PAYLOAD_STABLE_ON_BACKPRESSURE
    always @(posedge clk) begin
        if (f_past_valid && $past(rv_noc_out_valid_obs) && !$past(rv_noc_out_ready_obs)) begin
            assert(rv_noc_out_valid_obs);
            assert(rv_noc_out_payload_obs == $past(rv_noc_out_payload_obs));
        end
    end
    `endif

    // ── Reachability covers ──────────────────────────────────────────────
    always @(posedge clk) begin
        cover(rv_out_valid_obs && rv_out_ready);
        cover(rv_host_out_valid_obs);
        cover(rv_noc_out_valid_obs);
    end

`endif // FORMAL

endmodule

`default_nettype wire
