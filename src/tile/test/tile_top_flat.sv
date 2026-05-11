// tile_top_flat.sv — Flat-signal wrapper around tile_top for cocotb simulation.
//
// Exposes the tile_top rv_in/rv_out channels as flat {valid, payload, ready}
// signals so Verilator can elaborate tile_top as a testbench top-level.
//
// Parameters match the neutern_0 tile configuration:
//   NEURONS_PER_TILE=4, WORKER_CORES_PER_TILE=1, FANOUT_POOL_DEPTH=4,
//   MESSAGE_W=40, LOGICAL_EVENT_QUEUE_DEPTH=4, INGRESS_QUEUE_DEPTH=4
//
// Flat port contract:
//   rv_in_valid   : host drives 1 when sending a packet
//   rv_in_payload : packet bits [MESSAGE_W-1:0]
//   rv_in_ready   : tile asserts 1 when ready to accept
//   rv_out_valid  : tile asserts 1 when packet is on rv_out_payload
//   rv_out_payload: packet bits from tile [MESSAGE_W-1:0]
//   rv_out_ready  : host drives 1 when ready to receive
//
// Excluded from synthesis (simulation-only helper).

`default_nettype none

module tile_top_flat #(
    parameter int unsigned MESSAGE_W                = 40,
    parameter int unsigned NEURONS_PER_TILE         = 4,
    parameter int unsigned WORKER_CORES_PER_TILE    = 1,
    parameter int unsigned FANOUT_POOL_DEPTH        = 4,
    parameter int unsigned LOGICAL_EVENT_QUEUE_DEPTH = 4,
    parameter int unsigned INGRESS_QUEUE_DEPTH      = 4,
    parameter int unsigned HOST_OUTPUT_FIFO_DEPTH   = 4,
    parameter int unsigned FANOUT_QUEUE_DEPTH       = 4,
    parameter int unsigned TILE_COORD_X             = 0,
    parameter int unsigned TILE_COORD_Y             = 0,
    parameter int unsigned LOCAL_Z                  = 0,
    parameter int unsigned TILE_BANK_MEM_STYLE      = 0,
    parameter int unsigned FANOUT_DST_X_W           = 8,
    parameter int unsigned FANOUT_DST_Y_W           = 8,
    parameter int unsigned FANOUT_CORE_ID_W         = 8
) (
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire                  ena,

    // rv_in channel (host → tile)
    input  wire                  rv_in_valid,
    input  wire [MESSAGE_W-1:0]  rv_in_payload,
    output wire                  rv_in_ready,

    // rv_out channel (tile → host)
    output wire                  rv_out_valid,
    output wire [MESSAGE_W-1:0]  rv_out_payload,
    input  wire                  rv_out_ready
);

    // ── Interface instances ───────────────────────────────────────────────
    rv_if #(.WIDTH(MESSAGE_W)) rv_in_if  ();
    rv_if #(.WIDTH(MESSAGE_W)) rv_out_if ();

    // ── Flat → interface wiring (rv_in: host drives tx side) ─────────────
    assign rv_in_if.valid      = rv_in_valid;
    assign rv_in_if.rv_payload = rv_in_payload;
    assign rv_in_ready         = rv_in_if.ready;

    // ── Interface → flat wiring (rv_out: tile drives tx side) ────────────
    assign rv_out_valid        = rv_out_if.valid;
    assign rv_out_payload      = rv_out_if.rv_payload;
    assign rv_out_if.ready     = rv_out_ready;

    // ── Per-test waveform dump (simulation only) ──────────────────────────
    // Enabled when the simulator is invoked with +trace.
    // The output FST path is set via +tracefile=<path>.fst; defaults to
    // waves/tile_top_flat.fst when no +tracefile is given.
    // Usage:   SIM_ARGS="+trace +tracefile=waves/my_test.fst"
`ifndef SYNTHESIS
    initial begin : wave_dump
        string wave_path;
        if ($test$plusargs("trace")) begin
            if (!$value$plusargs("tracefile=%s", wave_path))
                wave_path = "waves/tile_top_flat.fst";
            $dumpfile(wave_path);
            $dumpvars(0, tile_top_flat);
        end
    end
`endif

    // ── tile_top instance ─────────────────────────────────────────────────
    tile_top #(
        .TILE_COORD_X             (TILE_COORD_X),
        .TILE_COORD_Y             (TILE_COORD_Y),
        .LOCAL_Z                  (LOCAL_Z),
        .NEURONS_PER_TILE         (NEURONS_PER_TILE),
        .WORKER_CORES_PER_TILE    (WORKER_CORES_PER_TILE),
        .FANOUT_POOL_DEPTH        (FANOUT_POOL_DEPTH),
        .TILE_BANK_MEM_STYLE      (TILE_BANK_MEM_STYLE),
        .LOGICAL_EVENT_QUEUE_DEPTH(LOGICAL_EVENT_QUEUE_DEPTH),
        .INGRESS_QUEUE_DEPTH      (INGRESS_QUEUE_DEPTH),
        .FANOUT_DST_X_W           (FANOUT_DST_X_W),
        .FANOUT_DST_Y_W           (FANOUT_DST_Y_W),
        .FANOUT_CORE_ID_W         (FANOUT_CORE_ID_W),
        .HOST_OUTPUT_FIFO_DEPTH   (HOST_OUTPUT_FIFO_DEPTH),
        .FANOUT_QUEUE_DEPTH       (FANOUT_QUEUE_DEPTH),
        .MESSAGE_W                (MESSAGE_W)
    ) u_tile_top (
        .clk    (clk),
        .rst_n  (rst_n),
        .ena    (ena),
        .rv_in  (rv_in_if),
        .rv_out (rv_out_if)
    );

endmodule
