`default_nettype none

module tile_event_queue_bank_handshake_formal #(
    parameter int unsigned WORKER_CORES_PER_TILE = 2,
    parameter int unsigned FIFO_DEPTH_PER_WORKER = 4,
    parameter int unsigned EVENT_W = 24
) (
    input logic clk,
    input logic rst_n
);
    logic ena;
    logic soft_reset_valid;

    logic [WORKER_CORES_PER_TILE-1:0] enqueue_valid;
    logic [EVENT_W*WORKER_CORES_PER_TILE-1:0] enqueue_data;
    logic [WORKER_CORES_PER_TILE-1:0] enqueue_ready;

    logic [WORKER_CORES_PER_TILE-1:0] dequeue_ready;
    logic [WORKER_CORES_PER_TILE-1:0] dequeue_valid;
    logic [EVENT_W*WORKER_CORES_PER_TILE-1:0] dequeue_data;

    logic [WORKER_CORES_PER_TILE-1:0] not_full;
    logic mem_stall;

    tile_event_queue_bank #(
        .WORKER_CORES_PER_TILE (WORKER_CORES_PER_TILE),
        .FIFO_DEPTH_PER_WORKER (FIFO_DEPTH_PER_WORKER),
        .EVENT_W               (EVENT_W)
    ) dut (
        .clk              (clk),
        .rst_n            (rst_n),
        .ena              (ena),
        .soft_reset_valid (soft_reset_valid),
        .enqueue_valid    (enqueue_valid),
        .enqueue_data     (enqueue_data),
        .enqueue_ready    (enqueue_ready),
        .dequeue_ready    (dequeue_ready),
        .dequeue_valid    (dequeue_valid),
        .dequeue_data     (dequeue_data),
        .not_full         (not_full),
        .mem_stall        (mem_stall)
    );

`ifdef FORMAL
    logic f_past_valid = 1'b0;
    always_ff @(posedge clk) f_past_valid <= 1'b1;

    logic f_first_cycle = 1'b1;
    always_ff @(posedge clk) f_first_cycle <= 1'b0;

    always_ff @(posedge clk) begin
        if (f_first_cycle) assume(!rst_n);
        if (!rst_n) begin
            assume(!ena);
            assume(!soft_reset_valid);
        end else begin
            assume(ena);
            assume(!soft_reset_valid);
        end
    end

`ifdef ENQUEUE_LIVENESS
    always_ff @(posedge clk) begin
        if (rst_n && ena && !soft_reset_valid) begin
            cover(|enqueue_valid);
            cover(|enqueue_ready);
        end
    end
`endif

`ifdef DEQUEUE_AFTER_PUSH
    genvar gi0;
    generate
        for (gi0 = 0; gi0 < WORKER_CORES_PER_TILE; gi0 = gi0 + 1) begin : g_deq_hold
            always_ff @(posedge clk) begin
                if (f_past_valid && $past(rst_n) && $past(ena) &&
                    !$past(soft_reset_valid) && $past(dequeue_valid[gi0]) &&
                    !$past(dequeue_ready[gi0])) begin
                    assert(dequeue_valid[gi0]);
                    assert(dequeue_data[gi0*EVENT_W +: EVENT_W] ==
                           $past(dequeue_data[gi0*EVENT_W +: EVENT_W]));
                end
            end
        end
    endgenerate
`endif

`ifdef COUNT_BOUND
    always_ff @(posedge clk) begin
        if (rst_n && ena) begin
            assert((dequeue_valid & {WORKER_CORES_PER_TILE{soft_reset_valid}}) == '0);
        end
    end
`endif
`endif
endmodule

`default_nettype wire
