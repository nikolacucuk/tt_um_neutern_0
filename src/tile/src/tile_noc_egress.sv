`default_nettype none

module tile_noc_egress #(
    parameter int unsigned PAYLOAD_W = 74,
    parameter int unsigned LOCAL_FIFO_DEPTH = 4
) (
    input  wire                 clk,
    input  wire                 rst_n,
    input  wire                 ena,
    input  wire                 graph_state_clear,
    rv_if.rx                    child,
    rv_if.tx                    rv_out,
    output logic                tile_valid_dbg
`ifdef FORMAL
    ,
    output logic [$clog2(LOCAL_FIFO_DEPTH + 1)-1:0] tile_count_obs
`endif
) ;
    localparam int unsigned FIFO_PTR_W =
        (LOCAL_FIFO_DEPTH <= 1) ? 1 : $clog2(LOCAL_FIFO_DEPTH);
    localparam int unsigned FIFO_COUNT_W = $clog2(LOCAL_FIFO_DEPTH + 1);

    logic [PAYLOAD_W-1:0] tile_payload_fifo_r [0:LOCAL_FIFO_DEPTH-1];
    logic [FIFO_PTR_W-1:0] tile_rd_ptr_r;
    logic [FIFO_PTR_W-1:0] tile_wr_ptr_r;
    logic [FIFO_COUNT_W-1:0] tile_count_r;
    logic push_c;
    logic pop_c;

    wire child_valid = child.valid;
    wire [PAYLOAD_W-1:0] child_payload = child.rv_payload;
    logic child_ready;
    wire rv_out_ready = rv_out.ready;
    logic rv_out_valid;
    logic [PAYLOAD_W-1:0] rv_out_payload;
    assign child.ready = child_ready;
    assign rv_out.valid = rv_out_valid;
    assign rv_out.rv_payload = rv_out_payload;

    function automatic [FIFO_PTR_W-1:0] next_fifo_ptr(
        input logic [FIFO_PTR_W-1:0] ptr
    );
        begin
            if (LOCAL_FIFO_DEPTH <= 1) begin
                next_fifo_ptr = '0;
            end else if (ptr == FIFO_PTR_W'(LOCAL_FIFO_DEPTH - 1)) begin
                next_fifo_ptr = '0;
            end else begin
                next_fifo_ptr = ptr + FIFO_PTR_W'(1);
            end
        end
    endfunction

    // Hold the FIFO quiescent while a graph clear is active so no stale
    // entry can be advertised or accepted during the flush window.
    assign tile_valid_dbg = ena &&
                            !graph_state_clear &&
                            (tile_count_r != FIFO_COUNT_W'(0));
    assign child_ready = ena &&
                         !graph_state_clear &&
                         ((tile_count_r < FIFO_COUNT_W'(LOCAL_FIFO_DEPTH)) || pop_c);
    assign rv_out_valid = ena &&
                          !graph_state_clear &&
                          (tile_count_r != FIFO_COUNT_W'(0));
    assign rv_out_payload = tile_payload_fifo_r[tile_rd_ptr_r];
`ifdef FORMAL
    assign tile_count_obs = tile_count_r;
`endif
    assign push_c = child_valid && child_ready;
    assign pop_c = rv_out_valid && rv_out_ready;

    integer fifo_idx;
    always_ff @(posedge clk) begin
        if (!rst_n) begin
            tile_rd_ptr_r <= '0;
            tile_wr_ptr_r <= '0;
            tile_count_r <= '0;
            for (fifo_idx = 0; fifo_idx < LOCAL_FIFO_DEPTH; fifo_idx = fifo_idx + 1) begin
                tile_payload_fifo_r[fifo_idx] <= '0;
            end
        end else if (graph_state_clear) begin
            tile_rd_ptr_r <= '0;
            tile_wr_ptr_r <= '0;
            tile_count_r <= '0;
            for (fifo_idx = 0; fifo_idx < LOCAL_FIFO_DEPTH; fifo_idx = fifo_idx + 1) begin
                tile_payload_fifo_r[fifo_idx] <= '0;
            end
        end else if (ena) begin
            if (push_c) begin
                tile_payload_fifo_r[tile_wr_ptr_r] <= child_payload;
                tile_wr_ptr_r <= next_fifo_ptr(tile_wr_ptr_r);
            end

            if (pop_c) begin
                tile_rd_ptr_r <= next_fifo_ptr(tile_rd_ptr_r);
            end

            case ({push_c, pop_c})
                2'b10: tile_count_r <= tile_count_r + FIFO_COUNT_W'(1);
                2'b01: tile_count_r <= tile_count_r - FIFO_COUNT_W'(1);
                default: begin
                end
            endcase
        end
    end
endmodule

`default_nettype wire
