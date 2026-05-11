`default_nettype none

module coldfoot_rr_arbiter #(
    parameter int unsigned NUM_REQS = 1
) (
    input  wire                             clk,
    input  wire                             rst_n,
    input  wire                             ena,
    input  wire [NUM_REQS-1:0]              req,
    input  wire                             grant_accept,
    output logic                            grant_valid,
    output logic [((NUM_REQS <= 1) ? 1 : $clog2(NUM_REQS))-1:0] grant_idx,
    output logic [NUM_REQS-1:0]             grant_onehot
);
    localparam int unsigned IDX_W = (NUM_REQS <= 1) ? 1 : $clog2(NUM_REQS);

    logic [IDX_W-1:0] rr_base_r;
    logic [IDX_W-1:0] scan_idx;

    function automatic [IDX_W-1:0] wrapped_index(
        input logic [IDX_W-1:0] base_idx,
        input integer offset
    );
        int unsigned candidate_idx;
        begin
            candidate_idx = base_idx + offset;
            if (candidate_idx >= NUM_REQS) begin
                candidate_idx = candidate_idx - NUM_REQS;
            end
            wrapped_index = candidate_idx[IDX_W-1:0];
        end
    endfunction
    integer scan_offset;

    always_comb begin
        scan_idx = '0;
        grant_valid = 1'b0;
        grant_idx = '0;
        grant_onehot = '0;

        if (rst_n && ena) begin
            for (scan_offset = 0; scan_offset < NUM_REQS; scan_offset = scan_offset + 1) begin
                scan_idx = wrapped_index(rr_base_r, scan_offset);
                if (!grant_valid && req[scan_idx]) begin
                    grant_valid = 1'b1;
                    grant_idx = scan_idx;
                    grant_onehot[scan_idx] = 1'b1;
                end
            end
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rr_base_r <= '0;
        end else if (!ena) begin
            rr_base_r <= '0;
        end else if (grant_valid && grant_accept) begin
            if (grant_idx == (NUM_REQS - 1)) begin
                rr_base_r <= '0;
            end else begin
                rr_base_r <= grant_idx + 1'b1;
            end
        end
    end
endmodule

`default_nettype wire
