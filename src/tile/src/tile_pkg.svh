// tile_pkg.svh
// ---------------------------------------------------------------------------
// Shared helper functions duplicated across the four tile_* decomposition
// modules (tile_ingress, tile_host_io, tile_fanout_executor,
// tile_dispatch_scheduler).  Each helper references the module's local
// params (NEURONS_PER_TILE, LOCAL_Z, NEURON_IDX_W), so this file MUST be
// `include`d inside each module's body (after the parameter list and any
// derived localparams) — NOT at file/compilation-unit scope.  Vivado
// correctly rejects file-scope placement because the function bodies
// can't see the unresolved module parameters.
//
// No include-guard: every module that uses these helpers needs its own
// copy at its own module scope (SystemVerilog allows function
// re-definition across distinct module scopes; a guard would silently
// leave later modules without the helpers).
//
// Bodies are byte-for-byte identical to the pre-consolidation copies;
// this file is a pure helper-promotion pass with no behavioural changes.
// ---------------------------------------------------------------------------

function automatic logic [NEURONS_PER_TILE-1:0] onehot_for_idx(
    input logic oh_valid,
    input logic [NEURON_IDX_W-1:0] oh_idx
);
    logic [NEURONS_PER_TILE-1:0] onehot_mask;
    begin
        onehot_mask = '0;
        if (oh_valid) begin
            onehot_mask[oh_idx] = 1'b1;
        end
        onehot_for_idx = onehot_mask;
    end
endfunction

function automatic logic [7:0] local_core_id_for_index(input integer neuron_idx);
    int unsigned core_id_sum;
    begin
        core_id_sum = LOCAL_Z + neuron_idx;
        local_core_id_for_index = core_id_sum[7:0];
    end
endfunction

function automatic logic local_target_valid_for_core_id(
    input logic [CORE_ID_W-1:0] route_core_id,
    output logic [NEURON_IDX_W-1:0] local_idx
);
    logic [CORE_ID_W-1:0] local_z_low;
    logic [10:0] local_idx_wide;  // wide enough for up to 1024-neuron NEURON_IDX_W=10
    begin
        local_z_low = CORE_ID_W'(LOCAL_Z);
        local_idx_wide = {(11-CORE_ID_W)'(0), route_core_id} - {(11-CORE_ID_W)'(0), local_z_low};
        local_idx = local_idx_wide[NEURON_IDX_W-1:0];
        local_target_valid_for_core_id =
            (route_core_id >= local_z_low) &&
            (local_idx_wide < 11'(NEURONS_PER_TILE));
    end
endfunction

