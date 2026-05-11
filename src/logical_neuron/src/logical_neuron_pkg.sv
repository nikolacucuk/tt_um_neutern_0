package logical_neuron_pkg;
    typedef struct packed {
        logic [23:0] rf_state_flat;
        logic [4:0] last_sid;
        logic [1:0] last_tag;
        // last_time removed: timestamps eliminated
        logic cmp_ge;
        logic cmp_eq;
        logic spike_flag;
    } logical_neuron_context_t;
endpackage
