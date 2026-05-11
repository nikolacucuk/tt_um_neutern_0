//------------------------------------------------------------------------------------------------//
// Generalized Ready-Valid Interface
//------------------------------------------------------------------------------------------------//
//
// Width-parameterised.  Earlier revisions used `parameter type RV_PAYLOAD_T`
// to carry an arbitrary packed struct through the interface; yosys (the
// formal / open-source synth frontend) does not support parameterised
// types, so this interface carries a plain `logic [WIDTH-1:0] rv_payload`.
// Consumers cast to/from their struct type at use sites, typically:
//
//     message_packet_t pkt = message_packet_t'(foo.rv_payload);
//
// The extra cast is cheap and keeps yosys, Verilator, and commercial
// simulators all on the same interface signature.
//------------------------------------------------------------------------------------------------//
interface rv_if #(
    parameter int WIDTH = 1
) (
    // NONE - Clockless
);

    // Receiver is ready to receive rv_payload.
    /* verilator lint_off UNOPTFLAT */
    logic ready;

    // Transmitter is sending valid rv_payload.
    logic valid;
    /* verilator lint_on UNOPTFLAT */

    // Payload, transferred when and only when ready==valid==1.
    logic [WIDTH-1:0] rv_payload;

    // Transmit modport (primary driver).
    modport tx      ( input  ready, output valid, rv_payload );

    // Receive modport (primary receiver).
    modport rx      ( output ready, input  valid, rv_payload );

    // Monitor modport (snoop without driving).
    modport monitor ( input  ready, input  valid, rv_payload );

endinterface
