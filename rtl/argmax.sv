module argmax #(
    parameter int NUM_NEURONS    = 10,
    parameter int POPCOUNT_WIDTH = 9,
    parameter int OUT_WIDTH      = 4
) (
    input  logic [POPCOUNT_WIDTH-1:0] popcounts [NUM_NEURONS-1:0],
    output logic [     OUT_WIDTH-1:0] max_index
);

    logic [POPCOUNT_WIDTH-1:0] max_value;

    always_comb begin
        max_value = '0;
        max_index = '0;
        for (int n = 0; n < NUM_NEURONS; n++) begin
            // Deliberately use a strict comparison: equal counts retain the
            // lowest-indexed neuron, matching the reference model.
            if (popcounts[n] > max_value) begin
                max_value = popcounts[n];
                max_index = OUT_WIDTH'(n);
            end
        end
    end

endmodule
