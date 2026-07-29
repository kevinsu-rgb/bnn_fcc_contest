module argmax #(
    parameter int NUM_NEURONS    = 10,
    parameter int POPCOUNT_WIDTH = 9,
    parameter int OUT_WIDTH      = 4
) (
    input  logic [POPCOUNT_WIDTH-1:0] popcounts [NUM_NEURONS-1:0],
    output logic [     OUT_WIDTH-1:0] max_index
);

    always_comb begin
        int max_val;
        max_val = -1;
        max_index = '0;
        for (int n = 0; n < NUM_NEURONS; n++) begin
            if (int'(popcounts[n]) > max_val) begin
                max_val = int'(popcounts[n]);
                max_index = OUT_WIDTH'(n);
            end
        end
    end

endmodule
