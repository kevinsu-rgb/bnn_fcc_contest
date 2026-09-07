module popcount #(
    parameter int INPUT_WIDTH = 256,
    parameter int OUT_WIDTH   = $clog2(INPUT_WIDTH + 1)
) (
    input  logic [INPUT_WIDTH-1:0] in_data,
    output logic [  OUT_WIDTH-1:0] out_count
);

    always_comb begin
        out_count = '0;
        for (int i = 0; i < INPUT_WIDTH; i++) begin
            // The left operand sets the result width, so a correctly sized
            // OUT_WIDTH can represent every value from 0 through INPUT_WIDTH.
            out_count = out_count + in_data[i];
        end
    end

endmodule
