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
            out_count = out_count + OUT_WIDTH'(in_data[i]);
        end
    end

endmodule
