module neuron_proc_popcount #(
    parameter int INPUT_DATA_WIDTH = 784,
    parameter int POPCOUNT_WIDTH   = $clog2(INPUT_DATA_WIDTH + 1)
) (
    input  logic [ INPUT_DATA_WIDTH-1:0] data_in,
    input  logic [ INPUT_DATA_WIDTH-1:0] weight,
    output logic [  POPCOUNT_WIDTH-1:0]  popcount_out
);

    logic [INPUT_DATA_WIDTH-1:0] xnor_result;
    assign xnor_result = ~(data_in ^ weight);

    always_comb begin
        popcount_out = '0;
        for (int i = 0; i < INPUT_DATA_WIDTH; i++) begin
            popcount_out = popcount_out + xnor_result[i];
        end

        // Both operands are unsigned packed vectors.  SystemVerilog extends
        // the narrower one before comparing, avoiding a fixed 32-bit cast.
    end

endmodule
