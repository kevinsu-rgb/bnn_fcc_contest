module neuron_proc #(
    parameter int INPUT_DATA_WIDTH = 784,
    parameter int THRESHOLD_WIDTH  = 32,
    parameter int POPCOUNT_WIDTH   = $clog2(INPUT_DATA_WIDTH + 1)
) (
    input  logic [ INPUT_DATA_WIDTH-1:0] in_data,
    input  logic [ INPUT_DATA_WIDTH-1:0] weight,
    input  logic [  THRESHOLD_WIDTH-1:0] threshold,
    output logic [  POPCOUNT_WIDTH-1:0]  popcount_out,
    output logic                         out_data
);

    logic [INPUT_DATA_WIDTH-1:0] xnor_result;
    assign xnor_result = ~(in_data ^ weight);

    always_comb begin
        popcount_out = '0;
        for (int i = 0; i < INPUT_DATA_WIDTH; i++) begin
            popcount_out = popcount_out + xnor_result[i];
        end

        // Both operands are unsigned packed vectors.  SystemVerilog extends
        // the narrower one before comparing, avoiding a fixed 32-bit cast.
        out_data = (popcount_out >= threshold);
    end

endmodule
