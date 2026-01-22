module neuron_proc #(
    parameter int INPUT_DATA_WIDTH  = 128,
    parameter int PW_WIDTH          = 16,
    parameter int OUTPUT_DATA_WIDTH = 1,
    parameter int THRESHOLD_WIDTH   = $clog2(INPUT_DATA_WIDTH+1)
) (
    input logic clk,
    input logic rst
    
);

endmodule