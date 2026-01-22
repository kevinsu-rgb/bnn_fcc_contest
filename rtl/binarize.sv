
// Currently combinational
module binarize #(
    parameter int PIXEL_NUM         = 784,
    parameter int PIXEL_DATA_WIDTH  = 8,
    parameter int OUTPUT_DATA_WIDTH = 784
) (
    input  logic                 clk,
    input  logic                 rst,
    input  logic [PIXEL_NUM-1:0] IN_DATA [PIXEL_DATA_WIDTH-1:0],
    input  logic                 EN,
    output logic                 OUT_DATA [OUTPUT_DATA_WIDTH-1:0]
    
);

    always_ff @(posedge(clk) | posedge(rst)) begin
        OUT_DATA = OUT_DATA;
        if(EN) begin
        
        end
    end

endmodule