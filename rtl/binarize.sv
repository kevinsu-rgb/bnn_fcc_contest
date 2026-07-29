module binarize #(
    parameter int PIXEL_NUM         = 784,
    parameter int PIXEL_DATA_WIDTH  = 8,
    parameter int OUTPUT_DATA_WIDTH = 784
) (
    input  logic                          clk,
    input  logic                          rst,
    input  logic                          EN,
    input  logic [PIXEL_DATA_WIDTH-1:0]   IN_DATA [PIXEL_NUM-1:0],
    output logic [OUTPUT_DATA_WIDTH-1:0]  OUT_DATA
);

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            OUT_DATA <= '0;
        end else if (EN) begin
            for (int i = 0; i < PIXEL_NUM; i++) begin
                OUT_DATA[i] <= (IN_DATA[i] >= 8'h80) ? 1'b1 : 1'b0;
            end
        end
    end

endmodule
