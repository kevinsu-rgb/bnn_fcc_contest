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

    logic [OUTPUT_DATA_WIDTH-1:0] binarized_data;

    // For an unsigned pixel, testing its MSB is equivalent to comparison
    // against the midpoint of its representable range (128 for 8-bit MNIST
    // pixels).  This form keeps the threshold well-defined for other legal
    // PIXEL_DATA_WIDTH values too.
    localparam logic [PIXEL_DATA_WIDTH-1:0] BINARIZE_THRESHOLD =
        {1'b1, {(PIXEL_DATA_WIDTH - 1){1'b0}}};

    always_comb begin
        binarized_data = '0;
        for (int i = 0; (i < PIXEL_NUM) && (i < OUTPUT_DATA_WIDTH); i++) begin
            binarized_data[i] = (IN_DATA[i] >= BINARIZE_THRESHOLD);
        end
    end

    always_ff @(posedge clk) begin
        if (rst) begin
            OUT_DATA <= '0;
        end else if (EN) begin
            OUT_DATA <= binarized_data;
        end
    end

endmodule
