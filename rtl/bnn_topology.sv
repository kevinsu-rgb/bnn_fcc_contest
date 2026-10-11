module bnn_topology #(
    parameter int TOTAL_LAYERS     = 4,
    parameter int TOPOLOGY[0:TOTAL_LAYERS-1] = {0: 784, 1: 256, 2: 256, 3: 10, default: 0},
    parameter int PIXEL_DATA_WIDTH = 8,
    parameter int OUT_WIDTH        = 4,
    parameter int THRESHOLD_WIDTH  = 32,
    parameter int PARALLEL_NEURONS[TOTAL_LAYERS-1] = '{default: 1}
) (
    input  logic                        clk,
    input  logic                        rst,
    input  logic                        en,
    input  logic                        data_valid_in,
    input  logic [PIXEL_DATA_WIDTH-1:0] data_in [TOPOLOGY[0]-1:0],
    input  logic [THRESHOLD_WIDTH-1:0]  layer1_thresholds [TOPOLOGY[1]-1:0],
    input  logic [TOPOLOGY[1]-1:0]      layer1_weights [TOPOLOGY[1]-1:0],
    input  logic [THRESHOLD_WIDTH-1:0]  layer2_thresholds [TOPOLOGY[2]-1:0],
    input  logic [TOPOLOGY[2]-1:0]      layer2_weights [TOPOLOGY[2]-1:0],
    input  logic [TOPOLOGY[3]-1:0]      layer3_weights [TOPOLOGY[3]-1:0],
    output  logic                       data_valid_out,
    output logic [     OUT_WIDTH-1:0]   data_out
);

//    initial begin
//        for(int i = 0; i < $size(TOPOLOGY); i++) begin
//            if (PARALLEL_NEURONS[i] > TOPOLOGY[i]) $fatal(1, $sformatf("Error at Layer %0d: PARALLEL_NEURONS (%0d) exceeds TOPOLOGY limit (%0d).", i, PARALLEL_NEURONS[i], TOPOLOGY[i]));
//        end
//    end

    logic [TOPOLOGY[0]-1:0] binarize_out;
    logic                   binarize_data_valid_out;
    logic [TOPOLOGY[1]-1:0] layer1_out;
    logic [$clog2(TOPOLOGY[1]/PARALLEL_NEURONS[1])-1:0] layer1_cycles;
    logic [TOPOLOGY[2]-1:0] layer2_out;
    logic [$clog2(TOPOLOGY[2]+1)-1:0] layer3_popcount [TOPOLOGY[3]-1:0];
    logic [TOPOLOGY[3]-1:0] layer3_out;
    logic layer1_out_valid;

    binarize #(
        .PIXEL_NUM(TOPOLOGY[0]),
        .PIXEL_DATA_WIDTH(PIXEL_DATA_WIDTH),
        .OUTPUT_DATA_WIDTH(TOPOLOGY[0])
    ) binarize_0 (
        .clk(clk),
        .rst(rst),
        .en(en),
        .data_in(data_in),
        .data_out(binarize_out)
    );

    delay #(
        .CYCLES(1),
        .WIDTH(1)
    ) binarize_delay_0 (
        .clk(clk),
        .rst(rst),
        .en(en),
        .data_in(data_valid_in),
        .data_out(binarize_data_valid_out)
    );

    genvar i;

    for (i = 0; i < PARALLEL_NEURONS[1]; i++) begin
        neuron_proc #(
            .INPUT_DATA_WIDTH(TOPOLOGY[0]),
            .THRESHOLD_WIDTH(THRESHOLD_WIDTH)
        ) neuron_proc_0 (
            .clk(clk),
            .rst(rst),
            .en(en),
            .data_in(binarize_out),
            .weight(layer1_weights[i]),
            .threshold(layer1_thresholds[i]),
            .data_out(layer1_out[i])
        );  
    end

    for (i = 0; i < PARALLEL_NEURONS[2]; i++) begin
        neuron_proc #(
            .INPUT_DATA_WIDTH(TOPOLOGY[1]),
            .THRESHOLD_WIDTH(THRESHOLD_WIDTH)
        ) neuron_proc_1 (
            .clk(clk),
            .rst(rst),
            .en(en),
            .data_in(layer1_out),
            .weight(layer2_weights[i]),
            .threshold(layer2_thresholds[i]),
            .data_out(layer2_out[i])
        );  
    end

    for (i = 0; i < PARALLEL_NEURONS[3]; i++) begin
        neuron_proc_popcount #(
            .INPUT_DATA_WIDTH(TOPOLOGY[2])
        ) neuron_proc_2 (
            .data_in(layer2_out),
            .weight(layer3_weights[i]),
            .popcount_out(layer3_popcount[i])
        );  
    end

    argmax #(
        .NUM_NEURONS(TOPOLOGY[3]),
        .POPCOUNT_WIDTH($clog2(TOPOLOGY[2]+1)),
        .OUT_WIDTH(OUT_WIDTH)
    ) argmax_0 (
        .clk(clk),
        .popcounts(layer3_popcount),
        .max_index(data_out)
    );

endmodule