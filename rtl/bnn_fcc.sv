// AXI4-Stream configurable binary fully connected classifier.
module bnn_fcc #(
    parameter int INPUT_DATA_WIDTH  = 8,
    parameter int INPUT_BUS_WIDTH   = 64,
    parameter int CONFIG_BUS_WIDTH  = 64,
    parameter int OUTPUT_DATA_WIDTH = 4,
    parameter int OUTPUT_BUS_WIDTH  = 8,
    parameter int TOTAL_LAYERS = 4,
    parameter int TOPOLOGY[0:TOTAL_LAYERS-1] =
        '{0: 784, 1: 256, 2: 256, 3: 10, default: 0},
    parameter bit PARALLELIZE_LAYERS = 1'b0,
    parameter int PARALLEL_NEURONS   = 1,
    parameter int PARALLEL_INPUTS    = 8
) (
    input logic clk,
    input logic rst,
    input  logic                          config_valid,
    output logic                          config_ready,
    input  logic [  CONFIG_BUS_WIDTH-1:0] config_data,
    input  logic [CONFIG_BUS_WIDTH/8-1:0] config_keep,
    input  logic                          config_last,
    input  logic                         data_in_valid,
    output logic                         data_in_ready,
    input  logic [  INPUT_BUS_WIDTH-1:0] data_in_data,
    input  logic [INPUT_BUS_WIDTH/8-1:0] data_in_keep,
    input  logic                         data_in_last,
    output logic                          data_out_valid,
    input  logic                          data_out_ready,
    output logic [  OUTPUT_BUS_WIDTH-1:0] data_out_data,
    output logic [OUTPUT_BUS_WIDTH/8-1:0] data_out_keep,
    output logic                          data_out_last
);

    localparam int COMPUTE_LAYERS = TOTAL_LAYERS - 1;
    localparam int CONFIG_BYTES = CONFIG_BUS_WIDTH / 8;
    localparam int INPUT_BYTES = INPUT_BUS_WIDTH / 8;

    function automatic int get_max_fan_in();
        int maximum;
        maximum = TOPOLOGY[0];
        for (int i = 1; i < COMPUTE_LAYERS; i++)
            if (TOPOLOGY[i] > maximum) maximum = TOPOLOGY[i];
        return maximum;
    endfunction

    function automatic int get_max_neurons();
        int maximum;
        maximum = TOPOLOGY[0];
        for (int i = 1; i < TOTAL_LAYERS; i++)
            if (TOPOLOGY[i] > maximum) maximum = TOPOLOGY[i];
        return maximum;
    endfunction

    localparam int MAX_FAN_IN = get_max_fan_in();
    localparam int MAX_NEURONS = get_max_neurons();
    localparam int THRESHOLD_LAYERS =
        (COMPUTE_LAYERS > 1) ? COMPUTE_LAYERS - 1 : 1;

    // [weighted layer][neuron][input bit]; weights are stored payload-LSB first.
    logic [MAX_FAN_IN-1:0] weight_mem
        [0:COMPUTE_LAYERS-1][0:MAX_NEURONS-1];
    logic [31:0] threshold_mem
        [0:THRESHOLD_LAYERS-1][0:MAX_NEURONS-1];

    typedef enum logic {CFG_HEADER, CFG_PAYLOAD} cfg_state_t;
    cfg_state_t cfg_state;
    logic [7:0] header [0:15];
    integer header_count, payload_count;
    logic [7:0] message_type, message_layer;
    logic [15:0] message_bytes_per_neuron;
    logic [31:0] message_total_bytes;
    logic configured;

    assign config_ready = !configured;

    // Parse 16-byte headers and neuron-major payloads. Local next-state values
    // permit a header/payload boundary in the middle of an AXI beat.
    always_ff @(posedge clk) begin : configuration
        cfg_state_t state_n;
        integer header_n, payload_n, neuron_n, byte_n, bit_n;
        logic [7:0] type_n, layer_n, incoming;
        logic [15:0] bytes_per_neuron_n;
        logic [31:0] total_bytes_n;
        logic [7:0] header_n_bytes [0:15];

        if (rst) begin
            cfg_state <= CFG_HEADER;
            header_count <= 0;
            payload_count <= 0;
            message_type <= '0;
            message_layer <= '0;
            message_bytes_per_neuron <= '0;
            message_total_bytes <= '0;
            configured <= 1'b0;
            for (int i = 0; i < 16; i++) header[i] <= '0;
        end else if (config_valid && config_ready) begin
            state_n = cfg_state;
            header_n = header_count;
            payload_n = payload_count;
            type_n = message_type;
            layer_n = message_layer;
            bytes_per_neuron_n = message_bytes_per_neuron;
            total_bytes_n = message_total_bytes;
            for (int i = 0; i < 16; i++) header_n_bytes[i] = header[i];

            for (int lane = 0; lane < CONFIG_BYTES; lane++) begin
                if (config_keep[lane]) begin
                    incoming = config_data[lane*8 +: 8];
                    if (state_n == CFG_HEADER) begin
                        header_n_bytes[header_n] = incoming;
                        if (header_n == 15) begin
                            type_n = header_n_bytes[0];
                            layer_n = header_n_bytes[1];
                            bytes_per_neuron_n =
                                {header_n_bytes[7], header_n_bytes[6]};
                            total_bytes_n = {header_n_bytes[11], header_n_bytes[10],
                                             header_n_bytes[9], header_n_bytes[8]};
                            header_n = 0;
                            payload_n = 0;
                            state_n = (total_bytes_n == 0) ? CFG_HEADER : CFG_PAYLOAD;
                        end else begin
                            header_n = header_n + 1;
                        end
                    end else begin
                        if (bytes_per_neuron_n != 0) begin
                            neuron_n = payload_n / bytes_per_neuron_n;
                            byte_n = payload_n % bytes_per_neuron_n;
                        end else begin
                            neuron_n = 0;
                            byte_n = 0;
                        end
                        if ((layer_n < COMPUTE_LAYERS) && (neuron_n < MAX_NEURONS)) begin
                            if (type_n == 0) begin
                                for (int bit_index = 0; bit_index < 8; bit_index++) begin
                                    bit_n = byte_n * 8 + bit_index;
                                    if (bit_n < MAX_FAN_IN)
                                        weight_mem[layer_n][neuron_n][bit_n] <=
                                            incoming[bit_index];
                                end
                            end else if ((type_n == 1) &&
                                         (layer_n < COMPUTE_LAYERS - 1) &&
                                         (byte_n < 4)) begin
                                threshold_mem[layer_n][neuron_n][byte_n*8 +: 8] <=
                                    incoming;
                            end
                        end
                        if (payload_n + 1 >= total_bytes_n) begin
                            state_n = CFG_HEADER;
                            header_n = 0;
                            payload_n = 0;
                        end else begin
                            payload_n = payload_n + 1;
                        end
                    end
                end
            end
            for (int i = 0; i < 16; i++) header[i] <= header_n_bytes[i];
            cfg_state <= state_n;
            header_count <= header_n;
            payload_count <= payload_n;
            message_type <= type_n;
            message_layer <= layer_n;
            message_bytes_per_neuron <= bytes_per_neuron_n;
            message_total_bytes <= total_bytes_n;
            // The provided driver asserts TLAST only on the whole model stream.
            if (config_last) configured <= 1'b1;
        end
    end

    typedef enum logic [1:0] {RX_IMAGE, EVAL_HIDDEN, EVAL_OUTPUT,
                              SEND_OUTPUT} inf_state_t;
    inf_state_t inf_state;
    logic [TOPOLOGY[0]-1:0] input_bits;
    logic [MAX_NEURONS-1:0] activation_bits;
    integer image_count, current_layer;

    assign data_in_ready = configured && (inf_state == RX_IMAGE);

    always_ff @(posedge clk) begin : inference
        integer valid_bytes, matches_b, best_count, best_neuron;
        logic selected_input;

        if (rst) begin
            inf_state <= RX_IMAGE;
            input_bits <= '0;
            activation_bits <= '0;
            image_count <= 0;
            current_layer <= 0;
            data_out_valid <= 1'b0;
            data_out_data <= '0;
            data_out_keep <= '0;
            data_out_last <= 1'b0;
        end else begin
            case (inf_state)
                RX_IMAGE: begin
                    data_out_valid <= 1'b0;
                    data_out_last <= 1'b0;
                    if (data_in_valid && data_in_ready) begin
                        valid_bytes = 0;
                        for (int lane = 0; lane < INPUT_BYTES; lane++) begin
                            if (data_in_keep[lane]) begin
                                if (image_count + valid_bytes < TOPOLOGY[0])
                                    input_bits[image_count + valid_bytes] <=
                                        (data_in_data[lane*8 +: 8] >= 8'd128);
                                valid_bytes = valid_bytes + 1;
                            end
                        end
                        if (data_in_last || image_count + valid_bytes >= TOPOLOGY[0]) begin
                            image_count <= 0;
                            current_layer <= 0;
                            inf_state <= (COMPUTE_LAYERS == 1) ?
                                         EVAL_OUTPUT : EVAL_HIDDEN;
                        end else begin
                            image_count <= image_count + valid_bytes;
                        end
                    end
                end

                // Activation bits are registered between hidden layers.
                EVAL_HIDDEN: begin
                    for (int n = 0; n < MAX_NEURONS; n++) begin
                        if (n < TOPOLOGY[current_layer+1]) begin
                            matches_b = 0;
                            for (int i = 0; i < MAX_FAN_IN; i++) begin
                                if (i < TOPOLOGY[current_layer]) begin
                                    selected_input = (current_layer == 0) ?
                                        input_bits[i] : activation_bits[i];
                                    if (selected_input == weight_mem[current_layer][n][i])
                                        matches_b = matches_b + 1;
                                end
                            end
                            activation_bits[n] <=
                                (matches_b >= threshold_mem[current_layer][n]);
                        end else begin
                            activation_bits[n] <= 1'b0;
                        end
                    end
                    if (current_layer == COMPUTE_LAYERS - 2) begin
                        current_layer <= COMPUTE_LAYERS - 1;
                        inf_state <= EVAL_OUTPUT;
                    end else begin
                        current_layer <= current_layer + 1;
                    end
                end

                // Final layer uses raw XNOR-popcounts, with a low-index tie win.
                EVAL_OUTPUT: begin
                    best_count = -1;
                    best_neuron = 0;
                    for (int n = 0; n < MAX_NEURONS; n++) begin
                        if (n < TOPOLOGY[COMPUTE_LAYERS]) begin
                            matches_b = 0;
                            for (int i = 0; i < MAX_FAN_IN; i++) begin
                                if (i < TOPOLOGY[COMPUTE_LAYERS-1]) begin
                                    selected_input = (COMPUTE_LAYERS == 1) ?
                                        input_bits[i] : activation_bits[i];
                                    if (selected_input ==
                                        weight_mem[COMPUTE_LAYERS-1][n][i])
                                        matches_b = matches_b + 1;
                                end
                            end
                            if (matches_b > best_count) begin
                                best_count = matches_b;
                                best_neuron = n;
                            end
                        end
                    end
                    data_out_data <= OUTPUT_BUS_WIDTH'(best_neuron);
                    data_out_keep <= 'd1;
                    data_out_last <= 1'b1;
                    data_out_valid <= 1'b1;
                    inf_state <= SEND_OUTPUT;
                end

                SEND_OUTPUT: begin
                    // Output data, keep, and last remain stable while stalled.
                    if (data_out_valid && data_out_ready) begin
                        data_out_valid <= 1'b0;
                        data_out_last <= 1'b0;
                        inf_state <= RX_IMAGE;
                    end
                end
                default: inf_state <= RX_IMAGE;
            endcase
        end
    end

    wire unused_parallel_options = PARALLELIZE_LAYERS ^
                                   (PARALLEL_NEURONS == 0) ^
                                   (PARALLEL_INPUTS == 0) ^
                                   (INPUT_DATA_WIDTH != 8) ^
                                   (OUTPUT_DATA_WIDTH == 0);
endmodule
