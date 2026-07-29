module bnn_fcc #(
    parameter int INPUT_DATA_WIDTH  = 16,
    parameter int INPUT_BUS_WIDTH   = 32,
    parameter int CONFIG_BUS_WIDTH  = 32,
    parameter int OUTPUT_DATA_WIDTH = 4,
    parameter int OUTPUT_BUS_WIDTH  = 8,

    parameter int TOTAL_LAYERS = 4,  // Includes input, hidden, and output
    parameter int TOPOLOGY[TOTAL_LAYERS] = '{0: 784, 1: 256, 2: 256, 3: 10, default: 0},  // 0: input, TOTAL_LAYERS-1: output

    parameter bit PARALLELIZE_LAYERS = 1'b0,
    parameter int PARALLEL_NEURONS   = 1,
    parameter int PARALLEL_INPUTS    = 32
) (
    input logic clk,
    input logic rst,

    // AXI streaming configuration interface (consumer)
    input  logic                          config_valid,
    output logic                          config_ready,
    input  logic [  CONFIG_BUS_WIDTH-1:0] config_data,
    input  logic [CONFIG_BUS_WIDTH/8-1:0] config_keep,
    input  logic                          config_last,

    // AXI streaming image input interface (consumer)
    input  logic                         data_in_valid,
    output logic                         data_in_ready,
    input  logic [  INPUT_BUS_WIDTH-1:0] data_in_data,
    input  logic [INPUT_BUS_WIDTH/8-1:0] data_in_keep,
    input  logic                         data_in_last,

    // AXI streaming classification output interface (producer)
    output logic                          data_out_valid,
    input  logic                          data_out_ready,
    output logic [  OUTPUT_BUS_WIDTH-1:0] data_out_data,
    output logic [OUTPUT_BUS_WIDTH/8-1:0] data_out_keep,
    output logic                          data_out_last
);

    // Lint suppression for unused top-level parameters required by DUT contract
    /* verilator lint_off UNUSEDPARAM */
    localparam int UNUSED_IN_WIDTH  = INPUT_DATA_WIDTH;
    localparam int UNUSED_OUT_WIDTH = OUTPUT_DATA_WIDTH;
    localparam bit UNUSED_PAR_LAYERS = PARALLELIZE_LAYERS;
    localparam int UNUSED_PAR_NEURONS = PARALLEL_NEURONS;
    localparam int UNUSED_PAR_INPUTS  = PARALLEL_INPUTS;
    /* verilator lint_on UNUSEDPARAM */

    // =========================================================================
    // Parameter & Dimension Calculations
    // =========================================================================
    localparam int NUM_COMPUTE_LAYERS = TOTAL_LAYERS - 1;

    function automatic int get_max_inputs();
        int m;
        m = TOPOLOGY[0];
        for (int i = 1; i < TOTAL_LAYERS-1; i++) begin
            if (TOPOLOGY[i] > m) m = TOPOLOGY[i];
        end
        return m;
    endfunction

    function automatic int get_max_neurons();
        int m;
        m = TOPOLOGY[1];
        for (int i = 2; i < TOTAL_LAYERS; i++) begin
            if (TOPOLOGY[i] > m) m = TOPOLOGY[i];
        end
        return m;
    endfunction

    localparam int MAX_FAN_IN  = get_max_inputs();
    localparam int MAX_NEURONS = get_max_neurons();
    localparam int CFG_BYTES_PER_BEAT = CONFIG_BUS_WIDTH / 8;
    localparam int IN_BYTES_PER_BEAT  = INPUT_BUS_WIDTH / 8;

    // =========================================================================
    // Memory for Weights & Thresholds
    // =========================================================================
    // Dimensions: [layer][neuron][weight_bit]
    logic [MAX_FAN_IN-1:0] weight_mem [NUM_COMPUTE_LAYERS-1:0][MAX_NEURONS-1:0];

    // Dimensions: [layer][neuron] (32-bit integer per threshold)
    logic [31:0] thresh_mem [NUM_COMPUTE_LAYERS-2:0][MAX_NEURONS-1:0];

    // =========================================================================
    // Configuration Engine FSM & Signals
    // =========================================================================
    typedef enum logic [1:0] {
        CFG_IDLE,
        CFG_HEADER,
        CFG_PAYLOAD,
        CFG_DONE
    } cfg_state_t;

    cfg_state_t cfg_state;

    logic [7:0]  header_buf [15:0];
    int          header_byte_count;

    logic [7:0]  cur_msg_type;
    logic [7:0]  cur_layer_id;
    logic [15:0] cur_layer_inputs;
    logic [15:0] cur_num_neurons;
    logic [15:0] cur_bytes_per_neuron;
    logic [31:0] cur_total_bytes;

    int          payload_byte_count;
    logic        config_done;

    assign config_ready = (cfg_state == CFG_HEADER || cfg_state == CFG_PAYLOAD);

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            cfg_state          <= CFG_HEADER;
            header_byte_count  <= 0;
            payload_byte_count <= 0;
            config_done        <= 1'b0;
            cur_msg_type       <= '0;
            cur_layer_id       <= '0;
            cur_layer_inputs   <= '0;
            cur_num_neurons    <= '0;
            cur_bytes_per_neuron <= '0;
            cur_total_bytes    <= '0;

            weight_mem <= '{default: '{default: '0}};
            thresh_mem <= '{default: '{default: '0}};
        end else begin
            case (cfg_state)
                CFG_HEADER: begin
                    if (config_valid && config_ready) begin
                        int valid_in_beat;
                        valid_in_beat = 0;
                        for (int k = 0; k < CFG_BYTES_PER_BEAT; k++) begin
                            if (config_keep[k]) valid_in_beat++;
                        end

                        for (int k = 0; k < CFG_BYTES_PER_BEAT; k++) begin
                            if (config_keep[k]) begin
                                if (header_byte_count < 16) begin
                                    header_buf[header_byte_count] <= config_data[k*8 +: 8];
                                    header_byte_count <= header_byte_count + 1;
                                end
                            end
                        end

                        if (header_byte_count + valid_in_beat >= 16) begin
                            logic [7:0] h [16];
                            int h_ptr;
                            int payload_start_k;
                            int p_count;
                            int payload_added;
                            int total_b_val;

                            h_ptr = 0;

                            for (int idx = 0; idx < 16; idx++) begin
                                if (idx < header_byte_count) begin
                                    h[idx] = header_buf[idx];
                                end else begin
                                    h[idx] = 8'h0;
                                end
                            end
                            for (int k = 0; k < CFG_BYTES_PER_BEAT; k++) begin
                                if (config_keep[k] && (header_byte_count + h_ptr < 16)) begin
                                    h[header_byte_count + h_ptr] = config_data[k*8 +: 8];
                                    h_ptr++;
                                end
                            end

                            cur_msg_type         <= h[0];
                            cur_layer_id         <= h[1];
                            cur_layer_inputs     <= {h[3], h[2]};
                            cur_num_neurons      <= {h[5], h[4]};
                            cur_bytes_per_neuron <= {h[7], h[6]};
                            cur_total_bytes      <= {h[11], h[10], h[9], h[8]};

                            header_byte_count  <= 0;
                            payload_byte_count <= 0;

                            payload_start_k = 16 - header_byte_count;
                            p_count = 0;
                            total_b_val = int'({h[11], h[10], h[9], h[8]});

                            for (int k = 0; k < CFG_BYTES_PER_BEAT; k++) begin
                                if (config_keep[k]) begin
                                    if (p_count >= payload_start_k) begin
                                        int p_offset;
                                        int bpn;
                                        int n_idx;
                                        int b_idx;
                                        int l_idx;
                                        p_offset = p_count - payload_start_k;
                                        bpn = int'({h[7], h[6]});
                                        if (bpn > 0) begin
                                            n_idx = p_offset / bpn;
                                            b_idx = p_offset % bpn;
                                        end else begin
                                            n_idx = 0;
                                            b_idx = 0;
                                        end
                                        l_idx = int'(h[1]);

                                        if (l_idx < NUM_COMPUTE_LAYERS && n_idx < MAX_NEURONS) begin
                                            if (h[0] == 8'd0) begin
                                                weight_mem[l_idx][n_idx][b_idx*8 +: 8] <= config_data[k*8 +: 8];
                                            end else if (h[0] == 8'd1 && l_idx < NUM_COMPUTE_LAYERS-1) begin
                                                thresh_mem[l_idx][n_idx][b_idx*8 +: 8] <= config_data[k*8 +: 8];
                                            end
                                        end
                                    end
                                    p_count++;
                                end
                            end

                            payload_added = (valid_in_beat > payload_start_k) ? (valid_in_beat - payload_start_k) : 0;
                            payload_byte_count <= payload_added;

                            if (payload_added >= total_b_val) begin
                                if (h[0] == 8'd0 && int'(h[1]) == (NUM_COMPUTE_LAYERS - 1)) begin
                                    cfg_state   <= CFG_DONE;
                                    config_done <= 1'b1;
                                end else begin
                                    cfg_state <= CFG_HEADER;
                                end
                            end else begin
                                cfg_state <= CFG_PAYLOAD;
                            end
                        end
                    end
                end

                CFG_PAYLOAD: begin
                    if (config_valid && config_ready) begin
                        int bytes_added;
                        int bpn;
                        int l_idx;
                        bytes_added = 0;
                        bpn = int'(cur_bytes_per_neuron);
                        l_idx = int'(cur_layer_id);

                        for (int k = 0; k < CFG_BYTES_PER_BEAT; k++) begin
                            if (config_keep[k]) begin
                                int p_idx;
                                int n_idx;
                                int b_idx;
                                p_idx = payload_byte_count + bytes_added;
                                if (bpn > 0) begin
                                    n_idx = p_idx / bpn;
                                    b_idx = p_idx % bpn;
                                end else begin
                                    n_idx = 0;
                                    b_idx = 0;
                                end

                                if (l_idx < NUM_COMPUTE_LAYERS && n_idx < MAX_NEURONS) begin
                                    if (cur_msg_type == 8'd0) begin
                                        weight_mem[l_idx][n_idx][b_idx*8 +: 8] <= config_data[k*8 +: 8];
                                    end else if (cur_msg_type == 8'd1 && l_idx < NUM_COMPUTE_LAYERS-1) begin
                                        thresh_mem[l_idx][n_idx][b_idx*8 +: 8] <= config_data[k*8 +: 8];
                                    end
                                end
                                bytes_added++;
                            end
                        end

                        if (payload_byte_count + bytes_added >= int'(cur_total_bytes) || config_last) begin
                            payload_byte_count <= 0;
                            header_byte_count  <= 0;

                            if (cur_msg_type == 8'd0 && l_idx == (NUM_COMPUTE_LAYERS - 1)) begin
                                cfg_state   <= CFG_DONE;
                                config_done <= 1'b1;
                            end else begin
                                cfg_state <= CFG_HEADER;
                            end
                        end else begin
                            payload_byte_count <= payload_byte_count + bytes_added;
                        end
                    end
                end

                CFG_DONE: begin
                    config_done <= 1'b1;
                end

                default: cfg_state <= CFG_HEADER;
            endcase
        end
    end

    // =========================================================================
    // Image Input Buffer & Binarization
    // =========================================================================
    logic [TOPOLOGY[0]-1:0] img_buffer;
    int pixel_count;

    // =========================================================================
    // Inference Engine FSM & Layer Registers
    // =========================================================================
    typedef enum logic [2:0] {
        INF_IDLE,
        INF_RX_IMAGE,
        INF_EVAL_LAYER,
        INF_ARGMAX,
        INF_TX_OUTPUT
    } inf_state_t;

    inf_state_t inf_state;

    int current_layer;
    logic [MAX_NEURONS-1:0] current_act;

    // Classification Result Signals
    logic [OUTPUT_BUS_WIDTH-1:0] classified_result;

    assign data_in_ready = (inf_state == INF_RX_IMAGE && config_done);

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            inf_state         <= INF_IDLE;
            pixel_count       <= 0;
            current_layer     <= 0;
            img_buffer        <= '0;
            current_act       <= '0;
            classified_result <= '0;

            data_out_valid    <= 1'b0;
            data_out_data     <= '0;
            data_out_keep     <= '0;
            data_out_last     <= 1'b0;
        end else begin
            case (inf_state)
                INF_IDLE: begin
                    data_out_valid <= 1'b0;
                    data_out_last  <= 1'b0;
                    if (config_done) begin
                        inf_state   <= INF_RX_IMAGE;
                        pixel_count <= 0;
                    end
                end

                INF_RX_IMAGE: begin
                    if (data_in_valid && data_in_ready) begin
                        int pix_added;
                        pix_added = 0;

                        for (int k = 0; k < IN_BYTES_PER_BEAT; k++) begin
                            if (data_in_keep[k]) begin
                                if (pixel_count + pix_added < TOPOLOGY[0]) begin
                                    logic [7:0] pix;
                                    pix = data_in_data[k*8 +: 8];
                                    img_buffer[pixel_count + pix_added] <= (pix >= 8'h80) ? 1'b1 : 1'b0;
                                    pix_added++;
                                end
                            end
                        end

                        if (data_in_last || (pixel_count + pix_added >= TOPOLOGY[0])) begin
                            pixel_count   <= 0;
                            current_layer <= 0;
                            inf_state     <= INF_EVAL_LAYER;
                        end else begin
                            pixel_count <= pixel_count + pix_added;
                        end
                    end
                end

                INF_EVAL_LAYER: begin
                    if (current_layer < NUM_COMPUTE_LAYERS - 1) begin
                        int fan_in;
                        int n_neurons;
                        fan_in    = TOPOLOGY[current_layer];
                        n_neurons = TOPOLOGY[current_layer + 1];

                        for (int n = 0; n < MAX_NEURONS; n++) begin
                            if (n < n_neurons) begin
                                int popc;
                                popc = 0;
                                for (int i = 0; i < MAX_FAN_IN; i++) begin
                                    if (i < fan_in) begin
                                        bit in_b;
                                        bit w_b;
                                        in_b = (current_layer == 0) ? img_buffer[i] : current_act[i];
                                        w_b  = weight_mem[current_layer][n][i];
                                        if (in_b == w_b) popc++;
                                    end
                                end
                                current_act[n] <= (popc >= int'(thresh_mem[current_layer][n])) ? 1'b1 : 1'b0;
                            end else begin
                                current_act[n] <= 1'b0;
                            end
                        end

                        current_layer <= current_layer + 1;
                    end else begin
                        inf_state <= INF_ARGMAX;
                    end
                end

                INF_ARGMAX: begin
                    int out_layer;
                    int fan_in;
                    int n_neurons;
                    int max_val;
                    logic [OUTPUT_BUS_WIDTH-1:0] winner;

                    out_layer = NUM_COMPUTE_LAYERS - 1;
                    fan_in    = TOPOLOGY[out_layer];
                    n_neurons = TOPOLOGY[out_layer + 1];

                    max_val = -1;
                    winner  = '0;

                    for (int n = 0; n < MAX_NEURONS; n++) begin
                        if (n < n_neurons) begin
                            int popc;
                            popc = 0;
                            for (int i = 0; i < MAX_FAN_IN; i++) begin
                                if (i < fan_in) begin
                                    bit in_b;
                                    bit w_b;
                                    in_b = (out_layer == 0) ? img_buffer[i] : current_act[i];
                                    w_b  = weight_mem[out_layer][n][i];
                                    if (in_b == w_b) popc++;
                                end
                            end

                            if (popc > max_val) begin
                                max_val = popc;
                                winner  = OUTPUT_BUS_WIDTH'(n);
                            end
                        end
                    end

                    classified_result <= winner;
                    inf_state         <= INF_TX_OUTPUT;
                end

                INF_TX_OUTPUT: begin
                    data_out_valid <= 1'b1;
                    data_out_data  <= classified_result;
                    data_out_keep  <= '1;
                    data_out_last  <= 1'b1;

                    if (data_out_valid && data_out_ready) begin
                        data_out_valid <= 1'b0;
                        data_out_last  <= 1'b0;
                        pixel_count    <= 0;
                        inf_state      <= INF_RX_IMAGE;
                    end
                end

                default: inf_state <= INF_IDLE;
            endcase
        end
    end

    // Dummy references to prevent lint warnings for unused internal decode signals
    /* verilator lint_off UNUSEDSIGNAL */
    wire [31:0] unused_decodes = {16'b0, cur_layer_inputs} | {16'b0, cur_num_neurons};
    /* verilator lint_on UNUSEDSIGNAL */

endmodule
