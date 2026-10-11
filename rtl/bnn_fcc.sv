// Binary neural network fully connected classifier (AXI4-Stream), layer-pipelined.
//   config AXIS -> cfg_byte_serializer -> cfg_parser -> cfg_layer_writer[l] -> bnn_layer[l] RAMs
//   data_in AXIS -> input_packer -> pingpong[0] -> bnn_layer[0] -> pingpong[1] -> ... -> bnn_layer[L-1]
//                -> argmax -> axis_out -> data_out AXIS
// Layer l (l = 0..L-1, L = TOTAL_LAYERS-1) has fan-in F = TOPOLOGY[l], N = TOPOLOGY[l+1] neurons,
// effective parallelism PN = min(PARALLEL_NEURONS[l], N), PW = min(PARALLEL_INPUTS, F).
// g_layer[l] holds layer l's input ping-pong buffer (written by input_packer or layer l-1), its
// config writer and its bnn_layer.
// Glue:
//   accept_new    (registered) = configured & !cfg_busy & all writers idle: images are admitted only
//                 when every memory is loaded and every writer has finished its last RAM write.
//   pipeline_idle (registered) = no image anywhere (input packer, ping-pong banks, layers, argmax)
//                 and no image beat accepted this cycle (accept_new is one cycle stale when the
//                 parser enters WAIT; the parser ignores the first WAIT cycle).
//   credits: the output layer starts an image only while
//            axis_out.count + in_flight + (image starting now) < OUT_FIFO_DEPTH (registered,
//            conservative), so the output FIFO never overflows.
module bnn_fcc import bnn_fcc_pkg::*; #(
    parameter int INPUT_DATA_WIDTH  = 8,
    parameter int INPUT_BUS_WIDTH   = 64,
    parameter int CONFIG_BUS_WIDTH  = 64,
    parameter int OUTPUT_DATA_WIDTH = 4,
    parameter int OUTPUT_BUS_WIDTH  = 8,

    parameter int TOTAL_LAYERS = 4,  // Includes input, hidden, and output
    parameter int TOPOLOGY[TOTAL_LAYERS] = '{0: 784, 1: 256, 2: 256, 3: 10, default: 0},  // 0: input, TOTAL_LAYERS-1: output

    parameter int PARALLEL_INPUTS = 8,
    parameter int PARALLEL_NEURONS[TOTAL_LAYERS-1] = '{default: 8},

    parameter int    POPCOUNT_LEVELS_PER_STAGE = 3,
    parameter int    ARGMAX_LEVELS_PER_STAGE   = 2,
    parameter int    OUT_FIFO_DEPTH            = 4,
    parameter string RAM_STYLE                 = "auto"
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
    localparam int L     = TOTAL_LAYERS - 1;                       // non-input layers
    localparam int F0    = TOPOLOGY[0];
    localparam int PW0   = imin(PARALLEL_INPUTS, F0);
    localparam int F_O   = TOPOLOGY[L - 1];                        // output layer
    localparam int N_O   = TOPOLOGY[L];
    localparam int PN_O  = imin(PARALLEL_NEURONS[L - 1], N_O);
    localparam int ACC_O = bits(F_O + 1);
    localparam int IDX_W = bits(N_O);
    localparam int CRD_W = bits(2 * OUT_FIFO_DEPTH + 2);

    // ---------------- elaboration-time checks (spec §3.1; generate-scope so synthesis enforces them too)
    if (TOTAL_LAYERS < 2) begin : g_chk1
        $fatal(1, "bnn_fcc: TOTAL_LAYERS must be >= 2");
    end
    if (PARALLEL_INPUTS < 1) begin : g_chk2
        $fatal(1, "bnn_fcc: PARALLEL_INPUTS must be >= 1");
    end
    for (genvar i = 0; i < TOTAL_LAYERS; i++) begin : g_chk_topo
        if (TOPOLOGY[i] < 1 || TOPOLOGY[i] >= 65536) begin : g_bad
            $fatal(1, "bnn_fcc: TOPOLOGY[%0d] = %0d out of range 1..65535", i, TOPOLOGY[i]);
        end
    end
    for (genvar l = 0; l < TOTAL_LAYERS - 1; l++) begin : g_chk_pn
        if (PARALLEL_NEURONS[l] < 1) begin : g_bad
            $fatal(1, "bnn_fcc: PARALLEL_NEURONS[%0d] must be >= 1", l);
        end
    end
    if (INPUT_DATA_WIDTH % 8 != 0) begin : g_chk3
        $fatal(1, "bnn_fcc: INPUT_DATA_WIDTH must be a multiple of 8");
    end
    if (INPUT_BUS_WIDTH % INPUT_DATA_WIDTH != 0) begin : g_chk4
        $fatal(1, "bnn_fcc: INPUT_BUS_WIDTH must be a multiple of INPUT_DATA_WIDTH");
    end
    if (CONFIG_BUS_WIDTH % 8 != 0) begin : g_chk5
        $fatal(1, "bnn_fcc: CONFIG_BUS_WIDTH must be a multiple of 8");
    end
    if (bits(N_O) > OUTPUT_DATA_WIDTH) begin : g_chk6
        $fatal(1, "bnn_fcc: OUTPUT_DATA_WIDTH too small for %0d classes", N_O);
    end
    if (OUTPUT_DATA_WIDTH > OUTPUT_BUS_WIDTH) begin : g_chk7
        $fatal(1, "bnn_fcc: OUTPUT_DATA_WIDTH must be <= OUTPUT_BUS_WIDTH");
    end
    if (OUT_FIFO_DEPTH < 1) begin : g_chk8
        $fatal(1, "bnn_fcc: OUT_FIFO_DEPTH must be >= 1");
    end
    if (POPCOUNT_LEVELS_PER_STAGE < 1) begin : g_chk9
        $fatal(1, "bnn_fcc: POPCOUNT_LEVELS_PER_STAGE must be >= 1");
    end
    if (ARGMAX_LEVELS_PER_STAGE < 1) begin : g_chk10
        $fatal(1, "bnn_fcc: ARGMAX_LEVELS_PER_STAGE must be >= 1");
    end
    if (RAM_STYLE != "auto" && RAM_STYLE != "block" && RAM_STYLE != "distributed") begin : g_chk11
        $fatal(1, "bnn_fcc: RAM_STYLE must be \"auto\", \"block\" or \"distributed\"");
    end

    // ---------------- configuration path
    logic       sr_valid, sr_ready, sr_null, sr_beat_last, drop_rest;
    logic [7:0] sr_byte;
    /* verilator lint_off UNUSEDSIGNAL */
    logic       sr_end_of_beat;   // not needed by the parser
    /* verilator lint_on UNUSEDSIGNAL */

    cfg_byte_serializer #(.BUS_W(CONFIG_BUS_WIDTH)) u_ser (
        .clk(clk), .rst(rst),
        .s_valid(config_valid), .s_ready(config_ready), .s_data(config_data),
        .s_keep(config_keep), .s_last(config_last),
        .m_valid(sr_valid), .m_ready(sr_ready), .m_byte(sr_byte), .m_null(sr_null),
        .m_end_of_beat(sr_end_of_beat), .m_beat_last(sr_beat_last), .drop_rest(drop_rest)
    );

    logic       pipeline_idle_q, cfg_busy, configured;
    logic       msg_start, msg_abort, msg_type, pl_valid, pl_ready;
    logic [7:0] msg_layer, pl_byte;

    cfg_parser #(.TOTAL_LAYERS(TOTAL_LAYERS), .TOPOLOGY(TOPOLOGY)) u_parser (
        .clk(clk), .rst(rst),
        .s_valid(sr_valid), .s_ready(sr_ready), .s_byte(sr_byte), .s_null(sr_null),
        .s_beat_last(sr_beat_last), .drop_rest(drop_rest),
        .pipeline_idle(pipeline_idle_q), .cfg_busy(cfg_busy), .configured(configured),
        .msg_start(msg_start), .msg_abort(msg_abort), .msg_layer(msg_layer), .msg_type(msg_type),
        .pl_valid(pl_valid), .pl_ready(pl_ready), .pl_byte(pl_byte)
    );

    logic [L-1:0] cw_sel, cw_ready, cw_idle;     // per-layer writer routing / status
    assign pl_ready = |(cw_sel & cw_ready);

    // ---------------- image input
    logic                          accept_new_q;
    logic [1:0]                    l0_full;     // layer-0 ping-pong full flags (driven in g_layer[0])
    logic                          ip_wr_en, ip_wr_bank, ip_commit_en, ip_commit_bank, ip_mid;
    logic [bits(cdiv(F0,PW0))-1:0] ip_wr_idx;
    logic [PW0-1:0]                ip_wr_data;

    input_packer #(
        .DATA_W(INPUT_DATA_WIDTH), .BUS_W(INPUT_BUS_WIDTH), .F0(F0), .PW0(PW0)
    ) u_in (
        .clk(clk), .rst(rst),
        .s_valid(data_in_valid), .s_ready(data_in_ready), .s_data(data_in_data),
        .s_keep(data_in_keep), .s_last(data_in_last),
        .accept_new(accept_new_q), .buf_full(l0_full),
        .wr_en(ip_wr_en), .wr_bank(ip_wr_bank), .wr_idx(ip_wr_idx), .wr_data(ip_wr_data),
        .commit_en(ip_commit_en), .commit_bank(ip_commit_bank), .mid_image(ip_mid)
    );

    // ---------------- layers
    logic [L-1:0] lyr_busy, pp_any_full;
    logic         start_ok_q;

    for (genvar l = 0; l < L; l++) begin : g_layer
        localparam int  F_L      = TOPOLOGY[l];
        localparam int  N_L      = TOPOLOGY[l + 1];
        localparam int  PN_L     = imin(PARALLEL_NEURONS[l], N_L);
        localparam int  PW_L     = imin(PARALLEL_INPUTS, F_L);
        // producer chunk width: layer-0 words from input_packer, else the previous layer's PN
        localparam int  WRW_L    = (l == 0) ? PW_L : imin(PARALLEL_NEURONS[(l == 0) ? 0 : l - 1], F_L);
        localparam int  GROUPS_L = cdiv(N_L, PN_L);
        localparam int  PASSES_L = cdiv(F_L, PW_L);
        localparam bit  IS_OUT   = (l == L - 1);

        // input ping-pong
        logic [1:0]                         in_full;
        logic                               pp_wr_en, pp_wr_bank, pp_commit_en, pp_commit_bank;
        logic [bits(cdiv(F_L,WRW_L))-1:0]   pp_wr_idx;
        logic [WRW_L-1:0]                   pp_wr_data;
        logic                               rd_bank, rls_en, rls_bank;
        logic [bits(PASSES_L)-1:0]          rd_idx;
        logic [PW_L-1:0]                    rd_data;

        // layer outputs (hidden: next ping-pong; output: result stream)
        /* verilator lint_off UNUSEDSIGNAL */   // unused half depends on IS_OUT
        logic [1:0]                         out_full;
        logic                               out_wr_en, out_wr_bank, out_commit_en, out_commit_bank;
        logic [bits(GROUPS_L)-1:0]          out_wr_idx;
        logic [PN_L-1:0]                    out_wr_data;
        logic                               start_ok, res_valid, res_last, img_start, busy;
        logic [PN_L*bits(F_L+1)-1:0]        res_counts;
        logic [PN_L-1:0]                    res_mask;
        /* verilator lint_on UNUSEDSIGNAL */

        // config writer
        logic                               cw_start, cw_abort, cw_valid;
        logic                               w_we, t_we;
        logic [bits(PN_L)-1:0]              w_lane, t_lane;
        logic [bits(GROUPS_L*PASSES_L)-1:0] w_addr;
        logic [PW_L-1:0]                    w_data;
        logic [bits(GROUPS_L)-1:0]          t_addr;
        logic [bits(F_L+2)-1:0]             t_data;

        if (l == 0) begin : g_src
            assign pp_wr_en       = ip_wr_en;
            assign pp_wr_bank     = ip_wr_bank;
            assign pp_wr_idx      = ip_wr_idx;
            assign pp_wr_data     = ip_wr_data;
            assign pp_commit_en   = ip_commit_en;
            assign pp_commit_bank = ip_commit_bank;
            assign l0_full        = in_full;
        end else begin : g_src
            assign pp_wr_en       = g_layer[l-1].out_wr_en;
            assign pp_wr_bank     = g_layer[l-1].out_wr_bank;
            assign pp_wr_idx      = g_layer[l-1].out_wr_idx;
            assign pp_wr_data     = g_layer[l-1].out_wr_data;
            assign pp_commit_en   = g_layer[l-1].out_commit_en;
            assign pp_commit_bank = g_layer[l-1].out_commit_bank;
        end

        if (IS_OUT) begin : g_dst
            assign out_full = 2'b00;
            assign start_ok = start_ok_q;
        end else begin : g_dst
            assign out_full = g_layer[l+1].in_full;
            assign start_ok = 1'b1;
        end

        pingpong_buf #(.NBITS(F_L), .WR_W(WRW_L), .RD_W(PW_L)) u_pp (
            .clk(clk), .rst(rst),
            .wr_en(pp_wr_en), .wr_bank(pp_wr_bank), .wr_idx(pp_wr_idx), .wr_data(pp_wr_data),
            .commit_en(pp_commit_en), .commit_bank(pp_commit_bank),
            .rd_bank(rd_bank), .rd_idx(rd_idx), .rd_data(rd_data),
            .rls_en(rls_en), .rls_bank(rls_bank), .full(in_full)
        );

        assign cw_sel[l] = (msg_layer == 8'(l));
        assign cw_start  = msg_start && cw_sel[l];
        assign cw_abort  = msg_abort && cw_sel[l];
        assign cw_valid  = pl_valid && cw_sel[l];

        cfg_layer_writer #(.F(F_L), .N(N_L), .PW(PW_L), .PN(PN_L), .HAS_THR(!IS_OUT)) u_cw (
            .clk(clk), .rst(rst),
            .start(cw_start), .abort(cw_abort), .msg_type(msg_type),
            .s_valid(cw_valid), .s_ready(cw_ready[l]), .s_byte(pl_byte),
            .w_we(w_we), .w_lane(w_lane), .w_addr(w_addr), .w_data(w_data),
            .t_we(t_we), .t_lane(t_lane), .t_addr(t_addr), .t_data(t_data),
            .idle(cw_idle[l])
        );

        bnn_layer #(
            .F(F_L), .N(N_L), .PW(PW_L), .PN(PN_L), .IS_OUTPUT(IS_OUT),
            .POPCOUNT_LEVELS_PER_STAGE(POPCOUNT_LEVELS_PER_STAGE), .RAM_STYLE(RAM_STYLE)
        ) u_layer (
            .clk(clk), .rst(rst),
            .w_we(w_we), .w_lane(w_lane), .w_addr(w_addr), .w_data(w_data),
            .t_we(t_we), .t_lane(t_lane), .t_addr(t_addr), .t_data(t_data),
            .in_full(in_full), .in_rd_bank(rd_bank), .in_rd_idx(rd_idx), .in_rd_data(rd_data),
            .in_rls_en(rls_en), .in_rls_bank(rls_bank),
            .out_full(out_full), .out_wr_en(out_wr_en), .out_wr_bank(out_wr_bank),
            .out_wr_idx(out_wr_idx), .out_wr_data(out_wr_data),
            .out_commit_en(out_commit_en), .out_commit_bank(out_commit_bank),
            .start_ok(start_ok), .res_valid(res_valid), .res_counts(res_counts),
            .res_mask(res_mask), .res_last(res_last), .img_start(img_start), .busy(busy)
        );

        assign lyr_busy[l]    = busy;
        assign pp_any_full[l] = |in_full;
    end

    // ---------------- argmax and output
    logic             am_valid, am_busy;
    logic [IDX_W-1:0] am_idx;

    argmax #(
        .PN(PN_O), .ACC_W(ACC_O), .N(N_O), .IDX_W(IDX_W), .LEVELS_PER_STAGE(ARGMAX_LEVELS_PER_STAGE)
    ) u_argmax (
        .clk(clk), .rst(rst),
        .in_valid(g_layer[L-1].res_valid), .in_counts(g_layer[L-1].res_counts),
        .in_mask(g_layer[L-1].res_mask), .in_last(g_layer[L-1].res_last),
        .out_valid(am_valid), .out_idx(am_idx), .busy(am_busy)
    );

    logic [bits(OUT_FIFO_DEPTH+1)-1:0] ao_count;

    axis_out #(.DATA_W(OUTPUT_DATA_WIDTH), .BUS_W(OUTPUT_BUS_WIDTH), .DEPTH(OUT_FIFO_DEPTH)) u_out (
        .clk(clk), .rst(rst),
        .push(am_valid), .push_data(OUTPUT_DATA_WIDTH'(am_idx)), .count(ao_count),
        .m_valid(data_out_valid), .m_ready(data_out_ready), .m_data(data_out_data),
        .m_keep(data_out_keep), .m_last(data_out_last)
    );

    // ---------------- glue: credits, admission, idle
    logic [CRD_W-1:0] in_flight_q;   // output-layer images started, result not yet pushed
    logic             out_start;
    assign out_start = g_layer[L-1].img_start;

    always_ff @(posedge clk) begin
        if (rst) begin
            in_flight_q     <= '0;
            start_ok_q      <= 1'b0;
            accept_new_q    <= 1'b0;
            pipeline_idle_q <= 1'b0;
        end else begin
            in_flight_q <= in_flight_q + CRD_W'(out_start) - CRD_W'(am_valid);
            // occupancy next cycle <= count + in_flight + out_start (a push this cycle moves an
            // image from in_flight to count, already counted once)
            start_ok_q  <= (CRD_W'(ao_count) + in_flight_q + CRD_W'(out_start)) < CRD_W'(OUT_FIFO_DEPTH);
            accept_new_q    <= configured && !cfg_busy && (&cw_idle);
            pipeline_idle_q <= !ip_mid && !(data_in_valid && data_in_ready) &&
                               !(|pp_any_full) && !(|lyr_busy) && !am_busy;
        end
    end

endmodule
