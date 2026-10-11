// Layer engine: PN neuron processors share one PW-bit input word per cycle.
// Sequencer issues one (g, p) per cycle (p inner, g outer) from the full input bank; an image starts
// in the same cycle its first (g, p) issues (img_start), as soon as the input bank is full and the
// output side can take it (hidden: output bank neither full nor awaiting this layer's commit;
// output layer: start_ok). The next image may start the cycle after the last (g, p) of the previous.
// Weight RAM (GROUPS*PASSES x PW) and threshold RAM (GROUPS x THR_W) per lane, 1-cycle registered
// read aligned with the ping-pong read. Results leave through one register stage; the output group
// index and bank are tracked by counters at the output (results come out in issue order).
// Layout contract: neuron n -> lane n % PN, group n / PN; weight word at w_addr = group*PASSES + p,
// bit k = weight index p*PW + k (pad bits are 1); threshold at t_addr = group.
module bnn_layer import bnn_fcc_pkg::*; #(
  parameter int    F                         = 8,
  parameter int    N                         = 8,
  parameter int    PW                        = 8,
  parameter int    PN                        = 8,
  parameter bit    IS_OUTPUT                 = 1'b0,
  parameter int    POPCOUNT_LEVELS_PER_STAGE = 3,
  parameter string RAM_STYLE                 = "auto"
) (
  input  logic                                    clk,
  input  logic                                    rst,
  // config writes (from cfg_layer_writer)
  input  logic                                    w_we,
  input  logic [bits(PN)-1:0]                     w_lane,
  input  logic [bits(cdiv(N,PN)*cdiv(F,PW))-1:0]  w_addr,
  input  logic [PW-1:0]                           w_data,
  /* verilator lint_off UNUSEDSIGNAL */   // threshold port unused when IS_OUTPUT
  input  logic                                    t_we,
  input  logic [bits(PN)-1:0]                     t_lane,
  input  logic [bits(cdiv(N,PN))-1:0]             t_addr,
  input  logic [bits(F+2)-1:0]                    t_data,
  /* verilator lint_on UNUSEDSIGNAL */
  // input ping-pong (consumer side)
  input  logic [1:0]                              in_full,
  output logic                                    in_rd_bank,
  output logic [bits(cdiv(F,PW))-1:0]             in_rd_idx,
  input  logic [PW-1:0]                           in_rd_data,
  output logic                                    in_rls_en,
  output logic                                    in_rls_bank,
  // hidden-layer output ping-pong (producer side); out_full tied to 0 when IS_OUTPUT
  input  logic [1:0]                              out_full,
  output logic                                    out_wr_en,
  output logic                                    out_wr_bank,
  output logic [bits(cdiv(N,PN))-1:0]             out_wr_idx,
  output logic [PN-1:0]                           out_wr_data,
  output logic                                    out_commit_en,
  output logic                                    out_commit_bank,
  // output-layer result stream; start_ok tied to 1 when !IS_OUTPUT
  input  logic                                    start_ok,
  output logic                                    res_valid,
  output logic [PN*bits(F+1)-1:0]                 res_counts,
  output logic [PN-1:0]                           res_mask,
  output logic                                    res_last,
  output logic                                    img_start,
  output logic                                    busy
);
  localparam int GROUPS    = cdiv(N, PN);
  localparam int PASSES    = cdiv(F, PW);
  localparam int ACC_W     = bits(F + 1);
  localparam int THR_W     = bits(F + 2);
  localparam int LANE_W    = bits(PN);
  localparam int WADDR_W   = bits(GROUPS * PASSES);
  localparam int TADDR_W   = bits(GROUPS);
  localparam int IN_IDX_W  = bits(PASSES);
  localparam int OUT_IDX_W = bits(GROUPS);
  localparam int NPL       = np_latency(PW, POPCOUNT_LEVELS_PER_STAGE);
  localparam int LAST_LANES = N - (GROUPS - 1) * PN;          // valid lanes in the last group
  localparam logic [PN-1:0] LAST_MASK = {PN{1'b1}} >> (PN - LAST_LANES);

  // ---------------- sequencer
  logic                 busy_img;    // an image is mid-issue; (g_r, p_r) is its next (g, p)
  logic [IN_IDX_W-1:0]  p_r;
  logic [OUT_IDX_W-1:0] g_r;
  logic [WADDR_W-1:0]   waddr_r;     // = g_r*PASSES + p_r
  logic                 rd_bank, wr_bank;
  logic [1:0]           pending;     // output bank started but not yet committed (hidden only)
  logic                 last_p, last_g, last_gp, out_ok, start, issue;

  assign last_p  = (p_r == IN_IDX_W'(PASSES - 1));
  assign last_g  = (g_r == OUT_IDX_W'(GROUPS - 1));
  assign last_gp = last_p & last_g;
  assign out_ok  = start_ok & (IS_OUTPUT | (~out_full[wr_bank] & ~pending[wr_bank]));
  assign start   = ~busy_img & in_full[rd_bank] & out_ok;
  assign issue   = busy_img | start;

  assign img_start   = start;
  assign in_rd_bank  = rd_bank;
  assign in_rd_idx   = p_r;
  assign in_rls_en   = issue & last_gp;   // read issued this cycle still returns this bank's data
  assign in_rls_bank = rd_bank;

  always_ff @(posedge clk) begin
    if (rst) begin
      busy_img <= 1'b0;
      p_r      <= '0;
      g_r      <= '0;
      waddr_r  <= '0;
      rd_bank  <= 1'b0;
      wr_bank  <= 1'b0;
    end else if (issue) begin
      if (last_gp) begin
        busy_img <= 1'b0;
        p_r      <= '0;
        g_r      <= '0;
        waddr_r  <= '0;
        rd_bank  <= ~rd_bank;
        wr_bank  <= ~wr_bank;
      end else begin
        busy_img <= 1'b1;
        waddr_r  <= waddr_r + 1'b1;
        if (last_p) begin
          p_r <= '0;
          g_r <= g_r + 1'b1;
        end else begin
          p_r <= p_r + 1'b1;
        end
      end
    end
  end

  // ---------------- side-band for the NP input stage (aligned with the 1-cycle RAM/ping-pong read)
  logic           s1_first, s1_last;
  logic [NPL+1:0] vpipe;   // vpipe[0]: NP input valid; vpipe[NPL+1]: result register stage

  always_ff @(posedge clk) begin
    s1_first <= (p_r == '0);
    s1_last  <= last_p;
  end

  always_ff @(posedge clk) begin
    if (rst) vpipe <= '0;
    else     vpipe <= {vpipe[NPL:0], issue};
  end

  assign busy = issue | (|vpipe);

  // ---------------- lanes: weight RAM, threshold RAM, neuron processor
  /* verilator lint_off UNUSEDSIGNAL */  // lanes run in lockstep (only np_ov[0] used); y or acc per mode
  logic [PN-1:0]       np_ov;
  logic [PN-1:0]       np_y;
  logic [PN*ACC_W-1:0] np_acc;
  /* verilator lint_on UNUSEDSIGNAL */

  for (genvar k = 0; k < PN; k++) begin : g_lane
    logic             w_lane_we;
    logic [PW-1:0]    w_rd;
    logic [THR_W-1:0] t_rd;

    assign w_lane_we = w_we & (w_lane == LANE_W'(k));

    if (RAM_STYLE == "block") begin : g_wram
      (* ram_style = "block" *) logic [PW-1:0] wmem [GROUPS*PASSES];
      always_ff @(posedge clk) begin
        if (w_lane_we) wmem[w_addr] <= w_data;
        w_rd <= wmem[waddr_r];
      end
    end else if (RAM_STYLE == "distributed") begin : g_wram
      (* ram_style = "distributed" *) logic [PW-1:0] wmem [GROUPS*PASSES];
      always_ff @(posedge clk) begin
        if (w_lane_we) wmem[w_addr] <= w_data;
        w_rd <= wmem[waddr_r];
      end
    end else begin : g_wram
      logic [PW-1:0] wmem [GROUPS*PASSES];
      always_ff @(posedge clk) begin
        if (w_lane_we) wmem[w_addr] <= w_data;
        w_rd <= wmem[waddr_r];
      end
    end

    if (!IS_OUTPUT) begin : g_tram
      logic [THR_W-1:0] tmem [GROUPS];
      always_ff @(posedge clk) begin
        if (t_we && t_lane == LANE_W'(k)) tmem[t_addr] <= t_data;
        t_rd <= tmem[TADDR_W'(g_r)];
      end
    end else begin : g_no_tram
      assign t_rd = '0;
    end

    neuron_processor #(
      .PW(PW), .ACC_W(ACC_W), .THR_W(THR_W),
      .POPCOUNT_LEVELS_PER_STAGE(POPCOUNT_LEVELS_PER_STAGE)
    ) u_np (
      .clk(clk), .rst(rst),
      .in_valid(vpipe[0]), .first_pass(s1_first), .last_pass(s1_last),
      .x(in_rd_data), .w(w_rd), .thr(t_rd),
      .out_valid(np_ov[k]), .out_y(np_y[k]), .out_acc(np_acc[k*ACC_W +: ACC_W])
    );
  end

  // ---------------- result stage: group/bank counters and one output register
  logic                 np_valid;    // all lanes are in lockstep; lane 0 stands for all
  logic [OUT_IDX_W-1:0] g_o;
  logic                 bank_o, last_o;
  logic [PN-1:0]        mask_o;
  logic                 o_valid, o_last;

  assign np_valid = np_ov[0];
  assign last_o   = (g_o == OUT_IDX_W'(GROUPS - 1));
  assign mask_o   = last_o ? LAST_MASK : '1;

  always_ff @(posedge clk) begin
    if (rst) begin
      g_o     <= '0;
      bank_o  <= 1'b0;
      o_valid <= 1'b0;
    end else begin
      o_valid <= np_valid;
      if (np_valid) begin
        g_o <= last_o ? '0 : g_o + 1'b1;
        if (last_o) bank_o <= ~bank_o;
      end
    end
  end

  // NP results change on every pass, so all result registers capture in the np_valid cycle.
  always_ff @(posedge clk) if (np_valid) o_last <= last_o;

  always_ff @(posedge clk) begin
    if (rst) pending <= '0;
    else begin
      if (out_commit_en) pending[out_commit_bank] <= 1'b0;
      if (start)         pending[wr_bank]         <= 1'b1;
    end
  end

  if (!IS_OUTPUT) begin : g_hidden_out
    logic [PN-1:0]        o_y;
    logic                 o_bank;
    logic [OUT_IDX_W-1:0] o_idx;
    always_ff @(posedge clk) begin
      if (np_valid) begin
        o_y    <= np_y & mask_o;   // invalid lanes forced to 0
        o_bank <= bank_o;
        o_idx  <= g_o;
      end
    end

    assign out_wr_en       = o_valid;
    assign out_wr_bank     = o_bank;
    assign out_wr_idx      = o_idx;
    assign out_wr_data     = o_y;
    assign out_commit_en   = o_valid & o_last;
    assign out_commit_bank = o_bank;
    assign res_valid       = 1'b0;
    assign res_counts      = '0;
    assign res_mask        = '0;
    assign res_last        = 1'b0;
  end else begin : g_result_out
    logic [PN*ACC_W-1:0] o_acc;
    logic [PN-1:0]       o_mask;
    always_ff @(posedge clk) begin
      if (np_valid) begin
        o_acc  <= np_acc;
        o_mask <= mask_o;
      end
    end

    assign res_valid       = o_valid;
    assign res_counts      = o_acc;
    assign res_mask        = o_mask;
    assign res_last        = o_valid & o_last;
    assign out_wr_en       = 1'b0;
    assign out_wr_bank     = 1'b0;
    assign out_wr_idx      = '0;
    assign out_wr_data     = '0;
    assign out_commit_en   = 1'b0;
    assign out_commit_bank = 1'b0;
  end
endmodule
