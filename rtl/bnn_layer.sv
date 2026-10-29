// Layer engine: PN neuron processors share one PW-bit input word per cycle.
// Sequencer issues one (g, p) per cycle (p inner, g outer) from the full input bank; an image starts
// in the same cycle its first (g, p) issues (img_start), as soon as the input bank is full and the
// output side can take it (hidden: output bank neither full nor awaiting this layer's commit;
// output layer: start_ok). The next image may start the cycle after the last (g, p) of the previous.
// Read timing: a (g, p) issued in cycle t advances the input ping-pong (2-cycle read: one rd_data copy
// per group of 8 lanes, valid in t+2). Each lane registers its own copy of the weight/threshold read
// address in t+1 and reads its weight RAM (GROUPS*PASSES x PW) and threshold RAM (GROUPS x THR_W)
// with a registered read, so x, w and thr reach the neuron processor together in t+2.
// Config writes pass a two-level register tree (lane group of 8, then lane) and reach the RAMs 2 cycles
// after the w_*/t_* ports; cfg_wr_busy is 1 while a write is in the tree. The image start does not
// look at it (the top waits for it before admitting images).
// Results leave through one register stage; the output group (one-hot) and bank are tracked at the
// output (results come out in issue order).
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
  output logic                                    in_rd_adv,
  input  logic [cdiv(PN,8)-1:0][PW-1:0]           in_rd_data,
  output logic                                    in_rls_en,
  output logic                                    in_rls_bank,
  // hidden-layer output ping-pong (producer side); out_full tied to 0 when IS_OUTPUT
  input  logic [1:0]                              out_full,
  output logic                                    out_wr_en,
  output logic                                    out_wr_bank,
  output logic [cdiv(N,PN)-1:0]                   out_wr_oh,
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
  output logic                                    busy,
  output logic                                    cfg_wr_busy
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
  localparam int LG        = 8;                              // lanes per operand copy / write group
  localparam int NG        = cdiv(PN, LG);
  localparam int LO_W      = (PN > LG) ? bits(LG) : LANE_W;  // lane index within a group
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
  assign in_rd_adv   = issue;
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

  // ---------------- side-band for the NP input stage (aligned with the 2-cycle ping-pong/RAM read)
  logic           s1_first, s1_last, s2_first, s2_last;
  logic [NPL+2:0] vpipe;   // vpipe[1]: NP input valid; vpipe[NPL+2]: result register stage

  always_ff @(posedge clk) begin
    s1_first <= (p_r == '0);
    s1_last  <= last_p;
    s2_first <= s1_first;
    s2_last  <= s1_last;
  end

  always_ff @(posedge clk) begin
    if (rst) vpipe <= '0;
    else     vpipe <= {vpipe[NPL+1:0], issue};
  end

  assign busy = issue | (|vpipe);

  // ---------------- config write tree, level 1: one register set per lane group, loaded only when
  // that group is written (so the copies differ); data gets a second group stage to stay aligned
  // with level 2 when the same group is written in consecutive cycles
  logic [NG-1:0]      gw_q, gt_q;
  logic [LO_W-1:0]    glane_q [NG], gtlane_q [NG];
  logic [WADDR_W-1:0] gaddr_q [NG];
  logic [PW-1:0]      gdata_q [NG], gdata2_q [NG];
  logic [TADDR_W-1:0] gtaddr_q [NG];
  /* verilator lint_off UNUSEDSIGNAL */   // threshold tree unused when IS_OUTPUT
  logic [THR_W-1:0]   gtdata_q [NG], gtdata2_q [NG];
  /* verilator lint_on UNUSEDSIGNAL */
  logic [PN-1:0]      lw_q, lt_q;   // level 2 write enables (per lane)

  for (genvar gi = 0; gi < NG; gi++) begin : g_wgrp
    logic wsel, tsel;
    assign wsel = w_we && (int'(w_lane) / LG == gi);
    assign tsel = !IS_OUTPUT && t_we && (int'(t_lane) / LG == gi);

    always_ff @(posedge clk) begin
      if (rst) begin
        gw_q[gi] <= 1'b0;
        gt_q[gi] <= 1'b0;
      end else begin
        gw_q[gi] <= wsel;
        gt_q[gi] <= tsel;
      end
      if (wsel) begin
        glane_q[gi] <= LO_W'(int'(w_lane) % LG);
        gaddr_q[gi] <= w_addr;
        gdata_q[gi] <= w_data;
      end
      if (tsel) begin
        gtlane_q[gi] <= LO_W'(int'(t_lane) % LG);
        gtaddr_q[gi] <= t_addr;
        gtdata_q[gi] <= t_data;
      end
      gdata2_q[gi]  <= gdata_q[gi];
      gtdata2_q[gi] <= gtdata_q[gi];
    end
  end

  assign cfg_wr_busy = (|gw_q) | (|gt_q) | (|lw_q) | (|lt_q);

  // ---------------- lanes: weight RAM, threshold RAM, neuron processor
  /* verilator lint_off UNUSEDSIGNAL */  // lanes run in lockstep (only np_ov[0] used); y or acc per mode
  logic [PN-1:0]       np_ov;
  logic [PN-1:0]       np_y;
  logic [PN*ACC_W-1:0] np_acc;
  /* verilator lint_on UNUSEDSIGNAL */

  for (genvar k = 0; k < PN; k++) begin : g_lane
    localparam int GI = k / LG;
    logic [PW-1:0]      w_rd;
    logic [THR_W-1:0]   t_rd;
    logic               lsel, ltsel;
    logic [WADDR_W-1:0] laddr_q;   // write address, level 2 (loads only for this lane)
    /* verilator lint_off UNUSEDSIGNAL */   // unused when IS_OUTPUT
    logic [TADDR_W-1:0] ltaddr_q;
    /* verilator lint_on UNUSEDSIGNAL */
    // per-lane read address copies (kept: identical across lanes, each drives only this lane's RAMs)
    (* keep = "true" *) logic [WADDR_W-1:0] la_q;

    assign lsel  = gw_q[GI] && (glane_q[GI] == LO_W'(k % LG));
    assign ltsel = gt_q[GI] && (gtlane_q[GI] == LO_W'(k % LG));

    always_ff @(posedge clk) begin
      if (rst) begin
        lw_q[k] <= 1'b0;
        lt_q[k] <= 1'b0;
      end else begin
        lw_q[k] <= lsel;
        lt_q[k] <= ltsel;
      end
      if (lsel)  laddr_q  <= gaddr_q[GI];
      if (ltsel) ltaddr_q <= gtaddr_q[GI];
      la_q <= waddr_r;
    end

    if (RAM_STYLE == "block") begin : g_wram
      (* ram_style = "block" *) logic [PW-1:0] wmem [GROUPS*PASSES];
      always_ff @(posedge clk) begin
        if (lw_q[k]) wmem[laddr_q] <= gdata2_q[GI];
        w_rd <= wmem[la_q];
      end
    end else if (RAM_STYLE == "distributed") begin : g_wram
      (* ram_style = "distributed" *) logic [PW-1:0] wmem [GROUPS*PASSES];
      always_ff @(posedge clk) begin
        if (lw_q[k]) wmem[laddr_q] <= gdata2_q[GI];
        w_rd <= wmem[la_q];
      end
    end else begin : g_wram
      logic [PW-1:0] wmem [GROUPS*PASSES];
      always_ff @(posedge clk) begin
        if (lw_q[k]) wmem[laddr_q] <= gdata2_q[GI];
        w_rd <= wmem[la_q];
      end
    end

    if (!IS_OUTPUT) begin : g_tram
      logic [THR_W-1:0] tmem [GROUPS];
      (* keep = "true" *) logic [TADDR_W-1:0] lg_q;
      always_ff @(posedge clk) lg_q <= TADDR_W'(g_r);
      always_ff @(posedge clk) begin
        if (lt_q[k]) tmem[ltaddr_q] <= gtdata2_q[GI];
        t_rd <= tmem[lg_q];
      end
    end else begin : g_no_tram
      assign t_rd = '0;
    end

    neuron_processor #(
      .PW(PW), .ACC_W(ACC_W), .THR_W(THR_W),
      .POPCOUNT_LEVELS_PER_STAGE(POPCOUNT_LEVELS_PER_STAGE)
    ) u_np (
      .clk(clk), .rst(rst),
      .in_valid(vpipe[1]), .first_pass(s2_first), .last_pass(s2_last),
      .x(in_rd_data[k / LG]), .w(w_rd), .thr(t_rd),
      .out_valid(np_ov[k]), .out_y(np_y[k]), .out_acc(np_acc[k*ACC_W +: ACC_W])
    );
  end

  // ---------------- result stage: one-hot group pointer, bank, and one output register
  logic                 np_valid;    // all lanes are in lockstep; lane 0 stands for all
  logic [GROUPS-1:0]    oh_o;
  logic                 bank_o, last_o;
  logic [PN-1:0]        mask_o;
  logic                 o_valid, o_last;

  assign np_valid = np_ov[0];
  assign last_o   = oh_o[GROUPS-1];
  assign mask_o   = last_o ? LAST_MASK : '1;

  always_ff @(posedge clk) begin
    if (rst) begin
      oh_o    <= GROUPS'(1);
      bank_o  <= 1'b0;
      o_valid <= 1'b0;
    end else begin
      o_valid <= np_valid;
      if (np_valid) begin
        oh_o <= last_o ? GROUPS'(1) : (oh_o << 1);
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
    logic [GROUPS-1:0]    o_oh;
    always_ff @(posedge clk) begin
      if (np_valid) begin
        o_y    <= np_y & mask_o;   // invalid lanes forced to 0
        o_bank <= bank_o;
        o_oh   <= oh_o;
      end
    end

    assign out_wr_en       = o_valid;
    assign out_wr_bank     = o_bank;
    assign out_wr_oh       = o_oh;
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
    assign out_wr_oh       = '0;
    assign out_wr_data     = '0;
    assign out_commit_en   = 1'b0;
    assign out_commit_bank = 1'b0;
  end
endmodule
