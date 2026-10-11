// Streaming argmax over the output layer's result groups.
// Per group a pipelined compare tree (a register after every LEVELS_PER_STAGE
// levels and after the final level; one register if PN == 1) picks the maximum
// valid lane, lowest lane on ties. A running best across groups is replaced only
// by a strictly greater group maximum, so the lowest neuron index always wins.
// out_valid pulses for one cycle per image with the winning index.
/* verilator lint_off UNUSEDPARAM */
module argmax import bnn_fcc_pkg::*; #(
  parameter int PN = 1,
  parameter int ACC_W = 4,
  parameter int N = 1,   // informational (index range is set by IDX_W)
  parameter int IDX_W = 4,
  parameter int LEVELS_PER_STAGE = 2
) (
  input  logic                clk,
  input  logic                rst,
  input  logic                in_valid,
  input  logic [PN*ACC_W-1:0] in_counts,
  input  logic [PN-1:0]       in_mask,
  input  logic                in_last,
  output logic                out_valid,
  output logic [IDX_W-1:0]    out_idx,
  output logic                busy
);
  localparam int LW = bits(PN);                       // lane index width
  localparam int T  = $clog2(PN);                     // compare levels
  localparam int L  = (T == 0) ? 1 : cdiv(T, LEVELS_PER_STAGE);  // register stages

  // ---- group base index (input side) ----
  logic [IDX_W-1:0] base_q;
  always_ff @(posedge clk) begin
    if (rst) base_q <= '0;
    else if (in_valid) base_q <= in_last ? '0 : base_q + IDX_W'(PN);
  end

  // ---- compare tree ----
  // Level 0: leaves. Level l node n merges nodes 2n and 2n+1 of level l-1.
  for (genvar l = 0; l <= T; l++) begin : g_lv
    localparam int NN = cdiv(PN, 1 << l);
    localparam bit IS_REG = (T == 0) ? 1'b1 : ((l > 0) && ((l % LEVELS_PER_STAGE) == 0 || l == T));
    /* verilator lint_off UNUSEDSIGNAL */
    logic [NN-1:0]         sv;  logic [NN-1:0]         qv;   // lane valid
    logic [ACC_W-1:0]      sc [NN]; logic [ACC_W-1:0]  qc [NN];
    logic [LW-1:0]         sl [NN]; logic [LW-1:0]     ql [NN];
    /* verilator lint_on UNUSEDSIGNAL */

    for (genvar n = 0; n < NN; n++) begin : g_n
      if (l == 0) begin : g_leaf
        assign sv[n] = in_mask[n];
        assign sc[n] = in_counts[n*ACC_W +: ACC_W];
        assign sl[n] = LW'(n);
      end else begin : g_mrg
        localparam int PNN = cdiv(PN, 1 << (l - 1));
        if (2*n + 1 < PNN) begin : g_two
          wire take_r = g_lv[l-1].qv[2*n+1] &&
                        (!g_lv[l-1].qv[2*n] || (g_lv[l-1].qc[2*n+1] > g_lv[l-1].qc[2*n]));
          assign sv[n] = g_lv[l-1].qv[2*n] | g_lv[l-1].qv[2*n+1];
          assign sc[n] = take_r ? g_lv[l-1].qc[2*n+1] : g_lv[l-1].qc[2*n];
          assign sl[n] = take_r ? g_lv[l-1].ql[2*n+1] : g_lv[l-1].ql[2*n];
        end else begin : g_one
          assign sv[n] = g_lv[l-1].qv[2*n];
          assign sc[n] = g_lv[l-1].qc[2*n];
          assign sl[n] = g_lv[l-1].ql[2*n];
        end
      end

      if (IS_REG) begin : g_r
        always_ff @(posedge clk) begin
          qc[n] <= sc[n];
          ql[n] <= sl[n];
        end
      end else begin : g_c
        assign qc[n] = sc[n];
        assign ql[n] = sl[n];
      end
    end
    if (IS_REG) begin : g_rv
      always_ff @(posedge clk) qv <= sv;   // lane valid bits are data only
    end else begin : g_cv
      assign qv = sv;
    end
  end

  // ---- sideband delay line (control; reset) ----
  logic [L-1:0]      pv;                   // group valid per stage
  logic [L-1:0]      pl;                   // last flag
  logic [IDX_W-1:0]  pb [L];               // group base
  if (L == 1) begin : g_sb1
    always_ff @(posedge clk) begin
      if (rst) pv <= 1'b0;
      else     pv <= in_valid;
      pl[0] <= in_last;
      pb[0] <= base_q;
    end
  end else begin : g_sbn
    always_ff @(posedge clk) begin
      if (rst) pv <= '0;
      else     pv <= {pv[L-2:0], in_valid};
      pl <= {pl[L-2:0], in_last};
      pb[0] <= base_q;
      for (int i = 1; i < L; i++) pb[i] <= pb[i-1];
    end
  end

  wire                g_valid = pv[L-1];
  wire                g_last  = pl[L-1];
  wire [ACC_W-1:0]    g_cnt   = g_lv[T].qc[0];
  wire [LW-1:0]       g_lane  = g_lv[T].ql[0];
  wire [IDX_W-1:0]    g_idx   = pb[L-1] + IDX_W'(g_lane);

  // ---- running best across groups ----
  logic              have_q;
  logic [ACC_W-1:0]  best_c;
  logic [IDX_W-1:0]  best_i;
  wire               upd = g_valid && (!have_q || g_cnt > best_c);
  wire [IDX_W-1:0]   fin_i = upd ? g_idx : best_i;

  always_ff @(posedge clk) begin
    if (rst) begin
      have_q    <= 1'b0;
      out_valid <= 1'b0;
    end else begin
      out_valid <= g_valid && g_last;
      if (g_valid) have_q <= !g_last;
    end
  end
  always_ff @(posedge clk) begin
    if (g_valid) begin
      if (upd) begin
        best_c <= g_cnt;
        best_i <= g_idx;
      end
      if (g_last) out_idx <= fin_i;
    end
  end

  assign busy = (|pv) | have_q | out_valid;
endmodule
/* verilator lint_on UNUSEDPARAM */
