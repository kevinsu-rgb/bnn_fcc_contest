// Two-bank, width-converting buffer. Registers (not RAM); data is not reset.
// Write: one register stage. Every cycle the buffer registers the request ({per-bank enable,
// one-hot chunk select wr_oh, data (one copy per bank)}) and commit_en; chunk c of bank b is written
// (bit j <- data[j % WR_W], chunk j / WR_W) in the next cycle. The full output includes the
// registered commit, so full rises in the cycle after commit_en (as for an unregistered write port),
// and a read started in that cycle still sees the last write.
// Read (NRD = cdiv(NBITS, RD_W) words): rd_adv in cycle t reads bank rd_bank; the word appears on every
// rd_data copy in cycle t+2 (defined only then; the copies reload every cycle). For NRD > 1 the read
// bank rotates right by RD_W on each advance (one cycle after rd_adv), so the next word is always at
// bits [0, RD_W). The consumer issues GROUPS x NRD advances per image on a full bank (GROUPS full
// turns) and releases it with the last one; banks rotate only while full and producers only write
// banks that are not full, so writes always land at aligned positions. Bits >= NBITS read as 0.
// full: release clears, the registered commit sets; a release in the cycle after commit_en (same bank)
// wins, since that commit is logically older; commit_en and rls_en on one bank in one cycle end full.
module pingpong_buf import bnn_fcc_pkg::*; #(
  parameter int NBITS     = 8,
  parameter int WR_W      = 8,
  parameter int RD_W      = 8,
  parameter int RD_COPIES = 1
) (
  input  logic                           clk,
  input  logic                           rst,
  input  logic                           wr_en,
  input  logic                           wr_bank,
  input  logic [cdiv(NBITS,WR_W)-1:0]    wr_oh,
  input  logic [WR_W-1:0]                wr_data,
  input  logic                           commit_en,
  input  logic                           commit_bank,
  /* verilator lint_off UNUSEDSIGNAL */   // rd_adv only drives the rotation (NRD > 1)
  input  logic                           rd_adv,
  /* verilator lint_on UNUSEDSIGNAL */
  input  logic                           rd_bank,
  output logic [RD_COPIES-1:0][RD_W-1:0] rd_data,
  input  logic                           rls_en,
  input  logic                           rls_bank,
  output logic [1:0]                     full
);
  localparam int NWR = cdiv(NBITS, WR_W);
  localparam int NRD = cdiv(NBITS, RD_W);
  localparam int LEN = (NRD > 1) ? NRD * RD_W : NBITS;   // stored bits per bank (rotating: whole words)

  // ---------------- write register stage
  logic [1:0]                           en_q;
  logic [NWR-1:0]                       oh_q;
  (* keep = "true" *) logic [WR_W-1:0]  dat_q [2];
  logic                                 cm_q, cm_bank_q;

  always_ff @(posedge clk) begin
    if (rst) begin
      en_q <= '0;
      cm_q <= 1'b0;
    end else begin
      en_q <= {wr_en && wr_bank, wr_en && !wr_bank};
      cm_q <= commit_en;
    end
    oh_q      <= wr_oh;
    cm_bank_q <= commit_bank;
    dat_q[0]  <= wr_data;
    dat_q[1]  <= wr_data;
  end

  // ---------------- banks
  logic [LEN-1:0] mem [2];

  if (NRD > 1) begin : g_rot
    // registered advance, one copy per bank row
    (* keep = "true" *) logic adv_q [2][NRD];
    always_ff @(posedge clk) begin
      for (int b = 0; b < 2; b++)
        for (int r = 0; r < NRD; r++)
          adv_q[b][r] <= !rst && rd_adv && (rd_bank == 1'(b));
    end

    for (genvar b = 0; b < 2; b++) begin : g_bank
      for (genvar j = 0; j < LEN; j++) begin : g_bit
        if (j < NBITS) begin : g_data
          always_ff @(posedge clk)
            if (en_q[b] && oh_q[j / WR_W]) mem[b][j] <= dat_q[b][j % WR_W];
            else if (adv_q[b][j / RD_W])   mem[b][j] <= mem[b][(j + RD_W) % LEN];
        end else begin : g_pad   // pad bits: 0 from reset, never written, only rotated
          always_ff @(posedge clk)
            if (rst)                     mem[b][j] <= 1'b0;
            else if (adv_q[b][j / RD_W]) mem[b][j] <= mem[b][(j + RD_W) % LEN];
        end
      end
    end
  end else begin : g_flat
    for (genvar b = 0; b < 2; b++) begin : g_bank
      for (genvar j = 0; j < LEN; j++) begin : g_bit
        always_ff @(posedge clk)
          if (en_q[b] && oh_q[j / WR_W]) mem[b][j] <= dat_q[b][j % WR_W];
      end
    end
  end

  // ---------------- read: bank select copies, word at bits [0, RD_W)
  (* keep = "true" *) logic bsel_q [RD_COPIES];
  logic [RD_W-1:0] low [2];

  always_comb begin
    for (int b = 0; b < 2; b++) begin
      low[b] = '0;
      for (int k = 0; k < RD_W; k++) if (k < NBITS) low[b][k] = mem[b][k];
    end
  end

  always_ff @(posedge clk) begin
    for (int c = 0; c < RD_COPIES; c++) begin
      bsel_q[c]  <= rd_bank;
      rd_data[c] <= bsel_q[c] ? low[1] : low[0];
    end
  end

  // ---------------- full flags
  logic [1:0] full_q;

  always_ff @(posedge clk) begin
    if (rst) full_q <= '0;
    else begin
      if (cm_q)   full_q[cm_bank_q] <= 1'b1;
      if (rls_en) full_q[rls_bank]  <= 1'b0;
    end
  end

  assign full = full_q | {cm_q && cm_bank_q, cm_q && !cm_bank_q};
endmodule
