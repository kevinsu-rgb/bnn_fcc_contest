// Two-bank, width-converting buffer. Registers (not RAM); data is not reset.
// Write: per-bit enable from a shared chunk-enable vector (bit j <- wr_data[j % WR_W], chunk j / WR_W).
// Read: rd_data is registered (1-cycle latency) and independent of the full flags, so a read
// issued in the same cycle as rls_en still returns that bank's data. Bits >= NBITS read as 0.
// If commit_en and rls_en target the same bank in the same cycle, commit wins.
module pingpong_buf import bnn_fcc_pkg::*; #(
  parameter int NBITS = 8,
  parameter int WR_W  = 8,
  parameter int RD_W  = 8
) (
  input  logic                         clk,
  input  logic                         rst,
  input  logic                         wr_en,
  input  logic                         wr_bank,
  input  logic [bits(cdiv(NBITS,WR_W))-1:0] wr_idx,
  input  logic [WR_W-1:0]              wr_data,
  input  logic                         commit_en,
  input  logic                         commit_bank,
  input  logic                         rd_bank,
  input  logic [bits(cdiv(NBITS,RD_W))-1:0] rd_idx,
  output logic [RD_W-1:0]              rd_data,
  input  logic                         rls_en,
  input  logic                         rls_bank,
  output logic [1:0]                   full
);
  localparam int NWR    = cdiv(NBITS, WR_W);
  localparam int NRD    = cdiv(NBITS, RD_W);
  localparam int RIDX_W = bits(NRD);

  logic [NBITS-1:0] mem [2];
  logic [NWR-1:0]   we  [2];

  for (genvar b = 0; b < 2; b++) begin : g_bank
    for (genvar c = 0; c < NWR; c++) begin : g_we
      assign we[b][c] = wr_en && (wr_bank == 1'(b)) && (int'(wr_idx) == c);
    end
    for (genvar j = 0; j < NBITS; j++) begin : g_bit
      always_ff @(posedge clk)
        if (we[b][j / WR_W]) mem[b][j] <= wr_data[j % WR_W];
    end
  end

  // Word view of each bank: rows >= NRD and bits >= NBITS are constant 0.
  logic [RD_W-1:0] words [2][2**RIDX_W];
  always_comb begin
    for (int b = 0; b < 2; b++)
      for (int r = 0; r < 2**RIDX_W; r++)
        for (int k = 0; k < RD_W; k++)
          words[b][r][k] = (r * RD_W + k < NBITS) ? mem[b][r * RD_W + k] : 1'b0;
  end

  always_ff @(posedge clk) rd_data <= words[rd_bank][rd_idx];

  always_ff @(posedge clk) begin
    if (rst) full <= '0;
    else begin
      if (rls_en)    full[rls_bank]    <= 1'b0;
      if (commit_en) full[commit_bank] <= 1'b1;
    end
  end
endmodule
