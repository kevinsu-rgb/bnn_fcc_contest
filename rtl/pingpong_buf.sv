// Two-bank, width-converting buffer. Registers (not RAM); data is not reset.
// Write: per-bit enable decoded from wr_idx (bit j <- wr_data[j % WR_W], chunk j / WR_W).
// Read: rd_data is registered (1-cycle latency) and independent of the full flags, so a read
// issued in the same cycle as rls_en still returns that bank's data.
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
  localparam int NRD = cdiv(NBITS, RD_W);

  logic [NBITS-1:0] mem [2];

  for (genvar b = 0; b < 2; b++) begin : g_bank
    for (genvar j = 0; j < NBITS; j++) begin : g_bit
      always_ff @(posedge clk)
        if (wr_en && wr_bank == 1'(b) && int'(wr_idx) == j / WR_W)
          mem[b][j] <= wr_data[j % WR_W];
    end
  end

  logic [RD_W-1:0] rd_nxt;
  always_comb begin
    rd_nxt = '0;
    for (int i = 0; i < NRD; i++)
      if (int'(rd_idx) == i)
        for (int k = 0; k < RD_W; k++)
          if (i * RD_W + k < NBITS)
            rd_nxt[k] = mem[rd_bank][i * RD_W + k];
  end

  always_ff @(posedge clk) rd_data <= rd_nxt;

  always_ff @(posedge clk) begin
    if (rst) full <= '0;
    else begin
      if (rls_en)    full[rls_bank]    <= 1'b0;
      if (commit_en) full[commit_bank] <= 1'b1;
    end
  end
endmodule
