// AXI4-Stream result output: DEPTH-entry FIFO feeding a registered output stage.
// push must not occur when count >= DEPTH (credit-protected by the top).
// count = entries held (FIFO + output register).
module axis_out import bnn_fcc_pkg::*; #(
  parameter int DATA_W = 4,
  parameter int BUS_W = 8,
  parameter int DEPTH = 4
) (
  input  logic                      clk,
  input  logic                      rst,
  input  logic                      push,
  input  logic [DATA_W-1:0]         push_data,
  output logic [bits(DEPTH+1)-1:0]  count,
  output logic                      m_valid,
  input  logic                      m_ready,
  output logic [BUS_W-1:0]          m_data,
  output logic [BUS_W/8-1:0]        m_keep,
  output logic                      m_last
);
  localparam int PW = bits(DEPTH);
  localparam int KEEP_N = cdiv(DATA_W, 8);

  logic [DATA_W-1:0] mem [DEPTH];
  logic [PW-1:0]     wr_q, rd_q;
  logic [bits(DEPTH+1)-1:0] fcnt;

  wire load = (!m_valid || m_ready) && (fcnt != '0);

  always_ff @(posedge clk) begin
    if (push) mem[wr_q] <= push_data;
    if (load) begin
      m_data <= BUS_W'(mem[rd_q]);
      m_keep <= (BUS_W/8)'((1 << KEEP_N) - 1);
      m_last <= 1'b1;
    end
  end

  always_ff @(posedge clk) begin
    if (rst) begin
      wr_q    <= '0;
      rd_q    <= '0;
      fcnt    <= '0;
      m_valid <= 1'b0;
    end else begin
      if (push) wr_q <= (wr_q == PW'(DEPTH - 1)) ? '0 : wr_q + 1'b1;
      if (load) rd_q <= (rd_q == PW'(DEPTH - 1)) ? '0 : rd_q + 1'b1;
      if (push && !load)      fcnt <= fcnt + 1'b1;
      else if (!push && load) fcnt <= fcnt - 1'b1;
      if (load)         m_valid <= 1'b1;
      else if (m_ready) m_valid <= 1'b0;
    end
  end

  assign count = fcnt + (bits(DEPTH+1))'(m_valid);
endmodule
