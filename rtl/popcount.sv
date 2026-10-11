// Pipelined population count. Latency = popcount_latency(W, LEVELS_PER_STAGE).
// Level 0 counts 6-bit chunks; a binary adder tree sums them. A register is
// placed after every LEVELS_PER_STAGE levels (level 0 counts as level 1) and
// after the final level.
module popcount import bnn_fcc_pkg::*; #(
  parameter int W = 8,
  parameter int LEVELS_PER_STAGE = 3
) (
  input  logic                 clk,
  input  logic [W-1:0]         in_bits,
  output logic [bits(W+1)-1:0] count
);
  localparam int OW = bits(W+1);
  localparam int C  = cdiv(W, 6);
  localparam int T  = 1 + $clog2(C);

  // Zero-extend input to a whole number of 6-bit chunks.
  logic [6*C-1:0] padded;
  assign padded = {{(6*C-W){1'b0}}, in_bits};

  for (genvar l = 0; l < T; l++) begin : g_lv
    localparam int N     = cdiv(C, 1 << l);
    localparam bit IS_REG = (((l + 1) % LEVELS_PER_STAGE) == 0) || (l == T - 1);
    logic [OW-1:0] s [N];       // combinational result of this level
    logic [OW-1:0] q [N];       // after optional register

    for (genvar n = 0; n < N; n++) begin : g_n
      if (l == 0) begin : g_cnt
        assign s[n] = OW'($countones(padded[6*n +: 6]));
      end else begin : g_add
        localparam int PN = cdiv(C, 1 << (l - 1));
        if (2*n + 1 < PN) begin : g_two
          assign s[n] = g_lv[l-1].q[2*n] + g_lv[l-1].q[2*n+1];
        end else begin : g_one
          assign s[n] = g_lv[l-1].q[2*n];
        end
      end

      if (IS_REG) begin : g_r
        always_ff @(posedge clk) q[n] <= s[n];
      end else begin : g_c
        assign q[n] = s[n];
      end
    end
  end

  assign count = g_lv[T-1].q[0];
endmodule
