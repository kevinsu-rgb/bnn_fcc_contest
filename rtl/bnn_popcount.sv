// Pipelined XNOR-popcount: count = #(a[i] == b[i]) over WIDTH bit pairs.
//
// Stage 1 (leaf): groups of 3 bit-pairs -> 2-bit count, registered. Every
// output bit is a function of 6 inputs, i.e. one LUT6.
// Stages 2..: balanced binary tree of registered additions, one level per
// cycle. An odd node at a level is carried forward through a register so all
// paths stay level-aligned. Node sums are sized by the bits they cover.
// Latency = popcount_latency(WIDTH). No reset: the pipeline is pure datapath.
module bnn_popcount import bnn_fcc_pkg::*; #(
    parameter int WIDTH = 16
) (
    input  logic                        clk,
    input  logic [         WIDTH-1:0]   a,
    input  logic [         WIDTH-1:0]   b,
    output logic [width_for(WIDTH)-1:0] count
);

  localparam int CW = width_for(WIDTH);
  localparam int GROUPS = ceil_div(WIDTH, 3);
  localparam int LEVELS = ceil_log2(GROUPS);

  // Node count at tree level l (level 0 = leaves).
  function automatic int nodes_at(int l);
    int n;
    n = GROUPS;
    for (int i = 0; i < l; i++) n = ceil_div(n, 2);
    return n;
  endfunction

  // Number of input bit pairs covered by node i at level l.
  function automatic int bits_at(int l, int i);
    int lo;
    int hi;
    lo = i * 3 * (2 ** l);
    hi = min_int(WIDTH, (i + 1) * 3 * (2 ** l));
    return hi - lo;
  endfunction

  // lv[l][i]: registered output of node i at level l, zero-extended to CW.
  logic [CW-1:0] lv[LEVELS+1][GROUPS];

  // Leaf level.
  for (genvar i = 0; i < GROUPS; i++) begin : g_leaf
    localparam int NB = width_for(bits_at(0, i));
    logic [2:0] eq;
    logic [NB-1:0] cnt;
    for (genvar p = 0; p < 3; p++) begin : g_p
      if (3 * i + p < WIDTH) begin : g_real
        assign eq[p] = ~(a[3*i+p] ^ b[3*i+p]);
      end else begin : g_pad
        // Pad pair contributes 0.
        assign eq[p] = 1'b0;
      end
    end
    always_ff @(posedge clk) cnt <= NB'(eq[0]) + NB'(eq[1]) + NB'(eq[2]);
    assign lv[0][i] = CW'(cnt);
  end

  // Adder tree levels.
  for (genvar l = 0; l < LEVELS; l++) begin : g_lvl
    localparam int NIN = nodes_at(l);
    localparam int NOUT = ceil_div(NIN, 2);
    for (genvar i = 0; i < NOUT; i++) begin : g_n
      localparam int NB = width_for(bits_at(l + 1, i));
      logic [NB-1:0] r;
      if (2 * i + 1 < NIN) begin : g_add
        always_ff @(posedge clk) r <= NB'(lv[l][2*i]) + NB'(lv[l][2*i+1]);
      end else begin : g_carry
        always_ff @(posedge clk) r <= NB'(lv[l][2*i]);
      end
      assign lv[l+1][i] = CW'(r);
    end
    // Unused node slots of the next level are tied off.
    for (genvar i = NOUT; i < GROUPS; i++) begin : g_unused
      assign lv[l+1][i] = '0;
    end
  end

  assign count = lv[LEVELS][0];

endmodule
