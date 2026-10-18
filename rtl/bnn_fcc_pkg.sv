// BNN FCC shared package: constant helper functions and global localparams.
// All functions are synthesizable constant functions (integer loops only).
package bnn_fcc_pkg;

  // Package constants are consumed by other modules; not all are used by every top.
  /* verilator lint_off UNUSEDPARAM */
  localparam int BCAST_FANOUT = 16;
  localparam int CFG_FLUSH_CYCLES = 16;
  localparam int BINARIZE_THRESHOLD = 128;

  localparam logic [7:0] MSG_WEIGHTS = 8'd0;
  localparam logic [7:0] MSG_THRESHOLDS = 8'd1;
  /* verilator lint_on UNUSEDPARAM */

  function automatic int min_int(int a, int b);
    return (a < b) ? a : b;
  endfunction

  function automatic int max_int(int a, int b);
    return (a > b) ? a : b;
  endfunction

  function automatic int ceil_div(int a, int b);
    return (a + b - 1) / b;
  endfunction

  // Smallest k >= 0 with 2**k >= x (x >= 1).
  function automatic int ceil_log2(int x);
    int k;
    int p;
    k = 0;
    p = 1;
    while (p < x) begin
      p = p * 2;
      k = k + 1;
    end
    return k;
  endfunction

  // Bits needed to represent 0..max_value (at least 1).
  function automatic int width_for(int max_value);
    return max_int(1, ceil_log2(max_value + 1));
  endfunction

  // Address bits for a memory of the given depth (at least 1).
  function automatic int addr_w(int depth);
    return max_int(1, ceil_log2(depth));
  endfunction

  // 1 register stage of 3:1 LUT compression, then a 2:1 adder tree of
  // ceil(log2(ceil(width/3))) levels (no extra level when one group remains).
  function automatic int popcount_latency(int width);
    return 1 + ceil_log2(ceil_div(width, 3));
  endfunction

  function automatic int np_latency(int pw);
    return popcount_latency(pw) + 2;
  endfunction

  // Smallest d >= 1 with BCAST_FANOUT**d >= n_out.
  function automatic int bcast_depth(int n_out);
    int d;
    int cap;
    d = 1;
    cap = BCAST_FANOUT;
    while (cap < n_out) begin
      cap = cap * BCAST_FANOUT;
      d = d + 1;
    end
    return d;
  endfunction

endpackage
