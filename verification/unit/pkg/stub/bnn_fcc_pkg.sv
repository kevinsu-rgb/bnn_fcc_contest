// Port-accurate stub of rtl/bnn_fcc_pkg.sv: every function returns 0 and every
// localparam is 0. Contains no behavior.
package bnn_fcc_pkg;

  function automatic int min_int(int a, int b);
    return 0;
  endfunction

  function automatic int max_int(int a, int b);
    return 0;
  endfunction

  function automatic int ceil_div(int a, int b);
    return 0;
  endfunction

  function automatic int width_for(int max_value);
    return 0;
  endfunction

  function automatic int addr_w(int depth);
    return 0;
  endfunction

  function automatic int popcount_latency(int width);
    return 0;
  endfunction

  function automatic int np_latency(int pw);
    return 0;
  endfunction

  function automatic int bcast_depth(int n_out);
    return 0;
  endfunction

  localparam int BCAST_FANOUT       = 0;
  localparam int CFG_FLUSH_CYCLES   = 0;
  localparam int BINARIZE_THRESHOLD = 0;
  localparam logic [7:0] MSG_WEIGHTS    = 8'd0;
  localparam logic [7:0] MSG_THRESHOLDS = 8'd0;

endpackage
