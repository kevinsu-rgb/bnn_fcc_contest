// Independent reference implementation of the BNN_FCC derivation formulas.
//
// This package is written from the plan's formulas only (loops instead of
// $clog2 / log2 tricks) so that it shares no code with rtl/bnn_fcc_pkg.sv.
// Unit environments take expected values from here; the pkg environment
// cross-checks every function below against literals.
//
// Conventions: layer l (0 .. NL-1, NL = topo.size()-1) has N_l = topo[l]
// inputs and M_l = topo[l+1] outputs. pi = PARALLEL_INPUTS, pn[l] =
// PARALLEL_NEURONS[l].
package bnn_ref_pkg;

  localparam int REF_BCAST_FANOUT       = 16;
  localparam int REF_CFG_FLUSH_CYCLES   = 16;
  localparam int REF_BINARIZE_THRESHOLD = 128;
  localparam logic [7:0] REF_MSG_WEIGHTS    = 8'd0;
  localparam logic [7:0] REF_MSG_THRESHOLDS = 8'd1;

  // ---------------------------------------------------------------- scalars
  function automatic int ref_min_int(int a, int b);
    if (a < b) return a;
    return b;
  endfunction

  function automatic int ref_max_int(int a, int b);
    if (a > b) return a;
    return b;
  endfunction

  function automatic int ref_ceil_div(int a, int b);
    int q;
    q = a / b;
    if (q * b < a) q++;
    return q;
  endfunction

  // Number of bits needed to hold values 0..max_value (at least 1).
  function automatic int ref_width_for(int max_value);
    int w;
    longint reach;   // 2**w
    w = 1;
    reach = 2;
    while (reach <= max_value) begin
      reach *= 2;
      w++;
    end
    return w;
  endfunction

  // Number of address bits for `depth` entries (at least 1).
  function automatic int ref_addr_w(int depth);
    int w;
    longint reach;   // 2**w
    w = 1;
    reach = 2;
    while (reach < depth) begin
      reach *= 2;
      w++;
    end
    return w;
  endfunction

  // 1 + ceil(log2(ceil(width/3))).
  function automatic int ref_popcount_latency(int width);
    int groups, levels, reach;
    groups = ref_ceil_div(width, 3);
    levels = 0;
    reach  = 1;
    while (reach < groups) begin
      reach *= 2;
      levels++;
    end
    return 1 + levels;
  endfunction

  function automatic int ref_np_latency(int pw);
    return ref_popcount_latency(pw) + 2;
  endfunction

  // Smallest d >= 1 with 16**d >= n_out.
  function automatic int ref_bcast_depth(int n_out);
    int d;
    longint reach;
    d = 1;
    reach = 16;
    while (reach < n_out) begin
      reach *= 16;
      d++;
    end
    return d;
  endfunction

  // ------------------------------------------------------------- per layer
  function automatic int ref_l_n(int l, int topo[], int pi, int pn[]);
    return topo[l];
  endfunction

  function automatic int ref_l_m(int l, int topo[], int pi, int pn[]);
    return topo[l+1];
  endfunction

  // Input parallelism of layer l (PW_l).
  function automatic int ref_l_pw(int l, int topo[], int pi, int pn[]);
    if (l == 0) return ref_min_int(pi, topo[0]);
    return ref_min_int(pn[l-1], topo[l]);
  endfunction

  // Neuron parallelism of layer l (PN_l).
  function automatic int ref_l_pn(int l, int topo[], int pi, int pn[]);
    return ref_min_int(pn[l], topo[l+1]);
  endfunction

  // Input cycles per neuron group, C_l = ceil(N_l / PW_l).
  function automatic int ref_l_c(int l, int topo[], int pi, int pn[]);
    return ref_ceil_div(ref_l_n(l, topo, pi, pn), ref_l_pw(l, topo, pi, pn));
  endfunction

  // Neuron groups, G_l = ceil(M_l / PN_l).
  function automatic int ref_l_g(int l, int topo[], int pi, int pn[]);
    return ref_ceil_div(ref_l_m(l, topo, pi, pn), ref_l_pn(l, topo, pi, pn));
  endfunction

  // Accumulator width: width_for(N).
  function automatic int ref_l_aw(int l, int topo[], int pi, int pn[]);
    return ref_width_for(ref_l_n(l, topo, pi, pn));
  endfunction

  // Threshold width: width_for(N+1) (= $clog2(N+2)).
  function automatic int ref_l_tw(int l, int topo[], int pi, int pn[]);
    return ref_width_for(ref_l_n(l, topo, pi, pn) + 1);
  endfunction

  // ------------------------------------------------- weight-bus (wb_*) widths
  // addr_w(max_l PN_l)
  function automatic int ref_wb_np_w(int topo[], int pi, int pn[]);
    int nl, mx;
    nl = topo.size() - 1;
    mx = 0;
    for (int l = 0; l < nl; l++) mx = ref_max_int(mx, ref_l_pn(l, topo, pi, pn));
    return ref_addr_w(mx);
  endfunction

  // max_l addr_w(C_l * G_l)
  function automatic int ref_wb_addr_w(int topo[], int pi, int pn[]);
    int nl, mx;
    nl = topo.size() - 1;
    mx = 0;
    for (int l = 0; l < nl; l++)
      mx = ref_max_int(mx, ref_addr_w(ref_l_c(l, topo, pi, pn) * ref_l_g(l, topo, pi, pn)));
    return mx;
  endfunction

  // max_l max(PW_l, TW_l)
  function automatic int ref_wb_data_w(int topo[], int pi, int pn[]);
    int nl, mx;
    nl = topo.size() - 1;
    mx = 0;
    for (int l = 0; l < nl; l++) begin
      mx = ref_max_int(mx, ref_l_pw(l, topo, pi, pn));
      mx = ref_max_int(mx, ref_l_tw(l, topo, pi, pn));
    end
    return mx;
  endfunction

  // addr_w(M_{NL-1}): width of the classification index.
  function automatic int ref_idx_w(int topo[], int pi, int pn[]);
    return ref_addr_w(topo[topo.size()-1]);
  endfunction

endpackage
