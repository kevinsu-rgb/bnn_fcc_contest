// Per-layer derivation functions, `include'd at module scope right after the
// port list of every module that has the parameters TOTAL_LAYERS, TOPOLOGY,
// PARALLEL_INPUTS and PARALLEL_NEURONS. Never listed as a compile unit.
//
// Layer l (0 = first hidden layer, NL = TOTAL_LAYERS-1 layers in total) maps
// N_l = TOPOLOGY[l] inputs to M_l = TOPOLOGY[l+1] outputs.
//
// Note: results of function calls are assigned to temporaries before being
// passed to another function (xsim mis-evaluates nested calls inside
// module-scope functions).

// Not every including module calls every function / uses every argument bit.
/* verilator lint_off UNUSEDSIGNAL */

function automatic int l_n(int l);
  return TOPOLOGY[l];
endfunction

function automatic int l_m(int l);
  return TOPOLOGY[l+1];
endfunction

// Input-side parallel width (inputs consumed per cycle).
function automatic int l_pw(int l);
  if (l == 0) return min_int(PARALLEL_INPUTS, TOPOLOGY[0]);
  else return min_int(PARALLEL_NEURONS[l-1], TOPOLOGY[l]);
endfunction

// Neurons computed in parallel.
function automatic int l_pn(int l);
  return min_int(PARALLEL_NEURONS[l], TOPOLOGY[l+1]);
endfunction

// Cycles per neuron group.
function automatic int l_c(int l);
  int n, pw;
  n  = l_n(l);
  pw = l_pw(l);
  return ceil_div(n, pw);
endfunction

// Neuron groups per image.
function automatic int l_g(int l);
  int m, pn;
  m  = l_m(l);
  pn = l_pn(l);
  return ceil_div(m, pn);
endfunction

// Accumulator width.
function automatic int l_aw(int l);
  int n;
  n = l_n(l);
  return width_for(n);
endfunction

// Threshold width: thresholds in [0, N+1] -> $clog2(N+2) bits.
function automatic int l_tw(int l);
  int n;
  n = l_n(l);
  return width_for(n + 1);
endfunction

// Weight-bank neuron-select width.
function automatic int wb_np_w();
  int mx, t;
  mx = 1;
  for (int l = 0; l < TOTAL_LAYERS - 1; l++) begin
    t  = l_pn(l);
    mx = max_int(mx, t);
  end
  return addr_w(mx);
endfunction

// Weight-bank row address width.
function automatic int wb_addr_w();
  int mx, c, g, t;
  mx = 1;
  for (int l = 0; l < TOTAL_LAYERS - 1; l++) begin
    c  = l_c(l);
    g  = l_g(l);
    t  = addr_w(c * g);
    mx = max_int(mx, t);
  end
  return mx;
endfunction

// Weight-bank write data width (weights PW wide, thresholds TW wide).
function automatic int wb_data_w();
  int mx, pw, tw, t;
  mx = 1;
  for (int l = 0; l < TOTAL_LAYERS - 1; l++) begin
    pw = l_pw(l);
    tw = l_tw(l);
    t  = max_int(pw, tw);
    mx = max_int(mx, t);
  end
  return mx;
endfunction

// Output class index width.
function automatic int idx_w();
  int m;
  m = l_m(TOTAL_LAYERS - 2);
  return addr_w(m);
endfunction

/* verilator lint_on UNUSEDSIGNAL */
