// Shared constants and helper functions for the BNN FCC design.
package bnn_fcc_pkg;

  function automatic int cdiv(int a, int b);
    return (a + b - 1) / b;
  endfunction

  function automatic int imin(int a, int b);
    return (a < b) ? a : b;
  endfunction

  function automatic int imax(int a, int b);
    return (a > b) ? a : b;
  endfunction

  // Bits needed to index/represent x distinct values (at least 1).
  function automatic int bits(int x);
    return imax(1, $clog2(x));
  endfunction

  // Pipeline latency of popcount: level 0 counts 6-bit chunks, then a binary
  // tree; one register per LPS levels, output always registered.
  function automatic int popcount_latency(int w, int lps);
    int t;
    t = 1 + $clog2(cdiv(w, 6));
    return cdiv(t, lps);
  endfunction

  // Neuron-processor latency: XNOR register, popcount, accumulate stage, compare stage.
  function automatic int np_latency(int w, int lps);
    return popcount_latency(w, lps) + 3;
  endfunction

endpackage
