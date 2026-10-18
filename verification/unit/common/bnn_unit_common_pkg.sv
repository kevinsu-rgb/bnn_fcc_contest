// Common UVM infrastructure shared by every BNN_FCC unit environment.
package bnn_unit_common_pkg;
  import uvm_pkg::*;
  `include "uvm_macros.svh"

  // Standard watchdog fatal ID. sim/run_unit.sh maps a UVM_FATAL with this ID
  // to `RESULT: FAIL reason=timeout`.
  localparam string BNN_WATCHDOG_ID = "WATCHDOG";

  // Fork this (fork ... join_none) from a test's run_phase: aborts the run
  // with a UVM_FATAL if `budget_ns` of simulated time elapse. The thread is
  // killed automatically when the run phase ends normally.
  task automatic bnn_watchdog(string what, time budget_ns);
    #(budget_ns * 1ns);
    `uvm_fatal(BNN_WATCHDOG_ID, $sformatf("%s: watchdog expired after %0d ns", what, budget_ns))
  endtask

  // Base class of every unit scoreboard. Derived classes call check_eq() for
  // each compared value. report_phase prints
  //   SCB: <name> checked=<n> mismatches=<m>
  // and raises a uvm_error if nothing was checked or anything mismatched.
  virtual class bnn_scb_base extends uvm_scoreboard;
    protected int checked;
    protected int mismatches;

    // Only the first few mismatches are logged in detail.
    localparam int MAX_DETAILED_MISMATCHES = 20;

    function new(string name, uvm_component parent);
      super.new(name, parent);
      checked    = 0;
      mismatches = 0;
    endfunction

    function void check_eq(string what, logic [1023:0] got, logic [1023:0] exp);
      checked++;
      if (got !== exp) begin
        mismatches++;
        if (mismatches <= MAX_DETAILED_MISMATCHES)
          `uvm_info("SCB_MISMATCH",
                    $sformatf("%s: got=%0d (0x%0h) exp=%0d (0x%0h)", what, got, got, exp, exp), UVM_NONE)
      end
    endfunction

    function void report_phase(uvm_phase phase);
      super.report_phase(phase);
      `uvm_info("SCB_RPT",
                $sformatf("SCB: %s checked=%0d mismatches=%0d", get_name(), checked, mismatches),
                UVM_NONE)
      if (checked == 0)
        `uvm_error("SCB_RPT", $sformatf("%s checked nothing", get_name()))
      if (mismatches != 0)
        `uvm_error("SCB_RPT", $sformatf("%s had %0d mismatches", get_name(), mismatches))
    endfunction
  endclass

  // Layout model of one layer's private memories (spec §6.5), written from
  // the plan's formulas only:
  //   neuron n -> NP p = n mod PN, group g = n / PN
  //   weight word of (n, chunk c) at address g*C + c, C = ceil(N/PW)
  //   threshold of n at address g
  //   packed weight word bit j = w[c*PW + j] if c*PW + j < N, else 1 (padding)
  //   thresholds clamped to [0, N+1]
  class bnn_layout;
    int n;
    int m;
    int pw;
    int pn;

    function new(int n, int m, int pw, int pn);
      this.n  = n;
      this.m  = m;
      this.pw = pw;
      this.pn = pn;
    endfunction

    function int np_of(int neuron);
      return neuron % pn;
    endfunction

    function int group_of(int neuron);
      return neuron / pn;
    endfunction

    // C = ceil(N / PW)
    function int c_count();
      return (n + pw - 1) / pw;
    endfunction

    // G = ceil(M / PN)
    function int g_count();
      return (m + pn - 1) / pn;
    endfunction

    function int waddr(int neuron, int chunk);
      int g, c;
      g = group_of(neuron);
      c = c_count();
      return g * c + chunk;
    endfunction

    // Bits >= pw of the result are 0.
    function logic [1023:0] pack_word(bit w[], int chunk);
      logic [1023:0] r;
      int idx;
      r = '0;
      for (int j = 0; j < pw; j++) begin
        idx = chunk * pw + j;
        if (idx < n) r[j] = w[idx];
        else r[j] = 1'b1;
      end
      return r;
    endfunction

    function int clamp_thr(longint t, int n);
      if (t < 0) return 0;
      if (t > longint'(n) + 1) return n + 1;
      return int'(t);
    endfunction
  endclass

endpackage
