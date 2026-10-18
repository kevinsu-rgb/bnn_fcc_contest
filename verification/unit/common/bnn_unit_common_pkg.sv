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

endpackage
