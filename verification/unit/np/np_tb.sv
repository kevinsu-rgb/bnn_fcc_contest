// Testbench top of the `np` environment.
module np_tb;
  import uvm_pkg::*;
  import np_pkg::*;

  logic clk = 1'b0;
  always #0.5555 clk = ~clk;

  np_if #(.PW(NP_PW), .AW(NP_AW), .TW(NP_TW)) vif (.clk(clk));

  bnn_np #(
      .PW(`PSET_PW),
      .AW(`PSET_AW),
      .TW(`PSET_TW)
  ) dut (
      .clk      (clk),
      .in_first (vif.first),
      .in_last  (vif.last),
      .in_x     (vif.x),
      .in_w     (vif.w),
      .in_thr   (vif.thr),
      .out_bit  (vif.out_bit),
      .out_count(vif.out_count)
  );

  initial begin
    // Defined inputs before the driver takes over (the DUT has no reset).
    vif.first = 1'b0;
    vif.last  = 1'b0;
    vif.x     = '0;
    vif.w     = '0;
    vif.thr   = '0;
    uvm_config_db#(virtual np_if #(NP_PW, NP_AW, NP_TW))::set(null, "*", "vif", vif);
    run_test();
  end
endmodule
