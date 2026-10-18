// Testbench top of the `popcount` environment.
module popcount_tb;
  import uvm_pkg::*;
  import popcount_pkg::*;

  logic clk = 1'b0;
  always #0.5555 clk = ~clk;

  popcount_if #(.WIDTH(PC_WIDTH), .CW(PC_CW)) vif (.clk(clk));

  bnn_popcount #(
      .WIDTH(`PSET_WIDTH)
  ) dut (
      .clk  (clk),
      .a    (vif.a),
      .b    (vif.b),
      .count(vif.count)
  );

  initial begin
    // Defined inputs before the driver takes over (the DUT has no reset).
    vif.a = '0;
    vif.b = '0;
    uvm_config_db#(virtual popcount_if #(PC_WIDTH, PC_CW))::set(null, "*", "vif", vif);
    run_test();
  end
endmodule
