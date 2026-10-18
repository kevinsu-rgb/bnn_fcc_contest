// Testbench top of the `bcast` environment.
module bcast_tb;
  import uvm_pkg::*;
  import bcast_pkg::*;

  logic clk = 1'b0;
  always #0.5555 clk = ~clk;

  bcast_if #(.WIDTH(BC_WIDTH), .N_OUT(BC_N_OUT)) vif (.clk(clk));

  bnn_bcast #(
      .WIDTH(`PSET_WIDTH),
      .N_OUT(`PSET_N_OUT)
  ) dut (
      .clk (clk),
      .din (vif.din),
      .dout(vif.dout)
  );

  initial begin
    // Defined input before the driver takes over (the DUT has no reset).
    vif.din = '0;
    uvm_config_db#(virtual bcast_if #(BC_WIDTH, BC_N_OUT))::set(null, "*", "vif", vif);
    run_test();
  end
endmodule
