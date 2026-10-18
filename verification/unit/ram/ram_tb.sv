// Testbench top of the `ram` environment.
module ram_tb;
  import uvm_pkg::*;
  import ram_pkg::*;

  logic clk = 1'b0;
  always #0.5555 clk = ~clk;

  ram_if #(.WIDTH(RAM_WIDTH), .AW(RAM_AW)) vif (.clk(clk));

  bnn_ram #(
      .WIDTH       (`PSET_WIDTH),
      .DEPTH       (`PSET_DEPTH),
      .RAM_STYLE   (`PSET_RAM_STYLE),
      .READ_LATENCY(`PSET_READ_LATENCY),
      .PORTS       (`PSET_PORTS)
  ) dut (
      .clk    (clk),
      .we     (vif.we),
      .wr_addr(vif.wr_addr),
      .wr_data(vif.wr_data),
      .rd_addr(vif.rd_addr),
      .rd_data(vif.rd_data)
  );

  initial begin
    // Quiet, defined inputs before the driver takes over.
    vif.we      = 1'b0;
    vif.wr_addr = '0;
    vif.wr_data = '0;
    vif.rd_addr = '0;
    uvm_config_db#(virtual ram_if #(RAM_WIDTH, RAM_AW))::set(null, "*", "vif", vif);
    run_test();
  end
endmodule
