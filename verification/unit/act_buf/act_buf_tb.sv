// Testbench top of the `act_buf` environment.
module act_buf_tb;
  import uvm_pkg::*;
  import act_buf_pkg::*;

  logic clk = 1'b0;
  always #0.5555 clk = ~clk;

  act_buf_if #(.WIDTH(AB_WIDTH), .AW(AB_AW)) vif (.clk(clk));

  bnn_act_buf #(
      .WIDTH       (`PSET_WIDTH),
      .DEPTH       (`PSET_DEPTH),
      .RAM_STYLE   (`PSET_RAM_STYLE),
      .READ_LATENCY(`PSET_READ_LATENCY)
  ) dut (
      .clk         (clk),
      .rst         (vif.rst),
      .wr_can_claim(vif.wr_can_claim),
      .wr_claim    (vif.wr_claim),
      .wr_en       (vif.wr_en),
      .wr_addr     (vif.wr_addr),
      .wr_data     (vif.wr_data),
      .wr_commit   (vif.wr_commit),
      .rd_full     (vif.rd_full),
      .rd_en       (vif.rd_en),
      .rd_addr     (vif.rd_addr),
      .rd_data     (vif.rd_data),
      .rd_release  (vif.rd_release),
      .idle        (vif.idle)
  );

  initial begin
    // Reset asserted and quiet, defined inputs before the driver takes over.
    vif.rst        = 1'b1;
    vif.wr_claim   = 1'b0;
    vif.wr_en      = 1'b0;
    vif.wr_addr    = '0;
    vif.wr_data    = '0;
    vif.wr_commit  = 1'b0;
    vif.rd_en      = 1'b0;
    vif.rd_addr    = '0;
    vif.rd_release = 1'b0;
    uvm_config_db#(virtual act_buf_if #(AB_WIDTH, AB_AW))::set(null, "*", "vif", vif);
    run_test();
  end
endmodule
