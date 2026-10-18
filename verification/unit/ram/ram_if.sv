// Port bundle of bnn_ram (clock supplied by the testbench top).
interface ram_if #(
    parameter int WIDTH = 16,
    parameter int AW    = 7
) (
    input logic clk
);
  logic             we;
  logic [   AW-1:0] wr_addr;
  logic [WIDTH-1:0] wr_data;
  logic [   AW-1:0] rd_addr;
  logic [WIDTH-1:0] rd_data;
endinterface
