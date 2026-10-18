// Port bundle of bnn_popcount (clock supplied by the testbench top).
interface popcount_if #(
    parameter int WIDTH = 16,
    parameter int CW    = 5
) (
    input logic clk
);
  logic [WIDTH-1:0] a;
  logic [WIDTH-1:0] b;
  logic [   CW-1:0] count;
endinterface
