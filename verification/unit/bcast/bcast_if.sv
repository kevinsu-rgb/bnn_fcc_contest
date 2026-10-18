// Port bundle of bnn_bcast (clock supplied by the testbench top).
interface bcast_if #(
    parameter int WIDTH = 8,
    parameter int N_OUT = 128
) (
    input logic clk
);
  logic [WIDTH-1:0] din;
  logic [WIDTH-1:0] dout[N_OUT];
endinterface
