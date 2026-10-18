// Port bundle of bnn_np (clock supplied by the testbench top).
interface np_if #(
    parameter int PW = 16,
    parameter int AW = 10,
    parameter int TW = 10
) (
    input logic clk
);
  logic          first;
  logic          last;
  logic [PW-1:0] x;
  logic [PW-1:0] w;
  logic [TW-1:0] thr;
  logic          out_bit;
  logic [AW-1:0] out_count;
endinterface
