// Port-accurate stub of rtl/bnn_np.sv: all outputs tied to '0, no behavior.
module bnn_np import bnn_fcc_pkg::*; #(
    int PW = 16,
    int AW = 10,
    int TW = 10
) (
    input  logic          clk,
    input  logic          in_first,
    input  logic          in_last,
    input  logic [PW-1:0] in_x,
    input  logic [PW-1:0] in_w,
    input  logic [TW-1:0] in_thr,
    output logic          out_bit,
    output logic [AW-1:0] out_count
);
  assign out_bit   = '0;
  assign out_count = '0;
endmodule
