// Port-accurate stub of rtl/bnn_bcast.sv: all outputs tied to '0, no behavior.
module bnn_bcast import bnn_fcc_pkg::*; #(
    parameter int WIDTH = 8,
    parameter int N_OUT = 128
) (
    input  logic             clk,
    input  logic [WIDTH-1:0] din,
    output logic [WIDTH-1:0] dout[N_OUT]
);
  for (genvar i = 0; i < N_OUT; i++) begin : g_o
    assign dout[i] = '0;
  end
endmodule
