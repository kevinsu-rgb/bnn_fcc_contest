// Port-accurate stub of rtl/bnn_popcount.sv: all outputs tied to '0, no behavior.
module bnn_popcount import bnn_fcc_pkg::*; #(
    parameter int WIDTH = 16
) (
    input  logic                       clk,
    input  logic [        WIDTH-1:0]   a,
    input  logic [        WIDTH-1:0]   b,
    output logic [width_for(WIDTH)-1:0] count
);
  assign count = '0;
endmodule
