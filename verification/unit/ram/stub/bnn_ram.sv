// Port-accurate stub of rtl/bnn_ram.sv: all outputs tied to '0, no behavior.
module bnn_ram import bnn_fcc_pkg::*; #(
    parameter int    WIDTH        = 16,
    parameter int    DEPTH        = 98,
    parameter string RAM_STYLE    = "distributed",
    parameter int    READ_LATENCY = 1,
    parameter string PORTS        = "SP"
) (
    input  logic                     clk,
    input  logic                     we,
    input  logic [addr_w(DEPTH)-1:0] wr_addr,
    input  logic [    WIDTH-1:0]     wr_data,
    input  logic [addr_w(DEPTH)-1:0] rd_addr,
    output logic [    WIDTH-1:0]     rd_data
);
  assign rd_data = '0;
endmodule
