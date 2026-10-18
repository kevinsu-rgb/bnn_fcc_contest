// Port-accurate stub of rtl/bnn_act_buf.sv: all outputs tied to '0, no behavior.
module bnn_act_buf import bnn_fcc_pkg::*; #(
    parameter int    WIDTH        = 16,
    parameter int    DEPTH        = 49,
    parameter string RAM_STYLE    = "distributed",
    parameter int    READ_LATENCY = 1
) (
    input  logic                     clk,
    input  logic                     rst,
    output logic                     wr_can_claim,
    input  logic                     wr_claim,
    input  logic                     wr_en,
    input  logic [addr_w(DEPTH)-1:0] wr_addr,
    input  logic [    WIDTH-1:0]     wr_data,
    input  logic                     wr_commit,
    output logic                     rd_full,
    output logic                     rd_full_next,
    input  logic                     rd_en,
    input  logic [addr_w(DEPTH)-1:0] rd_addr,
    output logic [    WIDTH-1:0]     rd_data,
    input  logic                     rd_release,
    output logic                     idle
);
  assign wr_can_claim = '0;
  assign rd_full      = '0;
  assign rd_full_next = '0;
  assign rd_data      = '0;
  assign idle         = '0;
endmodule
