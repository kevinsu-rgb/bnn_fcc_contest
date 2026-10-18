// Port-accurate stub of rtl/bnn_layer.sv: all outputs tied to '0, no behavior.
module bnn_layer import bnn_fcc_pkg::*; #(
    parameter int    LAYER_ID     = 0,
    parameter int    N            = 784,
    parameter int    M            = 256,
    parameter int    PW           = 16,
    parameter int    PN           = 128,
    parameter bit    IS_OUTPUT    = 0,
    parameter string RAM_STYLE    = "distributed",
    parameter int    READ_LATENCY = 1,
    parameter int    WB_NP_W      = 7,
    parameter int    WB_ADDR_W    = 7,
    parameter int    WB_DATA_W    = 128,
    localparam int   C            = (N + PW - 1) / PW,
    localparam int   G            = (M + PN - 1) / PN
) (
    input  logic                       clk,
    input  logic                       rst,
    input  logic                       cfg_mode,
    input  logic                       cfg_wr_valid,
    input  logic [                7:0] cfg_wr_layer,
    input  logic                       cfg_wr_kind,
    input  logic [        WB_NP_W-1:0] cfg_wr_np,
    input  logic [      WB_ADDR_W-1:0] cfg_wr_addr,
    input  logic [      WB_DATA_W-1:0] cfg_wr_data,
    input  logic                       in_full,
    output logic                       in_rd_en,
    output logic [      addr_w(C)-1:0] in_rd_addr,
    input  logic [             PW-1:0] in_rd_data,
    output logic                       in_release,
    input  logic                       out_can_claim,
    output logic                       out_claim,
    output logic                       out_wr_en,
    output logic [      addr_w(G)-1:0] out_wr_addr,
    output logic [             PN-1:0] out_wr_data,
    output logic                       out_commit,
    output logic                       res_valid,
    output logic                       res_last,
    output logic [PN*width_for(N)-1:0] res_counts,
    output logic [             PN-1:0] res_mask,
    output logic                       idle
);
  assign in_rd_en    = '0;
  assign in_rd_addr  = '0;
  assign in_release  = '0;
  assign out_claim   = '0;
  assign out_wr_en   = '0;
  assign out_wr_addr = '0;
  assign out_wr_data = '0;
  assign out_commit  = '0;
  assign res_valid   = '0;
  assign res_last    = '0;
  assign res_counts  = '0;
  assign res_mask    = '0;
  assign idle        = '0;
endmodule
