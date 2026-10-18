// Signal bundle of the `layer` environment (clock supplied by the testbench top).
//
// Holds every bnn_layer port, the write side of the upstream bnn_act_buf (driven
// by the TB producer), the read side of the downstream bnn_act_buf (driven by the
// TB consumer; hidden layers only) and the TB-modelled out_can_claim
// (output-layer credit FIFO, or the ideal sink used by layer_back_to_back_test).
interface layer_if #(
    parameter int NPW = 7,    // WB_NP_W
    parameter int AWB = 7,    // WB_ADDR_W
    parameter int DW  = 128,  // WB_DATA_W
    parameter int PW  = 16,
    parameter int PN  = 128,
    parameter int CAW = 6,    // addr_w(C)
    parameter int GAW = 1,    // addr_w(G)
    parameter int AW  = 10    // width_for(N)
) (
    input logic clk
);
  logic rst;

  // ---- write bus (TB -> DUT)
  logic           cfg_mode;
  logic           cfg_wr_valid;
  logic [    7:0] cfg_wr_layer;
  logic           cfg_wr_kind;
  logic [NPW-1:0] cfg_wr_np;
  logic [AWB-1:0] cfg_wr_addr;
  logic [ DW-1:0] cfg_wr_data;

  // ---- DUT input side (upstream act_buf read side <-> DUT)
  logic           in_full;
  logic           in_rd_en;
  logic [CAW-1:0] in_rd_addr;
  logic [ PW-1:0] in_rd_data;
  logic           in_release;

  // ---- DUT output side
  logic             out_can_claim;
  logic             out_claim;
  logic             out_wr_en;
  logic [  GAW-1:0] out_wr_addr;
  logic [   PN-1:0] out_wr_data;
  logic             out_commit;
  logic             res_valid;
  logic             res_last;
  logic [PN*AW-1:0] res_counts;
  logic [   PN-1:0] res_mask;
  logic             idle;

  // ---- upstream act_buf write side (TB producer)
  logic           up_can_claim;
  logic           up_claim;
  logic           up_wr_en;
  logic [CAW-1:0] up_wr_addr;
  logic [ PW-1:0] up_wr_data;
  logic           up_commit;
  logic           up_idle;

  // ---- downstream act_buf read side (TB consumer, hidden layers)
  logic           dn_full;
  logic           dn_rd_en;
  logic [GAW-1:0] dn_rd_addr;
  logic [ PN-1:0] dn_rd_data;
  logic           dn_release;
  logic           dn_idle;

  // ---- TB-modelled out_can_claim
  logic dn_ideal;      // hidden: 1 = downstream act_buf bypassed, out_can_claim = tb_can_claim
  logic tb_can_claim;  // output layer: credit FIFO; hidden + dn_ideal: ideal sink
endinterface
