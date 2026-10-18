// Testbench top of the `layer` environment.
//
//   TB producer -> upstream bnn_act_buf (WIDTH=PW, DEPTH=C) -> bnn_layer (DUT)
//     hidden: -> downstream bnn_act_buf (WIDTH=PN, DEPTH=G) -> TB consumer
//             (or, with vif.dn_ideal = 1, an ideal sink: out_can_claim = tb_can_claim)
//     output: res_* captured by the monitor, out_can_claim from the TB credit FIFO
module layer_tb;
  import uvm_pkg::*;
  import layer_pkg::*;

  logic clk = 1'b0;
  always #0.5555 clk = ~clk;

  layer_if #(
      .NPW(L_NPW), .AWB(L_AWB), .DW(L_DW), .PW(L_PW), .PN(L_PN),
      .CAW(L_CAW), .GAW(L_GAW), .AW(L_AW)
  ) vif (.clk(clk));

  bnn_act_buf #(
      .WIDTH       (`PSET_PW),
      .DEPTH       (L_C),
      .RAM_STYLE   (`PSET_RAM_STYLE),
      .READ_LATENCY(`PSET_READ_LATENCY)
  ) u_up (
      .clk         (clk),
      .rst         (vif.rst),
      .wr_can_claim(vif.up_can_claim),
      .wr_claim    (vif.up_claim),
      .wr_en       (vif.up_wr_en),
      .wr_addr     (vif.up_wr_addr),
      .wr_data     (vif.up_wr_data),
      .wr_commit   (vif.up_commit),
      .rd_full     (vif.in_full),
      .rd_en       (vif.in_rd_en),
      .rd_addr     (vif.in_rd_addr),
      .rd_data     (vif.in_rd_data),
      .rd_release  (vif.in_release),
      .idle        (vif.up_idle)
  );

  bnn_layer #(
      .LAYER_ID    (`PSET_LAYER_ID),
      .N           (`PSET_N),
      .M           (`PSET_M),
      .PW          (`PSET_PW),
      .PN          (`PSET_PN),
      .IS_OUTPUT   (`PSET_IS_OUTPUT),
      .RAM_STYLE   (`PSET_RAM_STYLE),
      .READ_LATENCY(`PSET_READ_LATENCY),
      .WB_NP_W     (`PSET_WB_NP_W),
      .WB_ADDR_W   (`PSET_WB_ADDR_W),
      .WB_DATA_W   (`PSET_WB_DATA_W)
  ) dut (
      .clk          (clk),
      .rst          (vif.rst),
      .cfg_mode     (vif.cfg_mode),
      .cfg_wr_valid (vif.cfg_wr_valid),
      .cfg_wr_layer (vif.cfg_wr_layer),
      .cfg_wr_kind  (vif.cfg_wr_kind),
      .cfg_wr_np    (vif.cfg_wr_np),
      .cfg_wr_addr  (vif.cfg_wr_addr),
      .cfg_wr_data  (vif.cfg_wr_data),
      .in_full      (vif.in_full),
      .in_rd_en     (vif.in_rd_en),
      .in_rd_addr   (vif.in_rd_addr),
      .in_rd_data   (vif.in_rd_data),
      .in_release   (vif.in_release),
      .out_can_claim(vif.out_can_claim),
      .out_claim    (vif.out_claim),
      .out_wr_en    (vif.out_wr_en),
      .out_wr_addr  (vif.out_wr_addr),
      .out_wr_data  (vif.out_wr_data),
      .out_commit   (vif.out_commit),
      .res_valid    (vif.res_valid),
      .res_last     (vif.res_last),
      .res_counts   (vif.res_counts),
      .res_mask     (vif.res_mask),
      .idle         (vif.idle)
  );

  if (`PSET_IS_OUTPUT == 0) begin : g_dn
    logic dn_can_claim;
    bnn_act_buf #(
        .WIDTH       (`PSET_PN),
        .DEPTH       (L_G),
        .RAM_STYLE   (`PSET_RAM_STYLE),
        .READ_LATENCY(`PSET_READ_LATENCY)
    ) u_dn (
        .clk         (clk),
        .rst         (vif.rst),
        .wr_can_claim(dn_can_claim),
        .wr_claim    (vif.out_claim  & ~vif.dn_ideal),
        .wr_en       (vif.out_wr_en  & ~vif.dn_ideal),
        .wr_addr     (vif.out_wr_addr),
        .wr_data     (vif.out_wr_data),
        .wr_commit   (vif.out_commit & ~vif.dn_ideal),
        .rd_full     (vif.dn_full),
        .rd_en       (vif.dn_rd_en),
        .rd_addr     (vif.dn_rd_addr),
        .rd_data     (vif.dn_rd_data),
        .rd_release  (vif.dn_release),
        .idle        (vif.dn_idle)
    );
    assign vif.out_can_claim = vif.dn_ideal ? vif.tb_can_claim : dn_can_claim;
  end else begin : g_res
    assign vif.out_can_claim = vif.tb_can_claim;
    assign vif.dn_full       = 1'b0;
    assign vif.dn_rd_data    = '0;
    assign vif.dn_idle       = 1'b1;
  end

  initial begin
    // Reset asserted and every TB-driven input defined before the drivers take over.
    vif.rst          = 1'b1;
    vif.cfg_mode     = 1'b0;
    vif.cfg_wr_valid = 1'b0;
    vif.cfg_wr_layer = '0;
    vif.cfg_wr_kind  = 1'b0;
    vif.cfg_wr_np    = '0;
    vif.cfg_wr_addr  = '0;
    vif.cfg_wr_data  = '0;
    vif.up_claim     = 1'b0;
    vif.up_wr_en     = 1'b0;
    vif.up_wr_addr   = '0;
    vif.up_wr_data   = '0;
    vif.up_commit    = 1'b0;
    vif.dn_rd_en     = 1'b0;
    vif.dn_rd_addr   = '0;
    vif.dn_release   = 1'b0;
    vif.dn_ideal     = 1'b0;
    vif.tb_can_claim = 1'b0;
    uvm_config_db#(layer_vif_t)::set(null, "*", "vif", vif);
    run_test();
  end
endmodule
