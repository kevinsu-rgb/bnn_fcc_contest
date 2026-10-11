`timescale 1ns / 100ps
// Regression configuration c_13753_bus8 for the provided bnn_fcc_tb (custom-topology mode);
// xsim_tb_fix.svh works around xsim std::randomize / real-queue sum issues without editing the TB.
module cfg_c_13753_bus8;
  bnn_fcc_tb #(
    .USE_CUSTOM_TOPOLOGY      (1),
    .CUSTOM_LAYERS            (4),
    .CUSTOM_TOPOLOGY          ('{13,7,5,3}),
    .NUM_TEST_IMAGES          (200),
    .TOGGLE_DATA_OUT_READY    (1'b1),
    .CONFIG_VALID_PROBABILITY (0.8),
    .DATA_IN_VALID_PROBABILITY(0.8),
    .TIMEOUT                  (100ms),
    .CONFIG_BUS_WIDTH         (8),
    .INPUT_BUS_WIDTH          (8),
    .PARALLEL_INPUTS          (3),
    .PARALLEL_NEURONS         ('{2,4,3})
  ) tb ();
`include "xsim_tb_fix.svh"
endmodule
