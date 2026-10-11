`timescale 1ns / 100ps
// Regression configuration c_5layer for the provided bnn_fcc_tb (custom-topology mode);
// xsim_tb_fix.svh works around xsim std::randomize / real-queue sum issues without editing the TB.
module cfg_c_5layer;
  bnn_fcc_tb #(
    .USE_CUSTOM_TOPOLOGY      (1),
    .CUSTOM_LAYERS            (5),
    .CUSTOM_TOPOLOGY          ('{30,17,9,12,6}),
    .NUM_TEST_IMAGES          (200),
    .TOGGLE_DATA_OUT_READY    (1'b1),
    .CONFIG_VALID_PROBABILITY (0.8),
    .DATA_IN_VALID_PROBABILITY(0.8),
    .TIMEOUT                  (100ms),
    .PARALLEL_INPUTS          (5),
    .PARALLEL_NEURONS         ('{4,9,5,6})
  ) tb ();
`include "xsim_tb_fix.svh"
endmodule
