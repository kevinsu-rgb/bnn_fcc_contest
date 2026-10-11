`timescale 1ns / 100ps
// Regression configuration c_tiny for the provided bnn_fcc_tb (custom-topology mode);
// xsim_tb_fix.svh works around xsim std::randomize / real-queue sum issues without editing the TB.
module cfg_c_tiny;
  bnn_fcc_tb #(
    .USE_CUSTOM_TOPOLOGY      (1),
    .CUSTOM_LAYERS            (4),
    .CUSTOM_TOPOLOGY          ('{5,1,3,2}),
    .NUM_TEST_IMAGES          (200),
    .TOGGLE_DATA_OUT_READY    (1'b1),
    .CONFIG_VALID_PROBABILITY (0.8),
    .DATA_IN_VALID_PROBABILITY(0.8),
    .TIMEOUT                  (100ms),
    .PARALLEL_INPUTS          (1),
    .PARALLEL_NEURONS         ('{1,1,1})
  ) tb ();
`include "xsim_tb_fix.svh"
endmodule
