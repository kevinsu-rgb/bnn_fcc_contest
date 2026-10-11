`timescale 1ns / 100ps
// Regression configuration c_100_bus for the provided bnn_fcc_tb (custom-topology mode);
// xsim_tb_fix.svh works around xsim std::randomize / real-queue sum issues without editing the TB.
module cfg_c_100_bus;
  bnn_fcc_tb #(
    .USE_CUSTOM_TOPOLOGY      (1),
    .CUSTOM_LAYERS            (4),
    .CUSTOM_TOPOLOGY          ('{100,37,19,10}),
    .NUM_TEST_IMAGES          (200),
    .TOGGLE_DATA_OUT_READY    (1'b1),
    .CONFIG_VALID_PROBABILITY (0.8),
    .DATA_IN_VALID_PROBABILITY(0.8),
    .TIMEOUT                  (100ms),
    .CONFIG_BUS_WIDTH         (32),
    .INPUT_BUS_WIDTH          (128),
    .PARALLEL_INPUTS          (7),
    .PARALLEL_NEURONS         ('{5,3,4})
  ) tb ();
`include "xsim_tb_fix.svh"
endmodule
