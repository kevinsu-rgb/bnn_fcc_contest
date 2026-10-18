// Testbench top of the `pkg` environment.
//
// derive_probe instantiates bnn_fcc_derive.svh the way every RTL module does
// (right after the port list) and publishes the derived values through
// pkg_pkg::derive_vals so the UVM test can check them.
module derive_probe import bnn_fcc_pkg::*; #(
    parameter int TOTAL_LAYERS = 4,
    parameter int TOPOLOGY[TOTAL_LAYERS] = '{784, 256, 256, 10},
    parameter int PARALLEL_INPUTS = 16,
    parameter int PARALLEL_NEURONS[TOTAL_LAYERS-1] = '{128, 8, 5}
) ();
`include "bnn_fcc_derive.svh"

  localparam int NL = TOTAL_LAYERS - 1;

  initial begin
    pkg_pkg::derive_vals::n  = new[NL];
    pkg_pkg::derive_vals::m  = new[NL];
    pkg_pkg::derive_vals::pw = new[NL];
    pkg_pkg::derive_vals::pn = new[NL];
    pkg_pkg::derive_vals::c  = new[NL];
    pkg_pkg::derive_vals::g  = new[NL];
    pkg_pkg::derive_vals::aw = new[NL];
    pkg_pkg::derive_vals::tw = new[NL];
    for (int l = 0; l < NL; l++) begin
      pkg_pkg::derive_vals::n[l]  = l_n(l);
      pkg_pkg::derive_vals::m[l]  = l_m(l);
      pkg_pkg::derive_vals::pw[l] = l_pw(l);
      pkg_pkg::derive_vals::pn[l] = l_pn(l);
      pkg_pkg::derive_vals::c[l]  = l_c(l);
      pkg_pkg::derive_vals::g[l]  = l_g(l);
      pkg_pkg::derive_vals::aw[l] = l_aw(l);
      pkg_pkg::derive_vals::tw[l] = l_tw(l);
    end
    pkg_pkg::derive_vals::wb_np_w   = wb_np_w();
    pkg_pkg::derive_vals::wb_addr_w = wb_addr_w();
    pkg_pkg::derive_vals::wb_data_w = wb_data_w();
    pkg_pkg::derive_vals::idx_w     = idx_w();
    pkg_pkg::derive_vals::valid     = 1'b1;
  end
endmodule

module pkg_tb;
  import uvm_pkg::*;
  import pkg_pkg::*;

  derive_probe #(
      .TOTAL_LAYERS    (`PSET_TOTAL_LAYERS),
      .TOPOLOGY        (`PSET_TOPOLOGY),
      .PARALLEL_INPUTS (`PSET_PARALLEL_INPUTS),
      .PARALLEL_NEURONS(`PSET_PARALLEL_NEURONS)
  ) probe ();

  initial run_test();
endmodule
