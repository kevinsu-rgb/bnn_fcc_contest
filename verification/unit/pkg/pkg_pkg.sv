// UVM classes of the `pkg` environment: checks bnn_fcc_pkg and the
// module-scope derivation functions of bnn_fcc_derive.svh against literals,
// and cross-checks bnn_ref_pkg against the same literals.
package pkg_pkg;
  import uvm_pkg::*;
  `include "uvm_macros.svh"
  import bnn_unit_common_pkg::*;
  import bnn_ref_pkg::*;

  // Values exposed by derive_probe (pkg_tb.sv), which instantiates the
  // derivation include with the pset parameters. Index = layer l (0 .. NL-1).
  class derive_vals;
    static bit valid = 0;
    static int n[], m[], pw[], pn[], c[], g[], aw[], tw[];
    static int wb_np_w, wb_addr_w, wb_data_w, idx_w;
  endclass

  // Expected values of one pset.
  typedef struct {
    int topo[];
    int pi;
    int pnv[];
    int pw[], pn[], c[], g[], aw[], tw[];
    int wb_np_w, wb_addr_w, wb_data_w, idx_w;
  } pset_exp_t;

  function automatic pset_exp_t expected_for(string name);
    pset_exp_t e;
    case (name)
      "sfc": begin
        e.topo = '{784, 256, 256, 10};
        e.pi   = 16;
        e.pnv  = '{128, 8, 5};
        e.pw   = '{16, 128, 8};
        e.pn   = '{128, 8, 5};
        e.c    = '{49, 2, 32};
        e.g    = '{2, 32, 2};
        e.aw   = '{10, 9, 9};
        e.tw   = '{10, 9, 9};
        e.wb_np_w = 7; e.wb_addr_w = 7; e.wb_data_w = 128; e.idx_w = 4;
      end
      "odd": begin
        e.topo = '{13, 7, 5, 3};
        e.pi   = 5;
        e.pnv  = '{3, 9, 2};
        e.pw   = '{5, 3, 5};
        e.pn   = '{3, 5, 2};
        e.c    = '{3, 3, 1};
        e.g    = '{3, 1, 2};
        e.aw   = '{4, 3, 3};
        e.tw   = '{4, 4, 3};
        e.wb_np_w = 3; e.wb_addr_w = 4; e.wb_data_w = 5; e.idx_w = 2;
      end
      "t2": begin
        e.topo = '{784, 10};
        e.pi   = 16;
        e.pnv  = '{5};
        e.pw   = '{16};
        e.pn   = '{5};
        e.c    = '{49};
        e.g    = '{2};
        e.aw   = '{10};
        e.tw   = '{10};
        e.wb_np_w = 3; e.wb_addr_w = 7; e.wb_data_w = 16; e.idx_w = 4;
      end
      default: `uvm_fatal("PKG", $sformatf("no expected values for pset '%s'", name))
    endcase
    return e;
  endfunction

  class pkg_scb extends bnn_scb_base;
    `uvm_component_utils(pkg_scb)

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    // Compare an array of per-layer values (size and every element).
    protected function void check_arr(string what, int got[], int exp[]);
      check_eq({what, ".size"}, got.size(), exp.size());
      for (int i = 0; i < exp.size() && i < got.size(); i++)
        check_eq($sformatf("%s[%0d]", what, i), got[i], exp[i]);
    endfunction

    // Check the package function (DUT) and its reference against a literal.
    protected function void check_fn(string what, int dut, int ref_v, int exp);
      check_eq({"bnn_fcc_pkg::", what}, dut, exp);
      check_eq({"bnn_ref_pkg::ref_", what}, ref_v, exp);
    endfunction

    function void check_scalars();
      // constants
      check_eq("BCAST_FANOUT",       bnn_fcc_pkg::BCAST_FANOUT,       16);
      check_eq("CFG_FLUSH_CYCLES",   bnn_fcc_pkg::CFG_FLUSH_CYCLES,   16);
      check_eq("BINARIZE_THRESHOLD", bnn_fcc_pkg::BINARIZE_THRESHOLD, 128);
      check_eq("MSG_WEIGHTS",        bnn_fcc_pkg::MSG_WEIGHTS,        8'd0);
      check_eq("MSG_THRESHOLDS",     bnn_fcc_pkg::MSG_THRESHOLDS,     8'd1);
      check_eq("ref BCAST_FANOUT",       REF_BCAST_FANOUT,       16);
      check_eq("ref CFG_FLUSH_CYCLES",   REF_CFG_FLUSH_CYCLES,   16);
      check_eq("ref BINARIZE_THRESHOLD", REF_BINARIZE_THRESHOLD, 128);
      check_eq("ref MSG_WEIGHTS",        REF_MSG_WEIGHTS,        8'd0);
      check_eq("ref MSG_THRESHOLDS",     REF_MSG_THRESHOLDS,     8'd1);

      // min / max / ceil_div
      check_fn("min_int(3,5)",    bnn_fcc_pkg::min_int(3, 5),    ref_min_int(3, 5),    3);
      check_fn("min_int(5,3)",    bnn_fcc_pkg::min_int(5, 3),    ref_min_int(5, 3),    3);
      check_fn("max_int(3,5)",    bnn_fcc_pkg::max_int(3, 5),    ref_max_int(3, 5),    5);
      check_fn("max_int(5,3)",    bnn_fcc_pkg::max_int(5, 3),    ref_max_int(5, 3),    5);
      check_fn("ceil_div(784,16)", bnn_fcc_pkg::ceil_div(784, 16), ref_ceil_div(784, 16), 49);
      check_fn("ceil_div(10,5)",   bnn_fcc_pkg::ceil_div(10, 5),   ref_ceil_div(10, 5),   2);
      check_fn("ceil_div(11,5)",   bnn_fcc_pkg::ceil_div(11, 5),   ref_ceil_div(11, 5),   3);

      // width_for
      check_fn("width_for(0)",   bnn_fcc_pkg::width_for(0),   ref_width_for(0),   1);
      check_fn("width_for(1)",   bnn_fcc_pkg::width_for(1),   ref_width_for(1),   1);
      check_fn("width_for(255)", bnn_fcc_pkg::width_for(255), ref_width_for(255), 8);
      check_fn("width_for(256)", bnn_fcc_pkg::width_for(256), ref_width_for(256), 9);
      check_fn("width_for(784)", bnn_fcc_pkg::width_for(784), ref_width_for(784), 10);

      // addr_w
      check_fn("addr_w(1)",  bnn_fcc_pkg::addr_w(1),  ref_addr_w(1),  1);
      check_fn("addr_w(2)",  bnn_fcc_pkg::addr_w(2),  ref_addr_w(2),  1);
      check_fn("addr_w(3)",  bnn_fcc_pkg::addr_w(3),  ref_addr_w(3),  2);
      check_fn("addr_w(4)",  bnn_fcc_pkg::addr_w(4),  ref_addr_w(4),  2);
      check_fn("addr_w(5)",  bnn_fcc_pkg::addr_w(5),  ref_addr_w(5),  3);
      check_fn("addr_w(98)", bnn_fcc_pkg::addr_w(98), ref_addr_w(98), 7);

      // popcount_latency
      check_fn("popcount_latency(1)",   bnn_fcc_pkg::popcount_latency(1),   ref_popcount_latency(1),   1);
      check_fn("popcount_latency(3)",   bnn_fcc_pkg::popcount_latency(3),   ref_popcount_latency(3),   1);
      check_fn("popcount_latency(4)",   bnn_fcc_pkg::popcount_latency(4),   ref_popcount_latency(4),   2);
      check_fn("popcount_latency(8)",   bnn_fcc_pkg::popcount_latency(8),   ref_popcount_latency(8),   3);
      check_fn("popcount_latency(16)",  bnn_fcc_pkg::popcount_latency(16),  ref_popcount_latency(16),  4);
      check_fn("popcount_latency(128)", bnn_fcc_pkg::popcount_latency(128), ref_popcount_latency(128), 7);
      check_fn("popcount_latency(784)", bnn_fcc_pkg::popcount_latency(784), ref_popcount_latency(784), 10);

      // np_latency
      check_fn("np_latency(16)", bnn_fcc_pkg::np_latency(16), ref_np_latency(16), 6);

      // bcast_depth
      check_fn("bcast_depth(1)",   bnn_fcc_pkg::bcast_depth(1),   ref_bcast_depth(1),   1);
      check_fn("bcast_depth(16)",  bnn_fcc_pkg::bcast_depth(16),  ref_bcast_depth(16),  1);
      check_fn("bcast_depth(17)",  bnn_fcc_pkg::bcast_depth(17),  ref_bcast_depth(17),  2);
      check_fn("bcast_depth(128)", bnn_fcc_pkg::bcast_depth(128), ref_bcast_depth(128), 2);
      check_fn("bcast_depth(256)", bnn_fcc_pkg::bcast_depth(256), ref_bcast_depth(256), 2);
      check_fn("bcast_depth(257)", bnn_fcc_pkg::bcast_depth(257), ref_bcast_depth(257), 3);
    endfunction

    // Per-pset checks: tb_* are the values the tb passed to the derive_probe
    // DUT (from the PSET_* macros).
    function void check_pset(string name, int tb_tl, int tb_topo[], int tb_pi, int tb_pn[]);
      pset_exp_t e;
      int nl;
      int ref_n[], ref_m[], ref_pw[], ref_pn[], ref_c[], ref_g[], ref_aw[], ref_tw[];
      e  = expected_for(name);
      nl = e.topo.size() - 1;

      // The pset macros must describe the pset this test thinks it is running.
      check_eq("pset TOTAL_LAYERS", tb_tl, e.topo.size());
      check_arr("pset TOPOLOGY", tb_topo, e.topo);
      check_eq("pset PARALLEL_INPUTS", tb_pi, e.pi);
      check_arr("pset PARALLEL_NEURONS", tb_pn, e.pnv);

      ref_n = new[nl]; ref_m = new[nl]; ref_pw = new[nl]; ref_pn = new[nl];
      ref_c = new[nl]; ref_g = new[nl]; ref_aw = new[nl]; ref_tw = new[nl];
      for (int l = 0; l < nl; l++) begin
        ref_n[l]  = ref_l_n(l,  e.topo, e.pi, e.pnv);
        ref_m[l]  = ref_l_m(l,  e.topo, e.pi, e.pnv);
        ref_pw[l] = ref_l_pw(l, e.topo, e.pi, e.pnv);
        ref_pn[l] = ref_l_pn(l, e.topo, e.pi, e.pnv);
        ref_c[l]  = ref_l_c(l,  e.topo, e.pi, e.pnv);
        ref_g[l]  = ref_l_g(l,  e.topo, e.pi, e.pnv);
        ref_aw[l] = ref_l_aw(l, e.topo, e.pi, e.pnv);
        ref_tw[l] = ref_l_tw(l, e.topo, e.pi, e.pnv);
      end

      // bnn_ref_pkg vs literals
      for (int l = 0; l < nl; l++) begin
        check_eq($sformatf("ref_l_n[%0d]", l), ref_n[l], e.topo[l]);
        check_eq($sformatf("ref_l_m[%0d]", l), ref_m[l], e.topo[l+1]);
      end
      check_arr("ref_l_pw", ref_pw, e.pw);
      check_arr("ref_l_pn", ref_pn, e.pn);
      check_arr("ref_l_c",  ref_c,  e.c);
      check_arr("ref_l_g",  ref_g,  e.g);
      check_arr("ref_l_aw", ref_aw, e.aw);
      check_arr("ref_l_tw", ref_tw, e.tw);
      check_eq("ref_wb_np_w",   ref_wb_np_w(e.topo, e.pi, e.pnv),   e.wb_np_w);
      check_eq("ref_wb_addr_w", ref_wb_addr_w(e.topo, e.pi, e.pnv), e.wb_addr_w);
      check_eq("ref_wb_data_w", ref_wb_data_w(e.topo, e.pi, e.pnv), e.wb_data_w);
      check_eq("ref_idx_w",     ref_idx_w(e.topo, e.pi, e.pnv),     e.idx_w);

      // bnn_fcc_derive.svh (via derive_probe) vs literals
      check_arr("l_n",  derive_vals::n,  ref_n);
      check_arr("l_m",  derive_vals::m,  ref_m);
      check_arr("l_pw", derive_vals::pw, e.pw);
      check_arr("l_pn", derive_vals::pn, e.pn);
      check_arr("l_c",  derive_vals::c,  e.c);
      check_arr("l_g",  derive_vals::g,  e.g);
      check_arr("l_aw", derive_vals::aw, e.aw);
      check_arr("l_tw", derive_vals::tw, e.tw);
      check_eq("wb_np_w()",   derive_vals::wb_np_w,   e.wb_np_w);
      check_eq("wb_addr_w()", derive_vals::wb_addr_w, e.wb_addr_w);
      check_eq("wb_data_w()", derive_vals::wb_data_w, e.wb_data_w);
      check_eq("idx_w()",     derive_vals::idx_w,     e.idx_w);
    endfunction
  endclass

  class pkg_values_test extends uvm_test;
    `uvm_component_utils(pkg_values_test)

    pkg_scb scb;

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      scb = pkg_scb::type_id::create("pkg_scb", this);
    endfunction

    task run_phase(uvm_phase phase);
      int tb_topo[] = `PSET_TOPOLOGY;
      int tb_pn[]   = `PSET_PARALLEL_NEURONS;
      phase.raise_objection(this);
      fork
        bnn_watchdog("pkg_values_test", 1000);
      join_none
      // poll: derive_probe fills derive_vals from an initial block at time 0
      while (!derive_vals::valid) #1ns;
      scb.check_scalars();
      scb.check_pset(`PSET_NAME, `PSET_TOTAL_LAYERS, tb_topo, `PSET_PARALLEL_INPUTS, tb_pn);
      phase.drop_objection(this);
    endtask
  endclass

endpackage
