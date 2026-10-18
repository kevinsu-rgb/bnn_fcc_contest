// UVM classes of the `bcast` environment (DUT: bnn_bcast).
//
// Reference: every dout[i] equals din delayed exactly ref_bcast_depth(N_OUT) cycles.
// din changes every cycle and there is no valid signal, so the scoreboard checks all
// N_OUT outputs in every cycle against the din sampled exactly BC_DEPTH edges before.
// The first BC_DEPTH cycles are not checked (pipeline contents undefined: no reset).
// Sampled outputs are 4-state, so X/Z is a mismatch.
package bcast_pkg;
  import uvm_pkg::*;
  `include "uvm_macros.svh"
  import bnn_unit_common_pkg::*;
  import bnn_ref_pkg::*;

  // ---- configuration (from the pset macros) -------------------------------
  localparam int BC_WIDTH = `PSET_WIDTH;
  localparam int BC_N_OUT = `PSET_N_OUT;
  localparam int BC_DEPTH = ref_bcast_depth(BC_N_OUT);
  localparam int BC_RANDOM_CYCLES = 4000;
  localparam int BC_DIRECTED_CYCLES = 64;

  typedef bit [BC_WIDTH-1:0] bc_data_t;

  // ---- items ---------------------------------------------------------------
  class bc_item extends uvm_sequence_item;
    `uvm_object_utils(bc_item)
    bc_data_t din;
    function new(string name = "bc_item");
      super.new(name);
    endfunction
  endclass

  // One observed clock cycle: din sampled at the edge, dout as seen at the same
  // edge (value produced by earlier din). 4-state on purpose.
  class bc_txn extends uvm_sequence_item;
    `uvm_object_utils(bc_txn)
    logic [BC_WIDTH-1:0] din;
    logic [BC_WIDTH-1:0] dout[BC_N_OUT];
    function new(string name = "bc_txn");
      super.new(name);
    endfunction
  endclass

  // ---- sequence ------------------------------------------------------------
  class bc_random_seq extends uvm_sequence #(bc_item);
    `uvm_object_utils(bc_random_seq)

    function new(string name = "bc_random_seq");
      super.new(name);
    endfunction

    function bc_data_t rnd_data();
      bc_data_t d;
      d = '0;
      for (int i = 0; i < BC_WIDTH; i += 32) d |= bc_data_t'($urandom()) << i;
      return d;
    endfunction

    task send(bc_data_t d);
      bc_item it;
      it = bc_item::type_id::create("it");
      start_item(it);
      it.din = d;
      finish_item(it);
    endtask

    task body();
      bc_data_t d, ones;
      ones = '1;

      // Directed: alternating all-zero / all-one (every cycle differs from its neighbours).
      for (int r = 0; r < BC_DIRECTED_CYCLES; r++) send((r % 2 == 0) ? '0 : ones);

      // Directed: walking one over the data bits.
      for (int r = 0; r < 2 * BC_WIDTH; r++) begin
        d = '0;
        d[r % BC_WIDTH] = 1'b1;
        send(d);
      end

      // Random every cycle, with occasional short holds (a hold must not disturb the delay).
      for (int r = 0; r < BC_RANDOM_CYCLES; r++) begin
        d = rnd_data();
        send(d);
        if ($urandom_range(15) == 0) repeat ($urandom_range(1, 3)) send(d);
      end

      // Idle tail: random data, lets the last results drain.
      for (int r = 0; r < BC_DEPTH + 2; r++) send(rnd_data());
    endtask
  endclass

  // ---- driver / monitor ----------------------------------------------------
  class bc_driver extends uvm_driver #(bc_item);
    `uvm_component_utils(bc_driver)
    virtual bcast_if #(BC_WIDTH, BC_N_OUT) vif;
    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction
    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      if (!uvm_config_db#(virtual bcast_if #(BC_WIDTH, BC_N_OUT))::get(this, "", "vif", vif))
        `uvm_fatal("NOVIF", "bcast_if not set")
    endfunction
    task run_phase(uvm_phase phase);
      forever begin
        seq_item_port.get_next_item(req);
        @(posedge vif.clk);
        vif.din <= req.din;
        seq_item_port.item_done();
      end
    endtask
  endclass

  class bc_monitor extends uvm_monitor;
    `uvm_component_utils(bc_monitor)
    virtual bcast_if #(BC_WIDTH, BC_N_OUT) vif;
    uvm_analysis_port #(bc_txn) ap;
    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction
    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      ap = new("ap", this);
      if (!uvm_config_db#(virtual bcast_if #(BC_WIDTH, BC_N_OUT))::get(this, "", "vif", vif))
        `uvm_fatal("NOVIF", "bcast_if not set")
    endfunction
    task run_phase(uvm_phase phase);
      bc_txn t;
      forever begin
        @(posedge vif.clk);
        t = bc_txn::type_id::create("t");
        t.din = vif.din;
        for (int i = 0; i < BC_N_OUT; i++) t.dout[i] = vif.dout[i];
        ap.write(t);
      end
    endtask
  endclass

  // ---- scoreboard ------------------------------------------------------------
  class bc_scb extends bnn_scb_base;
    `uvm_component_utils(bc_scb)
    uvm_analysis_imp #(bc_txn, bc_scb) imp;

    bc_data_t    pipe[$];   // din of the last BC_DEPTH cycles
    int unsigned cyc;

    function new(string name, uvm_component parent);
      super.new(name, parent);
      imp = new("imp", this);
      cyc = 0;
    endfunction

    function void write(bc_txn t);
      bc_data_t e;
      // Every dout[i] now belongs to the din presented BC_DEPTH cycles ago.
      if (pipe.size() == BC_DEPTH) begin
        e = pipe.pop_front();
        for (int i = 0; i < BC_N_OUT; i++) begin
          if (t.dout[i] === e) check_eq("dout", t.dout[i], e);
          else check_eq($sformatf("dout[%0d][cycle=%0d]", i, cyc), t.dout[i], e);
        end
      end
      pipe.push_back(bc_data_t'(t.din));
      cyc++;
    endfunction
  endclass

  // ---- coverage ----------------------------------------------------------------
  class bc_cov extends uvm_subscriber #(bc_txn);
    `uvm_component_utils(bc_cov)
    bit        changed;
    bc_data_t  prev, cur;

    covergroup cg;
      cp_val: coverpoint cur {
        bins zero = {0};
        bins ones = {'1};
        bins other = default;
      }
      cp_chg: coverpoint changed;
    endgroup

    function new(string name, uvm_component parent);
      super.new(name, parent);
      cg = new();
    endfunction

    function void write(bc_txn t);
      cur     = bc_data_t'(t.din);
      changed = (cur != prev);
      prev    = cur;
      cg.sample();
    endfunction
  endclass

  // ---- env / test --------------------------------------------------------------------
  class bc_env extends uvm_env;
    `uvm_component_utils(bc_env)
    uvm_sequencer #(bc_item) sqr;
    bc_driver  drv;
    bc_monitor mon;
    bc_scb     scb;
    bc_cov     cov;
    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction
    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      sqr = uvm_sequencer #(bc_item)::type_id::create("sqr", this);
      drv = bc_driver::type_id::create("drv", this);
      mon = bc_monitor::type_id::create("mon", this);
      scb = bc_scb::type_id::create("bcast", this);
      cov = bc_cov::type_id::create("cov", this);
    endfunction
    function void connect_phase(uvm_phase phase);
      drv.seq_item_port.connect(sqr.seq_item_export);
      mon.ap.connect(scb.imp);
      mon.ap.connect(cov.analysis_export);
    endfunction
  endclass

  class bcast_random_test extends uvm_test;
    `uvm_component_utils(bcast_random_test)
    bc_env env;
    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction
    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      env = bc_env::type_id::create("env", this);
    endfunction
    task run_phase(uvm_phase phase);
      bc_random_seq seq;
      phase.raise_objection(this);
      fork
        // Random phase averages ~1.5 items per iteration with holds; budget generously.
        bnn_watchdog("bcast_random_test",
                     3 * (BC_DIRECTED_CYCLES + 2 * BC_WIDTH + 2 * BC_RANDOM_CYCLES + 100));
      join_none
      seq = bc_random_seq::type_id::create("seq");
      seq.start(env.sqr);
      repeat (BC_DEPTH + 4) @(posedge env.drv.vif.clk);
      phase.drop_objection(this);
    endtask
  endclass

endpackage
