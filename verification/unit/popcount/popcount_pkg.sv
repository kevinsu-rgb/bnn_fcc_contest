// UVM classes of the `popcount` environment (DUT: bnn_popcount).
//
// Reference: count = #(a[i] == b[i]) of the inputs presented ref_popcount_latency(WIDTH)
// cycles earlier. Inputs change every cycle and there is no valid signal, so the
// scoreboard checks `count` in every cycle against the inputs sampled exactly
// PC_LATENCY edges before. The first PC_LATENCY cycles are not checked (the pipeline
// contents are undefined: no reset). Sampled outputs are 4-state, so X/Z is a mismatch.
package popcount_pkg;
  import uvm_pkg::*;
  `include "uvm_macros.svh"
  import bnn_unit_common_pkg::*;
  import bnn_ref_pkg::*;

  // ---- configuration (from the pset macros) -------------------------------
  localparam int PC_WIDTH   = `PSET_WIDTH;
  localparam int PC_CW      = ref_width_for(PC_WIDTH);
  localparam int PC_LATENCY = ref_popcount_latency(PC_WIDTH);
  localparam int PC_RANDOM_CYCLES = 3000;
  localparam int PC_DIRECTED_REPS = 16;

  typedef bit [PC_WIDTH-1:0] pc_vec_t;

  // ---- items ---------------------------------------------------------------
  // One clock cycle of stimulus.
  class pc_item extends uvm_sequence_item;
    `uvm_object_utils(pc_item)
    pc_vec_t a;
    pc_vec_t b;
    function new(string name = "pc_item");
      super.new(name);
    endfunction
  endclass

  // One observed clock cycle: inputs sampled at the edge, count as seen at the
  // same edge (value produced by inputs presented earlier). 4-state on purpose.
  class pc_txn extends uvm_sequence_item;
    `uvm_object_utils(pc_txn)
    logic [PC_WIDTH-1:0] a;
    logic [PC_WIDTH-1:0] b;
    logic [PC_CW-1:0]    count;
    function new(string name = "pc_txn");
      super.new(name);
    endfunction
  endclass

  // ---- sequence ------------------------------------------------------------
  class pc_random_seq extends uvm_sequence #(pc_item);
    `uvm_object_utils(pc_random_seq)

    function new(string name = "pc_random_seq");
      super.new(name);
    endfunction

    function pc_vec_t rnd_vec();
      pc_vec_t v;
      v = '0;
      for (int i = 0; i < PC_WIDTH; i += 32) v |= pc_vec_t'($urandom()) << i;
      return v;
    endfunction

    task send(pc_vec_t a, pc_vec_t b);
      pc_item it;
      it = pc_item::type_id::create("it");
      start_item(it);
      it.a = a;
      it.b = b;
      finish_item(it);
    endtask

    task body();
      pc_vec_t a, b, ones;
      ones = '1;

      // Directed: all equal (expect WIDTH), including all-0 and all-1 patterns.
      send('0, '0);
      send(ones, ones);
      for (int r = 0; r < PC_DIRECTED_REPS; r++) begin
        a = rnd_vec();
        send(a, a);
      end

      // Directed: all different (expect 0).
      send('0, ones);
      send(ones, '0);
      for (int r = 0; r < PC_DIRECTED_REPS; r++) begin
        a = rnd_vec();
        send(a, ~a);
      end

      // Directed: alternating all-equal / all-different (adjacent-cycle sensitivity).
      for (int r = 0; r < 2 * PC_DIRECTED_REPS; r++) begin
        a = rnd_vec();
        if (r % 2 == 0) send(a, a);
        else send(a, ~a);
      end

      // Directed: walking single mismatch (expect WIDTH-1) ...
      a = rnd_vec();
      for (int i = 0; i < PC_WIDTH; i++) begin
        b = a;
        b[i] = ~b[i];
        send(a, b);
      end
      // ... and walking single match (expect 1).
      a = rnd_vec();
      for (int i = 0; i < PC_WIDTH; i++) begin
        b = ~a;
        b[i] = ~b[i];
        send(a, b);
      end

      // Random every cycle, mixed with biased modes.
      for (int r = 0; r < PC_RANDOM_CYCLES; r++) begin
        a = rnd_vec();
        case ($urandom_range(9))
          0: b = a;                                  // all equal
          1: b = ~a;                                 // all different
          2: begin                                   // one flipped bit
            b = a;
            b[$urandom_range(PC_WIDTH - 1)] ^= 1'b1;
          end
          default: b = rnd_vec();
        endcase
        send(a, b);
      end

      // Idle tail: random inputs, lets the last results drain.
      for (int r = 0; r < PC_LATENCY + 2; r++) send(rnd_vec(), rnd_vec());
    endtask
  endclass

  // ---- driver / monitor ----------------------------------------------------
  class pc_driver extends uvm_driver #(pc_item);
    `uvm_component_utils(pc_driver)
    virtual popcount_if #(PC_WIDTH, PC_CW) vif;
    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction
    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      if (!uvm_config_db#(virtual popcount_if #(PC_WIDTH, PC_CW))::get(this, "", "vif", vif))
        `uvm_fatal("NOVIF", "popcount_if not set")
    endfunction
    task run_phase(uvm_phase phase);
      forever begin
        seq_item_port.get_next_item(req);
        @(posedge vif.clk);
        vif.a <= req.a;
        vif.b <= req.b;
        seq_item_port.item_done();
      end
    endtask
  endclass

  class pc_monitor extends uvm_monitor;
    `uvm_component_utils(pc_monitor)
    virtual popcount_if #(PC_WIDTH, PC_CW) vif;
    uvm_analysis_port #(pc_txn) ap;
    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction
    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      ap = new("ap", this);
      if (!uvm_config_db#(virtual popcount_if #(PC_WIDTH, PC_CW))::get(this, "", "vif", vif))
        `uvm_fatal("NOVIF", "popcount_if not set")
    endfunction
    task run_phase(uvm_phase phase);
      pc_txn t;
      forever begin
        @(posedge vif.clk);
        t = pc_txn::type_id::create("t");
        t.a     = vif.a;
        t.b     = vif.b;
        t.count = vif.count;
        ap.write(t);
      end
    endtask
  endclass

  // ---- scoreboard ------------------------------------------------------------
  class pc_scb extends bnn_scb_base;
    `uvm_component_utils(pc_scb)
    uvm_analysis_imp #(pc_txn, pc_scb) imp;

    int unsigned pipe[$];   // expected counts of the inputs of the last PC_LATENCY cycles
    int unsigned cyc;

    function new(string name, uvm_component parent);
      super.new(name, parent);
      imp = new("imp", this);
      cyc = 0;
    endfunction

    function int unsigned ref_count(logic [PC_WIDTH-1:0] a, logic [PC_WIDTH-1:0] b);
      int unsigned n;
      n = 0;
      for (int i = 0; i < PC_WIDTH; i++) if (a[i] === b[i]) n++;
      return n;
    endfunction

    function void write(pc_txn t);
      int unsigned e;
      // count now belongs to the inputs presented PC_LATENCY cycles ago.
      if (pipe.size() == PC_LATENCY) begin
        e = pipe.pop_front();
        if (t.count === e[PC_CW-1:0]) check_eq("count", t.count, e);
        else check_eq($sformatf("count[cycle=%0d]", cyc), t.count, e);
      end
      e = ref_count(t.a, t.b);
      pipe.push_back(e);
      cyc++;
    endfunction
  endclass

  // ---- coverage ----------------------------------------------------------------
  class pc_cov extends uvm_subscriber #(pc_txn);
    `uvm_component_utils(pc_cov)
    int unsigned n;

    covergroup cg;
      cp_n: coverpoint n {
        bins none = {0};
        bins one  = {1};
        bins mid  = {[2 : (PC_WIDTH > 3 ? PC_WIDTH - 2 : 2)]};
        bins most = {(PC_WIDTH > 1 ? PC_WIDTH - 1 : 0)};
        bins all  = {PC_WIDTH};
      }
    endgroup

    function new(string name, uvm_component parent);
      super.new(name, parent);
      cg = new();
    endfunction

    function void write(pc_txn t);
      n = 0;
      for (int i = 0; i < PC_WIDTH; i++) if (t.a[i] === t.b[i]) n++;
      cg.sample();
    endfunction
  endclass

  // ---- env / test --------------------------------------------------------------------
  class pc_env extends uvm_env;
    `uvm_component_utils(pc_env)
    uvm_sequencer #(pc_item) sqr;
    pc_driver  drv;
    pc_monitor mon;
    pc_scb     scb;
    pc_cov     cov;
    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction
    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      sqr = uvm_sequencer #(pc_item)::type_id::create("sqr", this);
      drv = pc_driver::type_id::create("drv", this);
      mon = pc_monitor::type_id::create("mon", this);
      scb = pc_scb::type_id::create("popcount", this);
      cov = pc_cov::type_id::create("cov", this);
    endfunction
    function void connect_phase(uvm_phase phase);
      drv.seq_item_port.connect(sqr.seq_item_export);
      mon.ap.connect(scb.imp);
      mon.ap.connect(cov.analysis_export);
    endfunction
  endclass

  class popcount_random_test extends uvm_test;
    `uvm_component_utils(popcount_random_test)
    pc_env env;
    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction
    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      env = pc_env::type_id::create("env", this);
    endfunction
    task run_phase(uvm_phase phase);
      pc_random_seq seq;
      phase.raise_objection(this);
      fork
        bnn_watchdog("popcount_random_test",
                     2 * (2 * PC_DIRECTED_REPS * 4 + 2 * PC_WIDTH + PC_RANDOM_CYCLES + 100));
      join_none
      seq = pc_random_seq::type_id::create("seq");
      seq.start(env.sqr);
      repeat (PC_LATENCY + 4) @(posedge env.drv.vif.clk);
      phase.drop_objection(this);
    endtask
  endclass

endpackage
