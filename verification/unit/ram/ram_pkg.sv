// UVM classes of the `ram` environment (DUT: bnn_ram).
//
// Reference model: a flat array written at the write address (SP: rd_addr,
// SDP: wr_addr). The read issued in cycle k is checked against the array
// contents *before* the write of cycle k (read-before-write) when rd_data is
// observed READ_LATENCY cycles later. Reads of addresses never written so far
// have undefined data and are not checked; reads in an SP write cycle do not
// exist (the address bus is used for the write) and are not checked either.
package ram_pkg;
  import uvm_pkg::*;
  `include "uvm_macros.svh"
  import bnn_unit_common_pkg::*;
  import bnn_ref_pkg::*;

  // ---- configuration (from the pset macros) -------------------------------
  localparam int RAM_WIDTH   = `PSET_WIDTH;
  localparam int RAM_DEPTH   = `PSET_DEPTH;
  localparam int RAM_LATENCY = `PSET_READ_LATENCY;
  localparam bit RAM_IS_SDP  = (`PSET_PORTS == "SDP");
  localparam int RAM_AW      = ref_addr_w(RAM_DEPTH);
  localparam int RAM_RANDOM_CYCLES = 12000;

  typedef bit [RAM_WIDTH-1:0] ram_data_t;
  typedef bit [RAM_AW-1:0]    ram_addr_t;

  // ---- items ---------------------------------------------------------------
  // One clock cycle of stimulus.
  class ram_item extends uvm_sequence_item;
    `uvm_object_utils(ram_item)
    bit        we;
    ram_addr_t wr_addr;
    ram_data_t wr_data;
    ram_addr_t rd_addr;
    function new(string name = "ram_item");
      super.new(name);
    endfunction
  endclass

  // One observed clock cycle (inputs sampled at the edge, rd_data as seen at
  // the same edge, i.e. the value produced by earlier inputs).
  class ram_txn extends uvm_sequence_item;
    `uvm_object_utils(ram_txn)
    bit        we;
    ram_addr_t wr_addr;
    ram_data_t wr_data;
    ram_addr_t rd_addr;
    ram_data_t rd_data;
    function new(string name = "ram_txn");
      super.new(name);
    endfunction
  endclass

  // ---- sequence ------------------------------------------------------------
  class ram_rw_seq extends uvm_sequence #(ram_item);
    `uvm_object_utils(ram_rw_seq)

    ram_addr_t recent[$];

    function new(string name = "ram_rw_seq");
      super.new(name);
    endfunction

    function ram_data_t rnd_data();
      ram_data_t d;
      d = '0;
      for (int i = 0; i < RAM_WIDTH; i += 32) d |= ram_data_t'($urandom()) << i;
      return d;
    endfunction

    function ram_addr_t rnd_addr();
      ram_addr_t a;
      int unsigned idx;
      if (recent.size() > 0 && $urandom_range(99) < 40) begin
        idx = $urandom_range(recent.size() - 1);
        a   = recent[idx];
      end else begin
        a = ram_addr_t'($urandom_range(RAM_DEPTH - 1));
      end
      return a;
    endfunction

    task send(bit we, ram_addr_t wa, ram_data_t wd, ram_addr_t ra);
      ram_item it;
      it = ram_item::type_id::create("it");
      start_item(it);
      it.we      = we;
      it.wr_addr = wa;
      it.wr_data = wd;
      it.rd_addr = ra;
      finish_item(it);
    endtask

    task note_write(ram_addr_t a);
      recent.push_back(a);
      if (recent.size() > 8) void'(recent.pop_front());
    endtask

    task body();
      bit        we;
      ram_addr_t wa, ra, a;
      ram_data_t wd;

      // Initialise every address once so later reads are checkable.
      for (int i = 0; i < RAM_DEPTH; i++) begin
        a  = ram_addr_t'(i);
        wd = rnd_data();
        if (RAM_IS_SDP) begin
          ra = rnd_addr();
          send(1'b1, a, wd, ra);
        end else begin
          send(1'b1, '0, wd, a);
        end
        note_write(a);
      end

      // Random interleaving.
      for (int i = 0; i < RAM_RANDOM_CYCLES; i++) begin
        wd = rnd_data();
        if (RAM_IS_SDP) begin
          we = ($urandom_range(99) < 55);
          wa = rnd_addr();
          if (we && $urandom_range(99) < 30) ra = wa;   // same-address read+write
          else ra = rnd_addr();
          send(we, wa, wd, ra);
          if (we) note_write(wa);
        end else begin
          we = ($urandom_range(99) < 35);                // never read+write together
          a  = rnd_addr();
          send(we, '0, wd, a);
          if (we) note_write(a);
        end
      end

      // Idle tail: pure reads, lets the last reads drain.
      for (int i = 0; i < RAM_LATENCY + 2; i++) begin
        ra = rnd_addr();
        send(1'b0, '0, '0, ra);
      end
    endtask
  endclass

  // ---- driver / monitor ----------------------------------------------------
  class ram_driver extends uvm_driver #(ram_item);
    `uvm_component_utils(ram_driver)
    virtual ram_if #(RAM_WIDTH, RAM_AW) vif;
    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction
    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      if (!uvm_config_db#(virtual ram_if #(RAM_WIDTH, RAM_AW))::get(this, "", "vif", vif))
        `uvm_fatal("NOVIF", "ram_if not set")
    endfunction
    task run_phase(uvm_phase phase);
      forever begin
        seq_item_port.get_next_item(req);
        @(posedge vif.clk);
        vif.we      <= req.we;
        vif.wr_addr <= req.wr_addr;
        vif.wr_data <= req.wr_data;
        vif.rd_addr <= req.rd_addr;
        seq_item_port.item_done();
      end
    endtask
  endclass

  class ram_monitor extends uvm_monitor;
    `uvm_component_utils(ram_monitor)
    virtual ram_if #(RAM_WIDTH, RAM_AW) vif;
    uvm_analysis_port #(ram_txn) ap;
    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction
    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      ap = new("ap", this);
      if (!uvm_config_db#(virtual ram_if #(RAM_WIDTH, RAM_AW))::get(this, "", "vif", vif))
        `uvm_fatal("NOVIF", "ram_if not set")
    endfunction
    task run_phase(uvm_phase phase);
      ram_txn t;
      forever begin
        @(posedge vif.clk);
        t = ram_txn::type_id::create("t");
        t.we      = vif.we;
        t.wr_addr = vif.wr_addr;
        t.wr_data = vif.wr_data;
        t.rd_addr = vif.rd_addr;
        t.rd_data = vif.rd_data;
        ap.write(t);
      end
    endtask
  endclass

  // ---- scoreboard ------------------------------------------------------------
  class ram_scb extends bnn_scb_base;
    `uvm_component_utils(ram_scb)
    uvm_analysis_imp #(ram_txn, ram_scb) imp;

    typedef struct {
      bit        valid;
      ram_addr_t addr;
      ram_data_t data;
    } pend_t;

    ram_data_t mem[];
    bit        wrote[];
    pend_t     pipe[$];

    function new(string name, uvm_component parent);
      super.new(name, parent);
      imp   = new("imp", this);
      mem   = new[RAM_DEPTH];
      wrote = new[RAM_DEPTH];
    endfunction

    function void write(ram_txn t);
      pend_t     p, e;
      ram_addr_t wa;
      int        ra_i, wa_i;

      // rd_data now belongs to the read issued RAM_LATENCY cycles ago.
      if (pipe.size() == RAM_LATENCY) begin
        e = pipe.pop_front();
        if (e.valid) check_eq($sformatf("rd_data[addr=%0d]", e.addr), t.rd_data, e.data);
      end

      // Expected result of the read issued this cycle (state before this write).
      p.valid = 1'b0;
      p.addr  = t.rd_addr;
      p.data  = '0;
      ra_i    = int'(t.rd_addr);
      if ((RAM_IS_SDP || !t.we) && ra_i < RAM_DEPTH && wrote[ra_i]) begin
        p.valid = 1'b1;
        p.data  = mem[ra_i];
      end
      pipe.push_back(p);

      // Apply this cycle's write.
      if (t.we) begin
        wa   = RAM_IS_SDP ? t.wr_addr : t.rd_addr;
        wa_i = int'(wa);
        if (wa_i < RAM_DEPTH) begin
          mem[wa_i]   = t.wr_data;
          wrote[wa_i] = 1'b1;
        end
      end
    endfunction
  endclass

  // ---- coverage ----------------------------------------------------------------
  class ram_cov extends uvm_subscriber #(ram_txn);
    `uvm_component_utils(ram_cov)
    bit        we;
    bit        same;
    int        addr_q;

    covergroup cg;
      cp_we:   coverpoint we;
      cp_same: coverpoint same;
      cp_x:    cross cp_we, cp_same;
      cp_addr: coverpoint addr_q {
        bins lo  = {0};
        bins mid = {[1 : (RAM_DEPTH > 2 ? RAM_DEPTH - 2 : 1)]};
        bins hi  = {RAM_DEPTH - 1};
      }
    endgroup

    function new(string name, uvm_component parent);
      super.new(name, parent);
      cg = new();
    endfunction

    function void write(ram_txn t);
      we     = t.we;
      same   = t.we && (t.wr_addr == t.rd_addr);
      addr_q = int'(t.rd_addr);
      cg.sample();
    endfunction
  endclass

  // ---- env / test --------------------------------------------------------------------
  class ram_env extends uvm_env;
    `uvm_component_utils(ram_env)
    uvm_sequencer #(ram_item) sqr;
    ram_driver  drv;
    ram_monitor mon;
    ram_scb     scb;
    ram_cov     cov;
    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction
    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      sqr = uvm_sequencer #(ram_item)::type_id::create("sqr", this);
      drv = ram_driver::type_id::create("drv", this);
      mon = ram_monitor::type_id::create("mon", this);
      scb = ram_scb::type_id::create("ram", this);
      cov = ram_cov::type_id::create("cov", this);
    endfunction
    function void connect_phase(uvm_phase phase);
      drv.seq_item_port.connect(sqr.seq_item_export);
      mon.ap.connect(scb.imp);
      mon.ap.connect(cov.analysis_export);
    endfunction
  endclass

  class ram_rw_test extends uvm_test;
    `uvm_component_utils(ram_rw_test)
    ram_env env;
    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction
    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      env = ram_env::type_id::create("env", this);
    endfunction
    task run_phase(uvm_phase phase);
      ram_rw_seq seq;
      phase.raise_objection(this);
      fork
        bnn_watchdog("ram_rw_test", 2 * (RAM_DEPTH + RAM_RANDOM_CYCLES + 100));
      join_none
      seq = ram_rw_seq::type_id::create("seq");
      seq.start(env.sqr);
      repeat (RAM_LATENCY + 4) @(posedge env.drv.vif.clk);
      phase.drop_objection(this);
    endtask
  endclass

endpackage
