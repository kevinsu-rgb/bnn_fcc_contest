// UVM classes of the `np` environment (DUT: bnn_np, the neuron processor).
//
// Stimulus: neurons of C contiguous cycles (in_first on the first, in_last on the
// last), driven back-to-back from time 0 with no idle gaps. in_thr carries the real
// threshold only on the in_last cycle; on every other cycle it is random garbage.
//
// Reference: for each neuron, count = sum over its cycles of #(x == w) and
// bit = (count >= thr), with thr taken from the in_last cycle. Both outputs are
// checked in the cycle exactly NP_LATENCY = ref_np_latency(PW) edges after the edge
// that sampled the neuron's in_last cycle; outputs in all other cycles are
// don't-care. Sampled outputs are 4-state, so X/Z is a mismatch.
package np_pkg;
  import uvm_pkg::*;
  `include "uvm_macros.svh"
  import bnn_unit_common_pkg::*;
  import bnn_ref_pkg::*;

  // ---- configuration (from the pset macros) -------------------------------
  localparam int NP_PW      = `PSET_PW;
  localparam int NP_AW      = `PSET_AW;
  localparam int NP_TW      = `PSET_TW;
  localparam int NP_LATENCY = ref_np_latency(NP_PW);

  // Largest C <= 8 such that the count (C*PW) fits AW bits and thr = N+1 fits TW bits.
  function automatic int np_cmax();
    int c;
    c = 8;
    while (c > 1 && !((c * NP_PW) < (1 << NP_AW) && (c * NP_PW + 1) < (1 << NP_TW))) c--;
    return c;
  endfunction

  localparam int NP_CMAX             = np_cmax();
  localparam int NP_RANDOM_NEURONS   = 4000;
  localparam int NP_DIRECTED_NEURONS = 8 * 5 * 5 + 64;   // upper bound, for the watchdog

  typedef bit [NP_PW-1:0] np_vec_t;
  typedef bit [NP_TW-1:0] np_thr_t;

  // ---- items ---------------------------------------------------------------
  // One clock cycle of stimulus.
  class np_item extends uvm_sequence_item;
    `uvm_object_utils(np_item)
    bit      first;
    bit      last;
    np_vec_t x;
    np_vec_t w;
    np_thr_t thr;
    function new(string name = "np_item");
      super.new(name);
    endfunction
  endclass

  // One observed clock cycle: inputs sampled at the edge, outputs as seen at the
  // same edge (produced by inputs presented earlier). 4-state on purpose.
  class np_txn extends uvm_sequence_item;
    `uvm_object_utils(np_txn)
    logic          first;
    logic          last;
    logic [NP_PW-1:0] x;
    logic [NP_PW-1:0] w;
    logic [NP_TW-1:0] thr;
    logic          out_bit;
    logic [NP_AW-1:0] out_count;
    function new(string name = "np_txn");
      super.new(name);
    endfunction
  endclass

  // ---- sequence ------------------------------------------------------------
  class np_random_seq extends uvm_sequence #(np_item);
    `uvm_object_utils(np_random_seq)

    function new(string name = "np_random_seq");
      super.new(name);
    endfunction

    function np_vec_t rnd_vec();
      np_vec_t v;
      v = '0;
      for (int i = 0; i < NP_PW; i += 32) v |= np_vec_t'($urandom()) << i;
      return v;
    endfunction

    function np_thr_t rnd_thr();
      np_thr_t v;
      v = '0;
      for (int i = 0; i < NP_TW; i += 32) v |= np_thr_t'($urandom()) << i;
      return v;
    endfunction

    // Send one neuron of c cycles back-to-back with the previous one.
    //   target >= 0 : exactly `target` of the N = c*PW positions match (x == w)
    //   target <  0 : every position matches with probability 1/2
    //   thr_kind 0  : threshold from {0, 1, N-1, N, N+1, count, count+1, count-1, random}
    //   thr_kind 1  : threshold = count + thr_arg  (clamped to [0, N+1])
    //   thr_kind 2  : threshold = thr_arg          (clamped to [0, N+1])
    task send_neuron(int c, int target, int thr_kind, int thr_arg);
      int n;
      bit match[];
      int perm[];
      int cnt, thr, j, k, tmp;
      np_item it;
      np_vec_t xv, wv;

      n = c * NP_PW;
      match = new[n];
      if (target >= 0) begin
        perm = new[n];
        for (int i = 0; i < n; i++) perm[i] = i;
        for (int i = n - 1; i > 0; i--) begin   // Fisher-Yates
          j = $urandom_range(i);
          tmp = perm[i]; perm[i] = perm[j]; perm[j] = tmp;
        end
        for (int i = 0; i < n; i++) match[perm[i]] = (i < target);
      end else begin
        for (int i = 0; i < n; i++) match[i] = $urandom_range(1);
      end
      cnt = 0;
      for (int i = 0; i < n; i++) cnt += match[i];

      case (thr_kind)
        1: thr = cnt + thr_arg;
        2: thr = thr_arg;
        default: begin
          case ($urandom_range(9))
            0: thr = 0;
            1: thr = 1;
            2: thr = n - 1;
            3: thr = n;
            4: thr = n + 1;
            5: thr = cnt;
            6: thr = cnt + 1;
            7: thr = cnt - 1;
            default: thr = $urandom_range(n + 1);
          endcase
        end
      endcase
      if (thr < 0) thr = 0;
      if (thr > n + 1) thr = n + 1;

      for (int cy = 0; cy < c; cy++) begin
        xv = rnd_vec();
        wv = '0;
        for (int b = 0; b < NP_PW; b++) wv[b] = match[cy * NP_PW + b] ? xv[b] : ~xv[b];
        it = np_item::type_id::create("it");
        start_item(it);
        it.first = (cy == 0);
        it.last  = (cy == c - 1);
        it.x     = xv;
        it.w     = wv;
        // The threshold is only defined on the in_last cycle; garbage otherwise.
        it.thr   = (cy == c - 1) ? np_thr_t'(thr) : rnd_thr();
        finish_item(it);
      end
    endtask

    function int rnd_target(int n);
      case ($urandom_range(7))
        0: return n;
        1: return 0;
        2: return n - 1;
        3: return 1;
        4: return n / 2;
        default: return $urandom_range(n);
      endcase
    endfunction

    task body();
      int n, cnts[5];
      // Directed: popcount == threshold (output 1), popcount == threshold - 1 (output 0),
      // popcount == threshold + 1 (output 1), for every C and a spread of counts.
      for (int c = 1; c <= NP_CMAX; c++) begin
        n = c * NP_PW;
        cnts[0] = 0;
        cnts[1] = 1;
        cnts[2] = n / 2;
        cnts[3] = n - 1;
        cnts[4] = n;
        foreach (cnts[i]) begin
          send_neuron(c, cnts[i], 1, 0);    // count == thr   -> 1
          send_neuron(c, cnts[i], 1, 1);    // count == thr-1 -> 0
          send_neuron(c, cnts[i], 1, -1);   // count == thr+1 -> 1 (thr clamped at 0)
        end
        send_neuron(c, 0, 2, 0);            // thr = 0 always fires (count 0)
        send_neuron(c, n, 2, n + 1);        // thr = N+1 never fires (count N)
        send_neuron(c, n, 2, n);            // thr = N, count N       -> 1
        send_neuron(c, n - 1, 2, n);        // thr = N, count N-1     -> 0
        send_neuron(c, n, 2, 1);            // thr = 1, count N       -> 1
        send_neuron(c, 0, 2, 1);            // thr = 1, count 0       -> 0
      end

      // Constrained random: random C in [1, NP_CMAX], mixed count and threshold modes.
      for (int r = 0; r < NP_RANDOM_NEURONS; r++) begin
        int c;
        c = $urandom_range(1, NP_CMAX);
        n = c * NP_PW;
        if ($urandom_range(3) == 0) send_neuron(c, -1, 0, 0);
        else send_neuron(c, rnd_target(n), 0, 0);
      end

      // Short tail so the last random neuron is followed by more contiguous data.
      for (int r = 0; r < 4; r++) send_neuron(1, -1, 0, 0);
    endtask
  endclass

  // ---- driver / monitor ----------------------------------------------------
  class np_driver extends uvm_driver #(np_item);
    `uvm_component_utils(np_driver)
    virtual np_if #(NP_PW, NP_AW, NP_TW) vif;
    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction
    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      if (!uvm_config_db#(virtual np_if #(NP_PW, NP_AW, NP_TW))::get(this, "", "vif", vif))
        `uvm_fatal("NOVIF", "np_if not set")
    endfunction
    task run_phase(uvm_phase phase);
      bit started;
      started = 0;
      forever begin
        seq_item_port.get_next_item(req);
        if (!started) begin
          // First cycle of the first neuron is on the bus from time 0.
          vif.first = req.first;
          vif.last  = req.last;
          vif.x     = req.x;
          vif.w     = req.w;
          vif.thr   = req.thr;
          started   = 1;
        end else begin
          @(posedge vif.clk);
          vif.first <= req.first;
          vif.last  <= req.last;
          vif.x     <= req.x;
          vif.w     <= req.w;
          vif.thr   <= req.thr;
        end
        seq_item_port.item_done();
      end
    endtask
  endclass

  class np_monitor extends uvm_monitor;
    `uvm_component_utils(np_monitor)
    virtual np_if #(NP_PW, NP_AW, NP_TW) vif;
    uvm_analysis_port #(np_txn) ap;
    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction
    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      ap = new("ap", this);
      if (!uvm_config_db#(virtual np_if #(NP_PW, NP_AW, NP_TW))::get(this, "", "vif", vif))
        `uvm_fatal("NOVIF", "np_if not set")
    endfunction
    task run_phase(uvm_phase phase);
      np_txn t;
      forever begin
        @(posedge vif.clk);
        t = np_txn::type_id::create("t");
        t.first     = vif.first;
        t.last      = vif.last;
        t.x         = vif.x;
        t.w         = vif.w;
        t.thr       = vif.thr;
        t.out_bit   = vif.out_bit;
        t.out_count = vif.out_count;
        ap.write(t);
      end
    endtask
  endclass

  // ---- scoreboard ------------------------------------------------------------
  class np_scb extends bnn_scb_base;
    `uvm_component_utils(np_scb)
    uvm_analysis_imp #(np_txn, np_scb) imp;

    // Expected result of a completed neuron, due at cycle `due`.
    typedef struct {
      int unsigned due;
      int unsigned count;
      bit          bit_exp;
      int unsigned id;
    } exp_t;

    exp_t        pend[$];
    bit          in_neuron;
    int unsigned acc;
    int unsigned cyc;
    int unsigned neurons;
    int unsigned cur_c;

    // coverage sample variables
    int unsigned cv_c;
    int unsigned cv_thr_class;
    int unsigned cv_rel;
    bit          cv_bit;

    covergroup cg;
      cp_c: coverpoint cv_c {
        bins c[] = {[1 : NP_CMAX]};
      }
      cp_thr: coverpoint cv_thr_class {
        bins zero  = {0};
        bins one   = {1};
        bins nm1   = {2};
        bins n     = {3};
        bins np1   = {4};
        bins other = {5};
      }
      // count relative to thr
      cp_rel: coverpoint cv_rel {
        bins count_lt_thr_m1 = {0};   // count <  thr-1
        bins count_eq_thr_m1 = {1};   // count == thr-1
        bins count_eq_thr    = {2};
        bins count_gt_thr    = {3};
      }
      cp_bit: coverpoint cv_bit {
        bins zero = {0};
        bins one  = {1};
      }
      x_rel_bit: cross cp_rel, cp_bit {
        ignore_bins impossible = binsof(cp_rel.count_lt_thr_m1) && binsof(cp_bit.one) ||
                                 binsof(cp_rel.count_eq_thr_m1) && binsof(cp_bit.one) ||
                                 binsof(cp_rel.count_eq_thr)    && binsof(cp_bit.zero) ||
                                 binsof(cp_rel.count_gt_thr)    && binsof(cp_bit.zero);
      }
    endgroup

    function new(string name, uvm_component parent);
      super.new(name, parent);
      imp = new("imp", this);
      cg = new();
      cyc = 0;
      neurons = 0;
      in_neuron = 0;
      acc = 0;
      cur_c = 0;
    endfunction

    function void write(np_txn t);
      exp_t e;
      int unsigned thr, n;

      // 1) check the neuron whose results are due at this edge
      if (pend.size() > 0 && pend[0].due == cyc) begin
        e = pend.pop_front();
        check_eq($sformatf("out_count[neuron=%0d cycle=%0d]", e.id, cyc), t.out_count, e.count);
        check_eq($sformatf("out_bit[neuron=%0d cycle=%0d]", e.id, cyc), t.out_bit, e.bit_exp);
      end

      // 2) track the neuron stream on the inputs
      if (t.first === 1'b1) begin
        in_neuron = 1;
        acc = 0;
        cur_c = 0;
      end
      if (in_neuron) begin
        for (int i = 0; i < NP_PW; i++) if (t.x[i] === t.w[i]) acc++;
        cur_c++;
        if (t.last === 1'b1) begin
          thr = t.thr;
          e.due     = cyc + NP_LATENCY;
          e.count   = acc;
          e.bit_exp = (acc >= thr);
          e.id      = neurons;
          pend.push_back(e);
          neurons++;
          in_neuron = 0;

          // coverage
          n = cur_c * NP_PW;
          cv_c = cur_c;
          if (thr == 0) cv_thr_class = 0;
          else if (thr == 1) cv_thr_class = 1;
          else if (thr == n - 1) cv_thr_class = 2;
          else if (thr == n) cv_thr_class = 3;
          else if (thr == n + 1) cv_thr_class = 4;
          else cv_thr_class = 5;
          if (acc + 1 < thr) cv_rel = 0;
          else if (acc + 1 == thr) cv_rel = 1;
          else if (acc == thr) cv_rel = 2;
          else cv_rel = 3;
          cv_bit = e.bit_exp;
          if (cur_c >= 1 && cur_c <= NP_CMAX) cg.sample();
        end
      end
      cyc++;
    endfunction
  endclass

  // ---- env / test --------------------------------------------------------------------
  class np_env extends uvm_env;
    `uvm_component_utils(np_env)
    uvm_sequencer #(np_item) sqr;
    np_driver  drv;
    np_monitor mon;
    np_scb     scb;
    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction
    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      sqr = uvm_sequencer #(np_item)::type_id::create("sqr", this);
      drv = np_driver::type_id::create("drv", this);
      mon = np_monitor::type_id::create("mon", this);
      scb = np_scb::type_id::create("np", this);
    endfunction
    function void connect_phase(uvm_phase phase);
      drv.seq_item_port.connect(sqr.seq_item_export);
      mon.ap.connect(scb.imp);
    endfunction
  endclass

  class np_random_test extends uvm_test;
    `uvm_component_utils(np_random_test)
    np_env env;
    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction
    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      env = np_env::type_id::create("env", this);
    endfunction
    task run_phase(uvm_phase phase);
      np_random_seq seq;
      phase.raise_objection(this);
      fork
        bnn_watchdog("np_random_test",
                     2 * ((NP_RANDOM_NEURONS + NP_DIRECTED_NEURONS + 8) * NP_CMAX + 200));
      join_none
      seq = np_random_seq::type_id::create("seq");
      seq.start(env.sqr);
      repeat (NP_LATENCY + 4) @(posedge env.drv.vif.clk);
      phase.drop_objection(this);
    endtask
  endclass

endpackage
