// UVM classes of the `layer` environment (DUT: bnn_layer, one computing layer).
//
// Structure
//   cfg agent   : sequencer + driver for the write bus (one item = one clock cycle
//                 of {cfg_mode, cfg_wr_*}). Sessions follow controller ruling R11:
//                 cfg_mode rises only while the layer is idle and no image is
//                 offered, the first write comes >= 1 cycle after cfg_mode rises,
//                 at most one write per cycle, cfg_mode stays high for >=
//                 CFG_FLUSH_CYCLES (16) cycles after the last cfg_wr_valid.
//                 Weight data in cfg_wr_data[PW-1:0], thresholds in
//                 cfg_wr_data[TW-1:0] (upper bits 0). Neurons >= M are never written
//                 (as in the real config manager), so padded NPs hold X in sim.
//   producer    : sequencer + driver writing binarized images into the upstream
//                 bnn_act_buf (claim -> C writes -> commit, commit may coincide with
//                 the last write, R10). Word c holds input bits c*PW .. c*PW+PW-1,
//                 positions >= N are 0.
//   sink        : hidden layers: consumer draining the downstream bnn_act_buf
//                 (fast / random gaps / long stalls), or an ideal sink
//                 (dn_ideal: out_can_claim held 1). Output layer: credit FIFO model
//                 driving out_can_claim (occupancy + reserved < depth; claim
//                 reserves, the image's res_last converts the reservation into
//                 occupancy, a random drain pops occupancy). out_can_claim only
//                 falls because of a claim, like a real act_buf / FIFO.
//   monitor     : samples every signal once per cycle (4-state), at the falling
//                 clock edge: every TB drive and every registered DUT output
//                 changes only right after the rising edge, so the negedge values
//                 are exactly what the next rising edge samples. (xsim does not
//                 order a class-based monitor consistently against other class
//                 processes woken by the same rising edge, so sampling at the
//                 rising edge sometimes sees post-NBA values.) txn.cyc = index of
//                 the rising edge that samples these values.
//   scoreboard  : port-level checks of the DUT against BNN_FCC_Model
//                 (create_random '{N, M, 1} for hidden psets / '{N, M} for output
//                 psets, layer 0, pixels bit ? 255 : 0):
//                   * out_claim only with out_can_claim && in_full && !cfg_mode
//                   * hidden: each group of each image written exactly once, data =
//                     expected word (bit p = out(g*PN+p), 0 for padded neurons),
//                     out_commit once per image after all G writes (may coincide
//                     with the last one); res_* always 0; downstream read-back of
//                     every word through the real act_buf
//                   * output: one res_valid per group in group order, res_mask,
//                     res_counts of the valid neurons, res_last on group G-1;
//                     out_wr_* / out_commit always 0
//                   * in_release once per image, in the same cycle as a read (R13:
//                     it coincides with the last read); control outputs never X/Z
//                   * idle == 0 while an image is in flight (claim+6 .. finish-1),
//                     idle == 1 after 16 quiet cycles
//                   * back-to-back: consecutive out_claim exactly C*G cycles apart
//                 Otherwise the input-side read pattern (in_rd_en / in_rd_addr) is
//                 not checked directly; only results are. in_full_next (R12) is
//                 the upstream act_buf's rd_full_next.
package layer_pkg;
  import uvm_pkg::*;
  `include "uvm_macros.svh"
  import bnn_unit_common_pkg::*;
  import bnn_ref_pkg::*;
  import bnn_fcc_tb_pkg::*;

  // ---- configuration (from the pset macros) -------------------------------
  localparam int L_LAYER_ID  = `PSET_LAYER_ID;
  localparam int L_N         = `PSET_N;
  localparam int L_M         = `PSET_M;
  localparam int L_PW        = `PSET_PW;
  localparam int L_PN        = `PSET_PN;
  localparam bit L_IS_OUTPUT = `PSET_IS_OUTPUT;
  localparam int L_RL        = `PSET_READ_LATENCY;
  localparam int L_NPW       = `PSET_WB_NP_W;
  localparam int L_AWB       = `PSET_WB_ADDR_W;
  localparam int L_DW        = `PSET_WB_DATA_W;
  localparam int L_C         = ref_ceil_div(L_N, L_PW);
  localparam int L_G         = ref_ceil_div(L_M, L_PN);
  localparam int L_CG        = L_C * L_G;
  localparam int L_CAW       = ref_addr_w(L_C);
  localparam int L_GAW       = ref_addr_w(L_G);
  localparam int L_AW        = ref_width_for(L_N);
  localparam int L_TW        = ref_width_for(L_N + 1);
  localparam int L_FLUSH     = REF_CFG_FLUSH_CYCLES;

  // In-flight idle check starts this many cycles after the claim.
  localparam int L_IDLE_LAG   = 6;
  // Quiet cycles after which idle must be 1.
  localparam int L_QUIET_IDLE = 16;

  typedef virtual layer_if #(L_NPW, L_AWB, L_DW, L_PW, L_PN, L_CAW, L_GAW, L_AW) layer_vif_t;
  typedef BNN_FCC_Model #(32) model_t;
  typedef logic [L_PW-1:0] l_word_t;
  typedef logic [L_PN-1:0] l_out_t;

  // Cfg item tags (coverage / documentation).
  typedef enum int {
    CFG_IDLE,        // cfg_mode = 0, no write
    CFG_FLUSH,       // cfg_mode = 1, cfg_wr_valid = 0, garbage on the bus
    CFG_REAL,        // real write for this layer
    CFG_WRONG_LAYER, // cfg_mode = 1, valid, cfg_wr_layer != LAYER_ID
    CFG_MODE0,       // cfg_mode = 0, cfg_wr_valid = 1 (must be ignored)
    CFG_OUT_THR      // output layer: kind = 1 write for LAYER_ID (no thresholds)
  } cfg_tag_e;

  function automatic bit [1023:0] rnd_bits();
    bit [1023:0] v;
    for (int i = 0; i < 1024; i += 32) v[i +: 32] = $urandom();
    return v;
  endfunction

  // ---- reference helpers -------------------------------------------------------
  class layer_ref;
    // New random model for this layer. Hidden psets: '{N, M, 1}, about 1/3 of the
    // thresholds replaced by 0, N, N+1, a negative value or a value > N+1 (both
    // clamped when loaded).
    static function model_t new_model();
      model_t m;
      int topo[];
      int r;
      m = new();
      if (L_IS_OUTPUT) topo = '{L_N, L_M};
      else topo = '{L_N, L_M, 1};
      m.create_random(topo);
      // xsim: std::randomize() of an unconstrained dynamic array (as done inside
      // create_random) resizes it to 0, leaving every weight row empty. Refill
      // any row whose size is not its fan-in with random bits.
      for (int l = 0; l < m.num_layers; l++) begin
        for (int n = 0; n < topo[l+1]; n++) begin
          if (m.weight[l][n].size() != topo[l]) begin
            m.weight[l][n] = new[topo[l]];
            for (int i = 0; i < topo[l]; i++) m.weight[l][n][i] = $urandom_range(1);
          end
        end
      end
      if (!L_IS_OUTPUT) begin
        for (int n = 0; n < L_M; n++) begin
          r = $urandom_range(14);
          case (r)
            0: m.threshold[0][n] = 0;
            1: m.threshold[0][n] = L_N;
            2: m.threshold[0][n] = L_N + 1;
            3: m.threshold[0][n] = -1 - int'($urandom_range(40));
            4: m.threshold[0][n] = L_N + 2 + int'($urandom_range(40));
            default: ;
          endcase
        end
      end
      return m;
    endfunction
  endclass

  // ---- items -------------------------------------------------------------------
  // One image: binarized inputs, upstream words, expected results, producer timing.
  class layer_img extends uvm_sequence_item;
    `uvm_object_utils(layer_img)
    int      id;
    int      phase;       // which model (coverage)
    int      kind;        // stimulus class 0..6 (coverage)
    bit      x[];         // N inputs
    l_word_t words[];     // C upstream words
    l_out_t  exp_word[];  // hidden: G expected output words
    int      exp_cnt[];   // output: M expected popcounts
    // producer timing
    int      pre_delay;
    int      order[];
    int      gap[];
    bit      commit_with_last;
    int      commit_delay;

    function new(string name = "layer_img");
      super.new(name);
    endfunction

    // kind: 0 random, 1 sparse, 2 dense, 3 all-0, 4 all-1,
    //       5 = weights of a random neuron (its popcount = N), 6 = complement (0)
    function void gen(int kind_i, model_t m);
      int nn;
      kind = kind_i;
      x = new[L_N];
      nn = $urandom_range(L_M - 1);
      for (int i = 0; i < L_N; i++) begin
        case (kind)
          1: x[i] = ($urandom_range(7) == 0);
          2: x[i] = ($urandom_range(7) != 0);
          3: x[i] = 1'b0;
          4: x[i] = 1'b1;
          5: x[i] = m.weight[0][nn][i];
          6: x[i] = ~m.weight[0][nn][i];
          default: x[i] = $urandom_range(1);
        endcase
      end
      words = new[L_C];
      for (int c = 0; c < L_C; c++) begin
        words[c] = '0;
        for (int j = 0; j < L_PW; j++)
          if (c * L_PW + j < L_N) words[c][j] = x[c * L_PW + j];
      end
      compute_expected(m);
    endfunction

    function void compute_expected(model_t m);
      bit [7:0] pixels[];
      int n;
      int row[];
      pixels = new[L_N];
      for (int i = 0; i < L_N; i++) pixels[i] = x[i] ? 8'd255 : 8'd0;
      void'(m.compute_reference(pixels));
      row = m.layer_outputs[0];
      if (L_IS_OUTPUT) begin
        exp_cnt = new[L_M];
        for (int i = 0; i < L_M; i++) exp_cnt[i] = row[i];
      end else begin
        exp_word = new[L_G];
        for (int g = 0; g < L_G; g++) begin
          exp_word[g] = '0;
          for (int p = 0; p < L_PN; p++) begin
            n = g * L_PN + p;
            if (n < L_M) exp_word[g][p] = (row[n] != 0);
          end
        end
      end
    endfunction

    // mode 0: as fast as possible (sequential writes, commit with the last one)
    // mode 1: random start delay, write order, gaps, commit timing
    function void gen_timing(int mode);
      order = new[L_C];
      gap = new[L_C];
      for (int c = 0; c < L_C; c++) begin
        order[c] = c;
        gap[c] = 0;
      end
      pre_delay = 0;
      commit_with_last = 1;
      commit_delay = 0;
      if (mode == 1) begin
        order.shuffle();
        foreach (gap[c]) if ($urandom_range(3) == 0) gap[c] = $urandom_range(1, 3);
        if ($urandom_range(1)) pre_delay = $urandom_range(0, L_CG + 4);
        commit_with_last = $urandom_range(1);
        commit_delay = $urandom_range(0, 4);
      end
    endfunction
  endclass

  // One clock cycle of the write bus.
  class layer_cfg_item extends uvm_sequence_item;
    `uvm_object_utils(layer_cfg_item)
    bit             mode;
    bit             valid;
    bit [      7:0] layer;
    bit             kind;
    bit [L_NPW-1:0] np;
    bit [L_AWB-1:0] addr;
    bit [ L_DW-1:0] data;
    cfg_tag_e       tag;
    function new(string name = "layer_cfg_item");
      super.new(name);
    endfunction
  endclass

  // ---- sequences -------------------------------------------------------------------
  // Write-bus stimulus. Build the cycle list with build_*() before start().
  class layer_cfg_seq extends uvm_sequence #(layer_cfg_item);
    `uvm_object_utils(layer_cfg_seq)
    layer_cfg_item items[$];
    bnn_layout     lay;

    function new(string name = "layer_cfg_seq");
      super.new(name);
      lay = new(L_N, L_M, L_PW, L_PN);
    endfunction

    function layer_cfg_item mk(bit mode, bit valid, bit [7:0] layer, bit kind, int np, int addr,
                               bit [L_DW-1:0] data, cfg_tag_e tag);
      layer_cfg_item it;
      it = layer_cfg_item::type_id::create("it");
      it.mode  = mode;
      it.valid = valid;
      it.layer = layer;
      it.kind  = kind;
      it.np    = np;
      it.addr  = addr;
      it.data  = data;
      it.tag   = tag;
      return it;
    endfunction

    // No write; random garbage on every other field (layer often == LAYER_ID).
    function layer_cfg_item mk_nowrite(bit mode);
      layer_cfg_item it;
      bit [1023:0] r;
      bit [7:0] lyr;
      bit kd;
      r = rnd_bits();
      lyr = $urandom_range(1) ? 8'(L_LAYER_ID) : 8'($urandom_range(255));
      kd = $urandom_range(1);
      it = mk(mode, 1'b0, lyr, kd, 0, 0, r[L_DW-1:0], mode ? CFG_FLUSH : CFG_IDLE);
      r = rnd_bits();
      it.np   = r[L_NPW-1:0];
      it.addr = r[L_NPW+L_AWB-1:L_NPW];
      return it;
    endfunction

    // A layer id != LAYER_ID: LAYER_ID with one bit flipped, or random.
    // (xsim: a single-statement `do ... while (...)` loop misbehaves; not used.)
    function bit [7:0] wrong_layer();
      bit [7:0] v;
      int b, t;
      if ($urandom_range(1)) begin
        b = $urandom_range(7);
        t = L_LAYER_ID ^ (1 << b);
      end else begin
        t = $urandom_range(255);
        if (t == L_LAYER_ID) t = t ^ 8'h80;
      end
      v = t[7:0];
      if (v == 8'(L_LAYER_ID)) `uvm_fatal("TB", "wrong_layer() returned LAYER_ID")
      return v;
    endfunction

    // (xsim: an element of a nested dynamic array (e.g. m.threshold[0][n]) that is
    // widened on assignment or argument passing (int -> longint) gets garbage upper
    // bits, so such elements are first copied into same-typed temporaries.)
    function bit [L_DW-1:0] wdata(model_t m, int n, int c);
      logic [1023:0] t;
      bit [L_DW-1:0] d;
      bit w[];
      w = m.weight[0][n];
      t = lay.pack_word(w, c);
      d = '0;
      d[L_PW-1:0] = t[L_PW-1:0];
      return d;
    endfunction

    function bit [L_DW-1:0] tdata(model_t m, int n);
      int v, ti;
      bit [L_DW-1:0] d;
      ti = m.threshold[0][n];   // int -> int first (xsim garbles a direct int -> longint)
      v = lay.clamp_thr(ti, L_N);
      d = '0;
      d[L_TW-1:0] = v;
      return d;
    endfunction

    // Every real write of model m for this layer (neurons < M only), shuffled.
    function void real_writes(model_t m, ref layer_cfg_item q[$]);
      int np, a;
      bit [L_DW-1:0] d;
      q.delete();
      for (int n = 0; n < L_M; n++) begin
        np = lay.np_of(n);
        for (int c = 0; c < L_C; c++) begin
          a = lay.waddr(n, c);
          d = wdata(m, n, c);
          q.push_back(mk(1'b1, 1'b1, 8'(L_LAYER_ID), 1'b0, np, a, d, CFG_REAL));
        end
        if (!L_IS_OUTPUT) begin
          a = lay.group_of(n);
          d = tdata(m, n);
          q.push_back(mk(1'b1, 1'b1, 8'(L_LAYER_ID), 1'b1, np, a, d, CFG_REAL));
        end
      end
      q.shuffle();
    endfunction

    // A copy of real write r whose data differs from the real value.
    function layer_cfg_item corrupt(layer_cfg_item r);
      layer_cfg_item it;
      bit [L_DW-1:0] d;
      int v;
      it = mk(r.mode, r.valid, r.layer, r.kind, r.np, r.addr, r.data, r.tag);
      d = '0;
      if (r.kind == 1'b0) begin
        d[L_PW-1:0] = ~r.data[L_PW-1:0];
      end else begin
        v = r.data[L_TW-1:0];
        v = (v + 1 + int'($urandom_range(L_N))) % (L_N + 2);
        d[L_TW-1:0] = v;
      end
      it.data = d;
      return it;
    endfunction

    // Output layer only: a kind = 1 write addressed to this layer (must not touch
    // the weights; the output layer has no thresholds).
    function layer_cfg_item out_thr_write();
      bit [L_DW-1:0] d;
      int np, a;
      d = '0;
      d[L_TW-1:0] = $urandom_range((1 << L_TW) - 1);
      np = $urandom_range(L_PN - 1);
      a = $urandom_range(L_G - 1);
      return mk(1'b1, 1'b1, 8'(L_LAYER_ID), 1'b1, np, a, d, CFG_OUT_THR);
    endfunction

    function void open_session();
      repeat ($urandom_range(1, 3)) items.push_back(mk_nowrite(1'b1));
    endfunction

    function void close_session();
      repeat (L_FLUSH + $urandom_range(0, 6)) items.push_back(mk_nowrite(1'b1));
      items.push_back(mk_nowrite(1'b0));
    endfunction

    // Full load of model m.
    //   noise       : random no-write cycles and wrong-layer writes between the real ones
    //   post_garbage: after all real writes, wrong-layer copies of ~1/2 of them with
    //                 corrupted data (+ kind=1 writes on the output layer)
    function void build_load(model_t m, bit noise, bit post_garbage);
      layer_cfg_item q[$];
      layer_cfg_item w;
      real_writes(m, q);
      open_session();
      foreach (q[i]) begin
        if (noise && $urandom_range(4) == 0) begin
          repeat ($urandom_range(1, 3)) begin
            if ($urandom_range(1)) items.push_back(mk_nowrite(1'b1));
            else begin
              w = corrupt(q[$urandom_range(q.size() - 1)]);
              w.layer = wrong_layer();
              w.tag   = CFG_WRONG_LAYER;
              items.push_back(w);
            end
          end
        end
        items.push_back(q[i]);
      end
      if (post_garbage) begin
        foreach (q[i]) begin
          if ($urandom_range(1)) begin
            w = corrupt(q[i]);
            w.layer = wrong_layer();
            w.tag   = CFG_WRONG_LAYER;
            items.push_back(w);
          end
          if (noise && $urandom_range(7) == 0) items.push_back(mk_nowrite(1'b1));
          if (L_IS_OUTPUT && $urandom_range(7) == 0) items.push_back(out_thr_write());
        end
      end
      close_session();
    endfunction

    // A cfg_mode session without a single real write: wrong-layer copies of every
    // real write with corrupted data, no-write garbage, and (output layer) kind=1
    // writes for this layer.
    function void build_garbage_session(model_t m);
      layer_cfg_item q[$];
      layer_cfg_item w;
      real_writes(m, q);
      open_session();
      foreach (q[i]) begin
        w = corrupt(q[i]);
        w.layer = wrong_layer();
        w.tag   = CFG_WRONG_LAYER;
        items.push_back(w);
        if ($urandom_range(3) == 0) items.push_back(mk_nowrite(1'b1));
        if (L_IS_OUTPUT && $urandom_range(3) == 0) items.push_back(out_thr_write());
      end
      close_session();
    endfunction

    // ncyc cycles with cfg_mode = 0: mostly valid writes for this layer with
    // corrupted data (must all be ignored), ends with an idle cycle.
    function void build_mode0_garbage(model_t m, int ncyc);
      layer_cfg_item q[$];
      layer_cfg_item w;
      real_writes(m, q);
      repeat (ncyc) begin
        if ($urandom_range(3) != 0) begin
          w = corrupt(q[$urandom_range(q.size() - 1)]);
          w.mode = 1'b0;
          if ($urandom_range(4) == 0) w.layer = wrong_layer();
          w.tag = CFG_MODE0;
          items.push_back(w);
        end else begin
          items.push_back(mk_nowrite(1'b0));
        end
      end
      items.push_back(mk_nowrite(1'b0));
    endfunction

    task body();
      foreach (items[i]) begin
        start_item(items[i]);
        finish_item(items[i]);
      end
    endtask
  endclass

  class layer_img_seq extends uvm_sequence #(layer_img);
    `uvm_object_utils(layer_img_seq)
    layer_img imgs[$];
    function new(string name = "layer_img_seq");
      super.new(name);
    endfunction
    task body();
      foreach (imgs[i]) begin
        start_item(imgs[i]);
        finish_item(imgs[i]);
      end
    endtask
  endclass

  // ---- drivers ---------------------------------------------------------------------
  class layer_cfg_driver extends uvm_driver #(layer_cfg_item);
    `uvm_component_utils(layer_cfg_driver)
    layer_vif_t vif;
    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction
    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      if (!uvm_config_db#(layer_vif_t)::get(this, "", "vif", vif)) `uvm_fatal("NOVIF", "layer_if not set")
    endfunction
    // One item per clock cycle; the bus holds the last item between sequences
    // (every sequence ends with a cfg_mode = 0, no-write item).
    task run_phase(uvm_phase phase);
      forever begin
        seq_item_port.get_next_item(req);
        @(posedge vif.clk);
        vif.cfg_mode     <= req.mode;
        vif.cfg_wr_valid <= req.valid;
        vif.cfg_wr_layer <= req.layer;
        vif.cfg_wr_kind  <= req.kind;
        vif.cfg_wr_np    <= req.np;
        vif.cfg_wr_addr  <= req.addr;
        vif.cfg_wr_data  <= req.data;
        seq_item_port.item_done();
      end
    endtask
  endclass

  // Writes images into the upstream act_buf. Every image is reported to the
  // scoreboard (ap) when its commit is driven.
  class layer_prod_driver extends uvm_driver #(layer_img);
    `uvm_component_utils(layer_prod_driver)
    layer_vif_t vif;
    uvm_analysis_port #(layer_img) ap;
    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction
    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      ap = new("ap", this);
      if (!uvm_config_db#(layer_vif_t)::get(this, "", "vif", vif)) `uvm_fatal("NOVIF", "layer_if not set")
    endfunction

    function void drive_idle();
      bit [1023:0] r;
      r = rnd_bits();
      vif.up_claim   <= 1'b0;
      vif.up_wr_en   <= 1'b0;
      vif.up_commit  <= 1'b0;
      vif.up_wr_addr <= r[L_CAW-1:0];
      vif.up_wr_data <= r[L_CAW+L_PW-1:L_CAW];
    endfunction

    task run_phase(uvm_phase phase);
      int a;
      forever begin
        seq_item_port.get_next_item(req);
        repeat (req.pre_delay) @(posedge vif.clk);
        // Claim as soon as wr_can_claim is sampled 1.
        forever begin
          @(posedge vif.clk);
          if (vif.up_can_claim === 1'b1) break;
        end
        vif.up_claim <= 1'b1;
        @(posedge vif.clk);   // claim sampled here; writes start the cycle after
        drive_idle();
        for (int k = 0; k < L_C; k++) begin
          repeat (req.gap[k]) begin
            @(posedge vif.clk);
            drive_idle();
          end
          a = req.order[k];
          vif.up_wr_en   <= 1'b1;
          vif.up_wr_addr <= a;
          vif.up_wr_data <= req.words[a];
          if (k == L_C - 1 && req.commit_with_last) begin
            vif.up_commit <= 1'b1;
            ap.write(req);
          end
          @(posedge vif.clk);
          drive_idle();
        end
        if (!req.commit_with_last) begin
          repeat (req.commit_delay) @(posedge vif.clk);
          vif.up_commit <= 1'b1;
          ap.write(req);
          @(posedge vif.clk);
          drive_idle();
        end
        seq_item_port.item_done();
      end
    endtask
  endclass

  // Downstream side: consumer of the downstream act_buf (hidden), or the
  // out_can_claim model (output-layer credit FIFO / ideal sink).
  //   mode 0: fast   1: random gaps   2: long random stalls
  class layer_sink extends uvm_component;
    `uvm_component_utils(layer_sink)
    layer_vif_t vif;
    int mode;
    bit ideal;
    int depth;        // credit FIFO depth (output layer)
    int stalls;       // number of long stalls taken (coverage / report)

    function new(string name, uvm_component parent);
      super.new(name, parent);
      mode = 1;
      ideal = 0;
      depth = 2;
      stalls = 0;
    endfunction
    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      if (!uvm_config_db#(layer_vif_t)::get(this, "", "vif", vif)) `uvm_fatal("NOVIF", "layer_if not set")
    endfunction

    function void drive_idle();
      bit [31:0] r;
      r = $urandom();
      vif.dn_rd_en   <= 1'b0;
      vif.dn_release <= 1'b0;
      vif.dn_rd_addr <= r[L_GAW-1:0];
    endfunction

    task run_phase(uvm_phase phase);
      if (L_IS_OUTPUT || ideal) credit_loop();
      else consume_loop();
    endtask

    // Samples at the falling edge (values the next rising edge samples), updates
    // out_can_claim right after that rising edge (registered behaviour).
    task credit_loop();
      int occ, res, stall_left;
      occ = 0;
      res = 0;
      stall_left = 0;
      forever begin
        @(negedge vif.clk);
        if (vif.rst !== 1'b0) begin
          occ = 0;
          res = 0;
        end else begin
          if (vif.out_claim === 1'b1) res++;
          if (vif.res_valid === 1'b1 && vif.res_last === 1'b1 && res > 0) begin
            res--;
            occ++;
          end
          if (stall_left > 0) begin
            stall_left--;
          end else if (occ > 0) begin
            case (mode)
              0: occ--;
              1: if ($urandom_range(1)) occ--;
              default: begin
                if ($urandom_range(1)) begin
                  occ--;
                  if ($urandom_range(2) == 0) begin
                    stall_left = $urandom_range(1, 4 * L_CG);
                    stalls++;
                  end
                end
              end
            endcase
          end
        end
        @(posedge vif.clk);
        vif.tb_can_claim <= ideal || (occ + res < depth);
      end
    endtask

    task consume_loop();
      int order[];
      int stall, gap;
      bit rel_with_last;
      order = new[L_G];
      drive_idle();
      forever begin
        forever begin
          @(posedge vif.clk);
          if (vif.dn_full === 1'b1) break;
        end
        foreach (order[i]) order[i] = i;
        stall = 0;
        rel_with_last = 1;
        if (mode != 0) begin
          order.shuffle();
          rel_with_last = $urandom_range(1);
          stall = $urandom_range(0, 3);
        end
        if (mode == 2 && $urandom_range(1)) begin
          stall = $urandom_range(1, 4 * L_CG);
          stalls++;
        end
        repeat (stall) begin
          drive_idle();
          @(posedge vif.clk);
        end
        for (int k = 0; k < L_G; k++) begin
          gap = 0;
          if (mode != 0 && $urandom_range(3) == 0) gap = $urandom_range(1, 3);
          repeat (gap) begin
            drive_idle();
            @(posedge vif.clk);
          end
          vif.dn_rd_en   <= 1'b1;
          vif.dn_rd_addr <= order[k];
          vif.dn_release <= (k == L_G - 1) && rel_with_last;
          @(posedge vif.clk);
        end
        drive_idle();
        if (!rel_with_last) begin
          repeat ($urandom_range(0, 3)) @(posedge vif.clk);
          vif.dn_release <= 1'b1;
          @(posedge vif.clk);
          drive_idle();
        end
      end
    endtask
  endclass

  // ---- monitor -----------------------------------------------------------------------
  class layer_txn extends uvm_sequence_item;
    `uvm_object_utils(layer_txn)
    int unsigned          cyc;
    logic                 rst;
    logic                 cfg_mode;
    logic                 cfg_wr_valid;
    logic [          7:0] cfg_wr_layer;
    logic                 cfg_wr_kind;
    logic [     L_DW-1:0] cfg_wr_data;
    logic                 in_full;
    logic                 in_full_next;
    logic                 in_rd_en;
    logic                 in_release;
    logic                 out_can_claim;
    logic                 out_claim;
    logic                 out_wr_en;
    logic [    L_GAW-1:0] out_wr_addr;
    logic [     L_PN-1:0] out_wr_data;
    logic                 out_commit;
    logic                 res_valid;
    logic                 res_last;
    logic [L_PN*L_AW-1:0] res_counts;
    logic [     L_PN-1:0] res_mask;
    logic                 idle;
    logic                 dn_rd_en;
    logic [    L_GAW-1:0] dn_rd_addr;
    logic [     L_PN-1:0] dn_rd_data;
    logic                 dn_release;
    logic                 dn_ideal;
    function new(string name = "layer_txn");
      super.new(name);
    endfunction
  endclass

  class layer_monitor extends uvm_monitor;
    `uvm_component_utils(layer_monitor)
    layer_vif_t vif;
    uvm_analysis_port #(layer_txn) ap;
    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction
    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      ap = new("ap", this);
      if (!uvm_config_db#(layer_vif_t)::get(this, "", "vif", vif)) `uvm_fatal("NOVIF", "layer_if not set")
    endfunction
    task run_phase(uvm_phase phase);
      layer_txn t;
      int unsigned cyc;
      cyc = 1;   // the first falling edge precedes rising edge 1
      forever begin
        @(negedge vif.clk);
        t = layer_txn::type_id::create("t");
        t.cyc           = cyc;
        t.rst           = vif.rst;
        t.cfg_mode      = vif.cfg_mode;
        t.cfg_wr_valid  = vif.cfg_wr_valid;
        t.cfg_wr_layer  = vif.cfg_wr_layer;
        t.cfg_wr_kind   = vif.cfg_wr_kind;
        t.cfg_wr_data   = vif.cfg_wr_data;
        t.in_full       = vif.in_full;
        t.in_full_next  = vif.in_full_next;
        t.in_rd_en      = vif.in_rd_en;
        t.in_release    = vif.in_release;
        t.out_can_claim = vif.out_can_claim;
        t.out_claim     = vif.out_claim;
        t.out_wr_en     = vif.out_wr_en;
        t.out_wr_addr   = vif.out_wr_addr;
        t.out_wr_data   = vif.out_wr_data;
        t.out_commit    = vif.out_commit;
        t.res_valid     = vif.res_valid;
        t.res_last      = vif.res_last;
        t.res_counts    = vif.res_counts;
        t.res_mask      = vif.res_mask;
        t.idle          = vif.idle;
        t.dn_rd_en      = vif.dn_rd_en;
        t.dn_rd_addr    = vif.dn_rd_addr;
        t.dn_rd_data    = vif.dn_rd_data;
        t.dn_release    = vif.dn_release;
        t.dn_ideal      = vif.dn_ideal;
        ap.write(t);
        cyc++;
      end
    endtask
  endclass

  // ---- scoreboard ----------------------------------------------------------------------
  `uvm_analysis_imp_decl(_txn)
  `uvm_analysis_imp_decl(_img)

  // A claimed image whose results are not complete.
  class layer_active;
    layer_img    img;
    int unsigned claim_cyc;
    bit          written[];
    int          nwritten;
    int          next_g;
    function new(layer_img img, int unsigned claim_cyc);
      this.img       = img;
      this.claim_cyc = claim_cyc;
      written        = new[L_G];
      nwritten       = 0;
      next_g         = 0;
    endfunction
  endclass

  class layer_scb extends bnn_scb_base;
    `uvm_component_utils(layer_scb)
    uvm_analysis_imp_txn #(layer_txn, layer_scb) txn_imp;
    uvm_analysis_imp_img #(layer_img, layer_scb) img_imp;

    typedef struct {
      int unsigned due;
      layer_img    img;
      int          addr;
    } rd_t;

    layer_img    sub_q[$];   // committed upstream, not yet claimed
    layer_active act_q[$];   // claimed, results incomplete
    layer_img    dn_q[$];    // hidden: committed downstream, not yet released by the consumer
    rd_t         rd_q[$];    // downstream reads awaiting data
    int unsigned claim_cycs[$];
    int          n_submitted;
    int          n_claims;
    int          n_finished;
    int          n_releases;
    int          tie0_viol;
    int          x_viol;
    int          quiet_cnt;
    int          blocked_cnt;  // cycles blocked by the downstream before the next claim
    int unsigned last_claim_cyc;
    bit          claimed_once;

    // back-to-back check (set by layer_back_to_back_test)
    bit check_b2b;
    int b2b_images;

    // coverage sample variables
    int cv_kind, cv_phase, cv_blocked, cv_spacing, cv_cw;
    int cv_tag, cv_thr;

    covergroup cg_img;
      cp_kind: coverpoint cv_kind {bins k[] = {[0 : 6]};}
      cp_phase: coverpoint cv_phase {bins p[] = {[0 : 1]};}
      cp_blocked: coverpoint cv_blocked {
        bins none = {0};
        bins short_ = {[1 : 10]};
        bins mid = {[11 : 100]};
        bins long_ = {[101 : $]};
      }
      // claim spacing relative to C*G: 0 = first, 1 = zero bubble, 2 = < 2CG, 3 = longer
      cp_spacing: coverpoint cv_spacing {bins s[] = {[0 : 3]};}
      // hidden: 1 = commit coincided with the last write
      cp_commit_with_write: coverpoint cv_cw {bins b[] = {[0 : 1]};}
    endgroup

    covergroup cg_cfg;
      cp_tag: coverpoint cv_tag {
        bins flush = {CFG_FLUSH};
        bins real_w = {CFG_REAL};
        bins wrong = {CFG_WRONG_LAYER};
        bins mode0 = {CFG_MODE0};
      }
      // real threshold writes: 0 = zero, 1 = N, 2 = N+1, 3 = other
      cp_thr: coverpoint cv_thr {bins t[] = {[0 : 3]};}
    endgroup

    function new(string name, uvm_component parent);
      super.new(name, parent);
      txn_imp = new("txn_imp", this);
      img_imp = new("img_imp", this);
      cg_img = new();
      cg_cfg = new();
      n_submitted = 0;
      n_claims = 0;
      n_finished = 0;
      n_releases = 0;
      tie0_viol = 0;
      x_viol = 0;
      quiet_cnt = 0;
      blocked_cnt = 0;
      claimed_once = 0;
      check_b2b = 0;
      b2b_images = 0;
    endfunction

    function void bad(string what);
      check_eq(what, 1, 0);
    endfunction

    function void write_img(layer_img img);
      sub_q.push_back(img);
      n_submitted++;
    endfunction

    function void check_known(string what, logic v, int unsigned cyc);
      if (v !== 1'b0 && v !== 1'b1) begin
        x_viol++;
        if (x_viol <= 5) bad($sformatf("%s is X/Z at cycle %0d", what, cyc));
      end
    endfunction

    function void finish_image(layer_active a, bit cw);
      n_finished++;
      cv_kind = a.img.kind;
      cv_phase = a.img.phase;
      cv_cw = cw;
      cg_img.sample();
    endfunction

    function void write_txn(layer_txn t);
      layer_active a;
      rd_t r;
      int g, n, addr;
      bit exp_mask;
      bit cw;
      // (xsim: array elements are copied into these before being passed to check_eq)
      logic [1023:0] got, exp;

      if (t.rst !== 1'b0) begin
        quiet_cnt = 0;
        return;
      end

      check_known("out_claim", t.out_claim, t.cyc);
      check_known("out_wr_en", t.out_wr_en, t.cyc);
      check_known("out_commit", t.out_commit, t.cyc);
      check_known("in_rd_en", t.in_rd_en, t.cyc);
      check_known("in_release", t.in_release, t.cyc);
      check_known("res_valid", t.res_valid, t.cyc);
      check_known("idle", t.idle, t.cyc);

      // ---- cfg coverage
      if (t.cfg_mode === 1'b1 || t.cfg_wr_valid === 1'b1) begin
        cv_thr = -1;
        if (t.cfg_mode === 1'b1 && t.cfg_wr_valid !== 1'b1) cv_tag = CFG_FLUSH;
        else if (t.cfg_mode !== 1'b1) cv_tag = CFG_MODE0;
        else if (t.cfg_wr_layer !== 8'(L_LAYER_ID)) cv_tag = CFG_WRONG_LAYER;
        else begin
          cv_tag = CFG_REAL;
          if (t.cfg_wr_kind === 1'b1 && !L_IS_OUTPUT) begin
            if (t.cfg_wr_data[L_TW-1:0] == 0) cv_thr = 0;
            else if (t.cfg_wr_data[L_TW-1:0] == L_N) cv_thr = 1;
            else if (t.cfg_wr_data[L_TW-1:0] == L_N + 1) cv_thr = 2;
            else cv_thr = 3;
          end
        end
        cg_cfg.sample();
      end

      // ---- downstream read-back data due at this edge
      while (rd_q.size() > 0 && rd_q[0].due == t.cyc) begin
        r = rd_q.pop_front();
        got = t.dn_rd_data;
        exp = r.img.exp_word[r.addr];
        check_eq($sformatf("downstream read img %0d word %0d", r.img.id, r.addr), got, exp);
      end

      // ---- claim
      if (t.in_full === 1'b1 && t.out_can_claim !== 1'b1 && act_q.size() == 0) blocked_cnt++;
      if (t.out_claim === 1'b1) begin
        check_eq($sformatf("claim at cycle %0d: out_can_claim", t.cyc), t.out_can_claim, 1);
        check_eq($sformatf("claim at cycle %0d: in_full", t.cyc), t.in_full, 1);
        check_eq($sformatf("claim at cycle %0d: cfg_mode", t.cyc), t.cfg_mode, 0);
        cv_blocked = blocked_cnt;
        blocked_cnt = 0;
        if (!claimed_once) cv_spacing = 0;
        else if (t.cyc - last_claim_cyc == L_CG) cv_spacing = 1;
        else if (t.cyc - last_claim_cyc < 2 * L_CG) cv_spacing = 2;
        else cv_spacing = 3;
        claimed_once = 1;
        last_claim_cyc = t.cyc;
        claim_cycs.push_back(t.cyc);
        n_claims++;
        if (sub_q.size() == 0) bad($sformatf("out_claim at cycle %0d without a submitted image", t.cyc));
        else begin
          a = new(sub_q.pop_front(), t.cyc);
          act_q.push_back(a);
        end
      end

      // ---- results
      if (!L_IS_OUTPUT) begin
        cw = 0;
        if (t.out_wr_en === 1'b1) begin
          if (act_q.size() == 0) bad($sformatf("out_wr_en at cycle %0d without a claimed image", t.cyc));
          else begin
            a = act_q[0];
            addr = t.out_wr_addr;
            if (^t.out_wr_addr === 1'bx || addr >= L_G)
              bad($sformatf("img %0d: out_wr_addr %0h out of range", a.img.id, t.out_wr_addr));
            else begin
              if (a.written[addr]) bad($sformatf("img %0d: group %0d written twice", a.img.id, addr));
              else begin
                a.written[addr] = 1;
                a.nwritten++;
              end
              got = t.out_wr_data;
              exp = a.img.exp_word[addr];
              check_eq($sformatf("img %0d group %0d out_wr_data", a.img.id, addr), got, exp);
            end
            cw = 1;
          end
        end
        if (t.out_commit === 1'b1) begin
          if (act_q.size() == 0) bad($sformatf("out_commit at cycle %0d without a claimed image", t.cyc));
          else begin
            a = act_q.pop_front();
            check_eq($sformatf("img %0d: groups written at commit", a.img.id), a.nwritten, L_G);
            finish_image(a, cw);
            if (t.dn_ideal !== 1'b1) dn_q.push_back(a.img);
          end
        end
        if (t.res_valid !== 1'b0 || t.res_last !== 1'b0 || t.res_counts !== '0 || t.res_mask !== '0) begin
          tie0_viol++;
          if (tie0_viol <= 5) bad($sformatf("hidden layer: res_* not 0 at cycle %0d", t.cyc));
        end
      end else begin
        if (t.res_valid === 1'b1) begin
          if (act_q.size() == 0) bad($sformatf("res_valid at cycle %0d without a claimed image", t.cyc));
          else begin
            a = act_q[0];
            g = a.next_g;
            for (int p = 0; p < L_PN; p++) begin
              n = g * L_PN + p;
              exp_mask = (n < L_M);
              got = t.res_mask[p];
              exp = exp_mask;
              check_eq($sformatf("img %0d group %0d res_mask[%0d]", a.img.id, g, p), got, exp);
              if (exp_mask) begin
                got = t.res_counts[p*L_AW+:L_AW];
                exp = a.img.exp_cnt[n];
                check_eq($sformatf("img %0d neuron %0d res_counts", a.img.id, n), got, exp);
              end
            end
            check_eq($sformatf("img %0d group %0d res_last", a.img.id, g), t.res_last, (g == L_G - 1));
            a.next_g++;
            if (a.next_g == L_G) begin
              void'(act_q.pop_front());
              finish_image(a, 0);
            end
          end
        end
        if (t.out_wr_en !== 1'b0 || t.out_commit !== 1'b0 || t.out_wr_addr !== '0 || t.out_wr_data !== '0) begin
          tie0_viol++;
          if (tie0_viol <= 5) bad($sformatf("output layer: out_wr_*/out_commit not 0 at cycle %0d", t.cyc));
        end
      end

      // ---- input release
      if (t.in_release === 1'b1) begin
        // R13: in_release coincides with the image's last read
        if (t.in_rd_en !== 1'b1) bad($sformatf("in_release at cycle %0d without a read in the same cycle", t.cyc));
        n_releases++;
        if (n_releases > n_claims) bad($sformatf("in_release at cycle %0d without a started image", t.cyc));
      end

      // ---- downstream consumer (hidden, real downstream act_buf)
      if (!L_IS_OUTPUT && t.dn_ideal !== 1'b1) begin
        if (t.dn_rd_en === 1'b1) begin
          if (dn_q.size() == 0) bad($sformatf("downstream read at cycle %0d without a committed image", t.cyc));
          else begin
            r.due  = t.cyc + L_RL;
            r.img  = dn_q[0];
            r.addr = t.dn_rd_addr;
            rd_q.push_back(r);
          end
        end
        if (t.dn_release === 1'b1 && dn_q.size() > 0) void'(dn_q.pop_front());
      end

      // ---- idle
      if (act_q.size() > 0 && t.cyc >= act_q[0].claim_cyc + L_IDLE_LAG)
        check_eq($sformatf("idle while img %0d in flight (cycle %0d)", act_q[0].img.id, t.cyc), t.idle, 0);
      if (act_q.size() == 0 && t.in_full === 1'b0 && t.cfg_mode === 1'b0 && t.cfg_wr_valid === 1'b0 &&
          n_releases == n_claims && t.out_claim !== 1'b1) begin
        quiet_cnt++;
        if (quiet_cnt == L_QUIET_IDLE)
          check_eq($sformatf("idle after %0d quiet cycles (cycle %0d)", L_QUIET_IDLE, t.cyc), t.idle, 1);
      end else begin
        quiet_cnt = 0;
      end
    endfunction

    function void check_phase(uvm_phase phase);
      int nb, ns, na, nd, nr, sp;
      super.check_phase(phase);
      ns = sub_q.size();
      na = act_q.size();
      nd = dn_q.size();
      nr = rd_q.size();
      nb = claim_cycs.size();
      check_eq("images never claimed", ns, 0);
      check_eq("images with incomplete results", na, 0);
      check_eq("finished images vs submitted", n_finished, n_submitted);
      check_eq("in_release pulses vs claims", n_releases, n_claims);
      check_eq("claims vs submitted images", n_claims, n_submitted);
      check_eq("tied-0 output violations", tie0_viol, 0);
      check_eq("X/Z control outputs", x_viol, 0);
      if (!L_IS_OUTPUT) begin
        check_eq("downstream images not drained", nd, 0);
        check_eq("downstream reads pending", nr, 0);
      end
      if (check_b2b) begin
        check_eq("back-to-back: number of out_claim pulses", nb, b2b_images);
        for (int k = 1; k < b2b_images; k++) begin
          if (k < nb) begin
            sp = claim_cycs[k] - claim_cycs[k-1];
            check_eq($sformatf("back-to-back: cycles between out_claim %0d and %0d", k, k + 1), sp, L_CG);
          end else bad($sformatf("back-to-back: out_claim %0d missing", k + 1));
        end
      end
    endfunction

    function void report_phase(uvm_phase phase);
      `uvm_info("COV", $sformatf("layer coverage: images %0.1f%%, cfg %0.1f%% (claims=%0d finished=%0d)",
                                 cg_img.get_inst_coverage(), cg_cfg.get_inst_coverage(), n_claims,
                                 n_finished), UVM_NONE)
      super.report_phase(phase);
    endfunction
  endclass

  // ---- env ---------------------------------------------------------------------------
  class layer_env extends uvm_env;
    `uvm_component_utils(layer_env)
    uvm_sequencer #(layer_cfg_item) cfg_sqr;
    layer_cfg_driver                cfg_drv;
    uvm_sequencer #(layer_img)      prod_sqr;
    layer_prod_driver               prod_drv;
    layer_sink                      sink;
    layer_monitor                   mon;
    layer_scb                       scb;
    layer_vif_t                     vif;

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      if (!uvm_config_db#(layer_vif_t)::get(this, "", "vif", vif)) `uvm_fatal("NOVIF", "layer_if not set")
      if (L_NPW < ref_addr_w(L_PN) || L_AWB < ref_addr_w(L_CG) || L_DW < L_PW || L_DW < L_TW)
        `uvm_fatal("PSET", "WB_NP_W / WB_ADDR_W / WB_DATA_W too small for this pset")
      cfg_sqr  = uvm_sequencer#(layer_cfg_item)::type_id::create("cfg_sqr", this);
      cfg_drv  = layer_cfg_driver::type_id::create("cfg_drv", this);
      prod_sqr = uvm_sequencer#(layer_img)::type_id::create("prod_sqr", this);
      prod_drv = layer_prod_driver::type_id::create("prod_drv", this);
      sink     = layer_sink::type_id::create("sink", this);
      mon      = layer_monitor::type_id::create("mon", this);
      scb      = layer_scb::type_id::create("layer", this);
    endfunction

    function void connect_phase(uvm_phase phase);
      cfg_drv.seq_item_port.connect(cfg_sqr.seq_item_export);
      prod_drv.seq_item_port.connect(prod_sqr.seq_item_export);
      mon.ap.connect(scb.txn_imp);
      prod_drv.ap.connect(scb.img_imp);
    endfunction
  endclass

  // ---- tests ---------------------------------------------------------------------------
  class layer_base_test extends uvm_test;
    `uvm_component_utils(layer_base_test)
    layer_env   env;
    layer_vif_t vif;
    int         img_id;
    int         phase_id;

    function new(string name, uvm_component parent);
      super.new(name, parent);
      img_id = 0;
      phase_id = 0;
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      env = layer_env::type_id::create("env", this);
      if (!uvm_config_db#(layer_vif_t)::get(this, "", "vif", vif)) `uvm_fatal("NOVIF", "layer_if not set")
    endfunction

    function void start_of_simulation_phase(uvm_phase phase);
      `uvm_info("PSET", $sformatf("%s: LAYER_ID=%0d N=%0d M=%0d PW=%0d PN=%0d IS_OUTPUT=%0d C=%0d G=%0d RL=%0d",
                                  `PSET_NAME, L_LAYER_ID, L_N, L_M, L_PW, L_PN, L_IS_OUTPUT, L_C, L_G, L_RL),
                UVM_NONE)
    endfunction

    // Cycle budget of one cfg session (plus noise and garbage).
    function longint cfg_cycles();
      return 3 * (L_M * (L_C + 1)) + 200;
    endfunction

    // Watchdog for `cycles` clock cycles (with margin).
    // (1 cycle = 1.11 ns, so this is ~1.8x the estimate; the estimates
    // themselves are ~3x the reference-DUT run time.)
    function time budget(longint cycles);
      return time'(2 * cycles + 5000);
    endfunction

    // Fork the watchdog for an estimated `cycles` clock cycles.
    function void start_watchdog(string name, longint cycles);
      time b;
      b = budget(cycles);
      `uvm_info("WDOG", $sformatf("%s: estimate %0d cycles, watchdog %0d ns", name, cycles, b), UVM_NONE)
      fork
        bnn_watchdog(name, b);
      join_none
    endfunction

    // Knobs of the sink / scoreboard; set before any run_phase starts.
    virtual function void configure();
    endfunction

    function void end_of_elaboration_phase(uvm_phase phase);
      super.end_of_elaboration_phase(phase);
      configure();
    endfunction

    task do_reset();
      vif.rst <= 1'b1;
      repeat (32) @(posedge vif.clk);
      vif.rst <= 1'b0;
      repeat (16) @(posedge vif.clk);
    endtask

    task load(model_t m, bit noise, bit post_garbage);
      layer_cfg_seq s;
      s = layer_cfg_seq::type_id::create("cfg_load");
      s.build_load(m, noise, post_garbage);
      s.start(env.cfg_sqr);
    endtask

    task garbage_session(model_t m);
      layer_cfg_seq s;
      s = layer_cfg_seq::type_id::create("cfg_garbage");
      s.build_garbage_session(m);
      s.start(env.cfg_sqr);
    endtask

    task mode0_garbage(model_t m, int ncyc);
      layer_cfg_seq s;
      s = layer_cfg_seq::type_id::create("cfg_mode0");
      s.build_mode0_garbage(m, ncyc);
      s.start(env.cfg_sqr);
    endtask

    // n images from model m; prod_mode 0 = fast, 1 = random timing.
    task run_images(model_t m, int n, int prod_mode);
      layer_img_seq s;
      layer_img im;
      int k;
      s = layer_img_seq::type_id::create("imgs");
      for (int i = 0; i < n; i++) begin
        im = layer_img::type_id::create($sformatf("img%0d", img_id));
        im.id = img_id++;
        im.phase = phase_id;
        // first images of a run cover the directed kinds, then mostly random
        if (i < 7) k = (i + 3) % 7;
        else k = ($urandom_range(2) == 0) ? $urandom_range(6) : 0;
        im.gen(k, m);
        im.gen_timing(prod_mode);
        s.imgs.push_back(im);
      end
      s.start(env.prod_sqr);
    endtask

    // Wait until every submitted image has been processed and released (and, for
    // hidden layers, drained downstream), then a few quiet cycles.
    task wait_done();
      forever begin
        @(posedge vif.clk);
        if (env.scb.n_finished == env.scb.n_submitted && env.scb.n_releases == env.scb.n_claims &&
            env.scb.n_claims == env.scb.n_submitted && env.scb.dn_q.size() == 0 &&
            env.scb.rd_q.size() == 0)
          break;
      end
      repeat (L_QUIET_IDLE + 8) @(posedge vif.clk);
    endtask
  endclass

  // 30 images, random producer and consumer timing, model reloaded after 15.
  class layer_random_test extends layer_base_test;
    `uvm_component_utils(layer_random_test)
    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction
    function void configure();
      env.sink.mode  = 1;
      env.sink.depth = $urandom_range(1, 4);
    endfunction
    task run_phase(uvm_phase phase);
      model_t m;
      phase.raise_objection(this);
      start_watchdog("layer_random_test", 2 * cfg_cycles() + 30 * (2 * L_CG + 6 * L_C + 60));
      do_reset();
      for (int ph = 0; ph < 2; ph++) begin
        phase_id = ph;
        m = layer_ref::new_model();
        load(m, 1, 0);
        run_images(m, 15, 1);
        wait_done();
      end
      phase.drop_objection(this);
    endtask
  endclass

  // 10 images, upstream kept full, ideal downstream sink: out_claim pulses exactly
  // C*G cycles apart.
  class layer_back_to_back_test extends layer_base_test;
    `uvm_component_utils(layer_back_to_back_test)
    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction
    function void configure();
      env.sink.ideal     = 1;
      env.sink.mode      = 0;
      vif.dn_ideal       = 1'b1;
      vif.tb_can_claim   = 1'b1;
      env.scb.check_b2b  = 1;
      env.scb.b2b_images = 10;
    endfunction
    task run_phase(uvm_phase phase);
      model_t m;
      phase.raise_objection(this);
      start_watchdog("layer_back_to_back_test", cfg_cycles() + 10 * (2 * L_CG + 2 * L_C + 60));
      do_reset();
      m = layer_ref::new_model();
      load(m, 0, 0);
      run_images(m, 10, 0);
      wait_done();
      phase.drop_objection(this);
    endtask
  endclass

  // Downstream stalls for long random periods while the upstream is kept full.
  class layer_backpressure_test extends layer_base_test;
    `uvm_component_utils(layer_backpressure_test)
    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction
    function void configure();
      env.sink.mode  = 2;
      env.sink.depth = $urandom_range(1, 2);
    endfunction
    task run_phase(uvm_phase phase);
      model_t m;
      phase.raise_objection(this);
      start_watchdog("layer_backpressure_test", cfg_cycles() + 20 * (6 * L_CG + 4 * L_C + 80));
      do_reset();
      m = layer_ref::new_model();
      load(m, 1, 0);
      run_images(m, 20, 0);
      wait_done();
      if (env.sink.stalls == 0) `uvm_error("TB", "backpressure test took no downstream stall")
      phase.drop_objection(this);
    endtask
  endclass

  // Writes for other layers, writes with cfg_wr_valid = 0 and writes while
  // cfg_mode = 0 must not change any result.
  class layer_cfg_isolation_test extends layer_base_test;
    `uvm_component_utils(layer_cfg_isolation_test)
    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction
    function void configure();
      env.sink.mode  = 1;
      env.sink.depth = $urandom_range(1, 4);
    endfunction
    task run_phase(uvm_phase phase);
      model_t m;
      phase.raise_objection(this);
      start_watchdog("layer_cfg_isolation_test", 3 * cfg_cycles() + 20 * (3 * L_CG + 6 * L_C + 80));
      do_reset();
      m = layer_ref::new_model();
      // real load followed by wrong-layer copies with corrupted data
      load(m, 1, 1);
      // images while cfg_mode = 0 writes with corrupted data hit the bus
      fork
        mode0_garbage(m, 10 * L_CG / 2 + 64);
        run_images(m, 10, 1);
      join
      wait_done();
      // a whole cfg_mode session containing only writes that must be ignored
      garbage_session(m);
      fork
        mode0_garbage(m, 10 * L_CG / 2 + 64);
        run_images(m, 10, 1);
      join
      wait_done();
      phase.drop_objection(this);
    endtask
  endclass

endpackage
