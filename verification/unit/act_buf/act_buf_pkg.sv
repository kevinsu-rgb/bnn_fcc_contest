// UVM classes of the `act_buf` environment (DUT: bnn_act_buf).
//
// Two slots, each EMPTY -> CLAIMED -> FULL -> EMPTY, with claim / write /
// read pointers that alternate 0,1,0,1...
//   wr_can_claim = slot[claim_ptr] EMPTY      wr_claim  : slot[claim_ptr] -> CLAIMED
//   wr_en/commit apply to slot[write_ptr]     wr_commit : slot[write_ptr] -> FULL
//   rd_full      = slot[read_ptr]  FULL       rd_release: slot[read_ptr]  -> EMPTY
//   rd_full_next = slot[read_ptr+1] FULL (look-ahead, ruling R12)
//   idle         = no slot FULL
// All three status outputs are registered: an event sampled by the DUT at
// clock edge k is visible in the values sampled at edge k+1. rd_data
// follows rd_en/rd_addr by READ_LATENCY cycles (read at edge k -> data sampled
// at edge k+READ_LATENCY).
//
// Structure: one agent (sequencer, driver, monitor). The stimulus sequence
// steps a producer and a consumer behaviour object once per clock; both
// decide from a reference slot model (never from the DUT's outputs), so a
// broken DUT cannot hang the test - it just fails the scoreboard. The
// scoreboard owns an independent model fed from the monitor and checks
// wr_can_claim / rd_full / idle every cycle and every rd_data at its latency.
// The protocol monitor (own model) is a self-check of the TB itself.
//
// TB conservatism (outside the stated contract, so never exercised):
//   * no wr_en in the cycle of the wr_claim of its slot. (Controller ruling R10:
//     wr_commit MAY coincide with the image's final wr_en, and the producer
//     does so for ~50% of the images; the write lands in the committing slot.)
//   * images are fully written (every address at least once) before commit.
//   * at least one read per image; addresses always < DEPTH.
package act_buf_pkg;
  import uvm_pkg::*;
  `include "uvm_macros.svh"
  import bnn_unit_common_pkg::*;
  import bnn_ref_pkg::*;

  // ---- configuration (from the pset macros) -------------------------------
  localparam int AB_WIDTH = `PSET_WIDTH;
  localparam int AB_DEPTH = `PSET_DEPTH;
  localparam int AB_LAT   = `PSET_READ_LATENCY;
  localparam int AB_AW    = ref_addr_w(AB_DEPTH);

  typedef bit [AB_WIDTH-1:0] ab_data_t;
  typedef bit [AB_AW-1:0]    ab_addr_t;
  typedef enum int {SL_EMPTY, SL_CLAIMED, SL_FULL} slot_st_e;

  // Per-cycle DUT inputs (excluding rst).
  typedef struct {
    bit       claim;
    bit       wr_en;
    ab_addr_t wr_addr;
    ab_data_t wr_data;
    bit       commit;
    bit       rd_en;
    ab_addr_t rd_addr;
    bit       rel;
  } ab_in_t;

  function automatic ab_data_t ab_rnd_data();
    ab_data_t d;
    d = '0;
    for (int i = 0; i < AB_WIDTH; i += 32) d |= ab_data_t'($urandom()) << i;
    return d;
  endfunction

  function automatic ab_addr_t ab_rnd_addr();
    return ab_addr_t'($urandom_range(AB_DEPTH - 1));
  endfunction

  // ---- reference slot model ------------------------------------------------
  class actbuf_model;
    slot_st_e st[2];
    int       cp, wp, rp;
    ab_data_t mem[2][];
    bit       wrote[2][];
    int       nwrote[2];

    function new();
      mem[0]   = new[AB_DEPTH];
      mem[1]   = new[AB_DEPTH];
      wrote[0] = new[AB_DEPTH];
      wrote[1] = new[AB_DEPTH];
      reset();
    endfunction

    function void reset();
      st[0] = SL_EMPTY;
      st[1] = SL_EMPTY;
      cp = 0;
      wp = 0;
      rp = 0;
      for (int s = 0; s < 2; s++) begin
        nwrote[s] = 0;
        for (int a = 0; a < AB_DEPTH; a++) wrote[s][a] = 1'b0;
      end
    endfunction

    // Expected (registered) status outputs for the state *before* an edge.
    function bit exp_can_claim();
      return st[cp] == SL_EMPTY;
    endfunction
    function bit exp_full();
      return st[rp] == SL_FULL;
    endfunction
    function bit exp_full_next();
      return st[1 - rp] == SL_FULL;
    endfunction
    function bit exp_idle();
      return !(st[0] == SL_FULL || st[1] == SL_FULL);
    endfunction

    // Data a read at the current state returns.
    function ab_data_t rd_value(ab_addr_t a);
      return mem[rp][int'(a)];
    endfunction

    // Empty string if the inputs are legal in the current state.
    function string violation(ab_in_t i);
      int nw;
      if (i.claim && st[cp] != SL_EMPTY) return "wr_claim without wr_can_claim";
      if (i.wr_en) begin
        if (st[wp] != SL_CLAIMED) return "wr_en with no CLAIMED slot";
        if (int'(i.wr_addr) >= AB_DEPTH) return "wr_addr out of range";
      end
      if (i.commit) begin
        if (st[wp] != SL_CLAIMED) return "wr_commit with no CLAIMED slot";
        // A write coinciding with the commit lands in the committing slot.
        nw = nwrote[wp];
        if (i.wr_en && int'(i.wr_addr) < AB_DEPTH && !wrote[wp][int'(i.wr_addr)]) nw++;
        if (nw < AB_DEPTH) return "wr_commit of an incompletely written image";
      end
      if (i.rd_en) begin
        if (st[rp] != SL_FULL) return "rd_en without rd_full";
        if (int'(i.rd_addr) >= AB_DEPTH) return "rd_addr out of range";
      end
      if (i.rel && st[rp] != SL_FULL) return "rd_release without rd_full";
      return "";
    endfunction

    // Apply one clock edge (all decisions use the pre-edge state).
    function void step(ab_in_t i);
      int cs, ws, rs;
      cs = cp;
      ws = wp;
      rs = rp;
      if (i.wr_en) begin
        mem[ws][int'(i.wr_addr)] = i.wr_data;
        if (!wrote[ws][int'(i.wr_addr)]) begin
          wrote[ws][int'(i.wr_addr)] = 1'b1;
          nwrote[ws]++;
        end
      end
      if (i.commit) begin
        st[ws] = SL_FULL;
        wp = 1 - ws;
      end
      if (i.claim) begin
        st[cs] = SL_CLAIMED;
        cp = 1 - cs;
        nwrote[cs] = 0;
        for (int a = 0; a < AB_DEPTH; a++) wrote[cs][a] = 1'b0;
      end
      if (i.rel) begin
        st[rs] = SL_EMPTY;
        rp = 1 - rs;
      end
    endfunction
  endclass

  // ---- items -----------------------------------------------------------------
  // One clock cycle of stimulus.
  class actbuf_item extends uvm_sequence_item;
    `uvm_object_utils(actbuf_item)
    bit     rst;
    ab_in_t i;
    function new(string name = "actbuf_item");
      super.new(name);
    endfunction
  endclass

  // One observed clock cycle: inputs sampled at the edge, outputs as seen at
  // the same edge (i.e. produced by earlier inputs).
  class actbuf_txn extends uvm_sequence_item;
    `uvm_object_utils(actbuf_txn)
    bit                 rst;
    ab_in_t             i;
    logic               wr_can_claim;
    logic               rd_full;
    logic               rd_full_next;
    logic               idle;
    logic [AB_WIDTH-1:0] rd_data;
    function new(string name = "actbuf_txn");
      super.new(name);
    endfunction
  endclass

  // ---- producer / consumer behaviours ---------------------------------------
  class actbuf_job;
    int       slot;
    ab_addr_t order[$];
    int       pos;
    int       cwait;
    int       p_write;
  endclass

  function automatic int ab_pick(int a, int b, int c, int d);
    int r;
    r = $urandom_range(3);
    case (r)
      0: return a;
      1: return b;
      2: return c;
      default: return d;
    endcase
  endfunction

  class actbuf_prod;
    actbuf_job jobs[$];
    int        total;
    int        started;      // claims issued
    int        committed;
    int        claim_wait;
    bit        fast;         // no gaps, claim ahead eagerly
    int        p_ahead;

    function new(int total, bit fast);
      this.total      = total;
      this.fast       = fast;
      this.started    = 0;
      this.committed  = 0;
      this.claim_wait = 0;
      this.p_ahead    = fast ? 100 : 50;
    endfunction

    function bit done();
      return (committed >= total) && (jobs.size() == 0);
    endfunction

    function bit writing_midway();
      if (jobs.size() == 0) return 0;
      return jobs[0].pos > 0 && jobs[0].pos < jobs[0].order.size();
    endfunction

    function actbuf_job make_job(int slot);
      actbuf_job j;
      ab_addr_t  perm[$];
      int        n_extra, at;
      j = new();
      j.slot = slot;
      j.pos  = 0;
      for (int a = 0; a < AB_DEPTH; a++) perm.push_back(ab_addr_t'(a));
      perm.shuffle();
      n_extra = ($urandom_range(99) < 30) ? $urandom_range(1, 3) : 0;
      for (int e = 0; e < n_extra; e++) begin
        at = $urandom_range(perm.size());
        perm.insert(at, ab_rnd_addr());
      end
      j.order   = perm;
      j.p_write = fast ? 100 : ab_pick(100, 80, 50, 20);
      j.cwait   = 0;
      return j;
    endfunction

    function void step(actbuf_model m, ref ab_in_t it);
      actbuf_job j;
      bit        can, want;
      // Writes / commit of the oldest claimed image.
      if (jobs.size() > 0) begin
        j = jobs[0];
        if (m.st[j.slot] == SL_CLAIMED && m.wp == j.slot) begin
          if (j.pos < j.order.size()) begin
            if ($urandom_range(99) < j.p_write) begin
              it.wr_en   = 1'b1;
              it.wr_addr = j.order[j.pos];
              it.wr_data = ab_rnd_data();
              j.pos++;
              if (j.pos == j.order.size()) begin
                // ~50%: commit in the same cycle as the last write (R10);
                // otherwise 1..N cycles later (cwait counts the extra gap).
                if ($urandom_range(1) == 0) begin
                  it.commit = 1'b1;
                  void'(jobs.pop_front());
                  committed++;
                end else begin
                  j.cwait = fast ? 0 : ab_pick(0, 0, 1, 5);
                end
              end
            end
          end else begin
            if (j.cwait > 0) j.cwait--;
            else begin
              it.commit = 1'b1;
              void'(jobs.pop_front());
              committed++;
            end
          end
        end
      end
      // Claim (the slot is only writable from the next cycle on).
      can = m.exp_can_claim();
      if (started < total && can) begin
        if (claim_wait > 0) claim_wait--;
        else begin
          want = (jobs.size() == 0) || ($urandom_range(99) < p_ahead);
          if (want) begin
            it.claim = 1'b1;
            jobs.push_back(make_job(m.cp));
            started++;
            claim_wait = fast ? 0 : ab_pick(0, 0, 3, 12);
          end
        end
      end
    endfunction
  endclass

  class actbuf_cons;
    int       total;
    int       done_cnt;
    bit       zb;                // zero-bubble discipline
    bit       in_img;
    bit       relwait_phase;
    int       rel_wait;
    int       img_wait;
    int       p_read;
    int       reads_issued;
    ab_addr_t raddrs[$];
    int       pos;
    int       cyc;
    int       last_rel_cyc;
    int       zb_count;          // images whose first read followed a release by 1 cycle
    int       zb_possible;       // releases that had the next slot FULL (zb mode)
    bit       first_read_pending;

    function new(int total, bit zb);
      this.total        = total;
      this.zb           = zb;
      done_cnt          = 0;
      in_img            = 0;
      relwait_phase     = 0;
      img_wait          = 0;
      rel_wait          = 0;
      p_read            = 100;
      reads_issued      = 0;
      pos               = 0;
      cyc               = 0;
      last_rel_cyc      = -100;
      zb_count          = 0;
      zb_possible       = 0;
      first_read_pending = 0;
    endfunction

    function bit done();
      return done_cnt >= total;
    endfunction

    function void begin_image();
      ab_addr_t perm[$];
      int       n;
      raddrs.delete();
      if (zb ? ($urandom_range(99) < 85) : ($urandom_range(99) < 50)) begin
        for (int a = 0; a < AB_DEPTH; a++) perm.push_back(ab_addr_t'(a));
        perm.shuffle();
        raddrs = perm;
      end else begin
        n = zb ? $urandom_range((AB_DEPTH > 1) ? 2 : 1, AB_DEPTH) : $urandom_range(1, 2 * AB_DEPTH);
        for (int k = 0; k < n; k++) raddrs.push_back(ab_rnd_addr());
      end
      pos          = 0;
      reads_issued = 0;
      in_img       = 1;
      relwait_phase = 0;
      p_read       = zb ? 100 : ab_pick(100, 80, 50, 20);
      first_read_pending = 1;
    endfunction

    function void step(actbuf_model m, ref ab_in_t it);
      bit rd_now, last_one, next_full;
      cyc++;
      if (!in_img) begin
        if (!done() && m.exp_full()) begin
          if (img_wait > 0) img_wait--;
          else begin_image();
        end
      end
      if (in_img) begin
        if (!relwait_phase) begin
          rd_now = ($urandom_range(99) < p_read);
          last_one = (pos == raddrs.size() - 1);
          if (zb && last_one && (done_cnt + 1 < total) &&
              !(reads_issued == 0 && cyc == last_rel_cyc + 1)) begin
            // Hold the last read until the next slot is already FULL (except a
            // single-read image that is itself a zero-bubble continuation).
            next_full = (m.st[1 - m.rp] == SL_FULL);
            if (!next_full) rd_now = 0;
          end
          if (rd_now) begin
            it.rd_en   = 1'b1;
            it.rd_addr = raddrs[pos];
            pos++;
            reads_issued++;
            if (first_read_pending) begin
              first_read_pending = 0;
              if (cyc == last_rel_cyc + 1) zb_count++;
            end
            if (pos == raddrs.size()) begin
              if (zb || $urandom_range(99) < 55) finish(m, it);
              else begin
                relwait_phase = 1;
                rel_wait      = $urandom_range(1, 4);
              end
            end
          end
        end else begin
          if (rel_wait > 1) rel_wait--;
          else finish(m, it);
        end
      end
    endfunction

    function void finish(actbuf_model m, ref ab_in_t it);
      if (zb && done_cnt + 1 < total && m.st[1 - m.rp] == SL_FULL) zb_possible++;
      it.rel        = 1'b1;
      in_img        = 0;
      relwait_phase = 0;
      done_cnt++;
      last_rel_cyc  = cyc;
      img_wait      = zb ? 0 : ab_pick(0, 0, 2, 9);
    endfunction
  endclass

  // ---- sequence -----------------------------------------------------------------
  class actbuf_seq extends uvm_sequence #(actbuf_item);
    `uvm_object_utils(actbuf_seq)

    bit  zb_mode;
    int  n_images;       // images of the final (reset-free) segment
    int  reset_kinds[$]; // one mid-run reset per entry (trigger predicate kind)
    int  zb_count;       // out: zero-bubble transitions performed
    int  zb_expected;    // out: transitions that had to be zero-bubble
    int  cycles;         // out

    actbuf_model m;
    actbuf_prod  prod;
    actbuf_cons  cons;

    function new(string name = "actbuf_seq");
      super.new(name);
    endfunction

    task send(bit rst, ab_in_t i);
      actbuf_item it;
      it = actbuf_item::type_id::create("it");
      start_item(it);
      it.rst = rst;
      it.i   = i;
      finish_item(it);
      cycles++;
    endtask

    function ab_in_t idle_in();
      ab_in_t i;
      i.claim   = 0;
      i.wr_en   = 0;
      i.commit  = 0;
      i.rd_en   = 0;
      i.rel     = 0;
      // Garbage on gated buses: the DUT must ignore it.
      i.wr_addr = ab_rnd_addr();
      i.wr_data = ab_rnd_data();
      i.rd_addr = ab_rnd_addr();
      return i;
    endfunction

    task do_reset(int n);
      ab_in_t i;
      for (int k = 0; k < n; k++) begin
        i = idle_in();
        send(1'b1, i);
      end
      m.reset();
    endtask

    task run_cycle();
      ab_in_t i;
      string  v;
      i = idle_in();
      prod.step(m, i);
      cons.step(m, i);
      v = m.violation(i);
      if (v != "") `uvm_fatal("SEQ_PROTO", $sformatf("sequence generated an illegal cycle: %s", v))
      m.step(i);
      send(1'b0, i);
    endtask

    function bit trigger_met(int kind);
      case (kind)
        0: return ((m.st[0] == SL_FULL && m.st[1] == SL_CLAIMED) ||
                   (m.st[1] == SL_FULL && m.st[0] == SL_CLAIMED));
        1: return cons.in_img && cons.reads_issued > 0;
        2: return 1'b1;
        3: return prod.writing_midway();
        default: return 1'b1;
      endcase
    endfunction

    // Run one segment. kind < 0: run until all images are consumed.
    // kind >= 0: run until the trigger predicate holds (after a few images),
    // then return, leaving the activity in flight for the caller to reset.
    task run_segment(int n, bit fast, bit zb, int kind);
      int min_imgs, eligible_cyc, timeout, idle_cnt;
      prod = new((kind < 0) ? n : 100000, fast);
      cons = new((kind < 0) ? n : 100000, zb);
      min_imgs = (kind < 0) ? 0 : $urandom_range(1, 3);
      eligible_cyc = -1;
      timeout = 40 * AB_DEPTH + 100;
      if (kind == 2) timeout = $urandom_range(1, 4 * AB_DEPTH + 10);
      forever begin
        if (kind < 0) begin
          if (prod.done() && cons.done()) break;
        end else begin
          if (cons.done_cnt >= min_imgs) begin
            if (eligible_cyc < 0) eligible_cyc = 0;
            else eligible_cyc++;
            if (trigger_met(kind) && eligible_cyc >= ((kind == 2) ? timeout : 0)) break;
            if (eligible_cyc >= timeout) break;
          end
        end
        run_cycle();
      end
      zb_count    += cons.zb_count;
      zb_expected += cons.zb_possible;
    endtask

    task body();
      ab_in_t i;
      cycles = 0;
      zb_count = 0;
      zb_expected = 0;
      m = new();
      // Initial reset.
      do_reset(3);
      // Mid-run resets with activity in flight.
      foreach (reset_kinds[r]) begin
        run_segment(0, ($urandom_range(1) == 0), 1'b0, reset_kinds[r]);
        do_reset($urandom_range(1, 3));
      end
      // Final segment, runs to completion.
      run_segment(n_images, zb_mode, zb_mode, -1);
      // Drain: let the last reads' data and status settle.
      for (int k = 0; k < AB_LAT + 4; k++) begin
        i = idle_in();
        send(1'b0, i);
      end
    endtask
  endclass

  // ---- driver / monitor ----------------------------------------------------------
  class actbuf_driver extends uvm_driver #(actbuf_item);
    `uvm_component_utils(actbuf_driver)
    virtual act_buf_if #(AB_WIDTH, AB_AW) vif;
    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction
    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      if (!uvm_config_db#(virtual act_buf_if #(AB_WIDTH, AB_AW))::get(this, "", "vif", vif))
        `uvm_fatal("NOVIF", "act_buf_if not set")
    endfunction
    task run_phase(uvm_phase phase);
      forever begin
        seq_item_port.get_next_item(req);
        @(posedge vif.clk);
        vif.rst        <= req.rst;
        vif.wr_claim   <= req.i.claim;
        vif.wr_en      <= req.i.wr_en;
        vif.wr_addr    <= req.i.wr_addr;
        vif.wr_data    <= req.i.wr_data;
        vif.wr_commit  <= req.i.commit;
        vif.rd_en      <= req.i.rd_en;
        vif.rd_addr    <= req.i.rd_addr;
        vif.rd_release <= req.i.rel;
        seq_item_port.item_done();
      end
    endtask
  endclass

  class actbuf_monitor extends uvm_monitor;
    `uvm_component_utils(actbuf_monitor)
    virtual act_buf_if #(AB_WIDTH, AB_AW) vif;
    uvm_analysis_port #(actbuf_txn) ap;
    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction
    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      ap = new("ap", this);
      if (!uvm_config_db#(virtual act_buf_if #(AB_WIDTH, AB_AW))::get(this, "", "vif", vif))
        `uvm_fatal("NOVIF", "act_buf_if not set")
    endfunction
    task run_phase(uvm_phase phase);
      actbuf_txn t;
      forever begin
        @(posedge vif.clk);
        t = actbuf_txn::type_id::create("t");
        t.rst          = (vif.rst === 1'b1);
        t.i.claim      = (vif.wr_claim === 1'b1);
        t.i.wr_en      = (vif.wr_en === 1'b1);
        t.i.wr_addr    = vif.wr_addr;
        t.i.wr_data    = vif.wr_data;
        t.i.commit     = (vif.wr_commit === 1'b1);
        t.i.rd_en      = (vif.rd_en === 1'b1);
        t.i.rd_addr    = vif.rd_addr;
        t.i.rel        = (vif.rd_release === 1'b1);
        t.wr_can_claim = vif.wr_can_claim;
        t.rd_full      = vif.rd_full;
        t.rd_full_next = vif.rd_full_next;
        t.idle         = vif.idle;
        t.rd_data      = vif.rd_data;
        ap.write(t);
      end
    endtask
  endclass

  // ---- protocol monitor (self-check of the TB's own agents) ---------------------
  class actbuf_proto_mon extends uvm_subscriber #(actbuf_txn);
    `uvm_component_utils(actbuf_proto_mon)
    actbuf_model m;
    bit          seen_rst;
    function new(string name, uvm_component parent);
      super.new(name, parent);
      m = new();
    endfunction
    function void write(actbuf_txn t);
      string v;
      if (t.rst) begin
        m.reset();
        seen_rst = 1'b1;
      end else if (seen_rst) begin
        v = m.violation(t.i);
        if (v != "") `uvm_fatal("PROTO", $sformatf("TB protocol violation: %s", v))
        m.step(t.i);
      end
    endfunction
  endclass

  // ---- scoreboard ------------------------------------------------------------------
  class actbuf_scb extends bnn_scb_base;
    `uvm_component_utils(actbuf_scb)
    uvm_analysis_imp #(actbuf_txn, actbuf_scb) imp;

    typedef struct {
      bit       valid;
      ab_addr_t addr;
      ab_data_t data;
    } pend_t;

    actbuf_model m;
    pend_t       pipe[$];
    bit          seen_rst;
    int          zb_seen;
    int          cw_seen;   // commits coinciding with a write
    bit          prev_rd_rel;
    bit          zb_strict;     // set by the zero-bubble test: rd_full_next must have been 1
    logic        prev_rel_fn;   // rd_full_next sampled in the last release+read cycle

    function new(string name, uvm_component parent);
      super.new(name, parent);
      imp = new("imp", this);
      m   = new();
    endfunction

    function void write(actbuf_txn t);
      pend_t  p, e;
      bit     ec, ef, ei, efn;
      string  v;

      // rd_data now belongs to the read issued AB_LAT cycles ago.
      if (seen_rst && pipe.size() == AB_LAT) begin
        e = pipe.pop_front();
        if (e.valid) check_eq($sformatf("rd_data[addr=%0d]", e.addr), t.rd_data, e.data);
      end

      if (t.rst) begin
        m.reset();
        pipe.delete();
        seen_rst    = 1'b1;
        prev_rd_rel = 1'b0;
        return;
      end
      if (!seen_rst) return;

      // Registered status outputs reflect all events up to the previous edge.
      ec = m.exp_can_claim();
      ef = m.exp_full();
      ei = m.exp_idle();
      efn = m.exp_full_next();
      check_eq("wr_can_claim", t.wr_can_claim, ec);
      check_eq("rd_full", t.rd_full, ef);
      check_eq("rd_full_next", t.rd_full_next, efn);
      check_eq("idle", t.idle, ei);

      // Read issued this edge, data snapshot from the pre-edge state.
      p.valid = 1'b0;
      p.addr  = t.i.rd_addr;
      p.data  = '0;
      if (t.i.rd_en && ef) begin
        p.valid = 1'b1;
        p.data  = m.rd_value(t.i.rd_addr);
      end
      pipe.push_back(p);

      if (t.i.commit && t.i.wr_en) cw_seen++;
      // Back-to-back transition. In the zero-bubble test the consumer only releases
      // on its last read once the next slot is FULL, i.e. it relies on rd_full_next
      // having been 1 in the release cycle (4-state compare against 1). Not a
      // universal rule: a commit may land in the release cycle itself.
      if (prev_rd_rel && t.i.rd_en) begin
        zb_seen++;
        if (zb_strict) check_eq("rd_full_next in release cycle of back-to-back transition", prev_rel_fn, 1'b1);
      end
      prev_rd_rel = t.i.rd_en && t.i.rel;
      prev_rel_fn = t.rd_full_next;

      v = m.violation(t.i);
      if (v == "") m.step(t.i);
    endfunction
  endclass

  // ---- coverage ------------------------------------------------------------------------
  class actbuf_cov extends uvm_subscriber #(actbuf_txn);
    `uvm_component_utils(actbuf_cov)
    actbuf_model m;
    bit          seen_rst;
    int          s0, s1;
    bit          ev_claim, ev_wr, ev_commit, ev_rd, ev_rel;
    bit          claim_ahead, rd_wr_overlap, zb, commit_wr;
    bit          prev_rel_rd;

    covergroup cg;
      cp_s0: coverpoint s0 { bins e = {0}; bins c = {1}; bins f = {2}; }
      cp_s1: coverpoint s1 { bins e = {0}; bins c = {1}; bins f = {2}; }
      cp_states: cross cp_s0, cp_s1;
      cp_claim:  coverpoint ev_claim;
      cp_commit: coverpoint ev_commit;
      cp_rel:    coverpoint ev_rel;
      cp_ahead:  coverpoint claim_ahead { bins hit = {1}; }
      cp_ovl:    coverpoint rd_wr_overlap { bins hit = {1}; }
      cp_zb:     coverpoint zb { bins hit = {1}; }
      cp_cw:     coverpoint commit_wr { bins hit = {1}; }
      cp_ev:     cross cp_claim, cp_commit, cp_rel;
    endgroup

    function new(string name, uvm_component parent);
      super.new(name, parent);
      m  = new();
      cg = new();
    endfunction

    function void report_phase(uvm_phase phase);
      super.report_phase(phase);
      `uvm_info("COV", $sformatf("act_buf functional coverage = %0.1f%% (slot-state cross %0.1f%%)",
                                cg.get_inst_coverage(), cg.cp_states.get_inst_coverage()), UVM_NONE)
    endfunction

    function void write(actbuf_txn t);
      if (t.rst) begin
        m.reset();
        seen_rst = 1'b1;
        return;
      end
      if (!seen_rst) return;
      s0            = int'(m.st[0]);
      s1            = int'(m.st[1]);
      ev_claim      = t.i.claim;
      ev_commit     = t.i.commit;
      ev_rel        = t.i.rel;
      claim_ahead   = t.i.claim && (m.st[m.wp] == SL_CLAIMED);
      rd_wr_overlap = t.i.rd_en && t.i.wr_en;
      commit_wr     = t.i.commit && t.i.wr_en;
      zb            = prev_rel_rd && t.i.rd_en;
      prev_rel_rd   = t.i.rd_en && t.i.rel;
      cg.sample();
      if (m.violation(t.i) == "") m.step(t.i);
    endfunction
  endclass

  // ---- env / tests --------------------------------------------------------------------------
  class actbuf_env extends uvm_env;
    `uvm_component_utils(actbuf_env)
    uvm_sequencer #(actbuf_item) sqr;
    actbuf_driver     drv;
    actbuf_monitor    mon;
    actbuf_proto_mon  pmon;
    actbuf_scb        scb;
    actbuf_cov        cov;
    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction
    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      sqr  = uvm_sequencer #(actbuf_item)::type_id::create("sqr", this);
      drv  = actbuf_driver::type_id::create("drv", this);
      mon  = actbuf_monitor::type_id::create("mon", this);
      pmon = actbuf_proto_mon::type_id::create("pmon", this);
      scb  = actbuf_scb::type_id::create("act_buf", this);
      cov  = actbuf_cov::type_id::create("cov", this);
    endfunction
    function void connect_phase(uvm_phase phase);
      drv.seq_item_port.connect(sqr.seq_item_export);
      mon.ap.connect(scb.imp);
      mon.ap.connect(pmon.analysis_export);
      mon.ap.connect(cov.analysis_export);
    endfunction
  endclass

  class actbuf_base_test extends uvm_test;
    `uvm_component_utils(actbuf_base_test)
    actbuf_env env;
    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction
    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      env = actbuf_env::type_id::create("env", this);
    endfunction

    // Cycle budget: generous bound for n images plus resets.
    function int budget_ns(int n_images, int n_resets);
      return 2 * ((n_images + 4 * n_resets + 8) * (AB_DEPTH * 24 + 300) + 2000);
    endfunction
  endclass

  class actbuf_random_test extends actbuf_base_test;
    `uvm_component_utils(actbuf_random_test)
    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction
    task run_phase(uvm_phase phase);
      actbuf_seq seq;
      int        b;
      phase.raise_objection(this);
      b = budget_ns(24, 4);
      fork
        bnn_watchdog("actbuf_random_test", b);
      join_none
      seq = actbuf_seq::type_id::create("seq");
      seq.zb_mode  = 1'b0;
      seq.n_images = 24;
      seq.reset_kinds = '{0, 1, 3, 2};
      seq.start(env.sqr);
      repeat (AB_LAT + 6) @(posedge env.drv.vif.clk);
      if (env.scb.cw_seen == 0)
        `uvm_error("CW_TB", "no commit coinciding with a write was exercised")
      phase.drop_objection(this);
    endtask
  endclass

  class actbuf_zero_bubble_test extends actbuf_base_test;
    `uvm_component_utils(actbuf_zero_bubble_test)
    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction
    task run_phase(uvm_phase phase);
      actbuf_seq seq;
      int        b;
      phase.raise_objection(this);
      b = budget_ns(60, 0);
      fork
        bnn_watchdog("actbuf_zero_bubble_test", b);
      join_none
      seq = actbuf_seq::type_id::create("seq");
      env.scb.zb_strict = 1'b1;
      seq.zb_mode  = 1'b1;
      seq.n_images = 60;
      seq.start(env.sqr);
      repeat (AB_LAT + 6) @(posedge env.drv.vif.clk);
      if (env.scb.cw_seen == 0)
        `uvm_error("CW_TB", "no commit coinciding with a write was exercised")
      // TB self-check: every transition with the next slot already FULL must have
      // been zero-bubble, and (with DEPTH >= 2) that must be all of them.
      if (AB_DEPTH >= 2 && seq.zb_expected != 59)
        `uvm_error("ZB_TB", $sformatf("only %0d of 59 transitions had the next slot FULL",
                                      seq.zb_expected))
      if (AB_DEPTH < 2 && seq.zb_expected < 15)
        `uvm_error("ZB_TB", $sformatf("only %0d zero-bubble opportunities", seq.zb_expected))
      if (seq.zb_count != seq.zb_expected)
        `uvm_error("ZB_TB", $sformatf("stimulus did only %0d of %0d zero-bubble transitions",
                                      seq.zb_count, seq.zb_expected))
      if (env.scb.zb_seen != seq.zb_expected)
        `uvm_error("ZB_TB", $sformatf("monitor saw only %0d of %0d zero-bubble transitions",
                                      env.scb.zb_seen, seq.zb_expected))
      phase.drop_objection(this);
    endtask
  endclass

endpackage
