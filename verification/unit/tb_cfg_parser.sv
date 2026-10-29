// Self-checking testbench for cfg_parser, TOPOLOGY '{13,7,5,3} (L = 3 non-input layers:
// weights for layers 0..2, thresholds for layers 0..1). Byte items are driven directly (no serializer):
// each item carries a beat id, and drop_rest at a handshake removes the rest of that beat from the queue
// (as cfg_byte_serializer does). Payload beats end with garbage bytes, so a missing drop_rest corrupts
// the next header. pl_ready is random; pipeline_idle is random except in the directed WAIT_IDLE phase.
//
// Every cycle the monitor checks: configured and the load flags against a model (cleared at msg_start,
// set on clean completion); drop_rest / msg_abort only and exactly on the expected handshakes; payload
// bytes (with msg_sel/msg_type) against the expected queue; pl_valid only inside an accepted message;
// msg_start (layer/type) exactly when the WAIT_IDLE rule says (pipeline_idle honored from the second
// WAIT_IDLE cycle), with pl_valid low in that cycle; s_ready low in WAIT_IDLE; cfg_busy rising only after
// a valid header and falling only after completion/abort. At every negedge s_valid is flipped for a
// moment to check that s_ready does not depend combinationally on s_valid.
//
// Phases: (a) valid messages for every layer/type (TB order, then shuffled), (b) every invalid header kind,
// (c) truncated messages, (d) directed WAIT_IDLE, (e) TLAST during a header, (f) reset.
module tb_cfg_parser;
  localparam int TL = 4;
  localparam int TOPO [TL] = '{13, 7, 5, 3};
  localparam int L = TL - 1;

  // message end modes
  localparam int EM_FULL       = 0;  // all total_bytes, then garbage to the end of the beat
  localparam int EM_FULL_LAST  = 1;  // all total_bytes, TLAST on the last payload byte
  localparam int EM_TRUNC      = 2;  // npl bytes, TLAST on byte npl-1 (truncated)
  localparam int EM_TRUNC_NULL = 3;  // npl bytes, then a null item (TKEEP=0, TLAST=1)

  logic       clk = 1'b0;
  always #5 clk = ~clk;

  logic       rst = 1'b1;
  logic       s_valid = 1'b0, s_ready;
  logic [7:0] s_byte = '0;
  logic       s_null = 1'b0, s_beat_last = 1'b0;
  logic       drop_rest;
  logic       pipeline_idle = 1'b0;
  logic       cfg_busy, configured;
  logic       msg_start, msg_abort;
  logic [L-1:0] msg_sel;
  logic       msg_type;
  logic       pl_valid, pl_ready = 1'b0;
  logic [7:0] pl_byte;

  cfg_parser #(.TOTAL_LAYERS(TL), .TOPOLOGY(TOPO)) dut (.*);

  typedef struct {
    bit [7:0] b;
    bit       nul;
    bit       last;
    int       beat;
    bit       exp_drop;    // drop_rest expected at this item's handshake
    bit       exp_done;    // clean completion of an accepted message
    bit       exp_abort;   // truncation of an accepted message
    bit       exp_hv;      // 16th byte of a valid header
    int       layer;
    bit       typ;
  } item_t;

  typedef struct {
    bit [7:0] b;
    int       layer;
    bit       typ;
  } plb_t;

  typedef struct {
    int layer;
    bit typ;
  } st_t;

  item_t items [$];
  plb_t  exp_pl [$];
  st_t   exp_st [$];
  item_t cur;
  int    beat_id = 0;
  int    bpos = 0;            // position of the next item within the current beat
  int    errors = 0;
  bit    mon_on = 1'b0;
  bit    idle_rand = 1'b1;

  // model
  bit    tw [L];
  bit    tt [L];
  bit    started = 1'b0, go_prev = 1'b0, prev_busy = 1'b0, hs_hv_prev = 1'b0, hs_end_prev = 1'b0;
  int    wcyc = 0;
  int    last_wait_len = 0;
  int    n_start = 0, n_abort = 0, n_drop = 0, n_pl = 0, n_conf_rise = 0;
  bit    prev_conf = 1'b0;

  function automatic bit model_conf();
    bit c;
    c = 1'b1;
    for (int l = 0; l < L; l++) c &= tw[l];
    for (int l = 0; l < L - 1; l++) c &= tt[l];
    return c;
  endfunction

  // ---------------- monitor + driver (sample pre-edge values, drive with NBA)
  always @(posedge clk) begin : mon
    bit   hs, in_wait, go;
    plb_t p;
    st_t  st;
    hs = s_valid && s_ready;
    if (mon_on) begin
      if (configured !== model_conf()) begin
        errors++; $error("configured=%b, model %b", configured, model_conf());
      end
      for (int l = 0; l < L; l++) if (dut.w_loaded[l] !== tw[l]) begin
        errors++; $error("w_loaded[%0d]=%b, model %b", l, dut.w_loaded[l], tw[l]);
      end
      for (int l = 0; l < L - 1; l++) if (dut.t_loaded[l] !== tt[l]) begin
        errors++; $error("t_loaded[%0d]=%b, model %b", l, dut.t_loaded[l], tt[l]);
      end
      if (configured && !prev_conf) n_conf_rise++;
      prev_conf = configured;

      if (drop_rest && !hs) begin errors++; $error("drop_rest without a handshake"); end
      if (msg_abort && !hs) begin errors++; $error("msg_abort without a handshake"); end
      if (hs) begin
        if (drop_rest !== cur.exp_drop) begin
          errors++; $error("drop_rest=%b, expected %b (byte %02h)", drop_rest, cur.exp_drop, cur.b);
        end
        if (msg_abort !== cur.exp_abort) begin
          errors++; $error("msg_abort=%b, expected %b", msg_abort, cur.exp_abort);
        end
        if (drop_rest) n_drop++;
        if (msg_abort) n_abort++;
      end

      if (pl_valid && !(cfg_busy && started && !msg_start)) begin
        errors++; $error("pl_valid outside an accepted message payload");
      end
      if (pl_valid && pl_ready) begin
        n_pl++;
        if (!hs) begin errors++; $error("payload handshake without input handshake"); end
        if (exp_pl.size() == 0) begin
          errors++; $error("unexpected payload byte %02h", pl_byte);
        end else begin
          p = exp_pl.pop_front();
          if (pl_byte !== p.b || msg_sel !== (L'(1) << p.layer) || msg_type !== p.typ) begin
            errors++;
            $error("payload got %02h sel %b T%0d, expected %02h L%0d T%0d",
                   pl_byte, msg_sel, msg_type, p.b, p.layer, p.typ);
          end
        end
      end

      if (msg_start !== go_prev) begin
        errors++; $error("msg_start=%b, expected %b (WAIT_IDLE rule)", msg_start, go_prev);
      end
      if (msg_start) begin
        n_start++;
        if (pl_valid) begin errors++; $error("pl_valid in the msg_start cycle"); end
        if (exp_st.size() == 0) begin
          errors++; $error("unexpected msg_start sel %b T%0d", msg_sel, msg_type);
        end else begin
          st = exp_st.pop_front();
          if (!$onehot(msg_sel)) begin
            errors++; $error("msg_sel %b not one-hot at msg_start", msg_sel);
          end
          if (msg_sel !== (L'(1) << st.layer) || msg_type !== st.typ) begin
            errors++; $error("msg_start sel %b T%0d, expected L%0d T%0d", msg_sel, msg_type, st.layer, st.typ);
          end
          if (st.typ) tt[st.layer] = 1'b0;
          else        tw[st.layer] = 1'b0;
        end
      end
      if (hs && cur.exp_done) begin
        if (cur.typ) tt[cur.layer] = 1'b1;
        else         tw[cur.layer] = 1'b1;
      end

      if (!prev_busy && cfg_busy && !hs_hv_prev) begin errors++; $error("cfg_busy rose without a valid header"); end
      if (hs_hv_prev && !cfg_busy) begin errors++; $error("cfg_busy did not rise after a valid header"); end
      if (prev_busy && !cfg_busy && !hs_end_prev) begin errors++; $error("cfg_busy fell before the message ended"); end
      if (hs_end_prev && cfg_busy) begin errors++; $error("cfg_busy still set after the message ended"); end

      in_wait = cfg_busy && !started;
      if (in_wait && s_ready) begin errors++; $error("s_ready in WAIT_IDLE"); end
      go = in_wait && (wcyc >= 1) && pipeline_idle;
      if (in_wait) wcyc++;
      if (go) begin started = 1'b1; last_wait_len = wcyc; end
      if (!cfg_busy) begin started = 1'b0; wcyc = 0; end
      go_prev     = go;
      prev_busy   = cfg_busy;
      hs_hv_prev  = hs && cur.exp_hv;
      hs_end_prev = hs && (cur.exp_done || cur.exp_abort);
    end

    // ---- drivers
    if (rst) begin
      s_valid <= 1'b0;
    end else begin
      if (hs && drop_rest) while (items.size() != 0 && items[0].beat == cur.beat) void'(items.pop_front());
      if (!s_valid || s_ready) begin
        if (items.size() != 0 && $urandom_range(3, 0) != 0) begin
          cur = items.pop_front();
          s_valid     <= 1'b1;
          s_byte      <= cur.b;
          s_null      <= cur.nul;
          s_beat_last <= cur.last;
        end else begin
          s_valid     <= 1'b0;
          s_byte      <= 8'($urandom());
          s_null      <= 1'($urandom());
          s_beat_last <= 1'($urandom());
        end
      end
    end
    pl_ready <= ($urandom_range(3, 0) != 0);
    if (idle_rand) pipeline_idle <= ($urandom_range(2, 0) != 0);
  end

  // ---------------- s_ready must not depend combinationally on s_valid
  always @(negedge clk) if (mon_on && !rst) begin : indep
    logic sv, r0, r1;
    sv = s_valid;
    s_valid = !sv;
    #1 r0 = s_ready;
    s_valid = sv;
    #1 r1 = s_ready;
    if (r0 !== r1) begin errors++; $error("s_ready depends combinationally on s_valid"); end
  end

  // ---------------- stream construction
  function automatic void new_beat();
    if (bpos != 0) begin beat_id++; bpos = 0; end
  endfunction

  function automatic void push_item(item_t it, int bw);
    it.beat = beat_id;
    items.push_back(it);
    bpos++;
    if (it.last || bpos == bw) begin beat_id++; bpos = 0; end
  endfunction

  function automatic int rand_bw();
    int t [6] = '{1, 2, 3, 4, 8, 16};
    return t[$urandom_range(5, 0)];
  endfunction

  // One message on fresh beats of bw bytes. valid = the DUT must accept the header.
  function automatic void add_msg(int typ, int layer, int inputs, int neurons, int bpn, int unsigned total,
                                  bit valid, int em, int npl, int bw, int unsigned reserved = 0);
    item_t      it;
    bit [127:0] h;
    int         ng;
    h = '0;
    h[7:0]    = 8'(typ);
    h[15:8]   = 8'(layer);
    h[31:16]  = 16'(inputs);
    h[47:32]  = 16'(neurons);
    h[63:48]  = 16'(bpn);
    h[95:64]  = total;
    h[127:96] = reserved;
    new_beat();
    for (int i = 0; i < 16; i++) begin
      it = '{default: 0};
      it.b = h[8*i +: 8];
      it.exp_hv = valid && (i == 15);
      push_item(it, bw);
    end
    if (valid) exp_st.push_back('{layer, typ[0]});
    if (em == EM_FULL || em == EM_FULL_LAST) npl = int'(total);
    for (int i = 0; i < npl; i++) begin
      it = '{default: 0};
      it.b = 8'($urandom());
      it.layer = layer;
      it.typ = typ[0];
      if (i == npl - 1) begin
        if (em == EM_FULL || em == EM_FULL_LAST) begin
          it.exp_drop = 1'b1;
          it.exp_done = valid;
          it.last = (em == EM_FULL_LAST);
        end else if (em == EM_TRUNC) begin
          it.last = 1'b1;
          it.exp_abort = valid;
        end
      end
      if (valid && !(em == EM_TRUNC && i == npl - 1)) exp_pl.push_back('{it.b, layer, typ[0]});
      push_item(it, bw);
    end
    if (em == EM_FULL && npl > 0 && bpos != 0) begin   // garbage after the last payload byte, same beat
      ng = $urandom_range(bw - bpos, 1);
      for (int i = 0; i < ng; i++) begin
        it = '{default: 0};
        it.b = 8'($urandom());
        it.last = (i == ng - 1) && ($urandom_range(1, 0) == 1);
        push_item(it, (i == ng - 1) ? bpos + 1 : bw);   // last garbage byte closes the beat
      end
    end
    if (em == EM_TRUNC_NULL) begin
      new_beat();
      it = '{default: 0};
      it.nul = 1'b1;
      it.last = 1'b1;
      it.b = 8'($urandom());
      it.exp_abort = valid;
      push_item(it, 1);
    end
  endfunction

  function automatic int bpn_of(int l);
    return (TOPO[l] + 7) / 8;
  endfunction

  function automatic void add_w(int l, int em = EM_FULL, int npl = 0, int bw = 0);
    add_msg(0, l, TOPO[l], TOPO[l+1], bpn_of(l), TOPO[l+1] * bpn_of(l), 1'b1, em, npl,
            (bw == 0) ? rand_bw() : bw);
  endfunction

  function automatic void add_t(int l, int em = EM_FULL, int npl = 0, int bw = 0, int inputs = 32,
                                int unsigned reserved = 0);
    add_msg(1, l, inputs, TOPO[l+1], 4, 4 * TOPO[l+1], 1'b1, em, npl, (bw == 0) ? rand_bw() : bw, reserved);
  endfunction

  function automatic void add_bad(int typ, int layer, int inputs, int neurons, int bpn, int unsigned total,
                                  int em = EM_FULL, int npl = 0);
    add_msg(typ, layer, inputs, neurons, bpn, total, 1'b0, em, npl, rand_bw(), 32'($urandom()));
  endfunction

  // n header bytes (no TLAST) and then either a TLAST on the n-th byte or a null item.
  function automatic void add_partial_header(int n, bit use_null, int bw);
    item_t it;
    new_beat();
    for (int i = 0; i < n; i++) begin
      it = '{default: 0};
      it.b = 8'($urandom());
      it.last = !use_null && (i == n - 1);
      push_item(it, bw);
    end
    if (use_null) begin
      new_beat();
      it = '{default: 0};
      it.nul = 1'b1;
      it.last = 1'b1;
      push_item(it, 1);
    end
  endfunction

  // all valid messages; order 0 = TB order, otherwise shuffled
  function automatic void add_full_config(bit shuffled, int bw, bit tlast_on_last);
    int ord [5] = '{0, 1, 2, 3, 4};   // W0 T0 W1 T1 W2
    int k;
    if (shuffled) ord.shuffle();
    for (int i = 0; i < 5; i++) begin
      k = ord[i];
      if (k % 2 == 0) add_w(k / 2, (tlast_on_last && i == 4) ? EM_FULL_LAST : EM_FULL, 0, bw);
      else            add_t(k / 2, (tlast_on_last && i == 4) ? EM_FULL_LAST : EM_FULL, 0, bw);
    end
  endfunction

  task automatic wait_done();
    int guard;
    guard = 0;
    while ((items.size() != 0 || s_valid || cfg_busy) && guard < 200000) begin
      @(posedge clk);
      guard++;
    end
    repeat (4) @(posedge clk);
    if (guard >= 200000) begin errors++; $error("stream did not drain (%0d items left)", items.size()); end
    if (exp_pl.size() != 0) begin errors++; $error("%0d expected payload bytes not seen", exp_pl.size()); end
    if (exp_st.size() != 0) begin errors++; $error("%0d expected msg_start not seen", exp_st.size()); end
  endtask

  task automatic do_reset();
    @(negedge clk);
    mon_on = 1'b0;
    rst = 1'b1;
    items.delete();
    exp_pl.delete();
    exp_st.delete();
    repeat (3) @(negedge clk);
    for (int l = 0; l < L; l++) begin tw[l] = 1'b0; tt[l] = 1'b0; end
    started = 1'b0; go_prev = 1'b0; prev_busy = 1'b0; hs_hv_prev = 1'b0; hs_end_prev = 1'b0;
    wcyc = 0; prev_conf = 1'b0;
    new_beat();
    rst = 1'b0;
    if (configured !== 1'b0 || cfg_busy !== 1'b0 || dut.w_loaded !== '0 || dut.t_loaded !== '0) begin
      errors++; $error("state not cleared by reset");
    end
    mon_on = 1'b1;
  endtask

  task automatic check(bit cond, string what);
    if (!cond) begin errors++; $error("check failed: %s", what); end
  endtask

  bit snap_w [L];
  bit snap_t [L];

  task automatic bad_and_check(int typ, int layer, int inputs, int neurons, int bpn, int unsigned total,
                               int em = EM_FULL, int npl = 0);
    int s0;
    for (int l = 0; l < L; l++) begin snap_w[l] = dut.w_loaded[l]; snap_t[l] = dut.t_loaded[l]; end
    s0 = n_start;
    add_bad(typ, layer, inputs, neurons, bpn, total, em, npl);
    wait_done();
    check(n_start == s0, $sformatf("no msg_start for invalid header t%0d l%0d", typ, layer));
    for (int l = 0; l < L; l++)
      check(dut.w_loaded[l] == snap_w[l] && dut.t_loaded[l] == snap_t[l], "flags unchanged by invalid header");
    // the parser must be back in sync: a valid message right after
    add_w(1);
    wait_done();
  endtask

  initial begin
    #20ms;
    $error("tb_cfg_parser: timeout");
    $finish;
  end

  int s0, a0, d0;
  int ord [5];

  initial begin
    for (int l = 0; l < L; l++) begin tw[l] = 1'b0; tt[l] = 1'b0; end
    repeat (4) @(negedge clk);
    rst = 1'b0;
    mon_on = 1'b1;

    // ---------------- (a) valid messages for every layer/type
    add_full_config(1'b0, 8, 1'b1);                 // provided-TB layout: 8-byte beats, TLAST at the end
    wait_done();
    check(configured === 1'b1 && n_conf_rise == 1, "(a) configured after TB-order config");
    for (int r = 0; r < 6; r++) begin
      add_full_config(1'b1, 0, r % 2);
      wait_done();
      check(configured === 1'b1, "(a) configured after shuffled config");
    end
    // configured rises only after the last required message, in a fresh round from reset
    do_reset();
    begin : a_order
      ord = '{0, 1, 2, 3, 4};
      ord.shuffle();
      for (int i = 0; i < 5; i++) begin
        if (ord[i] % 2 == 0) add_w(ord[i] / 2); else add_t(ord[i] / 2);
        wait_done();
        check(configured === (i == 4), $sformatf("(a) configured after %0d of 5 messages", i + 1));
      end
    end
    check(n_abort == 0, "(a) no msg_abort for complete messages");

    // ---------------- (b) invalid headers are discarded
    bad_and_check(2,     0, 13, 7, 2, 14);          // unknown msg_type
    bad_and_check(8'h81, 0, 32, 7, 4, 28);          // unknown msg_type (bit 0 set)
    bad_and_check(0,     3, 3,  1, 1, 1);           // weights, layer_id 3
    bad_and_check(1,     2, 32, 3, 4, 12);          // thresholds for the output layer
    bad_and_check(0,     1, 8,  5, 1, 5);           // wrong layer_inputs
    bad_and_check(0,     0, 13, 6, 2, 12);          // wrong num_neurons (weights)
    bad_and_check(1,     1, 32, 4, 4, 16);          // wrong num_neurons (thresholds)
    bad_and_check(0,     2, 5,  3, 2, 3);           // wrong bytes_per_neuron (weights)
    bad_and_check(1,     0, 32, 7, 2, 14);          // wrong bytes_per_neuron (thresholds)
    bad_and_check(0,     2, 5,  3, 1, 4);           // wrong total_bytes (weights)
    bad_and_check(1,     0, 32, 7, 4, 27);          // wrong total_bytes (thresholds)
    d0 = n_drop;
    bad_and_check(2,     0, 0,  0, 0, 0);           // total_bytes == 0: back to HEADER, no drop_rest
    check(n_drop == d0 + 1, "(b) total_bytes == 0 discard has no drop_rest");   // +1 = the recovery msg
    bad_and_check(0,     3, 3,  1, 1, 1000, EM_TRUNC, 9);       // discard ended early by TLAST
    bad_and_check(5,     1, 7,  5, 1, 1000, EM_TRUNC_NULL, 4);  // discard ended early by a null item
    bad_and_check(5,     1, 7,  5, 1, 1000, EM_TRUNC_NULL, 0);
    // thresholds ignore layer_inputs and reserved
    add_t(0, EM_FULL, 0, 0, 16'hBEEF, 32'hDEADBEEF);
    add_t(1, EM_FULL, 0, 0, 0);
    wait_done();
    check(configured === 1'b1, "(b) configured after invalid headers");

    // ---------------- (c) truncated messages
    a0 = n_abort;
    add_w(0, EM_TRUNC, 7);
    wait_done();
    check(n_abort == a0 + 1 && !dut.w_loaded[0] && configured === 1'b0, "(c) truncated weights");
    add_w(0);
    wait_done();
    check(configured === 1'b1, "(c) reconfigured after truncation");
    add_t(1, EM_TRUNC_NULL, 5);
    add_w(2, EM_TRUNC, 1);
    add_t(0, EM_TRUNC_NULL, 0);
    add_w(1, EM_TRUNC_NULL, 4);
    wait_done();
    check(n_abort == a0 + 5 && configured === 1'b0, "(c) truncations");
    check(!dut.t_loaded[1] && !dut.w_loaded[2] && !dut.t_loaded[0] && !dut.w_loaded[1], "(c) flags cleared");
    add_full_config(1'b1, 0, 1'b0);
    wait_done();
    check(configured === 1'b1, "(c) configured again");

    // ---------------- (d) WAIT_IDLE
    idle_rand = 1'b0;
    @(negedge clk);
    pipeline_idle = 1'b0;
    s0 = n_start;
    add_w(1);
    while (!cfg_busy) @(negedge clk);              // first WAIT_IDLE cycle
    pipeline_idle = 1'b1;                          // idle in the first cycle only: must be ignored
    @(negedge clk);
    pipeline_idle = 1'b0;
    for (int i = 0; i < 20; i++) begin
      check(cfg_busy === 1'b1 && s_ready === 1'b0 && n_start == s0, "(d) waiting while not idle");
      @(negedge clk);
    end
    #3 check(s_valid === 1'b1, "(d) first payload byte pending during WAIT_IDLE");   // after the indep probe
    pipeline_idle = 1'b1;                          // one idle cycle from the second cycle on: proceed
    @(negedge clk);
    pipeline_idle = 1'b0;
    @(negedge clk);
    check(n_start == s0 + 1, "(d) msg_start after idle");
    wait_done();
    check(configured === 1'b1, "(d) configured");
    // idle held high: msg_start after exactly two WAIT_IDLE cycles
    pipeline_idle = 1'b1;
    add_t(0);
    wait_done();
    check(last_wait_len == 2 && configured === 1'b1, $sformatf("(d) wait length %0d", last_wait_len));
    idle_rand = 1'b1;

    // ---------------- (e) TLAST during a header discards the partial header
    s0 = n_start;
    add_partial_header(7, 1'b0, 8);
    add_t(0);
    add_partial_header(5, 1'b1, 4);
    add_w(2);
    add_partial_header(0, 1'b1, 1);
    add_partial_header(15, 1'b0, 16);
    add_w(0);
    add_partial_header(16, 1'b0, 16);              // TLAST on the 16th header byte: header dropped too
    add_t(1);
    wait_done();
    check(n_start == s0 + 4 && configured === 1'b1, "(e) partial headers discarded");

    // ---------------- (f) reset clears flags and returns to HEADER mid-payload
    s0 = n_start;
    add_w(0, EM_FULL, 0, 1);
    while (n_start == s0) @(negedge clk);
    repeat (5) @(negedge clk);
    do_reset();
    add_full_config(1'b1, 0, 1'b1);
    wait_done();
    check(configured === 1'b1, "(f) configured after reset");

    check(n_drop > 0 && n_pl > 0 && n_abort > 0, "activity seen");
    $display("tb_cfg_parser: %0d msg_start, %0d msg_abort, %0d drop_rest, %0d payload bytes, %0d errors",
             n_start, n_abort, n_drop, n_pl, errors);
    if (errors == 0) $display("SUCCESS: tb_cfg_parser");
    else $display("tb_cfg_parser: %0d mismatches", errors);
    $finish;
  end
endmodule
