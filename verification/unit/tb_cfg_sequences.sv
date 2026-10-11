// System tests for bnn_fcc configuration sequencing, reconfiguration, protocol misuse, backpressure
// and reset. The test flow lives in cfgseq_harness (DUT + drivers + monitor + tests), instantiated
// twice by tb_cfg_sequences and run one after the other:
//   topo4: TOPOLOGY '{20,12,9,5},      PARALLEL_INPUTS 4, PARALLEL_NEURONS '{3,4,2}
//   topo5: TOPOLOGY '{30,17,9,12,6},   PARALLEL_INPUTS 5, PARALLEL_NEURONS '{4,9,5,6}  (5 layers, R8)
// Every test starts with a reset and configures from scratch. Each test prints "PASS <test> [<harness>]".
//
// Reference models are BNN_FCC_Model objects built here (not create_random: under xsim
// std::randomize() empties dynamic arrays, so its weight rows would be empty). A modified model is a
// deep copy of another with one layer's weights or thresholds replaced. Expected classes come from
// compute_reference. No .sum() / package trackers are used (xsim returns 0 for .sum() of reals).
// xsim does not re-create uninitialized queues/dynamic arrays declared inside loop bodies, so every
// queue/dynamic array is declared at module or task scope and cleared explicitly.
//
// Drivers / monitor:
//   send_msg(stream, keep, tlast_on_last_beat)  config beats, random valid gaps
//   send_image(img, zero_gap)                   image beats (8 pixels/beat), random gaps unless zero_gap
//   expect_outputs(model_q, n, tag)             pops n sent images and n received outputs; each output
//                                               must equal the reference class of a model in model_q
//   monitor (always block)                      collects outputs; checks keep/last, valid/data stable
//                                               under backpressure, no output without an accepted image,
//                                               no X on data_out_valid
module cfgseq_harness #(
  parameter string NAME = "topo4",
  parameter int TOTAL_LAYERS = 4,
  parameter int TOPOLOGY[TOTAL_LAYERS] = '{20, 12, 9, 5},
  parameter int PARALLEL_INPUTS = 4,
  parameter int PARALLEL_NEURONS[TOTAL_LAYERS-1] = '{3, 4, 2},
  parameter int SEED = 1
) (
  input  logic go,
  output logic done,
  output int   nerr
);
  import bnn_fcc_tb_pkg::*;

  localparam int BUS_W     = 64;
  localparam int BPB       = BUS_W / 8;                 // bytes (pixels) per beat
  localparam int L         = TOTAL_LAYERS - 1;          // non-input layers
  localparam int F0        = TOPOLOGY[0];
  localparam int IMG_BEATS = (F0 + BPB - 1) / BPB;

  localparam int ORD_NORMAL  = 0;  // L0 W, L0 T, L1 W, ... (provided-TB order)
  localparam int ORD_REVERSE = 1;  // thresholds in reverse layer order, then weights in reverse order
  localparam int ORD_SHUFFLE = 2;

  localparam int RDY_ALWAYS = 0;
  localparam int RDY_RANDOM = 1;
  localparam int RDY_ZERO   = 2;

  typedef BNN_FCC_Model #(BUS_W) model_t;
  typedef bit [7:0] img_t[];
  typedef bit [7:0] byteq_t[$];

  // ---------------- DUT
  logic clk = 1'b0;
  logic rst = 1'b1;
  initial begin
    wait (go);
    forever #5 clk = ~clk;
  end

  logic               config_valid = 1'b0, config_ready, config_last = 1'b0;
  logic [BUS_W-1:0]   config_data = '0;
  logic [BPB-1:0]     config_keep = '0;
  logic               data_in_valid = 1'b0, data_in_ready, data_in_last = 1'b0;
  logic [BUS_W-1:0]   data_in_data = '0;
  logic [BPB-1:0]     data_in_keep = '0;
  logic               data_out_valid, data_out_ready = 1'b0, data_out_last;
  logic [7:0]         data_out_data;
  logic [0:0]         data_out_keep;

  bnn_fcc #(
    .INPUT_DATA_WIDTH(8), .INPUT_BUS_WIDTH(BUS_W), .CONFIG_BUS_WIDTH(BUS_W),
    .OUTPUT_DATA_WIDTH(4), .OUTPUT_BUS_WIDTH(8),
    .TOTAL_LAYERS(TOTAL_LAYERS), .TOPOLOGY(TOPOLOGY),
    .PARALLEL_INPUTS(PARALLEL_INPUTS), .PARALLEL_NEURONS(PARALLEL_NEURONS)
  ) dut (.*);

  // ---------------- TB state
  int         errors = 0;
  longint     cyc = 0;
  int         img_acc = 0;       // images whose first beat was accepted since reset
  int         out_cnt = 0;       // outputs since reset
  int         cfg_beats = 0;     // config beats accepted since reset
  bit         mon_in_mid = 1'b0;
  longint     img_cyc_q[$];      // cycle of each image's first accepted beat since reset
  longint     cfg_cyc_q[$];      // cycle of each accepted config beat since reset
  bit         release_img = 1'b0; // send_image(.., pause_at) waits for this before beat pause_at
  bit         ov_hold = 1'b0;
  logic [7:0] ov_data;
  int         ready_mode = RDY_RANDOM;
  int         cfg_pv = 800;      // valid probability (permille) of the config driver
  int         img_pv = 800;      // valid probability (permille) of the image driver (zero_gap = 0)

  img_t       img_q[$];          // images sent, not yet checked
  int         got_q[$];          // outputs received, not yet checked
  longint     out_cyc_q[$];      // cycle of every output handshake
  img_t       batch[$];
  bit [7:0]   mask_q[$];         // per checked output: bit j = matched model_q[j]
  byteq_t     mb, tq;
  model_t::bus_stream_t  ms;
  model_t::keep_stream_t mk;
  int         test_err0;
  string      test_name = "";

  function automatic void fail(string s);
    $error("[%s] %s", NAME, s);
    errors++;
  endfunction

  function automatic bit coin(int permille);
    return $urandom_range(999) < permille;
  endfunction

  function automatic int cdiv_i(int a, int b);
    return (a + b - 1) / b;
  endfunction

  function automatic int imin_i(int a, int b);
    return (a < b) ? a : b;
  endfunction

  // ---------------- monitor
  always @(posedge clk) begin
    if (rst) begin
      img_acc    = 0;
      out_cnt    = 0;
      cfg_beats  = 0;
      mon_in_mid = 1'b0;
      ov_hold    = 1'b0;
      img_cyc_q.delete();
      cfg_cyc_q.delete();
    end else begin
      if (config_valid && config_ready) begin
        cfg_beats++;
        cfg_cyc_q.push_back(cyc);
      end
      if (data_in_valid && data_in_ready) begin
        if (!mon_in_mid) begin
          img_acc++;
          img_cyc_q.push_back(cyc);
        end
        mon_in_mid = !data_in_last;
      end
      if ($isunknown(data_out_valid)) fail("data_out_valid is X");
      if (ov_hold) begin
        if (data_out_valid !== 1'b1) fail("data_out_valid dropped without a handshake");
        else if (data_out_data !== ov_data)
          fail($sformatf("data_out_data changed under backpressure: %0d -> %0d", ov_data, data_out_data));
      end
      if (data_out_valid && data_out_ready) begin
        if (data_out_keep !== 1'b1 || data_out_last !== 1'b1)
          fail($sformatf("output keep=%b last=%b", data_out_keep, data_out_last));
        if (out_cnt >= img_acc)
          fail($sformatf("extra output %0d (data %0d): only %0d images accepted since reset",
                         out_cnt, data_out_data, img_acc));
        out_cnt++;
        got_q.push_back(int'(data_out_data));
        out_cyc_q.push_back(cyc);
      end
      ov_hold = data_out_valid && !data_out_ready;
      ov_data = data_out_data;
    end
    case (ready_mode)
      RDY_ALWAYS: data_out_ready <= 1'b1;
      RDY_ZERO:   data_out_ready <= 1'b0;
      default:    data_out_ready <= coin(700);
    endcase
    cyc++;
  end

  // ---------------- models
  function automatic int rand_threshold(int f);
    int r = $urandom_range(99);
    if (r < 80) return $urandom_range(2 * f / 3, f / 3);
    if (r < 94) return $urandom_range(f, 0);
    case ($urandom_range(4))
      0: return 0;
      1: return f;
      2: return f + 1;
      3: return f + 7;
      default: return -2;  // negative: the DUT clamps to 0, the model compares signed: always 1
    endcase
  endfunction

  // random model; retried until it is not degenerate (>= 3 distinct classes over 16 random images),
  // so replacing a layer can change classifications
  function automatic model_t new_random_model();
    model_t m;
    bit     seen[int];
    img_t   img;
    for (int t = 0; t < 100; t++) begin
      m = new_random_model_raw();
      seen.delete();
      for (int i = 0; i < 16; i++) begin
        img = rand_image();
        seen[m.compute_reference(img)] = 1'b1;
      end
      if (seen.num() >= 3) return m;
    end
    fail("new_random_model: only degenerate models");
    return m;
  endfunction

  function automatic model_t new_random_model_raw();
    model_t m = new();
    m.num_layers = L;
    m.topology   = new[TOTAL_LAYERS];
    foreach (m.topology[i]) m.topology[i] = TOPOLOGY[i];
    m.weight    = new[L];
    m.threshold = new[L];
    for (int l = 0; l < L; l++) begin
      m.weight[l]    = new[TOPOLOGY[l+1]];
      m.threshold[l] = new[TOPOLOGY[l+1]];
      for (int n = 0; n < TOPOLOGY[l+1]; n++) begin
        m.weight[l][n] = new[TOPOLOGY[l]];
        for (int i = 0; i < TOPOLOGY[l]; i++) m.weight[l][n][i] = 1'($urandom);
        m.threshold[l][n] = (l == L - 1) ? 0 : rand_threshold(TOPOLOGY[l]);
      end
    end
    m.is_loaded     = 1;
    m.outputs_valid = 0;
    return m;
  endfunction

  function automatic model_t copy_model(model_t src);
    model_t m = new();
    m.num_layers = src.num_layers;
    m.topology   = src.topology;
    m.weight     = new[L];
    m.threshold  = new[L];
    for (int l = 0; l < L; l++) begin
      m.weight[l] = new[TOPOLOGY[l+1]];
      for (int n = 0; n < TOPOLOGY[l+1]; n++) m.weight[l][n] = src.weight[l][n];
      m.threshold[l] = src.threshold[l];
    end
    m.is_loaded     = 1;
    m.outputs_valid = 0;
    return m;
  endfunction

  function automatic void replace_layer(model_t dst, model_t src, int l, bit thr);
    if (thr) dst.threshold[l] = src.threshold[l];
    else for (int n = 0; n < TOPOLOGY[l+1]; n++) dst.weight[l][n] = src.weight[l][n];
  endfunction

  // number of images of the current batch classified differently by a and b
  function automatic int n_diff(model_t a, model_t b);
    int d = 0;
    foreach (batch[i]) if (a.compute_reference(batch[i]) != b.compute_reference(batch[i])) d++;
    return d;
  endfunction

  // copy of a with layer l (weights or thresholds), and optionally layer l2, from a fresh random model;
  // retried until at least `need` images of the current batch classify differently
  function automatic model_t variant(model_t a, int l, bit thr, int need, int l2 = -1, bit thr2 = 0);
    model_t b, fresh;
    for (int t = 0; t < 100; t++) begin
      fresh = new_random_model();
      b     = copy_model(a);
      replace_layer(b, fresh, l, thr);
      if (l2 >= 0) replace_layer(b, fresh, l2, thr2);
      if (n_diff(a, b) >= need) return b;
    end
    fail($sformatf("variant: no model differing on %0d images (layer %0d thr %0d)", need, l, thr));
    return b;
  endfunction

  function automatic img_t rand_image();
    img_t img;
    int   r = $urandom_range(99);
    img = new[F0];
    foreach (img[i]) img[i] = (r < 4) ? 8'h00 : (r < 8) ? 8'hFF : 8'($urandom);
    return img;
  endfunction

  function automatic void gen_batch(int n);
    batch.delete();
    for (int i = 0; i < n; i++) batch.push_back(rand_image());
  endfunction

  // ---------------- message building
  // fill_tail: the unused lanes of the last beat carry random bytes with TKEEP = 1 (the DUT must drop
  // the rest of the beat after a message's last byte)
  function automatic void pack_bytes(byteq_t bq, output model_t::bus_stream_t s,
                                     output model_t::keep_stream_t k, input bit fill_tail = 1'b0);
    int nb = (bq.size() + BPB - 1) / BPB;
    s = new[nb];
    k = new[nb];
    for (int b = 0; b < nb; b++) begin
      s[b] = '0;
      k[b] = '0;
    end
    foreach (bq[i]) begin
      s[i / BPB][(i % BPB)*8 +: 8] = bq[i];
      k[i / BPB][i % BPB]          = 1'b1;
    end
    if (fill_tail)
      for (int i = bq.size(); i < nb * BPB; i++) begin
        s[nb-1][(i % BPB)*8 +: 8] = 8'($urandom);
        k[nb-1][i % BPB]          = 1'b1;
      end
  endfunction

  // appends the kept bytes of a beat stream to tq
  function automatic void stream_to_bytes(model_t::bus_stream_t s, model_t::keep_stream_t k);
    foreach (s[b])
      for (int i = 0; i < BPB; i++) if (k[b][i]) tq.push_back(s[b][i*8 +: 8]);
  endfunction

  // appends a 16-byte header (and total random payload bytes if with_payload) to mb
  function automatic void hdr(int typ, int layer, int inputs, int neurons, int bpn, int total,
                              int rsv = 0, bit with_payload = 1);
    bit [127:0] h;
    h = '0;
    h[7:0]    = 8'(typ);
    h[15:8]   = 8'(layer);
    h[31:16]  = 16'(inputs);
    h[47:32]  = 16'(neurons);
    h[63:48]  = 16'(bpn);
    h[95:64]  = 32'(total);
    h[127:96] = 32'(rsv);
    for (int i = 0; i < 16; i++) mb.push_back(h[i*8 +: 8]);
    if (with_payload) for (int i = 0; i < total; i++) mb.push_back(8'($urandom));
  endfunction

  // invalid message k into mb; returns 0 when k is past the last kind
  function automatic bit build_invalid(int k, output string desc);
    int f0 = TOPOLOGY[0], n0 = TOPOLOGY[1], b0 = (TOPOLOGY[0] + 7) / 8;
    int f1 = TOPOLOGY[1], n1 = TOPOLOGY[2], b1 = (TOPOLOGY[1] + 7) / 8;
    int no = TOPOLOGY[L];
    model_t x;
    model_t::bus_stream_t s;
    model_t::keep_stream_t kp;
    mb.delete();
    case (k)
      0:  begin desc = "msg_type 2 (layer-0 weight fields)";            hdr(2, 0, f0, n0, b0, n0*b0); end
      1:  begin desc = "msg_type 2 with total_bytes 0";                 hdr(2, 0, f0, n0, b0, 0); end
      2:  begin desc = "msg_type 0x80 (layer-0 threshold fields)";      hdr(8'h80, 0, 32, n0, 4, 4*n0); end
      3:  begin desc = "weights layer_id L (beyond range)";             hdr(0, L, f0, n0, b0, n0*b0); end
      4:  begin desc = "weights layer_id 255";                          hdr(0, 255, f0, n0, b0, n0*b0); end
      5:  begin desc = "weights layer-0 dimensions with layer_id 1";    hdr(0, 1, f0, n0, b0, n0*b0); end
      6:  begin desc = "weights layer-1 dimensions with layer_id 0";    hdr(0, 0, f1, n1, b1, n1*b1); end
      7:  begin desc = "thresholds layer-1 N with layer_id 0";          hdr(1, 0, 32, n1, 4, 4*n1); end
      8:  begin desc = "thresholds layer-0 N with layer_id 1";          hdr(1, 1, 32, n0, 4, 4*n0); end
      9:  begin desc = "thresholds for the output layer";               hdr(1, L-1, 32, no, 4, 4*no); end
      10: begin desc = "thresholds layer_id L (beyond range)";          hdr(1, L, 32, n0, 4, 4*n0); end
      11: begin desc = "weights layer 0, layer_inputs F+1 (bpn/total consistent)";
                hdr(0, 0, f0 + 1, n0, b0, n0*b0); end
      12: begin desc = "weights layer 0, layer_inputs F+256";           hdr(0, 0, f0 + 256, n0, b0, n0*b0); end
      13: begin desc = "weights layer 0, num_neurons N+1, total_bytes N*bpn";
                hdr(0, 0, f0, n0 + 1, b0, n0*b0); end
      14: begin desc = "weights layer 0, num_neurons N+1, total_bytes (N+1)*bpn";
                hdr(0, 0, f0, n0 + 1, b0, (n0 + 1)*b0); end
      15: begin desc = "weights layer 0, num_neurons N-1, total_bytes (N-1)*bpn";
                hdr(0, 0, f0, n0 - 1, b0, (n0 - 1)*b0); end
      16: begin desc = "weights layer 0, bytes_per_neuron bpn+1, total_bytes N*bpn";
                hdr(0, 0, f0, n0, b0 + 1, n0*b0); end
      17: begin desc = "weights layer 0, bytes_per_neuron bpn+1, total_bytes N*(bpn+1)";
                hdr(0, 0, f0, n0, b0 + 1, n0*(b0 + 1)); end
      18: begin desc = "weights layer 0, total_bytes N*bpn+1";         hdr(0, 0, f0, n0, b0, n0*b0 + 1); end
      19: begin desc = "weights layer 0, total_bytes N*bpn-1";         hdr(0, 0, f0, n0, b0, n0*b0 - 1); end
      20: begin desc = "weights layer 0, total_bytes N*bpn+256";       hdr(0, 0, f0, n0, b0, n0*b0 + 256); end
      21: begin desc = "thresholds layer 0, num_neurons N+1, total_bytes 4N";
                hdr(1, 0, 32, n0 + 1, 4, 4*n0); end
      22: begin desc = "thresholds layer 0, bytes_per_neuron 2, total_bytes 4N";
                hdr(1, 0, 32, n0, 2, 4*n0); end
      23: begin desc = "thresholds layer 0, bytes_per_neuron 2, total_bytes 2N";
                hdr(1, 0, 32, n0, 2, 2*n0); end
      24: begin desc = "thresholds layer 0, total_bytes 4N+4";         hdr(1, 0, 32, n0, 4, 4*n0 + 4); end
      25: begin desc = "thresholds layer 0, total_bytes 4N-1";         hdr(1, 0, 32, n0, 4, 4*n0 - 1); end
      26: begin
        desc = "msg_type 2 whose payload is a complete valid layer-0 weights message";
        x = new_random_model();
        x.get_layer_config(0, 0, s, kp);
        tq.delete();
        stream_to_bytes(s, kp);
        hdr(2, 0, f0, n0, b0, tq.size(), 0, 0);
        foreach (tq[i]) mb.push_back(tq[i]);
      end
      default: return 0;
    endcase
    return 1;
  endfunction

  function automatic int config_beats_of(model_t m);
    model_t::bus_stream_t s;
    model_t::keep_stream_t k;
    int n = 0;
    for (int l = 0; l < L; l++) begin
      m.get_layer_config(l, 0, s, k);
      n += s.size();
      if (l < L - 1) begin
        m.get_layer_config(l, 1, s, k);
        n += s.size();
      end
    end
    return n;
  endfunction

  // ---------------- drivers
  task automatic send_msg(model_t::bus_stream_t stream, model_t::keep_stream_t keep, bit tlast_on_last_beat);
    int stall;
    for (int i = 0; i < stream.size(); i++) begin
      while (!coin(cfg_pv)) begin
        config_valid <= 1'b0;
        @(posedge clk);
      end
      config_valid <= 1'b1;
      config_data  <= stream[i];
      config_keep  <= keep[i];
      config_last  <= tlast_on_last_beat && (i == stream.size() - 1);
      stall = 0;
      do begin
        @(posedge clk);
        if (++stall == 50000) begin
          fail($sformatf("%s: config beat %0d of %0d not accepted for %0d cycles", test_name, i,
                         stream.size(), stall));
          config_valid <= 1'b0;
          config_last  <= 1'b0;
          return;
        end
      end while (!config_ready);
    end
    config_valid <= 1'b0;
    config_last  <= 1'b0;
    #1;  // return after the monitor has recorded the last handshake
  endtask

  task automatic send_layer_msg(model_t m, int l, bit thr, bit tlast);
    model_t::bus_stream_t s;
    model_t::keep_stream_t k;
    m.get_layer_config(l, thr, s, k);
    send_msg(s, k, tlast);
  endtask

  // all required messages (except code `omit`; code = 2*layer + is_threshold); TLAST on the last beat of
  // the last message, and of every message if per_msg_tlast
  task automatic send_config(model_t m, int order, bit per_msg_tlast, int omit = -1);
    int codes[$];
    int j, t;
    codes.delete();
    case (order)
      ORD_REVERSE: begin
        for (int l = L - 2; l >= 0; l--) codes.push_back(2*l + 1);
        for (int l = L - 1; l >= 0; l--) codes.push_back(2*l);
      end
      default: begin
        for (int l = 0; l < L; l++) begin
          codes.push_back(2*l);
          if (l < L - 1) codes.push_back(2*l + 1);
        end
      end
    endcase
    if (order == ORD_SHUFFLE)
      for (int i = codes.size() - 1; i > 0; i--) begin
        j = $urandom_range(i);
        t = codes[i]; codes[i] = codes[j]; codes[j] = t;
      end
    for (int i = 0; i < codes.size(); i++) if (codes[i] == omit) begin
      codes.delete(i);
      break;
    end
    foreach (codes[i]) send_layer_msg(m, codes[i] / 2, codes[i] % 2, per_msg_tlast || i == codes.size() - 1);
  endtask

  task automatic send_image(img_t img, bit zero_gap, int pause_at = -1);
    int stall;
    for (int b = 0; b < IMG_BEATS; b++) begin
      if (b == pause_at) begin
        data_in_valid <= 1'b0;
        data_in_last  <= 1'b0;
        @(posedge clk);
        wait (release_img);
      end
      if (!zero_gap)
        while (!coin(img_pv)) begin
          data_in_valid <= 1'b0;
          @(posedge clk);
        end
      data_in_valid <= 1'b1;
      for (int k = 0; k < BPB; k++) begin
        data_in_data[k*8 +: 8] <= (b*BPB + k < F0) ? img[b*BPB + k] : 8'($urandom);
        data_in_keep[k]        <= (b*BPB + k < F0);
      end
      data_in_last <= (b == IMG_BEATS - 1);
      stall = 0;
      do begin
        @(posedge clk);
        if (++stall == 50000) begin
          fail($sformatf("%s: image beat %0d not accepted for %0d cycles", test_name, b, stall));
          data_in_valid <= 1'b0;
          data_in_last  <= 1'b0;
          return;
        end
      end while (!data_in_ready);
    end
    data_in_valid <= 1'b0;
    data_in_last  <= 1'b0;
    img_q.push_back(img);
    #1;  // return after the monitor has recorded the last handshake (still before the next edge)
  endtask

  task automatic send_batch(bit zero_gap);
    foreach (batch[i]) send_image(batch[i], zero_gap);
  endtask

  // ---------------- checks
  task automatic expect_outputs(model_t model_q[$], int n, string tag);
    img_t     img;
    int       got, r;
    bit [7:0] m;
    longint   t0;
    string    refs;
    if (n < 0) n = img_q.size();
    if (n > img_q.size()) begin
      fail($sformatf("%s: expect %0d outputs but only %0d images sent", tag, n, img_q.size()));
      n = img_q.size();
    end
    t0 = cyc;
    while (got_q.size() < n) begin
      @(posedge clk);
      #1;
      if (cyc - t0 > 20000 + 2000 * n) begin
        fail($sformatf("%s: timeout, %0d/%0d outputs (data_in_ready=%b data_out_valid=%b)",
                       tag, got_q.size(), n, data_in_ready, data_out_valid));
        img_q.delete();
        got_q.delete();
        return;
      end
    end
    mask_q.delete();
    for (int i = 0; i < n; i++) begin
      img  = img_q.pop_front();
      got  = got_q.pop_front();
      m    = '0;
      refs = "";
      foreach (model_q[j]) begin
        r = model_q[j].compute_reference(img);
        refs = {refs, $sformatf(" %0d", r)};
        if (r == got) m[j] = 1'b1;
      end
      if (m == '0) fail($sformatf("%s: output %0d = %0d, expected one of{%s }", tag, i, got, refs));
      mask_q.push_back(m);
    end
  endtask

  task automatic expect1(model_t a, int n, string tag);
    model_t q[$];
    q.delete();
    q.push_back(a);
    expect_outputs(q, n, tag);
  endtask

  task automatic check_quiet(int n, string tag);
    repeat (n) @(posedge clk);
    #1;
    if (got_q.size() != 0) fail($sformatf("%s: %0d unexpected outputs", tag, got_q.size()));
    if (img_q.size() != 0) fail($sformatf("%s: %0d images without output", tag, img_q.size()));
    got_q.delete();
    img_q.delete();
  endtask

  task automatic wait_in_ready(string tag, int n = 2000);
    for (int i = 0; i < n; i++) begin
      @(posedge clk);
      if (data_in_ready) return;
    end
    fail($sformatf("%s: data_in_ready did not rise within %0d cycles", tag, n));
  endtask

  // offers an image beat for n cycles; data_in_ready must stay 0
  task automatic expect_not_ready(int n, string tag);
    data_in_valid <= 1'b1;
    data_in_data  <= {2{32'($urandom)}};
    data_in_keep  <= '1;
    data_in_last  <= 1'b0;
    for (int i = 0; i < n; i++) begin
      @(posedge clk);
      if (data_in_ready) begin
        fail($sformatf("%s: data_in_ready = 1 after %0d cycles", tag, i));
        break;
      end
    end
    data_in_valid <= 1'b0;
  endtask

  task automatic do_reset(int ncyc = 5);
    rst           <= 1'b1;
    config_valid  <= 1'b0;
    config_last   <= 1'b0;
    data_in_valid <= 1'b0;
    data_in_last  <= 1'b0;
    repeat (ncyc) @(posedge clk);
    img_q.delete();
    got_q.delete();
    out_cyc_q.delete();
    rst <= 1'b0;
  endtask

  task automatic test_begin(string name);
    test_name  = name;
    test_err0  = errors;
    ready_mode = RDY_RANDOM;
    do_reset();
    @(posedge clk);
  endtask

  task automatic test_end();
    if (errors == test_err0) $display("PASS %s [%s]", test_name, NAME);
    else $display("FAILED %s [%s]: %0d errors", test_name, NAME, errors - test_err0);
  endtask

  // ---------------- tests
  task automatic test_reordered();
    model_t a;
    test_begin("test_reordered");
    a = new_random_model();
    send_config(a, ORD_REVERSE, 1'b0);
    wait_in_ready("test_reordered");
    gen_batch(50);
    send_batch(1'b0);
    expect1(a, -1, "test_reordered");
    check_quiet(100, "test_reordered");
    test_end();
  endtask

  task automatic test_not_configured();
    model_t a;
    string  tag;
    test_begin("test_not_configured");
    for (int c = 0; c < 2*L - 1; c++) begin
      tag = $sformatf("test_not_configured(omit layer %0d %s)", c / 2, (c % 2) ? "thresholds" : "weights");
      do_reset();
      a = new_random_model();
      send_config(a, ORD_SHUFFLE, 1'b0, c);
      expect_not_ready(1000, tag);
      send_layer_msg(a, c / 2, c % 2, 1'b1);
      wait_in_ready(tag, 200);
      gen_batch(5);
      send_batch(1'b0);
      expect1(a, -1, tag);
      check_quiet(50, tag);
    end
    test_end();
  endtask

  // after 20 images, layer l weights (thr = 0) or thresholds (thr = 1) only are replaced without
  // waiting for the 20 outputs; the next 20 images use the new model
  task automatic reconfig_one(string name, int l, bit thr);
    model_t a, b;
    test_begin(name);
    a = new_random_model();
    send_config(a, ORD_NORMAL, 1'b0);
    wait_in_ready(name);
    gen_batch(20);
    send_batch(1'b0);
    gen_batch(20);
    b = variant(a, l, thr, 3);
    send_layer_msg(b, l, thr, 1'b1);
    send_batch(1'b0);
    expect1(a, 20, {name, " (model A)"});
    expect1(b, 20, {name, " (model B)"});
    check_quiet(100, name);
    test_end();
  endtask

  // Spec rule: an image whose first beat was accepted before the reconfiguration header was validated
  // uses the old model; images accepted after the message use the new one. Images cannot be accepted
  // while the message is pending, so the cycle of the message's last config beat (cfg_cyc_q[cfg_last])
  // separates the two. mask_q[i] (from expect_outputs with {old, new}) belongs to image img_base + i.
  function automatic void check_switch(int img_base, int cfg_last, string tag);
    longint tc, c;
    tc = cfg_cyc_q[cfg_last];
    foreach (mask_q[i]) begin
      c = img_cyc_q[img_base + i];
      if (c < tc && !mask_q[i][0])
        fail($sformatf("%s: image %0d accepted at %0d, before the message (last beat %0d), does not match the old model",
                       tag, img_base + i, c, tc));
      if (c > tc && !mask_q[i][1])
        fail($sformatf("%s: image %0d accepted at %0d, after the message (last beat %0d), does not match the new model",
                       tag, img_base + i, c, tc));
    end
  endfunction

  // new model differing from cur in layer-0 weights (only neuron 0's if n0_only) that classifies the
  // image batch[0] differently; batch[0] is redrawn every 20 tries, and n0_only is dropped after 300
  // (neuron 0 may be insensitive, e.g. an extreme threshold)
  function automatic model_t variant_on_first(model_t cur, bit n0_only = 1'b0);
    model_t nxt;
    for (int t = 0; t < 500; t++) begin
      if (t % 20 == 19) batch[0] = rand_image();
      if (n0_only && t < 300) begin
        nxt = copy_model(cur);
        for (int i = 0; i < TOPOLOGY[0]; i++) nxt.weight[0][0][i] = 1'($urandom);
      end else begin
        nxt = variant(cur, 0, 1'b0, 0);
      end
      if (nxt.compute_reference(batch[0]) != cur.compute_reference(batch[0])) return nxt;
    end
    fail("variant_on_first: no distinguishing model");
    return nxt;
  endfunction

  // Phase 1: images stream back to back while a layer-2 weights message is sent.
  // Phase 2: one image is offered d = 0..23 cycles after the header's second beat (sweeping the header
  //          validation / first-WAIT-cycle window) while a layer-0 weights message is sent gap-free;
  //          the new model differs only in layer-0 neuron 0 (the first weights written), so an early
  //          write is visible as a new-model result; three rounds.
  // Phase 3: an image's first beat is accepted, then the image pauses mid-way while a layer-0 weights
  //          header arrives; it must complete with the old model before the message is written.
  task automatic test_reconfig_while_streaming();
    model_t   a, b, cur, nxt;
    model_t   mq[$];
    bit       seen_b;
    int       n_a, n_b, nbeats, cbase, ibase;
    model_t::bus_stream_t s;
    model_t::keep_stream_t kp;
    localparam int NIMG = 60, NPRE = 15;
    test_begin("test_reconfig_while_streaming");
    a = new_random_model();
    send_config(a, ORD_NORMAL, 1'b0);
    wait_in_ready("test_reconfig_while_streaming");
    gen_batch(NIMG);
    b = variant(a, 2, 1'b0, NIMG / 3);
    b.get_layer_config(2, 0, s, kp);
    nbeats = s.size();
    cbase  = cfg_beats;
    fork
      send_batch(1'b1);
      begin
        wait (img_acc >= NPRE);
        send_layer_msg(b, 2, 1'b0, 1'b1);
      end
    join
    mq.delete();
    mq.push_back(a);
    mq.push_back(b);
    expect_outputs(mq, NIMG, "test_reconfig_while_streaming");
    check_switch(0, cbase + nbeats - 1, "test_reconfig_while_streaming (stream)");
    seen_b = 0;
    n_a    = 0;
    n_b    = 0;
    foreach (mask_q[i]) begin
      if (mask_q[i] == 8'b01) begin
        n_a++;
        if (seen_b) fail($sformatf("test_reconfig_while_streaming: output %0d matches only A after a B-only output", i));
      end
      if (mask_q[i] == 8'b10) begin
        n_b++;
        seen_b = 1;
        if (i < NPRE) fail($sformatf("test_reconfig_while_streaming: output %0d (sent before the message) matches only B", i));
      end
    end
    if (n_a == 0 || n_b == 0)
      fail($sformatf("test_reconfig_while_streaming: A-only %0d, B-only %0d (both must occur)", n_a, n_b));
    $display("[%s] test_reconfig_while_streaming: %0d A-only, %0d B-only, %0d both", NAME, n_a, n_b, NIMG - n_a - n_b);
    check_quiet(100, "test_reconfig_while_streaming (stream)");

    // phase 2
    cur    = b;
    cfg_pv = 1000;
    b.get_layer_config(0, 0, s, kp);
    nbeats = s.size();
    for (int it = 0; it < 3 * 24; it++) begin
      int d = it % 24;
      gen_batch(1);
      nxt   = variant_on_first(cur, 1'b1);
      cbase = cfg_beats;
      ibase = img_acc;
      fork
        send_layer_msg(nxt, 0, 1'b0, 1'b1);
        begin
          wait (cfg_beats >= cbase + 2);
          repeat (d) @(posedge clk);
          send_image(batch[0], 1'b1);
        end
      join
      mq.delete();
      mq.push_back(cur);
      mq.push_back(nxt);
      expect_outputs(mq, 1, $sformatf("test_reconfig_while_streaming (sweep d=%0d)", d));
      check_switch(ibase, cbase + nbeats - 1, $sformatf("test_reconfig_while_streaming (sweep d=%0d)", d));
      cur = nxt;
    end
    cfg_pv = 800;

    // phase 3
    gen_batch(4);
    nxt         = variant_on_first(cur);
    cbase       = cfg_beats;
    ibase       = img_acc;
    release_img = 1'b0;
    fork
      send_image(batch[0], 1'b1, 1);
      begin
        wait (img_acc >= ibase + 1);
        send_layer_msg(nxt, 0, 1'b0, 1'b1);
      end
      begin
        wait (cfg_beats >= cbase + 2);
        repeat (40) @(posedge clk);
        release_img = 1'b1;
      end
    join
    release_img = 1'b0;
    for (int i = 1; i < 4; i++) send_image(batch[i], 1'b0);
    mq.delete();
    mq.push_back(cur);
    mq.push_back(nxt);
    expect_outputs(mq, 4, "test_reconfig_while_streaming (image paused across the header)");
    check_switch(ibase, cbase + nbeats - 1, "test_reconfig_while_streaming (image paused across the header)");
    check_quiet(100, "test_reconfig_while_streaming");
    test_end();
  endtask

  task automatic test_invalid_headers();
    model_t a, c, cur, nxt;
    int     lyr;
    bit     thr;
    string  desc;
    model_t::bus_stream_t s;
    model_t::keep_stream_t k;
    int     k_inv;
    test_begin("test_invalid_headers");
    a = new_random_model();
    send_config(a, ORD_NORMAL, 1'b0);
    wait_in_ready("test_invalid_headers");
    // Each invalid message (odd kinds end with TLAST; kinds with k % 3 != 2 have TKEEP=1 garbage after
    // the payload in the last beat) is followed by 3 images that must match the current model. Every
    // 4th one is directly followed by a valid message that changes the model (so a misaligned header
    // after a discard is detected).
    cur   = a;
    k_inv = 0;
    while (build_invalid(k_inv, desc)) begin
      pack_bytes(mb, ms, mk, k_inv % 3 != 2);
      send_msg(ms, mk, k_inv % 2);
      gen_batch(3);
      if (k_inv % 4 == 0) begin
        lyr = (k_inv / 4) % L;
        thr = (lyr < L - 1) && ((k_inv / 4) % 2 == 1);
        nxt = variant(cur, lyr, thr, 1);
        send_layer_msg(nxt, lyr, thr, 1'b0);
        desc = $sformatf("%s, then valid layer-%0d %s", desc, lyr, thr ? "thresholds" : "weights");
        cur = nxt;
      end
      send_batch(1'b0);
      expect1(cur, -1, {"test_invalid_headers after: ", desc});
      k_inv++;
    end
    a = cur;
    gen_batch(20);
    send_batch(1'b0);
    expect1(a, -1, "test_invalid_headers (20 images after all invalid headers)");
    $display("[%s] test_invalid_headers: %0d invalid kinds discarded", NAME, k_inv);

    // valid messages with fields that must be ignored: thresholds with layer_inputs 12345 and
    // reserved != 0, weights with reserved != 0 -> both accepted (last beats padded with TKEEP=1 garbage)
    gen_batch(10);
    c = variant(a, 0, 1'b1, 2, 1, 1'b0);
    c.get_layer_config(0, 1, s, k);
    tq.delete();
    stream_to_bytes(s, k);
    tq[2]  = 8'h39; tq[3]  = 8'h30;
    tq[12] = 8'hEF; tq[13] = 8'hBE; tq[14] = 8'hAD; tq[15] = 8'hDE;
    pack_bytes(tq, ms, mk, 1'b1);
    send_msg(ms, mk, 1'b0);
    c.get_layer_config(1, 0, s, k);
    tq.delete();
    stream_to_bytes(s, k);
    tq[12] = 8'h01; tq[13] = 8'h02; tq[14] = 8'h03; tq[15] = 8'h04;
    pack_bytes(tq, ms, mk, 1'b1);
    send_msg(ms, mk, 1'b1);
    send_batch(1'b0);
    expect1(c, -1, "test_invalid_headers (valid messages with ignored fields set)");
    check_quiet(100, "test_invalid_headers");
    test_end();
  endtask

  // first k beats of a message, TLAST on beat k-1
  task automatic send_truncated(model_t m, int l, bit thr, int k);
    model_t::bus_stream_t s, ts;
    model_t::keep_stream_t kp, tk;
    m.get_layer_config(l, thr, s, kp);
    ts = new[k];
    tk = new[k];
    for (int i = 0; i < k; i++) begin
      ts[i] = s[i];
      tk[i] = kp[i];
    end
    send_msg(ts, tk, 1'b1);
  endtask

  task automatic test_truncated();
    model_t a, b, c;
    model_t::bus_stream_t s;
    model_t::keep_stream_t kp;
    int nb, kt;
    test_begin("test_truncated");
    a = new_random_model();
    send_config(a, ORD_NORMAL, 1'b0);
    wait_in_ready("test_truncated");
    gen_batch(5);
    send_batch(1'b0);
    expect1(a, -1, "test_truncated (initial)");

    // TLAST inside a header (8 of 16 bytes) and on the 16th header byte: header discarded, model kept
    gen_batch(5);
    b = variant(a, 0, 1'b0, 2);
    send_truncated(b, 0, 1'b0, 1);
    send_truncated(b, 0, 1'b0, 2);
    send_batch(1'b0);
    expect1(a, -1, "test_truncated (header truncated: model unchanged)");

    // layer-0 weights truncated mid-payload -> images blocked until the full message is resent
    b.get_layer_config(0, 0, s, kp);
    nb = s.size();
    kt = $urandom_range(nb - 1, 3);
    send_truncated(b, 0, 1'b0, kt);
    repeat (BPB + 10) @(posedge clk);
    if (data_in_ready) fail($sformatf("test_truncated: data_in_ready still 1 %0d cycles after the truncated message", BPB + 10));
    expect_not_ready(1000, $sformatf("test_truncated (weights, %0d of %0d beats)", kt, nb));
    send_layer_msg(b, 0, 1'b0, 1'b1);
    wait_in_ready("test_truncated (weights resent)", 200);
    send_batch(1'b0);
    expect1(b, -1, "test_truncated (weights resent)");

    // same for layer-0 thresholds
    gen_batch(5);
    c = variant(b, 0, 1'b1, 2);
    c.get_layer_config(0, 1, s, kp);
    nb = s.size();
    kt = $urandom_range(nb - 1, 3);
    send_truncated(c, 0, 1'b1, kt);
    repeat (BPB + 10) @(posedge clk);
    if (data_in_ready) fail("test_truncated: data_in_ready still 1 after a truncated threshold message");
    expect_not_ready(1000, $sformatf("test_truncated (thresholds, %0d of %0d beats)", kt, nb));
    send_layer_msg(c, 0, 1'b1, 1'b0);
    wait_in_ready("test_truncated (thresholds resent)", 200);
    send_batch(1'b0);
    expect1(c, -1, "test_truncated (thresholds resent)");
    check_quiet(100, "test_truncated");
    test_end();
  endtask

  task automatic test_per_message_tlast();
    model_t a;
    test_begin("test_per_message_tlast");
    a = new_random_model();
    send_config(a, ORD_NORMAL, 1'b1);
    wait_in_ready("test_per_message_tlast");
    gen_batch(20);
    send_batch(1'b0);
    expect1(a, -1, "test_per_message_tlast");
    check_quiet(100, "test_per_message_tlast");
    test_end();
  endtask

  // v0/v1: reset during the initial configuration (early / late); v2: reset while a reconfiguration
  // message is being written with images streaming
  task automatic test_reset_mid_config();
    model_t a, b;
    int     total, target;
    string  tag;
    test_begin("test_reset_mid_config");
    for (int v = 0; v < 3; v++) begin
      tag = $sformatf("test_reset_mid_config(v%0d)", v);
      do_reset();
      a = new_random_model();
      if (v < 2) begin
        total  = config_beats_of(a);
        target = (v == 0) ? $urandom_range(total / 2, 1) : $urandom_range(total - 1, total / 2 + 1);
        fork
          send_config(a, ORD_NORMAL, 1'b0);
          wait (cfg_beats >= target);
        join_any
        disable fork;
      end else begin
        send_config(a, ORD_NORMAL, 1'b0);
        wait_in_ready(tag);
        gen_batch(20);
        b = new_random_model();
        fork
          send_batch(1'b0);
          begin
            wait (img_acc >= 5);
            target = cfg_beats + 3;   // header (2 beats) + first payload beat accepted
            fork
              send_layer_msg(b, 0, 1'b0, 1'b1);
            join_none
            wait (cfg_beats >= target);
          end
        join_any
        disable fork;
      end
      do_reset();
      expect_not_ready(200, {tag, " after reset"});
      check_quiet(10, tag);
      b = new_random_model();
      send_config(b, ORD_NORMAL, 1'b0);
      wait_in_ready(tag);
      gen_batch(20);
      send_batch(1'b0);
      expect1(b, -1, tag);
      check_quiet(100, tag);
    end
    test_end();
  endtask

  task automatic test_reset_mid_image();
    model_t a, b;
    test_begin("test_reset_mid_image");
    a = new_random_model();
    send_config(a, ORD_NORMAL, 1'b0);
    wait_in_ready("test_reset_mid_image");
    gen_batch(10);
    fork
      send_batch(1'b0);
      wait (img_acc >= 4 && mon_in_mid);
    join_any
    disable fork;
    if (!mon_in_mid) fail("test_reset_mid_image: not mid-image at reset");
    do_reset();
    expect_not_ready(200, "test_reset_mid_image after reset");
    check_quiet(300, "test_reset_mid_image after reset");
    b = new_random_model();
    send_config(b, ORD_SHUFFLE, 1'b0);
    wait_in_ready("test_reset_mid_image");
    gen_batch(20);
    send_batch(1'b0);
    expect1(b, -1, "test_reset_mid_image");
    check_quiet(100, "test_reset_mid_image");
    test_end();
  endtask

  task automatic test_zero_gap_images();
    model_t a;
    int     bound, lc, d, dmax;
    test_begin("test_zero_gap_images");
    ready_mode = RDY_ALWAYS;
    bound = IMG_BEATS;
    for (int l = 0; l < L; l++) begin
      lc = cdiv_i(TOPOLOGY[l+1], imin_i(PARALLEL_NEURONS[l], TOPOLOGY[l+1])) *
           cdiv_i(TOPOLOGY[l], imin_i(PARALLEL_INPUTS, TOPOLOGY[l]));
      if (lc > bound) bound = lc;
    end
    bound += 2;
    a = new_random_model();
    send_config(a, ORD_NORMAL, 1'b0);
    wait_in_ready("test_zero_gap_images");
    gen_batch(100);
    out_cyc_q.delete();
    send_batch(1'b1);
    expect1(a, -1, "test_zero_gap_images");
    if (out_cyc_q.size() != 100) fail($sformatf("test_zero_gap_images: %0d output times", out_cyc_q.size()));
    dmax = 0;
    for (int i = 10; i < out_cyc_q.size(); i++) begin
      d = int'(out_cyc_q[i] - out_cyc_q[i-1]);
      if (d > dmax) dmax = d;
      if (d > bound) fail($sformatf("test_zero_gap_images: outputs %0d and %0d are %0d cycles apart (bound %0d)", i - 1, i, d, bound));
    end
    $display("[%s] test_zero_gap_images: max steady-state output interval %0d cycles (bound %0d)", NAME, dmax, bound);
    check_quiet(100, "test_zero_gap_images");
    test_end();
  endtask

  task automatic test_long_backpressure();
    model_t a;
    int     low_run;
    test_begin("test_long_backpressure");
    a = new_random_model();
    send_config(a, ORD_NORMAL, 1'b0);
    wait_in_ready("test_long_backpressure");
    ready_mode = RDY_ZERO;
    repeat (2) @(posedge clk);
    gen_batch(30);
    fork
      send_batch(1'b1);
    join_none
    low_run = 0;
    repeat (5000) begin
      @(posedge clk);
      low_run = data_in_ready ? 0 : low_run + 1;
    end
    if (low_run < 1000) fail($sformatf("test_long_backpressure: data_in_ready low for only %0d final cycles", low_run));
    if (img_acc >= 30) fail("test_long_backpressure: all 30 images accepted while data_out_ready = 0");
    if (out_cnt != 0) fail("test_long_backpressure: output handshake while data_out_ready = 0");
    if (data_out_valid !== 1'b1) fail("test_long_backpressure: no output pending");
    $display("[%s] test_long_backpressure: %0d images accepted during 5000 stalled cycles", NAME, img_acc);
    ready_mode = RDY_ALWAYS;
    wait fork;
    expect1(a, 30, "test_long_backpressure");
    check_quiet(100, "test_long_backpressure");
    test_end();
  endtask

  task automatic test_reset_with_output_pending();
    model_t a, b;
    int     t;
    test_begin("test_reset_with_output_pending");
    a = new_random_model();
    send_config(a, ORD_NORMAL, 1'b0);
    wait_in_ready("test_reset_with_output_pending");
    ready_mode = RDY_ZERO;
    gen_batch(3);
    send_batch(1'b0);
    t = 0;
    while (data_out_valid !== 1'b1 && t < 5000) begin
      @(posedge clk);
      t++;
    end
    if (data_out_valid !== 1'b1) fail("test_reset_with_output_pending: no output became pending");
    repeat (20) @(posedge clk);
    do_reset(1);
    #1;
    if (data_out_valid !== 1'b0) fail("test_reset_with_output_pending: data_out_valid != 0 the cycle after reset");
    ready_mode = RDY_ALWAYS;
    for (int i = 0; i < 200; i++) begin
      @(posedge clk);
      if (data_out_valid !== 1'b0) begin
        fail("test_reset_with_output_pending: data_out_valid after reset");
        break;
      end
    end
    check_quiet(10, "test_reset_with_output_pending after reset");
    b = new_random_model();
    send_config(b, ORD_NORMAL, 1'b0);
    wait_in_ready("test_reset_with_output_pending");
    ready_mode = RDY_RANDOM;
    gen_batch(10);
    send_batch(1'b0);
    expect1(b, 10, "test_reset_with_output_pending");
    check_quiet(200, "test_reset_with_output_pending");
    test_end();
  endtask

  // ---------------- main
  initial done = 1'b0;
  initial nerr = 0;

  initial begin : main
    wait (go);
    process::self().srandom(SEED);
    $display("[%s] TOPOLOGY %p PARALLEL_INPUTS %0d PARALLEL_NEURONS %p", NAME, TOPOLOGY, PARALLEL_INPUTS, PARALLEL_NEURONS);
    test_reordered();
    test_not_configured();
    reconfig_one("test_weights_only_reconfig", 1, 1'b0);
    reconfig_one("test_thresholds_only_reconfig", 0, 1'b1);
    test_reconfig_while_streaming();
    test_invalid_headers();
    test_truncated();
    test_per_message_tlast();
    test_reset_mid_config();
    test_reset_mid_image();
    test_zero_gap_images();
    test_long_backpressure();
    test_reset_with_output_pending();
    $display("[%s] done: %0d errors, %0d cycles", NAME, errors, cyc);
    nerr = errors;
    done = 1'b1;
  end

  initial begin : watchdog
    wait (go);
    #(10ns * 3000000);
    if (!done) $fatal(1, "[%s] TIMEOUT in %s", NAME, test_name);
  end
endmodule

module tb_cfg_sequences;
  logic go4 = 1'b0, go5 = 1'b0;
  logic done4, done5;
  int   nerr4, nerr5;

  cfgseq_harness #(
    .NAME("topo4"), .TOTAL_LAYERS(4), .TOPOLOGY('{20, 12, 9, 5}),
    .PARALLEL_INPUTS(4), .PARALLEL_NEURONS('{3, 4, 2}), .SEED(13)
  ) u_topo4 (.go(go4), .done(done4), .nerr(nerr4));

  cfgseq_harness #(
    .NAME("topo5"), .TOTAL_LAYERS(5), .TOPOLOGY('{30, 17, 9, 12, 6}),
    .PARALLEL_INPUTS(5), .PARALLEL_NEURONS('{4, 9, 5, 6}), .SEED(29)
  ) u_topo5 (.go(go5), .done(done5), .nerr(nerr5));

  initial begin
    go4 = 1'b1;
    wait (done4 === 1'b1);
    go5 = 1'b1;
    wait (done5 === 1'b1);
    if (nerr4 == 0 && nerr5 == 0) $display("SUCCESS: tb_cfg_sequences");
    else $display("tb_cfg_sequences: %0d + %0d errors", nerr4, nerr5);
    $finish;
  end
endmodule
