// Self-checking testbench for bnn_layer over several (F, N, PW, PN, IS_OUTPUT) configurations.
// Each case: producer -> input pingpong_buf -> bnn_layer -> (hidden) output pingpong_buf -> consumer,
// checked against a reference model. Images [0, NA) use random timing; [NA, NIMG) run back-to-back
// (producer without gaps, consumer releasing immediately, start_ok held 1) to exercise the no-bubble rule.
module bl_case import bnn_fcc_pkg::*; #(
  parameter int F         = 8,
  parameter int N         = 8,
  parameter int PW        = 8,
  parameter int PN        = 8,
  parameter bit IS_OUTPUT = 1'b0,
  parameter int NIMG      = 20,
  parameter int LPS       = 3     // POPCOUNT_LEVELS_PER_STAGE
) (
  input  logic clk,
  output logic done,
  output int   errors
);
  localparam int GROUPS    = cdiv(N, PN);
  localparam int PASSES    = cdiv(F, PW);
  localparam int GP        = GROUPS * PASSES;
  localparam int ACC_W     = bits(F + 1);
  localparam int THR_W     = bits(F + 2);
  localparam int LANE_W    = bits(PN);
  localparam int WADDR_W   = bits(GP);
  localparam int TADDR_W   = bits(GROUPS);
  localparam int NG        = cdiv(PN, 8);    // operand copies (lane groups of 8)
  localparam int NPL       = np_latency(PW, 3);
  localparam int NA        = NIMG / 2;
  // A no-bubble opportunity needs the producer to refill a bank (PASSES cycles) within one image and,
  // for hidden layers, the previous output bank to be committed and released within one image.
  // (issue -> output commit is 2 (read) + NPL + 1 (result register) cycles)
  localparam bit NB_REQ    = (GP >= PASSES + 1) && (IS_OUTPUT || GP >= NPL + 5);

  logic                 rst = 1'b1;
  // config ports
  logic                 w_we = 1'b0;
  logic [LANE_W-1:0]    w_lane = '0;
  logic [WADDR_W-1:0]   w_addr = '0;
  logic [PW-1:0]        w_data = '0;
  logic                 t_we = 1'b0;
  logic [LANE_W-1:0]    t_lane = '0;
  logic [TADDR_W-1:0]   t_addr = '0;
  logic [THR_W-1:0]     t_data = '0;
  // input ping-pong
  logic                 i_wr_en = 1'b0, i_wr_bank = 1'b0;
  logic [PASSES-1:0]    i_wr_oh = '0;
  logic [PW-1:0]        i_wr_data = '0;
  logic                 i_commit_en = 1'b0, i_commit_bank = 1'b0;
  logic [1:0]           in_full;
  logic                 in_rd_bank;
  logic                 in_rd_adv;
  logic [NG-1:0][PW-1:0] in_rd_data;
  logic                 in_rls_en, in_rls_bank;
  // output side
  logic [1:0]           out_full;
  logic                 out_wr_en, out_wr_bank;
  logic [GROUPS-1:0]    out_wr_oh;
  logic [PN-1:0]        out_wr_data;
  logic                 out_commit_en, out_commit_bank;
  logic                 o_rd_bank = 1'b0, o_rd_adv = 1'b0;
  logic                 o_rls_en = 1'b0, o_rls_bank = 1'b0;
  logic [N-1:0]         o_rd_data;
  logic                 start_ok = IS_OUTPUT ? 1'b0 : 1'b1;
  logic                 res_valid;
  logic [PN*ACC_W-1:0]  res_counts;
  logic [PN-1:0]        res_mask;
  logic                 res_last;
  logic                 img_start, busy;

  bnn_layer #(.F(F), .N(N), .PW(PW), .PN(PN), .IS_OUTPUT(IS_OUTPUT),
              .POPCOUNT_LEVELS_PER_STAGE(LPS), .RAM_STYLE("auto")) dut (
    .clk(clk), .rst(rst),
    .w_we(w_we), .w_lane(w_lane), .w_addr(w_addr), .w_data(w_data),
    .t_we(t_we), .t_lane(t_lane), .t_addr(t_addr), .t_data(t_data),
    .in_full(in_full), .in_rd_bank(in_rd_bank), .in_rd_adv(in_rd_adv),
    .in_rd_data(in_rd_data), .in_rls_en(in_rls_en), .in_rls_bank(in_rls_bank),
    .out_full(out_full), .out_wr_en(out_wr_en), .out_wr_bank(out_wr_bank), .out_wr_oh(out_wr_oh),
    .out_wr_data(out_wr_data), .out_commit_en(out_commit_en), .out_commit_bank(out_commit_bank),
    .start_ok(start_ok), .res_valid(res_valid), .res_counts(res_counts),
    .res_mask(res_mask), .res_last(res_last),
    .img_start(img_start), .busy(busy)
  );

  pingpong_buf #(.NBITS(F), .WR_W(PW), .RD_W(PW), .RD_COPIES(NG)) u_in (
    .clk(clk), .rst(rst),
    .wr_en(i_wr_en), .wr_bank(i_wr_bank), .wr_oh(i_wr_oh), .wr_data(i_wr_data),
    .commit_en(i_commit_en), .commit_bank(i_commit_bank),
    .rd_adv(in_rd_adv), .rd_bank(in_rd_bank), .rd_data(in_rd_data),
    .rls_en(in_rls_en), .rls_bank(in_rls_bank), .full(in_full)
  );

  if (!IS_OUTPUT) begin : g_out_pp
    pingpong_buf #(.NBITS(N), .WR_W(PN), .RD_W(N)) u_out (
      .clk(clk), .rst(rst),
      .wr_en(out_wr_en), .wr_bank(out_wr_bank), .wr_oh(out_wr_oh), .wr_data(out_wr_data),
      .commit_en(out_commit_en), .commit_bank(out_commit_bank),
      .rd_adv(o_rd_adv), .rd_bank(o_rd_bank), .rd_data(o_rd_data),
      .rls_en(o_rls_en), .rls_bank(o_rls_bank), .full(out_full)
    );
  end else begin : g_no_out_pp
    assign out_full  = 2'b00;
    assign o_rd_data = '0;
  end

  // ---------------- reference model
  bit wt  [N][F];
  int thr [N];
  bit img [NIMG][F];
  int cnt [NIMG][N];
  int order [];      // declared outside loops (xsim queue/dyn-array bug)

  task automatic fail(string m);
    errors++;
    $error("F=%0d N=%0d PW=%0d PN=%0d OUT=%0d: %s", F, N, PW, PN, IS_OUTPUT, m);
  endtask

  task automatic shuffle(int n);
    int j, t;
    order = new[n];
    for (int i = 0; i < n; i++) order[i] = i;
    for (int i = n - 1; i > 0; i--) begin
      j = $urandom_range(i, 0);
      t = order[i]; order[i] = order[j]; order[j] = t;
    end
  endtask

  function automatic logic [PW-1:0] rnd_word();
    logic [PW-1:0] r;
    for (int i = 0; i < PW; i++) r[i] = $urandom_range(1, 0);
    return r;
  endfunction

  task automatic build_model();
    int lo, hi;
    for (int n = 0; n < N; n++) begin
      for (int j = 0; j < F; j++) wt[n][j] = $urandom_range(1, 0);
      if (N >= 3 && n < 3) thr[n] = (n == 0) ? 0 : (n == 1) ? F : F + 1;
      else if ($urandom_range(1, 0)) thr[n] = $urandom_range(F + 1, 0);
      else begin
        lo = imax(0, F / 2 - 3);
        hi = imin(F + 1, F / 2 + 3);
        thr[n] = $urandom_range(hi, lo);
      end
    end
    for (int i = 0; i < NIMG; i++) begin
      for (int j = 0; j < F; j++) img[i][j] = $urandom_range(1, 0);
      for (int n = 0; n < N; n++) begin
        cnt[i][n] = 0;
        for (int j = 0; j < F; j++) cnt[i][n] += (img[i][j] == wt[n][j]);
      end
    end
  endtask

  // ---------------- configuration (layout contract; padding lanes get garbage)
  task automatic configure();
    int lane, addr, g, p, n;
    logic [PW-1:0] d;
    shuffle(PN * GP);
    for (int e = 0; e < PN * GP; e++) begin
      while ($urandom_range(2, 0) == 0) begin
        w_we <= 1'b0; w_lane <= $urandom; w_addr <= $urandom; w_data <= rnd_word();
        @(negedge clk);
      end
      lane = order[e] % PN;
      addr = order[e] / PN;
      g = addr / PASSES;
      p = addr % PASSES;
      n = g * PN + lane;
      d = rnd_word();
      if (n < N)
        for (int k = 0; k < PW; k++) d[k] = (p * PW + k < F) ? wt[n][p * PW + k] : 1'b1;
      w_we <= 1'b1; w_lane <= lane; w_addr <= addr; w_data <= d;
      @(negedge clk);
    end
    w_we <= 1'b0; w_lane <= $urandom; w_addr <= $urandom;
    if (!IS_OUTPUT) begin
      shuffle(PN * GROUPS);
      for (int e = 0; e < PN * GROUPS; e++) begin
        while ($urandom_range(2, 0) == 0) begin
          t_we <= 1'b0; t_lane <= $urandom; t_addr <= $urandom; t_data <= $urandom;
          @(negedge clk);
        end
        lane = order[e] % PN;
        g = order[e] / PN;
        n = g * PN + lane;
        t_we <= 1'b1; t_lane <= lane; t_addr <= g;
        t_data <= (n < N) ? THR_W'(thr[n]) : THR_W'($urandom);
        @(negedge clk);
      end
      t_we <= 1'b0; t_lane <= $urandom; t_addr <= $urandom;
    end
    @(negedge clk);
  endtask

  // ---------------- producer: writes image i into input bank i%2, chunk by chunk, then commits
  // Writes image i's chunks into bank i%2; random order and gaps unless fast. If commit_last, the last
  // chunk is written together with the commit.
  task automatic write_image(input int i, input bit fast, input bit commit_last);
    int b, c;
    logic [PW-1:0] d;
    b = i % 2;
    if (fast) begin
      order = new[PASSES];
      for (int k = 0; k < PASSES; k++) order[k] = k;
    end else shuffle(PASSES);
    for (int e = 0; e < PASSES; e++) begin
      while (!fast && $urandom_range(2, 0) == 0) begin
        i_wr_en <= 1'b0; i_wr_bank <= $urandom; i_wr_oh <= PASSES'(1) << $urandom_range(PASSES - 1, 0); i_wr_data <= rnd_word();
        @(negedge clk);
      end
      c = order[e];
      d = rnd_word();   // bits >= F are garbage; the ping-pong drops them
      for (int k = 0; k < PW; k++) if (c * PW + k < F) d[k] = img[i][c * PW + k];
      i_wr_en <= 1'b1; i_wr_bank <= b[0]; i_wr_oh <= PASSES'(1) << c; i_wr_data <= d;
      i_commit_en <= commit_last && (e == PASSES - 1); i_commit_bank <= b[0];
      @(negedge clk);
      i_commit_en <= 1'b0;
    end
    i_wr_en <= 1'b0; i_wr_oh <= PASSES'(1) << $urandom_range(PASSES - 1, 0);
  endtask

  task automatic commit_bank(input int b);
    i_commit_en <= 1'b1; i_commit_bank <= b[0];
    @(negedge clk);
    i_commit_en <= 1'b0; i_commit_bank <= $urandom;
  endtask

  // Random phase: random order/gaps, separate commit. Back-to-back phase: in order, commit with the
  // last chunk. At images NA and NIMG-2 the pipeline is drained and both input banks are pre-filled
  // (image i+1's bank committed first, then image i's), so even single-pass layers see a no-bubble check.
  task automatic produce();
    int i;
    i = 0;
    while (i < NIMG) begin
      if (i == NA || i == NIMG - 2) begin
        while (in_full !== 2'b00 || out_full !== 2'b00 || busy !== 1'b0) @(negedge clk);
        write_image(i, 1'b1, 1'b0);
        write_image(i + 1, 1'b1, 1'b0);
        commit_bank((i + 1) % 2);
        commit_bank(i % 2);
        i += 2;
      end else begin
        while (in_full[i % 2]) @(negedge clk);
        write_image(i, i >= NA, i >= NA);
        if (i < NA) begin
          repeat ($urandom_range(3, 0)) @(negedge clk);
          commit_bank(i % 2);
        end
        i++;
      end
    end
  endtask

  // ---------------- hidden-layer consumer: reads the whole output bank in one read (data 2 cycles
  // after the advance) and releases it with the read
  task automatic consume();
    int b;
    for (int i = 0; i < NIMG; i++) begin
      b = i % 2;
      while (!out_full[b]) @(negedge clk);
      if (i < NA) repeat ($urandom_range(200, 0)) @(negedge clk);
      o_rd_adv <= 1'b1; o_rd_bank <= b[0]; o_rls_en <= 1'b1; o_rls_bank <= b[0];
      @(negedge clk);
      o_rd_adv <= 1'b0; o_rls_en <= 1'b0; o_rls_bank <= $urandom; o_rd_bank <= $urandom;
      @(negedge clk);
      for (int n = 0; n < N; n++)
        if (o_rd_data[n] !== (cnt[i][n] >= thr[n]))
          fail($sformatf("consumer img %0d neuron %0d: got %b exp %b (cnt %0d thr %0d)",
                         i, n, o_rd_data[n], cnt[i][n] >= thr[n], cnt[i][n], thr[n]));
    end
  endtask

  // ---------------- monitor (samples pre-edge values at each posedge)
  bit run = 1'b0;
  int cyc = 0;
  int n_start = 0, n_done = 0, n_rls = 0, out_g = 0;
  int start_cyc [NIMG];
  int nb_cyc = -1;       // cycle at which the next img_start is required (no-bubble), -1 = none
  int n_nb = 0;          // number of no-bubble checks performed

  always @(posedge clk) begin
    if (run) begin
      int nb, n;
      cyc++;
      // no-bubble requirement from the previous cycle (output layer: start_ok must still be 1)
      if (nb_cyc == cyc) begin
        if (!IS_OUTPUT || start_ok) begin
          n_nb++;
          if (img_start !== 1'b1) fail($sformatf("bubble: image %0d did not start at cycle %0d", n_start, cyc));
        end
        nb_cyc = -1;
      end
      if (img_start === 1'b1) begin
        if (n_start >= NIMG) fail("extra img_start");
        else begin
          if (in_full[n_start % 2] !== 1'b1) fail($sformatf("img %0d started with input bank not full", n_start));
          if (!IS_OUTPUT && out_full[n_start % 2] !== 1'b0)
            fail($sformatf("img %0d started with output bank full", n_start));
          // its output bank must have received image n_start-2's commit already
          if (!IS_OUTPUT && n_done < n_start - 1)
            fail($sformatf("img %0d started before img %0d committed its output bank", n_start, n_start - 2));
          if (IS_OUTPUT && start_ok !== 1'b1) fail($sformatf("img %0d started without start_ok", n_start));
          start_cyc[n_start] = cyc;
          n_start++;
        end
      end else if (img_start !== 1'b0) fail("img_start X");
      // cycle of the last (g,p) of the most recent image: decide whether the next start is mandatory
      if (n_start > 0 && n_start < NIMG && cyc == start_cyc[n_start - 1] + GP - 1) begin
        nb = n_start % 2;
        if (in_full[nb] === 1'b1 &&
            (IS_OUTPUT ? start_ok === 1'b1 : (out_full[nb] === 1'b0 && n_done == n_start - 1)))
          nb_cyc = cyc + 1;
      end
      // busy while any image is started but not complete
      if (n_start > n_done && busy !== 1'b1) fail($sformatf("busy low with %0d images in flight", n_start - n_done));
      // input release: in order, alternating banks, not before the image's last issue
      if (in_rls_en === 1'b1) begin
        if (n_rls >= n_start) fail("input release without a started image");
        else begin
          if (in_rls_bank !== 1'(n_rls % 2)) fail($sformatf("release %0d of wrong bank", n_rls));
          if (cyc < start_cyc[n_rls] + GP - 1) fail($sformatf("img %0d released before its last issue", n_rls));
        end
        n_rls++;
      end
      // results
      if (!IS_OUTPUT) begin
        if (res_valid !== 1'b0) fail("res_valid on a hidden layer");
        if (out_wr_en === 1'b1) begin
          if (n_done >= n_start) fail("output write for an image not started");
          if (out_wr_bank !== 1'(n_done % 2)) fail($sformatf("img %0d written to wrong bank", n_done));
          if (out_wr_oh !== (GROUPS'(1) << out_g)) fail($sformatf("img %0d: chunk one-hot %b exp chunk %0d", n_done, out_wr_oh, out_g));
          for (int k = 0; k < PN; k++) begin
            n = out_g * PN + k;
            if (n >= N) begin
              if (out_wr_data[k] !== 1'b0) fail($sformatf("img %0d: padding lane %0d not 0", n_done, k));
            end else if (n_done < NIMG && out_wr_data[k] !== (cnt[n_done][n] >= thr[n]))
              fail($sformatf("img %0d neuron %0d: got %b exp %b (cnt %0d thr %0d)", n_done, n,
                             out_wr_data[k], cnt[n_done][n] >= thr[n], cnt[n_done][n], thr[n]));
          end
          if (out_commit_en !== (out_g == GROUPS - 1)) fail($sformatf("img %0d chunk %0d: commit=%b", n_done, out_g, out_commit_en));
          if (out_commit_en === 1'b1 && out_commit_bank !== 1'(n_done % 2)) fail("commit of wrong bank");
          if (out_g == GROUPS - 1) begin out_g = 0; n_done++; end
          else out_g++;
        end else begin
          if (out_wr_en !== 1'b0) fail("out_wr_en X");
          if (out_commit_en !== 1'b0) fail("commit without write");
        end
      end else begin
        if (out_wr_en !== 1'b0 || out_commit_en !== 1'b0) fail("output-layer wrote the hidden output port");
        if (res_valid === 1'b1) begin
          if (n_done >= n_start) fail("result for an image not started");
          for (int k = 0; k < PN; k++) begin
            n = out_g * PN + k;
            if (res_mask[k] !== (n < N)) fail($sformatf("img %0d group %0d: mask[%0d]=%b", n_done, out_g, k, res_mask[k]));
            if (n < N && n_done < NIMG && int'(res_counts[k * ACC_W +: ACC_W]) !== cnt[n_done][n])
              fail($sformatf("img %0d neuron %0d: count %0d exp %0d", n_done, n,
                             res_counts[k * ACC_W +: ACC_W], cnt[n_done][n]));
          end
          if (res_last !== (out_g == GROUPS - 1)) fail($sformatf("img %0d group %0d: res_last=%b", n_done, out_g, res_last));
          if (out_g == GROUPS - 1) begin out_g = 0; n_done++; end
          else out_g++;
        end else if (res_valid !== 1'b0) fail("res_valid X");
      end
    end
  end

  // ---------------- output-layer credit: random in the random phase, held 1 afterwards
  initial begin
    if (IS_OUTPUT) begin
      wait (run);
      while (n_done < NIMG) begin
        start_ok <= (n_start >= NA) ? 1'b1 : 1'($urandom_range(1, 0));
        @(negedge clk);
      end
    end
  end

  initial begin
    errors = 0;
    done = 1'b0;
    build_model();
    repeat (3) @(negedge clk);
    rst <= 1'b0;
    @(negedge clk);
    if (busy !== 1'b0 || img_start !== 1'b0) fail("busy/img_start not low after reset");
    configure();
    run = 1'b1;
    fork
      produce();
      if (!IS_OUTPUT) consume();
    join
    while (n_done < NIMG) @(negedge clk);
    repeat (5) @(negedge clk);
    if (n_start != NIMG) fail($sformatf("%0d images started, exp %0d", n_start, NIMG));
    if (n_rls != NIMG) fail($sformatf("%0d input releases, exp %0d", n_rls, NIMG));
    if (busy !== 1'b0) fail("busy high after drain");
    if (in_full !== 2'b00 || out_full !== 2'b00) fail($sformatf("full flags in=%b out=%b after drain", in_full, out_full));
    if (n_nb < (NB_REQ ? (NIMG - NA) / 2 : 2)) fail($sformatf("only %0d no-bubble checks", n_nb));
    $display("tb_bnn_layer: F=%0d N=%0d PW=%0d PN=%0d OUT=%0d: %0d images, %0d no-bubble checks, %0d cycles",
             F, N, PW, PN, IS_OUTPUT, n_done, n_nb, cyc);
    done = 1'b1;
  end
endmodule

module tb_bnn_layer;
  localparam int NC = 17;
  logic clk = 1'b0;
  always #5 clk = ~clk;

  logic [NC-1:0] done;
  int            errs [NC];

  bl_case #(.F(784), .N(256), .PW(66), .PN(32), .IS_OUTPUT(0)) c0 (.clk(clk), .done(done[0]), .errors(errs[0]));
  bl_case #(.F(256), .N(256), .PW(66), .PN(11), .IS_OUTPUT(0)) c1 (.clk(clk), .done(done[1]), .errors(errs[1]));
  bl_case #(.F(256), .N(10),  .PW(66), .PN(1),  .IS_OUTPUT(1)) c2 (.clk(clk), .done(done[2]), .errors(errs[2]));
  bl_case #(.F(13),  .N(7),   .PW(3),  .PN(2),  .IS_OUTPUT(0)) c3 (.clk(clk), .done(done[3]), .errors(errs[3]));
  bl_case #(.F(7),   .N(5),   .PW(2),  .PN(4),  .IS_OUTPUT(0)) c4 (.clk(clk), .done(done[4]), .errors(errs[4]));
  bl_case #(.F(5),   .N(3),   .PW(4),  .PN(3),  .IS_OUTPUT(1)) c5 (.clk(clk), .done(done[5]), .errors(errs[5]));
  bl_case #(.F(8),   .N(8),   .PW(8),  .PN(8),  .IS_OUTPUT(0)) c6 (.clk(clk), .done(done[6]), .errors(errs[6]));
  bl_case #(.F(1),   .N(1),   .PW(1),  .PN(1),  .IS_OUTPUT(0)) c7 (.clk(clk), .done(done[7]), .errors(errs[7]));
  bl_case #(.F(20),  .N(9),   .PW(64), .PN(9),  .IS_OUTPUT(1)) c8 (.clk(clk), .done(done[8]), .errors(errs[8]));
  bl_case #(.F(10),  .N(10),  .PW(3),  .PN(10), .IS_OUTPUT(1)) c9 (.clk(clk), .done(done[9]), .errors(errs[9]));
  // non-default popcount pipelining (LPS = 1: deepest pipeline; LPS = 2)
  bl_case #(.F(784), .N(256), .PW(66), .PN(32), .IS_OUTPUT(0), .LPS(1)) c10 (.clk(clk), .done(done[10]), .errors(errs[10]));
  bl_case #(.F(256), .N(10),  .PW(66), .PN(1),  .IS_OUTPUT(1), .LPS(1)) c11 (.clk(clk), .done(done[11]), .errors(errs[11]));
  bl_case #(.F(13),  .N(7),   .PW(3),  .PN(2),  .IS_OUTPUT(0), .LPS(1)) c12 (.clk(clk), .done(done[12]), .errors(errs[12]));
  bl_case #(.F(256), .N(256), .PW(66), .PN(11), .IS_OUTPUT(0), .LPS(2)) c13 (.clk(clk), .done(done[13]), .errors(errs[13]));
  // single-pass full-width hidden layers (PW = F; judged layers 1-2)
  bl_case #(.F(256), .N(256), .PW(256), .PN(3), .IS_OUTPUT(0))         c14 (.clk(clk), .done(done[14]), .errors(errs[14]));
  bl_case #(.F(256), .N(10),  .PW(256), .PN(1), .IS_OUTPUT(1))         c15 (.clk(clk), .done(done[15]), .errors(errs[15]));
  bl_case #(.F(256), .N(10),  .PW(256), .PN(1), .IS_OUTPUT(1), .LPS(1)) c16 (.clk(clk), .done(done[16]), .errors(errs[16]));

  initial begin
    #20ms;
    $error("tb_bnn_layer: timeout");
    $finish;
  end

  int total;
  initial begin
    wait (&done);
    @(posedge clk);
    total = 0;
    for (int i = 0; i < NC; i++) total += errs[i];
    if (total == 0) $display("SUCCESS: tb_bnn_layer");
    else $display("tb_bnn_layer: %0d mismatches", total);
    $finish;
  end
endmodule
