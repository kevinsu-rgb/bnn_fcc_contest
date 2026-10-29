// Self-checking testbench for pingpong_buf over several (NBITS, WR_W, RD_W, RD_COPIES) configurations.
// Write: one-hot chunk select, random chunk order and gaps; commit in the cycle of the last write or
// later. full must rise exactly in the cycle after commit_en.
// Read: an image is groups x NRD advances (random gaps between them) on a full bank, released with the
// last advance. The k-th advance must show word k % NRD on every rd_data copy exactly 2 cycles later
// (bits >= NBITS read 0). The first advance sometimes comes in the first cycle full shows (the last
// write may have arrived together with commit_en). Sometimes the producer refills the bank in the
// cycle right after release; the next image read checks that the rotation left it aligned.
// Tie-breaks: commit_en and rls_en on the same bank in one cycle end full; a release in the cycle after
// commit_en (the registered commit) ends empty.
module pp_case import bnn_fcc_pkg::*; #(
  parameter int NBITS     = 8,
  parameter int WR_W      = 8,
  parameter int RD_W      = 8,
  parameter int RD_COPIES = 1,
  parameter int ITERS     = 150
) (
  input  logic clk,
  output logic done,
  output int   errors
);
  localparam int NWR = cdiv(NBITS, WR_W);
  localparam int NRD = cdiv(NBITS, RD_W);

  logic                              rst = 1'b1;
  logic                              wr_en = 1'b0, wr_bank = 1'b0;
  logic [NWR-1:0]                    wr_oh = '0;
  logic [WR_W-1:0]                   wr_data = '0;
  logic                              commit_en = 1'b0, commit_bank = 1'bx;   // X until the first commit
  logic                              rd_adv = 1'b0, rd_bank = 1'b0;
  logic [RD_COPIES-1:0][RD_W-1:0]    rd_data;
  logic                              rls_en = 1'b0, rls_bank = 1'b0;
  logic [1:0]                        full;

  pingpong_buf #(.NBITS(NBITS), .WR_W(WR_W), .RD_W(RD_W), .RD_COPIES(RD_COPIES)) dut (.*);

  bit mdl [2][NBITS];
  int order [];      // declared outside loops (xsim queue/dyn-array bug)
  int ncyc = 0;      // posedges seen
  always @(posedge clk) ncyc++;

  task automatic fail(string m);
    errors++;
    $error("NBITS=%0d WR_W=%0d RD_W=%0d RD_COPIES=%0d: %s", NBITS, WR_W, RD_W, RD_COPIES, m);
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

  function automatic logic [WR_W-1:0] rnd_w();
    logic [WR_W-1:0] r;
    for (int i = 0; i < WR_W; i++) r[i] = $urandom_range(1, 0);
    return r;
  endfunction

  function automatic logic [RD_W-1:0] word_of(int b, int r);
    logic [RD_W-1:0] v;
    for (int k = 0; k < RD_W; k++) v[k] = (r * RD_W + k < NBITS) ? mdl[b][r * RD_W + k] : 1'b0;
    return v;
  endfunction

  // Drive at negedge; idle cycles carry garbage on the write bus.
  // cwl: commit_en in the cycle of the last write. nogap: no idle cycle before the first write.
  task automatic write_bank(input int b, input bit do_commit, input bit cwl, input bit nogap);
    int  ord [];
    logic [WR_W-1:0] d;
    shuffle(NWR);
    ord = order;
    for (int n = 0; n < NWR; n++) begin
      while (!(nogap && n == 0) && $urandom_range(2, 0) == 0) begin
        wr_en <= 1'b0; wr_bank <= $urandom; wr_oh <= NWR'(1) << $urandom_range(NWR - 1, 0);
        wr_data <= rnd_w();
        @(negedge clk);
      end
      d = rnd_w();
      wr_en <= 1'b1; wr_bank <= b[0]; wr_oh <= NWR'(1) << ord[n]; wr_data <= d;
      for (int k = 0; k < WR_W; k++)
        if (ord[n] * WR_W + k < NBITS) mdl[b][ord[n] * WR_W + k] = d[k];
      if (do_commit && cwl && n == NWR - 1) begin
        if (full[b] !== 1'b0) fail($sformatf("bank %0d full before commit", b));
        commit_en <= 1'b1; commit_bank <= b[0];
      end
      @(negedge clk);
      commit_en <= 1'b0; commit_bank <= $urandom;
    end
    wr_en <= 1'b0; wr_bank <= $urandom; wr_oh <= NWR'(1) << $urandom_range(NWR - 1, 0);
    if (do_commit && !cwl) begin
      while ($urandom_range(2, 0) == 0) @(negedge clk);
      if (full[b] !== 1'b0) fail($sformatf("bank %0d full before commit", b));
      commit_en <= 1'b1; commit_bank <= b[0];
      @(negedge clk);
      commit_en <= 1'b0; commit_bank <= $urandom;
    end
    if (do_commit && full[b] !== 1'b1) fail($sformatf("bank %0d not full the cycle after commit", b));
  endtask

  // groups x NRD advances; data due exactly 2 cycles after each advance, on every copy.
  // imm: first advance without an idle cycle. refill: refill the bank right after release.
  typedef struct { int due; logic [RD_W-1:0] w; } exp_t;
  exp_t exq [$];

  task automatic check_due();
    #1;
    while (exq.size() > 0 && exq[0].due == ncyc) begin
      for (int c = 0; c < RD_COPIES; c++)
        if (rd_data[c] !== exq[0].w)
          fail($sformatf("rd_data[%0d] %h != expected %h", c, rd_data[c], exq[0].w));
      void'(exq.pop_front());
    end
    if (exq.size() > 0 && exq[0].due < ncyc) begin
      fail("missed read check"); exq.delete();
    end
  endtask

  task automatic drain(int cycles);
    rd_adv <= 1'b0; rd_bank <= $urandom; rls_en <= 1'b0; rls_bank <= $urandom;
    for (int i = 0; i < cycles; i++) begin
      @(negedge clk);
      check_due();
    end
  endtask

  task automatic read_bank(input int b, input int groups, input bit imm, input bit refill);
    exp_t e;
    int   total;
    total = groups * NRD;
    exq.delete();
    for (int k = 0; k < total; k++) begin
      while (!(imm && k == 0) && $urandom_range(3, 0) == 0) begin
        rd_adv <= 1'b0; rd_bank <= $urandom; rls_en <= 1'b0; rls_bank <= $urandom;
        @(negedge clk);
        check_due();
      end
      rd_adv <= 1'b1; rd_bank <= b[0];
      e.due = ncyc + 2; e.w = word_of(b, k % NRD);
      exq.push_back(e);
      if (k == total - 1) begin rls_en <= 1'b1; rls_bank <= b[0]; end
      else begin rls_en <= 1'b0; rls_bank <= $urandom; end
      @(negedge clk);
      check_due();
    end
    if (refill) begin
      fork
        drain(3);
        begin
          if (full[b] !== 1'b0) fail($sformatf("bank %0d still full the cycle after release", b));
          write_bank(b, 1'b1, $urandom_range(1, 0), 1'b1);
        end
      join
    end else begin
      drain(3);
      if (full[b] !== 1'b0) fail($sformatf("bank %0d still full after release", b));
    end
  endtask

  initial begin
    int b, ov, g;
    errors = 0;
    done = 1'b0;
    repeat (3) @(negedge clk);
    rst <= 1'b0;
    @(negedge clk);
    if (full !== 2'b00) fail("full not clear after reset (commit_bank still X)");
    for (int it = 0; it < ITERS; it++) begin
      b  = $urandom_range(1, 0);
      ov = $urandom_range(1, 0);
      write_bank(b, 1'b1, $urandom_range(1, 0), 1'b0);
      if (full[1 - b] !== 1'b0 || full[b] !== 1'b1) fail("full flags wrong after commit");
      if (ov) begin
        fork
          write_bank(1 - b, 1'b1, $urandom_range(1, 0), 1'b0);
          read_bank(b, $urandom_range(3, 1), $urandom_range(1, 0), 1'b0);
        join
        if (full !== (2'b01 << (1 - b))) fail($sformatf("full=%b after overlap", full));
        read_bank(1 - b, $urandom_range(3, 1), $urandom_range(1, 0), 1'b0);
      end else if ($urandom_range(1, 0)) begin
        read_bank(b, $urandom_range(3, 1), $urandom_range(1, 0), 1'b1);   // refill right after release
        read_bank(b, $urandom_range(3, 1), $urandom_range(1, 0), 1'b0);
      end else begin
        read_bank(b, $urandom_range(3, 1), $urandom_range(1, 0), 1'b0);
      end
      if (full !== 2'b00) fail($sformatf("full=%b at end of iteration", full));
    end
    // Simultaneous commit and release on different banks.
    write_bank(0, 1'b1, 1'b0, 1'b0);
    write_bank(1, 1'b0, 1'b0, 1'b0);
    commit_en <= 1'b1; commit_bank <= 1'b1; rls_en <= 1'b1; rls_bank <= 1'b0;
    @(negedge clk);
    commit_en <= 1'b0; rls_en <= 1'b0;
    if (full !== 2'b10) fail($sformatf("simultaneous commit/release: full=%b", full));
    read_bank(1, 1, 1'b0, 1'b0);
    // commit_en and rls_en on the same bank in one cycle: the bank ends full.
    write_bank(0, 1'b0, 1'b0, 1'b0);
    commit_en <= 1'b1; commit_bank <= 1'b0; rls_en <= 1'b1; rls_bank <= 1'b0;
    @(negedge clk);
    commit_en <= 1'b0; rls_en <= 1'b0;
    @(negedge clk);
    if (full !== 2'b01) fail($sformatf("same-cycle commit/release: full=%b, expected 01", full));
    read_bank(0, 1, 1'b0, 1'b0);
    // Release in the cycle after commit_en (a single-word image consumed at once): the bank ends empty.
    write_bank(1, 1'b0, 1'b0, 1'b0);
    commit_en <= 1'b1; commit_bank <= 1'b1;
    @(negedge clk);
    commit_en <= 1'b0;
    if (full !== 2'b10) fail($sformatf("full=%b the cycle after commit, expected 10", full));
    rls_en <= 1'b1; rls_bank <= 1'b1;
    @(negedge clk);
    rls_en <= 1'b0;
    @(negedge clk);
    if (full !== 2'b00) fail($sformatf("release right after commit: full=%b, expected 00", full));
    done = 1'b1;
  end
endmodule

module tb_pingpong_buf;
  localparam int NC = 14;
  logic clk = 1'b0;
  always #5 clk = ~clk;

  logic [NC-1:0] done;
  int            errs [NC];

  pp_case #(.NBITS(784), .WR_W(66), .RD_W(66)) c0 (.clk(clk), .done(done[0]), .errors(errs[0]));
  pp_case #(.NBITS(256), .WR_W(32), .RD_W(66)) c1 (.clk(clk), .done(done[1]), .errors(errs[1]));
  pp_case #(.NBITS(256), .WR_W(11), .RD_W(66)) c2 (.clk(clk), .done(done[2]), .errors(errs[2]));
  pp_case #(.NBITS(10),  .WR_W(1),  .RD_W(66)) c3 (.clk(clk), .done(done[3]), .errors(errs[3]));
  pp_case #(.NBITS(13),  .WR_W(2),  .RD_W(3))  c4 (.clk(clk), .done(done[4]), .errors(errs[4]));
  pp_case #(.NBITS(7),   .WR_W(4),  .RD_W(2))  c5 (.clk(clk), .done(done[5]), .errors(errs[5]));
  pp_case #(.NBITS(5),   .WR_W(8),  .RD_W(8))  c6 (.clk(clk), .done(done[6]), .errors(errs[6]));
  pp_case #(.NBITS(100), .WR_W(7),  .RD_W(11)) c7 (.clk(clk), .done(done[7]), .errors(errs[7]));
  // full-width read (RD_W = NBITS): input buffer of a single-pass layer
  pp_case #(.NBITS(256), .WR_W(32), .RD_W(256)) c8 (.clk(clk), .done(done[8]), .errors(errs[8]));
  pp_case #(.NBITS(256), .WR_W(3),  .RD_W(256)) c9 (.clk(clk), .done(done[9]), .errors(errs[9]));
  // layer-0 input buffer: 8-bit image beats in, PARALLEL_INPUTS-wide words out
  pp_case #(.NBITS(784), .WR_W(8),  .RD_W(66))  c10 (.clk(clk), .done(done[10]), .errors(errs[10]));
  pp_case #(.NBITS(784), .WR_W(8),  .RD_W(66), .RD_COPIES(4)) c11 (.clk(clk), .done(done[11]), .errors(errs[11]));
  pp_case #(.NBITS(256), .WR_W(3),  .RD_W(256), .RD_COPIES(1)) c12 (.clk(clk), .done(done[12]), .errors(errs[12]));
  // single chunk, single word (NWR = NRD = 1)
  pp_case #(.NBITS(5),   .WR_W(8),  .RD_W(8), .RD_COPIES(2)) c13 (.clk(clk), .done(done[13]), .errors(errs[13]));

  initial begin
    #50ms;
    $error("tb_pingpong_buf: timeout");
    $finish;
  end

  int total;
  initial begin
    wait (&done);
    @(posedge clk);
    total = 0;
    for (int i = 0; i < NC; i++) total += errs[i];
    if (total == 0) $display("SUCCESS: tb_pingpong_buf");
    else $display("tb_pingpong_buf: %0d mismatches", total);
    $finish;
  end
endmodule
