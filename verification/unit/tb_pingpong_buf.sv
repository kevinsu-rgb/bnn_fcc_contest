// Self-checking testbench for pingpong_buf over several (NBITS, WR_W, RD_W) configurations.
module pp_case import bnn_fcc_pkg::*; #(
  parameter int NBITS = 8,
  parameter int WR_W  = 8,
  parameter int RD_W  = 8,
  parameter int ITERS = 200
) (
  input  logic clk,
  output logic done,
  output int   errors
);
  localparam int NWR = cdiv(NBITS, WR_W);
  localparam int NRD = cdiv(NBITS, RD_W);

  logic                       rst = 1'b1;
  logic                       wr_en = 1'b0, wr_bank = 1'b0;
  logic [bits(NWR)-1:0]       wr_idx = '0;
  logic [WR_W-1:0]            wr_data = '0;
  logic                       commit_en = 1'b0, commit_bank = 1'b0;
  logic                       rd_bank = 1'b0;
  logic [bits(NRD)-1:0]       rd_idx = '0;
  logic [RD_W-1:0]            rd_data;
  logic                       rls_en = 1'b0, rls_bank = 1'b0;
  logic [1:0]                 full;

  pingpong_buf #(.NBITS(NBITS), .WR_W(WR_W), .RD_W(RD_W)) dut (.*);

  bit mdl [2][NBITS];
  int order [];      // declared outside loops (xsim queue/dyn-array bug)

  task automatic fail(string m);
    errors++;
    $error("NBITS=%0d WR_W=%0d RD_W=%0d: %s", NBITS, WR_W, RD_W, m);
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

  // Drive at negedge; random gaps carry garbage on the idle write bus.
  task automatic write_bank(input int b, input bit do_commit);
    int  ord [];
    logic [WR_W-1:0] d;
    shuffle(NWR);
    ord = order;
    for (int n = 0; n < NWR; n++) begin
      while ($urandom_range(2, 0) == 0) begin
        wr_en <= 1'b0; wr_bank <= $urandom; wr_idx <= $urandom; wr_data <= $urandom;
        @(negedge clk);
      end
      d = $urandom;
      if (WR_W > 32) d = {$urandom, $urandom, $urandom};
      wr_en <= 1'b1; wr_bank <= b[0]; wr_idx <= ord[n]; wr_data <= d;
      for (int k = 0; k < WR_W; k++)
        if (ord[n] * WR_W + k < NBITS) mdl[b][ord[n] * WR_W + k] = d[k];
      @(negedge clk);
    end
    wr_en <= 1'b0; wr_bank <= $urandom; wr_idx <= $urandom;
    if (do_commit) begin
      commit_en <= 1'b1; commit_bank <= b[0];
      @(negedge clk);
      commit_en <= 1'b0; commit_bank <= $urandom;
      if (full[b] !== 1'b1) fail($sformatf("bank %0d not full after commit", b));
    end
  endtask

  // Latency-exact: each cycle a new request is driven, then (1 ns later) rd_data must still
  // hold the response to the PREVIOUS cycle's request (a combinational read would show the new one).
  task automatic read_bank(input int b);
    int  ord [];
    int  idx;
    logic [RD_W-1:0] exp_prev, exp_new;
    bit  have_prev;
    shuffle(NRD);
    ord = order;
    have_prev = 1'b0;
    for (int n = 0; n < NRD; n++) begin
      for (int g = 0; g < 2; g++) begin
        if (g == 0 && $urandom_range(3, 0) != 0) continue;   // g==0: occasional extra request
        idx = (g == 0) ? $urandom_range(NRD - 1, 0) : ord[n];
        for (int k = 0; k < RD_W; k++)
          exp_new[k] = (idx * RD_W + k < NBITS) ? mdl[b][idx * RD_W + k] : 1'b0;
        rd_bank <= b[0]; rd_idx <= idx;
        if (g == 1 && n == NRD - 1) begin rls_en <= 1'b1; rls_bank <= b[0]; end  // release with last read
        else begin rls_en <= 1'b0; rls_bank <= $urandom; end
        #1;
        if (have_prev && rd_data !== exp_prev)
          fail($sformatf("bank %0d: rd_data %h != previous request's data %h", b, rd_data, exp_prev));
        exp_prev = exp_new; have_prev = 1'b1;
        @(negedge clk);
      end
    end
    // drain: last response appears one cycle after the last request
    rls_en <= 1'b0; rls_bank <= $urandom;
    #1;
    if (rd_data !== exp_prev)
      fail($sformatf("bank %0d: final rd_data %h != %h", b, rd_data, exp_prev));
    if (full[b] !== 1'b0) fail($sformatf("bank %0d still full after release", b));
  endtask

  initial begin
    int b, ov;
    errors = 0;
    done = 1'b0;
    repeat (3) @(negedge clk);
    rst <= 1'b0;
    @(negedge clk);
    if (full !== 2'b00) fail("full not clear after reset");
    for (int it = 0; it < ITERS; it++) begin
      b  = $urandom_range(1, 0);
      ov = $urandom_range(1, 0);
      write_bank(b, 1'b1);
      if (full[1 - b] !== 1'b0 || full[b] !== 1'b1) fail("full flags wrong after commit");
      if (ov) begin
        fork
          write_bank(1 - b, 1'b1);
          read_bank(b);
        join
        if (full !== (2'b01 << (1 - b))) fail($sformatf("full=%b after overlap", full));
        read_bank(1 - b);
      end else begin
        read_bank(b);
      end
      if (full !== 2'b00) fail($sformatf("full=%b at end of iteration", full));
    end
    // Simultaneous commit and release on different banks.
    write_bank(0, 1'b1);
    write_bank(1, 1'b0);
    commit_en <= 1'b1; commit_bank <= 1'b1; rls_en <= 1'b1; rls_bank <= 1'b0;
    @(negedge clk);
    commit_en <= 1'b0; rls_en <= 1'b0;
    if (full !== 2'b10) fail($sformatf("simultaneous commit/release: full=%b", full));
    read_bank(1);
    done = 1'b1;
  end
endmodule

module tb_pingpong_buf;
  localparam int NC = 10;
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
