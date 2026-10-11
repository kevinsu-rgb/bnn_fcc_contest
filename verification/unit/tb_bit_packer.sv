// Self-checking testbench for bit_packer over several (IN_W, OUT_W, PAD) configurations.
module bp_case import bnn_fcc_pkg::*; #(
  parameter int IN_W  = 8,
  parameter int OUT_W = 8,
  parameter bit PAD   = 1'b0
) (
  input  logic clk,
  output logic done,
  output int   errors
);
  logic                     rst = 1'b1;
  logic                     in_valid = 1'b0;
  logic                     in_ready;
  logic [IN_W-1:0]          in_bits = '0;
  logic [bits(IN_W+1)-1:0]  in_count = '0;
  logic                     in_flush = 1'b0;
  logic                     out_valid;
  logic [OUT_W-1:0]         out_word;
  logic                     flush_done;

  bit_packer #(.IN_W(IN_W), .OUT_W(OUT_W), .PAD(PAD)) dut (
    .clk(clk), .rst(rst), .clear(1'b0),
    .in_valid(in_valid), .in_ready(in_ready), .in_bits(in_bits), .in_count(in_count),
    .in_flush(in_flush),
    .out_valid(out_valid), .out_word(out_word), .flush_done(flush_done)
  );

  // ---------------- reference model (updated at posedge, after checks) ----------------
  bit  exp_q [$];          // bits not yet emitted
  int  bnd_q [$];          // flush boundaries, as cumulative word counts
  int  words_out    = 0;   // words emitted so far
  int  total_bits   = 0;   // accepted bits (padded at each flush)
  int  flushes_acc  = 0;
  int  flushes_done = 0;
  bit  pend_model   = 1'b0;
  bit  checking     = 1'b0;

  always @(posedge clk) if (checking) begin : chk
    int  w_after;
    logic [OUT_W-1:0] expw;
    w_after = words_out;
    if (out_valid) begin
      if (exp_q.size() < OUT_W) begin
        errors++;
        $error("IN_W=%0d OUT_W=%0d PAD=%0d: word emitted but model has only %0d bits",
               IN_W, OUT_W, PAD, exp_q.size());
      end else begin
        for (int i = 0; i < OUT_W; i++) expw[i] = exp_q[i];
        if (out_word !== expw) begin
          errors++;
          $error("IN_W=%0d OUT_W=%0d PAD=%0d: word %0d exp=%h got=%h",
                 IN_W, OUT_W, PAD, words_out, expw, out_word);
        end
        for (int i = 0; i < OUT_W; i++) void'(exp_q.pop_front());
      end
      w_after = words_out + 1;
      words_out = w_after;
    end
    if (OUT_W >= IN_W && !in_ready && !pend_model) begin
      errors++;
      $error("IN_W=%0d OUT_W=%0d PAD=%0d: in_ready low without pending flush", IN_W, OUT_W, PAD);
    end
    if (flush_done) begin
      if (bnd_q.size() == 0) begin
        errors++;
        $error("IN_W=%0d OUT_W=%0d PAD=%0d: spurious flush_done", IN_W, OUT_W, PAD);
      end else begin
        if (bnd_q[0] != w_after) begin
          errors++;
          $error("IN_W=%0d OUT_W=%0d PAD=%0d: flush_done with %0d words, expected %0d",
                 IN_W, OUT_W, PAD, w_after, bnd_q[0]);
        end
        void'(bnd_q.pop_front());
      end
      flushes_done++;
      pend_model = 1'b0;
    end
    if (in_valid && in_ready) begin
      for (int i = 0; i < int'(in_count); i++) exp_q.push_back(in_bits[i]);
      total_bits += int'(in_count);
      if (in_flush) begin
        while (total_bits % OUT_W != 0) begin
          exp_q.push_back(PAD);
          total_bits++;
        end
        bnd_q.push_back(total_bits / OUT_W);
        flushes_acc++;
        pend_model = 1'b1;
      end
    end
  end

  // ---------------- stimulus ----------------
  logic [IN_W-1:0] rnd;
  int              target;
  initial begin
    done   = 1'b0;
    errors = 0;
    repeat (3) @(negedge clk);
    rst = 1'b0;
    checking = 1'b1;

    // Random phase
    for (int cyc = 0; cyc < 3000; cyc++) begin
      @(negedge clk);
      in_valid = ($urandom_range(99) < 70);
      rnd = '0;
      for (int k = 0; k < IN_W; k += 32) rnd[k +: 32] = $urandom;
      in_bits  = rnd;
      in_count = $urandom_range(IN_W);
      in_flush = ($urandom_range(99) < 10);
    end

    // Directed: flush with accumulated bits an exact multiple of OUT_W (twice)
    for (int rep = 0; rep < 2; rep++) begin
      target = flushes_acc + 1;
      while (flushes_acc < target) begin
        @(negedge clk);
        in_valid = 1'b1;
        rnd = '0;
        for (int k = 0; k < IN_W; k += 32) rnd[k +: 32] = $urandom;
        in_bits  = rnd;
        in_count = 1;
        in_flush = (((total_bits + 1) % OUT_W) == 0) && !pend_model;
        // wait for the cycle's posedge so the model updates before the next decision
        @(posedge clk);
        #1;
      end
    end
    // Directed: flush with no bits at all
    target = flushes_acc + 1;
    while (flushes_acc < target) begin
      @(negedge clk);
      in_valid = 1'b1;
      in_bits  = '0;
      in_count = 0;
      in_flush = 1'b1;
      @(posedge clk);
      #1;
    end
    @(negedge clk);
    in_valid = 1'b0;
    in_flush = 1'b0;
    repeat (OUT_W + IN_W + 20) @(negedge clk);
    if (exp_q.size() != 0 || bnd_q.size() != 0 || flushes_acc != flushes_done) begin
      errors++;
      $error("IN_W=%0d OUT_W=%0d PAD=%0d: leftover model bits=%0d bnd=%0d acc=%0d done=%0d",
             IN_W, OUT_W, PAD, exp_q.size(), bnd_q.size(), flushes_acc, flushes_done);
    end
    done = 1'b1;
  end
endmodule

module tb_bit_packer;
  localparam int NC = 14;
  localparam int OUTS [12] = '{1, 3, 5, 8, 16, 49, 1, 3, 5, 8, 16, 49};
  logic clk = 1'b0;
  always #5 clk = ~clk;

  logic [NC-1:0] done;
  int            errs [NC];

  generate
    for (genvar i = 0; i < 12; i++) begin : g_a
      bp_case #(.IN_W(8), .OUT_W(OUTS[i]), .PAD(i >= 6)) c (.clk(clk), .done(done[i]), .errors(errs[i]));
    end
    bp_case #(.IN_W(1), .OUT_W(1),   .PAD(1'b0)) c_1_1   (.clk(clk), .done(done[12]), .errors(errs[12]));
    bp_case #(.IN_W(8), .OUT_W(262), .PAD(1'b0)) c_8_262 (.clk(clk), .done(done[13]), .errors(errs[13]));
  endgenerate

  int total;
  initial begin
    wait (&done);
    @(posedge clk);
    total = 0;
    for (int i = 0; i < NC; i++) total += errs[i];
    if (total == 0) $display("SUCCESS: tb_bit_packer");
    else $display("tb_bit_packer: %0d mismatches", total);
    $finish;
  end
endmodule
