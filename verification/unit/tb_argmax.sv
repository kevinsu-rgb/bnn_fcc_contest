// Self-checking testbench for argmax over several (PN, N) configurations.
// Counts are drawn from 0..3 (many ties); masked lanes hold 4'hF garbage.
module am_case import bnn_fcc_pkg::*; #(
  parameter int PN  = 1,
  parameter int N   = 10,
  parameter int LPS = 2
) (
  input  logic clk,
  output logic done,
  output int   errors
);
  localparam int ACC_W  = 4;
  localparam int IDX_W  = bits(N);
  localparam int GROUPS = cdiv(N, PN);
  localparam int NIMG   = 500;

  logic                  rst = 1'b1;
  logic                  in_valid = 1'b0;
  logic [PN*ACC_W-1:0]   in_counts = '0;
  logic [PN-1:0]         in_mask = '0;
  logic                  in_last = 1'b0;
  logic                  out_valid;
  logic [IDX_W-1:0]      out_idx;
  logic                  busy;

  argmax #(.PN(PN), .ACC_W(ACC_W), .N(N), .IDX_W(IDX_W), .LEVELS_PER_STAGE(LPS)) dut (.*);

  int exp_q [$];
  int mid_image = 0;     // 1 while groups of an image have been accepted but not in_last
  int pend      = 0;     // images whose in_last was accepted but out_valid not yet seen
  int outs_seen = 0;
  bit checking  = 1'b0;

  always @(posedge clk) if (checking) begin
    if ((mid_image != 0 || pend > 0) && busy !== 1'b1) begin
      errors++;
      $error("PN=%0d N=%0d LPS=%0d: busy low while image in flight", PN, N, LPS);
    end
    if (out_valid === 1'b1) begin
      if (exp_q.size() == 0) begin
        errors++;
        $error("PN=%0d N=%0d: unexpected out_valid", PN, N);
      end else begin
        if (out_idx !== IDX_W'(exp_q[0])) begin
          errors++;
          $error("PN=%0d N=%0d LPS=%0d: image %0d exp=%0d got=%0d", PN, N, LPS, outs_seen, exp_q[0], out_idx);
        end
        void'(exp_q.pop_front());
      end
      outs_seen++;
      pend--;
    end else if (out_valid !== 1'b0) begin
      errors++;
      $error("PN=%0d N=%0d: out_valid is X", PN, N);
    end
    if (in_valid) begin
      mid_image = 1;
      if (in_last) begin mid_image = 0; pend++; end
    end
  end

  int vals [N];
  int seed_mode;
  int gap;
  int best, bidx;
  int lane_n;

  initial begin
    done = 1'b0;
    errors = 0;
    repeat (3) @(negedge clk);
    rst = 1'b0;
    @(negedge clk);
    checking = 1'b1;
    for (int img = 0; img < NIMG; img++) begin
      seed_mode = $urandom_range(0, 5);
      for (int n = 0; n < N; n++) begin
        case (seed_mode)
          0:       vals[n] = 2;                       // all equal
          1:       vals[n] = (n == N - 1) ? 3 : 0;    // max only at last neuron
          2:       vals[n] = $urandom_range(0, 1);
          default: vals[n] = $urandom_range(0, 3);
        endcase
      end
      best = -1; bidx = 0;
      for (int n = 0; n < N; n++) if (vals[n] > best) begin best = vals[n]; bidx = n; end
      exp_q.push_back(bidx);
      for (int g = 0; g < GROUPS; g++) begin
        gap = ($urandom_range(0, 2) == 0) ? $urandom_range(1, 4) : 0;
        if (gap != 0) begin
          in_valid <= 1'b0;
          in_counts <= {PN*ACC_W{1'bx}};   // data must be ignored when not valid
          in_mask <= {PN{1'bx}};
          in_last <= 1'bx;
          repeat (gap) @(negedge clk);
        end
        for (int k = 0; k < PN; k++) begin
          lane_n = g * PN + k;
          in_mask[k] = (lane_n < N);
          in_counts[k*ACC_W +: ACC_W] = (lane_n < N) ? ACC_W'(vals[lane_n]) : {ACC_W{1'b1}};
        end
        in_valid = 1'b1;
        in_last  = (g == GROUPS - 1);
        @(negedge clk);
      end
      in_valid = 1'b0;
      in_last  = 1'b0;
      if ($urandom_range(0, 3) == 0) repeat ($urandom_range(1, 20)) @(negedge clk);
    end
    in_valid = 1'b0;
    repeat (50) @(negedge clk);
    if (exp_q.size() != 0 || outs_seen != NIMG || pend != 0) begin
      errors++;
      $error("PN=%0d N=%0d: leftover expected=%0d outs=%0d pend=%0d", PN, N, exp_q.size(), outs_seen, pend);
    end
    if (busy !== 1'b0) begin
      errors++;
      $error("PN=%0d N=%0d: busy still high when idle", PN, N);
    end
    done = 1'b1;
  end
endmodule

module tb_argmax;
  localparam int NC = 8;
  logic clk = 1'b0;
  always #5 clk = ~clk;

  logic [NC-1:0] done;
  int            errs [NC];

  am_case #(.PN(1),  .N(10), .LPS(2)) c0 (.clk(clk), .done(done[0]), .errors(errs[0]));
  am_case #(.PN(10), .N(10), .LPS(2)) c1 (.clk(clk), .done(done[1]), .errors(errs[1]));
  am_case #(.PN(3),  .N(10), .LPS(2)) c2 (.clk(clk), .done(done[2]), .errors(errs[2]));
  am_case #(.PN(4),  .N(3),  .LPS(2)) c3 (.clk(clk), .done(done[3]), .errors(errs[3]));
  am_case #(.PN(2),  .N(5),  .LPS(2)) c4 (.clk(clk), .done(done[4]), .errors(errs[4]));
  am_case #(.PN(10), .N(10), .LPS(1)) c5 (.clk(clk), .done(done[5]), .errors(errs[5]));
  am_case #(.PN(3),  .N(10), .LPS(3)) c6 (.clk(clk), .done(done[6]), .errors(errs[6]));
  am_case #(.PN(8),  .N(10), .LPS(1)) c7 (.clk(clk), .done(done[7]), .errors(errs[7]));

  int total;
  initial begin
    fork
      wait (&done);
      begin #50_000_000; $error("tb_argmax: watchdog timeout"); $finish; end
    join_any
    @(posedge clk);
    total = 0;
    for (int i = 0; i < NC; i++) total += errs[i];
    if (total == 0) $display("SUCCESS: tb_argmax");
    else $display("tb_argmax: %0d mismatches", total);
    $finish;
  end
endmodule
