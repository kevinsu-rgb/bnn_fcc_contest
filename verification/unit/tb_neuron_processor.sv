// Self-checking testbench for neuron_processor, PW in {1, 8, 66}.
module np_case import bnn_fcc_pkg::*; #(
  parameter int PW = 8
) (
  input  logic clk,
  output logic done,
  output int   errors
);
  localparam int F     = 3 * PW;
  localparam int ACC_W = bits(F + 1);
  localparam int THR_W = bits(F + 2);
  localparam int LAT   = np_latency(PW, 3);
  localparam int NNEUR = 500;

  logic             rst = 1'b1;
  logic             in_valid = 1'b0, first_pass = 1'b0, last_pass = 1'b0;
  logic [PW-1:0]    x = '0, w = '0;
  logic [THR_W-1:0] thr = '0;
  logic             out_valid, out_y;
  logic [ACC_W-1:0] out_acc;

  neuron_processor #(.PW(PW), .ACC_W(ACC_W), .THR_W(THR_W), .POPCOUNT_LEVELS_PER_STAGE(3)) dut (.*);

  // Expected results keyed by the edge at which the last-pass input is sampled.
  int                exp_acc [int];
  bit                exp_y   [int];
  int                cyc = 0;
  int                racc = 0;
  int                pend = 0;
  int                n_out = 0;
  bit                run = 1'b0;

  // Driver state
  int  neuron = 0, pass = 0, passes = 0, gap_pct = 0, tail = 0;
  int  cur_thr = 0;
  initial begin done = 1'b0; errors = 0; end

  function automatic logic [PW-1:0] rnd_bits();
    logic [PW-1:0] r;
    for (int i = 0; i < PW; i++) r[i] = $urandom_range(0, 1);
    return r;
  endfunction

  always @(posedge clk) begin
    cyc = cyc + 1;
    if (run) begin
      // ---- sample inputs seen by the DUT at this edge
      if (in_valid) begin
        int c;
        c = $countones(~(x ^ w));
        racc = first_pass ? c : racc + c;
        if (last_pass) begin
          exp_acc[cyc] = racc;
          exp_y[cyc]   = (racc >= int'(thr));
          pend++;
        end
      end
      // ---- check outputs
      if (exp_acc.exists(cyc - LAT)) begin
        if (out_valid !== 1'b1) begin
          errors++; $error("PW=%0d cyc=%0d: out_valid missing", PW, cyc);
        end else begin
          if (out_acc !== ACC_W'(exp_acc[cyc-LAT])) begin
            errors++; $error("PW=%0d cyc=%0d: acc exp=%0d got=%0d", PW, cyc, exp_acc[cyc-LAT], out_acc);
          end
          if (out_y !== exp_y[cyc-LAT]) begin
            errors++; $error("PW=%0d cyc=%0d: y exp=%0d got=%0d", PW, cyc, exp_y[cyc-LAT], out_y);
          end
        end
        exp_acc.delete(cyc - LAT);
        exp_y.delete(cyc - LAT);
        pend--;
        n_out++;
      end else if (out_valid !== 1'b0) begin
        errors++; $error("PW=%0d cyc=%0d: spurious out_valid", PW, cyc);
      end
    end

    // ---- drive next inputs
    if (rst) begin
      // hold reset 3 edges then start
      if (cyc == 3) begin
        rst <= 1'b0; run = 1'b1;
        passes  = 1 + $urandom_range(0, 2);
        gap_pct = ($urandom_range(0, 1) == 0) ? 0 : 40;
        pass    = 0;
        case ($urandom_range(0, 3))
          0: cur_thr = 0;
          1: cur_thr = F;
          2: cur_thr = F + 1;
          default: cur_thr = $urandom_range(0, F + 1);
        endcase
      end
    end else if (neuron < NNEUR) begin
      if ($urandom_range(0, 99) < gap_pct) begin
        // idle cycle with garbage on all inputs
        in_valid <= 1'b0; first_pass <= $urandom(); last_pass <= $urandom();
        x <= rnd_bits(); w <= rnd_bits(); thr <= THR_W'($urandom());
      end else begin
        in_valid   <= 1'b1;
        first_pass <= (pass == 0);
        last_pass  <= (pass == passes - 1);
        x <= rnd_bits(); w <= rnd_bits();
        // thr only meaningful with last_pass; garbage otherwise
        thr <= (pass == passes - 1) ? THR_W'(cur_thr) : THR_W'($urandom());
        if (pass == passes - 1) begin
          neuron++;
          pass    = 0;
          passes  = 1 + $urandom_range(0, 2);
          if ($urandom_range(0, 9) == 0) gap_pct = ($urandom_range(0, 1) == 0) ? 0 : 40;
          case ($urandom_range(0, 3))
            0: cur_thr = 0;
            1: cur_thr = F;
            2: cur_thr = F + 1;
            default: cur_thr = $urandom_range(0, F + 1);
          endcase
        end else pass++;
      end
    end else begin
      in_valid <= 1'b0;
      tail++;
      if (tail > LAT + 4) begin
        if (pend != 0 || n_out != NNEUR) begin
          errors++; $error("PW=%0d: pend=%0d n_out=%0d", PW, pend, n_out);
        end
        done <= 1'b1;
      end
    end
  end
endmodule

module tb_neuron_processor;
  logic clk = 1'b0;
  always #5 clk = ~clk;

  logic done [3];
  int   errs [3];
  np_case #(.PW(1))  c0 (.clk(clk), .done(done[0]), .errors(errs[0]));
  np_case #(.PW(8))  c1 (.clk(clk), .done(done[1]), .errors(errs[1]));
  np_case #(.PW(66)) c2 (.clk(clk), .done(done[2]), .errors(errs[2]));

  initial begin : wd
    #5_000_000;
    $error("FAILURE: watchdog timeout");
    $finish;
  end

  initial begin : main
    wait (done[0] === 1'b1 && done[1] === 1'b1 && done[2] === 1'b1);
    @(posedge clk);
    if (errs[0] + errs[1] + errs[2] == 0) $display("SUCCESS: tb_neuron_processor");
    else $display("FAILURE: tb_neuron_processor errors=%0d", errs[0] + errs[1] + errs[2]);
    $finish;
  end
endmodule
