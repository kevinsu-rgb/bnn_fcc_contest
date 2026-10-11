// Self-checking testbench for popcount (all W x LPS combinations).
module tb_popcount;
  import bnn_fcc_pkg::*;

  localparam int NW = 11;
  localparam int WS [NW] = '{1, 5, 6, 7, 8, 16, 43, 49, 64, 66, 131};
  localparam int NL = 2;
  localparam int LPSS [NL] = '{1, 3};
  localparam int NVEC = 2000;

  logic clk = 1'b0;
  always #5 clk = ~clk;

  logic [NW*NL-1:0] done = '0;
  int               errors [NW*NL];

  generate
    for (genvar wi = 0; wi < NW; wi++) begin : g_w
      for (genvar li = 0; li < NL; li++) begin : g_l
        localparam int W   = WS[wi];
        localparam int LPS = LPSS[li];
        localparam int LAT = popcount_latency(W, LPS);
        localparam int IDX = wi * NL + li;

        logic [W-1:0]           in_bits = '0;
        logic [bits(W+1)-1:0]   count;
        int                     exp_q[$];

        popcount #(.W(W), .LEVELS_PER_STAGE(LPS)) dut (.clk(clk), .in_bits(in_bits), .count(count));

        logic [W-1:0] v;
        int           exp_v;

        initial begin
          errors[IDX] = 0;
          for (int j = 0; j < NVEC + 2 + LAT + 2; j++) begin
            @(negedge clk);
            // Compare the output against the vector driven LAT cycles ago.
            if (exp_q.size() == LAT) begin
              exp_v = exp_q.pop_front();
              if (int'(count) !== exp_v) begin
                errors[IDX]++;
                $error("W=%0d LPS=%0d exp=%0d got=%0d", W, LPS, exp_v, count);
              end
            end
            if (j == 0)      v = '0;
            else if (j == 1) v = '1;
            else if (j < NVEC + 2) begin
              for (int k = 0; k < W; k += 32) v[k +: 32] = $urandom;
            end else v = '0;
            in_bits = v;
            exp_q.push_back($countones(v));
          end
          done[IDX] = 1'b1;
        end
      end
    end
  endgenerate

  int total;
  initial begin
    assert (popcount_latency(66, 3) == 2) else $error("popcount_latency(66,3) != 2");
    assert (popcount_latency(1, 3) == 1)  else $error("popcount_latency(1,3) != 1");
    wait (&done);
    @(posedge clk);
    total = 0;
    begin
      for (int i = 0; i < NW*NL; i++) total += errors[i];
      if (total == 0) $display("SUCCESS: tb_popcount");
      else $display("tb_popcount: %0d mismatches", total);
    end
    $finish;
  end
endmodule
